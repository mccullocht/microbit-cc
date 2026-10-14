//! Capacitive touch on the micro:bit logo, the same way CODAL senses it.
//!
//! The logo pad (P1.04) has a high-value pull-up. Each cycle the pin is driven low to drain it,
//! then released and timed until the pull-up charges it back to a logic high. A finger adds
//! capacitance, so the pin charges more slowly. The timing is done in hardware: a 16 MHz timer
//! is cleared as the pin is released, and the pin's rising edge (a GPIOTE event) captures the
//! timer's count through PPI.

use embassy_nrf::Peri;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive, Pull};
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::peripherals::{GPIOTE_CH0, PPI_CH2, TIMER2};
use embassy_nrf::ppi::Ppi;
use embassy_nrf::timer::{Frequency, Timer};
use embassy_time::{Duration, Timer as Delay};

use crate::motor::{MAX_SPEED, MOTORS, Motor};

/// How long the pin is given to charge, and then to drain.
const CHARGE_TIME: Duration = Duration::from_millis(1);
const DRAIN_TIME: Duration = Duration::from_millis(1);
/// Charge time, in 16 MHz ticks, at or above which the logo counts as touched (about 220 µs).
/// This is CODAL's default for the micro:bit v2 logo.
const THRESHOLD: u32 = 3500;
/// Reported if the pin never reaches a logic high within `CHARGE_TIME`.
const NO_EDGE: u32 = u32::MAX;
/// Consecutive samples (2 ms each) needed to change between touched and released.
const DEBOUNCE: u8 = 5;

/// Hardware used to sense the logo.
pub struct TouchSensor {
    pub pin: Peri<'static, AnyPin>,
    pub gpiote: Peri<'static, GPIOTE_CH0>,
    pub ppi: Peri<'static, PPI_CH2>,
    pub timer: Peri<'static, TIMER2>,
}

/// Toggles `motor` between stopped and full speed each time the logo is touched.
#[embassy_executor::task]
pub async fn toggle_on_touch(mut sensor: TouchSensor, motor: Motor) -> ! {
    let timer = Timer::new(sensor.timer.reborrow());
    timer.set_frequency(Frequency::F16MHz);
    timer.start();
    let capture = timer.cc(1);

    let mut touched = false;
    let mut streak = 0;
    loop {
        // Drain the pad. Dropping the output releases the pin, and the pull-up starts charging it.
        let drain = Output::new(sensor.pin.reborrow(), Level::Low, OutputDrive::Standard);
        Delay::after(DRAIN_TIME).await;
        capture.write(NO_EDGE);
        timer.clear();
        drop(drain);

        let edge = InputChannel::new(
            sensor.gpiote.reborrow(),
            sensor.pin.reborrow(),
            Pull::None,
            InputChannelPolarity::LoToHi,
        );
        let mut ppi = Ppi::new_one_to_one(sensor.ppi.reborrow(), edge.event_in(), capture.task_capture());
        ppi.enable();
        Delay::after(CHARGE_TIME).await;
        let ticks = capture.read();
        drop(ppi);
        drop(edge);

        // Debounce: only change state after several samples agree.
        if (ticks >= THRESHOLD) != touched {
            streak += 1;
            if streak >= DEBOUNCE {
                touched = !touched;
                streak = 0;
                defmt::debug!("logo {}: {} ticks", if touched { "touched" } else { "released" }, ticks);
                if touched {
                    let speed = MOTORS.toggle(motor, MAX_SPEED);
                    defmt::info!("logo: motor {} -> {}", motor, speed);
                }
            }
        } else {
            streak = 0;
        }
    }
}
