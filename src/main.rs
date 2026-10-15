//! micro:bit v2 remote for a Tenka Circuit Cube.
//!
//! Tilting the board drives motors a (left) and c (right) like a tank: forward and back run both
//! together, and left and right spin them in opposite directions. Touching the logo toggles
//! motor b.
//! The LED matrix shows which way the board is tilted.

#![no_std]
#![no_main]

mod ble;
mod display;
mod motor;
mod tilt;
mod touch;

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::Peri;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::{RNG, TWISPI0};
use embassy_nrf::{bind_interrupts, interrupt, pac, rng, twim};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use trouble_host::prelude::{Address, DefaultPacketPool, PacketPool};
use {defmt_rtt as _, panic_probe as _};

use crate::motor::Motor;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    TWISPI0 => twim::InterruptHandler<TWISPI0>;
    EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
});

/// Outgoing / incoming L2CAP buffers per link.
const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;

/// SoftDevice Controller memory for one central link with the buffers above. The controller checks
/// this at startup and logs the size it needs if the configuration changes.
const SDC_MEM_SIZE: usize = 4904;

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // MPSL owns interrupt priority 0 for radio timing; keep Embassy's interrupts below it.
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    // The micro:bit v2 has no 32.768 kHz crystal, so the low-frequency clock runs from the RC
    // oscillator.
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));
    spawner.spawn(unwrap!(mpsl_task(mpsl)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );
    let mut rng = rng::Rng::new(p.RNG, Irqs);
    let mut sdc_mem = sdc::Mem::<SDC_MEM_SIZE>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem));

    // LED matrix: rows (anodes) top to bottom, columns (cathodes) left to right, all off.
    let led_row = |pin: Peri<'static, AnyPin>| Output::new(pin, Level::Low, OutputDrive::Standard);
    let led_col = |pin: Peri<'static, AnyPin>| Output::new(pin, Level::High, OutputDrive::Standard);
    let rows = [
        led_row(p.P0_21.into()),
        led_row(p.P0_22.into()),
        led_row(p.P0_15.into()),
        led_row(p.P0_24.into()),
        led_row(p.P0_19.into()),
    ];
    let cols = [
        led_col(p.P0_28.into()),
        led_col(p.P0_11.into()),
        led_col(p.P0_31.into()),
        led_col(p.P1_05.into()),
        led_col(p.P0_30.into()),
    ];
    spawner.spawn(unwrap!(display::run(rows, cols)));

    // The accelerometer is on the internal I2C bus (SDA P0.16, SCL P0.08), which has its own
    // pull-ups.
    static I2C_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let i2c = twim::Twim::new(
        p.TWISPI0,
        Irqs,
        p.P0_16,
        p.P0_08,
        twim::Config::default(),
        I2C_TX_BUF.init([0; 16]),
    );
    interrupt::TWISPI0.set_priority(Priority::P2);
    let drive = tilt::Drive {
        left: Motor::A,
        right: Motor::C,
    };
    spawner.spawn(unwrap!(tilt::run(i2c, drive)));

    let logo = touch::TouchSensor {
        pin: p.P1_04.into(),
        gpiote: p.GPIOTE_CH0,
        ppi: p.PPI_CH2,
        timer: p.TIMER2,
    };
    spawner.spawn(unwrap!(touch::toggle_on_touch(logo, Motor::B)));

    ble::run(sdc, device_address()).await
}

fn build_sdc<'d, const N: usize>(
    p: sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<sdc::SoftdeviceController<'d>, sdc::Error> {
    sdc::Builder::new()?
        .support_scan()
        .support_central()
        .central_count(1)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            L2CAP_TXQ,
            L2CAP_RXQ,
        )?
        .build(p, rng, mpsl, mem)
}

/// A random static address derived from the chip's factory-programmed device address, so each
/// micro:bit gets a stable, unique identity.
fn device_address() -> Address {
    let lo = pac::FICR.deviceaddr(0).read().to_le_bytes();
    let hi = pac::FICR.deviceaddr(1).read().to_le_bytes();
    // Random static addresses must have the two most significant bits set.
    Address::random([lo[0], lo[1], lo[2], lo[3], hi[0], hi[1] | 0xc0])
}
