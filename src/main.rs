//! micro:bit v2 remote for a Tenka Circuit Cube.
//!
//! Holding button A runs motor b; holding button B runs motor c.

#![no_std]
#![no_main]

mod ble;
mod buttons;
mod motor;

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, pac, rng};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use trouble_host::prelude::{Address, DefaultPacketPool, PacketPool};
use {defmt_rtt as _, panic_probe as _};

use crate::motor::Motor;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
});

/// Outgoing / incoming L2CAP buffers per link.
const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;

/// SoftDevice Controller memory for one central link with the buffers above.
const SDC_MEM_SIZE: usize = 5376;

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

    // Buttons A and B have external pull-ups and read low while pressed.
    let button_a = Input::new(p.P0_14, Pull::None);
    let button_b = Input::new(p.P0_23, Pull::None);
    spawner.spawn(unwrap!(buttons::hold_to_run(button_a, Motor::A)));
    spawner.spawn(unwrap!(buttons::hold_to_run(button_b, Motor::C)));

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
