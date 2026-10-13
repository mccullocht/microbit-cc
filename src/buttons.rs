//! micro:bit button input.

use embassy_nrf::gpio::Input;
use embassy_time::{Duration, Timer};

use crate::motor::{MAX_SPEED, MOTORS, Motor};

/// Speed a motor runs at while its button is held.
const BUTTON_SPEED: i16 = MAX_SPEED;

/// Time to ignore a button after it changes state, to ride out contact bounce.
const DEBOUNCE: Duration = Duration::from_millis(20);

/// Runs `motor` forward while `button` (active low) is held and stops it on release.
#[embassy_executor::task(pool_size = 2)]
pub async fn hold_to_run(mut button: Input<'static>, motor: Motor) -> ! {
    loop {
        button.wait_for_low().await;
        defmt::info!("motor {}: run", motor);
        MOTORS.set(motor, BUTTON_SPEED);
        Timer::after(DEBOUNCE).await;

        button.wait_for_high().await;
        defmt::info!("motor {}: stop", motor);
        MOTORS.set(motor, 0);
        Timer::after(DEBOUNCE).await;
    }
}
