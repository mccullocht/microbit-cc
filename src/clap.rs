//! Clap detection with the micro:bit's microphone.
//!
//! The microphone is an analog MEMS part on P0.05, powered from P0.20. It is sampled
//! continuously by the SAADC, triggered by a timer through PPI, and processed in 10 ms blocks.
//! A clap is a block whose peak-to-peak level jumps well above the background noise and falls
//! back within [`MAX_CLAP`]; anything louder for longer (speech, music, motors) is ignored.

use embassy_nrf::Peri;
use embassy_nrf::gpio::Output;
use embassy_nrf::peripherals::{PPI_CH0, PPI_CH1, TIMER1};
use embassy_nrf::saadc::{CallbackResult, Saadc};
use embassy_nrf::timer::Frequency;

use crate::motor::{MAX_SPEED, MOTORS, Motor};

/// Sample timer ticks per sample: 16 MHz / 1000 = 16 kHz.
const SAMPLE_PERIOD: u32 = 1000;
/// Samples per block: 10 ms at 16 kHz.
const BLOCK_LEN: usize = 160;

/// Quietest peak-to-peak level, in ADC counts, that can count as a clap. With the SAADC at
/// gain 4 a count is about 37 µV, so this is roughly 7 mV peak to peak.
const MIN_TRIGGER: f32 = 200.0;
/// How far above the background noise a block must be to start a clap.
const TRIGGER_RATIO: f32 = 4.0;
/// A clap has ended once the level drops below this fraction of its trigger level.
const RELEASE_RATIO: f32 = 0.5;
/// Background noise smoothing: each quiet block moves it 1/32 of the way to its own level.
const BACKGROUND_SMOOTHING: f32 = 1.0 / 32.0;

/// Longest a clap may stay loud, in blocks.
const MAX_CLAP: u8 = 10;
/// Time after a clap during which another is not detected, in blocks. Rides out echoes.
const LOCKOUT: u8 = 25;
/// Time to ignore after power-up while the microphone's coupling capacitor settles, in blocks.
const STARTUP: u8 = 50;

/// Peripherals that pace the microphone sampling: the timer triggers each sample through one PPI
/// channel, and the other restarts the SAADC on each buffer so no samples are missed.
pub struct SampleClock {
    pub timer: Peri<'static, TIMER1>,
    pub sample_ppi: Peri<'static, PPI_CH0>,
    pub restart_ppi: Peri<'static, PPI_CH1>,
}

/// Toggles `motor` between stopped and full speed each time a clap is heard.
#[embassy_executor::task]
pub async fn toggle_on_clap(
    mut mic: Saadc<'static, 1>,
    _mic_power: Output<'static>,
    mut clock: SampleClock,
    motor: Motor,
) -> ! {
    defmt::info!("mic: calibrating ADC");
    mic.calibrate().await;
    defmt::info!("mic: sampling");
    let mut detector = ClapDetector::new();
    let mut bufs = [[[0; 1]; BLOCK_LEN]; 2];
    loop {
        // Note: the SAADC's internal-timer mode (`run_timer_sampler`) never produced samples here.
        let sampling = mic.run_task_sampler(
            clock.timer.reborrow(),
            clock.restart_ppi.reborrow(),
            clock.sample_ppi.reborrow(),
            Frequency::F16MHz,
            SAMPLE_PERIOD,
            &mut bufs,
            |samples: &[[i16; 1]]| {
                if detector.process(level(samples)) {
                    let speed = if MOTORS.requested(motor) == 0 { MAX_SPEED } else { 0 };
                    defmt::info!("clap: motor {} -> {}", motor, speed);
                    MOTORS.set(motor, speed);
                }
                CallbackResult::Continue
            },
        );
        sampling.await;
    }
}

/// Peak-to-peak level of a block, which ignores the microphone's DC bias.
fn level(samples: &[[i16; 1]]) -> f32 {
    let (min, max) = samples
        .iter()
        .fold((i16::MAX, i16::MIN), |(lo, hi), s| (lo.min(s[0]), hi.max(s[0])));
    max.saturating_sub(min) as f32
}

enum State {
    /// Listening for the start of a clap.
    Quiet,
    /// A loud block was seen `blocks` ago; waiting for it to die away.
    Loud { blocks: u8, release: f32 },
    /// Ignoring input for `blocks` more blocks.
    Ignore { blocks: u8 },
}

struct ClapDetector {
    state: State,
    /// Smoothed peak-to-peak level of the background noise.
    background: f32,
}

impl ClapDetector {
    fn new() -> Self {
        Self {
            state: State::Ignore { blocks: STARTUP },
            background: 0.0,
        }
    }

    fn trigger_level(&self) -> f32 {
        (self.background * TRIGGER_RATIO).max(MIN_TRIGGER)
    }

    /// Processes one block's peak-to-peak level and returns whether it completed a clap.
    fn process(&mut self, level: f32) -> bool {
        let trigger = self.trigger_level();

        let (next, clap) = match self.state {
            State::Quiet if level >= trigger => (
                State::Loud {
                    blocks: 1,
                    release: trigger * RELEASE_RATIO,
                },
                false,
            ),
            State::Quiet => (State::Quiet, false),
            State::Loud { release, .. } if level < release => (State::Ignore { blocks: LOCKOUT }, true),
            State::Loud { blocks, .. } if blocks >= MAX_CLAP => {
                defmt::debug!("sustained sound ignored (level {})", level);
                (State::Ignore { blocks: LOCKOUT }, false)
            }
            State::Loud { blocks, release } => (
                State::Loud {
                    blocks: blocks + 1,
                    release,
                },
                false,
            ),
            State::Ignore { blocks: 0 } => (State::Quiet, false),
            State::Ignore { blocks } => (State::Ignore { blocks: blocks - 1 }, false),
        };

        // Keep a clap itself out of the background estimate, and cap any single block's influence
        // so that sustained noise raises the threshold gradually.
        if !matches!(next, State::Loud { .. }) {
            self.background += (level.min(trigger) - self.background) * BACKGROUND_SMOOTHING;
        }
        self.state = next;
        clap
    }
}
