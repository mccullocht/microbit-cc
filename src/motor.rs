//! Circuit Cube motor commands and the shared desired-speed state.
//!
//! Input tasks publish the speed they want for each motor with [`MotorState::set`]; the BLE task
//! picks up changes with [`MotorState::changed`] and forwards them to the cube. Each motor is
//! backed by a [`Signal`], so only the latest speed is kept: a burst of input changes collapses
//! into one command and a "stop" can never be lost behind a full queue.

use embassy_futures::select::select_array;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Largest speed magnitude the cube accepts.
pub const MAX_SPEED: i16 = 255;

/// One of the cube's motor outputs.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Motor {
    A,
    B,
    C,
}

impl Motor {
    pub const ALL: [Motor; 3] = [Motor::A, Motor::B, Motor::C];

    const fn index(self) -> usize {
        self as usize
    }

    const fn channel(self) -> u8 {
        match self {
            Motor::A => b'a',
            Motor::B => b'b',
            Motor::C => b'c',
        }
    }
}

/// A command setting one motor's speed, encoded on the wire as `(+|-)ddd(a|b|c)`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct Command {
    pub motor: Motor,
    /// Signed speed in `-MAX_SPEED..=MAX_SPEED`; the sign selects direction and 0 stops the motor.
    pub speed: i16,
}

impl Command {
    pub const LEN: usize = 5;

    /// Creates a command, clamping `speed` to the range the cube accepts.
    pub fn new(motor: Motor, speed: i16) -> Self {
        Self {
            motor,
            speed: speed.clamp(-MAX_SPEED, MAX_SPEED),
        }
    }

    /// Encodes the command as ASCII, e.g. `+255a`, `-064b` or `+000c`.
    pub fn encode(&self) -> [u8; Self::LEN] {
        let magnitude = self.speed.unsigned_abs();
        [
            if self.speed < 0 { b'-' } else { b'+' },
            b'0' + (magnitude / 100) as u8,
            b'0' + (magnitude / 10 % 10) as u8,
            b'0' + (magnitude % 10) as u8,
            self.motor.channel(),
        ]
    }
}

/// Latest requested speed for each motor, waiting to be sent to the cube.
pub struct MotorState {
    pending: [Signal<CriticalSectionRawMutex, i16>; Motor::ALL.len()],
}

impl MotorState {
    pub const fn new() -> Self {
        Self {
            pending: [const { Signal::new() }; Motor::ALL.len()],
        }
    }

    /// Requests a new speed for `motor`, replacing any request that has not been sent yet.
    pub fn set(&self, motor: Motor, speed: i16) {
        self.pending[motor.index()].signal(speed);
    }

    /// Waits for the next speed request on any motor.
    pub async fn changed(&self) -> Command {
        let (speed, i) = select_array(self.pending.each_ref().map(|s| s.wait())).await;
        Command::new(Motor::ALL[i], speed)
    }

    /// Takes the pending request for `motor`, if there is one.
    pub fn take(&self, motor: Motor) -> Option<i16> {
        self.pending[motor.index()].try_take()
    }
}

pub static MOTORS: MotorState = MotorState::new();
