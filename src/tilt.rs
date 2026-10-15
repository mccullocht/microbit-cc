//! Tilt control from the LSM303AGR accelerometer.
//!
//! Tilt control is on only while button A or B is held; otherwise the motors are stopped and the
//! display shows a lock (its whole outer edge lit).
//!
//! While it's on, with the board held flat and face up, tilting 20–40° in any direction lights the
//! edge on that side, and tilting further shows a chevron pointing that way. The tilt also drives a pair of
//! left and right motors, at half speed for a slight tilt and full speed for a steep one:
//! forward or back runs both that way, and tilting left or right spins them in opposite
//! directions to turn that way. Level stops them.
//!
//! A still board's acceleration is just gravity, so the tilt along an axis is asin(a / |a|). The
//! thresholds below are compared as sines, squared, so no trigonometry or square roots are needed.

use embassy_futures::select::select;
use embassy_nrf::gpio::Input;
use embassy_nrf::twim::{self, Twim};
use embassy_time::{Delay, Duration, Ticker};
use lsm303agr::interface::I2cInterface;
use lsm303agr::mode::MagOneShot;
use lsm303agr::{AccelMode, AccelOutputDataRate, AccelScale, Lsm303agr};

use crate::display::{self, Image};
use crate::motor::{MAX_SPEED, MOTORS, Motor};

type Sensor = Lsm303agr<I2cInterface<Twim<'static>>, MagOneShot>;
type Error = lsm303agr::Error<twim::Error>;

const SAMPLE_PERIOD: Duration = Duration::from_millis(20);
/// Low-pass filter: each sample moves the smoothed value this fraction of the way towards it.
const SMOOTHING: f32 = 0.3;
/// Motor speed for a slight tilt; a steep tilt runs at `MAX_SPEED`.
const SLIGHT_SPEED: i16 = 128;

/// sin 20° and sin 40°: tilts that light an edge and show an arrow.
const EDGE_ON: f32 = 0.342;
const ARROW_ON: f32 = 0.643;
/// sin 17° and sin 37°: once shown, an edge or arrow stays until the tilt drops below these, so
/// the display doesn't flicker at a threshold.
const EDGE_OFF: f32 = 0.292;
const ARROW_OFF: f32 = 0.602;
/// When tilted diagonally, the other axis must be this much steeper to change direction.
const SWITCH_RATIO: f32 = 1.25;

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Direction {
    Forward,
    Back,
    Left,
    Right,
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Tilt {
    Level,
    Slight(Direction),
    Steep(Direction),
}

impl Tilt {
    fn direction(self) -> Option<Direction> {
        match self {
            Tilt::Level => None,
            Tilt::Slight(d) | Tilt::Steep(d) => Some(d),
        }
    }

    /// Speeds for the (left, right) motors; positive is forward.
    fn speeds(self) -> (i16, i16) {
        let (direction, speed) = match self {
            Tilt::Level => return (0, 0),
            Tilt::Slight(d) => (d, SLIGHT_SPEED),
            Tilt::Steep(d) => (d, MAX_SPEED),
        };
        match direction {
            Direction::Forward => (speed, speed),
            Direction::Back => (-speed, -speed),
            Direction::Left => (-speed, speed),
            Direction::Right => (speed, -speed),
        }
    }

    fn image(self) -> Image {
        use Direction::*;
        match self {
            Tilt::Level => Image::from_rows([0, 0, 0b00100, 0, 0]),
            Tilt::Slight(Forward) => Image::from_rows([0b11111, 0, 0, 0, 0]),
            Tilt::Slight(Back) => Image::from_rows([0, 0, 0, 0, 0b11111]),
            Tilt::Slight(Left) => Image::from_rows([0b10000; 5]),
            Tilt::Slight(Right) => Image::from_rows([0b00001; 5]),
            Tilt::Steep(Forward) => Image::from_rows([0, 0b00100, 0b01010, 0, 0]),
            Tilt::Steep(Back) => Image::from_rows([0, 0, 0b01010, 0b00100, 0]),
            Tilt::Steep(Left) => Image::from_rows([0, 0b00100, 0b01000, 0b00100, 0]),
            Tilt::Steep(Right) => Image::from_rows([0, 0b00100, 0b00010, 0b00100, 0]),
        }
    }
}

/// Shown while no button is held and tilt control is off.
const LOCK_IMAGE: Image = Image::from_rows([0b11111, 0b10001, 0b10001, 0b10001, 0b11111]);
/// Shown if the accelerometer can't be set up.
const ERROR_IMAGE: Image = Image::from_rows([0b10001, 0b01010, 0b00100, 0b01010, 0b10001]);
/// Consecutive samples with both buttons up before tilt control turns off, so contact bounce on
/// release doesn't turn it straight back on.
const RELEASE_SAMPLES: u8 = 3;

/// The motors on each side of the model.
pub struct Drive {
    pub left: Motor,
    pub right: Motor,
}

impl Drive {
    /// Sends the speed changes needed to go from `from` to `to`.
    fn change(&self, from: Tilt, to: Tilt) {
        let (left, right) = from.speeds();
        let (next_left, next_right) = to.speeds();
        if next_left != left {
            MOTORS.set(self.left, next_left);
        }
        if next_right != right {
            MOTORS.set(self.right, next_right);
        }
    }
}

/// Buttons A and B, which read low while pressed.
pub struct Buttons {
    pub a: Input<'static>,
    pub b: Input<'static>,
}

impl Buttons {
    async fn wait_for_either(&mut self) {
        select(self.a.wait_for_low(), self.b.wait_for_low()).await;
    }

    fn either_held(&self) -> bool {
        self.a.is_low() || self.b.is_low()
    }
}

/// While either button is held, reads the accelerometer, keeps the display showing the current
/// tilt, and drives the motors. Otherwise the motors are stopped and the display shows a lock.
#[embassy_executor::task]
pub async fn run(i2c: Twim<'static>, drive: Drive, mut buttons: Buttons) -> ! {
    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    if let Err(e) = setup(&mut sensor).await {
        defmt::error!("accelerometer setup failed: {:?}", defmt::Debug2Format(&e));
        display::show(ERROR_IMAGE);
        core::future::pending().await
    }
    defmt::info!("accelerometer ready");

    loop {
        display::show(LOCK_IMAGE);
        buttons.wait_for_either().await;
        defmt::info!("tilt control on");
        drive_while_held(&mut sensor, &drive, &buttons).await;
        defmt::info!("tilt control off");
    }
}

/// Drives the motors from the tilt until both buttons are released, then stops them.
async fn drive_while_held(sensor: &mut Sensor, drive: &Drive, buttons: &Buttons) {
    let mut tilt = Tilt::Level;
    display::show(tilt.image());
    let mut smoothed: Option<Vector> = None;
    let mut released = 0;
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    loop {
        ticker.next().await;
        if buttons.either_held() {
            released = 0;
        } else {
            released += 1;
            if released >= RELEASE_SAMPLES {
                drive.change(tilt, Tilt::Level);
                return;
            }
        }

        let sample = match sensor.acceleration().await {
            Ok(a) => Vector::from_sensor(a.x_mg(), a.y_mg(), a.z_mg()),
            Err(e) => {
                defmt::warn!("accelerometer read failed: {:?}", defmt::Debug2Format(&e));
                continue;
            }
        };
        let v = match smoothed {
            Some(prev) => prev.towards(sample, SMOOTHING),
            None => sample,
        };
        smoothed = Some(v);

        let next = v.tilt(tilt);
        if next != tilt {
            defmt::info!("tilt: {}", next);
            display::show(next.image());
            drive.change(tilt, next);
            tilt = next;
        }
    }
}

async fn setup(sensor: &mut Sensor) -> Result<(), Error> {
    sensor.init().await?;
    sensor
        .set_accel_mode_and_odr(&mut Delay, AccelMode::HighResolution, AccelOutputDataRate::Hz50)
        .await?;
    sensor.set_accel_scale(AccelScale::G2).await
}

/// Acceleration in mg along the board's axes.
#[derive(Clone, Copy)]
struct Vector {
    /// Positive when the right edge is lower.
    right: f32,
    /// Positive when the top (logo) edge is lower.
    forward: f32,
    /// Through the board.
    z: f32,
}

impl Vector {
    /// Converts from the sensor's axes. CODAL's LSM303 driver and coordinate space work out to
    /// board x = sensor x and board y = -sensor y, and CODAL reads +x as tilted right and -y as
    /// tilted logo-down.
    fn from_sensor(x: i32, y: i32, z: i32) -> Self {
        Vector {
            right: x as f32,
            forward: y as f32,
            z: z as f32,
        }
    }

    fn towards(self, target: Vector, fraction: f32) -> Self {
        Vector {
            right: self.right + (target.right - self.right) * fraction,
            forward: self.forward + (target.forward - self.forward) * fraction,
            z: self.z + (target.z - self.z) * fraction,
        }
    }

    /// Classifies the tilt, given the previous classification for hysteresis.
    fn tilt(self, prev: Tilt) -> Tilt {
        let magnitude_sq = self.right * self.right + self.forward * self.forward + self.z * self.z;
        if magnitude_sq == 0.0 {
            return Tilt::Level;
        }

        // Use whichever axis is tilted more, but stick with the current one until the other is
        // clearly steeper.
        let along_forward = match prev.direction() {
            Some(Direction::Forward | Direction::Back) => {
                self.right.abs() <= self.forward.abs() * SWITCH_RATIO
            }
            Some(Direction::Left | Direction::Right) => {
                self.forward.abs() > self.right.abs() * SWITCH_RATIO
            }
            None => self.forward.abs() >= self.right.abs(),
        };
        let (component, direction) = if along_forward {
            let d = if self.forward > 0.0 { Direction::Forward } else { Direction::Back };
            (self.forward, d)
        } else {
            let d = if self.right > 0.0 { Direction::Right } else { Direction::Left };
            (self.right, d)
        };

        let same = prev.direction() == Some(direction);
        let arrow = if same && matches!(prev, Tilt::Steep(_)) { ARROW_OFF } else { ARROW_ON };
        let edge = if same { EDGE_OFF } else { EDGE_ON };
        // |component| / |a| >= sin θ, squared.
        let exceeds = |sin: f32| component * component >= sin * sin * magnitude_sq;
        if exceeds(arrow) {
            Tilt::Steep(direction)
        } else if exceeds(edge) {
            Tilt::Slight(direction)
        } else {
            Tilt::Level
        }
    }
}
