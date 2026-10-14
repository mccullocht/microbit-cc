//! Shows how the board is tilted on the LED matrix, from the LSM303AGR accelerometer.
//!
//! With the board held flat and face up, tilting 10–40° in any direction lights the edge on that
//! side, and tilting further shows a chevron pointing that way.
//!
//! A still board's acceleration is just gravity, so the tilt along an axis is asin(a / |a|). The
//! thresholds below are compared as sines, squared, so no trigonometry or square roots are needed.

use embassy_nrf::twim::{self, Twim};
use embassy_time::{Delay, Duration, Ticker};
use lsm303agr::interface::I2cInterface;
use lsm303agr::mode::MagOneShot;
use lsm303agr::{AccelMode, AccelOutputDataRate, AccelScale, Lsm303agr};

use crate::display::{self, Image};

type Sensor = Lsm303agr<I2cInterface<Twim<'static>>, MagOneShot>;
type Error = lsm303agr::Error<twim::Error>;

const SAMPLE_PERIOD: Duration = Duration::from_millis(20);
/// Low-pass filter: each sample moves the smoothed value this fraction of the way towards it.
const SMOOTHING: f32 = 0.3;

/// sin 10° and sin 40°: tilts that light an edge and show an arrow.
const EDGE_ON: f32 = 0.174;
const ARROW_ON: f32 = 0.643;
/// sin 7° and sin 37°: once shown, an edge or arrow stays until the tilt drops below these, so
/// the display doesn't flicker at a threshold.
const EDGE_OFF: f32 = 0.122;
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

/// Shown if the accelerometer can't be set up.
const ERROR_IMAGE: Image = Image::from_rows([0b10001, 0b01010, 0b00100, 0b01010, 0b10001]);

/// Reads the accelerometer and keeps the display showing the current tilt.
#[embassy_executor::task]
pub async fn run(i2c: Twim<'static>) -> ! {
    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    if let Err(e) = setup(&mut sensor).await {
        defmt::error!("accelerometer setup failed: {:?}", defmt::Debug2Format(&e));
        display::show(ERROR_IMAGE);
        core::future::pending().await
    }
    defmt::info!("accelerometer ready");

    let mut tilt = Tilt::Level;
    display::show(tilt.image());
    let mut smoothed: Option<Vector> = None;
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    loop {
        ticker.next().await;
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
