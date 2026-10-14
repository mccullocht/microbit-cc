//! 5×5 LED matrix.
//!
//! Each LED sits between a row line (anode) and a column line (cathode), so it lights when its row
//! is high and its column is low. Only one row is driven at a time; cycling through all five every
//! 10 ms is fast enough to look steady.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_nrf::gpio::Output;
use embassy_time::{Duration, Timer};

/// How long each row stays lit per refresh (5 rows → 100 Hz refresh).
const ROW_TIME: Duration = Duration::from_millis(2);

/// A 5×5 image. Row 0 is the top (logo end); column 0 is the left.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Image(u32);

impl Image {
    /// Builds an image from five rows, top first, each written left to right as the low five
    /// bits, e.g. `0b10001` lights the two outer LEDs.
    pub const fn from_rows(rows: [u8; 5]) -> Self {
        let mut bits = 0;
        let mut r = 0;
        while r < 5 {
            bits |= ((rows[r] & 0b11111) as u32) << (5 * r);
            r += 1;
        }
        Image(bits)
    }

    fn row(self, r: usize) -> u8 {
        ((self.0 >> (5 * r)) & 0b11111) as u8
    }
}

static FRAME: AtomicU32 = AtomicU32::new(0);

/// Replaces the image on the display.
pub fn show(image: Image) {
    FRAME.store(image.0, Ordering::Relaxed);
}

/// Refreshes the display forever. `rows` and `cols` are given top to bottom and left to right.
#[embassy_executor::task]
pub async fn run(mut rows: [Output<'static>; 5], mut cols: [Output<'static>; 5]) -> ! {
    loop {
        for (r, row) in rows.iter_mut().enumerate() {
            let bits = Image(FRAME.load(Ordering::Relaxed)).row(r);
            for (c, col) in cols.iter_mut().enumerate() {
                let lit = bits & (0b10000 >> c) != 0;
                col.set_level((!lit).into());
            }
            row.set_high();
            Timer::after(ROW_TIME).await;
            row.set_low();
        }
    }
}
