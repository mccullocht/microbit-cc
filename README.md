# microbit-cc

Firmware that turns a [BBC micro:bit v2](https://microbit.org/) into a tilt remote for a
[Tenka Circuit Cube](https://circuitcubes.com/). The micro:bit connects to the cube over
Bluetooth LE. Tilting the board drives two motors like a tank, and touching the logo toggles a
third.

It's written in Rust on [Embassy](https://embassy.dev/), and uses Nordic's SoftDevice Controller
through [`nrf-sdc`](https://github.com/alexmoon/nrf-sdc) and
[TrouBLE](https://github.com/embassy-rs/trouble) for Bluetooth.

## Hardware

This controller is setup for [XV-A1](https://cdn.shopify.com/s/files/1/0013/9585/1322/files/circuit-cubes-sr-xva1.pdf?v=1629265426)
rover from a since discontinued kit. The rover has a bluetooth battery cube with 3 outputs: two
power motors that turn wheels on the side of the rover, one powers a spinner on one end of the
rover.

The firmware expects:

| Cube output | Used for                             |
| ----------- | ------------------------------------ |
| A           | Left drive motor                     |
| B           | Accessory motor, toggled by the logo |
| C           | Right drive motor                    |

So long as you build something that uses a similar drive train setup (one motor for each side) it
should be easy to wire the cube or adapt the firmware to match.

## Software setup

### Rust toolchain

Install Rust with [rustup](https://rustup.rs/). The crate uses the 2024 edition, so you need
Rust 1.85 or newer.

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Then add the target for the micro:bit v2's nRF52833 (a Cortex-M4F):

```sh
rustup target add thumbv7em-none-eabihf
```

You don't need to pass `--target`, because [.cargo/config.toml](.cargo/config.toml) makes it
the default for this project.

### libclang

The Nordic SoftDevice bindings (`nrf-sdc-sys`, `nrf-mpsl-sys`) are generated with `bindgen` at
build time, so you need libclang:

- **macOS:** install the Xcode command line tools (`xcode-select --install`).
- **Debian/Ubuntu:** `sudo apt install libclang-dev`.
- **Windows:** install LLVM (for example `winget install LLVM.LLVM`) and set `LIBCLANG_PATH` if
  bindgen can't find it.

### probe-rs

[probe-rs](https://probe.rs/) flashes the board through its on-board debug probe and shows the
firmware's log output. Install it with cargo:

```sh
cargo install probe-rs-tools --locked
```

or use one of the prebuilt installers listed on the
[probe-rs installation page](https://probe.rs/docs/getting-started/installation/).

On Linux, you also need probe-rs's udev rules before you can open the probe as a normal user.
The installation page explains how to set them up.

## Flashing

Connect the micro:bit to your computer over USB, then run:

```sh
cargo run --release
```

This builds the firmware, flashes it with `probe-rs run --chip nRF52833_xxAA` (the runner set in
[.cargo/config.toml](.cargo/config.toml)), and then streams the board's
[defmt](https://defmt.ferrous-systems.com/) log output to your terminal. Press Ctrl+C to stop
watching the logs. The firmware stays on the board, so after that you can unplug it and power it
from a battery pack.

To see more detail in the logs, change the level at build time:

```sh
DEFMT_LOG=debug cargo run --release
```

## Controls

When the micro:bit powers up, it scans for a Circuit Cube and connects to the first one it
finds. A cube is recognized by its Bluetooth name (starting with "Tenka") or by the Nordic UART
service it advertises. If the connection drops, it starts looking again.

- **Lock:** while neither button is held, the drive motors are stopped and the display lights
  its whole outer edge.
- **Drive:** hold button **A** or **B** to turn on tilt control. Hold the board flat and face up,
  then tilt it in the direction you want to go:
  - Forward or back runs both drive motors that way.
  - Left or right spins the motors in opposite directions, turning the model in place.
  - A slight tilt (about 20–40°) runs at half speed. A steeper tilt runs at full speed.

  The display shows a wedge growing from the center toward the side you're tilting: one row for
  a slight tilt, and the full edge for a steep one. Let go of both buttons to stop.
- **Accessory motor:** touch the gold logo on the front to toggle motor B on (half speed) or off.

If the display shows an **X**, the accelerometer couldn't be set up. Check the log output for
details.
