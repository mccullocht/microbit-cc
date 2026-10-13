//! BLE central that finds a Circuit Cube, connects to it and forwards motor commands.
//!
//! The cube exposes the Nordic UART Service (NUS); commands are ASCII strings written to the
//! service's RX characteristic.

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use nrf_sdc::SoftdeviceController;
use trouble_host::advertise::AdStructure;
use trouble_host::prelude::*;

use crate::motor::{Command, MOTORS, Motor};

type Controller<'d> = SoftdeviceController<'d>;
type Error = BleHostError<nrf_sdc::Error>;

/// Nordic UART Service, advertised by the cube.
const NUS_SERVICE: u128 = 0x6e400001_b5a3_f393_e0a9_e50e24dcca9e;
/// NUS RX characteristic: the cube receives commands written here.
const NUS_RX: u128 = 0x6e400002_b5a3_f393_e0a9_e50e24dcca9e;

/// Advertised name prefix used by Circuit Cubes ("Tenka ...").
const CUBE_NAME_PREFIX: &[u8] = b"Tenka";

const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
const RETRY_DELAY: Duration = Duration::from_secs(1);

/// Max simultaneous connections.
const CONNECTIONS_MAX: usize = 1;
/// Max L2CAP channels: signalling + ATT.
const L2CAP_CHANNELS_MAX: usize = 2;
/// Max GATT services the client tracks.
const SERVICES_MAX: usize = 4;

/// Runs the BLE host forever: scan for a cube, connect, forward motor commands, and start over
/// whenever the connection drops.
pub async fn run(controller: Controller<'_>, address: Address) -> ! {
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut runner = stack.runner();
    let mut central = stack.central();
    let finder = CubeFinder::new();

    let app = async {
        // Speeds last requested for each motor. Kept across connections so a reconnect restores
        // whatever the buttons are currently asking for.
        let mut speeds = [0i16; Motor::ALL.len()];
        loop {
            match connect_and_drive(&stack, &mut central, &finder, &mut speeds).await {
                Ok(()) => defmt::info!("disconnected"),
                Err(e) => defmt::warn!("connection ended: {:?}", e),
            }
            Timer::after(RETRY_DELAY).await;
        }
    };

    match select(runner.run_with_handler(&finder), app).await {
        Either::First(Ok(())) => defmt::panic!("BLE host stopped"),
        Either::First(Err(e)) => defmt::panic!("BLE host failed: {:?}", e),
        Either::Second(never) => never,
    }
}

async fn connect_and_drive(
    stack: &Stack<'_, Controller<'_>, DefaultPacketPool>,
    central: &mut Central<'_, Controller<'_>, DefaultPacketPool>,
    finder: &CubeFinder,
    speeds: &mut [i16; Motor::ALL.len()],
) -> Result<(), Error> {
    defmt::info!("scanning for Circuit Cube");
    let target = finder.find(central).await?;
    defmt::info!("found cube at {:?}, connecting", target);

    let config = ConnectConfig {
        connect_params: Default::default(),
        scan_config: ScanConfig {
            filter_accept_list: &[target],
            ..Default::default()
        },
    };
    let conn = with_timeout(CONNECT_TIMEOUT, central.connect(&config))
        .await
        .map_err(|_| Error::from(trouble_host::Error::Timeout))??;
    defmt::info!("connected");

    let client = GattClient::<_, DefaultPacketPool, SERVICES_MAX>::new(stack, &conn).await?;
    match select(client.task(), drive(&client, speeds)).await {
        Either::First(r) | Either::Second(r) => r,
    }
}

/// Looks up the cube's command characteristic, then sends motor commands until an error
/// (typically a disconnect) occurs.
async fn drive(
    client: &GattClient<'_, Controller<'_>, DefaultPacketPool, SERVICES_MAX>,
    speeds: &mut [i16; Motor::ALL.len()],
) -> Result<(), Error> {
    let services = client.services_by_uuid(&uuid(NUS_SERVICE)).await?;
    let service = services
        .first()
        .ok_or(Error::from(trouble_host::Error::NotFound))?;
    let rx: Characteristic<[u8]> = client.characteristic_by_uuid(service, &uuid(NUS_RX)).await?;
    let acked = rx.props.any(&[CharacteristicProp::Write]);
    defmt::info!("ready (write {})", if acked { "with response" } else { "without response" });

    let send = async |command: Command| {
        defmt::info!("send {:?}", command);
        let bytes = command.encode();
        if acked {
            client.write_characteristic(&rx, &bytes).await
        } else {
            client.write_characteristic_without_response(&rx, &bytes).await
        }
    };

    // Bring the cube in line with the current inputs, including requests made while disconnected.
    for motor in Motor::ALL {
        if let Some(speed) = MOTORS.take(motor) {
            speeds[motor as usize] = speed;
        }
        send(Command::new(motor, speeds[motor as usize])).await?;
    }

    loop {
        let command = MOTORS.changed().await;
        speeds[command.motor as usize] = command.speed;
        send(command).await?;
    }
}

/// Converts a UUID written in its usual big-endian form into TrouBLE's little-endian layout.
const fn uuid(value: u128) -> Uuid {
    Uuid::new_long(value.to_le_bytes())
}

/// Scans advertising reports for a Circuit Cube.
struct CubeFinder {
    found: Signal<NoopRawMutex, Address>,
}

impl CubeFinder {
    fn new() -> Self {
        Self {
            found: Signal::new(),
        }
    }

    /// Scans until a cube is seen and returns its address.
    async fn find(
        &self,
        central: &mut Central<'_, Controller<'_>, DefaultPacketPool>,
    ) -> Result<Address, Error> {
        self.found.reset();
        let mut scanner = Scanner::new(central);
        let session = scanner.scan(&ScanConfig::default()).await?;
        let address = self.found.wait().await;
        session.stop().await;
        Ok(address)
    }

    /// Whether advertising data identifies a Circuit Cube, by NUS service UUID or name.
    fn is_cube(data: &[u8]) -> bool {
        let nus = NUS_SERVICE.to_le_bytes();
        AdStructure::decode(data).flatten().any(|ad| match ad {
            AdStructure::IncompleteServiceUuids128(uuids)
            | AdStructure::CompleteServiceUuids128(uuids) => uuids.contains(&nus),
            AdStructure::CompleteLocalName(name) | AdStructure::ShortenedLocalName(name) => {
                name.starts_with(CUBE_NAME_PREFIX)
            }
            _ => false,
        })
    }
}

impl EventHandler for CubeFinder {
    fn on_adv_reports(&self, mut reports: LeAdvReportsIter<'_>) {
        while let Some(Ok(report)) = reports.next() {
            if Self::is_cube(report.data) {
                self.found.signal(Address {
                    kind: report.addr_kind,
                    addr: report.addr,
                });
            }
        }
    }
}
