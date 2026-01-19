//! LSM9DS0 example for RP2040
//!
//! Demonstrates reading IMU data (accelerometer + gyroscope) and temperature
//! from the LSM9DS0 sensor using Embassy tasks and channels.
//!
//! Architecture:
//! - `sensor_task`: Owns the driver exclusively, reads IMU at IMU_SAMPLE_RATE_HZ
//!   and temperature at TEMPERATURE_SAMPLE_RATE_HZ
//! - `main`: Receives sensor updates, maintains combined state, and prints updates
//!
//! This single-task approach eliminates the need for a Mutex since only one task
//! accesses the driver.

#![no_std]
#![no_main]

use lsm9ds0::{AccelDataRate, GyroDataRate, I2cInterface, Lsm9ds0, Lsm9ds0Config, MagMode};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    i2c::{self, Config, InterruptHandler},
    peripherals::I2C1,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Delay, Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

const IMU_SAMPLE_RATE_HZ: u64 = 10;
const TEMPERATURE_SAMPLE_RATE_HZ: u64 = 1;

/// How many IMU samples between each temperature sample
const TEMP_SAMPLE_INTERVAL: u64 = IMU_SAMPLE_RATE_HZ / TEMPERATURE_SAMPLE_RATE_HZ;

/// IMU reading containing accelerometer, gyroscope, and magnetometer data
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct ImuReading {
    /// Accelerometer X axis (g)
    pub accel_x: f32,
    /// Accelerometer Y axis (g)
    pub accel_y: f32,
    /// Accelerometer Z axis (g)
    pub accel_z: f32,
    /// Gyroscope X axis (degrees/sec)
    pub gyro_x: f32,
    /// Gyroscope Y axis (degrees/sec)
    pub gyro_y: f32,
    /// Gyroscope Z axis (degrees/sec)
    pub gyro_z: f32,
    /// Magnetometer X axis (gauss)
    pub mag_x: f32,
    /// Magnetometer Y axis (gauss)
    pub mag_y: f32,
    /// Magnetometer Z axis (gauss)
    pub mag_z: f32,
}

/// Messages sent from sensor task to the main receiver
#[derive(Debug, Clone, Copy, Format)]
pub enum SensorMessage {
    Imu(ImuReading),
    Temperature(f32),
}

/// Combined telemetry state holding all sensor readings
#[derive(Debug, Clone, Copy, Default, Format)]
pub struct TelemetryState {
    pub imu: ImuReading,
    pub temperature_celsius: f32,
}

/// Channel for sensor data (capacity of 8 messages)
static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorMessage, 8> = Channel::new();

/// Type alias for the I2C bus used by the sensor
type I2cBus = i2c::I2c<'static, I2C1, i2c::Async>;

/// Type alias for the LSM9DS0 driver with I2C interface
type Imu = Lsm9ds0<I2cInterface<I2cBus>>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("LSM9DS0 RP2040 Example");
    info!("Setting up I2C...");

    // I2C pins. Don't forget, on pico H, this "PIN_14" is actually physical pin 19...
    let sda = p.PIN_14;
    let scl = p.PIN_15;

    let i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Config::default());

    // Create LSM9DS0 driver with I2C interface
    let interface = I2cInterface::init(i2c);
    // If the SA0_G and SA0_XM are connected to VCC (instead of assumed GND), then explicitly
    // specify the gyro and accel/mag's address.
    // use lsm9ds0::registers::device_constants::{gyro, xm};
    // let interface = I2cInterface::init_with_addresses(i2c, gyro::I2C_ADDR_1, xm::I2C_ADDR_1);

    // Configure sensors - defaults have everything in power-down mode
    let config = Lsm9ds0Config::new()
        .with_gyro_enabled(true)
        .with_gyro_data_rate(GyroDataRate::Hz95)
        .with_accel_data_rate(AccelDataRate::Hz100)
        .with_mag_mode(MagMode::ContinuousConversion)
        .with_temperature_enabled(true);

    let mut imu = Lsm9ds0::new_with_config(interface, config);

    info!("Initializing LSM9DS0...");
    if let Err(e) = imu.init(&mut Delay).await {
        error!("Failed to initialize sensor: {:?}", Debug2Format(&e));
        return;
    }
    info!("LSM9DS0 initialized successfully");

    // Channel is for the sensor task to send sensor readings to the main task for processing
    let sender = SENSOR_CHANNEL.sender();
    let receiver = SENSOR_CHANNEL.receiver();

    info!("Starting sensor task...");
    spawner.spawn(sensor_task(imu, sender)).unwrap();

    // Main loop receive messages and update sensor state
    let mut state = TelemetryState::default();

    info!("Starting sensor readings...");

    loop {
        let message = receiver.receive().await;

        match message {
            SensorMessage::Imu(reading) => state.imu = reading,
            SensorMessage::Temperature(temp) => state.temperature_celsius = temp,
        }

        info!(
            "Accel[{}, {}, {}]g Gyro[{}, {}, {}]dps Mag[{}, {}, {}]G Temp: {}°C",
            state.imu.accel_x / 1000.0,
            state.imu.accel_y / 1000.0,
            state.imu.accel_z / 1000.0,
            state.imu.gyro_x / 1000.0,
            state.imu.gyro_y / 1000.0,
            state.imu.gyro_z / 1000.0,
            state.imu.mag_x / 1000.0,
            state.imu.mag_y / 1000.0,
            state.imu.mag_z / 1000.0,
            state.temperature_celsius
        );
    }
}

/// Single task that owns the driver and handles all readings
///
/// Reads IMU data at IMU_SAMPLE_RATE_HZ (10 Hz) and temperature at TEMPERATURE_SAMPLE_RATE_HZ (1
/// Hz) by reading temperature every TEMP_SAMPLE_INTERVAL iterations.
#[embassy_executor::task]
async fn sensor_task(
    mut imu: Imu,
    sender: Sender<'static, CriticalSectionRawMutex, SensorMessage, 8>,
) {
    let mut ticker = Ticker::every(Duration::from_hz(IMU_SAMPLE_RATE_HZ));
    let mut sample_count: u64 = 0;

    loop {
        ticker.next().await;
        sample_count += 1;

        // Read IMU data every tick (10 Hz)
        let accel = match imu.read_accel().await {
            Ok((x, y, z)) => (x, y, z),
            Err(e) => {
                warn!("Failed to read accelerometer: {:?}", Debug2Format(&e));
                continue;
            }
        };

        let gyro = match imu.read_gyro().await {
            Ok((x, y, z)) => (x, y, z),
            Err(e) => {
                warn!("Failed to read gyroscope: {:?}", Debug2Format(&e));
                continue;
            }
        };

        let mag = match imu.read_mag().await {
            Ok((x, y, z)) => (x, y, z),
            Err(e) => {
                warn!("Failed to read magnetometer: {:?}", Debug2Format(&e));
                continue;
            }
        };

        let reading = ImuReading {
            accel_x: accel.0.into(),
            accel_y: accel.1.into(),
            accel_z: accel.2.into(),
            gyro_x: gyro.0.into(),
            gyro_y: gyro.1.into(),
            gyro_z: gyro.2.into(),
            mag_x: mag.0.into(),
            mag_y: mag.1.into(),
            mag_z: mag.2.into(),
        };

        sender.send(SensorMessage::Imu(reading)).await;

        // Read temperature at slower rate (1 Hz)
        if sample_count >= TEMP_SAMPLE_INTERVAL {
            sample_count = 0;

            match imu.read_temp().await {
                Ok(temp) => {
                    sender.send(SensorMessage::Temperature(temp.into())).await;
                }
                Err(e) => {
                    warn!("Failed to read temperature: {:?}", Debug2Format(&e));
                }
            }
        }
    }
}
