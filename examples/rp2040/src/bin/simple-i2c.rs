//! Async I2C test
//!
//! Run with: cargo run --bin simple-i2c
//!
//! Hardware connections:
//! - SDA: GPIO14
//! - SCL: GPIO15
//! - VCC: 3.3V
//! - GND: GND, SA0_G, SA0_XM

#![no_std]
#![no_main]

use lsm9ds0::{
    AccelDataRate, GyroDataRate, I2cInterface, Lsm9ds0, Lsm9ds0Config, MagMode,
    registers::device_constants::{gyro, xm},
};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::I2C1};
use embassy_time::{Delay, Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    let i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Config::default());

    // Create LSM9DS0 driver with I2C interface
    let interface = I2cInterface::init_with_addresses(i2c, gyro::I2C_ADDR_0, xm::I2C_ADDR_0);

    // Configure sensors - defaults have everything in power-down mode
    let config = Lsm9ds0Config::new()
        .with_gyro_enabled(true)
        .with_gyro_data_rate(GyroDataRate::Hz95)
        .with_accel_data_rate(AccelDataRate::Hz100)
        .with_mag_mode(MagMode::ContinuousConversion)
        .with_temperature_enabled(true);

    let mut imu = Lsm9ds0::new_with_config(interface, config);

    if let Err(e) = imu.init(&mut Delay).await {
        error!("Failed to initialize sensor: {:?}", Debug2Format(&e));
        return;
    }

    // Read IMU data every tick (10 Hz)
    const IMU_SAMPLE_RATE_HZ: u64 = 10;
    let mut ticker = Ticker::every(Duration::from_hz(IMU_SAMPLE_RATE_HZ));

    loop {
        ticker.next().await;

        // Read IMU data every tick
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
        let temperature_celsius = match imu.read_temp().await {
            Ok(temp) => temp,
            Err(e) => {
                warn!("Failed to read temperature: {:?}", Debug2Format(&e));
                continue;
            }
        };

        info!(
            "Accel[{}, {}, {}]g Gyro[{}, {}, {}]dps Mag[{}, {}, {}]gauss Temp: {}°C",
            accel.0.as_f32(),
            accel.1.as_f32(),
            accel.2.as_f32(),
            gyro.0.as_f32(),
            gyro.1.as_f32(),
            gyro.2.as_f32(),
            mag.0.as_f32(),
            mag.1.as_f32(),
            mag.2.as_f32(),
            temperature_celsius.as_f32(),
        );
    }
}
