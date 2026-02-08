//! Blocking I2C example using the async LSM9DS0 driver
//!
//! This demonstrates how to use the async LSM9DS0 driver with a blocking I2C bus.
//! The driver is async-only, but blocking users can wrap their calls in `block_on`.
//!
//! The key pieces:
//! 1. A small adapter that wraps a blocking `embedded_hal::i2c::I2c` to satisfy
//!    `embedded_hal_async::i2c::I2c` (the async fn just calls the blocking method).
//! 2. A similar adapter for `DelayNs`.
//! 3. `embassy_futures::block_on` to run the async driver methods synchronously.
//!
//! Run with: cargo run --bin simple-i2c-blocking
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
use embassy_futures::block_on;
use embassy_rp::i2c::{self, Config};
use embassy_time::Delay;
use {defmt_rtt as _, panic_probe as _};

/// Wraps a blocking `embedded_hal::i2c::I2c` to implement `embedded_hal_async::i2c::I2c`.
///
/// The async methods just call the blocking ones and return immediately — no executor
/// overhead, no actual suspension.
struct BlockingI2c<I>(I);

impl<I: embedded_hal::i2c::I2c> embedded_hal::i2c::ErrorType for BlockingI2c<I> {
    type Error = I::Error;
}

impl<I: embedded_hal::i2c::I2c> embedded_hal_async::i2c::I2c for BlockingI2c<I> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(address, read)
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.0.write(address, write)
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.write_read(address, write, read)
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(address, operations)
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    // Use blocking I2C — no interrupts needed
    let i2c = i2c::I2c::new_blocking(p.I2C1, scl, sda, Config::default());

    // Wrap the blocking I2C bus so it satisfies the async trait
    let interface =
        I2cInterface::init_with_addresses(BlockingI2c(i2c), gyro::I2C_ADDR_0, xm::I2C_ADDR_0);

    let config = Lsm9ds0Config::new()
        .with_gyro_enabled(true)
        .with_gyro_data_rate(GyroDataRate::Hz95)
        .with_accel_data_rate(AccelDataRate::Hz100)
        .with_mag_mode(MagMode::ContinuousConversion)
        .with_temperature_enabled(true);

    let mut imu = Lsm9ds0::new_with_config(interface, config);

    // block_on runs the async future to completion synchronously. Since BlockingI2c never actually
    // suspends, this completes immediately.
    block_on(async {
        if let Err(e) = imu.init(&mut Delay).await {
            error!("Failed to initialize sensor: {:?}", e);
            return;
        }

        loop {
            let accel = match imu.read_accel().await {
                Ok((x, y, z)) => (x, y, z),
                Err(e) => {
                    warn!("Failed to read accelerometer: {:?}", e);
                    continue;
                }
            };
            let gyro = match imu.read_gyro().await {
                Ok((x, y, z)) => (x, y, z),
                Err(e) => {
                    warn!("Failed to read gyroscope: {:?}", e);
                    continue;
                }
            };
            let mag = match imu.read_mag().await {
                Ok((x, y, z)) => (x, y, z),
                Err(e) => {
                    warn!("Failed to read magnetometer: {:?}", e);
                    continue;
                }
            };
            let temperature_celsius = match imu.read_temp().await {
                Ok(temp) => temp,
                Err(e) => {
                    warn!("Failed to read temperature: {:?}", e);
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
    });
}
