//! Kalman filter example for LSM9DS0 sensor data
//!
//! Demonstrates applying a simple 1D Kalman filter to smooth sensor readings.
//!
//! Run with: cargo run --bin kalman

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

/// Simple 1D Kalman filter
///
/// Core equations:
/// - Predict:
///   p = p + q
/// - Update:
///   k = p / (p + r)
///   x = x + k * (measurement - x)
///   p = (1 - k) * p
#[derive(Copy, Clone)]
struct Kalman {
    x: f32, // state estimate
    p: f32, // estimation error covariance
    q: f32, // process noise (how much true value changes between measurements)
    r: f32, // measurement noise (sensor noise)
}

impl Kalman {
    const fn new(q: f32, r: f32) -> Self {
        Self {
            x: 0.0,
            p: 1.0,
            q,
            r,
        }
    }

    fn update(&mut self, measurement: f32) -> f32 {
        // Predict step: increase uncertainty
        self.p += self.q;

        // Update step: incorporate measurement
        let k = self.p / (self.p + self.r); // Kalman gain
        self.x += k * (measurement - self.x); // update estimate
        self.p *= 1.0 - k; // update covariance

        self.x
    }
}

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

    // Configure sensors
    let config = Lsm9ds0Config::new()
        .with_gyro_enabled(true)
        .with_gyro_data_rate(GyroDataRate::Hz95)
        .with_accel_data_rate(AccelDataRate::Hz100)
        .with_mag_mode(MagMode::ContinuousConversion)
        .with_temperature_enabled(true)
        .with_auto_calibration(lsm9ds0::Orientation::ZUp);

    let mut imu = Lsm9ds0::new_with_config(interface, config);

    if let Err(e) = imu.init(&mut Delay).await {
        error!("Failed to initialize sensor: {:?}", e);
        return;
    }

    // Initialize Kalman filters for each sensor axis
    // q = process noise, r = measurement noise
    // Lower q = smoother output, higher r = trust measurements less
    let mut accel_kf = [Kalman::new(0.01, 0.1); 3];
    let mut gyro_kf = [Kalman::new(0.001, 0.05); 3];
    let mut mag_kf = [Kalman::new(0.01, 0.1); 3];
    let mut temp_kf = Kalman::new(0.0001, 0.01);

    const IMU_SAMPLE_RATE_HZ: u64 = 10;
    let mut ticker = Ticker::every(Duration::from_hz(IMU_SAMPLE_RATE_HZ));

    loop {
        ticker.next().await;

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
        let temp = match imu.read_temp().await {
            Ok(t) => t,
            Err(e) => {
                warn!("Failed to read temperature: {:?}", e);
                continue;
            }
        };

        // Apply Kalman filtering to all readings
        let accel_filtered = (
            accel_kf[0].update(accel.0.into()),
            accel_kf[1].update(accel.1.into()),
            accel_kf[2].update(accel.2.into()),
        );
        let gyro_filtered = (
            gyro_kf[0].update(gyro.0.into()),
            gyro_kf[1].update(gyro.1.into()),
            gyro_kf[2].update(gyro.2.into()),
        );
        let mag_filtered = (
            mag_kf[0].update(mag.0.into()),
            mag_kf[1].update(mag.1.into()),
            mag_kf[2].update(mag.2.into()),
        );
        let temp_filtered = temp_kf.update(temp.into());

        info!(
            "Accel[{}, {}, {}]g Gyro[{}, {}, {}]dps Mag[{}, {}, {}]gauss Temp: {}°C",
            accel_filtered.0,
            accel_filtered.1,
            accel_filtered.2,
            gyro_filtered.0,
            gyro_filtered.1,
            gyro_filtered.2,
            mag_filtered.0,
            mag_filtered.1,
            mag_filtered.2,
            temp_filtered,
        );
    }
}
