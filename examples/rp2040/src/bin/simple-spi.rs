//! Async SPI example for LSM9DS0
//!
//! Run with: cargo run --bin simple-spi
//!
//! Hardware connections:
//! - SPI0 CLK:  GPIO18
//! - SPI0 MOSI: GPIO19 (to sensor SDA pin)
//! - SPI0 MISO: GPIO16 (from sensor SDO_G and SDO_XM pins, directly tied together)
//! - CS_G (Gyro Chip Select):       GPIO17
//! - CS_XM (Accel/Mag Chip Select): GPIO20

#![no_std]
#![no_main]

use lsm9ds0::{AccelDataRate, GyroDataRate, Lsm9ds0, Lsm9ds0Config, MagMode, SpiInterface};

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

type Spi0Bus = Mutex<NoopRawMutex, Spi<'static, SPI0, spi::Async>>;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // SPI pins
    let miso = p.PIN_16;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;

    // Chip select pins
    let cs_g = Output::new(p.PIN_17, Level::High); // Gyro CS
    let cs_xm = Output::new(p.PIN_20, Level::High); // Accel/Mag CS

    // Create SPI bus
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 1_000_000; // 1 MHz (LSM9DS0 supports up to 10 MHz)

    let spi = Spi::new(p.SPI0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, spi_config);

    // Create shared SPI bus (stored in static memory for sharing between devices)
    static SPI_BUS: StaticCell<Spi0Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // Create separate SPI devices for gyro and accel/mag (each manages its own CS)
    let g_spi = SpiDevice::new(spi_bus, cs_g);
    let xm_spi = SpiDevice::new(spi_bus, cs_xm);

    // Create LSM9DS0 driver with SPI interface
    let interface = SpiInterface::init(g_spi, xm_spi);

    // Configure sensors - defaults have everything in power-down mode
    let config = Lsm9ds0Config::new()
        .with_gyro_enabled(true)
        .with_gyro_data_rate(GyroDataRate::Hz95)
        .with_accel_data_rate(AccelDataRate::Hz100)
        .with_mag_mode(MagMode::ContinuousConversion)
        .with_temperature_enabled(true)
        .with_auto_calibration(lsm9ds0::Orientation::ZUp);

    let mut imu = Lsm9ds0::new_with_config(interface, config);

    use embassy_time::Delay;

    if let Err(e) = imu.init(&mut Delay).await {
        error!("Failed to initialize sensor: {:?}", Debug2Format(&e));
        return;
    }

    info!("LSM9DS0 initialized successfully via SPI");

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
