# LSM9DS0

A platform-agnostic Rust driver for the ST LSM9DS0 9-axis IMU (3D accelerometer, 3D gyroscope, 3D magnetometer).

Built on [`embedded-hal-async`](https://docs.rs/embedded-hal-async) traits for I2C and SPI communication.

## Features

- I2C and SPI interface support
- Configurable sensor ranges and data rates
- Temperature sensor
- FIFO support
- Interrupt configuration
- `no_std` compatible

## Usage

Add the dependency to your `Cargo.toml`:

```toml
[dependencies]
lsm9ds0 = "0.1"
```

### I2C

```rust
use lsm9ds0::{
    AccelDataRate, GyroDataRate, I2cInterface, Lsm9ds0, Lsm9ds0Config, MagMode,
};
use embassy_time::Delay;

let interface = I2cInterface::init(i2c);

let config = Lsm9ds0Config::new()
    .with_gyro_enabled(true)
    .with_gyro_data_rate(GyroDataRate::Hz95)
    .with_accel_data_rate(AccelDataRate::Hz100)
    .with_mag_mode(MagMode::ContinuousConversion)
    .with_temperature_enabled(true)
    .with_auto_calibration(lsm9ds0::Orientation::ZUp);

let mut imu = Lsm9ds0::new_with_config(interface, config);
imu.init(&mut Delay).await?;

let (gx, gy, gz) = imu.read_gyro().await?;   // DegreesPerSecond
let (ax, ay, az) = imu.read_accel().await?;  // GForce
let (mx, my, mz) = imu.read_mag().await?;    // Gauss
let temp = imu.read_temp().await?;           // Celsius
```

### SPI

The LSM9DS0 has separate chip selects for the gyroscope and accelerometer/magnetometer. Provide two `SpiDevice` instances:

```rust
use lsm9ds0::{Lsm9ds0, SpiInterface};
use embassy_time::Delay;

let interface = SpiInterface::init(gyro_spi, accel_mag_spi);
let mut imu = Lsm9ds0::new(interface);
imu.init(&mut Delay).await?;
```

## Configuration

Sensor parameters can be set at initialization or changed at runtime:

```rust
use lsm9ds0::{AccelScale, GyroDataRate, GyroScale, MagScale, Orientation};
use embassy_time::Delay;

// At initialization
let config = Lsm9ds0Config::new()
    .with_gyro_scale(GyroScale::Dps500)
    .with_accel_scale(AccelScale::G4)
    .with_mag_scale(MagScale::Gauss8);

// At runtime
imu.set_gyro_scale(GyroScale::Dps2000).await?;
imu.set_accel_data_rate(AccelDataRate::Hz400).await?;

// Calibrate the gyroscope and accelerometer during runtime
imu.calibrate_bias(&mut Delay, Orientation::ZUp).await?;
```

## Default Configuration

The driver's default configuration matches the device's power-on-reset register values from the datasheet, with two exceptions where the actual hardware defaults differ from the documented values:

| Register | Datasheet | Actual |
|----------|-----------|--------|
| CTRL_REG7_XM | 0x02 | 0x03 |
| INT_CTRL_REG_M | 0x00 | 0xE8 |

## Examples

See the `examples/` directory for platform-specific examples, including RP2040.

## References

- [LSM9DS0 Datasheet](https://www.st.com/resource/en/datasheet/lsm9ds0.pdf)

## License

[MIT](LICENSE)
