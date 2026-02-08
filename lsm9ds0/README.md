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

### Blocking I2C / SPI

This driver is built on `embedded-hal-async` traits, but it works with blocking buses too. The
async methods on the driver still require `.await`, but if the underlying bus is blocking, they
complete immediately with no executor overhead.

You need two things:

1. **An adapter** that wraps your blocking `embedded_hal::i2c::I2c` (or `SpiDevice`) to satisfy the
   async trait. The async fn just calls the blocking method and returns — no actual suspension:

```rust
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
    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.0.write_read(address, write, read)
    }
    async fn transaction(&mut self, address: u8, operations: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), Self::Error> {
        self.0.transaction(address, operations)
    }
}
```

2. **A trivial executor** like `embassy_futures::block_on` to run the async code synchronously:

```rust
use embassy_futures::block_on;

let interface = I2cInterface::init(BlockingI2c(my_blocking_i2c));
let mut imu = Lsm9ds0::new_with_config(interface, config);

block_on(async {
    imu.init(&mut delay).await?;
    let (gx, gy, gz) = imu.read_gyro().await?;
});
```

See the `examples/rp2040/src/bin/simple-i2c-blocking.rs` example for a complete working implementation.

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
