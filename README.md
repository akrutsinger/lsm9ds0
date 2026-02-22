# lsm9ds0

Platform-agnostic async Rust driver for the ST LSM9DS0 IMU (accelerometer, gyroscope, magnetometer).

## Workspace Structure

| Crate | Description |
|-------|-------------|
| [`lsm9ds0`](lsm9ds0/) | Core driver library |
| [`examples/rp2040`](examples/rp2040/) | RP2040 examples using Embassy |

## Quick Start

```toml
[dependencies]
lsm9ds0 = "0.1"
```

```rust
use lsm9ds0::{Lsm9ds0, Lsm9ds0Config, I2cInterface, AccelDataRate, GyroDataRate, MagDataRate, MagMode};
use embedded_hal_async::delay::DelayNs;

let interface = I2cInterface::init(i2c);

let config = Lsm9ds0Config::new()
    .with_gyro_enabled(true)
    .with_gyro_data_rate(GyroDataRate::Hz95)
    .with_accel_data_rate(AccelDataRate::Hz100)
    .with_mag_mode(MagMode::ContinuousConversion)
    .with_mag_data_rate(MagDataRate::Hz50)
    .with_temperature_enabled(true);

let mut imu = Lsm9ds0::new_with_config(interface, config);
imu.init(&mut Delay).await?;

let (gx, gy, gz) = imu.read_gyro().await?;  // DegreesPerSecond
let (ax, ay, az) = imu.read_accel().await?; // GForce
let (mx, my, mz) = imu.read_mag().await?;   // Gauss
let temp_c = imu.read_temp().await?;        // Celsius
```

See the [driver README](lsm9ds0/README.md) for more details.

## Building

```bash
cargo build -p lsm9ds0
cargo test -p lsm9ds0
cargo doc -p lsm9ds0 --open
```

## Running Examples

The RP2040 examples use `probe-rs`. With a debug probe connected:

```bash
cd examples/rp2040
cargo run --release --bin simple-i2c
cargo run --release --bin simple-spi
```

## License

[MIT](LICENSE)
