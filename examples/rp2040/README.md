# RP2040 Examples

Example applications for the LSM9DS0 driver running on an RP2040 (specifically a Raspberry Pi Pico H) using Embassy.

## Examples

| Example | Description |
|---------|-------------|
| `simple-spi` | Basic SPI sensor reading at 10 Hz |
| `simple-i2c` | Basic I2C sensor reading at 10 Hz |
| `channels` | Multi-task architecture using Embassy channels |
| `kalman` | 1D Kalman filter applied to sensor data |
| `i2c_blocking_reg_dump` | Blocking I2C register dump (no async) |

## Building

Examples are built from within the `examples/rp2040` directory. The `.cargo/config.toml` sets the target to `thumbv6m-none-eabi` and configures `probe-rs` as the runner.

```sh
cd examples/rp2040

# Debug profile
cargo build --bin simple
# Release profile
cargo build --release-lto --bin simple
```

## Running

With a debug probe connected:

```sh
# Debug profile
cargo run --bin simple
# Release profile
cargo run --release-lto --bin simple
```

This flashes the binary and opens an RTT console for `defmt` log output.

## Wiring

### I2C

| RP2040 | LSM9DS0            |
|--------|--------------------|
| GPIO14 | SDA                |
| GPIO15 | SCL                |
| 3V3    | VCC                |
| GND    | GND, SA0_G, SA0_XM |

The SA0 pins select the I2C address. Active low. The examples use the default addresses (SA0 pins tied to GND).

### SPI

| RP2040 | LSM9DS0                                       |
|--------|-----------------------------------------------|
| GPIO18 | SCL (SPI Clock)                               |
| GPIO19 | SDA (MOSI)                                    |
| GPIO16 | SDO_G + SDO_XM (MISO, directly tied together) |
| GPIO17 | CS_G (Gyro Chip Select)                       |
| GPIO20 | CS_XM (Accel/Mag Chip Select)                 |
| 3V3    | VCC                                           |
| GND    | GND                                           |

The LSM9DS0 has separate chip select and data out lines for the gyroscope and accelerometer/magnetometer subsystems. The SDO pins can be tied together since only one device responds at a time.
