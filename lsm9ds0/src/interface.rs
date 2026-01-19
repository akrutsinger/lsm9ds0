//! Interface trait
pub mod i2c;
pub mod spi;

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;

use crate::errors::{GyroError, XmError};

/// Maximum number of data bytes that can be written in a single `write()` call.
///
/// This limit exists because the I2C and SPI implementations use a fixed-size stack buffer to
/// prepend the register address to the data. The value of 15 is sufficient for all contiguous
/// register blocks on the LSM9DS0 (the largest being 8 bytes for the XM control registers).
pub const MAX_WRITE_LEN: usize = 15;

/// Interface Trait. `SpiInterface` and `I2cInterface` implement this.
///
/// The error types use wrapper structs (`GyroError<E>`, `XmError<E>`) to enable
/// automatic conversion to `Error<E>` via the `?` operator while preserving the
/// underlying bus error.
pub trait Interface {
    /// The underlying bus error type (e.g., I2C or SPI error)
    type BusError;

    /// Writes bytes to the Gyroscope starting at the specified register address.
    ///
    /// The device auto-increments the register address after each byte, allowing efficient bulk
    /// configuration of contiguous registers.
    ///
    /// # Arguments
    /// * `addr` - starting register address
    /// * `data` - bytes to write to consecutive registers (max [`MAX_WRITE_LEN`] bytes)
    ///
    /// # Panics
    /// In debug builds, panics if `data.len()` exceeds [`MAX_WRITE_LEN`] (15 bytes).
    /// In release builds, excess bytes are silently ignored.
    async fn write_gyro(&mut self, addr: u8, data: &[u8]) -> Result<(), GyroError<Self::BusError>>;

    /// Writes a single byte to the gyroscope's specified register address.
    ///
    /// This is a convenience wrapper around `write_gyro()` for single-byte writes.
    ///
    /// # Arguments
    /// * `addr` - register address
    /// * `value` - value to write
    async fn write_byte_gyro(
        &mut self,
        addr: u8,
        value: u8,
    ) -> Result<(), GyroError<Self::BusError>> {
        self.write_gyro(addr, &[value]).await
    }

    /// Writes bytes to the Accelerometer/Magnetometer at the specified register address.
    ///
    /// The device auto-increments the register address after each byte, allowing efficient bulk
    /// configuration of contiguous registers.
    ///
    /// # Arguments
    /// * `addr` - starting register address
    /// * `data` - bytes to write to consecutive registers (max [`MAX_WRITE_LEN`] bytes)
    ///
    /// # Panics
    /// In debug builds, panics if `data.len()` exceeds [`MAX_WRITE_LEN`] (15 bytes).
    /// In release builds, excess bytes are silently ignored.
    async fn write_xm(&mut self, addr: u8, data: &[u8]) -> Result<(), XmError<Self::BusError>>;

    /// Writes a single byte to the accelerometer/magnetometer's specified register address.
    ///
    /// This is a convenience wrapper around `write_xm()` for single-byte writes.
    ///
    /// # Arguments
    /// * `addr` - register address
    /// * `value` - value to write
    async fn write_byte_xm(&mut self, addr: u8, value: u8) -> Result<(), XmError<Self::BusError>> {
        self.write_xm(addr, &[value]).await
    }

    /// Reads multiple bytes from the gyroscope's specified register address.
    /// # Arguments
    /// * `addr` - register address
    /// * `buffer` - buffer to store read data
    async fn read_gyro(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
    ) -> Result<(), GyroError<Self::BusError>>;

    /// Reads multiple bytes from the accelerometer/magnetometer's specified register address.
    /// # Arguments
    /// * `addr` - register address
    /// * `buffer` - buffer to store read data
    async fn read_xm(&mut self, addr: u8, buffer: &mut [u8])
    -> Result<(), XmError<Self::BusError>>;
}
