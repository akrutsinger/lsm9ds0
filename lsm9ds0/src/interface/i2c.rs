//! I2C communication interface for the LSM9DS0.
//!
//! This module provides [`I2cInterface`], which implements the [`Interface`] trait
//! for I2C communication with the LSM9DS0 sensor.
//!
//! # Example
//!
//! ```no_run
//! # use lsm9ds0::{I2cInterface, Lsm9ds0};
//! # async fn example<I: embedded_hal_async::i2c::I2c>(i2c: I) {
//! // Using default I2C addresses (SA0_G and SA0_XM connected to GND)
//! let interface = I2cInterface::init(i2c);
//! let mut imu = Lsm9ds0::new(interface);
//! # }
//! ```

use super::{Interface, MAX_WRITE_LEN};
use crate::errors::{GyroError, XmError};
use crate::registers::device_constants::{gyro, xm};
use embedded_hal_async::i2c::I2c;

/// Auto-increment bit for multi-byte I2C reads/writes.
/// When set, the register address automatically increments after each byte.
const AUTO_INCREMENT: u8 = 0x80;

/// I2C interface for communicating with the LSM9DS0.
///
/// The LSM9DS0 has two separate I2C sub-devices: the gyroscope and the accelerometer/magnetometer
/// (XM). This interface manages communication with both using their respective I2C addresses.
///
/// # I2C Addresses
///
/// | Device | SA0 = GND | SA0 = VCC |
/// |--------|-----------|-----------|
/// | Gyro   | 0x6A      | 0x6B      |
/// | XM     | 0x1E      | 0x1D      |
pub struct I2cInterface<I2C> {
    i2c: I2C,
    gyro_addr: u8,
    xm_addr: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Initializes an Interface with `I2C` instance using default addresses
    ///
    /// Default addresses assumes SA0_G and SA0_XM pins are connected to ground.
    pub fn init(i2c: I2C) -> Self {
        Self {
            i2c,
            gyro_addr: gyro::I2C_ADDR_0,
            xm_addr: xm::I2C_ADDR_0,
        }
    }

    /// Initializes an Interface with custom I2C addresses
    ///
    /// Use this when your board has different SA0 pin configurations:
    /// - Gyro: 0x6A (SA0_G=0; connected to GND) or 0x6B (SA0_G=1; connected to VCC)
    /// - XM: 0x1E (SA0_XM=0; connected to GND) or 0x1D (SA0_XM=1; connected to VCC)
    pub fn init_with_addresses(i2c: I2C, gyro_addr: u8, xm_addr: u8) -> Self {
        Self {
            i2c,
            gyro_addr,
            xm_addr,
        }
    }

    /// Release the I2C bus (consume self and return I2C instance)
    pub fn release(self) -> I2C {
        self.i2c
    }
}

/// Internal buffer size: register address (1 byte) + max data bytes
const WRITE_BUFFER_SIZE: usize = MAX_WRITE_LEN + 1;

/// Implementation of `Interface`
impl<I2C, E> Interface for I2cInterface<I2C>
where
    I2C: I2c<Error = E>,
{
    type BusError = E;

    async fn write_gyro(&mut self, addr: u8, data: &[u8]) -> Result<(), GyroError<E>> {
        debug_assert!(
            data.len() <= MAX_WRITE_LEN,
            "write() data length {} exceeds MAX_WRITE_LEN ({})",
            data.len(),
            MAX_WRITE_LEN
        );

        // Build buffer with register address followed by data
        // Set auto-increment bit for multi-byte writes
        let mut buffer = [0u8; WRITE_BUFFER_SIZE];
        buffer[0] = if data.len() > 1 {
            addr | AUTO_INCREMENT
        } else {
            addr
        };
        let len = data.len().min(MAX_WRITE_LEN);
        buffer[1..=len].copy_from_slice(&data[..len]);

        Ok(self.i2c.write(self.gyro_addr, &buffer[..=len]).await?)
    }

    async fn write_xm(&mut self, addr: u8, data: &[u8]) -> Result<(), XmError<E>> {
        debug_assert!(
            data.len() <= MAX_WRITE_LEN,
            "write() data length {} exceeds MAX_WRITE_LEN ({})",
            data.len(),
            MAX_WRITE_LEN
        );

        // Build buffer with register address followed by data
        // Set auto-increment bit for multi-byte writes
        let mut buffer = [0u8; WRITE_BUFFER_SIZE];
        buffer[0] = if data.len() > 1 {
            addr | AUTO_INCREMENT
        } else {
            addr
        };
        let len = data.len().min(MAX_WRITE_LEN);
        buffer[1..=len].copy_from_slice(&data[..len]);

        Ok(self.i2c.write(self.xm_addr, &buffer[..=len]).await?)
    }

    async fn read_gyro(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), GyroError<E>> {
        // Set auto-increment bit for multi-byte reads
        let reg_addr = if buffer.len() > 1 {
            addr | AUTO_INCREMENT
        } else {
            addr
        };

        Ok(self
            .i2c
            .write_read(self.gyro_addr, &[reg_addr], buffer)
            .await?)
    }

    async fn read_xm(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), XmError<E>> {
        // Set auto-increment bit for multi-byte reads
        let reg_addr = if buffer.len() > 1 {
            addr | AUTO_INCREMENT
        } else {
            addr
        };

        Ok(self
            .i2c
            .write_read(self.xm_addr, &[reg_addr], buffer)
            .await?)
    }
}
