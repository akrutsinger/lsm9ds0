//! SPI communication interface for the LSM9DS0.
//!
//! This module provides [`SpiInterface`], which implements the [`Interface`] trait for SPI
//! communication with the LSM9DS0 sensor.
//!
//! # Hardware Setup
//!
//! The LSM9DS0 has two separate SPI sub-devices with independent chip-select pins:
//! - Gyroscope (CS_G)
//! - Accelerometer/Magnetometer (CS_XM)
//!
//! Each requires its own [`SpiDevice`] instance that manages its chip-select line.
//!
//! # Example
//!
//! ```no_run
//! # use lsm9ds0::{SpiInterface, Lsm9ds0};
//! # async fn example<S: embedded_hal_async::spi::SpiDevice>(g_spi: S, xm_spi: S) {
//! // g_spi and xm_spi are SpiDevice instances with separate chip-selects
//! let interface = SpiInterface::init(g_spi, xm_spi);
//! let mut imu = Lsm9ds0::new(interface);
//! # }
//! ```

use super::{Interface, MAX_WRITE_LEN};
use crate::errors::{GyroError, XmError};
use embedded_hal_async::spi::{Operation, SpiDevice};

/// R/W bit should be high for SPI Read operation
const SPI_READ: u8 = 0x80;
/// Magnetometer MS bit. When 0, does not increment the address; when 1, increments the address in
/// multiple reads. (Refer to datasheet, page 34, DocID024763 Rev 2)
const MS_BIT: u8 = 0x40;

/// SPI interface for communicating with the LSM9DS0.
///
/// The LSM9DS0 has two separate SPI sub-devices: the gyroscope and the accelerometer/magnetometer
/// (XM). Each has its own chip-select pin, so this interface requires two [`SpiDevice`] instances.
///
/// # SPI Configuration
///
/// The LSM9DS0 supports SPI mode 0 (CPOL=0, CPHA=0) and mode 3 (CPOL=1, CPHA=1), with a maximum
/// clock frequency of 10 MHz.
pub struct SpiInterface<GSpi, XmSpi>
where
    GSpi: SpiDevice,
    XmSpi: SpiDevice,
{
    g_device: GSpi,
    xm_device: XmSpi,
}

impl<GSpi, XmSpi> SpiInterface<GSpi, XmSpi>
where
    GSpi: SpiDevice,
    XmSpi: SpiDevice,
{
    /// Initializes an SPI interface with two `SpiDevice` instances.
    ///
    /// # Arguments
    /// * `g_device` - SPI device for Gyroscope
    /// * `xm_device` - SPI device for Accelerometer/Magnetometer
    pub fn init(g_device: GSpi, xm_device: XmSpi) -> Self {
        Self {
            g_device,
            xm_device,
        }
    }

    /// Release the SPI devices (consume self and return both SPI device instances)
    pub fn release(self) -> (GSpi, XmSpi) {
        (self.g_device, self.xm_device)
    }
}

/// Internal buffer size: register address (1 byte) + max data bytes
const WRITE_BUFFER_SIZE: usize = MAX_WRITE_LEN + 1;

/// Implementation of `Interface`
impl<GSpi, XmSpi, E> Interface for SpiInterface<GSpi, XmSpi>
where
    GSpi: SpiDevice<Error = E>,
    XmSpi: SpiDevice<Error = E>,
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
        // Set MS_BIT for auto-increment only on multi-byte writes
        let mut buffer = [0u8; WRITE_BUFFER_SIZE];
        let len = data.len().min(MAX_WRITE_LEN);
        buffer[0] = if len > 1 { addr | MS_BIT } else { addr };
        buffer[1..=len].copy_from_slice(&data[..len]);

        Ok(self.g_device.write(&buffer[..=len]).await?)
    }

    async fn write_xm(&mut self, addr: u8, data: &[u8]) -> Result<(), XmError<E>> {
        debug_assert!(
            data.len() <= MAX_WRITE_LEN,
            "write() data length {} exceeds MAX_WRITE_LEN ({})",
            data.len(),
            MAX_WRITE_LEN
        );

        // Build buffer with register address followed by data
        // Set MS_BIT for auto-increment only on multi-byte writes
        let mut buffer = [0u8; WRITE_BUFFER_SIZE];
        let len = data.len().min(MAX_WRITE_LEN);
        buffer[0] = if len > 1 { addr | MS_BIT } else { addr };
        buffer[1..=len].copy_from_slice(&data[..len]);

        Ok(self.xm_device.write(&buffer[..=len]).await?)
    }

    async fn read_gyro(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), GyroError<E>> {
        // Set MS_BIT for auto-increment only on multi-byte reads
        let reg_addr = if buffer.len() > 1 {
            SPI_READ | MS_BIT | addr
        } else {
            SPI_READ | addr
        };

        Ok(self
            .g_device
            .transaction(&mut [Operation::Write(&[reg_addr]), Operation::Read(buffer)])
            .await?)
    }

    async fn read_xm(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), XmError<E>> {
        // Set MS_BIT for auto-increment only on multi-byte reads
        let reg_addr = if buffer.len() > 1 {
            SPI_READ | MS_BIT | addr
        } else {
            SPI_READ | addr
        };

        Ok(self
            .xm_device
            .transaction(&mut [Operation::Write(&[reg_addr]), Operation::Read(buffer)])
            .await?)
    }
}
