//! Error types for the LSM9DS0 driver

/// Errors that can occur when communicating with the LSM9DS0
#[derive(Debug)]
pub enum Error<E> {
    /// I2C/SPI bus error during gyroscope communication
    GyroBus(E),
    /// I2C/SPI bus error during accelerometer/magnetometer communication
    XmBus(E),
    /// Gyroscope WHO_AM_I returned unexpected value
    InvalidGyroId {
        /// Expected device ID (0xD4 for LSM9DS0)
        expected: u8,
        /// Actual value read from WHO_AM_I register
        actual: u8,
    },
    /// Accelerometer/magnetometer WHO_AM_I returned unexpected value
    InvalidXmId {
        /// Expected device ID (0x49 for LSM9DS0)
        expected: u8,
        /// Actual value read from WHO_AM_I register
        actual: u8,
    },
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::GyroBus(e) => write!(f, "gyroscope bus error: {:?}", e),
            Error::XmBus(e) => write!(f, "accelerometer/magnetometer bus error: {:?}", e),
            Error::InvalidGyroId { expected, actual } => write!(
                f,
                "invalid gyroscope ID: expected 0x{:02X}, got 0x{:02X}",
                expected, actual
            ),
            Error::InvalidXmId { expected, actual } => write!(
                f,
                "invalid XM ID: expected 0x{:02X}, got 0x{:02X}",
                expected, actual
            ),
        }
    }
}

/// Wrapper for gyroscope bus errors, enabling `?` operator in interface methods
#[derive(Debug)]
pub struct GyroError<E>(pub E);

/// Wrapper for accelerometer/magnetometer bus errors, enabling `?` operator in interface methods
#[derive(Debug)]
pub struct XmError<E>(pub E);

impl<E> From<E> for GyroError<E> {
    fn from(e: E) -> Self {
        GyroError(e)
    }
}

impl<E> From<E> for XmError<E> {
    fn from(e: E) -> Self {
        XmError(e)
    }
}

impl<E> From<GyroError<E>> for Error<E> {
    fn from(e: GyroError<E>) -> Self {
        Error::GyroBus(e.0)
    }
}

impl<E> From<XmError<E>> for Error<E> {
    fn from(e: XmError<E>) -> Self {
        Error::XmBus(e.0)
    }
}
