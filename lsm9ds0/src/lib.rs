//! Platform-agnostic async driver for the LSM9DS0 3D accelerometer, 3D gyroscope, and 3D
//! magnetometer inertial module.
//!
//! This driver supports both I2C and SPI communication interfaces and is built on
//! [`embedded-hal-async`](https://docs.rs/embedded-hal-async) traits.
//!
//! Datasheet: [LSM9DS0](https://www.st.com/resource/en/datasheet/lsm9ds0.pdf)
//!
//! # Quick Start
//!
//! ```no_run
//! # use lsm9ds0::{Lsm9ds0, I2cInterface, Error};
//! # struct Delay;
//! # impl embedded_hal_async::delay::DelayNs for Delay {
//! #     async fn delay_ns(&mut self, _ns: u32) {}
//! # }
//! # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
//! # where I: embedded_hal_async::i2c::I2c
//! # {
//! use lsm9ds0::{AccelDataRate, GyroDataRate, I2cInterface, Lsm9ds0, Lsm9ds0Config, MagMode};
//! // use embassy_time::Delay;  // or your platform's delay implementation
//!
//! // Create interface and driver with default configuration
//! let interface = I2cInterface::init(i2c);
//!
//! // Turn on the sensors (since they are powered down by default)
//! let config = Lsm9ds0Config::new()
//!     .with_gyro_enabled(true)
//!     .with_gyro_data_rate(GyroDataRate::Hz95)
//!     .with_accel_data_rate(AccelDataRate::Hz100)
//!     .with_mag_mode(MagMode::ContinuousConversion)
//!     .with_temperature_enabled(true);
//!
//! let mut imu = Lsm9ds0::new_with_config(interface, config);
//!
//! // Initialize the sensor (verifies device IDs and applies config)
//! imu.init(&mut Delay).await?;
//!
//! // Read sensor data (all return typed wrappers, use .as_f32() or .into() for raw values)
//! let (gx, gy, gz) = imu.read_gyro().await?;  // DegreesPerSecond
//! let (ax, ay, az) = imu.read_accel().await?; // GForce
//! let (mx, my, mz) = imu.read_mag().await?;   // Gauss
//! let temp_c = imu.read_temp().await?;        // Celsius
//! # Ok(())
//! # }
//! ```
//!
//! # Custom Configuration
//!
//! Use the builder pattern to configure sensors before initialization:
//!
//! ```no_run
//! # use lsm9ds0::Error;
//! # struct Delay;
//! # impl embedded_hal_async::delay::DelayNs for Delay {
//! #     async fn delay_ns(&mut self, _ns: u32) {}
//! # }
//! # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
//! # where I: embedded_hal_async::i2c::I2c
//! # {
//! use lsm9ds0::{
//!     Lsm9ds0, Lsm9ds0Config, I2cInterface,
//!     GyroScale, GyroDataRate, AccelScale, AccelDataRate, MagScale, MagDataRate, MagMode,
//! };
//! // use embassy_time::Delay;  // or your platform's delay implementation
//!
//! let config = Lsm9ds0Config::new()
//!     // Gyroscope: ±500 dps at 190 Hz
//!     .with_gyro_enabled(true)
//!     .with_gyro_scale(GyroScale::Dps500)
//!     .with_gyro_data_rate(GyroDataRate::Hz190)
//!     // Accelerometer: ±4g at 100 Hz
//!     .with_accel_scale(AccelScale::G4)
//!     .with_accel_data_rate(AccelDataRate::Hz100)
//!     // Magnetometer: ±8 gauss at 50 Hz, continuous mode
//!     .with_mag_scale(MagScale::Gauss8)
//!     .with_mag_data_rate(MagDataRate::Hz50)
//!     .with_mag_mode(MagMode::ContinuousConversion)
//!     // Enable temperature sensor and block data update
//!     .with_temperature_enabled(true)
//!     .with_block_data_update(true)
//!     .with_auto_calibration(lsm9ds0::Orientation::ZUp);
//!
//! let interface = I2cInterface::init(i2c);
//! let mut imu = Lsm9ds0::new_with_config(interface, config);
//! imu.init(&mut Delay).await?;
//! # Ok(())
//! # }
//! ```
//!
//! # SPI Interface
//!
//! For SPI, provide separate [`SpiDevice`](embedded_hal_async::spi::SpiDevice) instances for the
//! gyroscope and accelerometer/magnetometer (each managing its own chip-select):
//!
//! ```no_run
//! # use lsm9ds0::Error;
//! # struct Delay;
//! # impl embedded_hal_async::delay::DelayNs for Delay {
//! #     async fn delay_ns(&mut self, _ns: u32) {}
//! # }
//! # async fn example<S>(g_spi: S, xm_spi: S) -> Result<(), Error<S::Error>>
//! # where
//! #     S: embedded_hal_async::spi::SpiDevice,
//! # {
//! use lsm9ds0::{Lsm9ds0, SpiInterface};
//! // use embassy_time::Delay;  // or your platform's delay implementation
//!
//! // g_spi and xm_spi are SpiDevice instances (e.g., from embassy or embedded-hal-bus)
//! let interface = SpiInterface::init(g_spi, xm_spi);
//! let mut imu = Lsm9ds0::new(interface);
//! imu.init(&mut Delay).await?;
//! # Ok(())
//! # }
//! ```
//!
//! # Runtime Configuration
//!
//! Settings can be changed after initialization:
//!
//! ```no_run
//! # use lsm9ds0::{Lsm9ds0, I2cInterface, Error};
//! # struct Delay;
//! # impl embedded_hal_async::delay::DelayNs for Delay {
//! #     async fn delay_ns(&mut self, _ns: u32) {}
//! # }
//! # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
//! # where I: embedded_hal_async::i2c::I2c
//! # {
//! # let interface = I2cInterface::init(i2c);
//! # let mut imu = Lsm9ds0::new(interface);
//! # imu.init(&mut Delay).await?;
//! use lsm9ds0::{GyroScale, AccelDataRate};
//!
//! // Change gyroscope scale dynamically
//! imu.set_gyro_scale(GyroScale::Dps2000).await?;
//!
//! // Change accelerometer data rate
//! imu.set_accel_data_rate(AccelDataRate::Hz400).await?;
//! # Ok(())
//! # }
//! ```
//!
//! # Interrupt Configuration
//!
//! Configure motion detection interrupts:
//!
//! ```
//! use lsm9ds0::{Lsm9ds0Config, GyroDataRate};
//!
//! // Simple gyroscope motion detection
//! let config = Lsm9ds0Config::new()
//!     .with_gyro_enabled(true)
//!     .with_gyro_data_rate(GyroDataRate::Hz190)
//!     .with_gyro_motion_threshold(500);  // Threshold in raw units
//!
//! // Or configure accelerometer motion detection
//! let config = Lsm9ds0Config::new()
//!     .with_accel_motion_threshold(32);  // 32 * 16mg = ~0.5g threshold
//! ```
#![allow(async_fn_in_trait)]
#![no_std]

pub mod config;
pub mod errors;
pub mod interface;
pub mod registers;
pub mod types;

pub use config::*;
pub use errors::Error;
pub use interface::*;
// Re-export user-facing register types. Internal control registers remain accessible via
// `lsm9ds0::registers::` for advanced users.
pub use registers::{
    // Enums used by builder methods and runtime configuration
    AccelBandwidth,
    AccelDataRate,
    AccelHpfMode,
    // Register address enums
    AccelMagRegisters,
    AccelScale,
    AccelSelfTest,
    ActiveLevel,
    ActiveLevelInverted,
    BlockDataUpdate,
    ClickSign,
    // Status and source bitfield structs (returned by public methods)
    ClickSrc,
    Enable,
    Endianness,
    FifoMode,
    FifoSrcReg,
    FifoSrcRegG,
    GyroBandwidth,
    GyroDataRate,
    GyroHpfCutoff,
    GyroHpfMode,
    GyroOutputSel,
    GyroPowerMode,
    GyroRegisters,
    GyroScale,
    GyroSelfTest,
    Int1SrcG,
    IntGenSrc,
    IntSrcRegM,
    InterruptCombination,
    LatchInterrupt,
    MagDataRate,
    MagLowPower,
    MagMode,
    MagResolution,
    MagScale,
    OutputType,
    PowerMode,
    SpiMode,
    StatusRegA,
    StatusRegG,
    StatusRegM,
    // Device constants
    device_constants,
};
pub use types::*;

use embedded_hal_async::delay::DelayNs;

/// LSM9DS0 9-axis IMU driver.
///
/// This struct provides methods for reading sensor data (gyroscope, accelerometer, magnetometer,
/// temperature) and configuring the device. It is generic over the communication interface,
/// supporting both I2C ([`I2cInterface`]) and SPI ([`SpiInterface`]).
///
/// # Initialization
///
/// After creating a driver instance with [`Lsm9ds0::new`] or [`Lsm9ds0::new_with_config`], call
/// [`Lsm9ds0::init`] to verify device IDs and apply the configuration to hardware.
///
/// # Sensor Units
///
/// - Gyroscope: milli-degrees per second (mdps)
/// - Accelerometer: milli-g (mg)
/// - Magnetometer: milli-gauss (mgauss)
/// - Temperature: degrees Celsius (°C)
///
/// # Thread Safety
///
/// `Lsm9ds0` is [`Send`] when the underlying interface is `Send`, which makes it safe to move
/// between Embassy tasks or other async executors.
pub struct Lsm9ds0<I>
where
    I: Interface,
{
    /// Communication interface (I2C or SPI)
    interface: I,
    config: Lsm9ds0Config,
}

impl<I> Lsm9ds0<I>
where
    I: Interface,
{
    /// Create a new LSM9DS0 driver instance
    pub fn new(interface: I) -> Self {
        Self {
            interface,
            config: Lsm9ds0Config::default(),
        }
    }

    /// Create a new LSM9DS0 driver with custom configuration
    pub fn new_with_config(interface: I, config: Lsm9ds0Config) -> Self {
        Self { interface, config }
    }

    /// Initialize the sensor current configuration.
    ///
    /// This verifies communication with both the gyroscope and accelerometer/magnetometer chips by
    /// reading their WHO_AM_I registers, then applies the current configuration.
    ///
    /// **Important:** The device must be stationary during initialization if auto-calibration is
    /// enabled. The calibration process takes approximately 1 second.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Communication with the gyroscope fails ([`Error::GyroBus`])
    /// - Communication with the accelerometer/magnetometer fails ([`Error::XmBus`])
    /// - Device ID verification fails ([`Error::InvalidGyroId`] or [`Error::InvalidXmId`])
    ///
    /// # Example
    ///
    /// ```ignore
    /// use embassy_time::Delay;
    /// use lsm9ds0::{Lsm9ds0, Lsm9ds0Config, I2cInterface, Orientation};
    ///
    /// let config = Lsm9ds0Config::new()
    ///     .with_gyro_enabled(true)
    ///     .with_accel_data_rate(AccelDataRate::Hz100)
    ///     .with_auto_calibration(Orientation::ZUp);
    ///
    /// let mut imu = Lsm9ds0::new_with_config(interface, config);
    /// imu.init(&mut Delay).await?;  // Calibrates automatically
    /// ```
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::BusError>> {
        // Verify we can talk to both chips
        self.verify_device_ids().await?;

        self.apply_configs().await?;

        // Perform auto-calibration if configured
        if let Some(orientation) = self.config.auto_calibration {
            self.calibrate_bias(delay, orientation).await?;
        }

        Ok(())
    }

    /// Begin operation (alias for init; for folks familiar with Arduino-style APIs).
    ///
    /// # Errors
    ///
    /// See [`Lsm9ds0::init`] for error conditions.
    pub async fn begin<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::BusError>> {
        self.init(delay).await
    }

    /// Software reset.
    ///
    /// Triggers a reboot of both the gyroscope and accelerometer/magnetometer chips,
    /// then resets the driver's internal configuration to defaults.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails, or
    /// [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    ///
    /// # Example
    /// ```ignore
    /// # use lsm9ds0::Error;
    /// # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
    /// # where I: embedded_hal_async::i2c::I2c
    /// # {
    /// use embassy_time::Delay;
    /// use lsm9ds0::{I2cInterface, Lsm9ds0};
    ///
    /// let interface = I2cInterface::init(i2c);
    /// let imu = Lsm9ds0::new(interface);
    ///
    /// imu.software_reset(&mut Delay).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn software_reset<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg5_g.set_boot(Enable::Enabled);
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG5_G.addr(),
                self.config.ctrl_reg5_g.into(),
            )
            .await?;

        self.config.ctrl_reg0_xm.set_boot(Enable::Enabled);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG0_XM.addr(),
                self.config.ctrl_reg0_xm.into(),
            )
            .await?;

        // Give the sense some time for the reboot to complete. I couldn't actually find any
        // definitive timing in the datasheet, but 50ms seems respectable.
        delay.delay_ms(50).await;

        // Hardware resets all register to power on defaults. Reset shadow register to match
        self.config = Lsm9ds0Config::default();

        self.verify_device_ids().await?;

        Ok(())
    }

    /// Update configuration and apply to device.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails, or
    /// [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_config(&mut self, config: Lsm9ds0Config) -> Result<(), Error<I::BusError>> {
        self.config = config;
        self.apply_configs().await
    }

    /// Get current configuration
    pub fn config(&self) -> &Lsm9ds0Config {
        &self.config
    }

    /// Re-apply the current shadow register configuration to hardware.
    ///
    /// Use this to re-synchronize the device after a bus error, brown-out, or any event
    /// that may have reset the sensor without the driver's knowledge. Unlike [`init`](Self::init),
    /// this skips WHO_AM_I verification and calibration, making it faster for error recovery.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails, or
    /// [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use lsm9ds0::{Lsm9ds0, I2cInterface, Error};
    /// # struct Delay;
    /// # impl embedded_hal_async::delay::DelayNs for Delay {
    /// #     async fn delay_ns(&mut self, _ns: u32) {}
    /// # }
    /// # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
    /// # where I: embedded_hal_async::i2c::I2c
    /// # {
    /// let interface = I2cInterface::init(i2c);
    /// let mut imu = Lsm9ds0::new(interface);
    /// imu.init(&mut Delay).await?;
    ///
    /// // After a bus error or suspected sensor reset, re-sync config
    /// if imu.read_gyro().await.is_err() {
    ///     imu.reapply_config().await?;
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub async fn reapply_config(&mut self) -> Result<(), Error<I::BusError>> {
        self.apply_configs().await
    }

    /// Verify that hardware registers match the driver's shadow register state.
    ///
    /// Reads back all writable configuration registers from both the gyroscope and
    /// accelerometer/magnetometer and compares them against the expected shadow values.
    ///
    /// This is useful as a periodic health check to detect silent register corruption from EMI,
    /// brown-outs, or other events that may reset the sensor without the driver's knowledge.
    ///
    /// # Errors
    ///
    /// Returns [`Error::ConfigMismatch`] if any register does not match the expected value.
    /// Returns [`Error::GyroBus`] or [`Error::XmBus`] if a bus error occurs during readback.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use lsm9ds0::{Lsm9ds0, I2cInterface, Error};
    /// # struct Delay;
    /// # impl embedded_hal_async::delay::DelayNs for Delay {
    /// #     async fn delay_ns(&mut self, _ns: u32) {}
    /// # }
    /// # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
    /// # where I: embedded_hal_async::i2c::I2c
    /// # {
    /// let interface = I2cInterface::init(i2c);
    /// let mut imu = Lsm9ds0::new(interface);
    /// imu.init(&mut Delay).await?;
    ///
    /// // Periodic health check
    /// if imu.verify_config().await.is_err() {
    ///     // Registers corrupted — re-apply config
    ///     imu.reapply_config().await?;
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub async fn verify_config(&mut self) -> Result<(), Error<I::BusError>> {
        // Helper closure to compare a slice of expected bytes against hardware
        // starting at base_addr. Returns the first mismatch found.
        //
        // We can't use an actual closure because async closures capturing &mut self
        // don't work, so we inline the reads and use a shared comparison function.
        fn find_mismatch(base_addr: u8, expected: &[u8], actual: &[u8]) -> Option<(u8, u8, u8)> {
            for (i, (&exp, &act)) in expected.iter().zip(actual.iter()).enumerate() {
                if exp != act {
                    return Some((base_addr + i as u8, exp, act));
                }
            }
            None
        }

        // === Gyroscope registers ===

        // CTRL_REG1_G through CTRL_REG5_G (0x20-0x24)
        let expected_g_ctrl: [u8; 5] = [
            self.config.ctrl_reg1_g.into(),
            self.config.ctrl_reg2_g.into(),
            self.config.ctrl_reg3_g.into(),
            self.config.ctrl_reg4_g.into(),
            self.config.ctrl_reg5_g.into(),
        ];
        let mut actual_g_ctrl = [0u8; 5];
        self.interface
            .read_gyro(GyroRegisters::CTRL_REG1_G.addr(), &mut actual_g_ctrl)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            GyroRegisters::CTRL_REG1_G.addr(),
            &expected_g_ctrl,
            &actual_g_ctrl,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // FIFO_CTRL_REG_G (0x2E)
        let mut actual = [0u8; 1];
        self.interface
            .read_gyro(GyroRegisters::FIFO_CTRL_REG_G.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.fifo_ctrl_reg_g.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: GyroRegisters::FIFO_CTRL_REG_G.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // INT1_CFG_G (0x30)
        self.interface
            .read_gyro(GyroRegisters::INT1_CFG_G.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.int1_cfg_g.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: GyroRegisters::INT1_CFG_G.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // INT1_THS_XH_G through INT1_DURATION_G (0x32-0x38)
        let expected_g_int: [u8; 7] = [
            ((self.config.gyro_int_ths_x >> 8) & 0x7F) as u8,
            (self.config.gyro_int_ths_x & 0xFF) as u8,
            ((self.config.gyro_int_ths_y >> 8) & 0x7F) as u8,
            (self.config.gyro_int_ths_y & 0xFF) as u8,
            ((self.config.gyro_int_ths_z >> 8) & 0x7F) as u8,
            (self.config.gyro_int_ths_z & 0xFF) as u8,
            self.config.int1_duration_g.into(),
        ];
        let mut actual_g_int = [0u8; 7];
        self.interface
            .read_gyro(GyroRegisters::INT1_THS_XH_G.addr(), &mut actual_g_int)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            GyroRegisters::INT1_THS_XH_G.addr(),
            &expected_g_int,
            &actual_g_int,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // === Accelerometer/Magnetometer registers ===

        // INT_CTRL_REG_M (0x12)
        self.interface
            .read_xm(AccelMagRegisters::INT_CTRL_REG_M.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.int_ctrl_reg_m.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: AccelMagRegisters::INT_CTRL_REG_M.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // INT_THS_L_M, INT_THS_H_M (0x14-0x15)
        let expected_mag_ths: [u8; 2] = [
            (self.config.mag_int_ths & 0xFF) as u8,
            ((self.config.mag_int_ths >> 8) & 0x7F) as u8,
        ];
        let mut actual_mag_ths = [0u8; 2];
        self.interface
            .read_xm(AccelMagRegisters::INT_THS_L_M.addr(), &mut actual_mag_ths)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::INT_THS_L_M.addr(),
            &expected_mag_ths,
            &actual_mag_ths,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // OFFSET_X_L_M through OFFSET_Z_H_M (0x16-0x1B)
        let expected_mag_off: [u8; 6] = [
            (self.config.mag_offset_x & 0xFF) as u8,
            ((self.config.mag_offset_x >> 8) & 0xFF) as u8,
            (self.config.mag_offset_y & 0xFF) as u8,
            ((self.config.mag_offset_y >> 8) & 0xFF) as u8,
            (self.config.mag_offset_z & 0xFF) as u8,
            ((self.config.mag_offset_z >> 8) & 0xFF) as u8,
        ];
        let mut actual_mag_off = [0u8; 6];
        self.interface
            .read_xm(AccelMagRegisters::OFFSET_X_L_M.addr(), &mut actual_mag_off)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::OFFSET_X_L_M.addr(),
            &expected_mag_off,
            &actual_mag_off,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // CTRL_REG0_XM through CTRL_REG7_XM (0x1F-0x26)
        let expected_xm_ctrl: [u8; 8] = [
            self.config.ctrl_reg0_xm.into(),
            self.config.ctrl_reg1_xm.into(),
            self.config.ctrl_reg2_xm.into(),
            self.config.ctrl_reg3_xm.into(),
            self.config.ctrl_reg4_xm.into(),
            self.config.ctrl_reg5_xm.into(),
            self.config.ctrl_reg6_xm.into(),
            self.config.ctrl_reg7_xm.into(),
        ];
        let mut actual_xm_ctrl = [0u8; 8];
        self.interface
            .read_xm(AccelMagRegisters::CTRL_REG0_XM.addr(), &mut actual_xm_ctrl)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::CTRL_REG0_XM.addr(),
            &expected_xm_ctrl,
            &actual_xm_ctrl,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // FIFO_CTRL_REG (0x2E)
        self.interface
            .read_xm(AccelMagRegisters::FIFO_CTRL_REG.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.fifo_ctrl_reg.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: AccelMagRegisters::FIFO_CTRL_REG.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // INT_GEN_1_REG (0x30)
        self.interface
            .read_xm(AccelMagRegisters::INT_GEN_1_REG.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.int_gen_1_reg.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: AccelMagRegisters::INT_GEN_1_REG.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // INT_GEN_1_THS, INT_GEN_1_DURATION (0x32-0x33)
        let expected_gen1: [u8; 2] = [
            self.config.int_gen_1_ths & 0x7F,
            self.config.int_gen_1_duration & 0x7F,
        ];
        let mut actual_gen1 = [0u8; 2];
        self.interface
            .read_xm(AccelMagRegisters::INT_GEN_1_THS.addr(), &mut actual_gen1)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::INT_GEN_1_THS.addr(),
            &expected_gen1,
            &actual_gen1,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // INT_GEN_2_REG (0x34)
        self.interface
            .read_xm(AccelMagRegisters::INT_GEN_2_REG.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.int_gen_2_reg.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: AccelMagRegisters::INT_GEN_2_REG.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // INT_GEN_2_THS, INT_GEN_2_DURATION (0x36-0x37)
        let expected_gen2: [u8; 2] = [
            self.config.int_gen_2_ths & 0x7F,
            self.config.int_gen_2_duration & 0x7F,
        ];
        let mut actual_gen2 = [0u8; 2];
        self.interface
            .read_xm(AccelMagRegisters::INT_GEN_2_THS.addr(), &mut actual_gen2)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::INT_GEN_2_THS.addr(),
            &expected_gen2,
            &actual_gen2,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // CLICK_CFG (0x38)
        self.interface
            .read_xm(AccelMagRegisters::CLICK_CFG.addr(), &mut actual)
            .await?;
        let expected_val: u8 = self.config.click_cfg.into();
        if actual[0] != expected_val {
            return Err(Error::ConfigMismatch {
                register: AccelMagRegisters::CLICK_CFG.addr(),
                expected: expected_val,
                actual: actual[0],
            });
        }

        // CLICK_THS through TIME_WINDOW (0x3A-0x3D)
        let expected_click: [u8; 4] = [
            self.config.click_ths & 0x7F,
            self.config.time_limit_ms,
            self.config.time_latency_ms,
            self.config.time_window_ms,
        ];
        let mut actual_click = [0u8; 4];
        self.interface
            .read_xm(AccelMagRegisters::CLICK_THS.addr(), &mut actual_click)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::CLICK_THS.addr(),
            &expected_click,
            &actual_click,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        // ACT_THS, ACT_DUR (0x3E-0x3F)
        let expected_act: [u8; 2] = [self.config.act_ths & 0x7F, self.config.act_dur];
        let mut actual_act = [0u8; 2];
        self.interface
            .read_xm(AccelMagRegisters::ACT_THS.addr(), &mut actual_act)
            .await?;
        if let Some((reg, exp, act)) = find_mismatch(
            AccelMagRegisters::ACT_THS.addr(),
            &expected_act,
            &actual_act,
        ) {
            return Err(Error::ConfigMismatch {
                register: reg,
                expected: exp,
                actual: act,
            });
        }

        Ok(())
    }

    /// Verify device IDs match expected values
    async fn verify_device_ids(&mut self) -> Result<(), Error<I::BusError>> {
        let mut id = [0u8];

        // Check gyroscope WHO_AM_I register (0x0F)
        self.interface
            .read_gyro(GyroRegisters::WHO_AM_I_G.addr(), &mut id)
            .await?;

        if id[0] != registers::device_constants::gyro::DEVICE_ID {
            return Err(Error::InvalidGyroId {
                expected: registers::device_constants::gyro::DEVICE_ID,
                actual: id[0],
            });
        }

        // Check accelerometer/magnetometer WHO_AM_I register (0x0F)
        self.interface
            .read_xm(AccelMagRegisters::WHO_AM_I_XM.addr(), &mut id)
            .await?;

        if id[0] != registers::device_constants::xm::DEVICE_ID {
            return Err(Error::InvalidXmId {
                expected: registers::device_constants::xm::DEVICE_ID,
                actual: id[0],
            });
        }

        Ok(())
    }

    /// Apply configuration to the device
    ///
    /// Writes all shadow register values to the hardware using multi-byte writes where registers
    /// are contiguous. Since we maintain local copies of all register values, no read-modify-write
    /// is needed.
    async fn apply_configs(&mut self) -> Result<(), Error<I::BusError>> {
        // === Configure gyroscope ===

        // CTRL_REG1_G through CTRL_REG5_G (0x20-0x24)
        self.interface
            .write_gyro(
                GyroRegisters::CTRL_REG1_G.addr(),
                &[
                    self.config.ctrl_reg1_g.into(),
                    self.config.ctrl_reg2_g.into(),
                    self.config.ctrl_reg3_g.into(),
                    self.config.ctrl_reg4_g.into(),
                    self.config.ctrl_reg5_g.into(),
                ],
            )
            .await?;

        // FIFO_CTRL_REG_G (0x2E) - standalone (0x2F is read-only FIFO_SRC_REG_G)
        self.interface
            .write_byte_gyro(
                GyroRegisters::FIFO_CTRL_REG_G.addr(),
                self.config.fifo_ctrl_reg_g.into(),
            )
            .await?;

        // INT1_CFG_G (0x30) - standalone (0x31 is read-only INT1_SRC_G)
        self.interface
            .write_byte_gyro(
                GyroRegisters::INT1_CFG_G.addr(),
                self.config.int1_cfg_g.into(),
            )
            .await?;

        // INT1_THS_XH_G through INT1_DURATION_G (0x32-0x38)
        self.interface
            .write_gyro(
                GyroRegisters::INT1_THS_XH_G.addr(),
                &[
                    ((self.config.gyro_int_ths_x >> 8) & 0x7F) as u8,
                    (self.config.gyro_int_ths_x & 0xFF) as u8,
                    ((self.config.gyro_int_ths_y >> 8) & 0x7F) as u8,
                    (self.config.gyro_int_ths_y & 0xFF) as u8,
                    ((self.config.gyro_int_ths_z >> 8) & 0x7F) as u8,
                    (self.config.gyro_int_ths_z & 0xFF) as u8,
                    self.config.int1_duration_g.into(),
                ],
            )
            .await?;

        // === Configure accelerometer/magnetometer ===

        // INT_CTRL_REG_M (0x12) - standalone (0x13 is read-only INT_SRC_REG_M)
        self.interface
            .write_byte_xm(
                AccelMagRegisters::INT_CTRL_REG_M.addr(),
                self.config.int_ctrl_reg_m.into(),
            )
            .await?;

        // INT_THS_L_M, INT_THS_H_M (0x14-0x15)
        self.interface
            .write_xm(
                AccelMagRegisters::INT_THS_L_M.addr(),
                &[
                    (self.config.mag_int_ths & 0xFF) as u8,
                    ((self.config.mag_int_ths >> 8) & 0x7F) as u8,
                ],
            )
            .await?;

        // OFFSET_X_L_M through OFFSET_Z_H_M (0x16-0x1B)
        self.interface
            .write_xm(
                AccelMagRegisters::OFFSET_X_L_M.addr(),
                &[
                    (self.config.mag_offset_x & 0xFF) as u8,
                    ((self.config.mag_offset_x >> 8) & 0xFF) as u8,
                    (self.config.mag_offset_y & 0xFF) as u8,
                    ((self.config.mag_offset_y >> 8) & 0xFF) as u8,
                    (self.config.mag_offset_z & 0xFF) as u8,
                    ((self.config.mag_offset_z >> 8) & 0xFF) as u8,
                ],
            )
            .await?;

        // CTRL_REG0_XM through CTRL_REG7_XM (0x1F-0x26)
        self.interface
            .write_xm(
                AccelMagRegisters::CTRL_REG0_XM.addr(),
                &[
                    self.config.ctrl_reg0_xm.into(),
                    self.config.ctrl_reg1_xm.into(),
                    self.config.ctrl_reg2_xm.into(),
                    self.config.ctrl_reg3_xm.into(),
                    self.config.ctrl_reg4_xm.into(),
                    self.config.ctrl_reg5_xm.into(),
                    self.config.ctrl_reg6_xm.into(),
                    self.config.ctrl_reg7_xm.into(),
                ],
            )
            .await?;

        // FIFO_CTRL_REG (0x2E) - standalone (0x2F is read-only FIFO_SRC_REG)
        self.interface
            .write_byte_xm(
                AccelMagRegisters::FIFO_CTRL_REG.addr(),
                self.config.fifo_ctrl_reg.into(),
            )
            .await?;

        // INT_GEN_1_REG (0x30) - standalone (0x31 is read-only INT_GEN_1_SRC)
        self.interface
            .write_byte_xm(
                AccelMagRegisters::INT_GEN_1_REG.addr(),
                self.config.int_gen_1_reg.into(),
            )
            .await?;

        // INT_GEN_1_THS, INT_GEN_1_DURATION (0x32-0x33)
        self.interface
            .write_xm(
                AccelMagRegisters::INT_GEN_1_THS.addr(),
                &[
                    self.config.int_gen_1_ths & 0x7F,
                    self.config.int_gen_1_duration & 0x7F,
                ],
            )
            .await?;

        // INT_GEN_2_REG (0x34) - standalone (0x35 is read-only INT_GEN_2_SRC)
        self.interface
            .write_byte_xm(
                AccelMagRegisters::INT_GEN_2_REG.addr(),
                self.config.int_gen_2_reg.into(),
            )
            .await?;

        // INT_GEN_2_THS, INT_GEN_2_DURATION (0x36-0x37)
        self.interface
            .write_xm(
                AccelMagRegisters::INT_GEN_2_THS.addr(),
                &[
                    self.config.int_gen_2_ths & 0x7F,
                    self.config.int_gen_2_duration & 0x7F,
                ],
            )
            .await?;

        // CLICK_CFG (0x38) - standalone (0x39 is read-only CLICK_SRC)
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CLICK_CFG.addr(),
                self.config.click_cfg.into(),
            )
            .await?;

        // CLICK_THS through TIME_WINDOW (0x3A-0x3D)
        self.interface
            .write_xm(
                AccelMagRegisters::CLICK_THS.addr(),
                &[
                    self.config.click_ths & 0x7F,
                    self.config.time_limit_ms,
                    self.config.time_latency_ms,
                    self.config.time_window_ms,
                ],
            )
            .await?;

        // ACT_THS, ACT_DUR (0x3E-0x3F)
        self.interface
            .write_xm(
                AccelMagRegisters::ACT_THS.addr(),
                &[self.config.act_ths & 0x7F, self.config.act_dur],
            )
            .await?;

        Ok(())
    }

    /// Read gyroscope FIFO level (number of unread samples, 0-31).
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro_fifo_level(&mut self) -> Result<u8, Error<I::BusError>> {
        let status = self.read_gyro_fifo_status().await?;
        Ok(status.fss())
    }

    /// Read a single sample from the gyroscope FIFO.
    ///
    /// Returns raw (x, y, z) values. Call `read_gyro_fifo_level()` first to check
    /// how many samples are available. Reading when FIFO is empty returns stale data.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro_fifo(&mut self) -> Result<(i16, i16, i16), Error<I::BusError>> {
        // Reading from output registers pops from FIFO when FIFO is enabled
        self.read_gyro_raw().await
    }

    /// Read raw gyroscope data.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro_raw(&mut self) -> Result<(i16, i16, i16), Error<I::BusError>> {
        let mut bytes = [0u8; 6];
        self.interface
            .read_gyro(GyroRegisters::OUT_X_L_G.addr(), &mut bytes)
            .await?;
        let x = i16::from_le_bytes([bytes[0], bytes[1]]);
        let y = i16::from_le_bytes([bytes[2], bytes[3]]);
        let z = i16::from_le_bytes([bytes[4], bytes[5]]);
        Ok((x, y, z))
    }

    /// Read gyroscope measurements (x, y, z) in degrees per second.
    ///
    /// Returns calibrated values based on the configured scale.
    /// Use `.as_f32()` or `.into()` to get the raw f32 value.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro(
        &mut self,
    ) -> Result<
        (
            types::DegreesPerSecond,
            types::DegreesPerSecond,
            types::DegreesPerSecond,
        ),
        Error<I::BusError>,
    > {
        let (x, y, z) = self.read_gyro_raw().await?;
        // Sensitivity is in mdps/LSB, divide by 1000 to get dps
        let sensitivity = self.config.gyro_sensitivity() / 1000.0;
        let (bias_x, bias_y, bias_z) = self.config.gyro_bias;
        Ok((
            types::DegreesPerSecond::new(x as f32 * sensitivity - bias_x),
            types::DegreesPerSecond::new(y as f32 * sensitivity - bias_y),
            types::DegreesPerSecond::new(z as f32 * sensitivity - bias_z),
        ))
    }

    /// Read accelerometer FIFO level (number of unread samples, 0-31).
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_fifo_level(&mut self) -> Result<u8, Error<I::BusError>> {
        let status = self.read_accel_fifo_status().await?;
        Ok(status.fss())
    }

    /// Read a single sample from the accelerometer FIFO.
    ///
    /// Returns raw (x, y, z) values. Call `read_accel_fifo_level()` first to check
    /// how many samples are available. Reading when FIFO is empty returns stale data.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_fifo(&mut self) -> Result<(i16, i16, i16), Error<I::BusError>> {
        // Reading from output registers pops from FIFO when FIFO is enabled
        self.read_accel_raw().await
    }

    /// Read raw accelerometer data.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_raw(&mut self) -> Result<(i16, i16, i16), Error<I::BusError>> {
        let mut bytes = [0u8; 6];
        self.interface
            .read_xm(AccelMagRegisters::OUT_X_L_A.addr(), &mut bytes)
            .await?;
        let x = i16::from_le_bytes([bytes[0], bytes[1]]);
        let y = i16::from_le_bytes([bytes[2], bytes[3]]);
        let z = i16::from_le_bytes([bytes[4], bytes[5]]);
        Ok((x, y, z))
    }

    /// Read accelerometer measurements (x, y, z) in g-force.
    ///
    /// Returns calibrated values based on the configured scale.
    /// Use `.as_f32()` or `.into()` to get the raw f32 value.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel(
        &mut self,
    ) -> Result<(types::GForce, types::GForce, types::GForce), Error<I::BusError>> {
        let (x, y, z) = self.read_accel_raw().await?;
        // Sensitivity is in mg/LSB, divide by 1000 to get g
        let sensitivity = self.config.accel_sensitivity() / 1000.0;
        let (bias_x, bias_y, bias_z) = self.config.accel_bias;
        Ok((
            types::GForce::new(x as f32 * sensitivity - bias_x),
            types::GForce::new(y as f32 * sensitivity - bias_y),
            types::GForce::new(z as f32 * sensitivity - bias_z),
        ))
    }

    /// Read raw magnetometer data.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_mag_raw(&mut self) -> Result<(i16, i16, i16), Error<I::BusError>> {
        let mut bytes = [0u8; 6];
        self.interface
            .read_xm(AccelMagRegisters::OUT_X_L_M.addr(), &mut bytes)
            .await?;
        let x = i16::from_le_bytes([bytes[0], bytes[1]]);
        let y = i16::from_le_bytes([bytes[2], bytes[3]]);
        let z = i16::from_le_bytes([bytes[4], bytes[5]]);
        Ok((x, y, z))
    }

    /// Read magnetometer measurements (x, y, z) in gauss.
    ///
    /// Returns calibrated values based on the configured scale.
    /// Use `.as_f32()` or `.into()` to get the raw f32 value.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_mag(
        &mut self,
    ) -> Result<(types::Gauss, types::Gauss, types::Gauss), Error<I::BusError>> {
        let (x, y, z) = self.read_mag_raw().await?;
        // Sensitivity is in mgauss/LSB, divide by 1000 to get gauss
        let sensitivity = self.config.mag_sensitivity() / 1000.0;
        Ok((
            types::Gauss::new(x as f32 * sensitivity),
            types::Gauss::new(y as f32 * sensitivity),
            types::Gauss::new(z as f32 * sensitivity),
        ))
    }

    /// Read temperature in Celsius.
    ///
    /// The temperature value is calculated using the configured temperature offset.
    /// See [`Lsm9ds0Config::with_temperature_offset`] for calibration details.
    ///
    /// Use `.as_f32()` or `.into()` to get the raw f32 value.
    /// Use `.to_fahrenheit()` or `.to_kelvin()` for unit conversion.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_temp(&mut self) -> Result<types::Celsius, Error<I::BusError>> {
        let mut bytes = [0u8; 2];
        self.interface
            .read_xm(AccelMagRegisters::OUT_TEMP_L_XM.addr(), &mut bytes)
            .await?;
        let raw = i16::from_le_bytes([bytes[0], bytes[1]]);
        // Shift extends the two's compliment sign bit of the 12-bit temp value
        let result: i16 = (raw << 4) >> 4;
        Ok(types::Celsius::new(
            (result as f32) / TEMP_SCALE + self.config.temp_offset,
        ))
    }

    /// Read all sensors in a single call.
    ///
    /// Returns gyroscope, accelerometer, magnetometer, and temperature data in a
    /// [`SensorData`] struct. This is preferred over separate `read_gyro()`, `read_accel()`,
    /// `read_mag()`, and `read_temp()` calls when temporal coherence matters, such as in sensor
    /// fusion algorithms (Madgwick, Mahony, EKF).
    ///
    /// Note: while [`BlockDataUpdate`] prevents partial reads within each sensor, there is still
    /// inter-sensor skew between the sequential bus transactions. For the tightest synchronization,
    /// use hardware DRDY interrupts.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails, or
    /// [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use lsm9ds0::{Lsm9ds0, I2cInterface, Error};
    /// # struct Delay;
    /// # impl embedded_hal_async::delay::DelayNs for Delay {
    /// #     async fn delay_ns(&mut self, _ns: u32) {}
    /// # }
    /// # async fn example<I>(i2c: I) -> Result<(), Error<I::Error>>
    /// # where I: embedded_hal_async::i2c::I2c
    /// # {
    /// use lsm9ds0::{Lsm9ds0Config, AccelDataRate, GyroDataRate, MagMode};
    ///
    /// let config = Lsm9ds0Config::new()
    ///     .with_gyro_enabled(true)
    ///     .with_gyro_data_rate(GyroDataRate::Hz95)
    ///     .with_accel_data_rate(AccelDataRate::Hz100)
    ///     .with_mag_mode(MagMode::ContinuousConversion)
    ///     .with_temperature_enabled(true);
    ///
    /// let interface = I2cInterface::init(i2c);
    /// let mut imu = Lsm9ds0::new_with_config(interface, config);
    /// imu.init(&mut Delay).await?;
    ///
    /// let data = imu.read_all().await?;
    /// let (gx, gy, gz) = data.gyro;
    /// let (ax, ay, az) = data.accel;
    /// let (mx, my, mz) = data.mag;
    /// let temp = data.temp;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn read_all(&mut self) -> Result<types::SensorData, Error<I::BusError>> {
        let gyro = self.read_gyro().await?;
        let accel = self.read_accel().await?;
        let mag = self.read_mag().await?;
        let temp = self.read_temp().await?;

        Ok(types::SensorData {
            gyro,
            accel,
            mag,
            temp,
        })
    }

    // =========================================================================
    // RUNTIME CONFIGURATION METHODS
    //
    // These methods modify the shadow register and write directly to hardware.
    // =========================================================================

    /// Set gyroscope scale.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn set_gyro_scale(&mut self, scale: GyroScale) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg4_g.set_fs(scale);
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG4_G.addr(),
                self.config.ctrl_reg4_g.into(),
            )
            .await?;
        Ok(())
    }

    /// Set gyroscope data rate.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn set_gyro_data_rate(
        &mut self,
        rate: GyroDataRate,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg1_g.set_dr(rate);
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG1_G.addr(),
                self.config.ctrl_reg1_g.into(),
            )
            .await?;
        Ok(())
    }

    /// Enable/disable gyroscope (power mode).
    ///
    /// For more control including sleep mode, use [`set_gyro_power_mode`](Self::set_gyro_power_mode).
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn set_gyro_enabled(&mut self, enabled: bool) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg1_g.set_pd(if enabled {
            PowerMode::Normal
        } else {
            PowerMode::PowerDown
        });
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG1_G.addr(),
                self.config.ctrl_reg1_g.into(),
            )
            .await?;
        Ok(())
    }

    /// Set gyroscope power mode with sleep support.
    ///
    /// The gyroscope supports three power modes:
    /// - **PowerDown**: Lowest power consumption, sensor completely off
    /// - **Sleep**: Lower power than normal, but faster wake-up than power-down.
    ///   Useful for motion-triggered wake-up scenarios.
    /// - **Normal**: Full operation with configured axes enabled
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    ///
    /// # Example
    /// ```ignore
    /// use lsm9ds0::GyroPowerMode;
    ///
    /// // Enter sleep mode for faster wake-up
    /// imu.set_gyro_power_mode(GyroPowerMode::Sleep).await?;
    ///
    /// // Wake up to normal operation
    /// imu.set_gyro_power_mode(GyroPowerMode::Normal).await?;
    /// ```
    pub async fn set_gyro_power_mode(
        &mut self,
        mode: GyroPowerMode,
    ) -> Result<(), Error<I::BusError>> {
        match mode {
            GyroPowerMode::PowerDown => {
                self.config.ctrl_reg1_g.set_pd(PowerMode::PowerDown);
            }
            GyroPowerMode::Sleep => {
                // Sleep mode: PD=1, all axes disabled
                self.config.ctrl_reg1_g.set_pd(PowerMode::Normal);
                self.config.ctrl_reg1_g.set_xen(Enable::Disabled);
                self.config.ctrl_reg1_g.set_yen(Enable::Disabled);
                self.config.ctrl_reg1_g.set_zen(Enable::Disabled);
            }
            GyroPowerMode::Normal => {
                self.config.ctrl_reg1_g.set_pd(PowerMode::Normal);
                // Restore saved axis states
                let (x, y, z) = self.config.gyro_axes_enabled;
                self.config.ctrl_reg1_g.set_xen(Enable::from(x));
                self.config.ctrl_reg1_g.set_yen(Enable::from(y));
                self.config.ctrl_reg1_g.set_zen(Enable::from(z));
            }
        }
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG1_G.addr(),
                self.config.ctrl_reg1_g.into(),
            )
            .await?;
        Ok(())
    }

    /// Enable/disable gyroscope axes.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn set_gyro_axes(
        &mut self,
        x: bool,
        y: bool,
        z: bool,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg1_g.set_xen(Enable::from(x));
        self.config.ctrl_reg1_g.set_yen(Enable::from(y));
        self.config.ctrl_reg1_g.set_zen(Enable::from(z));
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG1_G.addr(),
                self.config.ctrl_reg1_g.into(),
            )
            .await?;
        Ok(())
    }

    /// Set gyroscope bandwidth.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn set_gyro_bandwidth(
        &mut self,
        bw: GyroBandwidth,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg1_g.set_bw(bw);
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG1_G.addr(),
                self.config.ctrl_reg1_g.into(),
            )
            .await?;
        Ok(())
    }

    /// Set gyroscope reference value for high-pass filter.
    ///
    /// This value is used as the reference for the gyroscope high-pass filter when HPM is set to
    /// "Reference" mode.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn set_gyro_reference(&mut self, value: u8) -> Result<(), Error<I::BusError>> {
        self.interface
            .write_byte_gyro(GyroRegisters::REFERENCE_G.addr(), value)
            .await?;
        Ok(())
    }

    /// Set accelerometer reference values for high-pass filter.
    ///
    /// These values are used as the reference for the accelerometer high-pass filter when AHPM is
    /// set to "Reference" mode.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_accel_reference(
        &mut self,
        x: u8,
        y: u8,
        z: u8,
    ) -> Result<(), Error<I::BusError>> {
        self.interface
            .write_xm(AccelMagRegisters::REFERENCE_X.addr(), &[x, y, z])
            .await?;
        Ok(())
    }

    /// Set accelerometer scale.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_accel_scale(&mut self, scale: AccelScale) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg2_xm.set_afs(scale);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG2_XM.addr(),
                self.config.ctrl_reg2_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set accelerometer data rate.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_accel_data_rate(
        &mut self,
        rate: AccelDataRate,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg1_xm.set_aodr(rate);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG1_XM.addr(),
                self.config.ctrl_reg1_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Enable/disable accelerometer axes.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_accel_axes(
        &mut self,
        x: bool,
        y: bool,
        z: bool,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg1_xm.set_axen(Enable::from(x));
        self.config.ctrl_reg1_xm.set_ayen(Enable::from(y));
        self.config.ctrl_reg1_xm.set_azen(Enable::from(z));
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG1_XM.addr(),
                self.config.ctrl_reg1_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set accelerometer bandwidth.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_accel_bandwidth(
        &mut self,
        bw: AccelBandwidth,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg2_xm.set_abw(bw);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG2_XM.addr(),
                self.config.ctrl_reg2_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set magnetometer scale.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_mag_scale(&mut self, scale: MagScale) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg6_xm.set_mfs(scale);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG6_XM.addr(),
                self.config.ctrl_reg6_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set magnetometer data rate.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_mag_data_rate(&mut self, rate: MagDataRate) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg5_xm.set_m_odr(rate);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG5_XM.addr(),
                self.config.ctrl_reg5_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set magnetometer operating mode.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_mag_mode(&mut self, mode: MagMode) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg7_xm.set_md(mode);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG7_XM.addr(),
                self.config.ctrl_reg7_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set magnetometer resolution.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_mag_resolution(
        &mut self,
        res: MagResolution,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg5_xm.set_m_res(res);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG5_XM.addr(),
                self.config.ctrl_reg5_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Enable/disable temperature sensor.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_temperature_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<I::BusError>> {
        self.config.ctrl_reg5_xm.set_temp_en(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG5_XM.addr(),
                self.config.ctrl_reg5_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Set block data update mode for both gyro and accel/mag.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails, or
    /// [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn set_block_data_update(&mut self, enabled: bool) -> Result<(), Error<I::BusError>> {
        let bdu = if enabled {
            BlockDataUpdate::WaitForRead
        } else {
            BlockDataUpdate::Continuous
        };
        self.config.ctrl_reg4_g.set_bdu(bdu);
        self.config.ctrl_reg1_xm.set_bdu(bdu);
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG4_G.addr(),
                self.config.ctrl_reg4_g.into(),
            )
            .await?;
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG1_XM.addr(),
                self.config.ctrl_reg1_xm.into(),
            )
            .await?;
        Ok(())
    }

    /// Get the gyroscope high-pass filter cutoff frequency in Hz
    /// based on the current configuration
    pub fn get_gyro_hpf_cutoff_hz(&self) -> f32 {
        self.config.gyro_hpf_cutoff_hz()
    }

    /// Get the gyroscope low-pass filter cutoff frequency in Hz
    /// based on the current configuration (ODR and bandwidth setting)
    pub fn get_gyro_lpf_cutoff_hz(&self) -> f32 {
        self.config.gyro_lpf_cutoff_hz()
    }

    // =========================================================================
    // STATUS AND FIFO METHODS
    // =========================================================================

    /// Read gyroscope FIFO source register to get FIFO status.
    ///
    /// Returns a typed struct with FIFO level, empty, overrun, and watermark status.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro_fifo_status(&mut self) -> Result<FifoSrcRegG, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_gyro(GyroRegisters::FIFO_SRC_REG_G.addr(), &mut data)
            .await?;
        Ok(FifoSrcRegG::from(data[0]))
    }

    /// Read accelerometer FIFO source register to get FIFO status.
    ///
    /// Returns a typed struct with FIFO level, empty, overrun, and watermark status.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_fifo_status(&mut self) -> Result<FifoSrcReg, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::FIFO_SRC_REG.addr(), &mut data)
            .await?;
        Ok(FifoSrcReg::from(data[0]))
    }

    /// Read gyroscope status register.
    ///
    /// Returns a typed struct with fields for data available and overrun status per axis.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro_status(&mut self) -> Result<StatusRegG, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_gyro(GyroRegisters::STATUS_REG_G.addr(), &mut data)
            .await?;
        Ok(StatusRegG::from(data[0]))
    }

    /// Read accelerometer status register.
    ///
    /// Returns a typed struct with fields for data available and overrun status per axis.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_status(&mut self) -> Result<StatusRegA, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::STATUS_REG_A.addr(), &mut data)
            .await?;
        Ok(StatusRegA::from(data[0]))
    }

    /// Read magnetometer status register.
    ///
    /// Returns a typed struct with fields for data available and overrun status per axis.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_mag_status(&mut self) -> Result<StatusRegM, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::STATUS_REG_M.addr(), &mut data)
            .await?;
        Ok(StatusRegM::from(data[0]))
    }

    // =========================================================================
    // DATA READY CONVENIENCE METHODS
    // =========================================================================

    /// Check if new gyroscope data is available on all axes.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn gyro_data_ready(&mut self) -> Result<bool, Error<I::BusError>> {
        let status = self.read_gyro_status().await?;
        Ok(status.zyxda())
    }

    /// Check if new accelerometer data is available on all axes.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn accel_data_ready(&mut self) -> Result<bool, Error<I::BusError>> {
        let status = self.read_accel_status().await?;
        Ok(status.zyxada())
    }

    /// Check if new magnetometer data is available on all axes.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn mag_data_ready(&mut self) -> Result<bool, Error<I::BusError>> {
        let status = self.read_mag_status().await?;
        Ok(status.zyxmda())
    }

    /// Check if gyroscope data overrun occurred (data was overwritten before being read).
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn gyro_data_overrun(&mut self) -> Result<bool, Error<I::BusError>> {
        let status = self.read_gyro_status().await?;
        Ok(status.zyxor())
    }

    /// Check if accelerometer data overrun occurred (data was overwritten before being read).
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn accel_data_overrun(&mut self) -> Result<bool, Error<I::BusError>> {
        let status = self.read_accel_status().await?;
        Ok(status.zyxaor())
    }

    /// Check if magnetometer data overrun occurred (data was overwritten before being read).
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn mag_data_overrun(&mut self) -> Result<bool, Error<I::BusError>> {
        let status = self.read_mag_status().await?;
        Ok(status.zyxmor())
    }

    // =========================================================================
    // INTERRUPT SOURCE METHODS
    // =========================================================================

    /// Read gyroscope interrupt source register.
    ///
    /// **Note:** Reading this register clears the interrupt if latched mode is enabled.
    ///
    /// # Errors
    ///
    /// Returns [`Error::GyroBus`] if communication with the gyroscope fails.
    pub async fn read_gyro_interrupt_source(&mut self) -> Result<Int1SrcG, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_gyro(GyroRegisters::INT1_SRC_G.addr(), &mut data)
            .await?;
        Ok(Int1SrcG::from(data[0]))
    }

    /// Read accelerometer interrupt generator 1 source register.
    ///
    /// **Note:** Reading this register clears the interrupt if latched mode is enabled.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_int1_source(&mut self) -> Result<IntGenSrc, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::INT_GEN_1_SRC.addr(), &mut data)
            .await?;
        Ok(IntGenSrc::from(data[0]))
    }

    /// Read accelerometer interrupt generator 2 source register.
    ///
    /// **Note:** Reading this register clears the interrupt if latched mode is enabled.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_accel_int2_source(&mut self) -> Result<IntGenSrc, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::INT_GEN_2_SRC.addr(), &mut data)
            .await?;
        Ok(IntGenSrc::from(data[0]))
    }

    /// Read magnetometer interrupt source register.
    ///
    /// **Note:** Reading this register clears the interrupt if latched mode is enabled.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_mag_interrupt_source(&mut self) -> Result<IntSrcRegM, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::INT_SRC_REG_M.addr(), &mut data)
            .await?;
        Ok(IntSrcRegM::from(data[0]))
    }

    /// Read click/tap detection source register.
    ///
    /// **Note:** Reading this register clears the interrupt if latched mode is enabled.
    ///
    /// # Errors
    ///
    /// Returns [`Error::XmBus`] if communication with the accelerometer/magnetometer fails.
    pub async fn read_click_source(&mut self) -> Result<ClickSrc, Error<I::BusError>> {
        let mut data = [0u8];
        self.interface
            .read_xm(AccelMagRegisters::CLICK_SRC.addr(), &mut data)
            .await?;
        Ok(ClickSrc::from(data[0]))
    }

    /// Calibrate gyroscope and accelerometer bias using FIFO averaging.
    ///
    /// This method collects samples while the device is stationary and calculates bias values that
    /// are then subtracted from future readings. The device **must be stationary** during
    /// calibration for accurate results.
    ///
    /// The calibration process:
    /// 1. Enables both gyro and accel FIFOs simultaneously in stream mode
    /// 2. Waits for FIFOs to fill (time based on ODR)
    /// 3. Reads and averages samples from both FIFOs
    /// 4. For accelerometer, subtracts expected gravity based on `orientation`
    /// 5. Restores original FIFO settings
    ///
    /// Determining the estimated delay for calibration:
    ///
    /// - delay_ms = 32_FIFO_samples / ODR, with 10% margin and a minimum floor.
    /// - Max delay between gyro and accelerometer sensor is used
    ///   - Gyro 760 Hz & Accel 1600 Hz → ~46 ms (because gyro takes longer)
    ///   - Gyro  95 Hz + Accel  100 Hz → ~370 ms (because gyro takes longer)
    ///   - Accel 3.125 Hz              → ~11,264 ms (because accel can be super slow)
    ///
    /// # Arguments
    ///
    /// * `delay` - A delay provider for timing
    /// * `orientation` - The sensor orientation during calibration. Use a specific
    ///   orientation (e.g., [`Orientation::ZUp`]) for gravity-referenced readings,
    ///   or [`Orientation::Unknown`] for pose-relative readings.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the sensor fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use embassy_time::Delay;
    /// use lsm9ds0::Orientation;
    ///
    /// // Calibrate with sensor Z-axis pointing up
    /// imu.calibrate_bias(&mut Delay, Orientation::ZUp).await?;
    ///
    /// // Now readings are bias-corrected
    /// let (gx, gy, gz) = imu.read_gyro().await?;
    /// ```
    pub async fn calibrate_bias<D: DelayNs>(
        &mut self,
        delay: &mut D,
        orientation: types::Orientation,
    ) -> Result<(), Error<I::BusError>> {
        // Save current FIFO settings for both Gyro and Accelerometer sensors
        let saved_gyro_fifo_en = self.config.ctrl_reg5_g.fifo_en();
        let saved_gyro_fifo_mode = self.config.fifo_ctrl_reg_g.fm();
        let saved_gyro_fifo_wtm = self.config.fifo_ctrl_reg_g.wtm();

        let saved_accel_fifo_en = self.config.ctrl_reg0_xm.fifo_en();
        let saved_accel_fifo_mode = self.config.fifo_ctrl_reg.fm();
        let saved_accel_fifo_wtm = self.config.fifo_ctrl_reg.fth();

        // Configure gyro FIFO: enable + stream mode + watermark at 31
        self.config.ctrl_reg5_g.set_fifo_en(Enable::Enabled);
        self.config.fifo_ctrl_reg_g.set_wtm(0x1F);

        // Configure accel FIFO: enable + stream mode + watermark at 31
        self.config.ctrl_reg0_xm.set_fifo_en(Enable::Enabled);
        self.config.fifo_ctrl_reg.set_fth(0x1F);

        // Write all FIFO configuration registers
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG5_G.addr(),
                self.config.ctrl_reg5_g.into(),
            )
            .await?;
        self.config.fifo_ctrl_reg_g.set_fm(FifoMode::Bypass);
        self.interface
            .write_byte_gyro(
                GyroRegisters::FIFO_CTRL_REG_G.addr(),
                self.config.fifo_ctrl_reg_g.into(),
            )
            .await?;
        self.config.fifo_ctrl_reg_g.set_fm(FifoMode::Stream);
        self.interface
            .write_byte_gyro(
                GyroRegisters::FIFO_CTRL_REG_G.addr(),
                self.config.fifo_ctrl_reg_g.into(),
            )
            .await?;

        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG0_XM.addr(),
                self.config.ctrl_reg0_xm.into(),
            )
            .await?;

        self.config.fifo_ctrl_reg.set_fm(FifoMode::Bypass);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::FIFO_CTRL_REG.addr(),
                self.config.fifo_ctrl_reg.into(),
            )
            .await?;

        self.config.fifo_ctrl_reg.set_fm(FifoMode::Stream);
        self.interface
            .write_byte_xm(
                AccelMagRegisters::FIFO_CTRL_REG.addr(),
                self.config.fifo_ctrl_reg.into(),
            )
            .await?;

        // Wait for both FIFOs to fill based on the configured data rate (ODR).
        // delay_ms = NUM_FIFO_SAMPLES / ODR, with 10% margin and a minimum floor.
        //
        // Use the max delay between the two sensors.
        //
        // Delay range by data rate (32 samples × 1.1 margin):
        //   Gyro 760 Hz & Accel 1600 Hz → ~46 ms
        //   Gyro  95 Hz + Accel  100 Hz → ~370 ms
        //   Accel 3.125 Hz              → ~11,264 ms
        const NUM_FIFO_SAMPLES: f32 = 32.0;
        const MARGIN: f32 = 1.1;
        const MIN_DELAY_MS: u32 = 10;

        let gyro_hz = self.config.ctrl_reg1_g.dr().hz();
        let accel_hz = self.config.ctrl_reg1_xm.aodr().hz();

        let gyro_fill_ms = NUM_FIFO_SAMPLES * 1000.0 / gyro_hz;
        let accel_fill_ms = if accel_hz > 0.0 {
            NUM_FIFO_SAMPLES * 1000.0 / accel_hz
        } else {
            0.0
        };

        let fill_ms = gyro_fill_ms.max(accel_fill_ms);

        let delay_ms = ((fill_ms * MARGIN) as u32).max(MIN_DELAY_MS);
        delay.delay_ms(delay_ms).await;

        // Read sample counts from both FIFOs
        let gyro_samples = self.read_gyro_fifo_level().await?;
        let accel_samples = self.read_accel_fifo_level().await?;

        // Accumulate gyro samples
        let mut gyro_sum: (i32, i32, i32) = (0, 0, 0);
        for _ in 0..gyro_samples {
            let (x, y, z) = self.read_gyro_fifo().await?;
            gyro_sum.0 += x as i32;
            gyro_sum.1 += y as i32;
            gyro_sum.2 += z as i32;
        }

        // Accumulate accel samples
        let mut accel_sum: (i32, i32, i32) = (0, 0, 0);
        for _ in 0..accel_samples {
            let (x, y, z) = self.read_accel_fifo().await?;
            accel_sum.0 += x as i32;
            accel_sum.1 += y as i32;
            accel_sum.2 += z as i32;
        }

        // Calculate gyro bias (average, then convert to dps)
        let gyro_sensitivity = self.config.gyro_sensitivity() / 1000.0;
        let gyro_bias = if gyro_samples > 0 {
            let n = gyro_samples as f32;
            (
                (gyro_sum.0 as f32 / n) * gyro_sensitivity,
                (gyro_sum.1 as f32 / n) * gyro_sensitivity,
                (gyro_sum.2 as f32 / n) * gyro_sensitivity,
            )
        } else {
            (0.0, 0.0, 0.0)
        };

        // Calculate accel bias (average, convert to g, subtract gravity)
        let accel_sensitivity = self.config.accel_sensitivity() / 1000.0;
        let (grav_x, grav_y, grav_z) = orientation.gravity_vector();
        let accel_bias = if accel_samples > 0 {
            let n = accel_samples as f32;
            (
                (accel_sum.0 as f32 / n) * accel_sensitivity - grav_x,
                (accel_sum.1 as f32 / n) * accel_sensitivity - grav_y,
                (accel_sum.2 as f32 / n) * accel_sensitivity - grav_z,
            )
        } else {
            (0.0, 0.0, 0.0)
        };

        // Store calculated bias values
        self.config.gyro_bias = gyro_bias;
        self.config.accel_bias = accel_bias;

        // Restore original FIFO settings
        self.config.ctrl_reg5_g.set_fifo_en(saved_gyro_fifo_en);
        self.config.fifo_ctrl_reg_g.set_fm(saved_gyro_fifo_mode);
        self.config.fifo_ctrl_reg_g.set_wtm(saved_gyro_fifo_wtm);

        self.config.ctrl_reg0_xm.set_fifo_en(saved_accel_fifo_en);
        self.config.fifo_ctrl_reg.set_fm(saved_accel_fifo_mode);
        self.config.fifo_ctrl_reg.set_fth(saved_accel_fifo_wtm);

        // Write restored FIFO settings
        self.interface
            .write_byte_gyro(
                GyroRegisters::CTRL_REG5_G.addr(),
                self.config.ctrl_reg5_g.into(),
            )
            .await?;
        self.interface
            .write_byte_gyro(
                GyroRegisters::FIFO_CTRL_REG_G.addr(),
                self.config.fifo_ctrl_reg_g.into(),
            )
            .await?;
        self.interface
            .write_byte_xm(
                AccelMagRegisters::CTRL_REG0_XM.addr(),
                self.config.ctrl_reg0_xm.into(),
            )
            .await?;
        self.interface
            .write_byte_xm(
                AccelMagRegisters::FIFO_CTRL_REG.addr(),
                self.config.fifo_ctrl_reg.into(),
            )
            .await?;

        Ok(())
    }

    /// Get the current gyroscope bias values (x, y, z) in degrees per second.
    pub fn gyro_bias(&self) -> (f32, f32, f32) {
        self.config.gyro_bias
    }

    /// Get the current accelerometer bias values (x, y, z) in g-force.
    pub fn accel_bias(&self) -> (f32, f32, f32) {
        self.config.accel_bias
    }

    /// Set gyroscope bias values directly.
    ///
    /// Use this to apply pre-calibrated bias values without running the calibration routine.
    pub fn set_gyro_bias(&mut self, x: f32, y: f32, z: f32) {
        self.config.gyro_bias = (x, y, z);
    }

    /// Set accelerometer bias values directly.
    ///
    /// Use this to apply pre-calibrated bias values without running the calibration routine.
    pub fn set_accel_bias(&mut self, x: f32, y: f32, z: f32) {
        self.config.accel_bias = (x, y, z);
    }

    /// Release the interface (consume self and return interface)
    pub fn release(self) -> I {
        self.interface
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use std::vec;

    // I2C addresses from device_constants
    const GYRO_ADDR: u8 = registers::device_constants::gyro::I2C_ADDR_0;
    const XM_ADDR: u8 = registers::device_constants::xm::I2C_ADDR_0;

    #[tokio::test]
    async fn test_read_gyro_raw() {
        const AUTO_INCREMENT: u8 = 0x80;
        // Gyro reads from OUT_X_L_G (0x28), reads 6 bytes
        // Expected I2C transaction: write register address, then read 6 bytes
        let expectations = [I2cTransaction::write_read(
            GYRO_ADDR,
            vec![GyroRegisters::OUT_X_L_G.addr() | AUTO_INCREMENT],
            // Response: X=0x0102, Y=0x0304, Z=0x0506 (little-endian)
            vec![0x02, 0x01, 0x04, 0x03, 0x06, 0x05],
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_gyro_raw().await.unwrap();

        // Little-endian: low byte first, then high byte
        assert_eq!(x, 0x0102); // 258
        assert_eq!(y, 0x0304); // 772
        assert_eq!(z, 0x0506); // 1286

        // Verify all expectations were met
        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_read_gyro_raw_negative_values() {
        const AUTO_INCREMENT: u8 = 0x80;
        let expectations = [I2cTransaction::write_read(
            GYRO_ADDR,
            vec![GyroRegisters::OUT_X_L_G.addr() | AUTO_INCREMENT],
            // X=-1 (0xFFFF), Y=-256 (0xFF00), Z=-32768 (0x8000)
            vec![0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x80],
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_gyro_raw().await.unwrap();

        assert_eq!(x, -1i16);
        assert_eq!(y, -256i16);
        assert_eq!(z, -32768i16);

        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_read_accel_raw() {
        const AUTO_INCREMENT: u8 = 0x80;
        // Accel reads from OUT_X_L_A (0x28) on XM address
        let expectations = [I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::OUT_X_L_A.addr() | AUTO_INCREMENT],
            // X=100, Y=200, Z=300 (little-endian)
            vec![0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01],
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_accel_raw().await.unwrap();

        assert_eq!(x, 100);
        assert_eq!(y, 200);
        assert_eq!(z, 300);

        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_read_accel_raw_negative_values() {
        const AUTO_INCREMENT: u8 = 0x80;

        let expectations = [I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::OUT_X_L_A.addr() | AUTO_INCREMENT],
            // X=-100, Y=-200, Z=-300
            vec![0x9C, 0xFF, 0x38, 0xFF, 0xD4, 0xFE],
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_accel_raw().await.unwrap();

        assert_eq!(x, -100i16);
        assert_eq!(y, -200i16);
        assert_eq!(z, -300i16);

        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_read_mag_raw() {
        const AUTO_INCREMENT: u8 = 0x80;
        // Mag reads from OUT_X_L_M (0x08) on XM address
        let expectations = [I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::OUT_X_L_M.addr() | AUTO_INCREMENT],
            // X=1000, Y=2000, Z=3000 (little-endian)
            vec![0xE8, 0x03, 0xD0, 0x07, 0xB8, 0x0B],
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_mag_raw().await.unwrap();

        assert_eq!(x, 1000);
        assert_eq!(y, 2000);
        assert_eq!(z, 3000);

        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_read_mag_raw_negative_values() {
        const AUTO_INCREMENT: u8 = 0x80;
        let expectations = [I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::OUT_X_L_M.addr() | AUTO_INCREMENT],
            // X=-1000, Y=-2000, Z=-3000
            vec![0x18, 0xFC, 0x30, 0xF8, 0x48, 0xF4],
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_mag_raw().await.unwrap();

        assert_eq!(x, -1000i16);
        assert_eq!(y, -2000i16);
        assert_eq!(z, -3000i16);

        driver.release().release().done();
    }

    // =========================================================================
    // SEND ASSERTION
    // =========================================================================

    #[test]
    fn assert_send() {
        fn is_send<T: Send>() {}
        is_send::<Lsm9ds0<I2cInterface<I2cMock>>>();
    }

    // =========================================================================
    // INIT SEQUENCE TEST
    // =========================================================================

    /// Build the I2C transaction expectations for a full `init()` with default config.
    fn default_init_expectations() -> std::vec::Vec<I2cTransaction> {
        let config = Lsm9ds0Config::default();

        vec![
            // verify_device_ids: read WHO_AM_I_G (single byte, no auto-increment)
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::WHO_AM_I_G.addr()],
                vec![registers::device_constants::gyro::DEVICE_ID],
            ),
            // verify_device_ids: read WHO_AM_I_XM (single byte, no auto-increment)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::WHO_AM_I_XM.addr()],
                vec![registers::device_constants::xm::DEVICE_ID],
            ),
            // apply_configs: CTRL_REG1_G through CTRL_REG5_G (0x20-0x24, 5 bytes, auto-increment)
            I2cTransaction::write(
                GYRO_ADDR,
                vec![
                    GyroRegisters::CTRL_REG1_G.addr() | 0x80,
                    config.ctrl_reg1_g.into(),
                    config.ctrl_reg2_g.into(),
                    config.ctrl_reg3_g.into(),
                    config.ctrl_reg4_g.into(),
                    config.ctrl_reg5_g.into(),
                ],
            ),
            // FIFO_CTRL_REG_G (single byte)
            I2cTransaction::write(
                GYRO_ADDR,
                vec![
                    GyroRegisters::FIFO_CTRL_REG_G.addr(),
                    config.fifo_ctrl_reg_g.into(),
                ],
            ),
            // INT1_CFG_G (single byte)
            I2cTransaction::write(
                GYRO_ADDR,
                vec![GyroRegisters::INT1_CFG_G.addr(), config.int1_cfg_g.into()],
            ),
            // INT1_THS_XH_G through INT1_DURATION_G (0x32-0x38, 7 bytes, auto-increment)
            I2cTransaction::write(
                GYRO_ADDR,
                vec![
                    GyroRegisters::INT1_THS_XH_G.addr() | 0x80,
                    0x00,
                    0x00, // gyro_int_ths_x
                    0x00,
                    0x00, // gyro_int_ths_y
                    0x00,
                    0x00, // gyro_int_ths_z
                    config.int1_duration_g.into(),
                ],
            ),
            // INT_CTRL_REG_M (single byte)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::INT_CTRL_REG_M.addr(),
                    config.int_ctrl_reg_m.into(),
                ],
            ),
            // INT_THS_L_M, INT_THS_H_M (2 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![AccelMagRegisters::INT_THS_L_M.addr() | 0x80, 0x00, 0x00],
            ),
            // OFFSET_X_L_M through OFFSET_Z_H_M (6 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::OFFSET_X_L_M.addr() | 0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                ],
            ),
            // CTRL_REG0_XM through CTRL_REG7_XM (8 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::CTRL_REG0_XM.addr() | 0x80,
                    config.ctrl_reg0_xm.into(),
                    config.ctrl_reg1_xm.into(),
                    config.ctrl_reg2_xm.into(),
                    config.ctrl_reg3_xm.into(),
                    config.ctrl_reg4_xm.into(),
                    config.ctrl_reg5_xm.into(),
                    config.ctrl_reg6_xm.into(),
                    config.ctrl_reg7_xm.into(),
                ],
            ),
            // FIFO_CTRL_REG (single byte)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::FIFO_CTRL_REG.addr(),
                    config.fifo_ctrl_reg.into(),
                ],
            ),
            // INT_GEN_1_REG (single byte)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::INT_GEN_1_REG.addr(),
                    config.int_gen_1_reg.into(),
                ],
            ),
            // INT_GEN_1_THS, INT_GEN_1_DURATION (2 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![AccelMagRegisters::INT_GEN_1_THS.addr() | 0x80, 0x00, 0x00],
            ),
            // INT_GEN_2_REG (single byte)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::INT_GEN_2_REG.addr(),
                    config.int_gen_2_reg.into(),
                ],
            ),
            // INT_GEN_2_THS, INT_GEN_2_DURATION (2 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![AccelMagRegisters::INT_GEN_2_THS.addr() | 0x80, 0x00, 0x00],
            ),
            // CLICK_CFG (single byte)
            I2cTransaction::write(
                XM_ADDR,
                vec![AccelMagRegisters::CLICK_CFG.addr(), config.click_cfg.into()],
            ),
            // CLICK_THS through TIME_WINDOW (4 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![
                    AccelMagRegisters::CLICK_THS.addr() | 0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                ],
            ),
            // ACT_THS, ACT_DUR (2 bytes, auto-increment)
            I2cTransaction::write(
                XM_ADDR,
                vec![AccelMagRegisters::ACT_THS.addr() | 0x80, 0x00, 0x00],
            ),
        ]
    }

    /// Mock delay that does nothing (for testing init without real hardware)
    struct MockDelay;
    impl embedded_hal_async::delay::DelayNs for MockDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    #[tokio::test]
    async fn test_init_sequence() {
        let expectations = default_init_expectations();
        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        driver.init(&mut MockDelay).await.unwrap();

        driver.release().release().done();
    }

    // =========================================================================
    // RUNTIME CONFIG TESTS
    // =========================================================================

    #[tokio::test]
    async fn test_set_gyro_scale_writes_register() {
        // Create driver, run init, then call set_gyro_scale
        let mut expectations = default_init_expectations();

        // After init, set_gyro_scale(Dps2000) writes CTRL_REG4_G (0x23)
        // Default CTRL_REG4_G is 0x00 (Dps245, continuous, little-endian, no self-test, 4-wire)
        // With Dps2000: FS bits [5:4] = 0b10, so byte = 0x20
        let expected_reg4: u8 = registers::CtrlReg4G::new()
            .with_fs(GyroScale::Dps2000)
            .into();
        expectations.push(I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG4_G.addr(), expected_reg4],
        ));

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        driver.init(&mut MockDelay).await.unwrap();
        driver.set_gyro_scale(GyroScale::Dps2000).await.unwrap();

        driver.release().release().done();
    }

    // =========================================================================
    // SCALED READ TESTS
    // =========================================================================

    #[tokio::test]
    async fn test_read_gyro_scaled() {
        const AUTO_INCREMENT: u8 = 0x80;
        // Raw values: X=1000, Y=-500, Z=250
        let expectations = [I2cTransaction::write_read(
            GYRO_ADDR,
            vec![GyroRegisters::OUT_X_L_G.addr() | AUTO_INCREMENT],
            vec![0xE8, 0x03, 0x0C, 0xFE, 0xFA, 0x00], // 1000, -500, 250 LE
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        // Default scale is Dps245 → sensitivity 8.75 mdps/LSB = 0.00875 dps/LSB
        // Set a known bias
        driver.config.gyro_bias = (1.0, -0.5, 0.0);

        let (gx, gy, gz) = driver.read_gyro().await.unwrap();

        // Expected: raw * (8.75/1000) - bias
        // X: 1000 * 0.00875 - 1.0 = 7.75
        // Y: -500 * 0.00875 - (-0.5) = -3.875
        // Z: 250 * 0.00875 - 0.0 = 2.1875
        let epsilon = 0.001;
        assert!((gx.as_f32() - 7.75).abs() < epsilon, "gx={}", gx.as_f32());
        assert!(
            (gy.as_f32() - (-3.875)).abs() < epsilon,
            "gy={}",
            gy.as_f32()
        );
        assert!((gz.as_f32() - 2.1875).abs() < epsilon, "gz={}", gz.as_f32());

        driver.release().release().done();
    }

    // =========================================================================
    // ERROR PROPAGATION TEST
    // =========================================================================

    #[tokio::test]
    async fn test_error_propagation() {
        use embedded_hal_async::i2c::ErrorKind;

        const AUTO_INCREMENT: u8 = 0x80;
        // Return an error on gyro read
        let expectations = [I2cTransaction::write_read(
            GYRO_ADDR,
            vec![GyroRegisters::OUT_X_L_G.addr() | AUTO_INCREMENT],
            vec![0u8; 6],
        )
        .with_error(ErrorKind::Other)];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let result = driver.read_gyro_raw().await;
        assert!(matches!(result, Err(Error::GyroBus(_))));

        driver.release().release().done();
    }

    // =========================================================================
    // TEMPERATURE READ TEST
    // =========================================================================

    #[tokio::test]
    async fn test_read_temp() {
        const AUTO_INCREMENT: u8 = 0x80;

        // Test positive value: raw 12-bit = 200 → 200/8.0 + 25.0 = 50.0°C
        let expectations = [I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::OUT_TEMP_L_XM.addr() | AUTO_INCREMENT],
            vec![0xC8, 0x00], // 200 LE
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let temp = driver.read_temp().await.unwrap();
        let epsilon = 0.01;
        assert!(
            (temp.as_f32() - 50.0).abs() < epsilon,
            "positive temp={}",
            temp.as_f32()
        );
        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_read_temp_negative() {
        const AUTO_INCREMENT: u8 = 0x80;

        // Test negative value: raw 12-bit = -80 (0xFB0 sign-extended in 16-bit = 0xFB0)
        // As i16 LE: low=0xB0, high=0x0F → i16 = 0x0FB0 = 4016
        // After (raw << 4) >> 4 sign extension: 0xFB0 → -80
        // Result: -80/8.0 + 25.0 = 15.0°C
        let expectations = [I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::OUT_TEMP_L_XM.addr() | AUTO_INCREMENT],
            vec![0xB0, 0x0F], // 12-bit -80 stored as i16 LE with upper 4 bits = 0xF
        )];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let temp = driver.read_temp().await.unwrap();
        let epsilon = 0.01;
        assert!(
            (temp.as_f32() - 15.0).abs() < epsilon,
            "negative temp={}",
            temp.as_f32()
        );
        driver.release().release().done();
    }

    // =========================================================================
    // READ_ALL TEST
    // =========================================================================

    #[tokio::test]
    async fn test_read_all() {
        const AUTO_INCREMENT: u8 = 0x80;

        // Gyro raw: X=1000, Y=-500, Z=250
        // Accel raw: X=100, Y=200, Z=300
        // Mag raw: X=500, Y=-1000, Z=1500
        // Temp raw: 200 → 200/8.0 + 25.0 = 50.0°C
        let expectations = [
            // read_gyro → read_gyro_raw
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::OUT_X_L_G.addr() | AUTO_INCREMENT],
                vec![0xE8, 0x03, 0x0C, 0xFE, 0xFA, 0x00],
            ),
            // read_accel → read_accel_raw
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::OUT_X_L_A.addr() | AUTO_INCREMENT],
                vec![0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01],
            ),
            // read_mag → read_mag_raw
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::OUT_X_L_M.addr() | AUTO_INCREMENT],
                vec![0xF4, 0x01, 0x18, 0xFC, 0xDC, 0x05],
            ),
            // read_temp
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::OUT_TEMP_L_XM.addr() | AUTO_INCREMENT],
                vec![0xC8, 0x00],
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let data = driver.read_all().await.unwrap();

        let epsilon = 0.001;

        // Gyro: default scale ±245 dps, sensitivity 8.75 mdps/LSB, no bias
        // X: 1000 * 0.00875 = 8.75, Y: -500 * 0.00875 = -4.375, Z: 250 * 0.00875 = 2.1875
        let (gx, gy, gz) = data.gyro;
        assert!((gx.as_f32() - 8.75).abs() < epsilon, "gx={}", gx.as_f32());
        assert!(
            (gy.as_f32() - (-4.375)).abs() < epsilon,
            "gy={}",
            gy.as_f32()
        );
        assert!(
            (gz.as_f32() - 2.1875).abs() < epsilon,
            "gz={}",
            gz.as_f32()
        );

        // Accel: default scale ±2g, sensitivity 0.061 mg/LSB, no bias
        // X: 100 * 0.000061 = 0.0061, Y: 200 * 0.000061 = 0.0122, Z: 300 * 0.000061 = 0.0183
        let (ax, ay, az) = data.accel;
        assert!(
            (ax.as_f32() - 0.0061).abs() < epsilon,
            "ax={}",
            ax.as_f32()
        );
        assert!(
            (ay.as_f32() - 0.0122).abs() < epsilon,
            "ay={}",
            ay.as_f32()
        );
        assert!(
            (az.as_f32() - 0.0183).abs() < epsilon,
            "az={}",
            az.as_f32()
        );

        // Mag: default scale ±4 gauss (datasheet Table 3), sensitivity 0.16 mgauss/LSB
        // X: 500 * 0.00016 = 0.08, Y: -1000 * 0.00016 = -0.16, Z: 1500 * 0.00016 = 0.24
        let (mx, my, mz) = data.mag;
        assert!(
            (mx.as_f32() - 0.08).abs() < epsilon,
            "mx={}",
            mx.as_f32()
        );
        assert!(
            (my.as_f32() - (-0.16)).abs() < epsilon,
            "my={}",
            my.as_f32()
        );
        assert!(
            (mz.as_f32() - 0.24).abs() < epsilon,
            "mz={}",
            mz.as_f32()
        );

        // Temp: 200/8.0 + 25.0 = 50.0
        assert!(
            (data.temp.as_f32() - 50.0).abs() < epsilon,
            "temp={}",
            data.temp.as_f32()
        );

        driver.release().release().done();
    }

    // =========================================================================
    // SPI TESTS
    // =========================================================================

    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    // SPI read bit and MS bit constants (from interface/spi.rs)
    const SPI_READ: u8 = 0x80;
    const MS_BIT: u8 = 0x40;

    #[tokio::test]
    async fn test_spi_read_gyro_raw() {
        // For gyro SPI read: transaction with Write([SPI_READ | MS_BIT | addr]) then Read(buffer)
        let gyro_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![SPI_READ | MS_BIT | GyroRegisters::OUT_X_L_G.addr()]),
            // Response: X=0x0102, Y=0x0304, Z=0x0506 (little-endian)
            SpiTransaction::read_vec(vec![0x02, 0x01, 0x04, 0x03, 0x06, 0x05]),
            SpiTransaction::transaction_end(),
        ];

        // XM device won't be used for gyro read, but we need to provide it
        let xm_expectations: [SpiTransaction<u8>; 0] = [];

        let gyro_spi = SpiMock::new(&gyro_expectations);
        let xm_spi = SpiMock::new(&xm_expectations);
        let interface = SpiInterface::init(gyro_spi, xm_spi);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_gyro_raw().await.unwrap();

        assert_eq!(x, 0x0102);
        assert_eq!(y, 0x0304);
        assert_eq!(z, 0x0506);

        let (mut gyro_spi, mut xm_spi) = driver.release().release();
        gyro_spi.done();
        xm_spi.done();
    }

    #[tokio::test]
    async fn test_spi_read_accel_raw() {
        // For accel/mag SPI read: transaction with Write([SPI_READ | MS_BIT | addr]) then Read(buffer)
        let gyro_expectations: [SpiTransaction<u8>; 0] = [];

        let xm_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![
                SPI_READ | MS_BIT | AccelMagRegisters::OUT_X_L_A.addr(),
            ]),
            // Response: X=100, Y=200, Z=300 (little-endian)
            SpiTransaction::read_vec(vec![0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01]),
            SpiTransaction::transaction_end(),
        ];

        let gyro_spi = SpiMock::new(&gyro_expectations);
        let xm_spi = SpiMock::new(&xm_expectations);
        let interface = SpiInterface::init(gyro_spi, xm_spi);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_accel_raw().await.unwrap();

        assert_eq!(x, 100);
        assert_eq!(y, 200);
        assert_eq!(z, 300);

        let (mut gyro_spi, mut xm_spi) = driver.release().release();
        gyro_spi.done();
        xm_spi.done();
    }

    #[tokio::test]
    async fn test_spi_read_mag_raw() {
        let gyro_expectations: [SpiTransaction<u8>; 0] = [];

        let xm_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![
                SPI_READ | MS_BIT | AccelMagRegisters::OUT_X_L_M.addr(),
            ]),
            // Response: X=1000, Y=2000, Z=3000 (little-endian)
            SpiTransaction::read_vec(vec![0xE8, 0x03, 0xD0, 0x07, 0xB8, 0x0B]),
            SpiTransaction::transaction_end(),
        ];

        let gyro_spi = SpiMock::new(&gyro_expectations);
        let xm_spi = SpiMock::new(&xm_expectations);
        let interface = SpiInterface::init(gyro_spi, xm_spi);
        let mut driver = Lsm9ds0::new(interface);

        let (x, y, z) = driver.read_mag_raw().await.unwrap();

        assert_eq!(x, 1000);
        assert_eq!(y, 2000);
        assert_eq!(z, 3000);

        let (mut gyro_spi, mut xm_spi) = driver.release().release();
        gyro_spi.done();
        xm_spi.done();
    }

    // =========================================================================
    // MULTI-BYTE WRITE TESTS
    // =========================================================================

    #[tokio::test]
    async fn test_i2c_multi_byte_write() {
        const AUTO_INCREMENT: u8 = 0x80;

        // Test writing 9 bytes to consecutive registers
        let test_addr = 0x20; // CTRL_REG1_G
        let test_data: [u8; 9] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];

        // Expected I2C write: [register_addr, data...]
        let mut expected_write = vec![test_addr | AUTO_INCREMENT];
        expected_write.extend_from_slice(&test_data);

        let expectations = [I2cTransaction::write(GYRO_ADDR, expected_write)];

        let i2c = I2cMock::new(&expectations);
        let mut interface = I2cInterface::init(i2c);

        interface.write_gyro(test_addr, &test_data).await.unwrap();

        interface.release().done();
    }

    // =========================================================================
    // VERIFY_CONFIG TESTS
    // =========================================================================

    /// Build the I2C read expectations for verify_config() with default config.
    /// All default shadow registers are 0x00 except CTRL_REG5_XM which defaults to 0x18.
    fn verify_config_expectations(config: &Lsm9ds0Config) -> std::vec::Vec<I2cTransaction> {
        const AUTO_INCREMENT: u8 = 0x80;

        let ctrl_reg5_xm_val: u8 = config.ctrl_reg5_xm.into();

        vec![
            // 1. Gyro CTRL_REG1_G through CTRL_REG5_G (5 bytes)
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::CTRL_REG1_G.addr() | AUTO_INCREMENT],
                vec![
                    config.ctrl_reg1_g.into(),
                    config.ctrl_reg2_g.into(),
                    config.ctrl_reg3_g.into(),
                    config.ctrl_reg4_g.into(),
                    config.ctrl_reg5_g.into(),
                ],
            ),
            // 2. Gyro FIFO_CTRL_REG_G (1 byte)
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::FIFO_CTRL_REG_G.addr()],
                vec![config.fifo_ctrl_reg_g.into()],
            ),
            // 3. Gyro INT1_CFG_G (1 byte)
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::INT1_CFG_G.addr()],
                vec![config.int1_cfg_g.into()],
            ),
            // 4. Gyro INT1_THS_XH through INT1_DURATION (7 bytes)
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::INT1_THS_XH_G.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, config.int1_duration_g.into()],
            ),
            // 5. XM INT_CTRL_REG_M (1 byte)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_CTRL_REG_M.addr()],
                vec![config.int_ctrl_reg_m.into()],
            ),
            // 6. XM INT_THS_L_M, INT_THS_H_M (2 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_THS_L_M.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00],
            ),
            // 7. XM OFFSET_X_L_M through OFFSET_Z_H_M (6 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::OFFSET_X_L_M.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            ),
            // 8. XM CTRL_REG0_XM through CTRL_REG7_XM (8 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::CTRL_REG0_XM.addr() | AUTO_INCREMENT],
                vec![
                    config.ctrl_reg0_xm.into(),
                    config.ctrl_reg1_xm.into(),
                    config.ctrl_reg2_xm.into(),
                    config.ctrl_reg3_xm.into(),
                    config.ctrl_reg4_xm.into(),
                    ctrl_reg5_xm_val,
                    config.ctrl_reg6_xm.into(),
                    config.ctrl_reg7_xm.into(),
                ],
            ),
            // 9. XM FIFO_CTRL_REG (1 byte)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::FIFO_CTRL_REG.addr()],
                vec![config.fifo_ctrl_reg.into()],
            ),
            // 10. XM INT_GEN_1_REG (1 byte)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_GEN_1_REG.addr()],
                vec![config.int_gen_1_reg.into()],
            ),
            // 11. XM INT_GEN_1_THS, INT_GEN_1_DURATION (2 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_GEN_1_THS.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00],
            ),
            // 12. XM INT_GEN_2_REG (1 byte)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_GEN_2_REG.addr()],
                vec![config.int_gen_2_reg.into()],
            ),
            // 13. XM INT_GEN_2_THS, INT_GEN_2_DURATION (2 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_GEN_2_THS.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00],
            ),
            // 14. XM CLICK_CFG (1 byte)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::CLICK_CFG.addr()],
                vec![config.click_cfg.into()],
            ),
            // 15. XM CLICK_THS through TIME_WINDOW (4 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::CLICK_THS.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00, 0x00, 0x00],
            ),
            // 16. XM ACT_THS, ACT_DUR (2 bytes)
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::ACT_THS.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00],
            ),
        ]
    }

    #[tokio::test]
    async fn test_verify_config_pass() {
        let config = Lsm9ds0Config::default();
        let expectations = verify_config_expectations(&config);

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let result = driver.verify_config().await;
        assert!(result.is_ok(), "verify_config should pass: {:?}", result);

        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_verify_config_detects_gyro_mismatch() {
        let config = Lsm9ds0Config::default();
        const AUTO_INCREMENT: u8 = 0x80;

        // Return corrupted CTRL_REG3_G (offset 2 from CTRL_REG1_G base 0x20 → register 0x22)
        let corrupted_reg3_g: u8 = 0xFF;
        let expectations = vec![
            // First read: Gyro CTRL_REG1-5 — CTRL_REG3_G is corrupted
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::CTRL_REG1_G.addr() | AUTO_INCREMENT],
                vec![
                    config.ctrl_reg1_g.into(),
                    config.ctrl_reg2_g.into(),
                    corrupted_reg3_g, // CTRL_REG3_G corrupted
                    config.ctrl_reg4_g.into(),
                    config.ctrl_reg5_g.into(),
                ],
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let result = driver.verify_config().await;
        match result {
            Err(Error::ConfigMismatch {
                register,
                expected,
                actual,
            }) => {
                assert_eq!(register, GyroRegisters::CTRL_REG3_G.addr());
                assert_eq!(expected, config.ctrl_reg3_g.into());
                assert_eq!(actual, corrupted_reg3_g);
            }
            other => panic!("expected ConfigMismatch, got {:?}", other),
        }

        driver.release().release().done();
    }

    #[tokio::test]
    async fn test_verify_config_detects_xm_mismatch() {
        let config = Lsm9ds0Config::default();
        const AUTO_INCREMENT: u8 = 0x80;

        // All gyro reads pass, but XM CTRL_REG1_XM (offset 1 from 0x1F → register 0x20)
        // returns a corrupted value
        let corrupted_ctrl_reg1_xm: u8 = 0xAB;

        let expectations = vec![
            // Gyro reads all pass
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::CTRL_REG1_G.addr() | AUTO_INCREMENT],
                vec![
                    config.ctrl_reg1_g.into(),
                    config.ctrl_reg2_g.into(),
                    config.ctrl_reg3_g.into(),
                    config.ctrl_reg4_g.into(),
                    config.ctrl_reg5_g.into(),
                ],
            ),
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::FIFO_CTRL_REG_G.addr()],
                vec![config.fifo_ctrl_reg_g.into()],
            ),
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::INT1_CFG_G.addr()],
                vec![config.int1_cfg_g.into()],
            ),
            I2cTransaction::write_read(
                GYRO_ADDR,
                vec![GyroRegisters::INT1_THS_XH_G.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, config.int1_duration_g.into()],
            ),
            // XM reads - first few pass
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_CTRL_REG_M.addr()],
                vec![config.int_ctrl_reg_m.into()],
            ),
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::INT_THS_L_M.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00],
            ),
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::OFFSET_X_L_M.addr() | AUTO_INCREMENT],
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            ),
            // CTRL_REG0-7_XM — CTRL_REG1_XM (index 1) is corrupted
            I2cTransaction::write_read(
                XM_ADDR,
                vec![AccelMagRegisters::CTRL_REG0_XM.addr() | AUTO_INCREMENT],
                vec![
                    config.ctrl_reg0_xm.into(),
                    corrupted_ctrl_reg1_xm, // CTRL_REG1_XM corrupted
                    config.ctrl_reg2_xm.into(),
                    config.ctrl_reg3_xm.into(),
                    config.ctrl_reg4_xm.into(),
                    config.ctrl_reg5_xm.into(),
                    config.ctrl_reg6_xm.into(),
                    config.ctrl_reg7_xm.into(),
                ],
            ),
        ];

        let i2c = I2cMock::new(&expectations);
        let interface = I2cInterface::init(i2c);
        let mut driver = Lsm9ds0::new(interface);

        let result = driver.verify_config().await;
        match result {
            Err(Error::ConfigMismatch {
                register,
                expected,
                actual,
            }) => {
                // CTRL_REG1_XM is at address 0x20
                assert_eq!(register, AccelMagRegisters::CTRL_REG1_XM.addr());
                assert_eq!(expected, config.ctrl_reg1_xm.into());
                assert_eq!(actual, corrupted_ctrl_reg1_xm);
            }
            other => panic!("expected ConfigMismatch, got {:?}", other),
        }

        driver.release().release().done();
    }

    #[tokio::test]
    #[should_panic(expected = "exceeds MAX_WRITE_LEN")]
    #[cfg(debug_assertions)]
    async fn test_i2c_write_exceeds_max_length_panics_in_debug() {
        // Test that writing more than MAX_WRITE_LEN bytes panics in debug builds
        let test_addr = 0x20;
        let test_data: [u8; 20] = [0u8; 20]; // 20 bytes > MAX_WRITE_LEN (15)

        // No expectations needed - we expect a panic before the I2C transaction
        let expectations: [I2cTransaction; 0] = [];

        let i2c = I2cMock::new(&expectations);
        let mut interface = I2cInterface::init(i2c);

        // This should panic in debug builds
        let _ = interface.write_gyro(test_addr, &test_data).await;
    }
}
