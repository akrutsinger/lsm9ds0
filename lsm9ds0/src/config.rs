//! Configuration for the LSM9DS0 Accelerometer/Magnetometer/Gyroscope sensors
//!
//! This module provides a shadow register-based configuration for the LSM9DS0 sensor. The config
//! maintains local copies of hardware register values, eliminating the need for read-modify-write
//! operations when changing settings.
//!
//! Use [`Lsm9ds0Config::new()`] with the builder pattern to configure sensor settings before
//! initialization. See the [crate-level documentation](crate) for complete usage examples.

use crate::registers::*;

/// Temperature sensor scale factor: 8 LSB/°C from datasheet Table 4
pub(crate) const TEMP_SCALE: f32 = 8.0;

/// Default temperature bias/offset in °C
///
/// **Important**: The LSM9DS0 datasheet does NOT specify an absolute temperature reference point.
/// The temperature sensor outputs relative temperature changes from an unspecified baseline.
/// This default value of 25°C is a reasonable assumption for typical room temperature conditions,
/// but for accurate absolute temperature readings, you should:
///
/// 1. Calibrate against a known reference thermometer
/// 2. Use [`Lsm9ds0Config::with_temperature_offset`] to set a custom offset
///
/// The temperature reading is calculated as: `raw_value / 8.0 + offset`
pub const DEFAULT_TEMP_BIAS: f32 = 25.0;

/// LSM9DS0 configuration with shadow registers
///
/// This struct maintains local copies of hardware register values, allowing configuration changes
/// without reading from the device first.
#[derive(Debug, Clone)]
pub struct Lsm9ds0Config {
    // === Gyro shadow registers ===
    pub(crate) ctrl_reg1_g: CtrlReg1G,
    pub(crate) ctrl_reg2_g: CtrlReg2G,
    pub(crate) ctrl_reg3_g: CtrlReg3G,
    pub(crate) ctrl_reg4_g: CtrlReg4G,
    pub(crate) ctrl_reg5_g: CtrlReg5G,
    pub(crate) fifo_ctrl_reg_g: FifoCtrlRegG,
    pub(crate) int1_cfg_g: Int1CfgG,
    pub(crate) int1_duration_g: Int1DurationG,

    // === Gyro multi-byte values (per-axis) ===
    pub(crate) gyro_int_ths_x: u16, // 15-bit
    pub(crate) gyro_int_ths_y: u16,
    pub(crate) gyro_int_ths_z: u16,

    // === XM shadow registers ===
    pub(crate) ctrl_reg0_xm: CtrlReg0Xm,
    pub(crate) ctrl_reg1_xm: CtrlReg1Xm,
    pub(crate) ctrl_reg2_xm: CtrlReg2Xm,
    pub(crate) ctrl_reg3_xm: CtrlReg3Xm,
    pub(crate) ctrl_reg4_xm: CtrlReg4Xm,
    pub(crate) ctrl_reg5_xm: CtrlReg5Xm,
    pub(crate) ctrl_reg6_xm: CtrlReg6Xm,
    pub(crate) ctrl_reg7_xm: CtrlReg7Xm,
    pub(crate) fifo_ctrl_reg: FifoCtrlReg,
    pub(crate) int_gen_1_reg: IntGenReg,
    pub(crate) int_gen_1_ths: u8,      // 7-bit
    pub(crate) int_gen_1_duration: u8, // 7-bit
    pub(crate) int_gen_2_reg: IntGenReg,
    pub(crate) int_gen_2_ths: u8,
    pub(crate) int_gen_2_duration: u8,
    pub(crate) int_ctrl_reg_m: IntCtrlRegM,
    pub(crate) click_cfg: ClickCfg,
    pub(crate) click_ths: u8, // 7-bit
    pub(crate) time_limit_ms: u8,
    pub(crate) time_latency_ms: u8,
    pub(crate) time_window_ms: u8,
    pub(crate) act_ths: u8, // 7-bit
    pub(crate) act_dur: u8,

    // === XM multi-byte values ===
    pub(crate) mag_int_ths: u16, // 15-bit
    pub(crate) mag_offset_x: i16,
    pub(crate) mag_offset_y: i16,
    pub(crate) mag_offset_z: i16,

    // Useful for gyro sleep mode where all axes are disabled.
    pub(crate) gyro_axes_enabled: (bool, bool, bool),

    // === Calibration ===
    pub(crate) temp_offset: f32,
    pub(crate) gyro_bias: (f32, f32, f32),
    pub(crate) accel_bias: (f32, f32, f32),
    pub(crate) auto_calibration: Option<crate::types::Orientation>,
}

impl Default for Lsm9ds0Config {
    /// Create a configuration with datasheet power-on-reset default values
    fn default() -> Self {
        Self {
            // Gyro defaults per datasheet
            // CTRL_REG1_G default: 0x07 (PD=0, all axes enabled)
            ctrl_reg1_g: CtrlReg1G::new()
                .with_dr(GyroDataRate::Hz95)
                .with_bw(GyroBandwidth::Bw0)
                .with_pd(PowerMode::PowerDown) // Power-down by default per datasheet
                .with_zen(Enable::Enabled)
                .with_yen(Enable::Enabled)
                .with_xen(Enable::Enabled),

            // CTRL_REG2_G default: 0x00
            ctrl_reg2_g: CtrlReg2G::new()
                .with_hpm(GyroHpfMode::NormalReset)
                .with_hpcf(GyroHpfCutoff::Cutoff0),

            // CTRL_REG3_G default: 0x00
            ctrl_reg3_g: CtrlReg3G::new(),

            // CTRL_REG4_G default: 0x00
            ctrl_reg4_g: CtrlReg4G::new()
                .with_fs(GyroScale::Dps245)
                .with_bdu(BlockDataUpdate::Continuous)
                .with_ble(Endianness::Little)
                .with_st(GyroSelfTest::Disabled)
                .with_sim(SpiMode::FourWire),

            // CTRL_REG5_G default: 0x00
            ctrl_reg5_g: CtrlReg5G::new(),

            // FIFO_CTRL_REG_G default: 0x00
            fifo_ctrl_reg_g: FifoCtrlRegG::new().with_fm(FifoMode::Bypass).with_wtm(0),

            // INT1_CFG_G default: 0x00
            int1_cfg_g: Int1CfgG::new(),

            // INT1_DURATION_G default: 0x00
            int1_duration_g: Int1DurationG::new(),

            // Gyro interrupt thresholds default: 0x00
            gyro_int_ths_x: 0,
            gyro_int_ths_y: 0,
            gyro_int_ths_z: 0,

            // XM defaults per datasheet
            // CTRL_REG0_XM default: 0x00
            ctrl_reg0_xm: CtrlReg0Xm::new(),

            // CTRL_REG1_XM default: 0x07 (all axes enabled, power-down)
            ctrl_reg1_xm: CtrlReg1Xm::new()
                .with_aodr(AccelDataRate::PowerDown)
                .with_bdu(BlockDataUpdate::Continuous)
                .with_azen(Enable::Enabled)
                .with_ayen(Enable::Enabled)
                .with_axen(Enable::Enabled),

            // CTRL_REG2_XM default: 0x00
            ctrl_reg2_xm: CtrlReg2Xm::new()
                .with_abw(AccelBandwidth::Hz773)
                .with_afs(AccelScale::G2)
                .with_ast(AccelSelfTest::Normal)
                .with_sim(SpiMode::FourWire),

            // CTRL_REG3_XM default: 0x00
            ctrl_reg3_xm: CtrlReg3Xm::new(),

            // CTRL_REG4_XM default: 0x00
            ctrl_reg4_xm: CtrlReg4Xm::new(),

            // CTRL_REG5_XM default: 0x18 (M_RES=0, M_ODR=0b110 but that's reserved, TEMP_EN=0)
            // Using reasonable defaults instead of reserved values
            ctrl_reg5_xm: CtrlReg5Xm::new()
                .with_temp_en(Enable::Disabled)
                .with_m_res(MagResolution::Low)
                .with_m_odr(MagDataRate::Hz50)
                .with_lir1(LatchInterrupt::NotLatched)
                .with_lir2(LatchInterrupt::NotLatched),

            // CTRL_REG6_XM default: 0x20 (MFS=01 = ±4 gauss)
            ctrl_reg6_xm: CtrlReg6Xm::new().with_mfs(MagScale::Gauss4),

            // CTRL_REG7_XM default: 0x03
            //
            // NOTE: Datasheet (pg 60/74 DocID024763 Rev 2) says the default value should be 0x02,
            // but directly reading this register from a fresh power-on shows a default value of
            // 0x03, so this initial default configuration reflects 0x03.
            ctrl_reg7_xm: CtrlReg7Xm::new()
                .with_md(MagMode::PowerDown)
                .with_mlp(MagLowPower::Normal)
                .with_afds(Enable::Disabled)
                .with_ahpm(AccelHpfMode::NormalReset),

            // FIFO_CTRL_REG default: 0x00
            fifo_ctrl_reg: FifoCtrlReg::new().with_fm(FifoMode::Bypass).with_fth(0),

            // Interrupt generator defaults: 0x00
            int_gen_1_reg: IntGenReg::new(),
            int_gen_1_ths: 0,
            int_gen_1_duration: 0,
            int_gen_2_reg: IntGenReg::new(),
            int_gen_2_ths: 0,
            int_gen_2_duration: 0,

            // INT_CTRL_REG_M default: 0xE8
            //
            // NOTE: Datasheet (pg 53/74 DocID024763 Rev 2) says the default value should be 0x00,
            // but directly reading this register from a fresh power-on shows a default value of
            // 0xE8, so this initial default configuration reflects 0xE8.
            int_ctrl_reg_m: IntCtrlRegM::new()
                .with_iea(ActiveLevelInverted::ActiveHigh)
                .with_zmien(Enable::Enabled)
                .with_ymien(Enable::Enabled)
                .with_xmien(Enable::Enabled),

            // Click defaults: 0x00
            click_cfg: ClickCfg::new(),
            click_ths: 0,
            time_limit_ms: 0,
            time_latency_ms: 0,
            time_window_ms: 0,

            // Activity defaults: 0x00
            act_ths: 0,
            act_dur: 0,

            // Multi-byte value defaults
            mag_int_ths: 0,
            mag_offset_x: 0,
            mag_offset_y: 0,
            mag_offset_z: 0,

            // Temperature calibration
            temp_offset: DEFAULT_TEMP_BIAS,

            // Gyro sleep mode - default to all axes enabled
            gyro_axes_enabled: (true, true, true),

            // Bias calibration - default to no bias correction
            gyro_bias: (0.0, 0.0, 0.0),
            accel_bias: (0.0, 0.0, 0.0),
            auto_calibration: None,
        }
    }
}

impl Lsm9ds0Config {
    /// Create a new configuration with datasheet default values
    pub fn new() -> Self {
        Self::default()
    }
}

/// # Derived Values
///
/// Methods for retrieving calculated values based on current configuration settings.
impl Lsm9ds0Config {
    /// Get gyro sensitivity in mdps/LSB based on current scale setting
    pub fn gyro_sensitivity(&self) -> f32 {
        self.ctrl_reg4_g.fs().sensitivity()
    }

    /// Get accel sensitivity in mg/LSB based on current scale setting
    pub fn accel_sensitivity(&self) -> f32 {
        self.ctrl_reg2_xm.afs().sensitivity()
    }

    /// Get mag sensitivity in mgauss/LSB based on current scale setting
    pub fn mag_sensitivity(&self) -> f32 {
        self.ctrl_reg6_xm.mfs().sensitivity()
    }

    /// Get gyro HPF cutoff frequency in Hz (depends on both ODR and HPCF)
    pub fn gyro_hpf_cutoff_hz(&self) -> f32 {
        const CUTOFF_TABLE: [[f32; 10]; 4] = [
            // ODR = 95 Hz
            [7.2, 3.5, 1.8, 0.9, 0.45, 0.18, 0.09, 0.045, 0.018, 0.009],
            // ODR = 190 Hz
            [13.5, 7.2, 3.5, 1.8, 0.9, 0.45, 0.18, 0.09, 0.045, 0.018],
            // ODR = 380 Hz
            [27.0, 13.5, 7.2, 3.5, 1.8, 0.9, 0.45, 0.18, 0.09, 0.045],
            // ODR = 760 Hz
            [51.4, 27.0, 13.5, 7.2, 3.5, 1.8, 0.9, 0.45, 0.18, 0.09],
        ];

        let odr_idx = self.ctrl_reg1_g.dr() as usize;
        let cutoff_idx = self.ctrl_reg2_g.hpcf() as usize;
        CUTOFF_TABLE[odr_idx][cutoff_idx.min(9)]
    }

    /// Get gyro LPF (low-pass filter) cutoff frequency in Hz
    ///
    /// The cutoff depends on both ODR and bandwidth setting (see datasheet Table 21).
    pub fn gyro_lpf_cutoff_hz(&self) -> f32 {
        const CUTOFF_TABLE: [[f32; 4]; 4] = [
            // ODR = 95 Hz:  BW0=12.5, BW1=25, BW2=25, BW3=25
            [12.5, 25.0, 25.0, 25.0],
            // ODR = 190 Hz: BW0=12.5, BW1=25, BW2=50, BW3=70
            [12.5, 25.0, 50.0, 70.0],
            // ODR = 380 Hz: BW0=20, BW1=25, BW2=50, BW3=100
            [20.0, 25.0, 50.0, 100.0],
            // ODR = 760 Hz: BW0=30, BW1=35, BW2=50, BW3=100
            [30.0, 35.0, 50.0, 100.0],
        ];

        let odr_idx = self.ctrl_reg1_g.dr() as usize;
        let bw_idx = self.ctrl_reg1_g.bw() as usize;
        CUTOFF_TABLE[odr_idx][bw_idx]
    }

    /// Get the configured temperature offset in °C
    ///
    /// This offset is added to the raw temperature reading to produce absolute temperature.
    /// See [`DEFAULT_TEMP_BIAS`] for calibration details.
    pub fn temperature_offset(&self) -> f32 {
        self.temp_offset
    }

    /// Get current gyro scale setting
    pub fn gyro_scale(&self) -> GyroScale {
        self.ctrl_reg4_g.fs()
    }

    /// Get current gyro data rate setting
    pub fn gyro_data_rate(&self) -> GyroDataRate {
        self.ctrl_reg1_g.dr()
    }

    /// Get current accel scale setting
    pub fn accel_scale(&self) -> AccelScale {
        self.ctrl_reg2_xm.afs()
    }

    /// Get current accel data rate setting
    pub fn accel_data_rate(&self) -> AccelDataRate {
        self.ctrl_reg1_xm.aodr()
    }

    /// Get current mag scale setting
    pub fn mag_scale(&self) -> MagScale {
        self.ctrl_reg6_xm.mfs()
    }

    /// Get current mag data rate setting
    pub fn mag_data_rate(&self) -> MagDataRate {
        self.ctrl_reg5_xm.m_odr()
    }
}

/// # Gyroscope Configuration
///
/// Methods for configuring the gyroscope sensor including scale, data rate, filtering, FIFO, and
/// interrupt settings.
impl Lsm9ds0Config {
    /// Enable or disable the gyroscope (sets power mode)
    ///
    /// When enabled, uses Normal mode with configured axes.
    /// When disabled, uses Power-down mode.
    /// For Sleep mode (faster wake-up), use [`with_gyro_power_mode`](Self::with_gyro_power_mode).
    pub fn with_gyro_enabled(mut self, enabled: bool) -> Self {
        if enabled {
            self.ctrl_reg1_g.set_pd(PowerMode::Normal);
            // Restore saved axis states
            let (x, y, z) = self.gyro_axes_enabled;
            self.ctrl_reg1_g
                .set_xen(if x { Enable::Enabled } else { Enable::Disabled });
            self.ctrl_reg1_g
                .set_yen(if y { Enable::Enabled } else { Enable::Disabled });
            self.ctrl_reg1_g
                .set_zen(if z { Enable::Enabled } else { Enable::Disabled });
        } else {
            self.ctrl_reg1_g.set_pd(PowerMode::PowerDown);
        }
        self
    }

    /// Set gyroscope power mode with sleep support
    ///
    /// The gyroscope supports three power modes (see datasheet Table 22):
    /// - **PowerDown**: Lowest power consumption, sensor completely off
    /// - **Sleep**: Lower power than normal, but faster wake-up than power-down.
    ///   PD=1 with all axes disabled. Useful for motion-triggered wake-up scenarios.
    /// - **Normal**: Full operation with configured axes enabled
    ///
    /// # Example
    /// ```
    /// use lsm9ds0::{Lsm9ds0Config, GyroPowerMode, GyroDataRate};
    ///
    /// // Configure for sleep mode with fast wake-up
    /// let config = Lsm9ds0Config::new()
    ///     .with_gyro_data_rate(GyroDataRate::Hz190)
    ///     .with_gyro_power_mode(GyroPowerMode::Sleep);
    /// ```
    pub fn with_gyro_power_mode(mut self, mode: GyroPowerMode) -> Self {
        match mode {
            GyroPowerMode::PowerDown => {
                self.ctrl_reg1_g.set_pd(PowerMode::PowerDown);
            }
            GyroPowerMode::Sleep => {
                // Sleep mode: PD=1, all axes disabled
                self.ctrl_reg1_g.set_pd(PowerMode::Normal);
                self.ctrl_reg1_g.set_xen(Enable::Disabled);
                self.ctrl_reg1_g.set_yen(Enable::Disabled);
                self.ctrl_reg1_g.set_zen(Enable::Disabled);
            }
            GyroPowerMode::Normal => {
                self.ctrl_reg1_g.set_pd(PowerMode::Normal);
                // Restore saved axis states
                let (x, y, z) = self.gyro_axes_enabled;
                self.ctrl_reg1_g
                    .set_xen(if x { Enable::Enabled } else { Enable::Disabled });
                self.ctrl_reg1_g
                    .set_yen(if y { Enable::Enabled } else { Enable::Disabled });
                self.ctrl_reg1_g
                    .set_zen(if z { Enable::Enabled } else { Enable::Disabled });
            }
        }
        self
    }

    /// Set the gyroscope full-scale range
    pub fn with_gyro_scale(mut self, scale: GyroScale) -> Self {
        self.ctrl_reg4_g.set_fs(scale);
        self
    }

    /// Set the gyroscope output data rate
    pub fn with_gyro_data_rate(mut self, rate: GyroDataRate) -> Self {
        self.ctrl_reg1_g.set_dr(rate);
        self
    }

    /// Set the gyroscope bandwidth
    pub fn with_gyro_bandwidth(mut self, bw: GyroBandwidth) -> Self {
        self.ctrl_reg1_g.set_bw(bw);
        self
    }

    /// Enable or disable individual gyroscope axes
    ///
    /// The axis enable state is saved and restored when switching between power modes.
    pub fn with_gyro_axes(mut self, x: bool, y: bool, z: bool) -> Self {
        // Save for sleep/wake restoration
        self.gyro_axes_enabled = (x, y, z);

        self.ctrl_reg1_g
            .set_xen(if x { Enable::Enabled } else { Enable::Disabled });
        self.ctrl_reg1_g
            .set_yen(if y { Enable::Enabled } else { Enable::Disabled });
        self.ctrl_reg1_g
            .set_zen(if z { Enable::Enabled } else { Enable::Disabled });
        self
    }

    /// Configure gyroscope high-pass filter
    pub fn with_gyro_hpf(
        mut self,
        enabled: bool,
        mode: GyroHpfMode,
        cutoff: GyroHpfCutoff,
    ) -> Self {
        self.ctrl_reg5_g.set_hpen(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg2_g.set_hpm(mode);
        self.ctrl_reg2_g.set_hpcf(cutoff);
        self
    }

    /// Configure gyroscope FIFO
    pub fn with_gyro_fifo(mut self, enabled: bool, mode: FifoMode, watermark: u8) -> Self {
        self.ctrl_reg5_g.set_fifo_en(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.fifo_ctrl_reg_g.set_fm(mode);
        self.fifo_ctrl_reg_g.set_wtm(watermark & 0x1F);
        self
    }

    /// Set gyroscope self-test mode
    pub fn with_gyro_self_test(mut self, mode: GyroSelfTest) -> Self {
        self.ctrl_reg4_g.set_st(mode);
        self
    }

    /// Set gyroscope block data update mode
    pub fn with_gyro_bdu(mut self, enabled: bool) -> Self {
        self.ctrl_reg4_g.set_bdu(if enabled {
            BlockDataUpdate::WaitForRead
        } else {
            BlockDataUpdate::Continuous
        });
        self
    }
}

/// # Gyroscope Interrupts
///
/// Methods for configuring gyroscope interrupt generation, thresholds, and pin routing.
impl Lsm9ds0Config {
    /// Enable gyroscope interrupt on INT_G pin
    pub fn with_gyro_int_enabled(mut self, enabled: bool) -> Self {
        self.ctrl_reg3_g.set_i1_int1(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Configure gyroscope interrupt polarity and pin mode
    pub fn with_gyro_int_pin_config(mut self, active_low: bool, open_drain: bool) -> Self {
        self.ctrl_reg3_g.set_h_lactive(if active_low {
            ActiveLevel::ActiveLow
        } else {
            ActiveLevel::ActiveHigh
        });
        self.ctrl_reg3_g.set_pp_od(if open_drain {
            OutputType::OpenDrain
        } else {
            OutputType::PushPull
        });
        self
    }

    /// Configure which gyroscope axes trigger interrupts
    pub fn with_gyro_int_axes(
        mut self,
        x_high: bool,
        x_low: bool,
        y_high: bool,
        y_low: bool,
        z_high: bool,
        z_low: bool,
    ) -> Self {
        self.int1_cfg_g.set_xhie(if x_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int1_cfg_g.set_xlie(if x_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int1_cfg_g.set_yhie(if y_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int1_cfg_g.set_ylie(if y_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int1_cfg_g.set_zhie(if z_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int1_cfg_g.set_zlie(if z_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set gyroscope interrupt combination mode and latch
    pub fn with_gyro_int_mode(mut self, and_combination: bool, latch: bool) -> Self {
        self.int1_cfg_g.set_and_or(if and_combination {
            InterruptCombination::And
        } else {
            InterruptCombination::Or
        });
        self.int1_cfg_g.set_lir(if latch {
            LatchInterrupt::Latched
        } else {
            LatchInterrupt::NotLatched
        });
        self
    }

    /// Set same gyroscope interrupt threshold for all axes (15-bit value, max 0x7FFF)
    pub fn with_gyro_int_threshold(mut self, threshold: u16) -> Self {
        debug_assert!(
            threshold <= 0x7FFF,
            "gyro interrupt threshold {threshold:#06X} exceeds 15-bit max (0x7FFF)"
        );
        let masked = threshold & 0x7FFF;
        self.gyro_int_ths_x = masked;
        self.gyro_int_ths_y = masked;
        self.gyro_int_ths_z = masked;
        self
    }

    /// Set per-axis gyroscope interrupt thresholds (15-bit values, max 0x7FFF each)
    pub fn with_gyro_int_threshold_xyz(mut self, x: u16, y: u16, z: u16) -> Self {
        debug_assert!(
            x <= 0x7FFF,
            "gyro X interrupt threshold {x:#06X} exceeds 15-bit max (0x7FFF)"
        );
        debug_assert!(
            y <= 0x7FFF,
            "gyro Y interrupt threshold {y:#06X} exceeds 15-bit max (0x7FFF)"
        );
        debug_assert!(
            z <= 0x7FFF,
            "gyro Z interrupt threshold {z:#06X} exceeds 15-bit max (0x7FFF)"
        );
        self.gyro_int_ths_x = x & 0x7FFF;
        self.gyro_int_ths_y = y & 0x7FFF;
        self.gyro_int_ths_z = z & 0x7FFF;
        self
    }

    /// Set gyroscope interrupt wait policy
    pub fn with_gyro_int_wait(mut self, wait: bool) -> Self {
        self.int1_duration_g.set_wait(wait);
        self
    }

    /// Set gyroscope interrupt duration (7-bit value, max 0x7F)
    pub fn with_gyro_int_duration(mut self, duration: u8) -> Self {
        debug_assert!(
            duration <= 0x7F,
            "gyro interrupt duration {duration:#04X} exceeds 7-bit max (0x7F)"
        );
        self.int1_duration_g.set_duration(duration & 0x7F);
        self
    }

    /// Simple gyroscope motion detection - triggers when any axis exceeds threshold
    ///
    /// Threshold is a 15-bit value (max 0x7FFF).
    pub fn with_gyro_motion_threshold(mut self, threshold: u16) -> Self {
        debug_assert!(
            threshold <= 0x7FFF,
            "gyro motion threshold {threshold:#06X} exceeds 15-bit max (0x7FFF)"
        );
        self.ctrl_reg3_g.set_i1_int1(Enable::Enabled);
        self.int1_cfg_g.set_and_or(InterruptCombination::Or); // OR combination
        self.int1_cfg_g.set_lir(LatchInterrupt::NotLatched);
        self.int1_cfg_g.set_xhie(Enable::Enabled);
        self.int1_cfg_g.set_xlie(Enable::Enabled);
        self.int1_cfg_g.set_yhie(Enable::Enabled);
        self.int1_cfg_g.set_ylie(Enable::Enabled);
        self.int1_cfg_g.set_zhie(Enable::Enabled);
        self.int1_cfg_g.set_zlie(Enable::Enabled);
        let masked = threshold & 0x7FFF;
        self.gyro_int_ths_x = masked;
        self.gyro_int_ths_y = masked;
        self.gyro_int_ths_z = masked;
        self.int1_duration_g = Int1DurationG::new();
        self
    }

    /// Configure gyroscope data-ready and FIFO interrupts on DRDY_G pin
    pub fn with_gyro_drdy_int(
        mut self,
        data_ready: bool,
        fifo_wtm: bool,
        fifo_overrun: bool,
        fifo_empty: bool,
    ) -> Self {
        self.ctrl_reg3_g.set_i2_drdy(if data_ready {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_g.set_i2_wtm(if fifo_wtm {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_g.set_i2_orun(if fifo_overrun {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_g.set_i2_empty(if fifo_empty {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }
}

/// # Accelerometer Configuration
///
/// Methods for configuring the accelerometer sensor including scale, data rate, filtering, and FIFO
/// settings.
impl Lsm9ds0Config {
    /// Set the accelerometer output data rate (also controls power mode)
    pub fn with_accel_data_rate(mut self, rate: AccelDataRate) -> Self {
        // If mag is at 100 Hz, accel must be > 50 Hz or power-down
        // (Datasheet Table 84, footnote 1)
        let mag_odr = self.ctrl_reg5_xm.m_odr();
        if mag_odr == MagDataRate::Hz100 {
            debug_assert!(
                matches!(
                    rate,
                    AccelDataRate::PowerDown
                        | AccelDataRate::Hz100
                        | AccelDataRate::Hz200
                        | AccelDataRate::Hz400
                        | AccelDataRate::Hz800
                        | AccelDataRate::Hz1600
                ),
                "AccelDataRate must be > 50 Hz or PowerDown when MagDataRate is Hz100, got {:?}",
                rate
            );
        }
        self.ctrl_reg1_xm.set_aodr(rate);
        self
    }

    /// Set the accelerometer full-scale range
    pub fn with_accel_scale(mut self, scale: AccelScale) -> Self {
        self.ctrl_reg2_xm.set_afs(scale);
        self
    }

    /// Set the accelerometer anti-alias filter bandwidth
    pub fn with_accel_bandwidth(mut self, bw: AccelBandwidth) -> Self {
        self.ctrl_reg2_xm.set_abw(bw);
        self
    }

    /// Enable or disable individual accelerometer axes
    pub fn with_accel_axes(mut self, x: bool, y: bool, z: bool) -> Self {
        self.ctrl_reg1_xm
            .set_axen(if x { Enable::Enabled } else { Enable::Disabled });
        self.ctrl_reg1_xm
            .set_ayen(if y { Enable::Enabled } else { Enable::Disabled });
        self.ctrl_reg1_xm
            .set_azen(if z { Enable::Enabled } else { Enable::Disabled });
        self
    }

    /// Set accelerometer block data update mode
    pub fn with_accel_bdu(mut self, enabled: bool) -> Self {
        self.ctrl_reg1_xm.set_bdu(if enabled {
            BlockDataUpdate::WaitForRead
        } else {
            BlockDataUpdate::Continuous
        });
        self
    }

    /// Set accelerometer self-test mode
    pub fn with_accel_self_test(mut self, mode: AccelSelfTest) -> Self {
        self.ctrl_reg2_xm.set_ast(mode);
        self
    }

    /// Configure accelerometer high-pass filter
    pub fn with_accel_hpf(mut self, enabled: bool, mode: AccelHpfMode) -> Self {
        self.ctrl_reg7_xm.set_afds(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg7_xm.set_ahpm(mode);
        self
    }

    /// Configure accelerometer HPF for click/interrupt functions
    pub fn with_accel_hpf_functions(
        mut self,
        for_click: bool,
        for_int1: bool,
        for_int2: bool,
    ) -> Self {
        self.ctrl_reg0_xm.set_hp_click(if for_click {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg0_xm.set_hpis1(if for_int1 {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg0_xm.set_hpis2(if for_int2 {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Configure accelerometer FIFO
    pub fn with_accel_fifo(mut self, enabled: bool, mode: FifoMode, watermark: u8) -> Self {
        self.ctrl_reg0_xm.set_fifo_en(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.fifo_ctrl_reg.set_fm(mode);
        self.fifo_ctrl_reg.set_fth(watermark & 0x1F);
        self
    }
}

/// # Accelerometer Interrupts
///
/// Methods for configuring accelerometer interrupt generation, thresholds, click/tap detection,
/// activity detection, and pin routing.
impl Lsm9ds0Config {
    /// Configure accelerometer interrupt generator 1
    pub fn with_accel_int1_axes(
        mut self,
        x_high: bool,
        x_low: bool,
        y_high: bool,
        y_low: bool,
        z_high: bool,
        z_low: bool,
    ) -> Self {
        self.int_gen_1_reg.set_xhie(if x_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_1_reg.set_xlie(if x_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_1_reg.set_yhie(if y_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_1_reg.set_ylie(if y_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_1_reg.set_zhie(if z_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_1_reg.set_zlie(if z_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set accelerometer interrupt 1 mode (AND/OR combination, 6D detection)
    pub fn with_accel_int1_mode(mut self, and_combination: bool, six_d: bool) -> Self {
        self.int_gen_1_reg.set_aoi(if and_combination {
            InterruptCombination::And
        } else {
            InterruptCombination::Or
        });
        self.int_gen_1_reg.set_six_d(if six_d {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set accelerometer interrupt 1 threshold (7-bit value, max 0x7F)
    pub fn with_accel_int1_threshold(mut self, threshold: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "accel int1 threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.int_gen_1_ths = threshold & 0x7F;
        self
    }

    /// Set accelerometer interrupt 1 duration (7-bit value, max 0x7F)
    pub fn with_accel_int1_duration(mut self, duration: u8) -> Self {
        debug_assert!(
            duration <= 0x7F,
            "accel int1 duration {duration:#04X} exceeds 7-bit max (0x7F)"
        );
        self.int_gen_1_duration = duration & 0x7F;
        self
    }

    /// Set accelerometer interrupt 1 latch mode
    pub fn with_accel_int1_latch(mut self, latch: bool) -> Self {
        self.ctrl_reg5_xm.set_lir1(if latch {
            LatchInterrupt::Latched
        } else {
            LatchInterrupt::NotLatched
        });
        self
    }

    /// Simple accelerometer motion detection using interrupt 1
    ///
    /// Threshold is a 7-bit value (max 0x7F).
    pub fn with_accel_motion_threshold(mut self, threshold: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "accel motion threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.ctrl_reg3_xm.set_p1_int1(Enable::Enabled);
        self.int_gen_1_reg.set_aoi(InterruptCombination::Or); // OR combination
        self.int_gen_1_reg.set_six_d(Enable::Disabled);
        self.int_gen_1_reg.set_xhie(Enable::Enabled);
        self.int_gen_1_reg.set_xlie(Enable::Enabled);
        self.int_gen_1_reg.set_yhie(Enable::Enabled);
        self.int_gen_1_reg.set_ylie(Enable::Enabled);
        self.int_gen_1_reg.set_zhie(Enable::Enabled);
        self.int_gen_1_reg.set_zlie(Enable::Enabled);
        self.int_gen_1_ths = threshold & 0x7F;
        self.int_gen_1_duration = 0;
        self
    }

    /// Configure accelerometer interrupt generator 2
    pub fn with_accel_int2_axes(
        mut self,
        x_high: bool,
        x_low: bool,
        y_high: bool,
        y_low: bool,
        z_high: bool,
        z_low: bool,
    ) -> Self {
        self.int_gen_2_reg.set_xhie(if x_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_2_reg.set_xlie(if x_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_2_reg.set_yhie(if y_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_2_reg.set_ylie(if y_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_2_reg.set_zhie(if z_high {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_gen_2_reg.set_zlie(if z_low {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set accelerometer interrupt 2 mode (AND/OR combination, 6D detection)
    pub fn with_accel_int2_mode(mut self, and_combination: bool, six_d: bool) -> Self {
        self.int_gen_2_reg.set_aoi(if and_combination {
            InterruptCombination::And
        } else {
            InterruptCombination::Or
        });
        self.int_gen_2_reg.set_six_d(if six_d {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set accelerometer interrupt 2 threshold (7-bit value, max 0x7F)
    pub fn with_accel_int2_threshold(mut self, threshold: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "accel int2 threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.int_gen_2_ths = threshold & 0x7F;
        self
    }

    /// Set accelerometer interrupt 2 duration (7-bit value, max 0x7F)
    pub fn with_accel_int2_duration(mut self, duration: u8) -> Self {
        debug_assert!(
            duration <= 0x7F,
            "accel int2 duration {duration:#04X} exceeds 7-bit max (0x7F)"
        );
        self.int_gen_2_duration = duration & 0x7F;
        self
    }

    /// Set accelerometer interrupt 2 latch mode
    pub fn with_accel_int2_latch(mut self, latch: bool) -> Self {
        self.ctrl_reg5_xm.set_lir2(if latch {
            LatchInterrupt::Latched
        } else {
            LatchInterrupt::NotLatched
        });
        self
    }

    /// Configure interrupt routing to INT1_XM pin
    #[allow(clippy::too_many_arguments)]
    pub fn with_int1_xm_routing(
        mut self,
        boot: bool,
        tap: bool,
        int1: bool,
        int2: bool,
        mag_int: bool,
        accel_drdy: bool,
        mag_drdy: bool,
        fifo_empty: bool,
    ) -> Self {
        self.ctrl_reg3_xm.set_p1_boot(if boot {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_tap(if tap {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_int1(if int1 {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_int2(if int2 {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_intm(if mag_int {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_drdya(if accel_drdy {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_drdym(if mag_drdy {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg3_xm.set_p1_empty(if fifo_empty {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Configure interrupt routing to INT2_XM pin
    #[allow(clippy::too_many_arguments)]
    pub fn with_int2_xm_routing(
        mut self,
        tap: bool,
        int1: bool,
        int2: bool,
        mag_int: bool,
        accel_drdy: bool,
        mag_drdy: bool,
        fifo_overrun: bool,
        fifo_wtm: bool,
    ) -> Self {
        self.ctrl_reg4_xm.set_p2_tap(if tap {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_int1(if int1 {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_int2(if int2 {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_intm(if mag_int {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_drdya(if accel_drdy {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_drdym(if mag_drdy {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_overrun(if fifo_overrun {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.ctrl_reg4_xm.set_p2_wtm(if fifo_wtm {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Configure click detection axes
    pub fn with_click_axes(
        mut self,
        x_single: bool,
        x_double: bool,
        y_single: bool,
        y_double: bool,
        z_single: bool,
        z_double: bool,
    ) -> Self {
        self.click_cfg.set_xs(if x_single {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.click_cfg.set_xd(if x_double {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.click_cfg.set_ys(if y_single {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.click_cfg.set_yd(if y_double {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.click_cfg.set_zs(if z_single {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.click_cfg.set_zd(if z_double {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set click detection threshold (7-bit value, max 0x7F)
    pub fn with_click_threshold(mut self, threshold: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "click threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.click_ths = threshold & 0x7F;
        self
    }

    /// Set click time limit (milliseconds)
    pub fn with_click_time_limit(mut self, limit: u8) -> Self {
        self.time_limit_ms = limit;
        self
    }

    /// Set click time latency (milliseconds)
    pub fn with_click_time_latency(mut self, latency: u8) -> Self {
        self.time_latency_ms = latency;
        self
    }

    /// Set click time window (milliseconds)
    pub fn with_click_time_window(mut self, window: u8) -> Self {
        self.time_window_ms = window;
        self
    }

    /// Simple single-click detection on all axes
    ///
    /// Threshold is a 7-bit value (max 0x7F).
    pub fn with_click_detection(mut self, threshold: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "click detection threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.click_cfg.set_xs(Enable::Enabled);
        self.click_cfg.set_ys(Enable::Enabled);
        self.click_cfg.set_zs(Enable::Enabled);
        self.click_ths = threshold & 0x7F;
        self.time_limit_ms = 10;
        self.time_latency_ms = 20;
        self.time_window_ms = 50;
        self
    }

    /// Set activity detection threshold (7-bit value, max 0x7F, 1 LSB = 16 mg)
    pub fn with_activity_threshold(mut self, threshold: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "activity threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.act_ths = threshold & 0x7F;
        self
    }

    /// Set activity detection duration
    pub fn with_activity_duration(mut self, duration: u8) -> Self {
        self.act_dur = duration;
        self
    }

    /// Configure activity/inactivity detection
    ///
    /// Threshold is a 7-bit value (max 0x7F).
    pub fn with_activity_detection(mut self, threshold: u8, duration: u8) -> Self {
        debug_assert!(
            threshold <= 0x7F,
            "activity threshold {threshold:#04X} exceeds 7-bit max (0x7F)"
        );
        self.act_ths = threshold & 0x7F;
        self.act_dur = duration;
        self
    }
}

/// # Magnetometer Configuration
///
/// Methods for configuring the magnetometer sensor including mode, scale, data rate, resolution,
/// and interrupt settings.
impl Lsm9ds0Config {
    /// Set the magnetometer operating mode
    pub fn with_mag_mode(mut self, mode: MagMode) -> Self {
        self.ctrl_reg7_xm.set_md(mode);
        self
    }

    /// Set the magnetometer output data rate
    pub fn with_mag_data_rate(mut self, rate: MagDataRate) -> Self {
        // MagDataRate::Hz100 requires accelerometer ODR > 50 Hz or power-down
        // (Datasheet Table 84, footnote 1)
        if rate == MagDataRate::Hz100 {
            let accel_odr = self.ctrl_reg1_xm.aodr();
            debug_assert!(
                matches!(
                    accel_odr,
                    AccelDataRate::PowerDown
                        | AccelDataRate::Hz100
                        | AccelDataRate::Hz200
                        | AccelDataRate::Hz400
                        | AccelDataRate::Hz800
                        | AccelDataRate::Hz1600
                ),
                "MagDataRate::Hz100 requires AccelDataRate > 50 Hz or PowerDown, got {:?}",
                accel_odr
            );
        }
        self.ctrl_reg5_xm.set_m_odr(rate);
        self
    }

    /// Set the magnetometer full-scale range
    pub fn with_mag_scale(mut self, scale: MagScale) -> Self {
        self.ctrl_reg6_xm.set_mfs(scale);
        self
    }

    /// Set the magnetometer resolution
    pub fn with_mag_resolution(mut self, res: MagResolution) -> Self {
        self.ctrl_reg5_xm.set_m_res(res);
        self
    }

    /// Enable magnetometer low power mode
    pub fn with_mag_low_power(mut self, enabled: bool) -> Self {
        self.ctrl_reg7_xm.set_mlp(if enabled {
            MagLowPower::LowPower
        } else {
            MagLowPower::Normal
        });
        self
    }

    /// Configure magnetometer interrupt
    pub fn with_mag_int(
        mut self,
        enabled: bool,
        x: bool,
        y: bool,
        z: bool,
        active_high: bool,
        latch: bool,
    ) -> Self {
        self.int_ctrl_reg_m.set_mien(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self.int_ctrl_reg_m
            .set_xmien(if x { Enable::Enabled } else { Enable::Disabled });
        self.int_ctrl_reg_m
            .set_ymien(if y { Enable::Enabled } else { Enable::Disabled });
        self.int_ctrl_reg_m
            .set_zmien(if z { Enable::Enabled } else { Enable::Disabled });
        self.int_ctrl_reg_m.set_iea(if active_high {
            ActiveLevelInverted::ActiveHigh
        } else {
            ActiveLevelInverted::ActiveLow
        });
        self.int_ctrl_reg_m.set_iel(if latch {
            LatchInterrupt::Latched
        } else {
            LatchInterrupt::NotLatched
        });
        self
    }

    /// Set magnetometer interrupt threshold (15-bit value, max 0x7FFF)
    pub fn with_mag_int_threshold(mut self, threshold: u16) -> Self {
        debug_assert!(
            threshold <= 0x7FFF,
            "mag interrupt threshold {threshold:#06X} exceeds 15-bit max (0x7FFF)"
        );
        self.mag_int_ths = threshold & 0x7FFF;
        self
    }

    /// Set magnetometer offset calibration for all axes
    pub fn with_mag_offset(mut self, x: i16, y: i16, z: i16) -> Self {
        self.mag_offset_x = x;
        self.mag_offset_y = y;
        self.mag_offset_z = z;
        self
    }

    /// Set magnetometer X-axis offset calibration
    pub fn with_mag_offset_x(mut self, offset: i16) -> Self {
        self.mag_offset_x = offset;
        self
    }

    /// Set magnetometer Y-axis offset calibration
    pub fn with_mag_offset_y(mut self, offset: i16) -> Self {
        self.mag_offset_y = offset;
        self
    }

    /// Set magnetometer Z-axis offset calibration
    pub fn with_mag_offset_z(mut self, offset: i16) -> Self {
        self.mag_offset_z = offset;
        self
    }
}

/// # Temperature Sensor Configuration
///
/// Methods for configuring the built-in temperature sensor.
impl Lsm9ds0Config {
    /// Enable or disable the temperature sensor
    pub fn with_temperature_enabled(mut self, enabled: bool) -> Self {
        self.ctrl_reg5_xm.set_temp_en(if enabled {
            Enable::Enabled
        } else {
            Enable::Disabled
        });
        self
    }

    /// Set the temperature offset for absolute temperature calculation
    ///
    /// The LSM9DS0 temperature sensor outputs relative temperature changes from an unspecified
    /// baseline. This offset is added to convert to absolute temperature.
    ///
    /// **Calibration procedure:**
    /// 1. Place the sensor in a known temperature environment
    /// 2. Read the raw temperature value
    /// 3. Calculate offset: `offset = known_temp - (raw_value / 8.0)`
    /// 4. Use this method to set the calculated offset
    ///
    /// # Example
    /// ```
    /// use lsm9ds0::Lsm9ds0Config;
    ///
    /// // After calibration, set custom offset
    /// let config = Lsm9ds0Config::new()
    ///     .with_temperature_enabled(true)
    ///     .with_temperature_offset(21.5); // Calibrated offset
    /// ```
    pub fn with_temperature_offset(mut self, offset: f32) -> Self {
        self.temp_offset = offset;
        self
    }
}

/// # Common Configuration
///
/// Convenience methods that configure multiple sensors at once.
impl Lsm9ds0Config {
    /// Set block data update for both gyro and accel
    pub fn with_block_data_update(mut self, enabled: bool) -> Self {
        let bdu = if enabled {
            BlockDataUpdate::WaitForRead
        } else {
            BlockDataUpdate::Continuous
        };
        self.ctrl_reg4_g.set_bdu(bdu);
        self.ctrl_reg1_xm.set_bdu(bdu);
        self
    }

    /// Configure SPI interface mode for both gyro and accel/mag
    ///
    /// - **FourWire** (default): Standard 4-wire SPI with separate SDI and SDO lines
    /// - **ThreeWire**: 3-wire SPI mode where SDI/SDO share a single bidirectional line
    ///
    /// Note: When using 3-wire mode, ensure your SPI bus and driver support bidirectional data
    /// transfer on a single line.
    pub fn with_spi_mode(mut self, mode: SpiMode) -> Self {
        self.ctrl_reg4_g.set_sim(mode);
        self.ctrl_reg2_xm.set_sim(mode);
        self
    }
}

/// # Bias Calibration Configuration
///
/// Methods for configuring gyroscope and accelerometer bias correction. Bias values are subtracted
/// from sensor readings to compensate for sensor offset errors.
impl Lsm9ds0Config {
    /// Set gyroscope bias correction values.
    ///
    /// These values (in degrees per second) are subtracted from gyroscope readings in
    /// [`Lsm9ds0::read_gyro`](crate::Lsm9ds0::read_gyro) to compensate for sensor offset.
    ///
    /// # Example
    /// ```
    /// use lsm9ds0::Lsm9ds0Config;
    ///
    /// // Apply pre-calibrated gyro bias values
    /// let config = Lsm9ds0Config::new()
    ///     .with_gyro_enabled(true)
    ///     .with_gyro_bias(0.5, -0.2, 0.1); // dps
    /// ```
    pub fn with_gyro_bias(mut self, x: f32, y: f32, z: f32) -> Self {
        self.gyro_bias = (x, y, z);
        self
    }

    /// Set accelerometer bias correction values.
    ///
    /// These values (in g-force) are subtracted from accelerometer readings in
    /// [`Lsm9ds0::read_accel`](crate::Lsm9ds0::read_accel) to compensate for sensor offset.
    ///
    /// # Example
    /// ```
    /// use lsm9ds0::Lsm9ds0Config;
    ///
    /// // Apply pre-calibrated accel bias values
    /// let config = Lsm9ds0Config::new()
    ///     .with_accel_data_rate(lsm9ds0::AccelDataRate::Hz100)
    ///     .with_accel_bias(0.01, -0.02, 0.03); // g
    /// ```
    pub fn with_accel_bias(mut self, x: f32, y: f32, z: f32) -> Self {
        self.accel_bias = (x, y, z);
        self
    }

    /// Enable automatic bias calibration during [`Lsm9ds0::init`](crate::Lsm9ds0::init).
    ///
    /// When enabled, the driver will use the FIFO to collect samples and calculate bias values
    /// automatically. The device must be stationary during initialization.
    ///
    /// The `orientation` parameter specifies which axis points "up" (against gravity) during
    /// calibration:
    /// - Use a specific orientation (e.g., [`Orientation::ZUp`](crate::Orientation::ZUp)) for
    ///   gravity-referenced readings where the accelerometer shows the gravity vector when stationary.
    /// - Use [`Orientation::Unknown`](crate::Orientation::Unknown) for pose-relative readings where
    ///   the accelerometer reads ~(0, 0, 0) in the calibration pose.
    ///
    /// **Note:** Auto-calibration requires a delay provider to be passed to
    /// [`Lsm9ds0::init`](crate::Lsm9ds0::init).
    ///
    /// # Example
    /// ```
    /// use lsm9ds0::{Lsm9ds0Config, Orientation};
    ///
    /// // Auto-calibrate with sensor Z-axis pointing up
    /// let config = Lsm9ds0Config::new()
    ///     .with_gyro_enabled(true)
    ///     .with_accel_data_rate(lsm9ds0::AccelDataRate::Hz100)
    ///     .with_auto_calibration(Orientation::ZUp);
    ///
    /// // Auto-calibrate without gravity compensation (pose-relative)
    /// let config = Lsm9ds0Config::new()
    ///     .with_gyro_enabled(true)
    ///     .with_accel_data_rate(lsm9ds0::AccelDataRate::Hz100)
    ///     .with_auto_calibration(Orientation::Unknown);
    /// ```
    pub fn with_auto_calibration(mut self, orientation: crate::types::Orientation) -> Self {
        self.auto_calibration = Some(orientation);
        self
    }

    /// Get the current gyroscope bias values (x, y, z) in degrees per second.
    pub fn gyro_bias(&self) -> (f32, f32, f32) {
        self.gyro_bias
    }

    /// Get the current accelerometer bias values (x, y, z) in g-force.
    pub fn accel_bias(&self) -> (f32, f32, f32) {
        self.accel_bias
    }

    /// Get the auto-calibration orientation, if configured.
    pub fn auto_calibration(&self) -> Option<crate::types::Orientation> {
        self.auto_calibration
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = Lsm9ds0Config::default();

        // Test gyroscope defaults match datasheet POR values
        assert_eq!(config.ctrl_reg1_g.pd(), PowerMode::PowerDown);
        assert_eq!(config.ctrl_reg1_g.dr(), GyroDataRate::Hz95);
        assert_eq!(config.ctrl_reg4_g.fs(), GyroScale::Dps245);
        assert_eq!(config.ctrl_reg4_g.bdu(), BlockDataUpdate::Continuous);

        // Test accelerometer defaults
        assert_eq!(config.ctrl_reg1_xm.aodr(), AccelDataRate::PowerDown);
        assert_eq!(config.ctrl_reg2_xm.afs(), AccelScale::G2);

        // Test magnetometer defaults
        assert_eq!(config.ctrl_reg6_xm.mfs(), MagScale::Gauss4);
        assert_eq!(config.ctrl_reg7_xm.md(), MagMode::PowerDown);

        // Test multi-byte value defaults
        assert_eq!(config.gyro_int_ths_x, 0);
        assert_eq!(config.mag_offset_x, 0);
    }

    #[test]
    fn test_builder_pattern() {
        let config = Lsm9ds0Config::new()
            .with_gyro_scale(GyroScale::Dps2000)
            .with_accel_scale(AccelScale::G8)
            .with_mag_scale(MagScale::Gauss12)
            .with_block_data_update(true)
            .with_temperature_enabled(true);

        assert_eq!(config.ctrl_reg4_g.fs(), GyroScale::Dps2000);
        assert_eq!(config.ctrl_reg2_xm.afs(), AccelScale::G8);
        assert_eq!(config.ctrl_reg6_xm.mfs(), MagScale::Gauss12);
        assert_eq!(config.ctrl_reg4_g.bdu(), BlockDataUpdate::WaitForRead);
        assert_eq!(config.ctrl_reg1_xm.bdu(), BlockDataUpdate::WaitForRead);
        assert_eq!(config.ctrl_reg5_xm.temp_en(), Enable::Enabled);
    }

    #[test]
    fn test_gyro_motion_threshold() {
        let config = Lsm9ds0Config::new().with_gyro_motion_threshold(1000);

        assert_eq!(config.ctrl_reg3_g.i1_int1(), Enable::Enabled);
        assert_eq!(config.int1_cfg_g.and_or(), InterruptCombination::Or);
        assert_eq!(config.int1_cfg_g.xhie(), Enable::Enabled);
        assert_eq!(config.int1_cfg_g.xlie(), Enable::Enabled);
        assert_eq!(config.gyro_int_ths_x, 1000);
        assert_eq!(config.gyro_int_ths_y, 1000);
        assert_eq!(config.gyro_int_ths_z, 1000);
    }

    #[test]
    fn test_per_axis_gyro_threshold() {
        let config = Lsm9ds0Config::new().with_gyro_int_threshold_xyz(100, 200, 300);

        assert_eq!(config.gyro_int_ths_x, 100);
        assert_eq!(config.gyro_int_ths_y, 200);
        assert_eq!(config.gyro_int_ths_z, 300);
    }

    #[test]
    fn test_threshold_at_max_value() {
        // Test that max valid values work correctly
        let config = Lsm9ds0Config::new()
            .with_gyro_int_threshold(0x7FFF) // Max 15-bit value
            .with_accel_int1_threshold(0x7F); // Max 7-bit value

        assert_eq!(config.gyro_int_ths_x, 0x7FFF);
        assert_eq!(config.int_gen_1_ths, 0x7F);
    }

    #[test]
    #[should_panic(expected = "exceeds 15-bit max")]
    #[cfg(debug_assertions)]
    fn test_gyro_threshold_overflow_panics_in_debug() {
        // In debug builds, exceeding max should panic
        let _ = Lsm9ds0Config::new().with_gyro_int_threshold(0xFFFF);
    }

    #[test]
    #[should_panic(expected = "exceeds 7-bit max")]
    #[cfg(debug_assertions)]
    fn test_accel_threshold_overflow_panics_in_debug() {
        // In debug builds, exceeding max should panic
        let _ = Lsm9ds0Config::new().with_accel_int1_threshold(0xFF);
    }

    #[test]
    fn test_sensitivity_values() {
        let config = Lsm9ds0Config::new()
            .with_gyro_scale(GyroScale::Dps500)
            .with_accel_scale(AccelScale::G4)
            .with_mag_scale(MagScale::Gauss8);

        assert_eq!(config.gyro_sensitivity(), 17.5);
        assert_eq!(config.accel_sensitivity(), 0.122);
        assert_eq!(config.mag_sensitivity(), 0.32);
    }

    #[test]
    fn test_click_detection() {
        let config = Lsm9ds0Config::new().with_click_detection(32);

        assert_eq!(config.click_cfg.xs(), Enable::Enabled);
        assert_eq!(config.click_cfg.ys(), Enable::Enabled);
        assert_eq!(config.click_cfg.zs(), Enable::Enabled);
        assert_eq!(config.click_ths, 32);
    }

    #[test]
    fn test_register_to_u8_conversion() {
        let config = Lsm9ds0Config::new()
            .with_gyro_enabled(true)
            .with_gyro_data_rate(GyroDataRate::Hz380)
            .with_gyro_axes(true, true, true);

        // CTRL_REG1_G should have: DR=10, BW=00, PD=1 (Normal), ZEN=1, YEN=1, XEN=1
        // = 0b10_00_1_1_1_1 = 0x8F
        let reg_value: u8 = config.ctrl_reg1_g.into();
        assert_eq!(reg_value, 0x8F);
    }
}
