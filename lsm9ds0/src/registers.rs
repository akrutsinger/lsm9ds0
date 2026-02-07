//! LSM9DS0 Register Definitions
//!
//! Register definitions for the LSM9DS0 9DOF IMU using bitfield-struct.
//! The device contains separate register banks for:
//! - Gyroscope (G)
//! - Accelerometer and Magnetometer (XM)

use bitfield_struct::bitfield;

// ============================================================================
// ENUMS FOR REGISTER FIELD VALUES
// ============================================================================

/// Gyroscope output data rate selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroDataRate {
    /// 95 Hz
    #[default]
    Hz95 = 0b00,
    /// 190 Hz
    Hz190 = 0b01,
    /// 380 Hz
    Hz380 = 0b10,
    /// 760 Hz
    Hz760 = 0b11,
}

impl GyroDataRate {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Hz95,
            0b01 => Self::Hz190,
            0b10 => Self::Hz380,
            _ => Self::Hz760,
        }
    }

    /// Get the output data rate in Hz
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz95 => 95.0,
            Self::Hz190 => 190.0,
            Self::Hz380 => 380.0,
            Self::Hz760 => 760.0,
        }
    }
}

/// Gyroscope bandwidth selection
///
/// The actual low-pass filter cutoff frequency depends on the output data rate (ODR).
/// Use [`Lsm9ds0Config::gyro_lpf_cutoff_hz`](crate::Lsm9ds0Config::gyro_lpf_cutoff_hz) to get the
/// actual cutoff frequency based on current ODR and bandwidth settings.
///
/// | Bandwidth | ODR=95Hz | ODR=190Hz | ODR=380Hz | ODR=760Hz |
/// |-----------|----------|-----------|-----------|-----------|
/// | Bw0       | 12.5 Hz  | 12.5 Hz   | 20 Hz     | 30 Hz     |
/// | Bw1       | 25 Hz    | 25 Hz     | 25 Hz     | 35 Hz     |
/// | Bw2       | 25 Hz    | 50 Hz     | 50 Hz     | 50 Hz     |
/// | Bw3       | 25 Hz    | 70 Hz     | 100 Hz    | 100 Hz    |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroBandwidth {
    /// Lowest bandwidth (12.5-30 Hz depending on ODR)
    #[default]
    Bw0 = 0b00,
    /// Low-medium bandwidth (25-35 Hz depending on ODR)
    Bw1 = 0b01,
    /// Medium-high bandwidth (25-50 Hz depending on ODR)
    Bw2 = 0b10,
    /// Highest bandwidth (25-100 Hz depending on ODR)
    Bw3 = 0b11,
}

impl GyroBandwidth {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Bw0,
            0b01 => Self::Bw1,
            0b10 => Self::Bw2,
            _ => Self::Bw3,
        }
    }
}

/// Gyroscope full-scale selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroScale {
    /// ±245 degrees per second
    #[default]
    Dps245 = 0b00,
    /// ±500 degrees per second
    Dps500 = 0b01,
    /// ±2000 degrees per second
    Dps2000 = 0b10,
}

impl GyroScale {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Dps245,
            0b01 => Self::Dps500,
            // 0b10 and 0b11 both map to Dps2000
            _ => Self::Dps2000,
        }
    }

    /// Get sensitivity in mdps/LSB
    pub const fn sensitivity(self) -> f32 {
        match self {
            Self::Dps245 => 8.75,
            Self::Dps500 => 17.5,
            Self::Dps2000 => 70.0,
        }
    }
}

/// Gyroscope high-pass filter mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroHpfMode {
    /// Normal mode (reset reading HP_RESET_FILTER)
    #[default]
    NormalReset = 0b00,
    /// Reference signal for filtering
    Reference = 0b01,
    /// Normal mode
    Normal = 0b10,
    /// Autoreset on interrupt event
    AutoReset = 0b11,
}

impl GyroHpfMode {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::NormalReset,
            0b01 => Self::Reference,
            0b10 => Self::Normal,
            _ => Self::AutoReset,
        }
    }
}

/// Gyroscope high-pass filter cutoff frequency selection
/// The actual cutoff frequency depends on the ODR (see datasheet Table 26)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroHpfCutoff {
    #[default]
    Cutoff0 = 0b0000,
    Cutoff1 = 0b0001,
    Cutoff2 = 0b0010,
    Cutoff3 = 0b0011,
    Cutoff4 = 0b0100,
    Cutoff5 = 0b0101,
    Cutoff6 = 0b0110,
    Cutoff7 = 0b0111,
    Cutoff8 = 0b1000,
    Cutoff9 = 0b1001,
}

impl GyroHpfCutoff {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1111 {
            0b0000 => Self::Cutoff0,
            0b0001 => Self::Cutoff1,
            0b0010 => Self::Cutoff2,
            0b0011 => Self::Cutoff3,
            0b0100 => Self::Cutoff4,
            0b0101 => Self::Cutoff5,
            0b0110 => Self::Cutoff6,
            0b0111 => Self::Cutoff7,
            0b1000 => Self::Cutoff8,
            _ => Self::Cutoff9,
        }
    }
}

/// Gyroscope self-test mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroSelfTest {
    #[default]
    Disabled = 0b00,
    /// Self-test 0 (X positive, Y and Z negative)
    Mode0 = 0b01,
    /// Self-test 1 (X negative, Y and Z positive)
    Mode1 = 0b11,
}

impl GyroSelfTest {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Disabled,
            0b01 => Self::Mode0,
            _ => Self::Mode1,
        }
    }
}

/// Gyroscope interrupt/output selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroOutputSel {
    #[default]
    LpfOnly = 0b00,
    LpfAndHpf = 0b01,
    LpfHpfAndLpf2 = 0b10,
}

impl GyroOutputSel {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::LpfOnly,
            0b01 => Self::LpfAndHpf,
            _ => Self::LpfHpfAndLpf2,
        }
    }
}

/// FIFO mode selection (shared by gyro and accel)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FifoMode {
    #[default]
    Bypass = 0b000,
    Fifo = 0b001,
    Stream = 0b010,
    StreamToFifo = 0b011,
    BypassToStream = 0b100,
}

impl FifoMode {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b111 {
            0b000 => Self::Bypass,
            0b001 => Self::Fifo,
            0b010 => Self::Stream,
            0b011 => Self::StreamToFifo,
            0b100 => Self::BypassToStream,
            _ => Self::Bypass,
        }
    }
}

/// Accelerometer output data rate selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelDataRate {
    #[default]
    PowerDown = 0b0000,
    Hz3_125 = 0b0001,
    Hz6_25 = 0b0010,
    Hz12_5 = 0b0011,
    Hz25 = 0b0100,
    Hz50 = 0b0101,
    Hz100 = 0b0110,
    Hz200 = 0b0111,
    Hz400 = 0b1000,
    Hz800 = 0b1001,
    Hz1600 = 0b1010,
}

impl AccelDataRate {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1111 {
            0b0000 => Self::PowerDown,
            0b0001 => Self::Hz3_125,
            0b0010 => Self::Hz6_25,
            0b0011 => Self::Hz12_5,
            0b0100 => Self::Hz25,
            0b0101 => Self::Hz50,
            0b0110 => Self::Hz100,
            0b0111 => Self::Hz200,
            0b1000 => Self::Hz400,
            0b1001 => Self::Hz800,
            0b1010 => Self::Hz1600,
            _ => Self::PowerDown,
        }
    }

    /// Get the output data rate in Hz (returns 0.0 for PowerDown)
    pub const fn hz(self) -> f32 {
        match self {
            Self::PowerDown => 0.0,
            Self::Hz3_125 => 3.125,
            Self::Hz6_25 => 6.25,
            Self::Hz12_5 => 12.5,
            Self::Hz25 => 25.0,
            Self::Hz50 => 50.0,
            Self::Hz100 => 100.0,
            Self::Hz200 => 200.0,
            Self::Hz400 => 400.0,
            Self::Hz800 => 800.0,
            Self::Hz1600 => 1600.0,
        }
    }
}

/// Accelerometer full-scale selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelScale {
    /// ±2g
    #[default]
    G2 = 0b000,
    /// ±4g
    G4 = 0b001,
    /// ±6g
    G6 = 0b010,
    /// ±8g
    G8 = 0b011,
    /// ±16g
    G16 = 0b100,
}

impl AccelScale {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b111 {
            0b000 => Self::G2,
            0b001 => Self::G4,
            0b010 => Self::G6,
            0b011 => Self::G8,
            _ => Self::G16,
        }
    }

    /// Get sensitivity in mg/LSB
    pub const fn sensitivity(self) -> f32 {
        match self {
            Self::G2 => 0.061,
            Self::G4 => 0.122,
            Self::G6 => 0.183,
            Self::G8 => 0.244,
            Self::G16 => 0.732,
        }
    }
}

/// Accelerometer anti-alias filter bandwidth
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelBandwidth {
    #[default]
    Hz773 = 0b00,
    Hz194 = 0b01,
    Hz362 = 0b10,
    Hz50 = 0b11,
}

impl AccelBandwidth {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Hz773,
            0b01 => Self::Hz194,
            0b10 => Self::Hz362,
            _ => Self::Hz50,
        }
    }
}

/// Accelerometer self-test mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelSelfTest {
    #[default]
    Normal = 0b00,
    PositiveSign = 0b01,
    NegativeSign = 0b10,
}

impl AccelSelfTest {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Normal,
            0b01 => Self::PositiveSign,
            _ => Self::NegativeSign,
        }
    }
}

/// Accelerometer high-pass filter mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelHpfMode {
    #[default]
    NormalReset = 0b00,
    Reference = 0b01,
    Normal = 0b10,
    AutoReset = 0b11,
}

impl AccelHpfMode {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::NormalReset,
            0b01 => Self::Reference,
            0b10 => Self::Normal,
            _ => Self::AutoReset,
        }
    }
}

/// Magnetometer output data rate selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagDataRate {
    Hz3_125 = 0b000,
    Hz6_25 = 0b001,
    Hz12_5 = 0b010,
    Hz25 = 0b011,
    #[default]
    Hz50 = 0b100,
    Hz100 = 0b101,
}

impl MagDataRate {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b111 {
            0b000 => Self::Hz3_125,
            0b001 => Self::Hz6_25,
            0b010 => Self::Hz12_5,
            0b011 => Self::Hz25,
            0b100 => Self::Hz50,
            0b101 => Self::Hz100,
            _ => Self::Hz50,
        }
    }
}

/// Magnetometer full-scale selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagScale {
    /// ±2 Gauss
    Gauss2 = 0b00,
    /// ±4 Gauss
    #[default]
    Gauss4 = 0b01,
    /// ±8 Gauss
    Gauss8 = 0b10,
    /// ±12 Gauss
    Gauss12 = 0b11,
}

impl MagScale {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Gauss2,
            0b01 => Self::Gauss4,
            0b10 => Self::Gauss8,
            _ => Self::Gauss12,
        }
    }

    /// Get sensitivity in mgauss/LSB
    pub const fn sensitivity(self) -> f32 {
        match self {
            Self::Gauss2 => 0.08,
            Self::Gauss4 => 0.16,
            Self::Gauss8 => 0.32,
            Self::Gauss12 => 0.48,
        }
    }
}

/// Magnetometer resolution selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagResolution {
    #[default]
    Low = 0b00,
    High = 0b11,
}

impl MagResolution {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::Low,
            _ => Self::High,
        }
    }
}

/// Magnetometer sensor mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagMode {
    ContinuousConversion = 0b00,
    SingleConversion = 0b01,
    #[default]
    PowerDown = 0b10,
}

impl MagMode {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b11 {
            0b00 => Self::ContinuousConversion,
            0b01 => Self::SingleConversion,
            _ => Self::PowerDown,
        }
    }
}

/// Enable/Disable state for features
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Enable {
    #[default]
    Disabled = 0,
    Enabled = 1,
}

impl Enable {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::Disabled,
            _ => Self::Enabled,
        }
    }
}

impl From<bool> for Enable {
    fn from(value: bool) -> Self {
        if value { Self::Enabled } else { Self::Disabled }
    }
}

/// Power mode for sensors (used in CTRL_REG1 PD bit)
///
/// Note: This represents the PD bit state only. For the gyroscope, the actual operating mode also
/// depends on axis enable bits. Use [`GyroPowerMode`] for complete gyroscope power mode control
/// including sleep mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PowerMode {
    #[default]
    PowerDown = 0,
    Normal = 1,
}

impl PowerMode {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::PowerDown,
            _ => Self::Normal,
        }
    }
}

/// Gyroscope power mode with sleep support
///
/// The gyroscope supports three power modes (see datasheet Table 22):
/// - **PowerDown**: Lowest power consumption (~0 µA)
/// - **Sleep**: Intermediate power (~0.35 mA), faster wake-up than power-down.
///   In sleep mode, PD=1 with all axes disabled. The gyroscope maintains its internal
///   state for faster wake-up.
/// - **Normal**: Full operation (~6.1 mA) with configured axes enabled
///
/// Use this enum with
/// [`Lsm9ds0Config::with_gyro_power_mode`](crate::Lsm9ds0Config::with_gyro_power_mode) or
/// [`Lsm9ds0::set_gyro_power_mode`](crate::Lsm9ds0::set_gyro_power_mode) for runtime control.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroPowerMode {
    /// Lowest power consumption, sensor completely off
    #[default]
    PowerDown = 0,
    /// Lower power with faster wake-up than power-down
    Sleep = 1,
    /// Full operation with configured axes enabled
    Normal = 2,
}

/// Block data update mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BlockDataUpdate {
    #[default]
    Continuous = 0,
    WaitForRead = 1,
}

impl BlockDataUpdate {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::Continuous,
            _ => Self::WaitForRead,
        }
    }
}

/// Output pin type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum OutputType {
    #[default]
    PushPull = 0,
    OpenDrain = 1,
}

impl OutputType {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::PushPull,
            _ => Self::OpenDrain,
        }
    }
}

/// Interrupt active level (for h_lactive in CtrlReg3G: 0=high, 1=low)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ActiveLevel {
    #[default]
    ActiveHigh = 0,
    ActiveLow = 1,
}

impl ActiveLevel {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::ActiveHigh,
            _ => Self::ActiveLow,
        }
    }
}

/// Interrupt active level inverted (for iea in IntCtrlRegM: 0=low, 1=high)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ActiveLevelInverted {
    #[default]
    ActiveLow = 0,
    ActiveHigh = 1,
}

impl ActiveLevelInverted {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::ActiveLow,
            _ => Self::ActiveHigh,
        }
    }
}

/// Data byte order
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Endianness {
    #[default]
    Little = 0,
    Big = 1,
}

impl Endianness {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::Little,
            _ => Self::Big,
        }
    }
}

/// SPI interface mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SpiMode {
    #[default]
    FourWire = 0,
    ThreeWire = 1,
}

impl SpiMode {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::FourWire,
            _ => Self::ThreeWire,
        }
    }
}

/// Latch interrupt request
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LatchInterrupt {
    #[default]
    NotLatched = 0,
    Latched = 1,
}

impl LatchInterrupt {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::NotLatched,
            _ => Self::Latched,
        }
    }
}

impl From<bool> for LatchInterrupt {
    fn from(value: bool) -> Self {
        if value {
            Self::Latched
        } else {
            Self::NotLatched
        }
    }
}

/// Interrupt combination mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InterruptCombination {
    #[default]
    Or = 0,
    And = 1,
}

impl InterruptCombination {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::Or,
            _ => Self::And,
        }
    }
}

/// Magnetometer low power mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MagLowPower {
    #[default]
    Normal = 0,
    LowPower = 1,
}

impl MagLowPower {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::Normal,
            _ => Self::LowPower,
        }
    }
}

/// Click sign/direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ClickSign {
    #[default]
    Positive = 0,
    Negative = 1,
}

impl ClickSign {
    const fn into_bits(self) -> u8 {
        self as u8
    }

    const fn from_bits(value: u8) -> Self {
        match value & 0b1 {
            0 => Self::Positive,
            _ => Self::Negative,
        }
    }
}

// ============================================================================
// GYROSCOPE REGISTER BITFIELD STRUCTS
// ============================================================================

/// CTRL_REG1_G - Gyroscope control register 1
///
/// Address: 0x20, Default value: 0x07
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg1G {
    /// X-axis enable
    #[bits(1)]
    pub xen: Enable,
    /// Y-axis enable
    #[bits(1)]
    pub yen: Enable,
    /// Z-axis enable
    #[bits(1)]
    pub zen: Enable,
    /// Power-down mode enable
    #[bits(1)]
    pub pd: PowerMode,
    /// Bandwidth selection
    #[bits(2)]
    pub bw: GyroBandwidth,
    /// Output data rate selection
    #[bits(2)]
    pub dr: GyroDataRate,
}

/// CTRL_REG2_G - Gyroscope control register 2
///
/// Address: 0x21, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg2G {
    /// High-pass filter cutoff frequency selection
    #[bits(4)]
    pub hpcf: GyroHpfCutoff,
    /// High-pass filter mode selection
    #[bits(2)]
    pub hpm: GyroHpfMode,
    /// Reserved (must be 0)
    #[bits(2)]
    __reserved: u8,
}

/// CTRL_REG3_G - Gyroscope control register 3
///
/// Address: 0x22, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg3G {
    /// FIFO empty interrupt on DRDY_G
    #[bits(1)]
    pub i2_empty: Enable,
    /// FIFO overrun interrupt on DRDY_G
    #[bits(1)]
    pub i2_orun: Enable,
    /// FIFO watermark interrupt on DRDY_G
    #[bits(1)]
    pub i2_wtm: Enable,
    /// Data-ready on DRDY_G
    #[bits(1)]
    pub i2_drdy: Enable,
    /// Push-pull / Open-drain
    #[bits(1)]
    pub pp_od: OutputType,
    /// Interrupt active configuration
    #[bits(1)]
    pub h_lactive: ActiveLevel,
    /// Boot status available on INT_G
    #[bits(1)]
    pub i1_boot: Enable,
    /// Interrupt enable on INT_G
    #[bits(1)]
    pub i1_int1: Enable,
}

/// CTRL_REG4_G - Gyroscope control register 4
///
/// Address: 0x23, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg4G {
    /// SPI mode selection
    #[bits(1)]
    pub sim: SpiMode,
    /// Self-test enable
    #[bits(2)]
    pub st: GyroSelfTest,
    /// Reserved
    #[bits(1)]
    __reserved: u8,
    /// Full-scale selection
    #[bits(2)]
    pub fs: GyroScale,
    /// Big/little endian
    #[bits(1)]
    pub ble: Endianness,
    /// Block data update
    #[bits(1)]
    pub bdu: BlockDataUpdate,
}

/// CTRL_REG5_G - Gyroscope control register 5
///
/// Address: 0x24, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg5G {
    /// Out selection configuration
    #[bits(2)]
    pub out_sel: GyroOutputSel,
    /// INT1 selection configuration
    #[bits(2)]
    pub int1_sel: GyroOutputSel,
    /// High-pass filter enable
    #[bits(1)]
    pub hpen: Enable,
    /// Reserved
    #[bits(1)]
    __reserved: u8,
    /// FIFO enable
    #[bits(1)]
    pub fifo_en: Enable,
    /// Reboot memory content
    #[bits(1)]
    pub boot: Enable,
}

/// STATUS_REG_G - Gyroscope status register
///
/// Address: 0x27, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct StatusRegG {
    /// X-axis new data available
    pub xda: bool,
    /// Y-axis new data available
    pub yda: bool,
    /// Z-axis new data available
    pub zda: bool,
    /// X, Y, Z-axis new data available
    pub zyxda: bool,
    /// X-axis data overrun
    pub xor: bool,
    /// Y-axis data overrun
    pub yor: bool,
    /// Z-axis data overrun
    pub zor: bool,
    /// X, Y, Z-axis data overrun
    pub zyxor: bool,
}

/// FIFO_CTRL_REG_G - Gyroscope FIFO control register
///
/// Address: 0x2E, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct FifoCtrlRegG {
    /// FIFO threshold (watermark level)
    #[bits(5)]
    pub wtm: u8,
    /// FIFO mode selection
    #[bits(3)]
    pub fm: FifoMode,
}

/// FIFO_SRC_REG_G - Gyroscope FIFO source register
///
/// Address: 0x2F, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct FifoSrcRegG {
    /// FIFO stored data level
    #[bits(5)]
    pub fss: u8,
    /// FIFO empty
    pub empty: bool,
    /// FIFO overrun status
    pub ovrn: bool,
    /// Watermark status
    pub wtm: bool,
}

/// INT1_CFG_G - Gyroscope interrupt configuration
///
/// Address: 0x30, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct Int1CfgG {
    /// Enable interrupt generation on X low event
    #[bits(1)]
    pub xlie: Enable,
    /// Enable interrupt generation on X high event
    #[bits(1)]
    pub xhie: Enable,
    /// Enable interrupt generation on Y low event
    #[bits(1)]
    pub ylie: Enable,
    /// Enable interrupt generation on Y high event
    #[bits(1)]
    pub yhie: Enable,
    /// Enable interrupt generation on Z low event
    #[bits(1)]
    pub zlie: Enable,
    /// Enable interrupt generation on Z high event
    #[bits(1)]
    pub zhie: Enable,
    /// Latch interrupt request
    #[bits(1)]
    pub lir: LatchInterrupt,
    /// AND/OR combination of interrupt events
    #[bits(1)]
    pub and_or: InterruptCombination,
}

/// INT1_SRC_G - Gyroscope interrupt source register
///
/// Address: 0x31, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct Int1SrcG {
    /// X low event
    pub xl: bool,
    /// X high event
    pub xh: bool,
    /// Y low event
    pub yl: bool,
    /// Y high event
    pub yh: bool,
    /// Z low event
    pub zl: bool,
    /// Z high event
    pub zh: bool,
    /// Interrupt active
    pub ia: bool,
    /// Reserved
    #[bits(1)]
    __reserved: u8,
}

/// INT1_DURATION_G - Gyroscope interrupt minimum duration for event recognition
///
/// Address: 0x38, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct Int1DurationG {
    /// Mimimum duration of the interrupt event to be recognized
    #[bits(7)]
    pub duration: u8,
    /// WAIT enable
    pub wait: bool,
}

// ============================================================================
// ACCELEROMETER/MAGNETOMETER REGISTER BITFIELD STRUCTS
// ============================================================================

/// STATUS_REG_M - Magnetometer status register
///
/// Address: 0x07, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct StatusRegM {
    /// X-axis new data available
    pub xmda: bool,
    /// Y-axis new data available
    pub ymda: bool,
    /// Z-axis new data available
    pub zmda: bool,
    /// X, Y, Z-axis new data available
    pub zyxmda: bool,
    /// X-axis data overrun
    pub xmor: bool,
    /// Y-axis data overrun
    pub ymor: bool,
    /// Z-axis data overrun
    pub zmor: bool,
    /// Magnetic X, Y, Z-axis data overrun
    pub zyxmor: bool,
}

/// INT_CTRL_REG_M - Magnetometer interrupt control register
///
/// Address: 0x12, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct IntCtrlRegM {
    /// Magnetic interrupt enable
    #[bits(1)]
    pub mien: Enable,
    /// 4D enable
    #[bits(1)]
    pub fourd: Enable,
    /// Latch interrupt request on accelerometer
    #[bits(1)]
    pub iel: LatchInterrupt,
    /// Interrupt polarity for both accelerometer and magnetometer
    #[bits(1)]
    pub iea: ActiveLevelInverted,
    /// Interrupt pin configuration
    #[bits(1)]
    pub pp_od: OutputType,
    /// Z-axis interrupt enable
    #[bits(1)]
    pub zmien: Enable,
    /// Y-axis interrupt enable
    #[bits(1)]
    pub ymien: Enable,
    /// X-axis interrupt enable
    #[bits(1)]
    pub xmien: Enable,
}

/// INT_SRC_REG_M - Magnetometer interrupt source register
///
/// Address: 0x13, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct IntSrcRegM {
    /// Magnetic interrupt event
    pub mint: bool,
    /// Magnetic measurement overflow
    pub mroi: bool,
    /// Z-axis negative threshold exceeded
    pub m_nth_z: bool,
    /// Y-axis negative threshold exceeded
    pub m_nth_y: bool,
    /// X-axis negative threshold exceeded
    pub m_nth_x: bool,
    /// Z-axis positive threshold exceeded
    pub m_pth_z: bool,
    /// Y-axis positive threshold exceeded
    pub m_pth_y: bool,
    /// X-axis positive threshold exceeded
    pub m_pth_x: bool,
}

/// CTRL_REG0_XM - Accel/Mag control register 0
///
/// Address: 0x1F, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg0Xm {
    /// High-pass filter for interrupt 2
    #[bits(1)]
    pub hpis2: Enable,
    /// High-pass filter for interrupt 1
    #[bits(1)]
    pub hpis1: Enable,
    /// High-pass filter for click
    #[bits(1)]
    pub hp_click: Enable,
    /// Reserved
    #[bits(2)]
    __reserved: u8,
    /// FIFO programmable watermark enable
    #[bits(1)]
    pub wtm_en: Enable,
    /// FIFO enable
    #[bits(1)]
    pub fifo_en: Enable,
    /// Reboot memory content
    #[bits(1)]
    pub boot: Enable,
}

/// CTRL_REG1_XM - Accelerometer control register 1
///
/// Address: 0x20, Default value: 0x07
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg1Xm {
    /// Acceleration X-axis enable
    #[bits(1)]
    pub axen: Enable,
    /// Acceleration Y-axis enable
    #[bits(1)]
    pub ayen: Enable,
    /// Acceleration Z-axis enable
    #[bits(1)]
    pub azen: Enable,
    /// Block data update
    #[bits(1)]
    pub bdu: BlockDataUpdate,
    /// Acceleration data rate
    #[bits(4)]
    pub aodr: AccelDataRate,
}

/// CTRL_REG2_XM - Accelerometer control register 2
///
/// Address: 0x21, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg2Xm {
    /// SPI mode selection
    #[bits(1)]
    pub sim: SpiMode,
    /// Acceleration self-test enable
    #[bits(2)]
    pub ast: AccelSelfTest,
    /// Acceleration full-scale selection
    #[bits(3)]
    pub afs: AccelScale,
    /// Accelerometer anti-alias filter bandwidth
    #[bits(2)]
    pub abw: AccelBandwidth,
}

/// CTRL_REG3_XM - Interrupt control register (INT1_XM)
///
/// Address: 0x22, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg3Xm {
    /// FIFO empty on INT1_XM
    #[bits(1)]
    pub p1_empty: Enable,
    /// Mag data ready on INT1_XM
    #[bits(1)]
    pub p1_drdym: Enable,
    /// Accel data ready on INT1_XM
    #[bits(1)]
    pub p1_drdya: Enable,
    /// Magnetic interrupt on INT1_XM
    #[bits(1)]
    pub p1_intm: Enable,
    /// Inertial interrupt 2 on INT1_XM
    #[bits(1)]
    pub p1_int2: Enable,
    /// Inertial interrupt 1 on INT1_XM
    #[bits(1)]
    pub p1_int1: Enable,
    /// Tap interrupt on INT1_XM
    #[bits(1)]
    pub p1_tap: Enable,
    /// Boot on INT1_XM
    #[bits(1)]
    pub p1_boot: Enable,
}

/// CTRL_REG4_XM - Interrupt control register (INT2_XM)
///
/// Address: 0x23, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg4Xm {
    /// FIFO watermark on INT2_XM
    #[bits(1)]
    pub p2_wtm: Enable,
    /// FIFO overrun on INT2_XM
    #[bits(1)]
    pub p2_overrun: Enable,
    /// Mag data ready on INT2_XM
    #[bits(1)]
    pub p2_drdym: Enable,
    /// Accel data ready on INT2_XM
    #[bits(1)]
    pub p2_drdya: Enable,
    /// Magnetic interrupt on INT2_XM
    #[bits(1)]
    pub p2_intm: Enable,
    /// Inertial interrupt 2 on INT2_XM
    #[bits(1)]
    pub p2_int2: Enable,
    /// Inertial interrupt 1 on INT2_XM
    #[bits(1)]
    pub p2_int1: Enable,
    /// Tap interrupt on INT2_XM
    #[bits(1)]
    pub p2_tap: Enable,
}

/// CTRL_REG5_XM - Magnetometer control register 5
///
/// Address: 0x24, Default value: 0x18
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg5Xm {
    /// Latch interrupt on INT1
    #[bits(1)]
    pub lir1: LatchInterrupt,
    /// Latch interrupt on INT2
    #[bits(1)]
    pub lir2: LatchInterrupt,
    /// Magnetic data rate
    #[bits(3)]
    pub m_odr: MagDataRate,
    /// Magnetic resolution
    #[bits(2)]
    pub m_res: MagResolution,
    /// Temperature sensor enable
    #[bits(1)]
    pub temp_en: Enable,
}

/// CTRL_REG6_XM - Magnetometer control register 6
///
/// Address: 0x25, Default value: 0x20
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg6Xm {
    /// Reserved
    #[bits(5)]
    __reserved: u8,
    /// Magnetic full-scale selection
    #[bits(2)]
    pub mfs: MagScale,
    /// Reserved
    #[bits(1)]
    __reserved2: u8,
}

/// CTRL_REG7_XM - Magnetometer control register 7
///
/// Address: 0x26, Default value: 0x01
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct CtrlReg7Xm {
    /// Magnetic sensor mode
    #[bits(2)]
    pub md: MagMode,
    /// Magnetic low power mode
    #[bits(1)]
    pub mlp: MagLowPower,
    /// Reserved
    #[bits(2)]
    __reserved: u8,
    /// Filtered acceleration data selection
    #[bits(1)]
    pub afds: Enable,
    /// Accel high-pass filter mode
    #[bits(2)]
    pub ahpm: AccelHpfMode,
}

/// STATUS_REG_A - Accelerometer status register
///
/// Address: 0x27, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct StatusRegA {
    /// X-axis new data available
    pub xada: bool,
    /// Y-axis new data available
    pub yada: bool,
    /// Z-axis new data available
    pub zada: bool,
    /// X, Y, Z-axis new data available
    pub zyxada: bool,
    /// X-axis data overrun
    pub xaor: bool,
    /// Y-axis data overrun
    pub yaor: bool,
    /// Z-axis data overrun
    pub zaor: bool,
    /// X, Y, Z-axis data overrun
    pub zyxaor: bool,
}

/// FIFO_CTRL_REG - Accelerometer FIFO control register
///
/// Address: 0x2E, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct FifoCtrlReg {
    /// FIFO threshold
    #[bits(5)]
    pub fth: u8,
    /// FIFO mode selection
    #[bits(3)]
    pub fm: FifoMode,
}

/// FIFO_SRC_REG - Accelerometer FIFO source register
///
/// Address: 0x2F, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct FifoSrcReg {
    /// FIFO stored data level
    #[bits(5)]
    pub fss: u8,
    /// FIFO empty
    pub empty: bool,
    /// FIFO overrun status
    pub ovrn: bool,
    /// Watermark status
    pub wtm: bool,
}

/// INT_GEN_REG - Interrupt generator register (shared for INT1 and INT2)
///
/// Addresses: 0x30 (INT_GEN_1_REG), 0x34 (INT_GEN_2_REG)
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct IntGenReg {
    /// Enable interrupt on X low event
    #[bits(1)]
    pub xlie: Enable,
    /// Enable interrupt on X high event
    #[bits(1)]
    pub xhie: Enable,
    /// Enable interrupt on Y low event
    #[bits(1)]
    pub ylie: Enable,
    /// Enable interrupt on Y high event
    #[bits(1)]
    pub yhie: Enable,
    /// Enable interrupt on Z low event
    #[bits(1)]
    pub zlie: Enable,
    /// Enable interrupt on Z high event
    #[bits(1)]
    pub zhie: Enable,
    /// 6-direction detection enable
    #[bits(1)]
    pub six_d: Enable,
    /// AND/OR combination
    #[bits(1)]
    pub aoi: InterruptCombination,
}

/// INT_GEN_SRC - Interrupt generator source register (shared for INT1 and INT2)
///
/// Addresses: 0x31 (INT_GEN_1_SRC), 0x35 (INT_GEN_2_SRC)
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct IntGenSrc {
    /// X low event
    pub xl: bool,
    /// X high event
    pub xh: bool,
    /// Y low event
    pub yl: bool,
    /// Y high event
    pub yh: bool,
    /// Z low event
    pub zl: bool,
    /// Z high event
    pub zh: bool,
    /// Interrupt active
    pub ia: bool,
    /// Reserved
    #[bits(1)]
    __reserved: u8,
}

/// CLICK_CFG - Click configuration register
///
/// Address: 0x38, Default value: 0x00
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct ClickCfg {
    /// X single-click enable
    #[bits(1)]
    pub xs: Enable,
    /// X double-click enable
    #[bits(1)]
    pub xd: Enable,
    /// Y single-click enable
    #[bits(1)]
    pub ys: Enable,
    /// Y double-click enable
    #[bits(1)]
    pub yd: Enable,
    /// Z single-click enable
    #[bits(1)]
    pub zs: Enable,
    /// Z double-click enable
    #[bits(1)]
    pub zd: Enable,
    /// Reserved
    #[bits(2)]
    __reserved: u8,
}

/// CLICK_SRC - Click source register
///
/// Address: 0x39, Read-only
#[bitfield(u8, defmt = cfg(feature = "defmt"))]
pub struct ClickSrc {
    /// X click detected
    pub x: bool,
    /// Y click detected
    pub y: bool,
    /// Z click detected
    pub z: bool,
    /// Click sign
    #[bits(1)]
    pub sign: ClickSign,
    /// Single-click detected
    pub sclick: bool,
    /// Double-click detected
    pub dclick: bool,
    /// Interrupt active
    pub ia: bool,
    /// Reserved
    #[bits(1)]
    __reserved: u8,
}

// ============================================================================
// REGISTER ADDRESS ENUMS
// ============================================================================

/// Gyroscope register address map
#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroRegisters {
    /// Device identification register
    WHO_AM_I_G = 0x0F,

    /// Control registers
    CTRL_REG1_G = 0x20,
    CTRL_REG2_G = 0x21,
    CTRL_REG3_G = 0x22,
    CTRL_REG4_G = 0x23,
    CTRL_REG5_G = 0x24,

    /// Reference/data capture register
    REFERENCE_G = 0x25,

    /// Status register
    STATUS_REG_G = 0x27,

    /// Output registers
    OUT_X_L_G = 0x28,
    OUT_X_H_G = 0x29,
    OUT_Y_L_G = 0x2A,
    OUT_Y_H_G = 0x2B,
    OUT_Z_L_G = 0x2C,
    OUT_Z_H_G = 0x2D,

    /// FIFO control registers
    FIFO_CTRL_REG_G = 0x2E,
    FIFO_SRC_REG_G = 0x2F,

    /// Interrupt configuration
    INT1_CFG_G = 0x30,
    INT1_SRC_G = 0x31,

    /// Interrupt threshold registers
    INT1_THS_XH_G = 0x32,
    INT1_THS_XL_G = 0x33,
    INT1_THS_YH_G = 0x34,
    INT1_THS_YL_G = 0x35,
    INT1_THS_ZH_G = 0x36,
    INT1_THS_ZL_G = 0x37,

    /// Interrupt duration
    INT1_DURATION_G = 0x38,
}

impl GyroRegisters {
    /// Get the register address as u8
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

/// Accelerometer and Magnetometer register address map
#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelMagRegisters {
    /// Temperature output registers
    OUT_TEMP_L_XM = 0x05,
    OUT_TEMP_H_XM = 0x06,

    /// Magnetometer status register
    STATUS_REG_M = 0x07,

    /// Magnetometer output registers
    OUT_X_L_M = 0x08,
    OUT_X_H_M = 0x09,
    OUT_Y_L_M = 0x0A,
    OUT_Y_H_M = 0x0B,
    OUT_Z_L_M = 0x0C,
    OUT_Z_H_M = 0x0D,

    /// Device identification register
    WHO_AM_I_XM = 0x0F,

    /// Magnetometer interrupt control
    INT_CTRL_REG_M = 0x12,
    INT_SRC_REG_M = 0x13,

    /// Magnetometer interrupt threshold
    INT_THS_L_M = 0x14,
    INT_THS_H_M = 0x15,

    /// Magnetometer offset registers
    OFFSET_X_L_M = 0x16,
    OFFSET_X_H_M = 0x17,
    OFFSET_Y_L_M = 0x18,
    OFFSET_Y_H_M = 0x19,
    OFFSET_Z_L_M = 0x1A,
    OFFSET_Z_H_M = 0x1B,

    /// Reference values for high-pass filter
    REFERENCE_X = 0x1C,
    REFERENCE_Y = 0x1D,
    REFERENCE_Z = 0x1E,

    /// Control registers
    CTRL_REG0_XM = 0x1F,
    CTRL_REG1_XM = 0x20,
    CTRL_REG2_XM = 0x21,
    CTRL_REG3_XM = 0x22,
    CTRL_REG4_XM = 0x23,
    CTRL_REG5_XM = 0x24,
    CTRL_REG6_XM = 0x25,
    CTRL_REG7_XM = 0x26,

    /// Accelerometer status register
    STATUS_REG_A = 0x27,

    /// Accelerometer output registers
    OUT_X_L_A = 0x28,
    OUT_X_H_A = 0x29,
    OUT_Y_L_A = 0x2A,
    OUT_Y_H_A = 0x2B,
    OUT_Z_L_A = 0x2C,
    OUT_Z_H_A = 0x2D,

    /// FIFO control
    FIFO_CTRL_REG = 0x2E,
    FIFO_SRC_REG = 0x2F,

    /// Interrupt generators
    INT_GEN_1_REG = 0x30,
    INT_GEN_1_SRC = 0x31,
    INT_GEN_1_THS = 0x32,
    INT_GEN_1_DURATION = 0x33,

    INT_GEN_2_REG = 0x34,
    INT_GEN_2_SRC = 0x35,
    INT_GEN_2_THS = 0x36,
    INT_GEN_2_DURATION = 0x37,

    /// Click detection
    CLICK_CFG = 0x38,
    CLICK_SRC = 0x39,
    CLICK_THS = 0x3A,
    TIME_LIMIT = 0x3B,
    TIME_LATENCY = 0x3C,
    TIME_WINDOW = 0x3D,

    /// Activity threshold and duration
    ACT_THS = 0x3E,
    ACT_DUR = 0x3F,
}

impl AccelMagRegisters {
    /// Get the register address as u8
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

// ============================================================================
// DEVICE CONSTANTS
// ============================================================================

/// Common register addresses and device IDs
pub mod device_constants {
    /// Gyroscope I2C addresses
    pub mod gyro {
        /// Default I2C address when SDO_G/SA0_G = 0
        pub const I2C_ADDR_0: u8 = 0x6A;
        /// Alternative I2C address when SDO_G/SA0_G = 1
        pub const I2C_ADDR_1: u8 = 0x6B;
        /// Expected WHO_AM_I value
        pub const DEVICE_ID: u8 = 0xD4;
    }

    /// Accelerometer/Magnetometer I2C addresses
    pub mod xm {
        /// Default I2C address when SDO_XM/SA0_XM = 0
        pub const I2C_ADDR_0: u8 = 0x1E;
        /// Alternative I2C address when SDO_XM/SA0_XM = 1
        pub const I2C_ADDR_1: u8 = 0x1D;
        /// Expected WHO_AM_I value
        pub const DEVICE_ID: u8 = 0x49;
    }
}
