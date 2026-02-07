//! Sensor measurement types with unit semantics.

/// Sensor orientation during calibration.
///
/// This enum specifies which axis is pointing "up" (against gravity) during bias calibration.
/// The calibration routine uses this to subtract the expected 1g gravity reading from the
/// accelerometer bias calculation, so that the resulting bias represents only sensor error.
///
/// Use [`Orientation::Unknown`] if the orientation is not known or if you want pose-relative
/// readings (accelerometer will read ~0 in the calibration pose).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Orientation {
    /// +X axis points up (gravity reads +1g on X)
    XUp,
    /// -X axis points up (gravity reads -1g on X)
    XDown,
    /// +Y axis points up (gravity reads +1g on Y)
    YUp,
    /// -Y axis points up (gravity reads -1g on Y)
    YDown,
    /// +Z axis points up (gravity reads +1g on Z)
    ZUp,
    /// -Z axis points up (gravity reads -1g on Z)
    ZDown,
    /// Unknown orientation - no gravity compensation applied.
    /// Accelerometer bias will include gravity, so readings will be ~0 in calibration pose.
    #[default]
    Unknown,
}

impl Orientation {
    /// Returns the expected gravity vector (in g) for this orientation.
    ///
    /// This is subtracted from the raw accelerometer average during calibration
    /// to isolate sensor bias from gravity.
    pub fn gravity_vector(self) -> (f32, f32, f32) {
        match self {
            Orientation::XUp => (1.0, 0.0, 0.0),
            Orientation::XDown => (-1.0, 0.0, 0.0),
            Orientation::YUp => (0.0, 1.0, 0.0),
            Orientation::YDown => (0.0, -1.0, 0.0),
            Orientation::ZUp => (0.0, 0.0, 1.0),
            Orientation::ZDown => (0.0, 0.0, -1.0),
            Orientation::Unknown => (0.0, 0.0, 0.0),
        }
    }
}

/// Acceleration measurement in g-force (1g ≈ 9.80665 m/s²).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GForce(f32);

impl GForce {
    /// Create a new g-force measurement.
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw f32 value.
    pub const fn as_f32(self) -> f32 {
        self.0
    }

    /// Convert to meters per second squared.
    pub fn to_meters_per_second_squared(self) -> f32 {
        self.0 * 9.80665
    }
}

impl From<GForce> for f32 {
    fn from(g: GForce) -> Self {
        g.0
    }
}

/// Magnetic field measurement in gauss.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gauss(f32);

impl Gauss {
    /// Create a new gauss measurement.
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw f32 value.
    pub const fn as_f32(self) -> f32 {
        self.0
    }

    /// Convert to tesla (1 gauss = 0.0001 tesla).
    pub fn to_tesla(self) -> f32 {
        self.0 * 0.0001
    }

    /// Convert to microtesla (1 gauss = 100 µT).
    pub fn to_microtesla(self) -> f32 {
        self.0 * 100.0
    }
}

impl From<Gauss> for f32 {
    fn from(g: Gauss) -> Self {
        g.0
    }
}

/// Angular velocity measurement in degrees per second.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DegreesPerSecond(f32);

impl DegreesPerSecond {
    /// Create a new angular velocity measurement.
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw f32 value.
    pub const fn as_f32(self) -> f32 {
        self.0
    }

    /// Convert to radians per second.
    pub fn to_radians_per_second(self) -> f32 {
        self.0 * core::f32::consts::PI / 180.0
    }
}

impl From<DegreesPerSecond> for f32 {
    fn from(dps: DegreesPerSecond) -> Self {
        dps.0
    }
}

/// Temperature measurement in Celsius.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Celsius(f32);

impl Celsius {
    /// Create a new temperature measurement.
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw f32 value.
    pub const fn as_f32(self) -> f32 {
        self.0
    }

    /// Convert to Fahrenheit.
    pub fn to_fahrenheit(self) -> f32 {
        self.0 * 9.0 / 5.0 + 32.0
    }

    /// Convert to Kelvin.
    pub fn to_kelvin(self) -> f32 {
        self.0 + 273.15
    }
}

impl From<Celsius> for f32 {
    fn from(c: Celsius) -> Self {
        c.0
    }
}
