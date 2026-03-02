# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `#[must_use]` attribute on `GForce`, `Gauss`, `DegreesPerSecond`, and `Celsius` types to warn when sensor readings are discarded
- `read_all()` method and `SensorData` struct for reading all sensors in a single call, useful for sensor fusion algorithms
- `reapply_config()` method to re-synchronize shadow registers to hardware after bus errors or sensor resets
- `verify_config()` method and `Error::ConfigMismatch` variant for detecting register corruption via readback comparison
- `self_test()` method and `SelfTestResult` struct for hardware self-test of gyroscope and accelerometer against datasheet thresholds
- Integration tests for `reapply_config()`, `software_reset()`, and `set_gyro_power_mode()`, with shadow-register state assertions after each call
- Runnable doc examples for `with_gyro_int_wait`, `with_gyro_hpf`, `with_accel_hpf`, and `with_mag_int` config builder methods

### Changed

- Move unit tests to `lsm9ds0/src/tests.rs` to reduce `lsm9ds0/src/lib.rs` size and clutter
- Improved doc comments on `Lsm9ds0::new`, `read_gyro_raw`, `read_accel_raw`, `read_mag_raw`, and several config builder methods to clarify units, parameter meanings, and conversion patterns
- Fixed `ignore` doc examples (`init`, `software_reset`, `set_gyro_power_mode`, `calibrate_bias`) to `no_run` so they are at least checked during compilation
- Refactored `default_init_expectations()` test helper into `write_config_expectations()` + `default_init_expectations()` for reuse across tests

## [0.2.0] - 2026-02-07

### Added

- Blocking example showing how to implement async traits for blocking bus calls
- `defmt` feature to support debugging
- `From<bool>` implementation to reduce boilerplate
- More integration tests

### Fixed

- Magnetometer interrupt latch
- Calibration example in README
- Flush stale data during calibration
- Check device identity after software reset

### Changed

- Make i16 sensor reads more idiomatic using the type system instead of manual bit manipulation
- Re-export a more manageable set of types
- Calibration time now depends on sensor update speed

## [0.1.0] - 2026-02-03

### Added

- Initial release of LSM9DS0 driver and examples
