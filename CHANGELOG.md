# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
