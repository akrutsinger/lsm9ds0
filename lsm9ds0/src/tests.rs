extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use registers::{AccelMagRegisters, CtrlReg0Xm, CtrlReg1G, CtrlReg4G, CtrlReg5G, GyroRegisters};
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

/// Build the I2C transaction expectations for `apply_configs()` with the given config.
/// This covers all register writes without the WHO_AM_I device ID reads.
fn write_config_expectations(config: &Lsm9ds0Config) -> std::vec::Vec<I2cTransaction> {
    vec![
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

/// Build the I2C transaction expectations for a full `init()` with default config.
fn default_init_expectations() -> std::vec::Vec<I2cTransaction> {
    let config = Lsm9ds0Config::default();
    let mut expectations = vec![
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
    ];
    expectations.extend(write_config_expectations(&config));
    expectations
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
    let expected_reg4: u8 = CtrlReg4G::new().with_fs(GyroScale::Dps2000).into();
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

// =========================================================================
// REAPPLY_CONFIG TEST
// =========================================================================

#[tokio::test]
async fn test_reapply_config() {
    // Use a non-default config so the expected register bytes are meaningfully non-zero.
    // If reapply_config() uses stale or default values, the mock will fail.
    let config = Lsm9ds0Config::new()
        .with_gyro_scale(GyroScale::Dps2000)
        .with_accel_scale(AccelScale::G8)
        .with_mag_scale(MagScale::Gauss8);
    let expectations = write_config_expectations(&config);

    let i2c = I2cMock::new(&expectations);
    let interface = I2cInterface::init(i2c);
    let mut driver = Lsm9ds0::new_with_config(interface, config);

    driver.reapply_config().await.unwrap();

    // Shadow config must be unchanged after a pure re-apply — sensitivities reflect the
    // scales we set, not the defaults, proving the stored config was used for the writes.
    assert_eq!(driver.config().gyro_sensitivity(), GyroScale::Dps2000.sensitivity());
    assert_eq!(driver.config().accel_sensitivity(), AccelScale::G8.sensitivity());
    assert_eq!(driver.config().mag_sensitivity(), MagScale::Gauss8.sensitivity());

    driver.release().release().done();
}

// =========================================================================
// SOFTWARE_RESET TEST
// =========================================================================

#[tokio::test]
async fn test_software_reset() {
    // software_reset() writes boot bits to both chips, delays, then re-verifies device IDs.
    // Default shadow registers are 0x00; boot is bit 7, so the written values are 0x80.
    let boot_val_g: u8 = CtrlReg5G::new().with_boot(Enable::Enabled).into();
    let boot_val_xm: u8 = CtrlReg0Xm::new().with_boot(Enable::Enabled).into();

    let expectations = vec![
        // Write boot bit to CTRL_REG5_G (single byte)
        I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG5_G.addr(), boot_val_g],
        ),
        // Write boot bit to CTRL_REG0_XM (single byte)
        I2cTransaction::write(
            XM_ADDR,
            vec![AccelMagRegisters::CTRL_REG0_XM.addr(), boot_val_xm],
        ),
        // After delay, verify_device_ids() re-reads WHO_AM_I registers
        I2cTransaction::write_read(
            GYRO_ADDR,
            vec![GyroRegisters::WHO_AM_I_G.addr()],
            vec![registers::device_constants::gyro::DEVICE_ID],
        ),
        I2cTransaction::write_read(
            XM_ADDR,
            vec![AccelMagRegisters::WHO_AM_I_XM.addr()],
            vec![registers::device_constants::xm::DEVICE_ID],
        ),
    ];

    let i2c = I2cMock::new(&expectations);
    let interface = I2cInterface::init(i2c);
    let mut driver = Lsm9ds0::new(interface);

    driver.software_reset(&mut MockDelay).await.unwrap();

    // After reset the driver calls `self.config = Lsm9ds0Config::default()`, so the boot
    // bits we set before the reset must be cleared in the shadow registers.
    assert_eq!(driver.config.ctrl_reg5_g.boot(), Enable::Disabled);
    assert_eq!(driver.config.ctrl_reg0_xm.boot(), Enable::Disabled);
    // Confirm the rest of the config reverted to defaults as well.
    assert_eq!(driver.config().gyro_sensitivity(), GyroScale::Dps245.sensitivity());

    driver.release().release().done();
}

// =========================================================================
// SET_GYRO_POWER_MODE TESTS
// =========================================================================

#[tokio::test]
async fn test_set_gyro_power_mode() {
    // CtrlReg1G layout (LSB→MSB): xen[0] yen[1] zen[2] pd[3] bw[5:4] dr[7:6]
    // Default: all zeros (power-down, no axes)
    // Sleep: pd=Normal(1), axes all disabled  → bit3=1 → 0x08
    // PowerDown: pd=PowerDown(0), axes unchanged → 0x00
    let sleep_val: u8 = CtrlReg1G::new()
        .with_pd(PowerMode::Normal)
        .with_xen(Enable::Disabled)
        .with_yen(Enable::Disabled)
        .with_zen(Enable::Disabled)
        .into();
    let power_down_val: u8 = CtrlReg1G::new().with_pd(PowerMode::PowerDown).into();

    let expectations = vec![
        // set_gyro_power_mode(Sleep) → write CTRL_REG1_G
        I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG1_G.addr(), sleep_val],
        ),
        // set_gyro_power_mode(PowerDown) → write CTRL_REG1_G
        I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG1_G.addr(), power_down_val],
        ),
    ];

    let i2c = I2cMock::new(&expectations);
    let interface = I2cInterface::init(i2c);
    let mut driver = Lsm9ds0::new(interface);

    driver
        .set_gyro_power_mode(GyroPowerMode::Sleep)
        .await
        .unwrap();
    // Sleep: pd=Normal(1), all axes explicitly disabled
    assert_eq!(driver.config.ctrl_reg1_g.pd(), PowerMode::Normal);
    assert_eq!(driver.config.ctrl_reg1_g.xen(), Enable::Disabled);
    assert_eq!(driver.config.ctrl_reg1_g.yen(), Enable::Disabled);
    assert_eq!(driver.config.ctrl_reg1_g.zen(), Enable::Disabled);

    driver
        .set_gyro_power_mode(GyroPowerMode::PowerDown)
        .await
        .unwrap();
    // PowerDown: pd=PowerDown(0)
    assert_eq!(driver.config.ctrl_reg1_g.pd(), PowerMode::PowerDown);

    driver.release().release().done();
}

// =========================================================================
// SET_GYRO_AXES REGRESSION TEST
//
// Regression: set_gyro_axes() must persist the axis selection into
// `gyro_axes_enabled` so that a subsequent set_gyro_power_mode(Normal)
// restores the correct axes and not the default (true, true, true).
// =========================================================================

#[tokio::test]
async fn test_set_gyro_axes_persists_for_power_mode_restore() {
    // Layout: xen[0] yen[1] zen[2] pd[3] bw[5:4] dr[7:6]
    // set_gyro_axes(false, false, true)  → zen=1             → 0x04
    // set_gyro_power_mode(Sleep)         → pd=1, axes all off → 0x08
    // set_gyro_power_mode(Normal)        → pd=1, restore (false,false,true) → 0x0C
    let axes_val: u8 = CtrlReg1G::new().with_zen(Enable::Enabled).into(); // 0x04
    let sleep_val: u8 = CtrlReg1G::new()
        .with_pd(PowerMode::Normal)
        .with_xen(Enable::Disabled)
        .with_yen(Enable::Disabled)
        .with_zen(Enable::Disabled)
        .into(); // 0x08
    let normal_val: u8 = CtrlReg1G::new()
        .with_pd(PowerMode::Normal)
        .with_xen(Enable::Disabled)
        .with_yen(Enable::Disabled)
        .with_zen(Enable::Enabled)
        .into(); // 0x0C

    let expectations = vec![
        I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG1_G.addr(), axes_val],
        ),
        I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG1_G.addr(), sleep_val],
        ),
        I2cTransaction::write(
            GYRO_ADDR,
            vec![GyroRegisters::CTRL_REG1_G.addr(), normal_val],
        ),
    ];

    let i2c = I2cMock::new(&expectations);
    let interface = I2cInterface::init(i2c);
    let mut driver = Lsm9ds0::new(interface);

    driver.set_gyro_axes(false, false, true).await.unwrap();
    driver
        .set_gyro_power_mode(GyroPowerMode::Sleep)
        .await
        .unwrap();
    driver
        .set_gyro_power_mode(GyroPowerMode::Normal)
        .await
        .unwrap();

    // The axis selection made before sleep must survive the sleep/wake cycle.
    // Without the gyro_axes_enabled fix, Normal would restore (true, true, true)
    // and xen/yen would be Enabled here — causing the mock and these assertions to fail.
    assert_eq!(driver.config.ctrl_reg1_g.xen(), Enable::Disabled);
    assert_eq!(driver.config.ctrl_reg1_g.yen(), Enable::Disabled);
    assert_eq!(driver.config.ctrl_reg1_g.zen(), Enable::Enabled);
    assert_eq!(driver.config.gyro_axes_enabled, (false, false, true));

    driver.release().release().done();
}
