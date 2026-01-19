//! Blocking I2C - no async, no problem. Just read out all the default values of the device
//! registers
//!
//! Run with: cargo run --bin i2c_blocking

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c::{self, Config};
use embedded_hal::i2c::I2c;
use lsm9ds0::registers::{AccelMagRegisters, GyroRegisters, device_constants};
use {defmt_rtt as _, panic_probe as _};

/// Register info: name, address, and expected default value
struct RegInfo {
    name: &'static str,
    addr: u8,
    default: u8,
}

/// Gyroscope registers with their default values (from datasheet)
const GYRO_REGISTERS: &[RegInfo] = &[
    RegInfo {
        name: "WHO_AM_I_G",
        addr: GyroRegisters::WHO_AM_I_G as u8,
        default: 0xD4,
    },
    RegInfo {
        name: "CTRL_REG1_G",
        addr: GyroRegisters::CTRL_REG1_G as u8,
        default: 0x07,
    },
    RegInfo {
        name: "CTRL_REG2_G",
        addr: GyroRegisters::CTRL_REG2_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG3_G",
        addr: GyroRegisters::CTRL_REG3_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG4_G",
        addr: GyroRegisters::CTRL_REG4_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG5_G",
        addr: GyroRegisters::CTRL_REG5_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "REFERENCE_G",
        addr: GyroRegisters::REFERENCE_G as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "STATUS_REG_G",
    //     addr: GyroRegisters::STATUS_REG_G as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_X_L_G",
    //     addr: GyroRegisters::OUT_X_L_G as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_X_H_G",
    //     addr: GyroRegisters::OUT_X_H_G as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Y_L_G",
    //     addr: GyroRegisters::OUT_Y_L_G as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Y_H_G",
    //     addr: GyroRegisters::OUT_Y_H_G as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Z_L_G",
    //     addr: GyroRegisters::OUT_Z_L_G as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Z_H_G",
    //     addr: GyroRegisters::OUT_Z_H_G as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "FIFO_CTRL_REG_G",
        addr: GyroRegisters::FIFO_CTRL_REG_G as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "FIFO_SRC_REG_G",
    //     addr: GyroRegisters::FIFO_SRC_REG_G as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "INT1_CFG_G",
        addr: GyroRegisters::INT1_CFG_G as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "INT1_SRC_G",
    //     addr: GyroRegisters::INT1_SRC_G as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "INT1_THS_XH_G",
        addr: GyroRegisters::INT1_THS_XH_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT1_THS_XL_G",
        addr: GyroRegisters::INT1_THS_XL_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT1_THS_YH_G",
        addr: GyroRegisters::INT1_THS_YH_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT1_THS_YL_G",
        addr: GyroRegisters::INT1_THS_YL_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT1_THS_ZH_G",
        addr: GyroRegisters::INT1_THS_ZH_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT1_THS_ZL_G",
        addr: GyroRegisters::INT1_THS_ZL_G as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT1_DURATION_G",
        addr: GyroRegisters::INT1_DURATION_G as u8,
        default: 0x00,
    },
];

/// Accelerometer/Magnetometer registers with their default values (from datasheet)
const XM_REGISTERS: &[RegInfo] = &[
    // RegInfo {
    //     name: "OUT_TEMP_L_XM",
    //     addr: AccelMagRegisters::OUT_TEMP_L_XM as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_TEMP_H_XM",
    //     addr: AccelMagRegisters::OUT_TEMP_H_XM as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "STATUS_REG_M",
    //     addr: AccelMagRegisters::STATUS_REG_M as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_X_L_M",
    //     addr: AccelMagRegisters::OUT_X_L_M as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_X_H_M",
    //     addr: AccelMagRegisters::OUT_X_H_M as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Y_L_M",
    //     addr: AccelMagRegisters::OUT_Y_L_M as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Y_H_M",
    //     addr: AccelMagRegisters::OUT_Y_H_M as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Z_L_M",
    //     addr: AccelMagRegisters::OUT_Z_L_M as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Z_H_M",
    //     addr: AccelMagRegisters::OUT_Z_H_M as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "WHO_AM_I_XM",
        addr: AccelMagRegisters::WHO_AM_I_XM as u8,
        default: 0x49,
    },
    RegInfo {
        name: "INT_CTRL_REG_M",
        addr: AccelMagRegisters::INT_CTRL_REG_M as u8,
        default: 0xE8,
    },
    // RegInfo {
    //     name: "INT_SRC_REG_M",
    //     addr: AccelMagRegisters::INT_SRC_REG_M as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "INT_THS_L_M",
        addr: AccelMagRegisters::INT_THS_L_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT_THS_H_M",
        addr: AccelMagRegisters::INT_THS_H_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "OFFSET_X_L_M",
        addr: AccelMagRegisters::OFFSET_X_L_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "OFFSET_X_H_M",
        addr: AccelMagRegisters::OFFSET_X_H_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "OFFSET_Y_L_M",
        addr: AccelMagRegisters::OFFSET_Y_L_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "OFFSET_Y_H_M",
        addr: AccelMagRegisters::OFFSET_Y_H_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "OFFSET_Z_L_M",
        addr: AccelMagRegisters::OFFSET_Z_L_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "OFFSET_Z_H_M",
        addr: AccelMagRegisters::OFFSET_Z_H_M as u8,
        default: 0x00,
    },
    RegInfo {
        name: "REFERENCE_X",
        addr: AccelMagRegisters::REFERENCE_X as u8,
        default: 0x00,
    },
    RegInfo {
        name: "REFERENCE_Y",
        addr: AccelMagRegisters::REFERENCE_Y as u8,
        default: 0x00,
    },
    RegInfo {
        name: "REFERENCE_Z",
        addr: AccelMagRegisters::REFERENCE_Z as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG0_XM",
        addr: AccelMagRegisters::CTRL_REG0_XM as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG1_XM",
        addr: AccelMagRegisters::CTRL_REG1_XM as u8,
        default: 0x07,
    },
    RegInfo {
        name: "CTRL_REG2_XM",
        addr: AccelMagRegisters::CTRL_REG2_XM as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG3_XM",
        addr: AccelMagRegisters::CTRL_REG3_XM as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG4_XM",
        addr: AccelMagRegisters::CTRL_REG4_XM as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CTRL_REG5_XM",
        addr: AccelMagRegisters::CTRL_REG5_XM as u8,
        default: 0x18,
    },
    RegInfo {
        name: "CTRL_REG6_XM",
        addr: AccelMagRegisters::CTRL_REG6_XM as u8,
        default: 0x20,
    },
    RegInfo {
        name: "CTRL_REG7_XM",
        addr: AccelMagRegisters::CTRL_REG7_XM as u8,
        default: 0x03,
    },
    // RegInfo {
    //     name: "STATUS_REG_A",
    //     addr: AccelMagRegisters::STATUS_REG_A as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_X_L_A",
    //     addr: AccelMagRegisters::OUT_X_L_A as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_X_H_A",
    //     addr: AccelMagRegisters::OUT_X_H_A as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Y_L_A",
    //     addr: AccelMagRegisters::OUT_Y_L_A as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Y_H_A",
    //     addr: AccelMagRegisters::OUT_Y_H_A as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Z_L_A",
    //     addr: AccelMagRegisters::OUT_Z_L_A as u8,
    //     default: 0x00,
    // },
    // RegInfo {
    //     name: "OUT_Z_H_A",
    //     addr: AccelMagRegisters::OUT_Z_H_A as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "FIFO_CTRL_REG",
        addr: AccelMagRegisters::FIFO_CTRL_REG as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "FIFO_SRC_REG",
    //     addr: AccelMagRegisters::FIFO_SRC_REG as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "INT_GEN_1_REG",
        addr: AccelMagRegisters::INT_GEN_1_REG as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "INT_GEN_1_SRC",
    //     addr: AccelMagRegisters::INT_GEN_1_SRC as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "INT_GEN_1_THS",
        addr: AccelMagRegisters::INT_GEN_1_THS as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT_GEN_1_DURATION",
        addr: AccelMagRegisters::INT_GEN_1_DURATION as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT_GEN_2_REG",
        addr: AccelMagRegisters::INT_GEN_2_REG as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "INT_GEN_2_SRC",
    //     addr: AccelMagRegisters::INT_GEN_2_SRC as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "INT_GEN_2_THS",
        addr: AccelMagRegisters::INT_GEN_2_THS as u8,
        default: 0x00,
    },
    RegInfo {
        name: "INT_GEN_2_DURATION",
        addr: AccelMagRegisters::INT_GEN_2_DURATION as u8,
        default: 0x00,
    },
    RegInfo {
        name: "CLICK_CFG",
        addr: AccelMagRegisters::CLICK_CFG as u8,
        default: 0x00,
    },
    // RegInfo {
    //     name: "CLICK_SRC",
    //     addr: AccelMagRegisters::CLICK_SRC as u8,
    //     default: 0x00,
    // },
    RegInfo {
        name: "CLICK_THS",
        addr: AccelMagRegisters::CLICK_THS as u8,
        default: 0x00,
    },
    RegInfo {
        name: "TIME_LIMIT",
        addr: AccelMagRegisters::TIME_LIMIT as u8,
        default: 0x00,
    },
    RegInfo {
        name: "TIME_LATENCY",
        addr: AccelMagRegisters::TIME_LATENCY as u8,
        default: 0x00,
    },
    RegInfo {
        name: "TIME_WINDOW",
        addr: AccelMagRegisters::TIME_WINDOW as u8,
        default: 0x00,
    },
    RegInfo {
        name: "ACT_THS",
        addr: AccelMagRegisters::ACT_THS as u8,
        default: 0x00,
    },
    RegInfo {
        name: "ACT_DUR",
        addr: AccelMagRegisters::ACT_DUR as u8,
        default: 0x00,
    },
];

/// Read a single register and return the value
fn read_register<I: I2c>(i2c: &mut I, device_addr: u8, reg_addr: u8) -> Option<u8> {
    let mut buf = [0u8; 1];
    match i2c.write_read(device_addr, &[reg_addr], &mut buf) {
        Ok(_) => Some(buf[0]),
        Err(_) => None,
    }
}

/// Read and print all registers for a device
fn dump_registers<I: I2c>(i2c: &mut I, device_name: &str, device_addr: u8, registers: &[RegInfo]) {
    info!("");
    info!("=== {} (I2C addr: 0x{:02X}) ===", device_name, device_addr);

    let mut match_count = 0;
    let mut total_count = 0;

    for reg in registers {
        total_count += 1;
        match read_register(i2c, device_addr, reg.addr) {
            Some(value) => {
                let matches = value == reg.default;
                if matches {
                    match_count += 1;
                    info!(
                        "{}: 0x{:02X} = 0x{:02X} (default: 0x{:02X}) OK",
                        reg.name, reg.addr, value, reg.default
                    );
                } else {
                    warn!(
                        "{}: 0x{:02X} = 0x{:02X} (default: 0x{:02X}) DIFF",
                        reg.name, reg.addr, value, reg.default
                    );
                }
            }
            None => {
                error!("{}: 0x{:02X} = READ ERROR", reg.name, reg.addr);
            }
        }
    }

    info!("");
    info!(
        "Summary: {}/{} registers match defaults",
        match_count, total_count
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("=== LSM9DS0 Register Dump ===");
    info!("Read all device registers and compare with datasheet defaults");
    info!("");
    info!("Using I2C1: GPIO14 = SDA, GPIO15 = SCL");

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    // Use blocking I2C (no interrupts)
    let mut i2c = i2c::I2c::new_blocking(p.I2C1, scl, sda, Config::default());

    info!("I2C initialized, scanning bus...");

    // Scan all valid I2C addresses
    let mut found_any = false;
    for addr in 0x08_u8..0x78 {
        let mut buf = [0u8; 1];
        if i2c.read(addr, &mut buf).is_ok() {
            info!("Found device at 0x{:02X}", addr);
            found_any = true;
        }
    }

    if !found_any {
        warn!("No devices found on I2C bus!");
    }

    // Get the I2C addresses from the library constants
    let gyro_addr = device_constants::gyro::I2C_ADDR_0;
    let xm_addr = device_constants::xm::I2C_ADDR_0;

    // Dump all gyroscope registers
    dump_registers(&mut i2c, "Gyroscope", gyro_addr, GYRO_REGISTERS);

    // Dump all accelerometer/magnetometer registers
    dump_registers(&mut i2c, "Accel/Mag", xm_addr, XM_REGISTERS);

    info!("");
    info!("=== Register dump complete ===");
}
