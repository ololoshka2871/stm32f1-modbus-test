pub const MCU_XTAL_HZ: u32 = 8_000_000;

//-----------------------------------------------------------------------------

pub const MB_DEV_ID: u16 = 0xDBFB;
pub const RS485_BOUD_RATE: u32 = 57_600;
pub const MODBUS_ADDR: u8 = 1;

pub const ADDR_BITS: [u8; 4] = [1, 2, 3, 7];

//-----------------------------------------------------------------------------

pub const SYSTICK_RATE_HZ: u32 = 1_000;

//-----------------------------------------------------------------------------

pub const HW_VERSION: u32 = 0x00000001;

//-----------------------------------------------------------------------------

pub const MIN_PWM_FREQ: u32 = 24; // pca9685 min
pub const MAX_PWM_FREQ: u32 = 1526; // pca9685 max
pub const MAX_PWM_VAL: u16 = (1 << 12) - 1;
