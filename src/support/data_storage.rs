use core::mem::size_of;

use byte::{BytesExt, BE};
use libremodbus_rs::{AccessMode, MbError};
use stm32f1xx_hal::time::{Hertz, Hz};

use super::num_ext::NumExt;

static VERSION: &str = env!("CARGO_PKG_VERSION");

pub struct DataStorage {
    pub pwm_base_freq: Hertz,
    pub channels_target: [u16; 20],

    pub total_load: f32,
    pub chank_load: [f32; 10],

    pub pwm_modified: bool,
    pub freq_modified: bool,
}

impl DataStorage {
    pub fn new() -> Self {
        Self {
            pwm_base_freq: Hz(1000u32),
            channels_target: Default::default(),
            total_load: 0.0,
            chank_load: Default::default(),
            pwm_modified: false,
            freq_modified: false,
        }
    }
}

impl libremodbus_rs::DataInterface for DataStorage {
    fn read_inputs(
        &mut self,
        reg_buff: &mut [u8],
        reg_addr: u16,
        reg_num: u16,
    ) -> Result<(), MbError> {
        match reg_addr {
            0x0000 | 0x0001 => {
                // total_load
                if (reg_num as usize) <= size_of::<u32>() / size_of::<u16>() {
                    return write_buf(
                        reg_buff,
                        self.total_load
                            .split_2u16()
                            .iter()
                            .skip(reg_addr as usize)
                            .take(reg_num as usize)
                            .copied(),
                    );
                }
            }
            0x100..=0x0113 => {
                // chank_load
                if (reg_addr - 0x100 + reg_num) as usize
                    <= size_of::<u32>() / size_of::<u16>() * self.chank_load.len()
                {
                    return write_buf(
                        reg_buff,
                        self.chank_load
                            .iter()
                            .map(|v| v.split_2u16())
                            .flatten()
                            .skip(reg_addr as usize - 0x100)
                            .take(reg_num as usize),
                    );
                }
            }
            _ => {}
        };
        Err(MbError::MB_ENOREG)
    }

    fn rw_holdings(
        &mut self,
        reg_buff: &mut [u8],
        reg_addr: u16,
        reg_num: u16,
        mode: AccessMode,
    ) -> Result<(), MbError> {
        match mode {
            AccessMode::READ => {
                match reg_addr {
                    0 => {
                        // ID
                        if reg_num > 1 {
                            Err(MbError::MB_ENOREG)?;
                        }
                        reg_buff
                            .write_with::<u16>(&mut 0, crate::config::MB_DEV_ID, BE)
                            .map_err(|_| MbError::MB_EIO)?;
                    }
                    0x0010..=0x0013 => {
                        // Device HW version
                        // MCU ID code
                        if (reg_addr - 0x0010 + reg_num) as usize > 2 * size_of::<u32>() {
                            Err(MbError::MB_ENOREG)?;
                        }

                        let hw_ver = crate::config::HW_VERSION.split_2u16();
                        let mut mcu_id = [0u8; size_of::<u32>()];
                        mcu_id.copy_from_slice(
                            &stm32_device_signature::device_id()[..size_of::<u32>()],
                        );
                        let mcu_id = u32::from_be_bytes(mcu_id); // вывернуть так чтобы читать было удобнее

                        let mut data = [0u16; 2 * size_of::<u32>() / size_of::<u16>()];
                        (&mut data[..size_of::<u32>() / size_of::<u16>()])
                            .clone_from_slice(&hw_ver);
                        (&mut data[size_of::<u32>() / size_of::<u16>()..])
                            .clone_from_slice(&mcu_id.split_2u16());

                        return write_buf(
                            reg_buff,
                            data.iter()
                                .skip(reg_addr as usize - 0x0010)
                                .take(reg_num as usize)
                                .copied(),
                        );
                    }
                    0x0020 | 0x0021 => {
                        // SW ver
                        if (reg_addr - 0x0020 + reg_num) as usize > size_of::<u32>() {
                            Err(MbError::MB_ENOREG)?;
                        }

                        let mut vers = VERSION.split('.').map(|v| {
                            let v = u32::from_str_radix(v, 10).unwrap();
                            if v > u8::MAX as u32 {
                                u8::MAX
                            } else {
                                v as u8
                            }
                        });
                        let mut vers_u32 = 0u32;
                        for offs in (0..32).step_by(u8::BITS as usize) {
                            if let Some(v) = vers.next() {
                                vers_u32 |= (v as u32) << (24 - offs);
                            }
                        }

                        return write_buf(
                            reg_buff,
                            vers_u32
                                .split_2u16()
                                .iter()
                                .skip(reg_addr as usize - 0x0020)
                                .take(reg_num as usize)
                                .copied(),
                        );
                    }
                    0x0100..=0x0113 => {
                        // channels duty
                        if (reg_addr - 0x0100 + reg_num) as usize > 20 {
                            Err(MbError::MB_ENOREG)?;
                        }

                        return write_buf(
                            reg_buff,
                            self.channels_target
                                .iter()
                                .skip(reg_addr as usize - 0x0100)
                                .take(reg_num as usize)
                                .copied(),
                        );
                    }
                    0x0200 => {
                        // PWM freq
                        if reg_num > 1 {
                            Err(MbError::MB_ENOREG)?;
                        }
                        reg_buff
                            .write_with::<u16>(&mut 0, self.pwm_base_freq.to_Hz() as u16, BE)
                            .map_err(|_| MbError::MB_EIO)?;
                    }
                    _ => Err(MbError::MB_ENOREG)?,
                }
                Ok(())
            }
            AccessMode::WRITE => {
                match reg_addr {
                    0x0100..=0x0113 => {
                        // channels duty
                        if (reg_addr - 0x0100 + reg_num) as usize > 20 {
                            Err(MbError::MB_ENOREG)?;
                        }

                        let offset = &mut 0;
                        for reg in (reg_addr - 0x0100)..(reg_addr - 0x0100 + reg_num) {
                            let v = reg_buff
                                .read_with::<u16>(offset, BE)
                                .map_err(|_| MbError::MB_EIO)?;
                            if v <= crate::config::MAX_PWM_VAL {
                                self.channels_target[reg as usize] = v;
                            } else {
                                return Err(MbError::MB_EINVAL);
                            }
                        }
                        self.pwm_modified = true;
                        return Ok(());
                    }
                    0x0200 => {
                        // PWM freq
                        if reg_num > 1 {
                            Err(MbError::MB_ENOREG)?;
                        }
                        let f = reg_buff
                            .read_with::<u16>(&mut 0, BE)
                            .map_err(|_| MbError::MB_EIO)?
                            .into();
                        if f >= crate::config::MIN_PWM_FREQ && f <= crate::config::MAX_PWM_FREQ {
                            self.pwm_base_freq = Hertz::Hz(f);
                            self.freq_modified = true;
                            return Ok(());
                        } else {
                            return Err(MbError::MB_EINVAL);
                        }
                    }
                    _ => {}
                }

                Err(MbError::MB_EINVAL)
            }
        }
    }
}

fn write_buf(target: &mut [u8], mut src: impl Iterator<Item = u16>) -> Result<(), MbError> {
    let offset = &mut 0;
    while let Some(v) = src.next() {
        target
            .write_with::<u16>(offset, v, BE)
            .map_err(|_| MbError::MB_EIO)?
    }
    Ok(())
}
