use core::mem::size_of;

use byte::{BytesExt, BE};
use libremodbus_rs::{AccessMode, MbError};
use stm32f1xx_hal::time::{Hertz, Hz};

use super::f32_ext::F32Ext;

static VERSION: &str = env!("CARGO_PKG_VERSION");

pub struct DataStorage {
    pub pwm_base_freq: Hertz,
    pub channels_target: [u16; 20],

    pub total_load: f32,
    pub chank_load: [f32; 10],
}

impl DataStorage {
    pub fn new() -> Self {
        Self {
            pwm_base_freq: Hz(1000u32),
            channels_target: Default::default(),
            total_load: 0.0,
            chank_load: Default::default(),
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
            0 | 1 => {
                // total_load
                if reg_num + reg_num <= 2 {
                    let w = self.total_load.split_2u16();
                    return write_buf(reg_buff, w.iter().skip(reg_addr as usize).copied());
                }
            }
            0x100..=0x111 => {
                // chank_load
                if reg_addr - 0x0100 + reg_num <= 0x0111 {
                    let d = self
                        .chank_load
                        .iter()
                        .map(|v| v.split_2u16())
                        .flatten()
                        .skip(reg_addr as usize - 0x0100);

                    return write_buf(reg_buff, d);
                }
            }
            _ => {}
        };
        Ok(())
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
                    0x0010 | 0x0011 => {
                        // HW ver
                        if (reg_addr - 0x0010 + reg_num) as usize > size_of::<u32>() {
                            Err(MbError::MB_ENOREG)?;
                        }
                        let data = crate::config::HW_VERSION.split_2u16();

                        return write_buf(
                            reg_buff,
                            data.iter()
                                .skip(reg_addr as usize - 0x0010)
                                .take(reg_num as usize)
                                .copied(),
                        );
                    }
                    0x0012 | 0x0013 => {
                        // MCU_id
                        if (reg_addr - 0x0012 + reg_num) as usize > size_of::<u32>() {
                            Err(MbError::MB_ENOREG)?;
                        }
                        let id = stm32_device_signature::device_id();
                        let data: &[u16; 6] = unsafe { core::mem::transmute(id) };

                        return write_buf(
                            reg_buff,
                            data.iter()
                                .skip(reg_addr as usize - 0x0012)
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
            AccessMode::WRITE => Err(MbError::MB_EINVAL),
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
