use byte::*;
use libremodbus_rs::{AccessMode, MbError};

pub struct DataStorage();

impl DataStorage {
    pub fn new() -> Self {
        Self()
    }
}

impl libremodbus_rs::DataInterface for DataStorage {
    fn read_inputs(&mut self, reg_buff: &mut [u8], reg_addr: u16, reg_num: u16) -> MbError {
        let offset = &mut 0;
        for i in 0..reg_num {
            if !reg_buff.write_with::<u16>(offset, i + reg_addr, BE).is_ok() {
                return MbError::MB_EIO;
            }
        }

        MbError::MB_ENOERR
    }

    fn rw_holdings(
        &mut self,
        reg_buff: &mut [u8],
        reg_addr: u16,
        reg_num: u16,
        mode: AccessMode,
    ) -> MbError {
        match mode {
            AccessMode::READ => {
                let offset = &mut 0;
                for i in 0..reg_num {
                    if !reg_buff.write_with::<u16>(offset, reg_addr + i, BE).is_ok() {
                        return MbError::MB_EIO;
                    }
                }
                MbError::MB_ENOERR
            }
            AccessMode::WRITE => MbError::MB_EINVAL,
        }
    }
}
