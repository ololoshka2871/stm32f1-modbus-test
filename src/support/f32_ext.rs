pub trait F32Ext {
    fn split_2u16(&self) -> [u16; 2];
}

impl F32Ext for f32 {
    fn split_2u16(&self) -> [u16; 2] {
        unsafe { core::mem::transmute(self.to_ne_bytes()) }
    }
}

impl F32Ext for u32 {
    fn split_2u16(&self) -> [u16; 2] {
        unsafe { core::mem::transmute(self.to_be_bytes()) }
    }
}
