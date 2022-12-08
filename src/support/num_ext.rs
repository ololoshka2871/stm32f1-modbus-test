pub trait NumExt {
    fn split_2u16(&self) -> [u16; 2];
}

impl NumExt for f32 {
    fn split_2u16(&self) -> [u16; 2] {
        unsafe { core::mem::transmute(self.to_ne_bytes()) }
    }
}

impl NumExt for u32 {
    fn split_2u16(&self) -> [u16; 2] {
        unsafe { core::mem::transmute(self.to_ne_bytes()) }
    }
}
