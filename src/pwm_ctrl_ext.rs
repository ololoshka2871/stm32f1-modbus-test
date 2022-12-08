pub struct PWMValues<const N: usize>(pub [u32; N]);

pub trait PWMCtrlExt<const N: usize> {
    fn process(&mut self) -> PWMValues<N>;
}
