use super::PWMChannelId;

pub struct PWMValues<const N: usize>(pub [u16; N]);

pub trait PWMCtrlExt<const N: usize> {
    fn process(&mut self, channels: &[&dyn PWMChannelId; N]) -> Option<PWMValues<N>>;
}
