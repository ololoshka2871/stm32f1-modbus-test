use fugit_timer::HertzU32;
use libm::roundf;

use super::PWMChannelId;

pub struct PWMValues<const N: usize>(pub [u16; N]);

pub trait PWMCtrlExt<const N: usize> {
    fn process(&mut self, channels: &[&dyn PWMChannelId; N]) -> Option<(PWMValues<N>, HertzU32)>;
}

impl<const N: usize> PWMValues<N> {
    pub fn as_range(&self, channel: usize, src_resolution: u16, target_resolution: u16) -> u16 {
        assert!(channel < N);

        roundf(self.0[channel] as f32 / src_resolution as f32 * target_resolution as f32) as u16
    }
}
