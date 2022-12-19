use fugit_timer::HertzU32;
use libm::roundf;

use super::PWMChannelId;

pub struct PWMValues<const N: usize> {
    pub values: [u16; N],
}

pub trait PWMCtrlExt<const N: usize> {
    fn process(
        &mut self,
        channels: &[&dyn PWMChannelId; N],
    ) -> (Option<PWMValues<N>>, Option<HertzU32>);
}

impl<const N: usize> PWMValues<N> {
    pub fn as_range(
        &self,
        channel: usize,
        src_resolution: u16,
        target_resolution: u16,
        is_inverted: bool,
    ) -> u16 {
        assert!(channel < N);

        let res =
            roundf(self.values[channel] as f32 / src_resolution as f32 * target_resolution as f32)
                as u16;
        if is_inverted {
            target_resolution - res
        } else {
            res
        }
    }
}

impl<const N: usize> Default for PWMValues<N> {
    fn default() -> Self {
        unsafe { core::mem::MaybeUninit::uninit().assume_init() }
    }
}
