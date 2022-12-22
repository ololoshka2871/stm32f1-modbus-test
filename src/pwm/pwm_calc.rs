use fugit_timer::HertzU32;

use super::{pwm_ctrl_ext::PWMValues, PWMChannelId, PWMCtrlExt, Position};
use crate::support;

impl PWMCtrlExt<20> for support::DataStorage {
    fn process(
        &mut self,
        channels: &[&dyn PWMChannelId; 20],
    ) -> (Option<PWMValues<20>>, Option<HertzU32>, bool) {
        fn f((i, chank): (usize, &mut f32), ch_target: u32, chank_len: u32) {
            let chank_start = i as u32 * chank_len;
            let chank_end = chank_start + chank_len;
            let load = if ch_target > chank_end {
                chank_len
            } else if ch_target > chank_start {
                ch_target - chank_start
            } else {
                0
            };

            *chank += support::map(load as f32, 0.0, chank_len as f32, 0.0, 100.0);
        }

        let freq = if self.freq_modified {
            self.freq_modified = false;
            Some(self.pwm_base_freq)
        } else {
            None
        };

        let res = if self.pwm_modified || freq.is_some() {
            self.pwm_modified = false;
            let mut res = PWMValues::default();
            res.values.copy_from_slice(&self.channels_target);

            let chank_len =
                ((crate::config::MAX_PWM_VAL + 1) as usize / self.chank_load.len()) as u32;

            self.chank_load.fill(0.0); // reset load

            let mut total_load = 0;
            channels.iter().for_each(|ch| {
                let id = ch.id();
                let ch_target = self.channels_target[id] as u32;
                total_load += ch_target;

                match ch.position() {
                    Position::LeftAligned => {
                        self.chank_load
                            .iter_mut()
                            .enumerate()
                            .for_each(|a| f(a, ch_target, chank_len));
                    }
                    Position::RightAligend => {
                        self.chank_load
                            .iter_mut()
                            .rev()
                            .enumerate()
                            .for_each(|a| f(a, ch_target, chank_len));
                    }
                }
            });

            self.total_load = support::map(
                total_load as f32,
                0.0,
                (crate::config::MAX_PWM_VAL as usize * self.chank_load.len()) as f32,
                0.0,
                (100 * self.chank_load.len()) as f32,
            );

            Some(res)
        } else {
            None
        };

        (res, freq, freq.is_some())
    }
}
