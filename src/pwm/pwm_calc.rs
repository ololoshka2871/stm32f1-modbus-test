use super::{pwm_ctrl_ext::PWMValues, PWMCtrlExt};
use crate::support;

impl PWMCtrlExt<20> for support::DataStorage {
    fn process(&mut self) -> Option<PWMValues<20>> {
        if self.modified {
            self.modified = false;

            Some(PWMValues([0u32; 20]))
        } else {
            None
        }
    }
}
