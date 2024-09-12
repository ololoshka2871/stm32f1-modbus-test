use fugit_timer::ExtU32;

pub struct Timer<TIM> {
    timer: TIM,
    timeout: fugit_timer::Duration<u32, 1, 1_000_000>,
}

impl<TIM> Timer<TIM> {
    pub fn new(timer: TIM) -> Self {
        Self {
            timer,
            timeout: 50u32.micros(),
        }
    }
}

impl<TIM> libremodbus_rs::TimerInterface for Timer<TIM>
where
    TIM: fugit_timer::Timer<1_000_000>,
{
    fn set_timeout(&mut self, timeout: fugit_timer::Duration<u32, 1, 1_000_000>) -> bool {
        self.timeout = timeout;
        true
    }

    fn close(&mut self) {
        let _ = self.timer.cancel();
    }

    fn start(&mut self) {
        let _ = self.timer.start(self.timeout);
    }

    fn stop(&mut self) {
        let _ = self.timer.cancel();
    }

    fn wait(&mut self) {
        let _ = self.timer.wait();
    }
}
