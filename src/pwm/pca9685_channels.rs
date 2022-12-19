use embedded_hal::blocking::i2c::{Write, WriteRead};
use pwm_pca9685::{Channel, Error, Pca9685};

use super::Position;

pub struct PCA9685Ch {
    channel: Channel,
    position: Position,
    current_target: u16,
}

impl PCA9685Ch {
    pub fn new(id: Channel, position: Position) -> Self {
        Self {
            channel: id,
            position,
            current_target: 0,
        }
    }

    pub fn prepare<T, E>(&mut self, controller: &mut Pca9685<T>) -> Result<(), Error<E>>
    where
        T: Write<Error = E> + WriteRead<Error = E>,
    {
        controller.set_channel_on(self.channel, 0)?;
        controller.set_channel_off(self.channel, 0)?;
        self.current_target = 0;
        Ok(())
    }

    pub fn configure<T, E>(
        &mut self,
        controller: &mut Pca9685<T>,
        target: u16,
    ) -> Result<(), Error<E>>
    where
        T: Write<Error = E> + WriteRead<Error = E>,
    {
        if self.current_target != target {
            match self.position {
                Position::LeftAligned => {
                    controller.set_channel_off(self.channel, target)?;
                }
                Position::RightAligend => {
                    controller.set_channel_on(
                        self.channel,
                        if target == 0 {
                            0
                        } else {
                            crate::config::MAX_PWM_VAL - target
                        },
                    )?;
                }
            }
            self.current_target = target;
        }

        Ok(())
    }
}

impl super::PWMChannelId for PCA9685Ch {
    fn position(&self) -> Position {
        self.position
    }

    fn id(&self) -> usize {
        self.channel as usize
    }
}
