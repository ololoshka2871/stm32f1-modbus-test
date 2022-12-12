#[derive(Clone, Copy, PartialEq)]
pub enum Position {
    LeftAligned,
    RightAligend,
}

pub trait PWMChannelId: Send {
    fn position(&self) -> Position;
    fn id(&self) -> usize;
}
