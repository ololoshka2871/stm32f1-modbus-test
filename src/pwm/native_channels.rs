use super::Position;

pub struct NativeCh {
    id: usize,
    position: Position,
}

impl NativeCh {
    pub fn new(id: usize, position: Position) -> Self {
        Self { id, position }
    }
}

impl super::PWMChannelId for NativeCh {
    fn position(&self) -> Position {
        self.position
    }

    fn id(&self) -> usize {
        self.id
    }
}
