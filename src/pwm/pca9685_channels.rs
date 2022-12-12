use super::Position;

pub struct PCA9685Ch {
    id: usize,
    position: Position,
}

impl PCA9685Ch {
    pub fn new(id: usize, position: Position) -> Self {
        Self { id, position }
    }
}

impl super::PWMChannelId for PCA9685Ch {
    fn position(&self) -> Position {
        self.position
    }

    fn id(&self) -> usize {
        self.id
    }
}
