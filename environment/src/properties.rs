pub trait Locatable {
    fn x(&self) -> u32;
    fn y(&self) -> u32;
}

pub trait Rectangle {
    fn length(&self) -> u32;
    fn width(&self) -> u32;
}

pub trait Collidable: Locatable + Rectangle {
    fn on_collision(&mut self, other: &dyn Collidable);
}