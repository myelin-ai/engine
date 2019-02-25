/// Tests wether wether this shape touches, contains or is contained in another shape
pub trait Intersects<Other: ?Sized = Self> {
    /// Returns wether this shape touches, contains or is contained in another shape
    fn intersects(&self, other: &Other) -> bool;
}
