/// Tests wether wether this shape touches, contains or is contained in another shape
/// ```other
/// ┼─────────────────────────────────────── x
/// │
/// │   ┌─────────────┐
/// │   │             │
/// │   │          ┌──│───────┐
/// │   └─────────────┘       │
/// │              └──────────┘
/// y
/// ```
pub trait Intersects<Other: ?Sized = Self> {
    /// Returns wether this shape touches, contains or is contained in another shape
    /// ```other
    /// ┼─────────────────────────────────────── x
    /// │
    /// │   ┌─────────────┐
    /// │   │             │
    /// │   │          ┌──│───────┐
    /// │   └─────────────┘       │
    /// │              └──────────┘
    /// y
    /// ```
    fn intersects(&self, other: &Other) -> bool;
}
