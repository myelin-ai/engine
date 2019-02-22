use crate::Point;

/// An axix-aligned bounding box
///
/// ```other
/// ┼─────────────────────────────────────── x
/// │
/// │  Upper left → ┌─────────────┐
/// │               │             │
/// │               │             │
/// │               └─────────────┘ ← Lower right
/// │
/// y
/// ```
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Aabb {
    /// The coordinates of the upper left corner of the box
    pub upper_left: Point,
    /// The coordinates of the lower right corner of the box
    pub lower_right: Point,
}

impl Aabb {
    /// Creates a new [`Aabb`] from two points.
    ///
    /// # Examples
    ///
    /// ## From tuples
    /// ```
    /// use myelin_geometry::Aabb;
    ///
    /// let area = Aabb::try_new((10.0, 10.0), (20.0, 0.0)).expect("Invalid aabb");
    /// ```
    ///
    /// ## From points
    /// ```
    /// use myelin_geometry::{Aabb, Point};
    ///
    /// let area =
    ///     Aabb::try_new(Point { x: 0.0, y: 10.0 }, Point { x: 20.0, y: 20.0 }).expect("Invalid aabb");
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error when both points are the same.
    ///
    /// [`Aabb`]: ./struct.Aabb.html
    pub fn try_new<P1, P2>(upper_left: P1, lower_right: P2) -> Result<Self, ()>
    where
        P1: Into<Point>,
        P2: Into<Point>,
    {
        let upper_left = upper_left.into();
        let lower_right = lower_right.into();

        if upper_left == lower_right {
            Err(())
        } else {
            Ok(Self {
                upper_left,
                lower_right,
            })
        }
    }
}
