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
    /// let area = Aabb::try_new((10.0, 0.0), (20.0, 10.0)).expect("Invalid aabb");
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

        if upper_left.x >= lower_right.x || upper_left.y >= lower_right.y {
            Err(())
        } else {
            Ok(Self {
                upper_left,
                lower_right,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn try_new_errors_for_equal_points() {
        assert!(Aabb::try_new((10.0, 10.0), (10.0, 10.0)).is_err());
    }

    #[test]
    fn try_new_errors_when_upper_left_is_larger_than_lower_right() {
        assert!(Aabb::try_new((10.0, 10.0), (0.0, 0.0)).is_err());
    }

    #[test]
    fn try_new_errors_when_upper_left_x_is_larger_than_lower_right_x() {
        assert!(Aabb::try_new((10.0, 0.0), (0.0, 5.5)).is_err());
    }

    #[test]
    fn try_new_errors_when_upper_left_y_is_larger_than_lower_right_y() {
        assert!(Aabb::try_new((0.0, 10.0), (5.0, 0.0)).is_err());
    }
}
