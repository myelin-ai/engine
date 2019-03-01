use crate::{Intersects, Point};

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

impl Intersects for Aabb {
    /// Returns wether the bounds of another `Aabb` are touching or
    /// inside this `Aabb`.
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
    fn intersects(&self, other: &Aabb) -> bool {
        let x_overlaps =
            self.upper_left.x <= other.lower_right.x && self.lower_right.x >= other.upper_left.x;
        let y_overlaps =
            self.upper_left.y <= other.lower_right.y && self.lower_right.y >= other.upper_left.y;

        x_overlaps || y_overlaps
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

    #[test]
    fn intersects_self() {
        let aabb = Aabb::try_new((0.0, 0.0), (10.0, 10.0)).unwrap();
        assert!(aabb.intersects(&aabb));
    }

    #[test]
    fn intersects_contained() {
        let bigger_aabb = Aabb::try_new((0.0, 0.0), (10.0, 10.0)).unwrap();
        let smaller_aabb = Aabb::try_new((2.0, 2.0), (8.0, 8.0)).unwrap();
        assert!(bigger_aabb.intersects(&smaller_aabb));
        assert!(smaller_aabb.intersects(&bigger_aabb));
    }

    #[test]
    fn intersects_touching() {
        let left_aabb = Aabb::try_new((0.0, 0.0), (10.0, 10.0)).unwrap();
        let right_aabb = Aabb::try_new((10.0, 0.0), (20.0, 10.0)).unwrap();
        assert!(left_aabb.intersects(&right_aabb));
        assert!(right_aabb.intersects(&left_aabb));
    }

    #[test]
    fn intersects_diagonally_touching() {
        let left_aabb = Aabb::try_new((0.0, 0.0), (10.0, 10.0)).unwrap();
        let right_aabb = Aabb::try_new((10.0, 10.0), (20.0, 11.0)).unwrap();
        assert!(left_aabb.intersects(&right_aabb));
        assert!(right_aabb.intersects(&left_aabb));
    }

    #[test]
    fn intersects_intersecting() {
        let first_aabb = Aabb::try_new((0.0, 0.0), (10.0, 10.0)).unwrap();
        let second_aabb = Aabb::try_new((8.0, 8.0), (20.0, 20.0)).unwrap();
        assert!(first_aabb.intersects(&second_aabb));
        assert!(second_aabb.intersects(&first_aabb));
    }

    #[test]
    fn intersects_intersecting_when_negative() {
        let first_aabb = Aabb::try_new((-10.0, -10.0), (-5.0, -5.0)).unwrap();
        let second_aabb = Aabb::try_new((-6.0, -20.0), (-3.0, -3.0)).unwrap();
        assert!(first_aabb.intersects(&second_aabb));
        assert!(second_aabb.intersects(&first_aabb));
    }

    #[test]
    fn intersects_intersecting_when_negative_and_positive() {
        let first_aabb = Aabb::try_new((-5.0, -5.0), (5.0, 5.0)).unwrap();
        let second_aabb = Aabb::try_new((-6.0, -20.0), (0.0, 2.0)).unwrap();
        assert!(first_aabb.intersects(&second_aabb));
        assert!(second_aabb.intersects(&first_aabb));
    }

    #[test]
    fn does_not_intersect_when_appart() {
        let first_aabb = Aabb::try_new((0.0, 0.0), (10.0, 10.0)).unwrap();
        let second_aabb = Aabb::try_new((20.0, 0.0), (21.0, 20.0)).unwrap();
        assert!(first_aabb.intersects(&second_aabb));
        assert!(second_aabb.intersects(&first_aabb));
    }
}
