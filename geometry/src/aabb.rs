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
    /// let area = Aabb::new((10.0, 10.0), (20.0, 0.0));
    /// ```
    ///
    /// ## From points
    /// ```
    /// use myelin_geometry::{Aabb, Point};
    ///
    /// let area = Aabb::new(Point { x: 0.0, y: 10.0 }, Point { x: 20.0, y: 20.0 });
    /// ```
    ///
    /// [`Aabb`]: ./struct.Aabb.html
    pub fn new<P1, P2>(upper_left: P1, lower_right: P2) -> Self
    where
        P1: Into<Point>,
        P2: Into<Point>,
    {
        Self {
            upper_left: upper_left.into(),
            lower_right: lower_right.into(),
        }
    }

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
    pub fn intersects(&self, other: Aabb) -> bool {
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
    fn intersects_self() {
        let aabb = Aabb::new((0.0, 0.0), (10.0, 10.0));
        assert!(aabb.intersects(aabb));
    }

    #[test]
    fn intersects_contained() {
        let bigger_aabb = Aabb::new((0.0, 0.0), (10.0, 10.0));
        let smaller_aabb = Aabb::new((2.0, 2.0), (8.0, 8.0));
        assert!(bigger_aabb.intersects(smaller_aabb));
        assert!(smaller_aabb.intersects(bigger_aabb));
    }

    #[test]
    fn intersects_touching() {
        let left_aabb = Aabb::new((0.0, 0.0), (10.0, 10.0));
        let right_aabb = Aabb::new((10.0, 0.0), (20.0, 10.0));
        assert!(left_aabb.intersects(right_aabb));
        assert!(right_aabb.intersects(left_aabb));
    }

    #[test]
    fn intersects_diagonally_touching() {
        let left_aabb = Aabb::new((0.0, 0.0), (10.0, 10.0));
        let right_aabb = Aabb::new((10.0, 10.0), (20.0, 11.0));
        assert!(left_aabb.intersects(right_aabb));
        assert!(right_aabb.intersects(left_aabb));
    }

    #[test]
    fn intersects_intersecting() {
        let first_aabb = Aabb::new((0.0, 0.0), (10.0, 10.0));
        let second_aabb = Aabb::new((8.0, 8.0), (20.0, 20.0));
        assert!(first_aabb.intersects(second_aabb));
        assert!(second_aabb.intersects(first_aabb));
    }

    #[test]
    fn intersects_intersecting_when_negative() {
        let first_aabb = Aabb::new((-10.0, -10.0), (-5.0, -5.0));
        let second_aabb = Aabb::new((-6.0, -20.0), (-3.0, -3.0));
        assert!(first_aabb.intersects(second_aabb));
        assert!(second_aabb.intersects(first_aabb));
    }

    #[test]
    fn intersects_intersecting_when_negative_and_positive() {
        let first_aabb = Aabb::new((-5.0, -5.0), (5.0, 5.0));
        let second_aabb = Aabb::new((-6.0, -20.0), (0.0, 2.0));
        assert!(first_aabb.intersects(second_aabb));
        assert!(second_aabb.intersects(first_aabb));
    }

    #[test]
    fn does_not_intersect_when_appart() {
        let first_aabb = Aabb::new((0.0, 0.0), (10.0, 10.0));
        let second_aabb = Aabb::new((20.0, 0.0), (21.0, 20.0));
        assert!(first_aabb.intersects(second_aabb));
        assert!(second_aabb.intersects(first_aabb));
    }
}
