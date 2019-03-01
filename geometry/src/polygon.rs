//! Types relating to 2D convex polygons and their construction

pub use self::builder::*;
use super::*;
use crate::ConvexHull;
use crate::Intersects;
use itertools::Itertools;
use serde_derive::{Deserialize, Serialize};

mod builder;

/// A convex polygon.
///
/// Can either be constructed using a [`PolygonBuilder`]
/// or with [`Polygon::try_new`].
///
/// [`PolygonBuilder`]: ./struct.PolygonBuilder.html
/// [`Polygon::try_new`]: ./struct.Polygon.html#method.try_new
#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Polygon {
    /// The vertices of the polygon
    vertices: Vec<Point>,
}

impl Polygon {
    /// Creates a new [`Polygon`] from the given [`Point`]s.
    ///
    /// # Errors
    /// This method will return an error if the number of configured
    /// vertices is less than three (as the resulting [`Polygon`]
    /// would not be two-dimensional), or if the resulting [`Polygon`] is not convex.
    ///
    /// [`Polygon`]: ./struct.Polygon.html
    /// [`Point`]: ./struct.Point.html
    pub fn try_new(vertices: Vec<Point>) -> Result<Self, ()> {
        const MINIMUM_VERTICES_IN_EUCLIDEAN_GEOMETRY: usize = 3;

        if vertices.len() >= MINIMUM_VERTICES_IN_EUCLIDEAN_GEOMETRY && is_convex_polygon(&vertices)
        {
            Ok(Self { vertices })
        } else {
            Err(())
        }
    }

    /// Returns the vertices of the polygon
    pub fn vertices(&self) -> &[Point] {
        &self.vertices
    }

    /// Apply translation specified by `translation`, represented as
    /// a relative point
    pub fn translate(&self, translation: Point) -> Self {
        let translated_vertices = self
            .vertices
            .iter()
            .map(|&vertex| vertex + translation)
            .collect();

        Polygon {
            vertices: translated_vertices,
        }
    }

    /// Rotate polygon by a `rotation` around a `point`
    pub fn rotate_around_point(&self, rotation: Radians, point: Point) -> Self {
        let rotation = rotation.value();
        let rotated_vertices = self
            .vertices
            .iter()
            .map(|&vertex| {
                // See https://en.wikipedia.org/wiki/Rotation_matrix
                let delta = vertex - point;
                let (rotation_sin, rotation_cos) = rotation.sin_cos();
                let rotated_x = rotation_cos * delta.x + rotation_sin * delta.y + point.x;
                let rotated_y = -rotation_sin * delta.x + rotation_cos * delta.y + point.y;
                Point {
                    x: rotated_x,
                    y: rotated_y,
                }
            })
            .collect();
        Self {
            vertices: rotated_vertices,
        }
    }

    /// Checks if a given point rests inside the polygon
    pub fn contains_point(&self, point: Point) -> bool {
        // Anything less than a line cannot "contain" anything
        if self.vertices.len() < 2 {
            return false;
        }

        let vector_to_point: Vector = point.into();
        // The following unwraps are safe, as we do an
        // early return if we don't contain at least 2 vertices
        let vector_to_last_point: Vector = (*self.vertices.last().unwrap()).into();
        let vector_to_first_point: Vector = (*self.vertices.first().unwrap()).into();
        let reference_side =
            calculate_facing_side(vector_to_last_point, vector_to_first_point, vector_to_point);

        // If the point lies on the same side of all lines of the polygon,
        // the point is contained in the polygon.
        self.vertices
            .iter()
            .tuple_windows()
            .map(|(&vector_to_point_a, &vector_to_point_b)| {
                calculate_facing_side(
                    vector_to_point_a.into(),
                    vector_to_point_b.into(),
                    vector_to_point,
                )
            })
            .all(|side| side == reference_side || side == Side::OnTheLine)
    }

    /// Returns an [`Aabb`] which fully contains this polygon.
    ///
    /// # Panics
    /// Panics if the floating-point values representing the vertices' coordinates
    /// are not comparable, e.g. `NaN` or if the polygon has no vertices.
    /// The latter should never occur, because the constructor validates that the polygon is valid.
    pub fn aabb(&self) -> Aabb {
        let mut vertices = self.vertices.clone();

        // Safe unwrap: A polygon's vertex should not be baloney like NaN
        vertices.sort_unstable_by(|a, b| a.x.partial_cmp(&b.x).unwrap());
        // Safe unwraps: A polygon should always have at least one vertex
        let min_x = vertices.first().unwrap().x;
        let max_x = vertices.last().unwrap().x;

        vertices.sort_unstable_by(|a, b| a.y.partial_cmp(&b.y).unwrap());
        let min_y = vertices.first().unwrap().y;
        let max_y = vertices.last().unwrap().y;

        // Safe unwrap: A polygon where all four points are the same is not valid
        Aabb::try_new((min_x, min_y), (max_x, max_y)).unwrap()
    }

    /// Returns the polygon's edges, i.e. the lines between vertices, as vectors.
    pub fn edges(&self) -> impl Iterator<Item = Vector> + '_ {
        let vertices = self.vertices();
        let shifted_vertices = vertices.iter().cycle().skip(1).take(vertices.len());
        vertices
            .iter()
            .zip(shifted_vertices)
            .map(|(&first_vertex, &second_vertex)| second_vertex - first_vertex)
            .map(Vector::from)
    }

    fn scalar_project_onto_unit_vector(&self, axis: Vector) -> (f64, f64) {
        let projection: Vec<_> = self
            .vertices()
            .iter()
            .cloned()
            .map(Vector::from)
            .map(|vector| vector.dot_product(axis))
            .collect();
        (
            *projection
                .iter()
                .min_by(|lhs, rhs| lhs.partial_cmp(rhs).unwrap())
                .unwrap(),
            *projection
                .iter()
                .max_by(|lhs, rhs| lhs.partial_cmp(rhs).unwrap())
                .unwrap(),
        )
    }
}

impl Intersects for Polygon {
    /// Returns wether this polygon touches, contains or is contained in another polygon
    fn intersects(&self, other: &Polygon) -> bool {
        // The following codes describes the Separating Axis Theorem (SAT),
        // which states that if we are able to draw a straight line (i.e. axis)
        // between two polygons (i.e. separating them), they are not intersecting

        // Take all edges
        self.edges()
            .chain(other.edges())
            // If we can draw a perpendicular (i.e. normal) between them,
            // the polygons are separate
            .map(Vector::normal)
            // Make axis a unit vector to simplify the following math:
            // If the axis has a magnitude of 1, we don't need to divide
            // the scalar projection by it.
            .map(Vector::unit)
            .all(|axis| {
                // Take the bounds of the line that is created by projecting all
                // vertices onto the axis
                let (own_min, own_max) = self.scalar_project_onto_unit_vector(axis);
                let (other_min, other_max) = other.scalar_project_onto_unit_vector(axis);

                // If both bounds are outside the other polygon's projection, we are
                // able to draw a separating axis between them
                own_min.max(other_min) <= own_max.min(other_max)
            })
    }
}

impl From<Aabb> for Polygon {
    fn from(aabb: Aabb) -> Self {
        Polygon {
            vertices: vec![
                Point {
                    x: aabb.upper_left.x,
                    y: aabb.upper_left.y,
                },
                Point {
                    x: aabb.upper_left.x,
                    y: aabb.lower_right.y,
                },
                Point {
                    x: aabb.lower_right.x,
                    y: aabb.upper_left.y,
                },
                Point {
                    x: aabb.lower_right.x,
                    y: aabb.lower_right.y,
                },
            ],
        }
    }
}

/// Calculate which on which side of a line from `a` to `b` a
/// given `point` is
fn calculate_facing_side(a: Vector, b: Vector, point: Vector) -> Side {
    let side_vector = b - a;
    let vector_from_a_to_point = point - a;
    let cross_product = side_vector.cross_product(vector_from_a_to_point);

    /// Minimal distance from `point` to the line to be considered
    /// exactly *on* the line
    const EPSILON: f64 = 0.000_001;
    if cross_product < -EPSILON {
        Side::Left
    } else if cross_product > EPSILON {
        Side::Right
    } else {
        Side::OnTheLine
    }
}

fn is_convex_polygon(vertices: &[Point]) -> bool {
    let convex_hull_vertice_count = ConvexHull::try_new(vertices).unwrap().count();
    convex_hull_vertice_count == vertices.len()
}

/// The side that a [`Point`] lies on, from the
/// point of view of a line
#[derive(Eq, PartialEq, Debug)]
enum Side {
    /// The point lies to the right of the line
    Right,
    /// The point lies to the left of the line
    Left,
    /// The point is exactly on the line
    OnTheLine,
}

#[cfg(test)]
mod tests {
    use self::builder::PolygonBuilder;
    use super::*;
    use std::f64::consts::PI;

    fn polygon() -> Polygon {
        PolygonBuilder::default()
            .vertex(-10.0, -10.0)
            .vertex(10.0, -10.0)
            .vertex(10.0, 10.0)
            .vertex(-10.0, 10.0)
            .build()
            .unwrap()
    }

    fn translation() -> Point {
        Point { x: 30.0, y: 40.0 }
    }

    #[test]
    fn translates() {
        let polygon = polygon();
        assert_eq!(
            Polygon {
                vertices: vec![
                    Point { x: 20.0, y: 30.0 },
                    Point { x: 40.0, y: 30.0 },
                    Point { x: 40.0, y: 50.0 },
                    Point { x: 20.0, y: 50.0 },
                ],
            },
            polygon.translate(translation())
        );
    }

    #[test]
    fn rotates_by_pi() {
        let polygon = polygon();

        const FLOATING_POINT_INACCURACY: f64 = 0.000_000_000_000_002;
        assert_eq!(
            Polygon {
                vertices: vec![
                    Point {
                        x: 10.0 - FLOATING_POINT_INACCURACY,
                        y: 10.0 + FLOATING_POINT_INACCURACY,
                    },
                    Point {
                        x: -10.0 - FLOATING_POINT_INACCURACY,
                        y: 10.0 - FLOATING_POINT_INACCURACY,
                    },
                    Point {
                        x: -10.0 + FLOATING_POINT_INACCURACY,
                        y: -10.0 - FLOATING_POINT_INACCURACY,
                    },
                    Point {
                        x: 10.0 + FLOATING_POINT_INACCURACY,
                        y: -10.0 + FLOATING_POINT_INACCURACY,
                    },
                ],
            },
            polygon.rotate_around_point(Radians::try_new(PI).unwrap(), Point::default())
        );
    }

    #[test]
    fn rotates_by_arbitrary_orientation() {
        let polygon = polygon();

        const ROTATION_A: f64 = 8.488_724_885_405_782;
        const ROTATION_B: f64 = 11.311_125_046_603_125;

        assert_eq!(
            Polygon {
                vertices: vec![
                    Point {
                        x: ROTATION_A,
                        y: ROTATION_B,
                    },
                    Point {
                        x: -ROTATION_B,
                        y: ROTATION_A,
                    },
                    Point {
                        x: -ROTATION_A,
                        y: -ROTATION_B,
                    },
                    Point {
                        x: ROTATION_B,
                        y: -ROTATION_A,
                    },
                ],
            },
            polygon.rotate_around_point(Radians::try_new(3.0).unwrap(), Point::default())
        );
    }

    #[test]
    fn translates_and_rotates() {
        let polygon = polygon();
        let translation = translation();
        let translated_polygon = polygon.translate(translation);

        assert_eq!(
            Polygon {
                vertices: vec![
                    Point {
                        x: 38.488_724_885_405_78,
                        y: 51.311_125_046_603_124,
                    },
                    Point {
                        x: 18.688_874_953_396_876,
                        y: 48.488_724_885_405_78,
                    },
                    Point {
                        x: 21.511_275_114_594_22,
                        y: 28.688_874_953_396_876,
                    },
                    Point {
                        x: 41.311_125_046_603_124,
                        y: 31.511_275_114_594_22,
                    },
                ],
            },
            translated_polygon.rotate_around_point(Radians::try_new(3.0).unwrap(), translation)
        );
    }

    #[test]
    fn contains_point_when_point_is_positive() {
        let translation = Point { x: 10.43, y: 20.1 };
        let polygon = polygon().translate(translation);
        let point = Point { x: 12.0, y: 18.0 };
        assert!(polygon.contains_point(point));
    }

    #[test]
    fn contains_point_when_point_is_negative() {
        let translation = Point { x: -20.0, y: -5.0 };
        let polygon = polygon().translate(translation);
        let point = Point { x: -21.70, y: -2.3 };
        assert!(polygon.contains_point(point));
    }

    #[test]
    fn contains_point_when_point_is_at_zero() {
        let polygon = polygon();
        let point = Point::default();
        assert!(polygon.contains_point(point));
    }

    #[test]
    fn contains_point_at_border() {
        let polygon = polygon();
        let point = Point { x: 10.0, y: -10.0 };
        assert!(polygon.contains_point(point));
    }

    #[test]
    fn does_not_contain_point_barely_outside_polygon() {
        let polygon = polygon();
        let point = Point { x: 10.1, y: -10.1 };
        assert!(!polygon.contains_point(point));
    }

    #[test]
    fn does_not_contain_point_way_outside_polygon() {
        let polygon = polygon();
        let point = Point {
            x: -9000.0,
            y: -9000.0,
        };
        assert!(!polygon.contains_point(point));
    }

    #[test]
    fn does_not_contain_point_when_point_is_at_zero() {
        let translation = Point { x: 11.0, y: 11.0 };
        let polygon = polygon().translate(translation);
        let point = Point::default();
        assert!(!polygon.contains_point(point));
    }

    #[test]
    #[should_panic]
    fn aabb_panics_when_polygon_has_zero_vertices() {
        let polygon = Polygon::default();
        let _aabb = polygon.aabb();
    }

    #[test]
    fn aabb_returns_works_with_three_vertices() {
        let polygon = Polygon {
            vertices: vec![
                Point { x: -5.0, y: -5.0 },
                Point { x: 5.0, y: 0.0 },
                Point { x: 5.0, y: 5.0 },
            ],
        };
        let expected_aabb = Aabb::try_new((-5.0, -5.0), (5.0, 5.0)).unwrap();

        assert_eq!(expected_aabb, polygon.aabb());
    }

    #[test]
    fn aabb_returns_works_with_four_vertices() {
        let polygon = Polygon {
            vertices: vec![
                Point { x: 5.0, y: 0.0 },
                Point { x: 0.0, y: 5.0 },
                Point { x: -5.0, y: -5.0 },
                Point { x: 5.0, y: 5.0 },
            ],
        };
        let expected_aabb = Aabb::try_new((-5.0, -5.0), (5.0, 5.0)).unwrap();

        assert_eq!(expected_aabb, polygon.aabb());
    }

    #[test]
    fn try_new_errors_for_zero_vertices() {
        assert!(Polygon::try_new(Vec::new()).is_err());
    }

    #[test]
    fn try_new_errors_for_one_vertex() {
        assert!(Polygon::try_new(vec![Point { x: 0.0, y: 0.0 }]).is_err());
    }

    #[test]
    fn try_new_errors_for_two_vertices() {
        assert!(
            Polygon::try_new(vec![Point { x: 0.0, y: 0.0 }, Point { x: 1.0, y: 0.0 }]).is_err()
        );
    }

    #[test]
    fn try_new_works_for_three_vertices() {
        assert_eq!(
            Ok(Polygon {
                vertices: vec![
                    Point { x: 0.0, y: 0.0 },
                    Point { x: 1.0, y: 0.0 },
                    Point { x: 0.0, y: 1.0 },
                ]
            }),
            Polygon::try_new(vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 1.0, y: 0.0 },
                Point { x: 0.0, y: 1.0 },
            ])
        );
    }

    #[test]
    fn try_new_works_for_four_vertices() {
        assert_eq!(
            Ok(Polygon {
                vertices: vec![
                    Point { x: 0.0, y: 0.0 },
                    Point { x: 1.0, y: 0.0 },
                    Point { x: 0.0, y: 1.0 },
                    Point { x: 1.0, y: 1.0 },
                ]
            }),
            Polygon::try_new(vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 1.0, y: 0.0 },
                Point { x: 0.0, y: 1.0 },
                Point { x: 1.0, y: 1.0 },
            ])
        );
    }

    #[test]
    fn try_new_does_not_work_with_concave_polygon() {
        assert!(Polygon::try_new(vec![
            Point { x: 10.0, y: 10.0 },
            Point { x: 5.0, y: 5.0 },
            Point { x: 10.0, y: 5.0 },
            Point { x: 15.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ])
        .is_err());
    }

    #[test]
    fn try_new_works_with_convex_polygon() {
        let vertices = vec![
            Point { x: 10.0, y: 10.0 },
            Point { x: 5.0, y: 5.0 },
            Point { x: 20.0, y: 5.0 },
            Point { x: 15.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ];

        assert_eq!(
            Ok(Polygon {
                vertices: vertices.clone(),
            }),
            Polygon::try_new(vertices)
        );
    }

    #[test]
    fn can_be_created_from_aabb() {
        let aabb = Aabb::try_new(Point { x: 10.0, y: 15.0 }, Point { x: 20.0, y: 30.0 }).unwrap();
        let expected_polygon = Polygon::try_new(vec![
            Point { x: 10.0, y: 15.0 },
            Point { x: 10.0, y: 30.0 },
            Point { x: 20.0, y: 15.0 },
            Point { x: 20.0, y: 30.0 },
        ])
        .unwrap();

        assert_eq!(expected_polygon, Polygon::from(aabb));
    }

    #[test]
    fn edges_are_reported_correctly() {
        let polygon = Polygon::try_new(vec![
            Point { x: 10.0, y: 15.0 },
            Point { x: 10.0, y: 30.0 },
            Point { x: 20.0, y: 15.0 },
            Point { x: 20.0, y: 30.0 },
        ])
        .unwrap();

        let expected_edges = vec![
            Vector { x: 0.0, y: 15.0 },
            Vector { x: 10.0, y: -15.0 },
            Vector { x: 0.0, y: 15.0 },
            Vector { x: -10.0, y: -15.0 },
        ];

        let edges: Vec<_> = polygon.edges().collect();
        assert_eq!(expected_edges, edges);
    }

    #[test]
    fn intersects_self() {
        let polygon = Polygon::try_new(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: 10.0 },
        ])
        .unwrap();
        assert!(polygon.intersects(&polygon));
    }

    #[test]
    fn intersects_contained() {
        let bigger_polygon = Polygon::try_new(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: 10.0 },
        ])
        .unwrap();
        let smaller_polygon = Polygon::try_new(vec![
            Point { x: 2.0, y: 2.0 },
            Point { x: 8.0, y: 2.0 },
            Point { x: 8.0, y: 8.0 },
        ])
        .unwrap();
        assert!(bigger_polygon.intersects(&smaller_polygon));
        assert!(smaller_polygon.intersects(&bigger_polygon));
    }

    #[test]
    fn intersects_touching_line() {
        let left_polygon = Polygon::try_new(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: 10.0 },
        ])
        .unwrap();
        let right_polygon = Polygon::try_new(vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 5.0, y: 5.0 },
            Point { x: 20.0, y: 10.0 },
        ])
        .unwrap();
        assert!(left_polygon.intersects(&right_polygon));
        assert!(right_polygon.intersects(&left_polygon));
    }

    #[test]
    fn intersects_touching_point() {
        let left_polygon = Polygon::try_new(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: 10.0 },
        ])
        .unwrap();
        let right_polygon = Polygon::try_new(vec![
            Point { x: 10.0, y: 10.0 },
            Point { x: 20.0, y: 10.0 },
            Point { x: 20.0, y: 11.0 },
        ])
        .unwrap();
        assert!(left_polygon.intersects(&right_polygon));
        assert!(right_polygon.intersects(&left_polygon));
    }

    #[test]
    fn intersects_intersecting() {
        let first_polygon = Polygon::try_new(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: 10.0 },
        ])
        .unwrap();
        let second_polygon = Polygon::try_new(vec![
            Point { x: 8.0, y: 8.0 },
            Point { x: 20.0, y: 8.0 },
            Point { x: 20.0, y: 20.0 },
        ])
        .unwrap();
        assert!(first_polygon.intersects(&second_polygon));
        assert!(second_polygon.intersects(&first_polygon));
    }

    #[test]
    fn intersects_intersecting_when_negative() {
        let first_polygon = Polygon::try_new(vec![
            Point { x: -10.0, y: -10.0 },
            Point { x: -5.0, y: -10.0 },
            Point { x: -5.0, y: -5.0 },
        ])
        .unwrap();
        let second_polygon = Polygon::try_new(vec![
            Point { x: -6.0, y: -20.0 },
            Point { x: -3.0, y: -20.0 },
            Point { x: -3.0, y: -3.0 },
        ])
        .unwrap();
        assert!(first_polygon.intersects(&second_polygon));
        assert!(second_polygon.intersects(&first_polygon));
    }

    #[test]
    fn intersects_intersecting_when_negative_and_positive() {
        let first_polygon = Polygon::try_new(vec![
            Point { x: -5.0, y: -5.0 },
            Point { x: 5.0, y: -5.0 },
            Point { x: 5.0, y: 5.0 },
        ])
        .unwrap();
        let second_polygon = Polygon::try_new(vec![
            Point { x: -6.0, y: -20.0 },
            Point { x: 0.0, y: -20.0 },
            Point { x: 0.0, y: 2.0 },
        ])
        .unwrap();
        assert!(first_polygon.intersects(&second_polygon));
        assert!(second_polygon.intersects(&first_polygon));
    }

    #[test]
    fn does_not_intersect_when_apart() {
        let first_polygon = Polygon::try_new(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: 10.0 },
        ])
        .unwrap();
        let second_polygon = Polygon::try_new(vec![
            Point { x: 20.0, y: 0.0 },
            Point { x: 21.0, y: 0.0 },
            Point { x: 21.0, y: 20.0 },
        ])
        .unwrap();
        assert!(!first_polygon.intersects(&second_polygon));
        assert!(!second_polygon.intersects(&first_polygon));
    }
}
