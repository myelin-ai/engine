use crate::Point;
use serde_derive::{Deserialize, Serialize};
use std::ops::{Add, Div, Mul, Sub};

/// A vector
#[derive(Debug, PartialEq, Copy, Clone, Default, Serialize, Deserialize)]
pub struct Vector {
    /// The x component of the Vector
    pub x: f64,
    /// The y component of the Vector
    pub y: f64,
}

impl Add for Vector {
    type Output = Vector;

    fn add(self, other: Self::Output) -> Self::Output {
        Vector {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vector {
    type Output = Vector;

    fn sub(self, other: Self::Output) -> Self::Output {
        Vector {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<f64> for Vector {
    type Output = Vector;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Div<f64> for Vector {
    type Output = Vector;

    fn div(self, rhs: f64) -> Self::Output {
        Vector {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl From<Point> for Vector {
    fn from(point: Point) -> Self {
        Self {
            x: point.x,
            y: point.y,
        }
    }
}

impl Vector {
    /// Calculates the dot product of itself and another vector
    /// # Examples
    /// ```
    /// use myelin_geometry::Vector;
    /// // a · b = c
    /// let a = Vector { x: 2.0, y: 3.0 };
    /// let b = Vector { x: -4.0, y: 10.0 };
    /// let c = a.dot_product(b);
    /// assert_eq!(22.0, c);
    /// ```
    pub fn dot_product(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Calculates the cross product of itself and another vector
    /// # Examples
    /// ```
    /// use myelin_geometry::Vector;
    /// // a × b = c
    /// let a = Vector { x: 2.0, y: 3.0 };
    /// let b = Vector { x: -4.0, y: 10.0 };
    /// let c = a.cross_product(b);
    /// assert_eq!(32.0, c);
    /// ```
    pub fn cross_product(self, other: Self) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Retuns the vector's normal vector, i.e. a vector that is perpendicular to this vector
    pub fn normal(self) -> Vector {
        Vector {
            x: -self.y,
            y: self.x,
        }
    }

    /// Returns the magnitude of the vector, i.e. its length if viewed as a line
    pub fn magnitude(self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    /// Returns unit vector of this vector, i.e. a vector with the same direction and a magnitude of 1
    pub fn unit(self) -> Vector {
        let magnitude = self.magnitude();
        assert!(
            magnitude != 0.0,
            "Attempted to take the unit vector of a zero vector (0, 0), which is undefined"
        );

        self / magnitude
    }

    /// Returns the projection of this vector onto another vector
    pub fn project_onto(self, other: Vector) -> Vector {
        other.unit() * self.dot_product(other) / other.magnitude()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nearly_eq::assert_nearly_eq;

    #[test]
    fn is_equal_to_itself() {
        let vector = Vector { x: -12.9, y: 45.1 };
        assert_eq!(vector, vector);
    }

    #[test]
    fn is_equal_to_itself_when_zero() {
        let vector = Vector { x: 0.0, y: 0.0 };
        assert_eq!(vector, vector);
    }

    #[test]
    fn is_no_equal_to_other_vector() {
        let vector = Vector { x: 12.3, y: 89.0 };
        let different_vector = Vector { x: 12.4, y: 89.0 };
        assert!(vector != different_vector);
    }

    #[test]
    fn adds_zero_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_add = Vector { x: 0.0, y: 0.0 };
        let expected_vector = original_vector;
        let added_vector = original_vector + vector_to_add;
        assert_eq!(expected_vector, added_vector);
    }

    #[test]
    fn adds_other_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_add = Vector { x: 3.0, y: 1.0 };
        let expected_vector = Vector { x: 15.0, y: 44.0 };
        let added_vector = original_vector + vector_to_add;
        assert_eq!(expected_vector, added_vector);
    }

    #[test]
    fn adds_negative_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_add = Vector { x: -10.0, y: -20.0 };
        let expected_vector = Vector { x: 2.0, y: 23.0 };
        let added_vector = original_vector + vector_to_add;
        assert_eq!(expected_vector, added_vector);
    }

    #[test]
    fn adds_to_zero_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_add = Vector { x: -12.0, y: -43.0 };
        let expected_vector = Vector { x: 0.0, y: 0.0 };
        let added_vector = original_vector + vector_to_add;
        assert_eq!(expected_vector, added_vector);
    }

    #[test]
    fn adds_when_negative() {
        let original_vector = Vector { x: -12.0, y: -43.0 };
        let vector_to_add = Vector { x: -4.0, y: -2.0 };
        let expected_vector = Vector { x: -16.0, y: -45.0 };
        let added_vector = original_vector + vector_to_add;
        assert_eq!(expected_vector, added_vector);
    }

    #[test]
    fn subtracts_zero_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_subtract = Vector { x: 0.0, y: 0.0 };
        let expected_vector = original_vector;
        let substracted_vector = original_vector - vector_to_subtract;
        assert_eq!(expected_vector, substracted_vector);
    }

    #[test]
    fn subtracts_other_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_subtract = Vector { x: 3.0, y: 1.0 };
        let expected_vector = Vector { x: 9.0, y: 42.0 };
        let substracted_vector = original_vector - vector_to_subtract;
        assert_eq!(expected_vector, substracted_vector);
    }

    #[test]
    fn subtracts_negative_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_subtract = Vector { x: -10.0, y: -20.0 };
        let expected_vector = Vector { x: 22.0, y: 63.0 };
        let substracted_vector = original_vector - vector_to_subtract;
        assert_eq!(expected_vector, substracted_vector);
    }

    #[test]
    fn subtracts_to_zero_vector() {
        let original_vector = Vector { x: 12.0, y: 43.0 };
        let vector_to_subtract = original_vector;
        let expected_vector = Vector { x: 0.0, y: 0.0 };
        let vector_to_subtract = original_vector - vector_to_subtract;
        assert_eq!(expected_vector, vector_to_subtract);
    }

    #[test]
    fn subtracts_when_negative() {
        let original_vector = Vector { x: -12.0, y: -43.0 };
        let vector_to_subtract = Vector { x: -4.0, y: -2.0 };
        let expected_vector = Vector { x: -8.0, y: -41.0 };
        let substracted_vector = original_vector - vector_to_subtract;
        assert_eq!(expected_vector, substracted_vector);
    }

    #[test]
    fn scales_positive_vector() {
        let original_vector = Vector { x: 1.0, y: 2.0 };
        let expected_vector = Vector { x: 2.0, y: 4.0 };

        let scaled_vector = original_vector * 2.0;

        assert_eq!(expected_vector, scaled_vector);
    }

    #[test]
    fn scales_vector_with_negative_component() {
        let original_vector = Vector { x: -4.0, y: 2.0 };
        let expected_vector = Vector { x: -8.0, y: 4.0 };

        let scaled_vector = original_vector * 2.0;

        assert_eq!(expected_vector, scaled_vector);
    }

    #[test]
    fn shrinks_positive_vector() {
        let original_vector = Vector { x: 1.0, y: 2.0 };
        let expected_vector = Vector { x: 0.5, y: 1.0 };

        let scaled_vector = original_vector / 2.0;

        assert_eq!(expected_vector, scaled_vector);
    }

    #[test]
    fn shrinks_vector_with_negative_component() {
        let original_vector = Vector { x: -4.0, y: 2.0 };
        let expected_vector = Vector { x: -2.0, y: 1.0 };

        let scaled_vector = original_vector / 2.0;

        assert_eq!(expected_vector, scaled_vector);
    }

    #[test]
    fn calculates_dot_product() {
        let a = Vector { x: 2.0, y: 3.0 };
        let b = Vector { x: -4.0, y: 10.0 };
        let expected_dot_product = 22.0;
        let dot_product = a.dot_product(b);
        assert_nearly_eq!(expected_dot_product, dot_product);
    }

    #[test]
    fn calculates_negative_dot_product() {
        let a = Vector { x: 2.0, y: 3.0 };
        let b = Vector { x: -40.0, y: 10.0 };
        let expected_dot_product = -50.0;
        let dot_product = a.dot_product(b);
        assert_nearly_eq!(expected_dot_product, dot_product);
    }

    #[test]
    fn dot_product_is_zero_when_one_side_is_zero() {
        let a = Vector { x: 2.0, y: 3.0 };
        let b = Vector { x: 0.0, y: 0.0 };
        let expected_dot_product = 0.0;
        let dot_product = a.dot_product(b);
        assert_nearly_eq!(expected_dot_product, dot_product);
    }

    #[test]
    fn dot_product_is_zero_when_both_sides_are_zero() {
        let a = Vector { x: 0.0, y: 0.0 };
        let b = Vector { x: 0.0, y: 0.0 };
        let expected_dot_product = 0.0;
        let dot_product = a.dot_product(b);
        assert_nearly_eq!(expected_dot_product, dot_product);
    }

    #[test]
    fn calculates_cross_product() {
        let a = Vector { x: 2.0, y: 3.0 };
        let b = Vector { x: -4.0, y: 10.0 };
        let expected_cross_product = 32.0;
        let cross_product = a.cross_product(b);
        assert_nearly_eq!(expected_cross_product, cross_product);
    }

    #[test]
    fn calculates_negative_cross_product() {
        let a = Vector { x: 2.0, y: 3.0 };
        let b = Vector { x: 40.0, y: 10.0 };
        let expected_cross_product = -100.0;
        let cross_product = a.cross_product(b);
        assert_nearly_eq!(expected_cross_product, cross_product);
    }

    #[test]
    fn cross_product_is_zero_when_one_side_is_zero() {
        let a = Vector { x: 2.0, y: 3.0 };
        let b = Vector { x: 0.0, y: 0.0 };
        let expected_cross_product = 0.0;
        let cross_product = a.cross_product(b);
        assert_nearly_eq!(expected_cross_product, cross_product);
    }

    #[test]
    fn cross_product_is_zero_when_both_sides_are_zero() {
        let a = Vector { x: 0.0, y: 0.0 };
        let b = Vector { x: 0.0, y: 0.0 };
        let expected_cross_product = 0.0;
        let cross_product = a.cross_product(b);
        assert_nearly_eq!(expected_cross_product, cross_product);
    }

    #[test]
    fn cross_product_of_self_is_zero() {
        let vector = Vector { x: 40.0, y: 10.0 };
        let expected_cross_product = 0.0;
        let cross_product = vector.cross_product(vector);
        assert_nearly_eq!(expected_cross_product, cross_product);
    }

    #[test]
    fn returns_correct_normal() {
        let vector = Vector { x: 10.0, y: 3.0 };
        let expected_normal = Vector { x: -3.0, y: 10.0 };
        let normal = vector.normal();

        assert_eq!(expected_normal, normal);
    }

    #[test]
    fn dot_product_of_normal_is_zero() {
        let vector = Vector { x: 10.0, y: 3.0 };
        let normal = vector.normal();
        let expected_dot_product = 0.0;
        let dot_product = vector.dot_product(normal);

        assert_nearly_eq!(expected_dot_product, dot_product);
    }

    #[test]
    fn magnitude_of_zero_vector_is_zero() {
        let vector = Vector::default();
        let expected_magnitude = 0.0;
        let magnitude = vector.magnitude();

        assert_nearly_eq!(expected_magnitude, magnitude);
    }

    #[test]
    fn magnitude_of_horizontal_vector_is_correct() {
        let vector = Vector { x: 5.0, y: 0.0 };
        let expected_magnitude = 5.0;
        let magnitude = vector.magnitude();

        assert_nearly_eq!(expected_magnitude, magnitude);
    }

    #[test]
    fn magnitude_of_rotated_vector_is_correct() {
        let vector = Vector { x: 9.0, y: 3.0 };
        let expected_magnitude = 9.486_832_980_505_138;
        let magnitude = vector.magnitude();

        assert_nearly_eq!(expected_magnitude, magnitude);
    }

    #[test]
    fn magnitude_of_negative_vector_is_correct() {
        let vector = Vector { x: -5.0, y: -2.0 };
        let expected_magnitude = 5.385_164_807_134_504;
        let magnitude = vector.magnitude();

        assert_nearly_eq!(expected_magnitude, magnitude);
    }

    #[test]
    fn unit_vector_is_correct_for_positive_numbers() {
        let vector = Vector { x: 4.0, y: 2.0 };
        let expected_unit_vector = Vector {
            x: 2.0 / 5.0f64.sqrt(),
            y: 1.0 / 5.0f64.sqrt(),
        };
        let unit_vector = vector.unit();

        assert_eq!(expected_unit_vector, unit_vector);
    }

    #[test]
    fn unit_vector_is_correct_for_negative_numbers() {
        let vector = Vector { x: -10.0, y: -6.0 };
        let expected_unit_vector = Vector {
            x: -5.0 / 34f64.sqrt(),
            y: -3.0 / 34f64.sqrt(),
        };
        let unit_vector = vector.unit();

        assert_eq!(expected_unit_vector, unit_vector);
    }

    #[test]
    fn unit_vector_is_stretched_when_original_magnitude_is_smaller_than_one() {
        let vector = Vector { x: 0.2, y: 0.5 };
        let expected_unit_vector = Vector {
            x: 0.371_390_676_354_103_67,
            y: 0.928_476_690_885_259_2,
        };
        let unit_vector = vector.unit();

        assert_eq!(expected_unit_vector, unit_vector);
    }

    #[test]
    fn magnitude_of_unit_vector_is_one() {
        let vector = Vector {
            x: 1_000.0,
            y: -2_000.0,
        };
        let expected_magnitude = 1.0;
        let magnitude = vector.unit().magnitude();

        assert_nearly_eq!(expected_magnitude, magnitude);
    }

    #[test]
    #[should_panic]
    fn unit_vector_of_zero_vector_is_undefined() {
        let zero_vector = Vector::default();
        let _unit_vector = zero_vector.unit();
    }

    #[test]
    fn projection_onto_zero_vector_is_zero_vector() {
        let vector = Vector {
            x: 1_000.0,
            y: -2_000.0,
        };
        let zero_vector = Vector::default();
        let expected_projection = zero_vector;
        let projection = vector.project_onto(zero_vector);

        assert_eq!(expected_projection, projection);
    }

    #[test]
    fn projected_zero_vector_is_zero_vector() {
        let vector = Vector {
            x: 1_000.0,
            y: -2_000.0,
        };
        let zero_vector = Vector::default();
        let expected_projection = zero_vector;
        let projection = zero_vector.project_onto(vector);

        assert_eq!(expected_projection, projection);
    }

    #[test]
    fn projection_of_self_is_self() {
        let vector = Vector { x: 5.0, y: -2.0 };
        let expected_projection = vector;
        let projection = vector.project_onto(vector);

        assert_eq!(expected_projection, projection);
    }

    #[test]
    fn projection_is_correct_for_positive_numbers() {
        let projected_vector = Vector { x: 5.0, y: 2.0 };
        let other_vector = Vector { x: 10.0, y: -7.0 };

        let expected_projection = Vector {
            x: 640.0 / 149.0,
            y: 448.0 / 149.0,
        };
        let projection = projected_vector.project_onto(other_vector);

        assert_eq!(expected_projection, projection);
    }

    #[test]
    fn projection_is_correct_for_negative_numbers() {
        let projected_vector = Vector { x: -8.0, y: -1.0 };
        let other_vector = Vector { x: -2.0, y: -4.0 };

        let expected_projection = Vector { x: -2.0, y: -4.0 };
        let projection = projected_vector.project_onto(other_vector);

        assert_eq!(expected_projection, projection);
    }
}
