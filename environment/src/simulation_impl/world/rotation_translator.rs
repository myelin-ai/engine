//! Implementation for `NphysicsRotationTranslator`

use super::NphysicsRotationTranslator;
use crate::object::Radians;
use std::f64::consts::PI;

/// Translates the rotation from Radians to the range (-π; π] defined by nphysics
#[derive(Default, Debug)]
pub struct NphysicsRotationTranslatorImpl {}

impl NphysicsRotationTranslator for NphysicsRotationTranslatorImpl {
    fn to_nphysics_rotation(&self, orientation: Radians) -> f64 {
        if orientation.value() <= PI {
            orientation.value()
        } else {
            orientation.value() - (2.0 * PI)
        }
    }

    fn to_radians(&self, nphysics_rotation: f64) -> Option<Radians> {
        if nphysics_rotation >= 0.0 {
            Radians::new(nphysics_rotation)
        } else {
            Radians::new((2.0 * PI) + nphysics_rotation)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn to_nphysics_rotation_returns_0_when_passed_0() {
        verify_to_nphysics_rotation_returns_exoected_result(Radians::new(0.0).unwrap(), 0.0)
    }

    #[test]
    fn to_nphysics_rotation_returns_half_pi_when_passed_half_pi() {
        verify_to_nphysics_rotation_returns_exoected_result(
            Radians::new(FRAC_PI_2).unwrap(),
            FRAC_PI_2,
        )
    }

    #[test]
    fn to_nphysics_rotation_returns_pi_when_passed_pi() {
        verify_to_nphysics_rotation_returns_exoected_result(Radians::new(PI).unwrap(), PI)
    }

    #[test]
    fn to_nphysics_rotation_returns_negative_half_pi_when_passed_one_and_a_half_pi() {
        verify_to_nphysics_rotation_returns_exoected_result(
            Radians::new(3.0 * FRAC_PI_2).unwrap(),
            -FRAC_PI_2,
        )
    }

    #[test]
    fn to_radians_returns_0_when_passed_0() {
        verify_to_radians_returns_expected_result(0.0, Radians::new(0.0))
    }

    #[test]
    fn to_radians_returns_half_pi_when_passed_half_pi() {
        verify_to_radians_returns_expected_result(FRAC_PI_2, Radians::new(FRAC_PI_2))
    }

    #[test]
    fn to_radians_returns_returns_pi_when_passed_pi() {
        verify_to_radians_returns_expected_result(PI, Radians::new(PI))
    }

    #[test]
    fn to_radians_returns_one_and_a_half_pi_when_passed_negative_half_pi() {
        verify_to_radians_returns_expected_result(-FRAC_PI_2, Radians::new(3.0 * FRAC_PI_2))
    }

    fn verify_to_nphysics_rotation_returns_exoected_result(input: Radians, expected: f64) {
        let translator = NphysicsRotationTranslatorImpl::default();
        assert_eq!(expected, translator.to_nphysics_rotation(input));
    }

    fn verify_to_radians_returns_expected_result(input: f64, expected: Option<Radians>) {
        let translator = NphysicsRotationTranslatorImpl::default();
        assert_eq!(expected, translator.to_radians(input));
    }

}
