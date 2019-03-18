//! Trait and implementation of [`NphysicsRotationTranslator`]

use crate::prelude::*;
use crate::private::Sealed;
#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;
use std::error::Error;
use std::fmt::{self, Debug};

mod rotation_translator_impl;
pub use self::rotation_translator_impl::*;

/// This trait translates the rotation from [`Radians`] to the range (-π; π] defined by nphysics
///
/// This trait is sealed and cannot be implemented by downstream crates.
///
/// [`Radians`]: ../../object/struct.Radians.html
#[cfg_attr(any(test, feature = "use-mocks"), mockable)]
pub trait NphysicsRotationTranslator: Debug + Sealed {
    /// Converts an `orientation` into a representation that is suitable for nphysics
    fn to_nphysics_rotation(&self, orientation: Radians) -> f64;

    /// Converts a rotation that originates from nphysics into [`Radians`]
    ///
    /// [`Radians`]: ../../object/struct.Radians.html
    fn to_radians(
        &self,
        nphysics_rotation: f64,
    ) -> Result<Radians, NphysicsRotationTranslatorError>;
}

#[cfg(any(test, feature = "use-mocks"))]
impl Sealed for NphysicsRotationTranslatorMock<'_> {}

/// The reason why a rotation could not be translated
#[derive(Debug, Clone, PartialEq)]
pub enum NphysicsRotationTranslatorError {
    /// The given nphysics value was not in the range (-π; π]
    InvalidNphysicsValue(f64),
}

impl fmt::Display for NphysicsRotationTranslatorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Given nphysics value is not in the range (-pi; pi]")
    }
}

impl Error for NphysicsRotationTranslatorError {}
