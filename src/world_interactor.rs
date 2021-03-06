//! Trait and implementation for [`WorldInteractor`]

use crate::prelude::*;
use crate::private::Sealed;
#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;
use std::fmt::Debug;
use std::time::Duration;

mod world_interactor_impl;
pub use self::world_interactor_impl::*;

mod interactable;
pub use self::interactable::*;

/// Provides information to an [`ObjectBehavior`] about
/// the world it is placed in.
///
/// This trait is sealed and cannot be implemented by downstream crates.
///
/// [`ObjectBehavior`]: ./trait.ObjectBehavior.html
#[cfg_attr(any(test, feature = "use-mocks"), mockable)]
pub trait WorldInteractor<T>: Debug + Sealed {
    /// Scans for objects in the area defined by an `Aabb`.
    ///
    /// Returns all objects either completely contained or intersecting
    /// within the area.
    fn find_objects_in_area(&self, area: Aabb) -> Snapshot<'_, T>;

    /// Scans for objects in the area defined by a `Polygon`.
    ///
    /// Returns all objects either completely contained or intersecting
    /// within the area.
    fn find_objects_in_polygon(&self, area: &Polygon) -> Snapshot<'_, T>;

    /// Returns read-only descriptions for all objects
    /// intersecting with the given vector.
    fn find_objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_, T>;

    /// Returns the amount of time that passed since the last call
    /// to the `step` function of [`Simulation`]
    fn elapsed_time_in_update(&self) -> Duration;

    /// Returns the complete calling object's description
    fn own_object(&self) -> Object<'_, T>;
}

#[cfg(any(test, feature = "use-mocks"))]
impl<T> Sealed for WorldInteractorMock<'_, T> {}
