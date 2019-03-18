use crate::prelude::*;
use crate::private::Sealed;
#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;
use std::fmt::Debug;
use std::time::Duration;

/// Trait used by [`WorldInteractor`].
/// Implementors of this trait provide the actual code used for the performed actions
///
/// This trait is sealed and cannot be implemented by downstream crates.
#[cfg_attr(any(test, feature = "use-mocks"), mockable)]
pub trait Interactable: Debug + Sealed {
    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_>;

    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_polygon(&self, area: &Polygon) -> Snapshot<'_>;

    /// Returns read-only descriptions for all objects
    /// intersecting with the given vector.
    fn objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_>;

    /// Returns the amount of time that passed since the last call
    /// to the `step` function of [`Simulation`]
    fn elapsed_time_in_update(&self) -> Duration;

    /// Returns an object, if the specified ID is valid
    fn object(&self, id: Id) -> Option<Object<'_>>;
}

#[cfg(any(test, feature = "use-mocks"))]
impl Sealed for InteractableMock<'_> {}
