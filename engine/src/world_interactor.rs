//! Trait and implementation for [`WorldInteractor`]

use crate::prelude::*;
use std::fmt::Debug;
use std::time::Duration;

mod world_interactor_impl;
pub use self::world_interactor_impl::*;

mod interactable;
pub use self::interactable::*;

#[cfg(any(test, feature = "use-mocks"))]
pub use self::mocks::*;

/// Provides information to an [`ObjectBehavior`] about
/// the world it is placed in.
///
/// [`ObjectBehavior`]: ./trait.ObjectBehavior.html
pub trait WorldInteractor: Debug {
    /// Scans for objects in the area defined by an [`Aabb`].
    ///
    /// Returns all objects either completely contained or intersecting
    /// with the area.
    ///
    /// [`Aabb`]: ./struct.Aabb.html
    fn find_objects_in_area(&self, area: Aabb) -> Snapshot<'_>;

    /// Returns the amount of time that passed since the last call
    /// to the `step` function of [`Simulation`]
    fn elapsed_time_in_update(&self) -> Duration;
}

#[cfg(any(test, feature = "use-mocks"))]
mod mocks {
    use super::*;

    /// Mock for [`WorldInteractor`]
    ///
    /// [`WorldInteractor`]: ../trait.WorldInteractor.html
    #[derive(Debug, Default)]
    pub struct WorldInteractorMock {}

    impl WorldInteractorMock {
        /// Constructs a new `WorldInteractorMock`
        pub fn new() -> Self {
            Default::default()
        }
    }

    impl WorldInteractor for WorldInteractorMock {
        fn find_objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
            unimplemented!()
        }

        fn elapsed_time_in_update(&self) -> Duration {
            unimplemented!()
        }
    }
}
