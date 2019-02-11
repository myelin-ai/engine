use crate::prelude::*;
use std::fmt::Debug;
use std::time::Duration;

#[cfg(any(test, feature = "use-mocks"))]
pub use self::mocks::*;

/// Trait used by [`WorldInteractor`].
/// Implementors of this trait provide the actual code used for the performed actions
pub trait Interactable: Debug {
    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_>;

    /// Returns the amount of time that passed since the last call
    /// to the `step` function of [`Simulation`]
    fn elapsed_time_in_update(&self) -> Duration;
}

#[cfg(any(test, feature = "use-mocks"))]
mod mocks {
    use super::*;

    /// Mock for [`Interactable`]
    ///
    /// [`Interactable`]: ../trait.Interactable.html
    #[derive(Debug, Default)]
    pub struct InteractableMock {}

    impl Interactable for InteractableMock {
        fn objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
            unimplemented!()
        }

        fn elapsed_time_in_update(&self) -> Duration {
            unimplemented!()
        }
    }
}
