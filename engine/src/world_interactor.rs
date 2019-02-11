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
    use std::cell::RefCell;
    use std::thread::panicking;

    /// Mock for [`WorldInteractor`]
    ///
    /// [`WorldInteractor`]: ../trait.WorldInteractor.html
    #[derive(Debug, Default)]
    pub struct WorldInteractorMock<'a> {
        expect_find_objects_in_area_and_return: Option<(Aabb, Snapshot<'a>)>,
        expect_elapsed_time_in_update_and_return: Option<(Duration,)>,

        find_objects_in_area_was_called: RefCell<bool>,
        elapsed_time_in_update_was_called: RefCell<bool>,
    }

    impl<'a> WorldInteractorMock<'a> {
        /// Constructs a new `WorldInteractorMock`
        pub fn new() -> Self {
            Default::default()
        }

        /// Expect a call to `find_objects_in_area`
        pub fn expect_find_objects_in_area_and_return(
            &mut self,
            area: Aabb,
            return_value: Snapshot<'a>,
        ) {
            self.expect_find_objects_in_area_and_return = Some((area, return_value));
        }

        /// Expect a call to `elapsed_time_in_update`
        pub fn expect_elapsed_time_in_update_and_return(&mut self, return_value: Duration) {
            self.expect_elapsed_time_in_update_and_return = Some((return_value,));
        }
    }

    impl<'a> WorldInteractor for WorldInteractorMock<'a> {
        fn find_objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
            *self.find_objects_in_area_was_called.borrow_mut() = true;

            let (expected_area, return_value) = self
                .expect_find_objects_in_area_and_return
                .clone()
                .expect("find_objects_in_area() was called unexpectedly");

            assert_eq!(
                expected_area, area,
                "find_objects_in_area() was called with {:?}, expected {:?}",
                area, expected_area
            );

            return_value.clone()
        }

        fn elapsed_time_in_update(&self) -> Duration {
            *self.elapsed_time_in_update_was_called.borrow_mut() = true;

            let (return_value,) = self
                .expect_elapsed_time_in_update_and_return
                .clone()
                .expect("elapsed_time_in_update() was called unexpectedly");

            return_value.clone()
        }
    }

    impl<'a> Drop for WorldInteractorMock<'a> {
        fn drop(&mut self) {
            if panicking() {
                return;
            }

            assert!(
                self.expect_find_objects_in_area_and_return.is_some()
                    == *self.find_objects_in_area_was_called.borrow(),
                "find_objects_in_area() was not called, but expected"
            );

            assert!(
                self.expect_elapsed_time_in_update_and_return.is_some()
                    == *self.elapsed_time_in_update_was_called.borrow(),
                "elapsed_time_in_update() was not called, but expected"
            );
        }
    }
}
