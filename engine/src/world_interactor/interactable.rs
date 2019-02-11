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
    use std::cell::RefCell;
    use std::thread::panicking;

    /// Mock for [`Interactable`]
    ///
    /// [`Interactable`]: ../trait.Interactable.html
    #[derive(Debug, Default)]
    pub struct InteractableMock<'a> {
        expect_objects_in_area_and_return: Option<(Aabb, Snapshot<'a>)>,
        objects_in_area_was_called: RefCell<bool>,
    }

    impl<'a> InteractableMock<'a> {
        /// Construct a new `InteractableMock`
        pub fn new() -> Self {
            Default::default()
        }

        pub fn expect_objects_in_area_and_return(
            &mut self,
            area: Aabb,
            return_value: Snapshot<'a>,
        ) {
            self.expect_objects_in_area_and_return = Some((area, return_value))
        }
    }

    impl<'a> Interactable for InteractableMock<'a> {
        fn objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
            *self.objects_in_area_was_called.borrow_mut() = true;

            let (expected_area, return_value) = self
                .expect_objects_in_area_and_return
                .clone()
                .expect("objects_in_area() was called unexpectedly");

            assert_eq!(
                expected_area, area,
                "objects_in_area() was called with {:?}, expected {:?}",
                area, expected_area
            );

            return_value.clone()
        }

        fn elapsed_time_in_update(&self) -> Duration {
            unimplemented!()
        }
    }

    impl<'a> Drop for InteractableMock<'a> {
        fn drop(&mut self) {
            if panicking() {
                return;
            }

            assert!(
                *self.objects_in_area_was_called.borrow(),
                "objects_in_area() was not called, but expected"
            )
        }
    }
}
