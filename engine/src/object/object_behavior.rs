use std::any::Any;
use std::fmt::Debug;

use crate::prelude::*;

#[cfg(any(test, feature = "use-mocks"))]
pub use self::mocks::*;

/// Behavior of an object
pub trait ObjectBehavior: Debug + ObjectBehaviorClone {
    /// Returns all actions performed by the object
    /// in the current simulation tick
    fn step(
        &mut self,
        own_description: &ObjectDescription,
        world_interactor: &dyn WorldInteractor,
    ) -> Option<Action>;

    /// Cast implementation to `Any`.
    /// This is needed in order to downcast trait objects of type `&dyn ObjectBehavior` to
    /// concrete types.
    ///
    /// Implement this as follows.
    /// ```rust,ignore
    /// fn as_any(&self) -> &Any {
    ///    self
    /// }
    /// ```
    /// [Additional information](https://stackoverflow.com/a/47642317/5903309)
    fn as_any(&self) -> &'_ dyn Any;
}

/// Supertrait used to make sure that all implementors
/// of [`ObjectBehavior`] are [`Clone`]. You don't need
/// to care about this type.
///
/// [`ObjectBehavior`]: ./trait.ObjectBehavior.html
/// [`Clone`]: https://doc.rust-lang.org/nightly/std/clone/trait.Clone.html
#[doc(hidden)]
pub trait ObjectBehaviorClone {
    fn clone_box(&self) -> Box<dyn ObjectBehavior>;
}

impl<T> ObjectBehaviorClone for T
where
    T: ObjectBehavior + Clone + 'static,
{
    default fn clone_box(&self) -> Box<dyn ObjectBehavior> {
        box self.clone()
    }
}

#[cfg(any(test, feature = "use-mocks"))]
mod mocks {
    use super::*;
    use std::cell::RefCell;
    use std::thread::panicking;

    /// Mock for [`ObjectBehavior`]
    ///
    /// [`ObjectBehavior`]: ../trait.ObjectBehavior.html
    #[derive(Debug, Default, Clone)]
    pub struct ObjectBehaviorMock {
        expect_step_and_return: Option<(ObjectDescription, Option<Action>)>,

        step_was_called: RefCell<bool>,
    }

    impl ObjectBehaviorMock {
        /// Construt a new `ObjectBehaviorMock`
        pub fn new() -> Self {
            Default::default()
        }

        /// Expect a call to `step`
        pub fn expect_step_and_return(
            &mut self,
            own_description: &ObjectDescription,
            return_value: Option<Action>,
        ) {
            self.expect_step_and_return = Some((own_description.clone(), return_value));
        }
    }

    impl ObjectBehavior for ObjectBehaviorMock {
        fn step(
            &mut self,
            own_description: &ObjectDescription,
            _world_interactor: &dyn WorldInteractor,
        ) -> Option<Action> {
            *self.step_was_called.borrow_mut() = true;

            let (expected_own_description, return_value) = self
                .expect_step_and_return
                .clone()
                .expect("step() was called unexpectedly");

            assert_eq!(
                expected_own_description, *own_description,
                "step() was called with {:?}, expected {:?}",
                own_description, expected_own_description
            );

            return_value.clone()
        }

        fn as_any(&self) -> &'_ dyn Any {
            self
        }
    }

    impl Drop for ObjectBehaviorMock {
        fn drop(&mut self) {
            if panicking() {
                return;
            }

            assert!(
                self.expect_step_and_return.is_some() == *self.step_was_called.borrow(),
                "step() was not called, but expected"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn object_behavior_can_be_downcast() {
        let object_behavior: Box<dyn ObjectBehavior> = box ObjectBehaviorMock::new();
        let object_behavior_as_any = object_behavior.as_any();
        let downcast_behavior = object_behavior_as_any.downcast_ref::<ObjectBehaviorMock>();
        assert!(downcast_behavior.is_some())
    }
}
