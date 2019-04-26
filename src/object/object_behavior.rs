use crate::prelude::*;
#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;
use std::any::Any;
use std::fmt::Debug;

/// Behavior of an object
#[cfg_attr(any(test, feature = "use-mocks"), mockable(static_references))]
pub trait ObjectBehavior: Debug + ObjectBehaviorClone {
    /// Returns all actions performed by the object
    /// in the current simulation tick
    fn step(&mut self, world_interactor: &dyn WorldInteractor) -> Option<Action>;

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

impl Clone for Box<dyn ObjectBehavior> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Debug)]
    struct ObjectBehaviorStub;

    impl ObjectBehavior for ObjectBehaviorStub {
        fn step(&mut self, _: &dyn WorldInteractor) -> Option<Action> {
            None
        }

        fn as_any(&self) -> &'_ dyn Any {
            self
        }
    }

    #[test]
    fn object_behavior_can_be_downcast() {
        let object_behavior: Box<dyn ObjectBehavior> = box ObjectBehaviorStub;

        let object_behavior_as_any = object_behavior.as_any();
        let downcast_behavior = object_behavior_as_any.downcast_ref::<ObjectBehaviorStub>();

        let _unwrapped_downcast_behavior: &ObjectBehaviorStub = downcast_behavior.unwrap();
    }
}
