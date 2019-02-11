use std::any::Any;
use std::fmt::Debug;

use crate::prelude::*;

#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;

/// Behavior of an object
#[cfg_attr(any(test, feature = "use-mocks"), mockable)]
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
