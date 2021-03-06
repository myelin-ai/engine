use crate::prelude::*;
#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;
use std::any::Any;
use std::fmt::Debug;

/// Behavior of an object
#[cfg_attr(any(test, feature = "use-mocks"), mockable(static_references))]
pub trait ObjectBehavior<T>: Debug + ObjectBehaviorClone<T> + ObjectBehaviorAsAny<T>
where
    T: AssociatedObjectData,
{
    /// Returns all actions performed by the object
    /// in the current simulation tick
    fn step(&mut self, world_interactor: Box<dyn WorldInteractor<T> + '_>) -> Option<Action<T>>;
}

/// Cast implementation to [`Any`] for [`ObjectBehavior`].
pub trait ObjectBehaviorAsAny<T> {
    /// Cast implementation to [`Any`].
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

impl<Behaviour, AssociatedData> ObjectBehaviorAsAny<AssociatedData> for Behaviour
where
    Behaviour: ObjectBehavior<AssociatedData> + 'static,
    AssociatedData: AssociatedObjectData,
{
    default fn as_any(&self) -> &'_ dyn Any {
        self
    }
}

/// Supertrait used to make sure that all implementors
/// of [`ObjectBehavior`] are [`Clone`]. You don't need
/// to care about this type.
///
/// [`ObjectBehavior`]: ./trait.ObjectBehavior.html
/// [`Clone`]: https://doc.rust-lang.org/nightly/std/clone/trait.Clone.html
#[doc(hidden)]
pub trait ObjectBehaviorClone<T> {
    fn clone_box(&self) -> Box<dyn ObjectBehavior<T>>;
}

impl<'de, Behaviour, AssociatedData> ObjectBehaviorClone<AssociatedData> for Behaviour
where
    Behaviour: ObjectBehavior<AssociatedData> + Clone + 'static,
    AssociatedData: AssociatedObjectData,
{
    default fn clone_box(&self) -> Box<dyn ObjectBehavior<AssociatedData>> {
        box self.clone()
    }
}

impl<T> Clone for Box<dyn ObjectBehavior<T>> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn object_behavior_can_be_downcast() {
        let object_behavior: Box<dyn ObjectBehavior<()>> = box ObjectBehaviorMock::new();

        let object_behavior_as_any = object_behavior.as_any();
        let downcast_behavior = object_behavior_as_any.downcast_ref();

        let _unwrapped_downcast_behavior: &ObjectBehaviorMock<'_, ()> = downcast_behavior.unwrap();
    }
}
