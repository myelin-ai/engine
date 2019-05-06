//! Entrypoint for the Simulaton API handling the world itself

mod simulation_impl;

pub use self::simulation_impl::*;
use crate::prelude::*;
use crate::private::Sealed;
#[cfg(any(test, feature = "use-mocks"))]
use mockiato::mockable;
use std::fmt::Debug;

/// A Simulation that can be filled with [`Object`] on
/// which it will apply physical rules when calling [`step`].
/// This trait represents our API.
///
/// This trait is sealed and cannot be implemented by downstream crates.
///
/// [`Object`]: ./object/struct.Object.html
/// [`step`]: ./trait.Simulation.html#tymethod.step
#[cfg_attr(any(test, feature = "use-mocks"), mockable)]
pub trait Simulation<T>: Debug + Sealed {
    /// Advance the simulation by one tick. This will apply
    /// forces to the objects, handle collisions and allow them to
    /// take action.
    fn step(&mut self);

    /// Add a new object to the world.
    fn add_object(
        &mut self,
        object_description: ObjectDescription<T>,
        object_behavior: Box<dyn ObjectBehavior<T>>,
    ) -> Object<'_, T>;

    /// Returns a read-only description of all objects currently inhabiting the simulation.
    fn objects(&self) -> Snapshot<'_, T>;

    /// Returns a read-only description an object, if the provided ID is valid.
    fn object(&self, id: Id) -> Option<Object<'_, T>>;

    /// Sets how much time in seconds is simulated for each step.
    /// # Examples
    /// If you want to run a simulation with 60 steps per second, you
    /// can run `set_simulated_timestep(1.0/60.0)`. Note that this method
    /// does not block the thread if called faster than expected.
    fn set_simulated_timestep(&mut self, timestep: f64);

    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_, T>;

    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_polygon(&self, area: &Polygon) -> Snapshot<'_, T>;

    /// Returns read-only descriptions for all objects
    /// intersecting with the given vector.
    fn objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_, T>;
}

/// Unique identifier of an Object
pub type Id = usize;

/// A representation of the current state of the simulation
pub type Snapshot<'a, T> = Vec<Object<'a, T>>;

#[cfg(any(test, feature = "use-mocks"))]
impl<T> Sealed for SimulationMock<'_, T> {}
