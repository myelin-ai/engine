//! Builder for the [`Simulation`]
//!
//! [`Simulation`]: ./../trait.Simulation.html

use super::time::InstantWrapperImpl;
use super::{InstantWrapperFactoryFn, World, WorldInteractorFactoryFn};
use crate::object::AssociatedObjectData;
use crate::simulation::simulation_impl::world::builder::NphysicsWorldBuilder;
use crate::simulation::simulation_impl::SimulationImpl;
use crate::simulation::Simulation;
use crate::world_interactor::WorldInteractorImpl;
use nameof::name_of_type;
use std::fmt;
use std::fmt::Debug;
use std::time::Instant;

/// Builder for the [`Simulation`]. This is the composition root.
/// Only advanced users should derive from the defaults.
///
/// # Example
/// ```
/// use myelin_engine::simulation::SimulationBuilder;
///
/// let simulation = SimulationBuilder::<()>::new().build();
/// ```
///
/// [`Simulation`]: ./../trait.Simulation.html
pub struct SimulationBuilder<T>
where
    T: AssociatedObjectData,
{
    world: Option<Box<dyn World>>,
    world_interactor_factory_fn: Option<Box<WorldInteractorFactoryFn<T>>>,
    instant_wrapper_factory_fn: Option<Box<InstantWrapperFactoryFn>>,
}

impl<T> Debug for SimulationBuilder<T>
where
    T: AssociatedObjectData,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct(name_of_type!(SimulationBuilder<T>))
            .field("world", &self.world)
            .finish()
    }
}

impl<T> Default for SimulationBuilder<T>
where
    T: AssociatedObjectData,
{
    fn default() -> Self {
        Self {
            world: None,
            world_interactor_factory_fn: None,
            instant_wrapper_factory_fn: None,
        }
    }
}

impl<T> SimulationBuilder<T>
where
    T: AssociatedObjectData,
{
    /// Creates a new builder by calling SimulationBuilder::default()
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the [`World`] which the [`Simulation`] is going to use.
    ///
    /// By default this builder uses the [`NphysicsWorldBuilder`] with default settings.
    ///
    /// [`World`]: ../trait.World.html
    /// [`Simulation`]: ./../trait.Simulation.html
    /// [`NphysicsWorldBuilder`]: ../builder/struct.NphysicsWorldBuilder.html
    pub fn world(&mut self, world: Box<dyn World>) {
        self.world = Some(world)
    }

    /// Sets the [`WorldInteractorFactoryFn`].
    ///
    /// [`WorldInteractorFactoryFn`]: ../simulation_impl/trait.WorldInteractorFactoryFn.html
    pub fn world_interactor_factory_fn(
        &mut self,
        world_interactor_factory_fn: Box<WorldInteractorFactoryFn<T>>,
    ) {
        self.world_interactor_factory_fn = Some(world_interactor_factory_fn)
    }

    /// Sets the [`InstantWrapperFactoryFn`].
    ///
    /// [`InstantWrapperFactoryFn`]: ../simulation_impl/trait.InstantWrapperFactoryFn.html
    pub fn instant_wrapper_factory_fn(
        &mut self,
        instant_wrapper_factory_fn: Box<InstantWrapperFactoryFn>,
    ) {
        self.instant_wrapper_factory_fn = Some(instant_wrapper_factory_fn)
    }

    /// Builds the [`Simulation`].
    ///
    /// [`Simulation`]: ./../trait.Simulation.html
    pub fn build<'a>(self) -> Box<dyn Simulation<T> + 'a>
    where
        T: 'a,
    {
        box SimulationImpl::new(
            self.world
                .unwrap_or_else(|| box NphysicsWorldBuilder::default().build()),
            self.world_interactor_factory_fn.unwrap_or_else(|| {
                box |simulation, id| box WorldInteractorImpl::new(simulation, id)
            }),
            self.instant_wrapper_factory_fn
                .unwrap_or_else(|| box || box InstantWrapperImpl::new(Instant::now())),
        )
    }
}
