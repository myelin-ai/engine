//! Builder for the [`Simulation`]
//!
//! [`Simulation`]: ./../trait.Simulation.html

use super::time::InstantWrapperImpl;
use super::{InstantWrapperFactoryFn, World, WorldInteractorFactoryFn};
use crate::simulation::simulation_impl::world::builder::NphysicsWorldBuilder;
use crate::simulation::simulation_impl::SimulationImpl;
use crate::simulation::Simulation;
use crate::world_interactor::WorldInteractorImpl;
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
/// let simulation = SimulationBuilder::new().build();
/// ```
///
/// [`Simulation`]: ./../trait.Simulation.html
#[derive(Default)]
pub struct SimulationBuilder {
    world: Option<Box<dyn World>>,
    world_interactor_factory_fn: Option<Box<WorldInteractorFactoryFn>>,
    instant_wrapper_factory_fn: Option<Box<InstantWrapperFactoryFn>>,
}

impl Debug for SimulationBuilder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct(name_of_type!(SimulationBuilder))
            .field("world", &self.world)
            .finish()
    }
}

impl SimulationBuilder {
    /// Creates a new builder by calling SimulationBuilder::default()
    pub fn new() -> SimulationBuilder {
        SimulationBuilder::default()
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
        world_interactor_factory_fn: Box<WorldInteractorFactoryFn>,
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
    pub fn build(self) -> Box<dyn Simulation> {
        box SimulationImpl::new(
            self.world
                .unwrap_or_else(|| box NphysicsWorldBuilder::default().build()),
            self.world_interactor_factory_fn
                .unwrap_or_else(|| box |simulation| box WorldInteractorImpl::new(simulation)),
            self.instant_wrapper_factory_fn
                .unwrap_or_else(|| box || box InstantWrapperImpl::new(Instant::now())),
        )
    }
}
