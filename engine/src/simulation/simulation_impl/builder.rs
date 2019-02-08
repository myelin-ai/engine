use super::time::InstantWrapperImpl;
use super::{InstantWrapperFactoryFn, World, WorldInteractorFactoryFn};
use crate::simulation::simulation_impl::world::builder::NphysicsWorldBuilder;
use crate::simulation::simulation_impl::SimulationImpl;
use crate::simulation::Simulation;
use crate::world_interactor::WorldInteractorImpl;
use std::time::Instant;

#[derive(Default)]
pub struct SimulationBuilder {
    world: Option<Box<dyn World>>,
    world_interactor_factory_fn: Option<Box<WorldInteractorFactoryFn>>,
    instant_wrapper_factory_fn: Option<Box<InstantWrapperFactoryFn>>,
}

impl SimulationBuilder {
    pub fn new() -> SimulationBuilder {
        SimulationBuilder::default()
    }

    pub fn world(&mut self, world: Box<dyn World>) {
        self.world = Some(world)
    }

    pub fn world_interactor_factory_fn(
        &mut self,
        world_interactor_factory_fn: Box<WorldInteractorFactoryFn>,
    ) {
        self.world_interactor_factory_fn = Some(world_interactor_factory_fn)
    }

    pub fn instant_wrapper_factory_fn(
        &mut self,
        instant_wrapper_factory_fn: Box<InstantWrapperFactoryFn>,
    ) {
        self.instant_wrapper_factory_fn = Some(instant_wrapper_factory_fn)
    }

    pub fn build(self) -> Box<dyn Simulation> {
        box SimulationImpl::new(
            self.world
                .unwrap_or(box NphysicsWorldBuilder::default().build()),
            self.world_interactor_factory_fn
                .unwrap_or(box |simulation| box WorldInteractorImpl::new(simulation)),
            self.instant_wrapper_factory_fn
                .unwrap_or(box || box InstantWrapperImpl::new(Instant::now())),
        )
    }
}
