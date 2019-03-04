use myelin_engine::prelude::*;
use myelin_engine::simulation::SimulationBuilder;
use std::any::Any;

#[derive(Debug, Clone, Default)]
struct StaticBehavior;

impl ObjectBehavior for StaticBehavior {
    fn step(&mut self, _world_interactor: &dyn WorldInteractor) -> Option<Action> {
        None
    }

    fn as_any(&self) -> &'_ dyn Any {
        self
    }
}

#[test]
fn simulation_runs() {
    let mut simulation = SimulationBuilder::new().build();
    let description = ObjectBuilder::default()
        .shape(
            PolygonBuilder::default()
                .vertex(-5.0, -5.0)
                .vertex(5.0, -5.0)
                .vertex(5.0, 5.0)
                .vertex(-5.0, 5.0)
                .build()
                .unwrap(),
        )
        .location(10.0, 15.0)
        .mobility(Mobility::Immovable)
        .build()
        .unwrap();

    let behavior: Box<ObjectBehavior> = Box::new(StaticBehavior::default());
    simulation.add_object(description, behavior);

    simulation.step();
    simulation.step();
}
