use myelin_engine::prelude::*;
use myelin_engine::simulation::SimulationBuilder;

#[derive(Debug, Clone, Default)]
struct StaticBehavior;

impl<T> ObjectBehavior<T> for StaticBehavior
where
    T: AssociatedObjectData,
{
    fn step(&mut self, _world_interactor: Box<dyn WorldInteractor<T> + '_>) -> Option<Action<T>> {
        None
    }
}

#[test]
fn simulation_runs() {
    let mut simulation = SimulationBuilder::new().build();
    let description = description();

    let behavior: Box<dyn ObjectBehavior<_>> = Box::new(StaticBehavior::default());
    simulation.add_object(description, behavior);

    simulation.step();
    simulation.step();
}

#[test]
fn simulation_returns_object() {
    let mut simulation = SimulationBuilder::new().build();
    let expected_description = description();

    let behavior: Box<dyn ObjectBehavior<_>> = Box::new(StaticBehavior::default());
    let object = simulation.add_object(expected_description.clone(), behavior);
    assert_eq!(expected_description, object.description);
}

#[test]
fn simulation_retrieves_object() {
    let (simulation, id, description) = {
        let mut simulation = SimulationBuilder::new().build();
        let expected_description = description();

        let behavior: Box<dyn ObjectBehavior<_>> = Box::new(StaticBehavior::default());
        let object = simulation.add_object(expected_description.clone(), behavior);
        let id = object.id;
        let description = object.description;
        (simulation, id, description)
    };
    let retrieved_object = simulation.object(id).unwrap();
    assert_eq!(retrieved_object.description, description);
}

fn description() -> ObjectDescription<()> {
    ObjectBuilder::default()
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
        .unwrap()
}
