//! A `Simulation` that outsources all physical
//! behaviour into a separate `World` type

pub mod builder;
pub mod time;
pub mod world;

use self::time::InstantWrapper;
use self::world::{BodyHandle, PhysicalBody, World};
use crate::prelude::*;
use crate::world_interactor::Interactable;
use nameof::name_of_type;
use std::cell::RefCell;
use std::collections::HashMap;
use std::error::Error;
use std::fmt::{self, Debug};
use std::time::Duration;

pub use self::builder::SimulationBuilder;

/// Factory used by [`SimulationImpl`] to create an [`WorldInteractor`].
///
/// [`SimulationImpl`]: ./struct.SimulationImpl.html
/// [`WorldInteractor`]: ./../object/trait.WorldInteractor.html
pub type WorldInteractorFactoryFn =
    dyn for<'a> Fn(&'a dyn Interactable) -> Box<dyn WorldInteractor + 'a>;

/// Factory to retrieve a current [`Instant`], wrapped by [`InstantWrapper`]
///
/// [`Instant`]: https://doc.rust-lang.org/std/time/struct.Instant.html
pub type InstantWrapperFactoryFn = dyn Fn() -> Box<dyn InstantWrapper>;

/// Implementation of [`Simulation`] that uses a physical
/// [`World`] in order to apply physics to objects.
///
/// [`Simulation`]: ./../trait.Simulation.html
/// [`World`]: ./trait.World.html
pub struct SimulationImpl {
    world: Box<dyn World>,
    non_physical_object_data: HashMap<BodyHandle, NonPhysicalObjectData>,
    world_interactor_factory_fn: Box<WorldInteractorFactoryFn>,
    instant_wrapper_factory_fn: Box<InstantWrapperFactoryFn>,
    last_step_instant: Option<Box<dyn InstantWrapper>>,
}

impl Debug for SimulationImpl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct(name_of_type!(SimulationImpl))
            .field("world", &self.world)
            .field("non_physical_object_data", &self.non_physical_object_data)
            .finish()
    }
}

#[derive(Debug)]
struct NonPhysicalObjectData {
    pub(crate) behavior: RefCell<Box<dyn ObjectBehavior>>,
    pub(crate) associated_data: Vec<u8>,
}

/// An error that can occur whenever an action is performed
#[derive(Debug, Clone)]
pub enum ActionError {
    /// The given handle was invalid
    InvalidHandle,
}

impl fmt::Display for ActionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Invalid handle")
    }
}

impl Error for ActionError {}

type ActionResult = Result<(), ActionError>;

impl SimulationImpl {
    /// Create a new SimulationImpl by injecting a [`World`]
    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    /// use myelin_engine::simulation::time::InstantWrapperImpl;
    /// use myelin_engine::simulation::world::{
    ///     rotation_translator::NphysicsRotationTranslatorImpl, NphysicsWorld,
    /// };
    /// use myelin_engine::simulation::SimulationImpl;
    /// use myelin_engine::world_interactor::WorldInteractorImpl;
    /// use std::sync::{Arc, RwLock};
    /// use std::time::Instant;
    ///
    /// let rotation_translator = NphysicsRotationTranslatorImpl::default();
    /// let world = Box::new(NphysicsWorld::with_timestep(
    ///     1.0,
    ///     Box::new(rotation_translator),
    /// ));
    /// let simulation = SimulationImpl::new(
    ///     world,
    ///     Box::new(|simulation| Box::new(WorldInteractorImpl::new(simulation))),
    ///     Box::new(|| Box::new(InstantWrapperImpl::new(Instant::now()))),
    /// );
    /// ```
    /// [`World`]: ./trait.World.html
    pub fn new(
        world: Box<dyn World>,
        world_interactor_factory_fn: Box<WorldInteractorFactoryFn>,
        instant_wrapper_factory_fn: Box<InstantWrapperFactoryFn>,
    ) -> Self {
        Self {
            world,
            non_physical_object_data: HashMap::new(),
            world_interactor_factory_fn,
            instant_wrapper_factory_fn,
            last_step_instant: None,
        }
    }

    fn handle_to_description(&self, body_handle: BodyHandle) -> Option<ObjectDescription> {
        let physics_body = self.world.body(body_handle)?;
        let non_physical_object_data = self.non_physical_object_data.get(&body_handle)?;
        Some(ObjectDescription {
            shape: physics_body.shape,
            location: physics_body.location,
            rotation: physics_body.rotation,
            mobility: physics_body.mobility,
            passable: self.world.is_body_passable(body_handle),
            associated_data: non_physical_object_data.associated_data.clone(),
        })
    }

    fn handle_action(&mut self, body_handle: BodyHandle, action: Action) -> ActionResult {
        match action {
            Action::Spawn(object_description, object_behavior) => {
                self.spawn(object_description, object_behavior)
            }
            Action::ApplyForce(force) => self.apply_force(body_handle, force),
            Action::Destroy(object_id) => self.destroy(object_id),
            Action::DestroySelf => self.destroy_self(body_handle),
        }
    }

    fn spawn(
        &mut self,
        object_description: ObjectDescription,
        object_behavior: Box<dyn ObjectBehavior>,
    ) -> ActionResult {
        self.add_object(object_description, object_behavior);
        Ok(())
    }

    fn apply_force(&mut self, body_handle: BodyHandle, force: Force) -> ActionResult {
        self.world
            .apply_force(body_handle, force)
            .to_action_result()
    }

    fn destroy(&mut self, object_id: Id) -> ActionResult {
        self.world
            .remove_body(BodyHandle(object_id))
            .to_action_result()
    }

    fn destroy_self(&mut self, body_handle: BodyHandle) -> ActionResult {
        self.world
            .remove_body(body_handle)
            .and(self.non_physical_object_data.remove(&body_handle))
            .to_action_result()
    }

    fn handle_to_object(&self, handle: BodyHandle) -> Object<'_> {
        Object {
            id: handle.0,
            description: self
                .handle_to_description(handle)
                .expect("Handle stored in simulation was not found in world"),
            behavior: self
                .handle_to_behavior(handle)
                .expect("Handle stored in simulation was not found in world"),
        }
    }

    fn handle_to_behavior(&self, handle: BodyHandle) -> Option<&dyn ObjectBehavior> {
        let ptr = self
            .non_physical_object_data
            .get(&handle)?
            .behavior
            .as_ptr();
        // This is safe as long as we don't hand out `&mut dyn ObjectBehavior` anywhere, ever.
        // We still have some guarantees regarding the borrow checker, as we currently hand out
        // `Vec<Object<'a>>`, where we maintain Rust's borrowing guarantees.
        // Only the `ObjectBehavior` of a given `Object` is borrowed unsafely, which is fine,
        // as the only possible way to mutate it is through `step`.
        // If you have an idea of how to restructure `Simulation` so that this unsafe call is no longer
        // needed, be my guest.
        let ref_to_behavior = unsafe { ptr.as_ref() }.unwrap().as_ref();
        Some(ref_to_behavior)
    }
}

trait HandleOption {
    fn to_action_result(self) -> ActionResult;
}

impl<T> HandleOption for Option<T> {
    fn to_action_result(self) -> ActionResult {
        self.map(|_| ()).ok_or(ActionError::InvalidHandle)
    }
}

impl Simulation for SimulationImpl {
    fn step(&mut self) {
        let object_handle_to_own_description: HashMap<_, _> = self
            .non_physical_object_data
            .keys()
            .map(|&object_handle| {
                (
                    object_handle,
                    self.handle_to_description(object_handle)
                        .expect("Internal error: Stored handle was invalid"),
                )
            })
            .collect();
        let mut actions = Vec::new();

        {
            let world_interactor = (self.world_interactor_factory_fn)(self);
            for (object_handle, non_physical_object_data) in &self.non_physical_object_data {
                let own_description = &object_handle_to_own_description[object_handle];
                let action = non_physical_object_data
                    .behavior
                    .borrow_mut()
                    .step(&own_description, world_interactor.as_ref());
                if let Some(action) = action {
                    actions.push((*object_handle, action));
                }
            }
        }

        for (body_handle, action) in actions {
            self.handle_action(body_handle, action)
                .expect("Body handle was not found within world");
        }
        self.world.step();
        self.last_step_instant = Some((self.instant_wrapper_factory_fn)());
    }

    fn add_object(
        &mut self,
        object_description: ObjectDescription,
        object_behavior: Box<dyn ObjectBehavior>,
    ) -> Object<'_> {
        let returned_object_descripton = object_description.clone();

        let physical_body = PhysicalBody {
            shape: object_description.shape,
            location: object_description.location,
            rotation: object_description.rotation,
            mobility: object_description.mobility,
            passable: object_description.passable,
        };

        let body_handle = self.world.add_body(physical_body);

        let non_physical_object_data = NonPhysicalObjectData {
            behavior: RefCell::new(object_behavior),
            associated_data: object_description.associated_data.clone(),
        };
        self.non_physical_object_data
            .insert(body_handle, non_physical_object_data);

        Object {
            id: body_handle.0,
            description: returned_object_descripton,
            behavior: self
                .handle_to_behavior(body_handle)
                .expect("Internal error: Handle of newly created body was invalid"),
        }
    }

    fn objects(&self) -> Snapshot<'_> {
        self.non_physical_object_data
            .keys()
            .map(|&handle| self.handle_to_object(handle))
            .collect()
    }

    fn set_simulated_timestep(&mut self, timestep: f64) {
        assert!(timestep >= 0.0, "Cannot set timestep to a negative value");
        self.world.set_simulated_timestep(timestep)
    }

    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
        self.world
            .bodies_in_area(area)
            .into_iter()
            .map(|handle| self.handle_to_object(handle))
            .collect()
    }

    fn objects_in_polygon(&self, area: Polygon) -> Snapshot<'_> {
        self.world
            .bodies_in_polygon(area)
            .into_iter()
            .map(|handle| self.handle_to_object(handle))
            .collect()
    }
}

impl Interactable for SimulationImpl {
    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
        Simulation::objects_in_area(self, area)
    }

    fn elapsed_time_in_update(&self) -> Duration {
        match &self.last_step_instant {
            Some(last_step_instant) => last_step_instant.to_inner().elapsed(),
            None => Duration::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use self::time::{InstantWrapperImpl, InstantWrapperMock};
    use super::*;
    use crate::object::ObjectBehaviorMock;
    use crate::object_builder::ObjectBuilder;
    use crate::simulation::simulation_impl::world::WorldMock;
    use crate::world_interactor::{Interactable, WorldInteractor, WorldInteractorMock};
    use mockiato::partial_eq;
    use myelin_geometry::PolygonBuilder;
    use std::thread::sleep;
    use std::time::Instant;

    fn world_interactor_factory_fn<'a>(
        _interactable: &'a dyn Interactable,
    ) -> Box<dyn WorldInteractor + 'a> {
        box WorldInteractorMock::new()
    }

    fn instant_wrapper_factory_fn() -> Box<dyn InstantWrapper> {
        box InstantWrapperMock::new()
    }

    #[test]
    fn propagates_step() {
        let mut world = box WorldMock::new();
        world.expect_step();
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        simulation.step();
    }

    #[test]
    fn propagates_simulated_timestep() {
        let mut world = box WorldMock::new();
        const EXPECTED_TIMESTEP: f64 = 1.0;
        world.expect_set_simulated_timestep(partial_eq(EXPECTED_TIMESTEP));
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        simulation.set_simulated_timestep(EXPECTED_TIMESTEP);
    }

    #[should_panic]
    #[test]
    fn panics_on_negative_timestep() {
        let world = box WorldMock::new();
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        const INVALID_TIMESTEP: f64 = -0.1;
        simulation.set_simulated_timestep(INVALID_TIMESTEP);
    }

    #[test]
    fn propagates_zero_timestep() {
        let mut world = box WorldMock::new();
        const EXPECTED_TIMESTEP: f64 = 0.0;
        world.expect_set_simulated_timestep(partial_eq(EXPECTED_TIMESTEP));
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        simulation.set_simulated_timestep(EXPECTED_TIMESTEP);
    }

    #[test]
    fn returns_no_objects_when_empty() {
        let world = box WorldMock::new();
        let simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        let objects = simulation.objects();
        assert!(objects.is_empty())
    }

    #[test]
    fn converts_to_physical_body() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1337);
        world
            .expect_add_body(partial_eq(expected_physical_body))
            .returns(returned_handle);

        let object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .passable(expected_passable)
            .build()
            .unwrap();
        let object_behavior = ObjectBehaviorMock::new();

        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        simulation.add_object(object_description, box object_behavior);
    }

    #[test]
    fn propagates_step_to_added_object() {
        let mut world = WorldMock::new();
        world.expect_step();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1337);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body));
        world
            .expect_is_body_passable(partial_eq(returned_handle))
            .returns(expected_passable);

        let expected_object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .passable(expected_passable)
            .build()
            .unwrap();

        let mut object_behavior = ObjectBehaviorMock::new();
        object_behavior.expect_step_and_return(&expected_object_description, None);

        let mut simulation = SimulationImpl::new(
            box world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        simulation.add_object(expected_object_description, box object_behavior);
        simulation.step();
    }

    #[test]
    fn returns_added_object() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1984);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body));
        world
            .expect_is_body_passable(partial_eq(returned_handle))
            .returns(expected_passable);

        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );
        let object_behavior = ObjectBehaviorMock::new();

        let expected_object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .passable(expected_passable)
            .build()
            .unwrap();

        simulation.add_object(expected_object_description.clone(), box object_behavior);

        let objects = simulation.objects();
        assert_eq!(1, objects.len());

        let object_description = &objects.iter().next().unwrap().description;
        assert_eq!(expected_object_description, *object_description);
    }

    // Something seems fishy with the following test
    // It fails because the expected step from the child
    // is not called.
    // Removing the expected step from the child
    // results in step being called unexpectedly.
    // I suspect it's a problem resulting from our mock
    // returning the same handle twice.
    #[ignore]
    #[test]
    fn spawns_object() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1984);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body));
        world.expect_step();

        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let mut object_behavior = ObjectBehaviorMock::new();

        let expected_object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .build()
            .unwrap();

        let mut child_object_behavior = ObjectBehaviorMock::new();
        child_object_behavior.expect_step_and_return(&expected_object_description, None);

        object_behavior.expect_step_and_return(
            &expected_object_description,
            Some(Action::Spawn(
                expected_object_description.clone(),
                box child_object_behavior,
            )),
        );

        simulation.add_object(expected_object_description.clone(), box object_behavior);

        simulation.step();
        simulation.step();
    }

    #[test]
    fn destroying_self_removes_object() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1984);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body.clone()));
        world.expect_step();
        world
            .expect_remove_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body));
        world
            .expect_is_body_passable(partial_eq(returned_handle))
            .returns(expected_passable);

        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let mut object_behavior = ObjectBehaviorMock::new();

        let expected_object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .passable(expected_passable)
            .build()
            .unwrap();

        object_behavior
            .expect_step_and_return(&expected_object_description, Some(Action::DestroySelf));

        simulation.add_object(expected_object_description.clone(), box object_behavior);

        simulation.step();
    }

    #[test]
    fn destroy_removes_object() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };

        let handle_one = BodyHandle(1);
        let handle_two = BodyHandle(2);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(handle_one);
        world
            .expect_body(partial_eq(handle_one))
            .returns(Some(expected_physical_body.clone()));
        world
            .expect_is_body_passable(partial_eq(handle_one))
            .returns(expected_passable);
        world.expect_step();
        world
            .expect_remove_body(partial_eq(handle_two))
            .returns(Some(expected_physical_body));

        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let mut object_behavior = ObjectBehaviorMock::new();

        let expected_object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .passable(expected_passable)
            .build()
            .unwrap();

        object_behavior.expect_step_and_return(
            &expected_object_description,
            Some(Action::Destroy(handle_two.0)),
        );

        simulation.add_object(expected_object_description.clone(), box object_behavior);

        simulation.step();
    }

    #[test]
    fn force_application_is_propagated() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1984);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body.clone()));
        world.expect_step();
        world
            .expect_is_body_passable(partial_eq(returned_handle))
            .returns(expected_passable);
        let expected_force = Force {
            linear: Vector { x: 20.0, y: -5.0 },
            torque: Torque(-8.0),
        };
        world
            .expect_apply_force(
                partial_eq(returned_handle),
                partial_eq(expected_force.clone()),
            )
            .returns(Some(()));

        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let mut object_behavior = ObjectBehaviorMock::new();

        let expected_object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .passable(expected_passable)
            .build()
            .unwrap();

        object_behavior.expect_step_and_return(
            &expected_object_description,
            Some(Action::ApplyForce(expected_force)),
        );

        simulation.add_object(expected_object_description.clone(), box object_behavior);

        simulation.step();
    }

    #[should_panic]
    #[test]
    fn panics_on_invalid_body() {
        let mut world = box WorldMock::new();
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;

        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let returned_handle = BodyHandle(1984);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world.expect_body(partial_eq(returned_handle)).returns(None);
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .build()
            .unwrap();

        let object_behavior = ObjectBehaviorMock::new();
        simulation.add_object(object_description, box object_behavior);
        simulation.objects();
    }

    #[test]
    fn propagates_objects_in_area() {
        let mut world = WorldMock::new();
        let (expected_physical_body, object_description) = object();
        let area = Aabb {
            upper_left: Point { x: 30.0, y: 30.0 },
            lower_right: Point { x: 10.0, y: 10.0 },
        };

        let returned_handle = BodyHandle(1234);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_bodies_in_area(partial_eq(area))
            .returns(vec![returned_handle]);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body.clone()));
        world
            .expect_is_body_passable(partial_eq(returned_handle))
            .returns(expected_physical_body.passable);

        let mut simulation = SimulationImpl::new(
            box world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let object_behavior = box ObjectBehaviorMock::new();
        simulation.add_object(object_description.clone(), object_behavior);

        let objects_in_area = Simulation::objects_in_area(&simulation, area);
        assert_eq!(1, objects_in_area.len());
        assert_eq!(returned_handle.0, objects_in_area[0].id);
        assert_eq!(object_description, objects_in_area[0].description);
    }

    #[test]
    #[should_panic]
    fn objects_in_area_panics_when_given_invalid_handle() {
        let mut world = WorldMock::new();
        let (expected_physical_body, object_description) = object();
        let area = Aabb {
            upper_left: Point { x: 30.0, y: 30.0 },
            lower_right: Point { x: 10.0, y: 10.0 },
        };

        let returned_handle = BodyHandle(1234);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_bodies_in_area(partial_eq(area))
            .returns(vec![returned_handle]);
        world.expect_body(partial_eq(returned_handle)).returns(None);

        let mut simulation = SimulationImpl::new(
            box world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let object_behavior = ObjectBehaviorMock::new();
        simulation.add_object(object_description.clone(), box object_behavior);
    }

    #[test]
    fn propagates_objects_in_polygon() {
        let mut world = WorldMock::new();
        let (expected_physical_body, object_description) = object();
        let area = PolygonBuilder::default()
            .vertex(30.0, 30.0)
            .vertex(45.0, 15.0)
            .vertex(60.0, 30.0)
            .build()
            .unwrap();
        let returned_handle = BodyHandle(1234);
        world
            .expect_add_body(partial_eq(expected_physical_body.clone()))
            .returns(returned_handle);
        world
            .expect_bodies_in_polygon(partial_eq(area.clone()))
            .returns(vec![returned_handle]);
        world
            .expect_body(partial_eq(returned_handle))
            .returns(Some(expected_physical_body.clone()));
        world
            .expect_is_body_passable(partial_eq(returned_handle))
            .returns(expected_physical_body.passable);

        let mut simulation = SimulationImpl::new(
            box world,
            box world_interactor_factory_fn,
            box instant_wrapper_factory_fn,
        );

        let object_behavior = box ObjectBehaviorMock::new();
        simulation.add_object(object_description.clone(), object_behavior);

        let objects_in_polygon = Simulation::objects_in_polygon(&simulation, area);
        assert_eq!(1, objects_in_polygon.len());
        assert_eq!(returned_handle.0, objects_in_polygon[0].id);
        assert_eq!(object_description, objects_in_polygon[0].description);
    }

    fn object() -> (PhysicalBody, ObjectDescription) {
        let expected_shape = shape();
        let expected_location = location();
        let expected_rotation = rotation();
        let expected_mobility = Mobility::Movable(Vector::default());
        let expected_passable = false;
        let expected_physical_body = PhysicalBody {
            shape: expected_shape.clone(),
            location: expected_location,
            rotation: expected_rotation,
            mobility: expected_mobility.clone(),
            passable: expected_passable,
        };
        let object_description = ObjectBuilder::default()
            .location(expected_location.x, expected_location.y)
            .rotation(expected_rotation)
            .shape(expected_shape)
            .mobility(expected_mobility)
            .build()
            .unwrap();

        (expected_physical_body, object_description)
    }

    fn shape() -> Polygon {
        PolygonBuilder::default()
            .vertex(-5.0, -5.0)
            .vertex(5.0, -5.0)
            .vertex(5.0, 5.0)
            .vertex(-5.0, 5.0)
            .build()
            .unwrap()
    }

    fn location() -> Point {
        Point { x: 30.0, y: 40.0 }
    }

    fn rotation() -> Radians {
        Radians::try_new(3.4).unwrap()
    }

    #[test]
    fn elapsed_time_in_update_is_initially_at_zero() {
        let world = box WorldMock::new();
        let simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box real_instant_wrapper_factory_fn,
        );
        let elapsed_time = simulation.elapsed_time_in_update();
        let no_time = Duration::default();
        assert_eq!(no_time, elapsed_time);
    }

    #[test]
    fn elapsed_time_in_update_is_correct_after_step() {
        let mut world = box WorldMock::new();
        world.expect_step();
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box real_instant_wrapper_factory_fn,
        );
        simulation.step();
        let sleep_time = Duration::from_millis(15);
        sleep(sleep_time);
        let elapsed_time = simulation.elapsed_time_in_update();

        assert!(elapsed_time >= sleep_time && elapsed_time <= MAX_DURATION);
    }

    #[test]
    fn elapsed_time_in_update_is_correct_after_multiple_steps() {
        let mut world = box WorldMock::new();
        world.expect_step().times(2);
        let mut simulation = SimulationImpl::new(
            world,
            box world_interactor_factory_fn,
            box real_instant_wrapper_factory_fn,
        );
        simulation.step();
        let sleep_time = Duration::from_millis(15);
        sleep(sleep_time);
        simulation.step();
        sleep(sleep_time);
        let elapsed_time = simulation.elapsed_time_in_update();

        assert!(elapsed_time >= sleep_time && elapsed_time <= MAX_DURATION);
    }

    fn real_instant_wrapper_factory_fn() -> Box<dyn InstantWrapper> {
        box InstantWrapperImpl::new(Instant::now())
    }

    const MAX_DURATION: Duration = Duration::from_millis(20);
}