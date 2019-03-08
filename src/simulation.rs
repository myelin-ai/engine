//! Entrypoint for the Simulaton API handling the world itself

mod simulation_impl;

pub use self::simulation_impl::*;
use crate::prelude::*;
use std::fmt::Debug;

#[cfg(any(test, feature = "use-mocks"))]
pub use self::mocks::*;

/// A Simulation that can be filled with [`Object`] on
/// which it will apply physical rules when calling [`step`].
/// This trait represents our API.
///
/// [`Object`]: ./object/struct.Object.html
/// [`step`]: ./trait.Simulation.html#tymethod.step
pub trait Simulation: Debug {
    /// Advance the simulation by one tick. This will apply
    /// forces to the objects, handle collisions and allow them to
    /// take action.
    fn step(&mut self);

    /// Add a new object to the world.
    fn add_object(
        &mut self,
        object_description: ObjectDescription,
        object_behavior: Box<dyn ObjectBehavior>,
    ) -> Object<'_>;

    /// Returns a read-only description of all objects currently inhabiting the simulation.
    fn objects(&self) -> Snapshot<'_>;

    /// Returns a read-only description an object, if the provided ID is valid.
    fn object(&self, id: Id) -> Option<Object<'_>>;

    /// Sets how much time in seconds is simulated for each step.
    /// # Examples
    /// If you want to run a simulation with 60 steps per second, you
    /// can run `set_simulated_timestep(1.0/60.0)`. Note that this method
    /// does not block the thread if called faster than expected.
    fn set_simulated_timestep(&mut self, timestep: f64);

    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_>;

    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_polygon(&self, area: &Polygon) -> Snapshot<'_>;

    /// Returns read-only descriptions for all objects
    /// intersecting with the given vector.
    fn objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_>;
}

/// Unique identifier of an Object
pub type Id = usize;

/// A representation of the current state of the simulation
pub type Snapshot<'a> = Vec<Object<'a>>;

#[cfg(any(test, feature = "use-mocks"))]
mod mocks {
    use super::*;
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::thread::panicking;

    #[derive(Debug, Clone)]
    enum AddObjectExpectation<Input, ReturnValue> {
        None,
        Any(ReturnValue),
        AtLeastOnce(Input, ReturnValue),
    }

    impl<Input, ReturnValue> Default for AddObjectExpectation<Input, ReturnValue> {
        fn default() -> Self {
            AddObjectExpectation::None
        }
    }

    #[derive(Debug, Clone)]
    enum ObjectsInAreaExpectation<Input, ReturnValue> {
        None,
        AtLeastOnce(Input, ReturnValue),
        Sequence(RefCell<VecDeque<(Input, ReturnValue)>>),
    }

    impl<Input, ReturnValue> Default for ObjectsInAreaExpectation<Input, ReturnValue> {
        fn default() -> Self {
            ObjectsInAreaExpectation::None
        }
    }

    /// Mock for [`Simulation`]
    ///
    /// [`Simulation`]: ../trait.Simulation.html
    #[derive(Debug, Default)]
    pub struct SimulationMock<'a> {
        expect_step: Option<()>,
        expect_objects_and_return: Option<(Snapshot<'a>,)>,
        expect_object_and_return: AddObjectExpectation<Id, Option<FullyOwnedObject>>,
        expect_add_object_and_return: AddObjectExpectation<ObjectDescription, FullyOwnedObject>,
        expect_set_simulated_timestep: Option<(f64,)>,
        expect_objects_in_area_and_return: ObjectsInAreaExpectation<Aabb, Snapshot<'a>>,
        expect_objects_in_polygon_and_return: ObjectsInAreaExpectation<Polygon, Snapshot<'a>>,
        expect_objects_in_ray_and_return: ObjectsInAreaExpectation<(Point, Vector), Snapshot<'a>>,

        step_was_called: RefCell<bool>,
        objects_was_called: RefCell<bool>,
        object_was_called: RefCell<bool>,
        add_object_was_called: RefCell<bool>,
        set_simulated_timestep_was_called: RefCell<bool>,
        objects_in_area_was_called: RefCell<bool>,
        objects_in_polygon_was_called: RefCell<bool>,
        objects_in_ray_was_called: RefCell<bool>,
    }

    /// A helper tuple with an owned [`ObjectBehavior`], used to assemble an [`Object`] in mocks.
    ///
    /// [`ObjectBehavior`]: ./object/trait.ObjectBehavior.html
    /// [`Object`]: ./object/struct.Object.html
    pub type FullyOwnedObject = (Id, ObjectDescription, Box<dyn ObjectBehavior>);

    impl<'a> SimulationMock<'a> {
        /// Construct a new `SimulationMock`
        pub fn new() -> Self {
            Default::default()
        }

        /// Expect a call to `step`
        pub fn expect_step(&mut self) {
            self.expect_step = Some(());
        }

        /// Expects an arbitrary amount of calls to `add_object` and return the specified object every time
        pub fn expect_add_object_any_times_and_return(&mut self, object: FullyOwnedObject) {
            self.expect_add_object_and_return = AddObjectExpectation::Any(object);
        }

        /// Expect a call to `add_object`
        pub fn expect_add_object_and_return(
            &mut self,
            object_description: ObjectDescription,
            return_value: FullyOwnedObject,
        ) {
            self.expect_add_object_and_return =
                AddObjectExpectation::AtLeastOnce(object_description, return_value);
        }

        /// Expect a call to `objects`
        pub fn expect_objects_and_return(&mut self, return_value: Snapshot<'a>) {
            self.expect_objects_and_return = Some((return_value,));
        }

        /// Expect a call to `set_simulated_timestep`
        pub fn expect_set_simulated_timestep(&mut self, timestep: f64) {
            self.expect_set_simulated_timestep = Some((timestep,));
        }

        /// Expect a call to `objects_in_area`
        pub fn expect_objects_in_area_and_return(
            &mut self,
            area: Aabb,
            return_value: Snapshot<'a>,
        ) {
            self.expect_objects_in_area_and_return =
                ObjectsInAreaExpectation::AtLeastOnce(area, return_value);
        }

        /// Expects a sequence of calls to `objects_in_area`
        pub fn expect_objects_in_area_and_return_in_sequence(
            &mut self,
            expected_calls_and_return_values: Vec<(Aabb, Snapshot<'a>)>,
        ) {
            self.expect_objects_in_area_and_return = ObjectsInAreaExpectation::Sequence(
                RefCell::new(expected_calls_and_return_values.into()),
            );
        }

        /// Expects a sequence of calls to `objects_in_ray`
        pub fn expect_objects_in_ray_and_return_in_sequence(
            &mut self,
            expected_calls_and_return_values: Vec<((Point, Vector), Snapshot<'a>)>,
        ) {
            self.expect_objects_in_ray_and_return = ObjectsInAreaExpectation::Sequence(
                RefCell::new(expected_calls_and_return_values.into()),
            );
        }
    }

    impl<'a> Simulation for SimulationMock<'a> {
        fn step(&mut self) {
            *self.step_was_called.borrow_mut() = true;
        }

        fn add_object(
            &mut self,
            object_description: ObjectDescription,
            _object_behavior: Box<dyn ObjectBehavior>,
        ) -> Object<'_> {
            *self.add_object_was_called.borrow_mut() = true;

            match &self.expect_add_object_and_return {
                AddObjectExpectation::None => panic!("add_object() was called unexpectedly"),
                AddObjectExpectation::Any((id, description, behavior)) => Object {
                    id: *id,
                    description: description.clone(),
                    behavior: behavior.as_ref(),
                },
                AddObjectExpectation::AtLeastOnce(expected_object_description, return_value) => {
                    assert_eq!(
                        *expected_object_description, object_description,
                        "add_object() was called with {:?}, expected {:?}",
                        object_description, expected_object_description
                    );

                    let (id, description, behavior) = return_value;
                    Object {
                        id: *id,
                        description: description.clone(),
                        behavior: behavior.as_ref(),
                    }
                }
            }
        }

        fn objects(&self) -> Snapshot<'_> {
            *self.objects_was_called.borrow_mut() = true;

            let (return_value,) = self
                .expect_objects_and_return
                .clone()
                .expect("objects() was called unexpectedly");

            return_value.clone()
        }

        fn object(&self, id: Id) -> Option<Object<'_>> {
            *self.object_was_called.borrow_mut() = true;

            match &self.expect_object_and_return {
                AddObjectExpectation::None => panic!("object() was called unexpectedly"),
                AddObjectExpectation::Any(return_value) => {
                    if let Some((id, description, behavior)) = return_value {
                        Some(Object {
                            id: *id,
                            description: description.clone(),
                            behavior: behavior.as_ref(),
                        })
                    } else {
                        None
                    }
                }
                AddObjectExpectation::AtLeastOnce(expected_id, return_value) => {
                    assert_eq!(
                        *expected_id, id,
                        "object() was called with {:?}, expected {:?}",
                        id, expected_id
                    );

                    if let Some((id, description, behavior)) = return_value {
                        Some(Object {
                            id: *id,
                            description: description.clone(),
                            behavior: behavior.as_ref(),
                        })
                    } else {
                        None
                    }
                }
            }
        }

        fn set_simulated_timestep(&mut self, timestep: f64) {
            *self.set_simulated_timestep_was_called.borrow_mut() = true;

            let (expected_timestep,) = self
                .expect_set_simulated_timestep
                .expect("set_simulated_timestep() was called unexpectedly");

            assert_eq!(
                expected_timestep, timestep,
                "set_simulated_timestep() was called with {:?}, expected {:?}",
                timestep, expected_timestep
            );
        }

        fn objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
            *self.objects_in_area_was_called.borrow_mut() = true;

            const UNEXPECTED_CALL_ERROR_MESSAGE: &str = "objects_in_area() was called unexpectedly";

            let (expected_area, return_value) = match self.expect_objects_in_area_and_return {
                ObjectsInAreaExpectation::None => panic!(UNEXPECTED_CALL_ERROR_MESSAGE),
                ObjectsInAreaExpectation::AtLeastOnce(ref expected_area, ref return_value) => {
                    (*expected_area, return_value.clone())
                }
                ObjectsInAreaExpectation::Sequence(ref expected_calls_and_return_values) => {
                    expected_calls_and_return_values
                        .borrow_mut()
                        .pop_front()
                        .expect(UNEXPECTED_CALL_ERROR_MESSAGE)
                }
            };

            assert_eq!(
                expected_area, area,
                "objects_in_area() was called with {:?}, expected {:?}",
                area, expected_area
            );

            return_value.clone()
        }

        fn objects_in_polygon(&self, area: &Polygon) -> Snapshot<'_> {
            *self.objects_in_polygon_was_called.borrow_mut() = true;

            const UNEXPECTED_CALL_ERROR_MESSAGE: &str =
                "objects_in_polygon() was called unexpectedly";

            let (expected_area, return_value) = match self.expect_objects_in_polygon_and_return {
                ObjectsInAreaExpectation::None => panic!(UNEXPECTED_CALL_ERROR_MESSAGE),
                ObjectsInAreaExpectation::AtLeastOnce(ref expected_area, ref return_value) => {
                    (expected_area.clone(), return_value.clone())
                }
                ObjectsInAreaExpectation::Sequence(ref expected_calls_and_return_values) => {
                    expected_calls_and_return_values
                        .borrow_mut()
                        .pop_front()
                        .expect(UNEXPECTED_CALL_ERROR_MESSAGE)
                }
            };

            assert_eq!(
                expected_area, *area,
                "objects_in_polygon() was called with {:?}, expected {:?}",
                *area, expected_area
            );

            return_value.clone()
        }

        fn objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_> {
            *self.objects_in_ray_was_called.borrow_mut() = true;

            const UNEXPECTED_CALL_ERROR_MESSAGE: &str = "objects_in_ray() was called unexpectedly";

            let ((expected_origin, expected_direction), return_value) = match self
                .expect_objects_in_ray_and_return
            {
                ObjectsInAreaExpectation::None => panic!(UNEXPECTED_CALL_ERROR_MESSAGE),
                ObjectsInAreaExpectation::AtLeastOnce(ref expected_direction, ref return_value) => {
                    (expected_direction.clone(), return_value.clone())
                }
                ObjectsInAreaExpectation::Sequence(ref expected_calls_and_return_values) => {
                    expected_calls_and_return_values
                        .borrow_mut()
                        .pop_front()
                        .expect(UNEXPECTED_CALL_ERROR_MESSAGE)
                }
            };

            assert_eq!(
                expected_direction, direction,
                "objects_in_ray() was called with direction = {:?}, expected {:?}",
                direction, expected_direction
            );

            assert_eq!(
                expected_origin, origin,
                "objects_in_ray() was called with origin = {:?}, expected {:?}",
                origin, expected_origin
            );

            return_value.clone()
        }
    }

    impl<'a> Drop for SimulationMock<'a> {
        fn drop(&mut self) {
            if panicking() {
                return;
            }

            assert!(
                self.expect_step.is_some() == *self.step_was_called.borrow(),
                "step() was not called, but expected"
            );

            if let AddObjectExpectation::AtLeastOnce(..) = self.expect_add_object_and_return {
                assert!(
                    *self.add_object_was_called.borrow(),
                    "add_object() was not called, but expected"
                );
            }

            assert!(
                self.expect_objects_and_return.is_some() == *self.objects_was_called.borrow(),
                "objects() was not called, but expected"
            );

            assert!(
                self.expect_set_simulated_timestep.is_some()
                    == *self.set_simulated_timestep_was_called.borrow(),
                "set_simulated_timestep() was not called, but expected"
            );

            if let ObjectsInAreaExpectation::AtLeastOnce(..)
            | ObjectsInAreaExpectation::Sequence(..) = self.expect_objects_in_area_and_return
            {
                assert!(
                    *self.objects_in_area_was_called.borrow(),
                    "objects_in_area() was not called, but expected"
                );
            }

            if let ObjectsInAreaExpectation::AtLeastOnce(..)
            | ObjectsInAreaExpectation::Sequence(..) = self.expect_objects_in_polygon_and_return
            {
                assert!(
                    *self.objects_in_polygon_was_called.borrow(),
                    "objects_in_polygon() was not called, but expected"
                );
            }

            if let ObjectsInAreaExpectation::AtLeastOnce(..)
            | ObjectsInAreaExpectation::Sequence(..) = self.expect_objects_in_ray_and_return
            {
                assert!(
                    *self.objects_in_ray_was_called.borrow(),
                    "objects_in_ray() was not called, but expected"
                );
            }
        }
    }
}
