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

    /// Sets how much time in seconds is simulated for each step.
    /// # Examples
    /// If you want to run a simulation with 60 steps per second, you
    /// can run `set_simulated_timestep(1.0/60.0)`. Note that this method
    /// does not block the thread if called faster than expected.
    fn set_simulated_timestep(&mut self, timestep: f64);

    /// Returns read-only descriptions for all objects either completely
    /// contained or intersecting with the given area.
    fn objects_in_area(&self, area: Aabb) -> Snapshot<'_>;
}

/// Unique identifier of an Object
pub type Id = usize;

/// A representation of the current state of the simulation
pub type Snapshot<'a> = Vec<Object<'a>>;

#[cfg(any(test, feature = "use-mocks"))]
mod mocks {
    use super::*;
    use std::cell::RefCell;
    use std::thread::panicking;

    /// Mock for [`Simulation`]
    ///
    /// [`Simulation`]: ../trait.Simulation.html
    #[derive(Debug, Default)]
    pub struct SimulationMock<'a> {
        expect_step: Option<()>,
        expect_objects_and_return: Option<(Snapshot<'a>,)>,
        expect_add_object_and_return: Option<(ObjectDescription, Object<'a>)>,
        expect_set_simulated_timestep: Option<(f64,)>,
        expect_objects_in_area_and_return: Option<(Aabb, Snapshot<'a>)>,

        step_was_called: RefCell<bool>,
        objects_was_called: RefCell<bool>,
        add_object_was_called: RefCell<bool>,
        set_simulated_timestep_was_called: RefCell<bool>,
        objects_in_area_was_called: RefCell<bool>,
    }

    impl<'a> SimulationMock<'a> {
        /// Construct a new `SimulationMock`
        pub fn new() -> Self {
            Default::default()
        }

        /// Expect a call to `step`
        pub fn expect_step(&mut self) {
            self.expect_step = Some(());
        }

        /// Expect a call to `add_object`
        pub fn expect_add_object_and_return(
            &mut self,
            object_description: ObjectDescription,
            return_value: Object<'a>,
        ) {
            self.expect_add_object_and_return = Some((object_description, return_value));
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
            self.expect_objects_in_area_and_return = Some((area, return_value));
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

            let (expected_object_description, return_value) = self
                .expect_add_object_and_return
                .clone()
                .expect("add_object() was called unexpectedly");

            assert_eq!(
                expected_object_description, object_description,
                "add_object() was called with {:?}, expected {:?}",
                object_description, expected_object_description
            );

            return_value.clone()
        }

        fn objects(&self) -> Snapshot<'_> {
            *self.objects_was_called.borrow_mut() = true;

            let (return_value,) = self
                .expect_objects_and_return
                .clone()
                .expect("objects() was called unexpectedly");

            return_value.clone()
        }

        fn set_simulated_timestep(&mut self, timestep: f64) {
            *self.set_simulated_timestep_was_called.borrow_mut() = true;

            let (expected_timestep,) = self
                .expect_set_simulated_timestep
                .clone()
                .expect("set_simulated_timestep() was called unexpectedly");

            assert_eq!(
                expected_timestep, timestep,
                "set_simulated_timestep() was called with {:?}, expected {:?}",
                timestep, expected_timestep
            );
        }

        fn objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
            *self.objects_in_area_was_called.borrow_mut() = true;

            let (expected_area, return_value) = self
                .expect_objects_in_area_and_return
                .clone()
                .expect("objects_in_area() was called unexpectedly");

            assert_eq!(
                expected_area, area,
                "objects_in_area() was called with {:?}, expected {:?}",
                area, expected_area
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

            assert!(
                self.expect_add_object_and_return.is_some() == *self.add_object_was_called.borrow(),
                "add_object() was not called, but expected"
            );

            assert!(
                self.expect_objects_and_return.is_some() == *self.objects_was_called.borrow(),
                "objects() was not called, but expected"
            );

            assert!(
                self.expect_set_simulated_timestep.is_some()
                    == *self.set_simulated_timestep_was_called.borrow(),
                "set_simulated_timestep() was not called, but expected"
            );

            assert!(
                self.expect_objects_in_area_and_return.is_some()
                    == *self.objects_in_area_was_called.borrow(),
                "objects_in_area() was not called, but expected"
            );
        }
    }
}
