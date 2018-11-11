//! Implementations of [`ForceGenerator`], which provide
//! the interface used to apply a [`Force`] on a body

use super::{PhysicsType, SingleTimeForceApplier};
use crate::object::Force;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::math::Force as NphysicsForce;
use nphysics2d::object::{BodyHandle, BodySet};
use nphysics2d::solver::IntegrationParameters;
use std::borrow::BorrowMut;
use std::collections::HashMap;

/// A [`ForceGenerator`] that applies a given force exactly once
#[derive(Default, Debug)]
pub struct SingleTimeForceApplierImpl {
    forces_to_apply: HashMap<BodyHandle, Force>,
}

impl SingleTimeForceApplier for SingleTimeForceApplierImpl {
    fn register_force(&mut self, handle: BodyHandle, force: Force) {
        self.forces_to_apply.insert(handle, force);
    }
}

impl ForceGenerator<PhysicsType> for SingleTimeForceApplierImpl {
    fn apply(
        &mut self,
        _: &IntegrationParameters<PhysicsType>,
        bodies: &mut BodySet<PhysicsType>,
    ) -> bool {
        for (body_handle, force) in self.forces_to_apply.drain() {
            if bodies.contains(body_handle) {
                let mut body = bodies.body_part_mut(body_handle);
                let nphysics_force = NphysicsForce::from_slice(&[
                    PhysicsType::from(force.linear.x),
                    PhysicsType::from(force.linear.y),
                    force.torque.0,
                ]);
                body.apply_force(&nphysics_force);
            }
        }

        const KEEP_FORCE_GENERATOR_AFTER_APPLICATION: bool = true;
        KEEP_FORCE_GENERATOR_AFTER_APPLICATION
    }
}

/// A wrapper that is used to implement [`ForceGenerator`] on a box of [`SingleTimeForceApplier`].
/// This is used to make [`SingleTimeForceApplier`] mockable.
#[derive(Debug)]
pub struct GenericSingleTimeForceApplierWrapper {
    force_applier: Box<dyn SingleTimeForceApplier>,
}

impl GenericSingleTimeForceApplierWrapper {
    /// Constructs a new wrapper around a [`SingleTimeForceApplier`]
    pub(crate) fn new(force_applier: Box<dyn SingleTimeForceApplier>) -> Self {
        Self { force_applier }
    }

    /// Retrieves the wrapped [`SingleTimeForceApplier`]
    pub(crate) fn inner_mut(&mut self) -> &mut dyn SingleTimeForceApplier {
        self.force_applier.borrow_mut()
    }
}

impl ForceGenerator<PhysicsType> for GenericSingleTimeForceApplierWrapper {
    fn apply(
        &mut self,
        integration_parameters: &IntegrationParameters<PhysicsType>,
        body_set: &mut BodySet<PhysicsType>,
    ) -> bool {
        self.force_applier.apply(integration_parameters, body_set)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::object::*;
    use crate::object_builder::PolygonBuilder;
    use crate::simulation_impl::world::collision_filter::IgnoringCollisionFilterMock;
    use crate::simulation_impl::world::rotation_translator::NphysicsRotationTranslatorImpl;
    use crate::simulation_impl::world::{NphysicsWorld, PhysicalBody, World};
    use std::collections::VecDeque;
    use std::sync::{Arc, RwLock};

    const DEFAULT_TIMESTEP: f64 = 1.0;

    #[test]
    fn can_be_injected() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let force_applier = SingleTimeForceApplierImpl::default();
        let collision_filter = Arc::new(RwLock::new(IgnoringCollisionFilterMock::default()));
        let _world = NphysicsWorld::with_timestep(
            DEFAULT_TIMESTEP,
            box rotation_translator,
            box force_applier,
            collision_filter,
        );
    }

    #[test]
    fn force_does_nothing_before_step() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let force_applier = SingleTimeForceApplierImpl::default();
        let collision_filter = Arc::new(RwLock::new(IgnoringCollisionFilterMock::default()));
        let mut world = NphysicsWorld::with_timestep(
            DEFAULT_TIMESTEP,
            box rotation_translator,
            box force_applier,
            collision_filter.clone(),
        );

        let expected_object = physical_body();
        let handle = world.add_body(expected_object.clone());

        collision_filter
            .write()
            .expect("RwLock was poisoned")
            .expect_is_handle_ignored_and_return(VecDeque::from(vec![(handle.into(), false)]));

        let force = Force {
            linear: LinearForce {
                x: 1000.0,
                y: 2000.0,
            },
            torque: Torque(9.0),
        };
        world
            .apply_force(handle, force)
            .expect("Invalid object handle");

        let actual_body = world.body(handle);
        assert_eq!(Some(expected_object), actual_body);
    }

    #[test]
    fn zero_force_is_ignored() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce::default(),
            torque: Torque::default(),
        };
        let expected_body = body.clone();
        test_force(&body, &expected_body, force);
    }

    #[test]
    fn torque_with_no_linear_force_changes_rotation() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce::default(),
            torque: Torque(101.55),
        };
        let expected_body = PhysicalBody {
            position: Position {
                rotation: Radians::try_new(0.6093).unwrap(),
                ..body.position.clone()
            },
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn negative_torque_results_in_negative_rotation() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce::default(),
            torque: Torque(-202.0),
        };
        let expected_body = PhysicalBody {
            position: Position {
                rotation: Radians::try_new(5.071_185_307_179_586_5).unwrap(),
                ..body.position.clone()
            },
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn linear_force_with_no_torque_changes_location_and_speed() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce { x: 100.0, y: 100.0 },
            torque: Torque::default(),
        };
        let expected_body = PhysicalBody {
            position: Position {
                location: Location { x: 14.0, y: 14.0 },
                ..body.position.clone()
            },
            mobility: Mobility::Movable(Velocity { x: 9.0, y: 9.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn negative_linear_force_results_in_lower_location() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce { x: -50.0, y: -50.0 },
            torque: Torque::default(),
        };
        let expected_body = PhysicalBody {
            position: Position {
                location: Location { x: 0.0, y: 0.0 },
                ..body.position.clone()
            },
            mobility: Mobility::Movable(Velocity { x: -4.0, y: -4.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn location_can_underflow() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce {
                x: -100.0,
                y: -200.0,
            },
            torque: Torque::default(),
        };
        let expected_body = PhysicalBody {
            position: Position {
                location: Location {
                    x: 4.0_294_967_292,
                    y: 4.0_294_967_282,
                },
                ..body.position.clone()
            },
            mobility: Mobility::Movable(Velocity { x: -9.0, y: -19.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn linear_force_and_torque_can_be_combined() {
        let body = physical_body();
        let force = Force {
            linear: LinearForce { x: 50.0, y: 100.0 },
            torque: Torque(1.5),
        };

        let expected_body = PhysicalBody {
            position: Position {
                location: Location { x: 10.0, y: 15.0 },
                rotation: Radians::try_new(0.009_000_000_000_000_001).unwrap(),
            },
            mobility: Mobility::Movable(Velocity { x: 4.0, y: 9.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    fn physical_body() -> PhysicalBody {
        PhysicalBody {
            position: Position {
                location: Location { x: 5.0, y: 5.0 },
                rotation: Radians::default(),
            },
            mobility: Mobility::Movable(Velocity::default()),
            shape: PolygonBuilder::default()
                .vertex(-5.0, -5.0)
                .vertex(-5.0, 5.0)
                .vertex(5.0, 5.0)
                .vertex(5.0, -5.0)
                .build()
                .unwrap(),
            passable: false,
        }
    }

    fn test_force(body: &PhysicalBody, expected_body: &PhysicalBody, force: Force) {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let force_applier = SingleTimeForceApplierImpl::default();
        let collision_filter = Arc::new(RwLock::new(IgnoringCollisionFilterMock::default()));
        let mut world = NphysicsWorld::with_timestep(
            DEFAULT_TIMESTEP,
            box rotation_translator,
            box force_applier,
            collision_filter.clone(),
        );

        let handle = world.add_body(body.clone());

        collision_filter
            .write()
            .expect("RwLock was poisoned")
            .expect_is_handle_ignored_and_return(VecDeque::from(vec![(handle.into(), false)]));

        const BODY_HANDLE_ERROR: &str = "Invalid object handle";
        world.apply_force(handle, force).expect(BODY_HANDLE_ERROR);

        world.step();
        world.step();

        let actual_body = world.body(handle).expect(BODY_HANDLE_ERROR);
        assert_eq!(*expected_body, actual_body);
    }

}
