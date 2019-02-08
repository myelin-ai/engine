//! Implementations of [`ForceGenerator`], which provide
//! the interface used to apply a [`Force`] on a body

use super::SingleTimeForceApplier;
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

impl ForceGenerator<f64> for SingleTimeForceApplierImpl {
    fn apply(&mut self, _: &IntegrationParameters<f64>, bodies: &mut BodySet<f64>) -> bool {
        for (body_handle, force) in self.forces_to_apply.drain() {
            if bodies.contains(body_handle) {
                /*
                let mut body = bodies.body_part_mut(body_handle);
                let nphysics_force =
                    NphysicsForce::from_slice(&[force.linear.x, force.linear.y, force.torque.0]);
                body.apply_force(&nphysics_force);
                */
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

impl ForceGenerator<f64> for GenericSingleTimeForceApplierWrapper {
    fn apply(
        &mut self,
        integration_parameters: &IntegrationParameters<f64>,
        body_set: &mut BodySet<f64>,
    ) -> bool {
        self.force_applier.apply(integration_parameters, body_set)
    }
}

#[cfg(test)]
mod tests {
    use self::nphysics_world::rotation_translator::NphysicsRotationTranslatorImpl;
    use self::nphysics_world::{NphysicsWorld, PhysicalBody, World};
    use super::*;
    use crate::prelude::*;
    use crate::simulation::simulation_impl::world::nphysics_world;

    const DEFAULT_TIMESTEP: f64 = 1.0;

    #[test]
    fn can_be_injected() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let force_applier = SingleTimeForceApplierImpl::default();
        let _world = NphysicsWorld::with_timestep(
            DEFAULT_TIMESTEP,
            box rotation_translator,
            box force_applier,
        );
    }

}
