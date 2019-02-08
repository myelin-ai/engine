//! Builder for the [`NphysicsWorld`]
//!
//! [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html

use super::force_applier::SingleTimeForceApplierImpl;
use super::rotation_translator::{NphysicsRotationTranslator, NphysicsRotationTranslatorImpl};
use super::NphysicsWorld;
use super::SingleTimeForceApplier;

/// Builder for the [`NphysicsWorld`].
///
/// [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html
#[derive(Default, Debug)]
pub struct NphysicsWorldBuilder {
    timestep: Option<f64>,
    rotation_translator: Option<Box<dyn NphysicsRotationTranslator>>,
    force_applier: Option<Box<dyn SingleTimeForceApplier>>,
}

impl NphysicsWorldBuilder {
    /// Creates a new builder by calling NphysicsWorldBuilder::default().
    pub fn new() -> NphysicsWorldBuilder {
        NphysicsWorldBuilder::default()
    }

    /// Sets the simulation timestep
    pub fn timestep(&mut self, timestep: f64) {
        self.timestep = Some(timestep)
    }

    /// Sets the rotation translator used between the engine and nphysics.
    pub fn rotation_translator(
        &mut self,
        rotation_translator: Box<dyn NphysicsRotationTranslator>,
    ) {
        self.rotation_translator = Some(rotation_translator)
    }

    /// Sets the force applier.
    pub fn force_applier(&mut self, force_applier: Box<dyn SingleTimeForceApplier>) {
        self.force_applier = Some(force_applier)
    }

    /// Builds the [`NphysicsWorld`].
    ///
    /// [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html
    pub fn build(self) -> NphysicsWorld {
        const DEFAULT_TIMESTEP: f64 = 1.0 / 60.0;
        NphysicsWorld::with_timestep(
            self.timestep.unwrap_or(DEFAULT_TIMESTEP),
            self.rotation_translator
                .unwrap_or(box NphysicsRotationTranslatorImpl::default()),
            self.force_applier
                .unwrap_or(box SingleTimeForceApplierImpl::default()),
        )
    }
}
