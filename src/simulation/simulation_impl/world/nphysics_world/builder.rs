//! Builder for the [`NphysicsWorld`]
//!
//! [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html

use super::rotation_translator::{NphysicsRotationTranslator, NphysicsRotationTranslatorImpl};
use super::NphysicsWorld;

/// Builder for the [`NphysicsWorld`].
///
/// [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html
#[derive(Default, Debug)]
pub struct NphysicsWorldBuilder {
    timestep: Option<f64>,
    rotation_translator: Option<Box<dyn NphysicsRotationTranslator>>,
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

    /// Builds the [`NphysicsWorld`].
    ///
    /// [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html
    pub fn build(self) -> NphysicsWorld {
        const DEFAULT_TIMESTEP: f64 = 1.0 / 60.0;
        NphysicsWorld::with_timestep(
            self.timestep.unwrap_or(DEFAULT_TIMESTEP),
            self.rotation_translator
                .unwrap_or_else(|| box NphysicsRotationTranslatorImpl::default()),
        )
    }
}
