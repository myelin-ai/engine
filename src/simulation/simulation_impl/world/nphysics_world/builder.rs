//! Builder for the [`NphysicsWorld`]
//!
//! [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html

use super::NphysicsWorld;

/// Builder for the [`NphysicsWorld`].
///
/// [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html
#[derive(Default, Debug)]
pub struct NphysicsWorldBuilder {
    timestep: Option<f64>,
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

    /// Builds the [`NphysicsWorld`].
    ///
    /// [`NphysicsWorld`]: ./../../struct.NphysicsWorld.html
    pub fn build(self) -> NphysicsWorld {
        const DEFAULT_TIMESTEP: f64 = 1.0 / 60.0;
        NphysicsWorld::with_timestep(self.timestep.unwrap_or(DEFAULT_TIMESTEP))
    }
}
