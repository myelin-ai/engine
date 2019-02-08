use super::force_applier::SingleTimeForceApplierImpl;
use super::rotation_translator::{NphysicsRotationTranslator, NphysicsRotationTranslatorImpl};
use super::NphysicsWorld;
use super::SingleTimeForceApplier;

#[derive(Default)]
pub struct NphysicsWorldBuilder {
    timestep: Option<f64>,
    rotation_translator: Option<Box<dyn NphysicsRotationTranslator>>,
    force_applier: Option<Box<dyn SingleTimeForceApplier>>,
}

impl NphysicsWorldBuilder {
    pub fn new() -> NphysicsWorldBuilder {
        NphysicsWorldBuilder::default()
    }

    pub fn timestep(&mut self, timestep: f64) {
        self.timestep = Some(timestep)
    }

    pub fn rotation_translator(
        &mut self,
        rotation_translator: Box<dyn NphysicsRotationTranslator>,
    ) {
        self.rotation_translator = Some(rotation_translator)
    }

    pub fn force_applier(&mut self, force_applier: Box<dyn SingleTimeForceApplier>) {
        self.force_applier = Some(force_applier)
    }

    pub fn build(self) -> NphysicsWorld {
        NphysicsWorld::with_timestep(
            self.timestep.unwrap_or((1 / 60) as f64),
            self.rotation_translator
                .unwrap_or(box NphysicsRotationTranslatorImpl::default()),
            self.force_applier
                .unwrap_or(box SingleTimeForceApplierImpl::default()),
        )
    }
}
