//! Objects that can be placed in a world and their components.
//! You can construct a [`ObjectDescription`] by using an [`ObjectBuilder`].
//!
//! [`ObjectBuilder`]: crate::object_builder::ObjectBuilder

use crate::prelude::*;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

mod object_behavior;
pub use self::object_behavior::*;

/// An object that is stored in the simulation
#[derive(Debug, Clone)]
pub struct Object<'a, T> {
    /// The object's unique ID.
    /// Can be stored in order to retrieve this object later on.
    pub id: Id,

    /// Physical description of the object
    pub description: ObjectDescription<T>,

    /// Custom behavior of the object
    pub behavior: &'a dyn ObjectBehavior<T>,
}

/// Possible actions performed by an [`Object`]
/// during a simulation step
///
/// [`Object`]: ./trait.Object.html
#[derive(Debug, Clone)]
pub enum Action<T> {
    /// Apply the specified force to the object
    ApplyForce(Force),
    /// Create a new object at the specified location
    Spawn(ObjectDescription<T>, Box<dyn ObjectBehavior<T>>),
    /// Destroys another object
    Destroy(Id),
    /// Destroy the object
    DestroySelf,
}

/// The behaviourless description of an object that has
/// been placed inside a [`Simulation`].
///
/// [`Simulation`]: ../simulation/trait.Simulation.html
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct ObjectDescription<T> {
    /// The vertices defining the shape of the object
    /// in relation to its [`position`]
    ///
    /// [`position`]: ./struct.ObjectDescription.html#structfield.location
    pub shape: Polygon,

    /// The global location of the center of the object
    pub location: Point,

    /// The object's rotation
    pub rotation: Radians,

    /// The current velocity of the object, defined
    /// as a two dimensional vector relative to the
    /// objects center
    pub mobility: Mobility,

    /// Whether the object is passable or not
    pub passable: bool,

    /// Arbitrary data associated with this object
    pub associated_data: T,
}

/// Required bounds for [`ObjectDescription::associated_data`].
/// This trait is automatically implemented for all types that satisfy the supertraits.
pub trait AssociatedObjectData = Clone + Debug + PartialEq + 'static;

/// An object's mobility and, if present, its
/// current velocity as a vector
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum Mobility {
    /// The object cannot have any velocity as
    /// it cannot be moved. Corresponds to [`ImmovableObject`]
    ///
    /// [`ImmovableObject`]: ./trait.ImmovableObject.html
    Immovable,
    /// A movable object's current velocity. Corresponds to [`MovableObject`]
    ///
    /// [`MovableObject`]: ./trait.MovableObject.html
    Movable(Vector),
}

/// Combination of a linear force and its torque,
/// resulting in a rotated force applied to an object
#[derive(Debug, PartialEq, Clone, Default)]
pub struct Force {
    /// The linear component of the [`Force`]
    pub linear: Vector,
    /// The torque (rotation) component of the [`Force`]
    pub torque: Torque,
}

/// Force of rotation
#[derive(Debug, PartialEq, Clone, Default)]
pub struct Torque(pub f64);
