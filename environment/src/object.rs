//! Objects that can be placed in a world and their components.
//! You can construct a [`ObjectDescription`] by using an [`ObjectBuilder`].
//!
//! [`ObjectBuilder`]: crate::object_builder::ObjectBuilder

pub use crate::object_builder::*;
use crate::{Id, Snapshot};
use myelin_geometry::*;
use std::fmt::Debug;

/// Behavior of an object
pub trait ObjectBehavior: Debug + ObjectBehaviorClone {
    /// Returns all actions performed by the object
    /// in the current simulation tick
    fn step(
        &mut self,
        own_description: &ObjectDescription,
        environment: &dyn ObjectEnvironment,
    ) -> Option<Action>;
}

/// Provides information to an [`ObjectBehavior`] about
/// the world it is placed in.
///
/// [`ObjectBehavior`]: ./trait.ObjectBehavior.html
pub trait ObjectEnvironment: Debug {
    /// Scans for objects in the area defined by an [`Aabb`].
    ///
    /// Returns all objects either completely contained or intersecting
    /// with the area.
    ///
    /// [`Aabb`]: ./struct.Aabb.html
    fn find_objects_in_area(&self, area: Aabb) -> Snapshot;
}

/// Possible actions performed by an [`Object`]
/// during a simulation step
///
/// [`Object`]: ./trait.Object.html
#[derive(Debug)]
pub enum Action {
    /// Apply the specified force to the object
    ApplyForce(Force),
    /// Create a new object at the specified location
    Reproduce(ObjectDescription, Box<dyn ObjectBehavior>),
    /// Destroys another object
    Destroy(Id),
    /// Destroy the object
    Die,
}

impl Clone for Action {
    fn clone(&self) -> Action {
        match self {
            Action::ApplyForce(force) => Action::ApplyForce(force.clone()),
            Action::Reproduce(object_description, object_behavior) => {
                Action::Reproduce(object_description.clone(), object_behavior.clone_box())
            }
            Action::Destroy(body_handle) => Action::Destroy(*body_handle),
            Action::Die => Action::Die,
        }
    }
}

/// An axix-aligned bounding box
#[derive(Debug, PartialEq, Clone)]
pub struct Aabb {
    /// The position with the smallest coordinates
    pub mins: Point,
    /// The position with the highest coordinates
    pub maxs: Point,
}

/// A sensor that can be attached to an [`Object`],
/// which will report any collisions to it.
///
/// [`Object`]: ./enum.Object.html
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Sensor {
    /// The shape of the sensor
    pub shape: Polygon,
    /// The sensor's location in relation to its
    /// parent [`Object`]
    ///
    /// [`Object`]: ./enum.Object.html
    pub location: Point,

    /// the sensor's rotation
    pub rotation: Radians,
}

/// The behaviourless description of an object that has
/// been placed inside a [`Simulation`].
///
/// [`Simulation`]: ../simulation/trait.Simulation.html
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct ObjectDescription {
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

    /// The object's kind
    pub kind: Kind,

    /// The object's sensor
    pub sensor: Option<Sensor>,

    /// Whether the object is passable or not
    pub passable: bool,
}

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

/// The part of an object that is responsible for custom
/// behavior and interactions
#[derive(Debug, Eq, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub enum Kind {
    /// An intelligent organism featuring a neural network
    Organism,
    /// A self-spreading plant, ready for consumption
    Plant,
    /// A stationary body of water
    Water,
    /// Impassable terrain
    Terrain,
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

/// Supertrait used to make sure that all implementors
/// of [`ObjectBehavior`] are [`Clone`]. You don't need
/// to care about this type.
///
/// [`ObjectBehavior`]: ./trait.ObjectBehavior.html
/// [`Clone`]: https://doc.rust-lang.org/nightly/std/clone/trait.Clone.html
#[doc(hidden)]
pub trait ObjectBehaviorClone {
    fn clone_box(&self) -> Box<dyn ObjectBehavior>;
}

impl<T> ObjectBehaviorClone for T
where
    T: ObjectBehavior + Clone + 'static,
{
    default fn clone_box(&self) -> Box<dyn ObjectBehavior> {
        box self.clone()
    }
}
