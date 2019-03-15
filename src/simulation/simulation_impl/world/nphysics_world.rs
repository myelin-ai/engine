//! This module contains a simulated [`World`] and its implementations,
//! in which one can place [`Objects`] in order for them to be influenced
//! by physics.
//!
//! [`World`]: ./trait.World.html
//! [`Objects`]: ../object/struct.Body.html
pub mod builder;
mod physics_world_wrapper;
pub mod rotation_translator;

use self::physics_world_wrapper::PhysicsWorldWrapper;
use self::rotation_translator::*;
use super::{BodyHandle, PhysicalBody, World};

use crate::prelude::*;
use nalgebra::base::{Scalar, Vector2};
use ncollide2d::bounding_volume::AABB as NcollideAabb;
use ncollide2d::math::Point as NcollidePoint;
use ncollide2d::query::Ray;
use ncollide2d::shape::{ConvexPolygon, ShapeHandle};
use ncollide2d::world::{CollisionGroups, CollisionObjectHandle};
use nphysics2d::algebra::ForceType;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::Force as NphysicsForce;
use nphysics2d::math::{Isometry, Point as NPhysicsPoint, Vector as NPhysicsVector};
use nphysics2d::object::{BodyPartHandle, Collider, ColliderDesc, ColliderHandle, RigidBodyDesc};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World as PhysicsWorld;

use std::collections::HashSet;

/// An implementation of [`World`] that uses nphysics
/// in the background.
///
/// [`World`]: ./trait.World.html
#[derive(Debug)]
pub struct NphysicsWorld {
    physics_world: PhysicsWorldWrapper,
    rotation_translator: Box<dyn NphysicsRotationTranslator>,
    passable_bodies: HashSet<BodyHandle>,
}

impl NphysicsWorld {
    /// Instantiates a new empty world
    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    /// use myelin_engine::simulation::world::{
    ///     rotation_translator::NphysicsRotationTranslatorImpl, NphysicsWorld,
    /// };
    /// use std::sync::{Arc, RwLock};
    ///
    /// let rotation_translator = NphysicsRotationTranslatorImpl::default();
    /// let mut world = NphysicsWorld::with_timestep(1.0, Box::new(rotation_translator));
    /// ```
    pub fn with_timestep(
        timestep: f64,
        rotation_translator: Box<dyn NphysicsRotationTranslator>,
    ) -> Self {
        let mut physics_world = PhysicsWorldWrapper(PhysicsWorld::new());

        physics_world.set_timestep(timestep);

        let passable_bodies = HashSet::new();

        Self {
            physics_world,
            rotation_translator,
            passable_bodies,
        }
    }

    fn get_body_from_handle(&self, collider_handle: ColliderHandle) -> Option<PhysicalBody> {
        let collider = self.physics_world.collider(collider_handle)?;

        let shape = self.get_shape(&collider);
        let location = self.get_location(&collider);
        let rotation = self.get_rotation(&collider);
        let mobility = self.get_mobility(&collider);
        let passable = self.passable(&collider);

        Some(PhysicalBody {
            shape,
            location,
            rotation,
            mobility,
            passable,
        })
    }

    fn get_shape(&self, collider: &Collider<f64>) -> Polygon {
        let convex_polygon: &ConvexPolygon<_> = collider
            .shape()
            .as_shape()
            .expect("Failed to cast shape to a ConvexPolygon");
        let vertices: Vec<_> = convex_polygon
            .points()
            .iter()
            .map(|point| Point {
                x: point.x,
                y: point.y,
            })
            .collect();
        Polygon::try_new(vertices).expect("The polygon from nphysics was not valid")
    }

    fn get_mobility(&self, collider: &Collider<f64>) -> Mobility {
        let body_handle = collider.body();
        if body_handle.is_ground() {
            Mobility::Immovable
        } else {
            let rigid_body = self
                .physics_world
                .rigid_body(body_handle)
                .expect("Body handle did not correspond to any rigid body");

            let linear_velocity = rigid_body.velocity().linear;
            let (x, y) = elements(&linear_velocity);
            Mobility::Movable(Vector { x, y })
        }
    }

    fn get_location(&self, collider: &Collider<f64>) -> Point {
        let position = collider.position();
        let (x, y) = elements(&position.translation.vector);
        Point { x, y }
    }

    fn get_rotation(&self, collider: &Collider<f64>) -> Radians {
        let position = collider.position();
        let rotation = position.rotation.angle();

        self.rotation_translator
            .to_radians(rotation)
            .unwrap_or_else(|error| match error {
                NphysicsRotationTranslatorError::InvalidNphysicsValue(value) => panic!(
                    "Rotation of a collider could not be translated into Radians. Invalid value {}",
                    value
                ),
            })
    }

    fn passable(&self, collider: &Collider<f64>) -> bool {
        let body_handle = to_body_handle(collider.handle());
        self.passable_bodies.contains(&body_handle)
    }
}

fn elements<N>(vector: &Vector2<N>) -> (N, N)
where
    N: Scalar,
{
    let mut iter = vector.iter();

    (*iter.next().unwrap(), *iter.next().unwrap())
}

fn to_nphysics_isometry(
    location: Point,
    rotation: Radians,
    rotation_translator: &dyn NphysicsRotationTranslator,
) -> Isometry<f64> {
    Isometry::new(
        NPhysicsVector::new(location.x, location.y),
        rotation_translator.to_nphysics_rotation(rotation),
    )
}

fn translate_shape(shape: &Polygon) -> ShapeHandle<f64> {
    let points: Vec<_> = shape
        .vertices()
        .iter()
        .map(|vertex| NPhysicsPoint::new(vertex.x, vertex.y))
        .collect();

    ShapeHandle::new(ConvexPolygon::try_new(points).expect("Polygon was not convex"))
}

const NON_PASSABLE_BODY_COLLISION_GROUP: usize = 1;
const PASSABLE_BODY_COLLISION_GROUP: usize = 2;
const QUERYING_COLLISION_GROUP: usize = 3;

fn passable_bodies_collision_groups() -> CollisionGroups {
    CollisionGroups::new()
        .with_membership(&[PASSABLE_BODY_COLLISION_GROUP])
        .with_blacklist(&[PASSABLE_BODY_COLLISION_GROUP])
}

fn non_passable_bodies_collision_groups() -> CollisionGroups {
    let mut collision_groups = CollisionGroups::new()
        .with_membership(&[NON_PASSABLE_BODY_COLLISION_GROUP])
        .with_blacklist(&[PASSABLE_BODY_COLLISION_GROUP]);
    collision_groups.enable_self_interaction();
    collision_groups
}

fn querying_collision_groups() -> CollisionGroups {
    // We can't query with the default collision groups membership (all groups)
    // because the passable and non-passable groups blacklist each other.
    // This means that we would find nothing. That's why we're using a separate collision
    // group that whitelists all other groups.
    CollisionGroups::new()
        .with_membership(&[QUERYING_COLLISION_GROUP])
        .with_whitelist(&[
            NON_PASSABLE_BODY_COLLISION_GROUP,
            PASSABLE_BODY_COLLISION_GROUP,
        ])
}

impl World for NphysicsWorld {
    fn step(&mut self) {
        self.physics_world.step();
    }

    fn add_body(&mut self, body: PhysicalBody) -> BodyHandle {
        let shape = translate_shape(&body.shape);
        /// Arbitrary value
        const OBJECT_DENSITY: f64 = 0.1;
        let local_inertia = shape.inertia(OBJECT_DENSITY);
        let local_center_of_mass = shape.center_of_mass();

        let isometry =
            to_nphysics_isometry(body.location, body.rotation, &*self.rotation_translator);
        let material = MaterialHandle::new(BasicMaterial::default());

        /// Arbitrary value
        const COLLIDER_MARGIN: f64 = 0.04;
        /// Arbitrary value
        const MASS_OF_BODY_IN_KG: f64 = 20.0;

        let collision_groups = if body.passable {
            passable_bodies_collision_groups()
        } else {
            non_passable_bodies_collision_groups()
        };

        let handle = match body.mobility {
            Mobility::Immovable => ColliderDesc::new(shape)
                .margin(COLLIDER_MARGIN)
                .position(isometry)
                .material(material)
                .collision_groups(collision_groups)
                .build_with_parent(BodyPartHandle::ground(), &mut self.physics_world)
                .expect("Internal nphysics error: Ground handle was invalid")
                .handle(),
            Mobility::Movable(velocity) => {
                let velocity = nphysics2d::algebra::Velocity2::linear(velocity.x, velocity.y);
                let rigid_body_handle = RigidBodyDesc::new()
                    .position(isometry)
                    .local_inertia(local_inertia)
                    .local_center_of_mass(local_center_of_mass)
                    .mass(MASS_OF_BODY_IN_KG)
                    .velocity(velocity)
                    .build(&mut self.physics_world)
                    .part_handle();
                ColliderDesc::new(shape)
                    .margin(COLLIDER_MARGIN)
                    .position(Isometry::identity())
                    .material(material)
                    .collision_groups(collision_groups)
                    .build_with_parent(rigid_body_handle, &mut self.physics_world)
                    .expect(
                        "Internal nphysics error: Handle of rigid body that was just built is \
                         invalid",
                    )
                    .handle()
            }
        };

        let body_handle = to_body_handle(handle);

        if body.passable {
            self.passable_bodies.insert(body_handle);
        }

        body_handle
    }

    #[must_use]
    fn remove_body(&mut self, body_handle: BodyHandle) -> Option<PhysicalBody> {
        let physical_body = self.body(body_handle)?;
        let collider_handle = to_collider_handle(body_handle);
        let nphysics_body_handle = self.physics_world.collider_body_handle(collider_handle)?;
        if physical_body.passable {
            self.passable_bodies.remove(&body_handle);
        }
        self.physics_world.remove_bodies(&[nphysics_body_handle]);
        Some(physical_body)
    }

    #[must_use]
    fn body(&self, handle: BodyHandle) -> Option<PhysicalBody> {
        let collider_handle = to_collider_handle(handle);
        self.get_body_from_handle(collider_handle)
    }

    #[must_use]
    fn apply_force(&mut self, body_handle: BodyHandle, force: Force) -> Option<()> {
        let collider_handle = to_collider_handle(body_handle);
        let nphysics_body_handle = self.physics_world.collider_body_handle(collider_handle)?;
        let body = self.physics_world.body_mut(nphysics_body_handle)?;

        let nphysics_force =
            NphysicsForce::from_slice(&[force.linear.x, force.linear.y, force.torque.0]);

        /// > `part_id` is the index of the body’s part you want to apply the force to.
        /// > For rigid bodies, this argument is ignored and can take any value.
        ///
        /// [Source](https://www.nphysics.org/rigid_body_simulations_with_contacts/#one-time-force-application-and-impulses)
        const PART_ID: usize = '🤦' as usize;

        /// > `auto_wake_up` controls whether the body affected by the force should be waken-up automatically
        /// > because of this force application. This should typically be set to true whenever
        /// > you are applying a one-time force manually.
        /// > This should likely be set to false if you are applying a continuous
        /// > force from a force generator (so that bodies reaching a dynamic
        /// > equilibrium can be put to sleep again).
        ///
        /// [Source](https://www.nphysics.org/rigid_body_simulations_with_contacts/#one-time-force-application-and-impulses)
        const AUTO_WAKE_UP: bool = true;
        body.apply_force(PART_ID, &nphysics_force, ForceType::Force, AUTO_WAKE_UP);
        Some(())
    }

    fn set_simulated_timestep(&mut self, timestep: f64) {
        self.physics_world.set_timestep(timestep);
    }

    fn is_body_passable(&self, body_handle: BodyHandle) -> bool {
        self.passable_bodies.contains(&body_handle)
    }

    fn bodies_in_area(&self, area: Aabb) -> Vec<BodyHandle> {
        let collision_groups = querying_collision_groups();

        self.physics_world
            .collider_world()
            .interferences_with_aabb(&to_ncollide_aabb(area), &collision_groups)
            .map(|collider| to_body_handle(collider.handle()))
            .collect()
    }

    fn bodies_in_polygon(&self, area: &Polygon) -> Vec<BodyHandle> {
        let area_aabb = area.aabb();

        self.bodies_in_area(area_aabb)
            .into_iter()
            .filter(|&body_handle| {
                let body = self
                    .body(body_handle)
                    .expect("Internal error: Nphysics returned invalid handle");
                let occupied_area = body
                    .shape
                    .translate(body.location)
                    .rotate_around_point(body.rotation, body.location);

                area.intersects(&occupied_area)
            })
            .collect()
    }

    fn bodies_in_ray(&self, origin: Point, direction: Vector) -> Vec<BodyHandle> {
        let collision_groups = querying_collision_groups();

        let origin = to_ncollide_point(origin);
        let direction = to_ncollide_vector(direction);
        let ray = Ray::new(origin, direction);

        self.physics_world
            .collider_world()
            .interferences_with_ray(&ray, &collision_groups)
            .map(|(collider, _)| to_body_handle(collider.handle()))
            .collect()
    }
}

fn to_body_handle(collider_handle: ColliderHandle) -> BodyHandle {
    BodyHandle(collider_handle.uid())
}

fn to_collider_handle(object_handle: BodyHandle) -> ColliderHandle {
    CollisionObjectHandle(object_handle.0)
}

fn to_ncollide_aabb(aabb: Aabb) -> NcollideAabb<f64> {
    NcollideAabb::new(
        to_ncollide_point(aabb.upper_left),
        to_ncollide_point(aabb.lower_right),
    )
}

fn to_ncollide_point(point: Point) -> NcollidePoint<f64> {
    NcollidePoint::from_slice(&[point.x, point.y])
}

fn to_ncollide_vector(vector: Vector) -> Vector2<f64> {
    Vector2::new(vector.x, vector.y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mockiato::partial_eq;
    use mockiato::ExpectedCalls;
    use std::f64::consts::FRAC_PI_2;

    const DEFAULT_TIMESTEP: f64 = 1.0;

    #[test]
    fn returns_none_when_calling_body_with_invalid_handle() {
        let rotation_translator = NphysicsRotationTranslatorMock::new();

        let world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let invalid_handle = BodyHandle(1337);
        let body = world.body(invalid_handle);
        assert!(body.is_none())
    }

    #[test]
    fn returns_none_when_removing_invalid_object() {
        let rotation_translator = NphysicsRotationTranslatorMock::new();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let invalid_handle = BodyHandle(123);
        let physical_body = world.remove_body(invalid_handle);
        assert!(physical_body.is_none())
    }

    #[test]
    fn can_return_rigid_object_with_valid_handle() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let movable_body = movable_body();

        let handle = world.add_body(movable_body);

        world.body(handle);
    }

    #[test]
    fn can_return_grounded_object_with_valid_handle() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let body = immovable_body();

        let handle = world.add_body(body);

        world.body(handle);
    }

    #[test]
    fn removing_object_returns_physical_body() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = movable_body();

        let handle = world.add_body(expected_body.clone());

        let physical_body = world.remove_body(handle).expect("Invalid handle");
        assert_eq!(expected_body, physical_body);
    }

    #[test]
    fn removed_object_cannot_be_accessed() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = movable_body();

        let handle = world.add_body(expected_body.clone());

        let _physical_body = world.remove_body(handle).expect("Invalid handle");
        let removed_body = world.body(handle);
        assert!(removed_body.is_none())
    }

    #[test]
    fn can_return_mixed_objects_with_valid_handles() {
        let mut rotation_translator = NphysicsRotationTranslatorMock::new();
        rotation_translator
            .expect_to_radians(partial_eq(FRAC_PI_2))
            .returns(Ok(Radians::try_new(FRAC_PI_2).unwrap()))
            .times(2);
        rotation_translator
            .expect_to_nphysics_rotation(partial_eq(Radians::try_new(FRAC_PI_2).unwrap()))
            .returns(FRAC_PI_2)
            .times(2);

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let rigid_object = movable_body();
        let grounded_object = immovable_body();

        let rigid_handle = world.add_body(rigid_object);
        let grounded_handle = world.add_body(grounded_object);

        let _rigid_body = world.body(rigid_handle);
        let _grounded_body = world.body(grounded_handle);
    }

    #[test]
    fn returns_correct_rigid_body() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = movable_body();
        let handle = world.add_body(expected_body.clone());

        let actual_body = world.body(handle);

        assert_eq!(Some(expected_body), actual_body)
    }

    #[test]
    fn returns_correct_grounded_body() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = immovable_body();
        let handle = world.add_body(expected_body.clone());

        let actual_body = world.body(handle);

        assert_eq!(Some(expected_body), actual_body)
    }

    #[test]
    fn timestep_is_respected() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);

        let local_object = movable_body();
        let handle = world.add_body(local_object.clone());

        world.step();
        world.step();

        let actual_body = world.body(handle);

        let expected_body = PhysicalBody {
            location: Point { x: 7.0, y: 7.0 },
            ..local_object
        };
        assert_eq!(Some(expected_body), actual_body);
    }

    #[test]
    fn timestep_can_be_changed() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        world.set_simulated_timestep(2.0);

        let local_object = movable_body();
        let handle = world.add_body(local_object.clone());

        world.step();
        world.step();

        let actual_body = world.body(handle);

        let expected_body = PhysicalBody {
            location: Point { x: 9.0, y: 9.0 },
            ..local_object
        };
        assert_eq!(Some(expected_body), actual_body);
    }

    #[test]
    fn step_is_ignored_for_rigid_objects_with_no_movement() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = immovable_body();
        let handle = world.add_body(expected_body.clone());

        world.step();
        world.step();

        let actual_body = world.body(handle);
        assert_eq!(Some(expected_body), actual_body)
    }

    #[test]
    fn step_is_ignored_for_grounded_objects() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let body = immovable_body();
        let still_body = PhysicalBody {
            mobility: Mobility::Movable(Vector { x: 0.0, y: 0.0 }),
            ..body
        };
        let handle = world.add_body(still_body.clone());

        world.step();
        world.step();

        let actual_body = world.body(handle);
        assert_eq!(Some(still_body), actual_body)
    }

    #[test]
    fn applied_force_is_propagated() {
        let rotation_translator = rotation_translator_for_adding_body();

        let expected_force = Force {
            linear: Vector { x: 4.0, y: 10.0 },
            torque: Torque(2.0),
        };

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = movable_body();
        let handle = world.add_body(expected_body.clone());

        world.apply_force(handle, expected_force);
    }

    #[test]
    fn bodies_in_area_returns_body_in_area() {
        let rotation_translator = rotation_translator_for_adding_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = movable_body();
        let handle = world.add_body(expected_body);

        world.step();

        let area = Aabb::try_new((-100.0, -100.0), (100.0, 100.0)).unwrap();
        let bodies = world.bodies_in_area(area);

        assert_eq!(vec![handle], bodies);
    }

    #[test]
    fn bodies_in_area_does_not_return_out_of_range_bodies() {
        let rotation_translator = rotation_translator_for_adding_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let body = movable_body();
        let _handle = world.add_body(body);

        world.step();

        let area = Aabb::try_new((20.0, 20.0), (30.0, 40.0)).unwrap();
        let bodies = world.bodies_in_area(area);

        assert!(bodies.is_empty());
    }

    #[test]
    fn bodies_in_polygon_returns_body_in_area() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let expected_body = movable_body();
        let handle = world.add_body(expected_body);

        world.step();

        let area = PolygonBuilder::default()
            .vertex(-100.0, -100.0)
            .vertex(100.0, -100.0)
            .vertex(0.0, 100.0)
            .build()
            .unwrap();
        let bodies = world.bodies_in_polygon(&area);

        assert_eq!(vec![handle], bodies);
    }

    #[test]
    fn bodies_in_polygon_does_not_return_out_of_range_bodies() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let body = movable_body();
        let _handle = world.add_body(body);

        world.step();

        const TRIANGLE_SIDE_LENGTH: f64 = 23.0;
        let area = PolygonBuilder::default()
            .vertex(TRIANGLE_SIDE_LENGTH, 0.0)
            .vertex(TRIANGLE_SIDE_LENGTH, TRIANGLE_SIDE_LENGTH)
            .vertex(0.0, TRIANGLE_SIDE_LENGTH)
            .build()
            .unwrap();
        let bodies = world.bodies_in_polygon(&area);

        assert!(bodies.is_empty())
    }

    #[test]
    fn bodies_in_ray_returns_bodies_in_area() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let first_body = movable_body();
        let first_handle = world.add_body(first_body);
        let second_body = PhysicalBody {
            location: Point { x: 15.0, y: 10.0 },
            ..movable_body()
        };
        let second_handle = world.add_body(second_body);

        world.step();

        let origin = Point { x: -4.0, y: -4.0 };
        let direction = Vector { x: 10.0, y: 5.0 };
        let bodies = world.bodies_in_ray(origin, direction);

        assert_eq!(vec![second_handle, first_handle], bodies);
    }

    #[test]
    fn bodies_in_ray_returns_passable_bodies_in_area() {
        let mut world = NphysicsWorld::with_timestep(
            DEFAULT_TIMESTEP,
            box NphysicsRotationTranslatorImpl::default(),
        );
        let first_body = passable_body();
        let first_handle = world.add_body(first_body);

        world.step();

        let origin = Point { x: -4.0, y: -4.0 };
        let direction = Vector { x: 10.0, y: 5.0 };
        let bodies = world.bodies_in_ray(origin, direction);

        assert_eq!(vec![first_handle], bodies);
    }

    #[test]
    fn bodies_in_ray_does_not_return_out_of_range_bodies() {
        let rotation_translator = rotation_translator_for_adding_and_reading_body();

        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);
        let first_body = movable_body();
        let _first_handle = world.add_body(first_body);
        let second_body = PhysicalBody {
            location: Point { x: 15.0, y: 10.0 },
            ..movable_body()
        };
        let _second_handle = world.add_body(second_body);

        world.step();

        let origin = Point { x: -4.0, y: -4.0 };
        let direction = Vector { x: -10.0, y: -5.0 };
        let bodies = world.bodies_in_ray(origin, direction);

        assert!(bodies.is_empty());
    }

    #[test]
    fn force_does_nothing_before_step() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);

        let expected_object = physical_body();
        let handle = world.add_body(expected_object.clone());

        let force = Force {
            linear: Vector {
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
    fn body_can_move_out_of_passable_body() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);

        let non_passable_body = physical_body();
        let non_passable_body_handle = world.add_body(non_passable_body.clone());

        let passable_body = passable_body();
        let passable_body_handle = world.add_body(passable_body.clone());

        let force = Force {
            torque: Torque::default(),
            linear: Vector { x: 40.0, y: 40.0 },
        };

        let expected_location_after_applying_force = Point { x: 7.0, y: 7.0 };

        world.apply_force(non_passable_body_handle, force);
        world.step();

        assert_eq!(
            expected_location_after_applying_force,
            world.body(non_passable_body_handle).unwrap().location
        );

        assert_eq!(
            passable_body.location,
            world.body(passable_body_handle).unwrap().location
        );
    }

    #[test]
    fn body_can_move_into_passable_body() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);

        let non_passable_body = PhysicalBody {
            location: Point { x: 15.0, y: 5.0 },
            ..physical_body()
        };
        let non_passable_body_handle = world.add_body(non_passable_body.clone());

        let passable_body = passable_body();
        let passable_body_handle = world.add_body(passable_body.clone());

        let force = Force {
            torque: Torque::default(),
            linear: Vector { x: -40.0, y: 0.0 },
        };

        let expected_location_after_applying_force = Point { x: 13.0, y: 5.0 };

        world.apply_force(non_passable_body_handle, force);
        world.step();

        assert_eq!(
            expected_location_after_applying_force,
            world.body(non_passable_body_handle).unwrap().location
        );

        assert_eq!(
            passable_body.location,
            world.body(passable_body_handle).unwrap().location
        );
    }

    #[test]
    fn passable_body_can_move_into_other_passable_body() {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);

        let moving_body = PhysicalBody {
            location: Point { x: 15.0, y: 5.0 },
            ..passable_body()
        };
        let moving_body_handle = world.add_body(moving_body.clone());

        let passable_body = passable_body();
        let passable_body_handle = world.add_body(passable_body.clone());

        let force = Force {
            torque: Torque::default(),
            linear: Vector { x: -40.0, y: 0.0 },
        };

        let expected_location_after_applying_force = Point { x: 13.0, y: 5.0 };

        world.apply_force(moving_body_handle, force);
        world.step();

        assert_eq!(
            expected_location_after_applying_force,
            world.body(moving_body_handle).unwrap().location
        );

        assert_eq!(
            passable_body.location,
            world.body(passable_body_handle).unwrap().location
        );
    }

    #[test]
    fn zero_force_is_ignored() {
        let body = physical_body();
        let force = Force {
            linear: Vector::default(),
            torque: Torque::default(),
        };
        let expected_body = body.clone();
        test_force(&body, &expected_body, force);
    }

    #[test]
    fn torque_with_no_linear_force_changes_rotation() {
        let body = physical_body();
        let force = Force {
            linear: Vector::default(),
            torque: Torque(100.0),
        };
        let expected_body = PhysicalBody {
            rotation: Radians::try_new(1.2).unwrap(),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn negative_torque_results_in_negative_rotation() {
        let body = physical_body();
        let force = Force {
            linear: Vector::default(),
            torque: Torque(-202.0),
        };
        let expected_body = PhysicalBody {
            rotation: Radians::try_new(3.859_185_307_179_586_7).unwrap(),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn linear_force_with_no_torque_changes_location_and_speed() {
        let body = physical_body();
        let force = Force {
            linear: Vector { x: 100.0, y: 100.0 },
            torque: Torque::default(),
        };
        let expected_body = PhysicalBody {
            location: Point { x: 15.0, y: 15.0 },
            mobility: Mobility::Movable(Vector { x: 5.0, y: 5.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn negative_linear_force_results_in_lower_location() {
        let body = physical_body();
        let force = Force {
            linear: Vector { x: -50.0, y: -50.0 },
            torque: Torque::default(),
        };
        let expected_body = PhysicalBody {
            location: Point { x: 0.0, y: 0.0 },
            mobility: Mobility::Movable(Vector { x: -2.5, y: -2.5 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn location_can_underflow() {
        let body = physical_body();
        let force = Force {
            linear: Vector {
                x: -100.0,
                y: -200.0,
            },
            torque: Torque::default(),
        };
        let expected_body = PhysicalBody {
            location: Point { x: -5.0, y: -15.0 },
            mobility: Mobility::Movable(Vector { x: -5.0, y: -10.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    #[test]
    fn linear_force_and_torque_can_be_combined() {
        let body = physical_body();
        let force = Force {
            linear: Vector { x: 50.0, y: 100.0 },
            torque: Torque(1.5),
        };

        let expected_body = PhysicalBody {
            location: Point { x: 10.0, y: 15.0 },
            rotation: Radians::try_new(0.018_000_000_000_000_002).unwrap(),
            mobility: Mobility::Movable(Vector { x: 2.5, y: 5.0 }),
            ..body
        };
        test_force(&physical_body(), &expected_body, force);
    }

    fn test_force(body: &PhysicalBody, expected_body: &PhysicalBody, force: Force) {
        let rotation_translator = NphysicsRotationTranslatorImpl::default();
        let mut world = NphysicsWorld::with_timestep(DEFAULT_TIMESTEP, box rotation_translator);

        let handle = world.add_body(body.clone());

        const BODY_HANDLE_ERROR: &str = "Invalid object handle";
        world.apply_force(handle, force).expect(BODY_HANDLE_ERROR);

        world.step();
        world.step();

        let actual_body = world.body(handle).expect(BODY_HANDLE_ERROR);
        assert_eq!(*expected_body, actual_body);
    }

    fn rotation_translator_for_adding_body() -> NphysicsRotationTranslatorMock<'static> {
        let mut rotation_translator = NphysicsRotationTranslatorMock::new();
        rotation_translator
            .expect_to_nphysics_rotation(partial_eq(Radians::try_new(FRAC_PI_2).unwrap()))
            .times(ExpectedCalls::any())
            .returns(FRAC_PI_2);
        rotation_translator
    }

    fn rotation_translator_for_adding_and_reading_body() -> NphysicsRotationTranslatorMock<'static>
    {
        let mut rotation_translator = rotation_translator_for_adding_body();
        rotation_translator
            .expect_to_radians(partial_eq(FRAC_PI_2))
            .times(ExpectedCalls::any())
            .returns(Ok(Radians::try_new(FRAC_PI_2).unwrap()));
        rotation_translator
    }

    fn movable_body() -> PhysicalBody {
        PhysicalBody {
            location: Point { x: 5.0, y: 5.0 },
            rotation: Radians::try_new(FRAC_PI_2).unwrap(),
            mobility: Mobility::Movable(Vector { x: 1.0, y: 1.0 }),
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

    fn immovable_body() -> PhysicalBody {
        PhysicalBody {
            shape: PolygonBuilder::default()
                .vertex(-100.0, -100.0)
                .vertex(100.0, -100.0)
                .vertex(100.0, 100.0)
                .vertex(-100.0, 100.0)
                .build()
                .unwrap(),
            mobility: Mobility::Immovable,
            location: Point { x: 300.0, y: 200.0 },
            rotation: Radians::try_new(FRAC_PI_2).unwrap(),
            passable: false,
        }
    }

    fn physical_body() -> PhysicalBody {
        PhysicalBody {
            location: Point { x: 5.0, y: 5.0 },
            rotation: Radians::default(),
            mobility: Mobility::Movable(Vector::default()),
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

    fn passable_body() -> PhysicalBody {
        PhysicalBody {
            location: Point { x: 5.0, y: 5.0 },
            rotation: Radians::default(),
            mobility: Mobility::Movable(Vector::default()),
            shape: PolygonBuilder::default()
                .vertex(-5.0, -5.0)
                .vertex(-5.0, 5.0)
                .vertex(5.0, 5.0)
                .vertex(5.0, -5.0)
                .build()
                .unwrap(),
            passable: true,
        }
    }
}
