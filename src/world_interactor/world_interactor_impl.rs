use super::Interactable;
use crate::prelude::*;
use std::time::Duration;

/// Default implementation of [`WorldInteractor`].
///
/// [`WorldInteractor`]: ./../object/trait.WorldInteractor.html
#[derive(Debug)]
pub struct WorldInteractorImpl<'a> {
    interactable: &'a dyn Interactable,
    id: Id,
}

impl<'a> WorldInteractorImpl<'a> {
    /// Creates a new instance of [`WorldInteractorImpl`].
    ///
    /// [`WorldInteractorImpl`]: ./struct.WorldInteractorImpl.html
    pub fn new(interactable: &'a dyn Interactable, id: Id) -> Self {
        Self { interactable, id }
    }
}

impl<'a> WorldInteractor for WorldInteractorImpl<'a> {
    fn find_objects_in_area(&self, area: Aabb) -> Snapshot<'_> {
        self.interactable.objects_in_area(area)
    }

    fn find_objects_in_polygon(&self, area: &Polygon) -> Snapshot<'_> {
        self.interactable.objects_in_polygon(area)
    }

    fn find_objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_> {
        self.interactable.objects_in_ray(origin, direction)
    }

    fn elapsed_time_in_update(&self) -> Duration {
        self.interactable.elapsed_time_in_update()
    }

    fn own_object(&self) -> Object<'_> {
        self.interactable
            .object(self.id)
            .expect("Internal error: Own ID stored in WorldInteractorImpl was invalid")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world_interactor::InteractableMock;
    use mockiato::{partial_eq, partial_eq_owned};
    use myelin_geometry::{Point, PolygonBuilder};

    fn object_description() -> ObjectDescription {
        ObjectBuilder::default()
            .location(10.0, 10.0)
            .mobility(Mobility::Immovable)
            .shape(
                PolygonBuilder::default()
                    .vertex(-5.0, -5.0)
                    .vertex(5.0, 5.0)
                    .vertex(5.0, -5.0)
                    .build()
                    .unwrap(),
            )
            .build()
            .unwrap()
    }

    #[test]
    fn find_objects_in_area_is_propagated() {
        let object_behavior = ObjectBehaviorMock::new();
        let objects = vec![Object {
            id: 125,
            description: object_description(),
            behavior: &object_behavior,
        }];
        let area = Aabb {
            upper_left: Point { x: 10.0, y: 10.0 },
            lower_right: Point { x: 20.0, y: 0.0 },
        };

        let mut interactable = InteractableMock::new();
        interactable
            .expect_objects_in_area(partial_eq(area))
            .returns(objects.clone());
        let world_interactor = WorldInteractorImpl::new(&interactable, 0);

        let objects_in_area = world_interactor.find_objects_in_area(area);
        assert_eq!(1, objects_in_area.len());
        assert_eq!(objects[0].id, objects_in_area[0].id);
        assert_eq!(objects[0].description, objects_in_area[0].description);
    }

    #[test]
    fn find_objects_in_polygon_is_propagated() {
        let object_behavior = ObjectBehaviorMock::new();
        let objects = vec![Object {
            id: 125,
            description: object_description(),
            behavior: &object_behavior,
        }];
        let area = PolygonBuilder::default()
            .vertex(0.0, 0.0)
            .vertex(20.0, 60.0)
            .vertex(150.0, 100.0)
            .vertex(180.0, 0.0)
            .vertex(150.0, -100.0)
            .vertex(20.0, -60.0)
            .build()
            .unwrap();

        let mut interactable = InteractableMock::new();
        interactable
            .expect_objects_in_polygon(partial_eq_owned(area.clone()))
            .returns(objects.clone());
        let world_interactor = WorldInteractorImpl::new(&interactable, 0);

        let objects_in_area = world_interactor.find_objects_in_polygon(&area);
        assert_eq!(1, objects_in_area.len());
        assert_eq!(objects[0].id, objects_in_area[0].id);
        assert_eq!(objects[0].description, objects_in_area[0].description);
    }

    #[test]
    fn find_objects_in_ray_is_propagated() {
        let object_behavior = ObjectBehaviorMock::new();
        let objects = vec![Object {
            id: 125,
            description: object_description(),
            behavior: &object_behavior,
        }];

        let origin = Point { x: 5.0, y: 10.0 };
        let direction = Vector { x: 3.0, y: -5.0 };

        let mut interactable = InteractableMock::new();
        interactable
            .expect_objects_in_ray(partial_eq(origin.clone()), partial_eq(direction.clone()))
            .returns(objects.clone());
        let world_interactor = WorldInteractorImpl::new(&interactable, 0);

        let objects_in_area = world_interactor.find_objects_in_ray(origin, direction);
        assert_eq!(1, objects_in_area.len());
        assert_eq!(objects[0].id, objects_in_area[0].id);
        assert_eq!(objects[0].description, objects_in_area[0].description);
    }

    #[test]
    fn object_is_propagated() {
        let object_behavior = ObjectBehaviorMock::new();
        let expected_object = Object {
            id: 125,
            description: object_description(),
            behavior: &object_behavior,
        };

        let mut interactable = InteractableMock::new();
        interactable
            .expect_object(partial_eq(expected_object.id))
            .returns(Some(expected_object.clone()));
        let world_interactor = WorldInteractorImpl::new(&interactable, expected_object.id);

        let object = world_interactor.own_object();
        assert_eq!(expected_object.id, object.id);
        assert_eq!(expected_object.description, object.description);
    }

    #[test]
    #[should_panic]
    fn object_panics_on_internal_error() {
        let object_behavior = ObjectBehaviorMock::new();
        let expected_object = Object {
            id: 125,
            description: object_description(),
            behavior: &object_behavior,
        };

        let mut interactable = InteractableMock::new();
        interactable
            .expect_object(partial_eq(expected_object.id))
            .returns(None);
        let world_interactor = WorldInteractorImpl::new(&interactable, expected_object.id);

        let _object = world_interactor.own_object();
    }
}
