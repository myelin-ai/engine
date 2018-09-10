use crate::object::{Kind, Location, Object, Polygon, Radians, Velocity, Vertex};

#[derive(Debug, Clone, Default, Eq, PartialEq)]
pub struct ObjectBuilderError {
    pub missing_shape: bool,
    pub missing_velocity: bool,
    pub missing_location: bool,
    pub missing_kind: bool,
}

#[derive(Default, Debug)]
pub struct ObjectBuilder {
    shape: Option<Polygon>,
    velocity: Option<Velocity>,
    location: Option<Location>,
    kind: Option<Kind>,
    orientation: Option<Radians>,
}

impl ObjectBuilder {
    pub fn shape(&mut self, polygon: Polygon) -> &mut Self {
        self.shape = Some(polygon);
        self
    }

    pub fn velocity(&mut self, x: i32, y: i32) -> &mut Self {
        self.velocity = Some(Velocity { x, y });
        self
    }

    pub fn location(&mut self, x: u32, y: u32) -> &mut Self {
        self.location = Some(Location { x, y });
        self
    }

    pub fn kind(&mut self, kind: Kind) -> &mut Self {
        self.kind = Some(kind);
        self
    }

    pub fn orientation(&mut self, orientation: Radians) -> &mut Self {
        self.orientation = Some(orientation);
        self
    }

    pub fn build(&mut self) -> Result<Object, ObjectBuilderError> {
        let error = ObjectBuilderError {
            missing_shape: self.shape.is_none(),
            missing_velocity: self.velocity.is_none(),
            missing_location: self.location.is_none(),
            missing_kind: self.kind.is_none(),
        };

        let object = Object {
            shape: self.shape.take().ok_or_else(|| error.clone())?,
            velocity: self.velocity.take().ok_or_else(|| error.clone())?,
            location: self.location.take().ok_or_else(|| error.clone())?,
            kind: self.kind.take().ok_or(error)?,
            orientation: self
                .orientation
                .take()
                .unwrap_or_else(|| Default::default()),
        };

        Ok(object)
    }
}

#[derive(Default, Debug)]
pub struct PolygonBuilder {
    vertices: Vec<Vertex>,
}

impl PolygonBuilder {
    pub fn vertex(mut self, x: u32, y: u32) -> Self {
        self.vertices.push(Vertex { x, y });
        self
    }

    pub fn build(self) -> Result<Polygon, ()> {
        const MINIMUM_VERTICES_IN_A_POLYGON: usize = 3;

        if self.vertices.len() < MINIMUM_VERTICES_IN_A_POLYGON {
            return Err(());
        }

        Ok(Polygon {
            vertices: self.vertices,
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_polygon_builder_works() {
        let polygon = PolygonBuilder::default()
            .vertex(0, 0)
            .vertex(0, 1)
            .vertex(1, 0)
            .vertex(1, 1)
            .build()
            .unwrap();

        let expected = Polygon {
            vertices: vec![
                Vertex { x: 0, y: 0 },
                Vertex { x: 0, y: 1 },
                Vertex { x: 1, y: 0 },
                Vertex { x: 1, y: 1 },
            ],
        };

        assert_eq!(expected, polygon);
    }

    #[test]
    fn test_polygon_builder_errors_for_no_vertices() {
        assert_eq!(Err(()), PolygonBuilder::default().build());
    }

    #[test]
    fn test_polygon_builder_errors_for_one_vertex() {
        assert_eq!(Err(()), PolygonBuilder::default().vertex(1, 1).build());
    }

    #[test]
    fn test_polygon_builder_panicks_for_two_vertices() {
        assert_eq!(
            Err(()),
            PolygonBuilder::default().vertex(0, 0).vertex(1, 1).build()
        );
    }

    #[test]
    fn test_object_builder_should_error_for_missing_kind() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0, 0)
                    .vertex(0, 1)
                    .vertex(1, 0)
                    .vertex(1, 1)
                    .build()
                    .unwrap(),
            ).velocity(10, 10)
            .location(10, 10)
            .orientation(Radians(0.0))
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_kind: true,
                ..Default::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_error_for_missing_shape() {
        let result = ObjectBuilder::default()
            .velocity(10, 10)
            .location(10, 10)
            .kind(Kind::Organism)
            .orientation(Radians(0.0))
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_shape: true,
                ..Default::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_error_for_missing_velocity() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0, 0)
                    .vertex(0, 1)
                    .vertex(1, 0)
                    .vertex(1, 1)
                    .build()
                    .unwrap(),
            ).location(10, 10)
            .kind(Kind::Organism)
            .orientation(Radians(0.0))
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_velocity: true,
                ..Default::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_error_for_missing_location() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0, 0)
                    .vertex(0, 1)
                    .vertex(1, 0)
                    .vertex(1, 1)
                    .build()
                    .unwrap(),
            ).velocity(10, 10)
            .kind(Kind::Organism)
            .orientation(Radians(0.0))
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_location: true,
                ..Default::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_use_default_orientation() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0, 0)
                    .vertex(0, 1)
                    .vertex(1, 0)
                    .vertex(1, 1)
                    .build()
                    .unwrap(),
            ).velocity(10, 20)
            .location(30, 40)
            .kind(Kind::Organism)
            .build();

        let expected = Object {
            orientation: Radians(0.0),
            shape: Polygon {
                vertices: vec![
                    Vertex { x: 0, y: 0 },
                    Vertex { x: 0, y: 1 },
                    Vertex { x: 1, y: 0 },
                    Vertex { x: 1, y: 1 },
                ],
            },
            velocity: Velocity { x: 10, y: 20 },
            location: Location { x: 30, y: 40 },
            kind: Kind::Organism,
        };

        assert_eq!(Ok(expected), result);
    }

    #[test]
    fn test_object_builder_should_error_with_everything_missing() {
        let result = ObjectBuilder::default().build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_shape: true,
                missing_velocity: true,
                missing_location: true,
                missing_kind: true,
                ..Default::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_build_object() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0, 0)
                    .vertex(0, 1)
                    .vertex(1, 0)
                    .vertex(1, 1)
                    .build()
                    .unwrap(),
            ).velocity(10, 20)
            .location(30, 40)
            .kind(Kind::Organism)
            .orientation(Radians(1.1))
            .build();

        let expected = Object {
            orientation: Radians(1.1),
            shape: Polygon {
                vertices: vec![
                    Vertex { x: 0, y: 0 },
                    Vertex { x: 0, y: 1 },
                    Vertex { x: 1, y: 0 },
                    Vertex { x: 1, y: 1 },
                ],
            },
            velocity: Velocity { x: 10, y: 20 },
            location: Location { x: 30, y: 40 },
            kind: Kind::Organism,
        };

        assert_eq!(Ok(expected), result);
    }
}
