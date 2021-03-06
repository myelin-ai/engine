//! [`ObjectDescription`] factory

use crate::object::*;
use myelin_geometry::*;

/// An error representing the values that have
/// wrongly been ommited when building finished
#[derive(Debug, Clone, Default, Eq, PartialEq)]
pub struct ObjectBuilderError {
    /// Flag signaling that `.shape(...)` was never called
    pub missing_shape: bool,
    /// Flag signaling that `.location(...)` was never called
    pub missing_location: bool,
    /// Flag signaling that `.mobility(...)` was never called
    pub missing_mobility: bool,
    /// Flag signaling that `.associated_data(...)` was never called
    pub missing_associated_data: bool,
}

/// [`ObjectDescription`] factory, which can be used in order to configure
/// the properties of a new object.
/// Methods can be chained on it in order to configure it.
///
/// If [`associated_data`] is never called and `T` implements [`Default`], then
/// the default value is used. Otherwise [`build`] will return an error.
///
/// [`build`]: ./struct.ObjectBuilder.html#method.build
/// [`associated_data`]: ./struct.ObjectBuilder.html#method.associated_data
///
/// # Examples
/// ```
/// use myelin_engine::prelude::*;
/// use std::f64::consts::FRAC_PI_2;
///
/// let object = ObjectBuilder::<()>::default()
///     .shape(
///         PolygonBuilder::default()
///             .vertex(-50.0, -50.0)
///             .vertex(50.0, -50.0)
///             .vertex(50.0, 50.0)
///             .vertex(-50.0, 50.0)
///             .build()
///             .unwrap(),
///     )
///     .location(300.0, 450.0)
///     .rotation(Radians::try_new(FRAC_PI_2).unwrap())
///     .mobility(Mobility::Movable(Vector { x: 3.0, y: 5.0 }))
///     .build()
///     .unwrap();
/// ```
#[derive(Debug)]
pub struct ObjectBuilder<T> {
    shape: Option<Polygon>,
    location: Option<Point>,
    rotation: Option<Radians>,
    mobility: Option<Mobility>,
    passable: bool,
    associated_data: Option<T>,
}

impl<T> Default for ObjectBuilder<T> {
    fn default() -> Self {
        Self {
            shape: None,
            location: None,
            rotation: None,
            mobility: None,
            passable: false,
            associated_data: None,
        }
    }
}

impl<T> ObjectBuilder<T> {
    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    ///
    /// ObjectBuilder::<()>::default().shape(
    ///     PolygonBuilder::default()
    ///         .vertex(-50.0, -50.0)
    ///         .vertex(50.0, -50.0)
    ///         .vertex(50.0, 50.0)
    ///         .vertex(-50.0, 50.0)
    ///         .build()
    ///         .unwrap(),
    /// );
    /// ```
    pub fn shape(&mut self, polygon: Polygon) -> &mut Self {
        self.shape = Some(polygon);
        self
    }

    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    ///
    /// ObjectBuilder::<()>::default().location(3.0, 2.0);
    /// ```
    pub fn location(&mut self, x: f64, y: f64) -> &mut Self {
        self.location = Some(Point { x, y });
        self
    }

    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    ///
    /// ObjectBuilder::<()>::default().mobility(Mobility::Movable(Vector { x: -12.0, y: 4.0 }));
    /// ```
    pub fn mobility(&mut self, mobility: Mobility) -> &mut Self {
        self.mobility = Some(mobility);
        self
    }

    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    ///
    /// ObjectBuilder::<()>::default().rotation(Radians::try_new(4.5).unwrap());
    /// ```
    pub fn rotation(&mut self, rotation: Radians) -> &mut Self {
        self.rotation = Some(rotation);
        self
    }

    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    ///
    /// let builder = ObjectBuilder::<()>::default();
    /// ```
    pub fn passable(&mut self, passable: bool) -> &mut Self {
        self.passable = passable;
        self
    }

    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    ///
    /// let builder = ObjectBuilder::<String>::default().associated_data(String::from("Foo"));
    /// ```
    pub fn associated_data(&mut self, associated_data: T) -> &mut Self {
        self.associated_data = Some(associated_data);
        self
    }

    /// Build the [`ObjectDescription`] with all specified settings
    /// # Errors
    /// If a non-optional member has not specified while building
    /// an error is returned, containing flags specifying which
    /// setting has been omitted
    /// # Examples
    /// ```
    /// use myelin_engine::prelude::*;
    /// use std::f64::consts::FRAC_PI_2;
    ///
    /// let object = ObjectBuilder::<()>::default()
    ///     .shape(
    ///         PolygonBuilder::default()
    ///             .vertex(-50.0, -50.0)
    ///             .vertex(50.0, -50.0)
    ///             .vertex(50.0, 50.0)
    ///             .vertex(-50.0, 50.0)
    ///             .build()
    ///             .unwrap(),
    ///     )
    ///     .location(300.0, 450.0)
    ///     .rotation(Radians::try_new(FRAC_PI_2).unwrap())
    ///     .mobility(Mobility::Movable(Vector { x: 3.0, y: 5.0 }))
    ///     .build()
    ///     .unwrap();
    /// ```
    pub fn build(&mut self) -> Result<ObjectDescription<T>, ObjectBuilderError> {
        let error = ObjectBuilderError {
            missing_shape: self.shape.is_none(),
            missing_location: self.location.is_none(),
            missing_mobility: self.mobility.is_none(),
            missing_associated_data: self.associated_data.is_none(),
        };

        let object = ObjectDescription {
            shape: self.shape.take().ok_or_else(|| error.clone())?,
            rotation: self.rotation.take().unwrap_or_else(Default::default),
            location: self.location.take().ok_or_else(|| error.clone())?,
            mobility: self.mobility.take().ok_or_else(|| error.clone())?,
            passable: self.passable,
            associated_data: self.take_associated_data().ok_or_else(|| error.clone())?,
        };

        Ok(object)
    }
}

trait TakeAssociatedData<T> {
    fn take_associated_data(&mut self) -> Option<T>;
}

impl<T> TakeAssociatedData<T> for ObjectBuilder<T> {
    default fn take_associated_data(&mut self) -> Option<T> {
        self.associated_data.take()
    }
}

impl<T> TakeAssociatedData<T> for ObjectBuilder<T>
where
    T: Default,
{
    default fn take_associated_data(&mut self) -> Option<T> {
        Some(self.associated_data.take().unwrap_or_default())
    }
}

impl<T> From<ObjectDescription<T>> for ObjectBuilder<T> {
    fn from(object_description: ObjectDescription<T>) -> Self {
        let ObjectDescription {
            shape,
            location,
            rotation,
            mobility,
            passable,
            associated_data,
        } = object_description;

        ObjectBuilder {
            shape: Some(shape),
            location: Some(location),
            rotation: Some(rotation),
            mobility: Some(mobility),
            passable,
            associated_data: Some(associated_data),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_object_builder_should_error_for_missing_shape() {
        let result = ObjectBuilder::<()>::default()
            .location(10.0, 10.0)
            .rotation(Radians::try_new(0.0).unwrap())
            .mobility(Mobility::Immovable)
            .associated_data(())
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_shape: true,
                ..ObjectBuilderError::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_error_for_missing_location() {
        let result = ObjectBuilder::<()>::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .rotation(Radians::try_new(0.0).unwrap())
            .mobility(Mobility::Immovable)
            .associated_data(())
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_location: true,
                ..ObjectBuilderError::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_error_for_missing_associated_data() {
        #[derive(Debug, PartialEq)]
        struct AssociatedData;

        let result = ObjectBuilder::<AssociatedData>::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .location(30.0, 40.0)
            .rotation(Radians::try_new(0.0).unwrap())
            .mobility(Mobility::Immovable)
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_associated_data: true,
                ..ObjectBuilderError::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_uses_default_value_for_associated_data() {
        let result = ObjectBuilder::<String>::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .location(30.0, 40.0)
            .rotation(Radians::try_new(0.0).unwrap())
            .mobility(Mobility::Immovable)
            .build();

        let expected = ObjectDescription {
            shape: PolygonBuilder::default()
                .vertex(0.0, 0.0)
                .vertex(0.0, 1.0)
                .vertex(1.0, 0.0)
                .vertex(1.0, 1.0)
                .build()
                .unwrap(),
            location: Point { x: 30.0, y: 40.0 },
            rotation: Radians::try_new(0.0).unwrap(),

            mobility: Mobility::Immovable,
            passable: false,
            associated_data: String::default(),
        };

        assert_eq!(Ok(expected), result);
    }

    #[test]
    fn test_object_builder_should_error_for_missing_mobility() {
        let result = ObjectBuilder::<()>::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .rotation(Radians::try_new(0.0).unwrap())
            .location(30.0, 40.0)
            .associated_data(())
            .build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_mobility: true,
                ..ObjectBuilderError::default()
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_use_default_rotation() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .location(30.0, 40.0)
            .mobility(Mobility::Immovable)
            .associated_data(())
            .build();

        let expected = ObjectDescription {
            shape: PolygonBuilder::default()
                .vertex(0.0, 0.0)
                .vertex(0.0, 1.0)
                .vertex(1.0, 0.0)
                .vertex(1.0, 1.0)
                .build()
                .unwrap(),
            location: Point { x: 30.0, y: 40.0 },
            rotation: Radians::try_new(0.0).unwrap(),

            mobility: Mobility::Immovable,
            passable: false,
            associated_data: (),
        };

        assert_eq!(Ok(expected), result);
    }

    #[test]
    fn test_object_builder_uses_passable() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .rotation(Radians::try_new(0.0).unwrap())
            .location(30.0, 40.0)
            .mobility(Mobility::Immovable)
            .passable(true)
            .associated_data(())
            .build();

        let expected = ObjectDescription {
            shape: Polygon::try_new(vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 0.0, y: 1.0 },
                Point { x: 1.0, y: 0.0 },
                Point { x: 1.0, y: 1.0 },
            ])
            .unwrap(),
            location: Point { x: 30.0, y: 40.0 },
            rotation: Radians::try_new(0.0).unwrap(),

            mobility: Mobility::Immovable,
            passable: true,
            associated_data: (),
        };

        assert_eq!(Ok(expected), result);
    }

    #[test]
    fn test_object_builder_uses_associated_data() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .rotation(Radians::try_new(0.0).unwrap())
            .location(30.0, 40.0)
            .mobility(Mobility::Immovable)
            .associated_data((10, "foo"))
            .build();

        let expected = ObjectDescription {
            shape: Polygon::try_new(vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 0.0, y: 1.0 },
                Point { x: 1.0, y: 0.0 },
                Point { x: 1.0, y: 1.0 },
            ])
            .unwrap(),
            location: Point { x: 30.0, y: 40.0 },
            rotation: Radians::try_new(0.0).unwrap(),

            mobility: Mobility::Immovable,
            passable: false,
            associated_data: (10, "foo"),
        };

        assert_eq!(Ok(expected), result);
    }

    #[test]
    fn test_object_builder_should_error_with_everything_missing() {
        let result = ObjectBuilder::<()>::default().build();

        assert_eq!(
            Err(ObjectBuilderError {
                missing_shape: true,
                missing_location: true,
                missing_mobility: true,
                missing_associated_data: true,
            }),
            result
        );
    }

    #[test]
    fn test_object_builder_should_build_object() {
        let result = ObjectBuilder::default()
            .shape(
                PolygonBuilder::default()
                    .vertex(0.0, 0.0)
                    .vertex(0.0, 1.0)
                    .vertex(1.0, 0.0)
                    .vertex(1.0, 1.0)
                    .build()
                    .unwrap(),
            )
            .mobility(Mobility::Movable(Vector { x: -12.0, y: 5.0 }))
            .location(30.0, 40.0)
            .rotation(Radians::try_new(1.1).unwrap())
            .build();

        let expected = ObjectDescription {
            location: Point { x: 30.0, y: 40.0 },
            rotation: Radians::try_new(1.1).unwrap(),
            mobility: Mobility::Movable(Vector { x: -12.0, y: 5.0 }),

            shape: Polygon::try_new(vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 0.0, y: 1.0 },
                Point { x: 1.0, y: 0.0 },
                Point { x: 1.0, y: 1.0 },
            ])
            .unwrap(),
            passable: false,
            associated_data: (),
        };

        assert_eq!(Ok(expected), result);
    }

    #[test]
    fn can_create_object_builder_from_object_description() {
        let object_description = ObjectDescription {
            location: Point { x: 30.0, y: 40.0 },
            rotation: Radians::try_new(1.1).unwrap(),
            mobility: Mobility::Movable(Vector { x: -12.0, y: 5.0 }),

            shape: Polygon::try_new(vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 0.0, y: 1.0 },
                Point { x: 1.0, y: 0.0 },
                Point { x: 1.0, y: 1.0 },
            ])
            .unwrap(),
            passable: true,
            associated_data: (),
        };

        assert_eq!(
            object_description,
            ObjectBuilder::from(object_description.clone())
                .build()
                .unwrap()
        );
    }
}
