# Changelog

## 0.1.0
- Initial release

## 0.2.0
- Make mocks for `Simulation` more flexible

## 0.4.1
- Add support for intersection tests with arbitrary polygons. [Issue #49](https://github.com/myelin-ai/engine/issues/49)

## 0.5.0
- Move `own_description` from the signature of `ObjectBehavior::step` to the new function `WorldInteractor::own_object(&self) -> Object<'_>`
    - This also allows access to an object's own ID
- Support object retrieval with an ID via `Simulation::object(&self, id: Id) -> Option<Object<'_>>`

## 0.6.0
- Actually expose the collision checks added in 0.4.1 in `WorldInteractor`
- Improve performance by using references to `Polygon`s everywhere

## 0.6.1
- Add raycast support via `Simulation::objects_in_ray(&self, origin: Point, direction: Vector) -> Snapshot<'_>;`

## 0.7.0
- Re-export `myelin_geometry` as `geometry`

## 0.8.0
- Expose `objects_in_ray` through `WorldIterator::find_objects_in_ray`

## TBD
- All traits except `ObjectBehavior` have been sealed and can therefore no longer be implemented by other crates.
