# Changelog

## 1.0

- Initial release

## 2.0

- Sanitize Aabbs by replacing `fn new(...) -> Self` with `fn try_new(...) -> Result<Self>`

## 0.2.1

- Add support for intersection tests between `Polygon`s.
- Unify the `intersects` methods of `Polygon` and `Aabb` under the `Intersects` trait
- Add some useful functions to `Vector`.
    - Support calculating the `normal` (i.e. perpendicular) vector
    - Support calculating the `magnitude`
    - Support calculating the `unit` vector
