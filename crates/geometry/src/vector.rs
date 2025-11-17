use nalgebra::{Vector2, Vector3};

use utils::epsilon;

/// Simple 2D vector type backed by `nalgebra::Vector2<f64>`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2d(pub Vector2<f64>);

impl Vector2d {
    pub fn new(x: f64, y: f64) -> Self {
        Self(Vector2::new(x, y))
    }

    pub fn x(&self) -> f64 { self.0.x }
    pub fn y(&self) -> f64 { self.0.y }
    pub fn z(&self) -> f64 { 0.0 }

    pub fn dot(&self, other: &Self) -> f64 {
        self.0.dot(&other.0)
    }

    pub fn norm(&self) -> f64 {
        self.0.norm()
    }

    pub fn normalize(&self) -> Self {
        Self(self.0.normalize())
    }

    pub fn is_approx(&self, other: &Self, precision: Option<f64>) -> bool {
        let eps = precision.unwrap_or_else(epsilon);
        (self.0 - other.0).norm() <= eps
    }
}

/// Simple 3D vector type backed by `nalgebra::Vector3<f64>`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3d(pub Vector3<f64>);

impl Vector3d {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z))
    }

    pub fn x(&self) -> f64 { self.0.x }
    pub fn y(&self) -> f64 { self.0.y }
    pub fn z(&self) -> f64 { self.0.z }

    pub fn dot(&self, other: &Self) -> f64 {
        self.0.dot(&other.0)
    }

    pub fn norm(&self) -> f64 {
        self.0.norm()
    }

    pub fn normalize(&self) -> Self {
        Self(self.0.normalize())
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self(self.0.cross(&other.0))
    }

    pub fn is_approx(&self, other: &Self, precision: Option<f64>) -> bool {
        let eps = precision.unwrap_or_else(epsilon);
        (self.0 - other.0).norm() <= eps
    }
}

impl From<Vector2d> for Vector3d {
    fn from(v: Vector2d) -> Self {
        Vector3d::new(v.x(), v.y(), 0.0)
    }
}

impl From<[f64; 3]> for Vector3d {
    fn from(values: [f64; 3]) -> Self {
        Vector3d::new(values[0], values[1], values[2])
    }
}

impl From<(f64, f64, f64)> for Vector3d {
    fn from(values: (f64, f64, f64)) -> Self {
        Vector3d::new(values.0, values.1, values.2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use utils::{assert_almost_eq, assert_vec3_almost_eq, DEFAULT_EPSILON};

    #[test]
    fn vector2d_dot_matches_expectation() {
        let a = Vector2d::new(3.0, -4.0);
        let b = Vector2d::new(-2.0, 5.0);
        assert_almost_eq!(a.dot(&b), -26.0);
    }

    #[test]
    fn vector3d_normalize_produces_unit_vector() {
        let v = Vector3d::new(3.0, 4.0, 0.0);
        let normalized = v.normalize();

        // Test with more tolerance to test tolerance handling
        assert_almost_eq!(normalized.norm(), 1.0);
        assert_almost_eq!(normalized.x(), 0.6, 1e-3);
        assert_almost_eq!(normalized.y(), 0.8, 1e-3);
        assert_almost_eq!(normalized.z(), 0.0, 1e-3);
    }

    #[test]
    fn vector3d_cross_returns_perpendicular_vector() {
        let x_axis = Vector3d::new(1.0, 0.0, 0.0);
        let y_axis = Vector3d::new(0.0, 1.0, 0.0);
        let z_axis = x_axis.cross(&y_axis);

        assert_vec3_almost_eq!(z_axis, Vector3d::new(0.0, 0.0, 1.0));
        assert_almost_eq!(z_axis.dot(&x_axis), 0.0);
        assert_almost_eq!(z_axis.dot(&y_axis), 0.0);
    }

    #[test]
    fn vector_is_approx_uses_global_epsilon() {
        let a = Vector2d::new(1.0, 1.0);
        let b = Vector2d::new(1.0 + DEFAULT_EPSILON / 2.0, 1.0 - DEFAULT_EPSILON / 2.0);
        assert!(a.is_approx(&b, None));
    }

    #[test]
    fn vector_is_approx_respects_custom_precision() {
        let a = Vector3d::new(0.0, 0.0, 0.0);
        let b = Vector3d::new(0.0, 0.0, 1e-6);
        assert!(!a.is_approx(&b, None));
        assert!(a.is_approx(&b, Some(1e-5)));
    }
}
