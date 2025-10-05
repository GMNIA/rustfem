use geometry::{epsilon, set_epsilon, Vector2d, Vector3d, DEFAULT_EPSILON};
use geometry::assert_almost_eq;

#[test]
fn vector2d_normalization_preserves_direction() {
    set_epsilon(DEFAULT_EPSILON);
    let v = Vector2d::new(3.0, 4.0);
    let normalized = v.normalize();

    assert_almost_eq!(normalized.x(), 0.6);
    assert_almost_eq!(normalized.y(), 0.8);
    assert_almost_eq!(normalized.norm(), 1.0);
}

#[test]
fn vector3d_cross_produces_perpendicular_vector() {
    set_epsilon(DEFAULT_EPSILON);
    let a = Vector3d::new(1.0, 0.0, 0.0);
    let b = Vector3d::new(0.0, 1.0, 0.0);
    let cross = a.cross(&b);

    assert_eq!(cross, Vector3d::new(0.0, 0.0, 1.0));
    assert_eq!(cross.dot(&a), 0.0);
    assert_eq!(cross.dot(&b), 0.0);
    assert_eq!(a.dot(&b), 0.0);
    assert_eq!(b.dot(&a), 0.0);
}

#[test]
fn epsilon_can_be_configured() {
    let original = epsilon();
    set_epsilon(1e-9);
    assert_eq!(epsilon(), 1e-9);
    set_epsilon(original);
}

#[test]
fn assert_almost_eq_uses_current_epsilon() {
    set_epsilon(1e-6);
    assert_almost_eq!(1.0 + 5e-7, 1.0);
    set_epsilon(DEFAULT_EPSILON);
}

#[test]
fn vector_is_approx_allows_custom_precision() {
    let a = Vector3d::new(1.0, 2.0, 3.0);
    let b = Vector3d::new(1.0, 2.0, 3.0 + 5e-6);
    assert!(!a.is_approx(&b, None));
    assert!(a.is_approx(&b, Some(1e-5)));
}
