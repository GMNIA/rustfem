mod edge;
mod arc;
mod polygon;
pub mod line;
mod precision;
mod vector;

// Public API: expose 3D concrete type aliases as canonical names; 2D inputs
// to public constructors will still be accepted but the canonical exported
// types are the 3D-specialized aliases below.
pub type Arc = arc::Arc<Vector3d>;
pub type Edge = edge::Edge<Vector3d>;
pub type Polygon = polygon::Polygon<Vector3d>;
pub use precision::{approx_eq, epsilon, set_epsilon, DEFAULT_EPSILON};
pub use vector::{Vector2d, Vector3d};
pub use line::{Axis, LocalAxis, Line3d};
pub use line::Line3d as Line;

#[macro_export]
macro_rules! assert_almost_eq {
    ($left:expr, $right:expr $(,)?) => {{
        let left_val = $left;
        let right_val = $right;
        if !$crate::approx_eq(left_val, right_val) {
            panic!(
                "assertion failed: |{} - {}| = {} >= {}",
                left_val,
                right_val,
                (left_val - right_val).abs(),
                $crate::epsilon()
            );
        }
    }};
    ($left:expr, $right:expr, $($arg:tt)+) => {{
        let left_val = $left;
        let right_val = $right;
        if !$crate::approx_eq(left_val, right_val) {
            panic!($($arg)+);
        }
    }};
}

/// Assert two Vector3d-like values are approximately equal using the crate epsilon.
/// Expands to three `assert_almost_eq!` calls on x/y/z components.
#[macro_export]
macro_rules! assert_vec3_almost_eq {
    ($a:expr, $b:expr $(,)?) => {{
        let a_val = $a;
        let b_val = $b;
        $crate::assert_almost_eq!(a_val.x(), b_val.x());
        $crate::assert_almost_eq!(a_val.y(), b_val.y());
        $crate::assert_almost_eq!(a_val.z(), b_val.z());
    }};
    ($a:expr, $b:expr, $($arg:tt)+) => {{
        let a_val = $a;
        let b_val = $b;
        $crate::assert_almost_eq!(a_val.x(), b_val.x(), $($arg)+);
        $crate::assert_almost_eq!(a_val.y(), b_val.y(), $($arg)+);
        $crate::assert_almost_eq!(a_val.z(), b_val.z(), $($arg)+);
    }};
}
