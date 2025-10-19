mod edge;
mod arc;
pub mod line;
mod precision;
mod vector;

// Public API: expose 3D types as canonical names; 2D inputs auto-promote to z=0
pub use arc::Arc3d as Arc;
pub use edge::Edge3d as Edge;
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
