mod edge;
mod arc;
mod polygon;
pub mod line;
mod shape;
mod precision;
mod vector;

// Public API: expose 3D concrete type aliases as canonical names; 2D inputs
// to public constructors will still be accepted but the canonical exported
// types are the 3D-specialized aliases below.
pub type Arc = arc::Arc<Vector3d>;
pub type Edge = edge::Edge<Vector3d>;
pub type Polygon = polygon::Polygon<Vector3d>;
pub use shape::{Disk, Rectangle, Shape, ShapeC, ShapeI, ShapeL, ShapeT};
// Re-export precision helpers at the crate root so macros can reference `$crate::…`
pub use precision::{approx_eq, epsilon, set_epsilon, DEFAULT_EPSILON};
pub use vector::{Vector2d, Vector3d};
pub use line::{Axis, LocalAxis, Line3d};
pub use line::Line3d as Line;

/// Boolean macro: are two scalars approximately equal under the current epsilon?
/// Returns a boolean expression; does not panic.
#[macro_export]
macro_rules! approx_eq {
    ($left:expr, $right:expr $(,)?) => {{
        (($left) - ($right)).abs() <= $crate::epsilon()
    }};
}

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

/// Assert that two Vector3d-like values are component-wise within the current epsilon.
/// Provides detailed debug output before panicking and supports label and custom message forms.
#[macro_export]
macro_rules! assert_vec3_almost_eq {
    // Labeled with custom message
    ($label:expr, $a:expr, $b:expr, $($arg:tt)+) => {{
        let label_val = $label;
        let a_val = $a;
        let b_val = $b;
        let tol = $crate::epsilon();
        let dx = (a_val.x() - b_val.x()).abs();
        let dy = (a_val.y() - b_val.y()).abs();
        let dz = (a_val.z() - b_val.z()).abs();
        if dx > tol || dy > tol || dz > tol {
            eprintln!(
                "[DEBUG] {} mismatch:\n  actual   = ({:.12}, {:.12}, {:.12})\n  expected = ({:.12}, {:.12}, {:.12})\n  diff     = ({:.12}, {:.12}, {:.12})\n  tol      = {:.12}",
                label_val,
                a_val.x(), a_val.y(), a_val.z(),
                b_val.x(), b_val.y(), b_val.z(),
                dx, dy, dz, tol
            );
            panic!($($arg)+);
        }
    }};
    // Labeled default message
    ($label:expr, $a:expr, $b:expr $(,)?) => {{
        let label_val = $label;
        let a_val = $a;
        let b_val = $b;
        let tol = $crate::epsilon();
        let dx = (a_val.x() - b_val.x()).abs();
        let dy = (a_val.y() - b_val.y()).abs();
        let dz = (a_val.z() - b_val.z()).abs();
        if dx > tol || dy > tol || dz > tol {
            eprintln!(
                "[DEBUG] {} mismatch:\n  actual   = ({:.12}, {:.12}, {:.12})\n  expected = ({:.12}, {:.12}, {:.12})\n  diff     = ({:.12}, {:.12}, {:.12})\n  tol      = {:.12}",
                label_val,
                a_val.x(), a_val.y(), a_val.z(),
                b_val.x(), b_val.y(), b_val.z(),
                dx, dy, dz, tol
            );
            panic!(
                "{} mismatch: actual=({:.12}, {:.12}, {:.12}), expected=({:.12}, {:.12}, {:.12}), diff=({:.12}, {:.12}, {:.12}), tol={:.12}",
                label_val,
                a_val.x(), a_val.y(), a_val.z(),
                b_val.x(), b_val.y(), b_val.z(),
                dx, dy, dz, tol
            );
        }
    }};
    // Unlabeled with custom message
    ($a:expr, $b:expr, $($arg:tt)+) => {{
        let a_val = $a;
        let b_val = $b;
        let tol = $crate::epsilon();
        let dx = (a_val.x() - b_val.x()).abs();
        let dy = (a_val.y() - b_val.y()).abs();
        let dz = (a_val.z() - b_val.z()).abs();
        if dx > tol || dy > tol || dz > tol {
            eprintln!(
                "[DEBUG] approx mismatch:\n  actual   = ({:.12}, {:.12}, {:.12})\n  expected = ({:.12}, {:.12}, {:.12})\n  diff     = ({:.12}, {:.12}, {:.12})\n  tol      = {:.12}",
                a_val.x(), a_val.y(), a_val.z(),
                b_val.x(), b_val.y(), b_val.z(),
                dx, dy, dz, tol
            );
            panic!($($arg)+);
        }
    }};
    // Unlabeled default message
    ($a:expr, $b:expr $(,)?) => {{
        let a_val = $a;
        let b_val = $b;
        let tol = $crate::epsilon();
        let dx = (a_val.x() - b_val.x()).abs();
        let dy = (a_val.y() - b_val.y()).abs();
        let dz = (a_val.z() - b_val.z()).abs();
        if dx > tol || dy > tol || dz > tol {
            eprintln!(
                "[DEBUG] approx mismatch:\n  actual   = ({:.12}, {:.12}, {:.12})\n  expected = ({:.12}, {:.12}, {:.12})\n  diff     = ({:.12}, {:.12}, {:.12})\n  tol      = {:.12}",
                a_val.x(), a_val.y(), a_val.z(),
                b_val.x(), b_val.y(), b_val.z(),
                dx, dy, dz, tol
            );
            panic!(
                "approx mismatch: actual=({:.12}, {:.12}, {:.12}), expected=({:.12}, {:.12}, {:.12}), diff=({:.12}, {:.12}, {:.12}), tol={:.12}",
                a_val.x(), a_val.y(), a_val.z(),
                b_val.x(), b_val.y(), b_val.z(),
                dx, dy, dz, tol
            );
        }
    }};
}
