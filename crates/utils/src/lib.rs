mod precision;

pub use precision::{approx_eq, epsilon, DEFAULT_EPSILON};

/// Boolean macro: are two scalars approximately equal under the current epsilon?
/// Returns a boolean expression; does not panic.
#[macro_export]
macro_rules! approx_eq {
    ($left:expr, $right:expr $(,)?) => {{
        $crate::approx_eq($left, $right, $crate::epsilon())
    }};
    ($left:expr, $right:expr, $epsilon:expr $(,)?) => {{
        $crate::approx_eq($left, $right, $epsilon)
    }};
}

#[macro_export]
macro_rules! assert_almost_eq {
    ($left:expr, $right:expr $(,)?) => {{
        $crate::assert_almost_eq!(@internal $left, $right, $crate::epsilon())
    }};
    ($left:expr, $right:expr, $epsilon:expr $(,)?) => {{
        $crate::assert_almost_eq!(@internal $left, $right, $epsilon)
    }};
    (@internal $left:expr, $right:expr, $epsilon:expr) => {{
        let left_val = $left;
        let right_val = $right;
        let eps = $epsilon;
        let is_zero = |value: f64| value.abs() <= eps;
        let ratio_diff_ok = if left_val == right_val {
            true
        } else if is_zero(left_val) && is_zero(right_val) {
            true
        } else if is_zero(left_val) || is_zero(right_val) {
            (left_val - right_val).abs() <= eps
        } else {
            ((left_val / right_val) - 1.0).abs() <= eps
        };
        if !ratio_diff_ok {
            panic!(
                "assertion failed: ratio between {} and {} deviates by {} >= {}",
                left_val,
                right_val,
                if is_zero(left_val) || is_zero(right_val) {
                    (left_val - right_val).abs()
                } else {
                    ((left_val / right_val) - 1.0).abs()
                },
                eps
            );
        }
    }};
}

/// Assert that two Vector3d-like values are component-wise within the current epsilon.
/// Provides detailed debug output before panicking and supports label and custom message forms.
#[macro_export]
macro_rules! assert_vec3_almost_eq {
    (@check $epsilon:expr, $label:expr, $a:expr, $b:expr) => {{
        let label_val = $label;
        let a_val = $a;
        let b_val = $b;
        let tol = $epsilon;
        let dx = (a_val.x() - b_val.x()).abs();
        let dy = (a_val.y() - b_val.y()).abs();
        let dz = (a_val.z() - b_val.z()).abs();
        if !$crate::approx_eq(a_val.x(), b_val.x(), tol)
            || !$crate::approx_eq(a_val.y(), b_val.y(), tol)
            || !$crate::approx_eq(a_val.z(), b_val.z(), tol)
        {
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
    ($label:expr, $a:expr, $b:expr $(,)?) => {{
        $crate::assert_vec3_almost_eq!(@check $crate::epsilon(), $label, $a, $b);
    }};
    ($label:expr, $a:expr, $b:expr, $epsilon:expr $(,)?) => {{
        $crate::assert_vec3_almost_eq!(@check $epsilon, $label, $a, $b);
    }};
    ($a:expr, $b:expr $(,)?) => {{
        $crate::assert_vec3_almost_eq!(@check $crate::epsilon(), "approx mismatch", $a, $b);
    }};
    ($a:expr, $b:expr, $epsilon:expr $(,)?) => {{
        $crate::assert_vec3_almost_eq!(@check $epsilon, "approx mismatch", $a, $b);
    }};
}
