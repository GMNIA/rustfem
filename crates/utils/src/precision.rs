pub const DEFAULT_EPSILON: f64 = 1e-12;

#[inline]
pub fn epsilon() -> f64 {
    DEFAULT_EPSILON
}

#[inline]
pub fn approx_eq(left: f64, right: f64, tol: f64) -> bool {
    if left == right {
        return true;
    }
    let diff = (left - right).abs();
    let magnitude = left.abs().max(right.abs()).max(1.0);
    diff / magnitude <= tol
}
