pub const DEFAULT_EPSILON: f64 = 1e-12;

#[inline]
pub fn epsilon() -> f64 {
    DEFAULT_EPSILON
}

#[inline]
pub fn approx_eq(left: f64, right: f64) -> bool {
    (left - right).abs() <= DEFAULT_EPSILON
}
