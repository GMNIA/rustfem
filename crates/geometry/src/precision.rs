use std::sync::atomic::{AtomicU64, Ordering};

#[cfg(test)]
use std::sync::Mutex;

pub const DEFAULT_EPSILON: f64 = 1e-12;
static EPSILON_BITS: AtomicU64 = AtomicU64::new(DEFAULT_EPSILON.to_bits());

pub fn epsilon() -> f64 {
    f64::from_bits(EPSILON_BITS.load(Ordering::Relaxed))
}

pub fn set_epsilon(value: f64) {
    assert!(value >= 0.0, "epsilon must be non-negative");
    EPSILON_BITS.store(value.to_bits(), Ordering::Relaxed);
}

pub fn approx_eq(left: f64, right: f64) -> bool {
    (left - right).abs() <= epsilon()
}


#[cfg(test)]
/// Temporarily overrides the global epsilon within a scoped callback.
pub(crate) fn with_epsilon<T>(value: f64, f: impl FnOnce() -> T) -> T {
    static GUARD: Mutex<()> = Mutex::new(());
    let _guard = GUARD.lock().unwrap();

    let original = epsilon();
    set_epsilon(value);
    let result = f();
    set_epsilon(original);
    result
}
