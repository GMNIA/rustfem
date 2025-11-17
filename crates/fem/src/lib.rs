pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        let epsilon = 1e-12;
        assert!((result as f64 - 4.0).abs() <= epsilon);
    }
}
