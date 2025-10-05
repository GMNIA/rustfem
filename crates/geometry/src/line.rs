use crate::{epsilon, Vector2d, Vector3d};

/// Internal trait to abstract over 2D and 3D vector behaviour for lines.
pub trait LineVector: Copy {
    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn scale(&self, factor: f64) -> Self;
    fn dot(&self, other: &Self) -> f64;
    fn norm(&self) -> f64;
    fn normalize(&self) -> Self;
    fn component_min(&self, other: &Self) -> Self;
    fn component_max(&self, other: &Self) -> Self;
    fn is_approx(&self, other: &Self, precision: Option<f64>) -> bool;
}

impl LineVector for Vector2d {
    fn add(&self, other: &Self) -> Self {
        Vector2d(self.0 + other.0)
    }

    fn sub(&self, other: &Self) -> Self {
        Vector2d(self.0 - other.0)
    }

    fn scale(&self, factor: f64) -> Self {
        Vector2d(self.0 * factor)
    }

    fn dot(&self, other: &Self) -> f64 {
        self.0.dot(&other.0)
    }

    fn norm(&self) -> f64 {
        self.0.norm()
    }

    fn normalize(&self) -> Self {
        Vector2d(self.0.normalize())
    }

    fn component_min(&self, other: &Self) -> Self {
        Vector2d(self.0.zip_map(&other.0, |a, b| a.min(*b)))
    }

    fn component_max(&self, other: &Self) -> Self {
        Vector2d(self.0.zip_map(&other.0, |a, b| a.max(*b)))
    }

    fn is_approx(&self, other: &Self, precision: Option<f64>) -> bool {
        self.is_approx(other, precision)
    }
}

impl LineVector for Vector3d {
    fn add(&self, other: &Self) -> Self {
        Vector3d(self.0 + other.0)
    }

    fn sub(&self, other: &Self) -> Self {
        Vector3d(self.0 - other.0)
    }

    fn scale(&self, factor: f64) -> Self {
        Vector3d(self.0 * factor)
    }

    fn dot(&self, other: &Self) -> f64 {
        self.0.dot(&other.0)
    }

    fn norm(&self) -> f64 {
        self.0.norm()
    }

    fn normalize(&self) -> Self {
        Vector3d(self.0.normalize())
    }

    fn component_min(&self, other: &Self) -> Self {
        Vector3d(self.0.zip_map(&other.0, |a, b| a.min(*b)))
    }

    fn component_max(&self, other: &Self) -> Self {
        Vector3d(self.0.zip_map(&other.0, |a, b| a.max(*b)))
    }

    fn is_approx(&self, other: &Self, precision: Option<f64>) -> bool {
        self.is_approx(other, precision)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line<V>
where
    V: LineVector,
{
    start: V,
    end: V,
}

pub type Line2d = Line<Vector2d>;
pub type Line3d = Line<Vector3d>;

impl<V> Line<V>
where
    V: LineVector,
{
    pub fn new(start: V, end: V) -> Self {
        Self { start, end }
    }

    pub fn start(&self) -> V {
        self.start
    }

    pub fn end(&self) -> V {
        self.end
    }

    pub fn direction(&self) -> Option<V> {
        let dir = self.end.sub(&self.start);
        let length = dir.norm();
        if length <= epsilon() {
            None
        } else {
            Some(dir.normalize())
        }
    }

    pub fn length(&self) -> f64 {
        self.end.sub(&self.start).norm()
    }

    pub fn midpoint(&self) -> V {
        self.start.add(&self.end).scale(0.5)
    }

    pub fn point_at(&self, t: f64) -> V {
        let dir = self.end.sub(&self.start);
        self.start.add(&dir.scale(t))
    }

    pub fn closest_point(&self, point: &V) -> V {
        let dir = self.end.sub(&self.start);
        let len_sq = dir.dot(&dir);
        if len_sq <= epsilon() {
            return self.start;
        }

        let to_point = point.sub(&self.start);
        let mut t = dir.dot(&to_point) / len_sq;
        t = t.clamp(0.0, 1.0);
        self.point_at(t)
    }

    pub fn projection(&self, point: &V) -> V {
        self.closest_point(point)
    }

    pub fn distance(&self, point: &V) -> f64 {
        let closest = self.closest_point(point);
        point.sub(&closest).norm()
    }

    pub fn contains(&self, point: &V) -> bool {
        let closest = self.closest_point(point);
        if !closest.is_approx(point, Some(epsilon())) {
            return false;
        }

        let dir = self.end.sub(&self.start);
        let len_sq = dir.dot(&dir);
        if len_sq <= epsilon() {
            return point.is_approx(&self.start, Some(epsilon()));
        }

        let to_point = point.sub(&self.start);
        let t = dir.dot(&to_point) / len_sq;
        t >= -epsilon() && t <= 1.0 + epsilon()
    }

    pub fn bounding_box(&self) -> (V, V) {
        (self.start.component_min(&self.end), self.start.component_max(&self.end))
    }

    pub fn break_at(&self, parameter: f64) -> Vec<Self> {
        if parameter <= 0.0 || parameter >= 1.0 {
            return vec![*self];
        }

        let split = self.point_at(parameter);
        vec![Self::new(self.start, split), Self::new(split, self.end)]
    }

    pub fn break_at_point(&self, point: &V) -> Vec<Self> {
        if !self.contains(point) {
            return vec![*self];
        }

        if point.is_approx(&self.start, Some(epsilon())) || point.is_approx(&self.end, Some(epsilon())) {
            return vec![*self];
        }

        vec![Self::new(self.start, *point), Self::new(*point, self.end)]
    }

    pub fn move_start(&mut self, start: V) {
        self.start = start;
    }

    pub fn move_end(&mut self, end: V) {
        self.end = end;
    }

    pub fn reverse(&mut self) {
        std::mem::swap(&mut self.start, &mut self.end);
    }

    pub fn reversed(&self) -> Self {
        Self::new(self.end, self.start)
    }

    pub fn length_at_point(&self, point: &V) -> f64 {
        let proj = self.closest_point(point);
        proj.sub(&self.start).norm()
    }

    pub fn point_parameter(&self, point: &V) -> Option<f64> {
        if !self.contains(point) {
            return None;
        }

        let dir = self.end.sub(&self.start);
        let len_sq = dir.dot(&dir);
        if len_sq <= epsilon() {
            return Some(0.0);
        }

        let to_point = point.sub(&self.start);
        Some((dir.dot(&to_point) / len_sq).clamp(0.0, 1.0))
    }

    pub fn intersection(&self, other: &Self, treat_as_ray: bool) -> Option<V> {
        let dir1 = self.end.sub(&self.start);
        let dir2 = other.end.sub(&other.start);

        let a = dir1.dot(&dir1);
        let e = dir2.dot(&dir2);
        let b = dir1.dot(&dir2);
        let r = self.start.sub(&other.start);
        let c = dir1.dot(&r);
        let f = dir2.dot(&r);

        let denom = a * e - b * b;
        if denom.abs() <= epsilon() {
            if self.contains(&other.start) {
                return Some(other.start);
            }
            return None;
        }

        let s = (b * f - c * e) / denom;
        let t = (a * f - b * c) / denom;

        if !treat_as_ray {
            if s < -epsilon() || s > 1.0 + epsilon() || t < -epsilon() || t > 1.0 + epsilon() {
                return None;
            }
        } else if s < -epsilon() || t < -epsilon() {
            return None;
        }

        let point_on_self = self.start.add(&dir1.scale(s));
        let point_on_other = other.start.add(&dir2.scale(t));

        if point_on_self.is_approx(&point_on_other, Some(epsilon())) {
            Some(point_on_self)
        } else {
            None
        }
    }

    pub fn ray_intersection(&self, other: &Self) -> Option<V> {
        self.intersection(other, true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{assert_almost_eq, DEFAULT_EPSILON};

    #[test]
    fn line_length_and_direction_2d() {
        let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(3.0, 4.0));
        assert_almost_eq!(line.length(), 5.0);
        let dir = line.direction().expect("direction defined");
        assert_almost_eq!(dir.x(), 0.6);
        assert_almost_eq!(dir.y(), 0.8);
    }

    #[test]
    fn midpoint_and_point_at() {
        let line = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(2.0, 2.0, 2.0));
        let midpoint = line.midpoint();
        assert_almost_eq!(midpoint.x(), 1.0);
        assert_almost_eq!(midpoint.y(), 1.0);
        assert_almost_eq!(midpoint.z(), 1.0);

        let quarter = line.point_at(0.25);
        assert_almost_eq!(quarter.x(), 0.5);
        assert_almost_eq!(quarter.y(), 0.5);
        assert_almost_eq!(quarter.z(), 0.5);
    }

    #[test]
    fn closest_point_and_distance() {
        let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(10.0, 0.0));
        let point = Vector2d::new(5.0, 3.0);
        let closest = line.closest_point(&point);
        assert_almost_eq!(closest.x(), 5.0);
        assert_almost_eq!(closest.y(), 0.0);
        assert_almost_eq!(line.distance(&point), 3.0);
    }

    #[test]
    fn intersection_segments() {
        let a = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 4.0));
        let b = Line2d::new(Vector2d::new(0.0, 4.0), Vector2d::new(4.0, 0.0));
        let intersection = a.intersection(&b, false).expect("segments intersect");
        assert_almost_eq!(intersection.x(), 2.0);
        assert_almost_eq!(intersection.y(), 2.0);
    }

    #[test]
    fn intersection_parallel_returns_none() {
        let a = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 0.0));
        let b = Line2d::new(Vector2d::new(0.0, 1.0), Vector2d::new(4.0, 1.0));
        assert!(a.intersection(&b, false).is_none());
    }

    #[test]
    fn break_at_parameter() {
        let line = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
        let pieces = line.break_at(0.5);
        assert_eq!(pieces.len(), 2);
        assert_almost_eq!(pieces[0].length(), 5.0);
        assert_almost_eq!(pieces[1].length(), 5.0);
    }

    #[test]
    fn contains_and_length_at_point() {
        let line = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
        let point = Vector3d::new(0.0, 0.0, 7.5);
        assert!(line.contains(&point));
        assert_almost_eq!(line.length_at_point(&point), 7.5);
    }

    #[test]
    fn ray_intersection_requires_forward_parameters() {
        let a = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(5.0, 0.0));
        let b = Line2d::new(Vector2d::new(10.0, 0.0), Vector2d::new(15.0, 0.0));
        assert!(a.ray_intersection(&b).is_none());
        let c = Line2d::new(Vector2d::new(5.0, -5.0), Vector2d::new(5.0, 5.0));
        let intersection = a.ray_intersection(&c).expect("rays meet");
        assert_almost_eq!(intersection.x(), 5.0);
        assert_almost_eq!(intersection.y(), 0.0);
    }

    #[test]
    fn contains_handles_degenerate_line() {
        let point = Vector2d::new(1.0, 1.0);
        let line = Line2d::new(point, point);
        assert!(line.contains(&point));
        assert!(line.contains(&Vector2d::new(1.0 + DEFAULT_EPSILON / 2.0, 1.0)));
    }
}
