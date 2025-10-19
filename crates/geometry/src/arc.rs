use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::line::{Line, LineVector};
use crate::{epsilon, Vector2d, Vector3d};

pub trait ArcVector: LineVector {
    fn to_vec3(&self) -> Vector3<f64>;
    fn from_vec3(vec: Vector3<f64>) -> Self;
}

impl ArcVector for Vector2d {
    fn to_vec3(&self) -> Vector3<f64> {
        Vector3::new(self.x(), self.y(), 0.0)
    }

    fn from_vec3(vec: Vector3<f64>) -> Self {
        Vector2d::new(vec.x, vec.y)
    }
}

impl ArcVector for Vector3d {
    fn to_vec3(&self) -> Vector3<f64> {
        self.0
    }

    fn from_vec3(vec: Vector3<f64>) -> Self {
        Vector3d(vec)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arc<V>
where
    V: ArcVector,
{
    center: V,
    start: V,
    end: V,
    normal: Vector3<f64>,
    sweep: f64,
    radius: f64,
}

pub type Arc2d = Arc<Vector2d>;
pub type Arc3d = Arc<Vector3d>;

impl<V> Arc<V>
where
    V: ArcVector,
{
    pub fn new<C, S, E>(center: C, start: S, end: E, clockwise: bool) -> Self
    where
        C: Into<V>,
        S: Into<V>,
        E: Into<V>,
    {
        let center: V = center.into();
        let start: V = start.into();
        let end: V = end.into();

        let start_vec = start.sub(&center);
        let end_vec = end.sub(&center);

        let start_vec3 = start_vec.to_vec3();
        let end_vec3 = end_vec.to_vec3();

        let start_len = start_vec.norm();
        let end_len = end_vec.norm();
        let radius = (start_len + end_len) * 0.5;

        let cross = start_vec3.cross(&end_vec3);
        let cross_norm = cross.norm();
        let mut normal = if cross_norm <= epsilon() {
            Vector3::new(0.0, 0.0, if clockwise { -1.0 } else { 1.0 })
        } else {
            cross / cross_norm
        };

        if cross_norm > epsilon() && clockwise {
            normal = -normal;
        }

        let start_dir = if start_len <= epsilon() {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            start_vec3 / start_len
        };

        let end_dir = if end_len <= epsilon() {
            start_dir
        } else {
            end_vec3 / end_len
        };

        let cross_dir = start_dir.cross(&end_dir);
        let dot = start_dir.dot(&end_dir).clamp(-1.0, 1.0);
        let mut sweep = cross_dir.norm().atan2(dot);
        if cross_dir.dot(&normal) < 0.0 {
            sweep = -sweep;
        }

        if clockwise && sweep > 0.0 {
            sweep -= 2.0 * PI;
        } else if !clockwise && sweep < 0.0 {
            sweep += 2.0 * PI;
        }

        if sweep.abs() <= epsilon() {
            sweep = if clockwise { -PI } else { PI };
        }

        normal = normal.normalize();

        Self { center, start, end, normal, sweep, radius }
    }

    pub fn from_three_points<P1, P2, P3>(p1: P1, p2: P2, p3: P3) -> Option<Self>
    where
        P1: Into<V>,
        P2: Into<V>,
        P3: Into<V>,
    {
        let p1: V = p1.into();
        let p2: V = p2.into();
        let p3: V = p3.into();
        let p1_vec = p1.to_vec3();
        let p2_vec = p2.to_vec3();
        let p3_vec = p3.to_vec3();

        let v1 = p2_vec - p1_vec;
        let v2 = p3_vec - p1_vec;
        if v1.norm() <= epsilon() || v2.norm() <= epsilon() {
            return None;
        }

        let normal = v1.cross(&v2);
        let normal_norm = normal.norm();
        if normal_norm <= epsilon() {
            return None;
        }
        let normal_unit = normal / normal_norm;

        let u = v1.normalize();
        let v = normal_unit.cross(&u);

        let x2 = v1.dot(&u);
        let diff3 = v2;
        let x3 = diff3.dot(&u);
        let y3 = diff3.dot(&v);

        if y3.abs() <= epsilon() {
            return None;
        }

        let cx = x2 * 0.5;
        let cy = (x3 * x3 + y3 * y3 - x2 * x3) / (2.0 * y3);

        let center_vec = p1_vec + u * cx + v * cy;
        let center = V::from_vec3(center_vec);
        let clockwise = cy < 0.0;
        Some(Self::new(center, p1, p3, clockwise))
    }

    pub fn center(&self) -> V {
        self.center
    }

    pub fn start(&self) -> V {
        self.start
    }

    pub fn end(&self) -> V {
        self.end
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    pub fn angle(&self) -> f64 {
        self.sweep
    }

    pub fn length(&self) -> f64 {
        self.radius * self.sweep.abs()
    }

    pub fn point_at(&self, t: f64) -> V {
        self.point_at_angle(self.sweep * t)
    }

    pub fn angle_at(&self, t: f64) -> f64 {
        self.sweep * t
    }

    pub fn point_at_angle(&self, angle: f64) -> V {
        let center_vec = self.center.to_vec3();
        let start_vec = self.start.to_vec3() - center_vec;
        let start_dir = if start_vec.norm() <= epsilon() {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            start_vec.normalize()
        };

        let perp = self.normal.cross(&start_dir);
        let rotated = start_dir * angle.cos() + perp * angle.sin();
        V::from_vec3(center_vec + rotated * self.radius)
    }

    pub fn closest_point(&self, point: &V) -> V {
        let angle = self.clamped_angle_from_point(point);
        self.point_at_angle(angle)
    }

    pub fn distance(&self, point: &V) -> f64 {
        let closest = self.closest_point(point);
        point.sub(&closest).norm()
    }

    pub fn contains(&self, point: &V) -> bool {
        let angle = self.angle_from_point(point);
        if !self.angle_in_range(angle) {
            return false;
        }
        let radial = point.sub(&self.center).norm();
        (radial - self.radius).abs() <= epsilon()
    }

    pub fn break_at(&self, t: f64) -> Vec<Self> {
        if t <= 0.0 || t >= 1.0 {
            return vec![*self];
        }

        let angle = self.sweep * t;
        vec![self.segment(0.0, angle), self.segment(angle, self.sweep)]
    }

    pub fn break_at_angle(&self, angle: f64) -> Vec<Self> {
        if self.sweep.abs() <= epsilon() {
            return vec![*self];
        }
        if !self.angle_in_range(angle) {
            return vec![*self];
        }
        vec![self.segment(0.0, angle), self.segment(angle, self.sweep)]
    }

    pub fn break_at_point(&self, point: &V) -> Vec<Self> {
        if !self.contains(point) {
            return vec![*self];
        }
        let angle = self.angle_from_point(point);
        self.break_at_angle(angle)
    }

    pub fn reverse(&mut self) {
        std::mem::swap(&mut self.start, &mut self.end);
        self.sweep = -self.sweep;
        self.normal = -self.normal;
    }

    pub fn reversed(&self) -> Self {
        let mut clone = *self;
        clone.reverse();
        clone
    }

    pub fn start_tangent(&self) -> V {
        self.tangent_at_angle(0.0)
    }

    pub fn end_tangent(&self) -> V {
        self.tangent_at_angle(self.sweep)
    }

    pub fn angle_from_point(&self, point: &V) -> f64 {
        let center_vec = self.center.to_vec3();
        let start_vec = self.start.to_vec3() - center_vec;
        let start_dir = if start_vec.norm() <= epsilon() {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            start_vec.normalize()
        };

        let perp = self.normal.cross(&start_dir);
        let vec = point.to_vec3() - center_vec;
        if vec.norm() <= epsilon() {
            return 0.0;
        }
        let vec_dir = vec.normalize();
        let x = vec_dir.dot(&start_dir);
        let y = vec_dir.dot(&perp);
        y.atan2(x)
    }

    pub fn length_at_angle(&self, angle: f64) -> f64 {
        self.radius * angle.abs()
    }

    pub fn length_at_point(&self, point: &V) -> f64 {
        let angle = self.clamped_angle_from_point(point);
        self.length_at_angle(angle)
    }

    pub fn intersection_with_line(&self, line: &Line<V>, treat_as_ray: bool) -> Vec<V> {
        let center_vec = self.center.to_vec3();
        let line_start = line.start().to_vec3();
        let line_dir = line.end().to_vec3() - line_start;

        let to_center = line_start - center_vec;
        let a = line_dir.dot(&line_dir);
        let b = 2.0 * line_dir.dot(&to_center);
        let c = to_center.dot(&to_center) - self.radius * self.radius;

        let discriminant = b * b - 4.0 * a * c;
        if discriminant < -epsilon() {
            return Vec::new();
        }

        let mut result = Vec::new();
        let disc_sqrt = discriminant.max(0.0).sqrt();
        if disc_sqrt <= epsilon() {
            // Tangent: single intersection
            let t = -b / (2.0 * a);
            let outside_segment = (!treat_as_ray && (t < -epsilon() || t > 1.0 + epsilon()))
                || (treat_as_ray && t < -epsilon());
            if !outside_segment {
                let point_vec = line_start + line_dir * t;
                let point = V::from_vec3(point_vec);
                if self.contains(&point) || treat_as_ray {
                    result.push(point);
                }
            }
        } else {
            for sign in [-1.0, 1.0] {
                let t = (-b + sign * disc_sqrt) / (2.0 * a);
                if !treat_as_ray && (t < -epsilon() || t > 1.0 + epsilon()) {
                    continue;
                }
                if treat_as_ray && t < -epsilon() {
                    continue;
                }
                let point_vec = line_start + line_dir * t;
                let point = V::from_vec3(point_vec);
                if self.contains(&point) || treat_as_ray {
                    result.push(point);
                }
            }
        }
        result
    }

    pub fn intersection_with_arc(&self, other: &Self) -> Vec<V> {
        // Solve circle-circle intersection in plane assuming shared plane.
        let c1 = self.center.to_vec3();
        let c2 = other.center.to_vec3();
        let diff = c2 - c1;
        let d = diff.norm();
        if d <= epsilon() {
            return Vec::new();
        }

        let r1 = self.radius;
        let r2 = other.radius;
        if d > r1 + r2 + epsilon() || d < (r1 - r2).abs() - epsilon() {
            return Vec::new();
        }

        let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
        let h_sq = r1 * r1 - a * a;
        if h_sq < -epsilon() {
            return Vec::new();
        }
        let h = h_sq.max(0.0).sqrt();
        let base = c1 + diff * (a / d);

        let diff_norm = diff / d;
        let perp = self.normal.cross(&diff_norm);

        let mut points = Vec::new();
        if h <= epsilon() {
            let point_vec = base;
            let point = V::from_vec3(point_vec);
            if self.contains(&point) && other.contains(&point) {
                points.push(point);
            }
        } else {
            for sign in [-1.0, 1.0] {
                let point_vec = base + perp * (sign * h);
                let point = V::from_vec3(point_vec);
                if self.contains(&point) && other.contains(&point) {
                    points.push(point);
                }
            }
        }
        points
    }

    pub fn linearized(&self, segments: usize) -> Vec<Line<V>> {
        let segments = segments.max(1);
        let mut lines = Vec::with_capacity(segments);
        let mut prev = self.start;
        for i in 1..=segments {
            let t = i as f64 / segments as f64;
            let next = self.point_at(t);
            lines.push(Line::new(prev, next));
            prev = next;
        }
        lines
    }

    fn tangent_at_angle(&self, angle: f64) -> V {
        let center_vec = self.center.to_vec3();
        let start_vec = self.start.to_vec3() - center_vec;
        let start_dir = if start_vec.norm() <= epsilon() {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            start_vec.normalize()
        };
        let perp = self.normal.cross(&start_dir);
        let dir = start_dir * (-angle.sin()) + perp * angle.cos();
        V::from_vec3(dir)
    }

    fn segment(&self, start_angle: f64, end_angle: f64) -> Self {
        let start = self.point_at_angle(start_angle);
        let end = self.point_at_angle(end_angle);
        Self {
            center: self.center,
            start,
            end,
            normal: self.normal,
            sweep: end_angle - start_angle,
            radius: self.radius,
        }
    }

    fn angle_in_range(&self, angle: f64) -> bool {
        if self.sweep >= 0.0 {
            angle >= -epsilon() && angle <= self.sweep + epsilon()
        } else {
            angle <= epsilon() && angle >= self.sweep - epsilon()
        }
    }

    fn clamp_angle(&self, angle: f64) -> f64 {
        if self.sweep >= 0.0 {
            angle.clamp(0.0, self.sweep)
        } else {
            angle.clamp(self.sweep, 0.0)
        }
    }

    fn clamped_angle_from_point(&self, point: &V) -> f64 {
        let angle = self.angle_from_point(point);
        self.clamp_angle(angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_almost_eq;

    #[test]
    fn arc_length_matches_radius_times_angle() {
        let arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
        assert_almost_eq!(arc.radius(), 1.0);
        assert_almost_eq!(arc.angle(), PI / 2.0);
        assert_almost_eq!(arc.length(), PI / 2.0);
    }

    #[test]
    fn arc_point_at_and_contains() {
        let arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
        let mid = arc.point_at(0.5);
        assert!(arc.contains(&mid));
        assert_almost_eq!(mid.x(), (2.0f64).sqrt() / 2.0);
        assert_almost_eq!(mid.y(), (2.0f64).sqrt() / 2.0);
    }

    #[test]
    fn arc_reverse_swaps_start_end() {
        let mut arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
        let reversed = arc.reversed();
        assert_eq!(reversed.start(), arc.end());
        assert_eq!(reversed.end(), arc.start());
        arc.reverse();
        assert_eq!(arc.start(), reversed.start());
    }

    #[test]
    fn arc_break_at_splits_arc() {
        let arc = Arc3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(1.0, 0.0, 0.0), Vector3d::new(0.0, 1.0, 0.0), false);
        let parts = arc.break_at(0.5);
        assert_eq!(parts.len(), 2);
        assert_almost_eq!(parts[0].length(), arc.length() / 2.0);
    }
}
