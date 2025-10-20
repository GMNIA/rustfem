use crate::{epsilon, Vector2d, Vector3d};

/// Canonical coordinate axes for 3D space.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    AxisX,
    AxisY,
    AxisZ,
}

impl Axis {
    /// Return the canonical unit vector for this axis in global coordinates.
    pub fn to_vector3d(&self) -> Vector3d {
        match self {
            Axis::AxisX => Vector3d::new(1.0, 0.0, 0.0),
            Axis::AxisY => Vector3d::new(0.0, 1.0, 0.0),
            Axis::AxisZ => Vector3d::new(0.0, 0.0, 1.0),
        }
    }
}

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
    fn add(&self, other: &Self) -> Self { Vector2d(self.0 + other.0) }
    fn sub(&self, other: &Self) -> Self { Vector2d(self.0 - other.0) }
    fn scale(&self, factor: f64) -> Self { Vector2d(self.0 * factor) }
    fn dot(&self, other: &Self) -> f64 { self.0.dot(&other.0) }
    fn norm(&self) -> f64 { self.0.norm() }
    fn normalize(&self) -> Self { Vector2d(self.0.normalize()) }
    fn component_min(&self, other: &Self) -> Self {
        Vector2d(self.0.zip_map(&other.0, |a, b| a.min(b)))
    }
    fn component_max(&self, other: &Self) -> Self {
        Vector2d(self.0.zip_map(&other.0, |a, b| a.max(b)))
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
        Vector3d(self.0.zip_map(&other.0, |a, b| a.min(b)))
    }

    fn component_max(&self, other: &Self) -> Self {
        Vector3d(self.0.zip_map(&other.0, |a, b| a.max(b)))
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

pub type Line3d = Line<Vector3d>;

impl<V> Line<V>
where
    V: LineVector,
{
    pub fn new<S, E>(start: S, end: E) -> Self
    where
        S: Into<V>,
        E: Into<V>,
    {
        Self { start: start.into(), end: end.into() }
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

    let mut s = (b * f - c * e) / denom;
    let t = (a * f - b * c) / denom;

        if !treat_as_ray {
            if s < -epsilon() || s > 1.0 + epsilon() || t < -epsilon() || t > 1.0 + epsilon() {
                return None;
            }
        } else {
            // Self treated as ray; only require s >= 0 (allow tiny negative due to FP)
            let tol = 1e-9_f64;
            if s < -tol { return None; }
            if s < 0.0 { s = 0.0; }
            // Do not constrain t: treat the other as an infinite line
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

impl Line<Vector3d> {
    /// Builds a rotation matrix whose first column aligns with the line direction and
    /// whose remaining columns provide an orthonormal frame around the tangent.
    pub fn rotation_matrix(&self) -> Option<nalgebra::Matrix3<f64>> {
        use nalgebra::{Matrix3, Vector3};

        let tangent = self.direction()?.0;
        let t_norm = tangent.norm();
        if t_norm <= epsilon() {
            return None;
        }
        let ex = tangent / t_norm;

        // Python logic compatibility:
        // Choose local Z axis so that it lies in the global XZ-plane (y=0) and is orthogonal to ex.
        // A robust way: use ez_raw = ex × (0,1,0). This always has y=0.
        // Then set ez = -normalize(ez_raw) to match the sign used in the provided examples.
        let global_y = Vector3::new(0.0, 1.0, 0.0);
        let mut ez_raw = ex.cross(&global_y); // (ex.z, 0, -ex.x)
        let mut ez_norm = ez_raw.norm();
        if ez_norm <= epsilon() {
            // ex is (anti)parallel to global Y; pick a stable Z axis in XZ-plane.
            // To match the reference behaviour, choose +Z here.
            ez_raw = Vector3::new(0.0, 0.0, 1.0);
            ez_norm = 1.0;
        }
    let ez = ez_raw / ez_norm;

        // e_y to complete a right-handed basis: ex × ey = ez => choose ey = ez × ex
        let ey = ez.cross(&ex);

        Some(Matrix3::from_columns(&[ex, ey, ez]))
    }

// (2D uses the generic Line<V> intersection with relaxed tolerance in ray mode)
    /// Converts a global-space point into the line's local coordinate frame where the
    /// origin lies at the line start and the X axis follows the tangent direction.
    pub fn to_local(&self, point: Vector3d) -> Option<Vector3d> {
        use nalgebra::Matrix3;

        let rotation: Matrix3<f64> = self.rotation_matrix()?;
        let offset = point.0 - self.start.0;
        let local = rotation.transpose() * offset;
        Some(Vector3d::new(local.x, local.y, local.z))
    }

    /// Converts a point expressed in the line's local coordinate frame back to the
    /// global frame.
    pub fn to_global(&self, point: Vector3d) -> Option<Vector3d> {
        let rotation = self.rotation_matrix()?;
        let global = self.start.0 + rotation * point.0;
        Some(Vector3d::new(global.x, global.y, global.z))
    }

    /// Build a LocalAxis object representing this line's local coordinate frame.
    pub fn local_axis(&self) -> Option<LocalAxis> {
        let rotation = self.rotation_matrix()?;
        Some(LocalAxis { origin: self.start, rotation })
    }

    /// Return one of the canonical axes expressed in the line's local frame.
    ///
    /// Example: `line.axis(Axis::AxisX)` returns the unit vector pointing along
    /// the line tangent (the local X axis) expressed in global coordinates.
    pub fn axis(&self, axis: Axis) -> Option<Vector3d> {
        let rotation = self.rotation_matrix()?;
        // rotation columns are the local basis expressed in global coordinates
        match axis {
            Axis::AxisX => Some(Vector3d::new(rotation.column(0)[0], rotation.column(0)[1], rotation.column(0)[2])),
            Axis::AxisY => Some(Vector3d::new(rotation.column(1)[0], rotation.column(1)[1], rotation.column(1)[2])),
            Axis::AxisZ => Some(Vector3d::new(rotation.column(2)[0], rotation.column(2)[1], rotation.column(2)[2])),
        }
    }
}

/// Represents a local coordinate frame (origin + orthonormal axes) in 3D.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalAxis {
    origin: Vector3d,
    // Columns form an orthonormal basis expressed in global coordinates
    rotation: nalgebra::Matrix3<f64>,
}

impl LocalAxis {
    pub fn new(origin: Vector3d, rotation: nalgebra::Matrix3<f64>) -> Self {
        Self { origin, rotation }
    }

    pub fn origin(&self) -> Vector3d { self.origin }

    /// Return global-space unit vector for the requested local axis.
    pub fn direction(&self, axis: Axis) -> Vector3d {
        match axis {
            Axis::AxisX => Vector3d::new(self.rotation.column(0)[0], self.rotation.column(0)[1], self.rotation.column(0)[2]),
            Axis::AxisY => Vector3d::new(self.rotation.column(1)[0], self.rotation.column(1)[1], self.rotation.column(1)[2]),
            Axis::AxisZ => Vector3d::new(self.rotation.column(2)[0], self.rotation.column(2)[1], self.rotation.column(2)[2]),
        }
    }

    /// Transform a global-space point to this local frame.
    pub fn to_local(&self, point: Vector3d) -> Vector3d {
        let offset = point.0 - self.origin.0;
        let local = self.rotation.transpose() * offset;
        Vector3d::new(local.x, local.y, local.z)
    }

    /// Transform a point from this local frame to global-space.
    pub fn to_global(&self, local: Vector3d) -> Vector3d {
        let global = self.origin.0 + self.rotation * local.0;
        Vector3d::new(global.x, global.y, global.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{assert_almost_eq, DEFAULT_EPSILON};

    #[test]
    fn line_length_and_direction_2d() {
        // 3D equivalent of the classic 2D test with z = 0
    let line = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(3.0, 4.0, 0.0));
        assert_almost_eq!(line.length(), 5.0);
        let dir = line.direction().expect("direction defined");
        assert_almost_eq!(dir.x(), 0.6);
        assert_almost_eq!(dir.y(), 0.8);
        assert_almost_eq!(dir.z(), 0.0);
    }

    #[test]
    fn midpoint_and_point_at() {
    let line = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(2.0, 2.0, 2.0));
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
    let line = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(10.0, 0.0, 0.0));
        let point = Vector3d::new(5.0, 3.0, 0.0);
        let closest = line.closest_point(&point);
        assert_almost_eq!(closest.x(), 5.0);
        assert_almost_eq!(closest.y(), 0.0);
        assert_almost_eq!(closest.z(), 0.0);
        assert_almost_eq!(line.distance(&point), 3.0);
    }

    #[test]
    fn intersection_segments() {
    let a = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(4.0, 4.0, 0.0));
    let b = Line::<Vector3d>::new(Vector3d::new(0.0, 4.0, 0.0), Vector3d::new(4.0, 0.0, 0.0));
        let intersection = a.intersection(&b, false).expect("segments intersect");
        assert_almost_eq!(intersection.x(), 2.0);
        assert_almost_eq!(intersection.y(), 2.0);
        assert_almost_eq!(intersection.z(), 0.0);
    }

    #[test]
    fn intersection_parallel_returns_none() {
    let a = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(4.0, 0.0, 0.0));
    let b = Line::<Vector3d>::new(Vector3d::new(0.0, 1.0, 0.0), Vector3d::new(4.0, 1.0, 0.0));
        assert!(a.intersection(&b, false).is_none());
    }

    #[test]
    fn break_at_parameter() {
    let line = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
        let pieces = line.break_at(0.5);
        assert_eq!(pieces.len(), 2);
        assert_almost_eq!(pieces[0].length(), 5.0);
        assert_almost_eq!(pieces[1].length(), 5.0);
    }

    #[test]
    fn contains_and_length_at_point() {
    let line = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
        let point = Vector3d::new(0.0, 0.0, 7.5);
        assert!(line.contains(&point));
        assert_almost_eq!(line.length_at_point(&point), 7.5);
    }

    #[test]
    fn ray_intersection_requires_forward_parameters() {
    let a = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(5.0, 0.0, 0.0));
    let b = Line::<Vector3d>::new(Vector3d::new(10.0, 0.0, 0.0), Vector3d::new(15.0, 0.0, 0.0));
        assert!(a.ray_intersection(&b).is_none());
    let c = Line::<Vector3d>::new(Vector3d::new(5.0, -5.0, 0.0), Vector3d::new(5.0, 5.0, 0.0));
        let intersection = a.ray_intersection(&c).expect("rays meet");
        assert_almost_eq!(intersection.x(), 5.0);
        assert_almost_eq!(intersection.y(), 0.0);
        assert_almost_eq!(intersection.z(), 0.0);
    }

    #[test]
    fn contains_handles_degenerate_line() {
        let point = Vector3d::new(1.0, 1.0, 0.0);
    let line = Line::<Vector3d>::new(point, point);
        assert!(line.contains(&point));
        assert!(line.contains(&Vector3d::new(1.0 + DEFAULT_EPSILON / 2.0, 1.0, 0.0)));
    }

    #[test]
    fn axis_enum_and_line_axis() {
        // Axis enum canonical vectors
        assert_almost_eq!(Axis::AxisX.to_vector3d().x(), 1.0);
        assert_almost_eq!(Axis::AxisX.to_vector3d().y(), 0.0);
        assert_almost_eq!(Axis::AxisZ.to_vector3d().z(), 1.0);

        // Line axis: X should align with tangent
    let line = Line::<Vector3d>::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(1.0, 0.0, 0.0));
        let ax = line.axis(Axis::AxisX).expect("axis available");
        assert_almost_eq!(ax.x(), 1.0);
        assert_almost_eq!(ax.y(), 0.0);
        assert_almost_eq!(ax.z(), 0.0);
    }

    #[test]
    fn local_axis_canonical_lines_match_reference() {
        // lineX = Line([0,0,0], [1,0,0])
    let line_x = Line::<Vector3d>::new(
            Vector3d::new(0.0, 0.0, 0.0),
            Vector3d::new(1.0, 0.0, 0.0),
        );
        let la_x = line_x.local_axis().expect("local axis defined");
        // Directions
        let dx = la_x.direction(Axis::AxisX);
        let dy = la_x.direction(Axis::AxisY);
        let dz = la_x.direction(Axis::AxisZ);
        assert_almost_eq!(dx.x(), 1.0); assert_almost_eq!(dx.y(), 0.0); assert_almost_eq!(dx.z(), 0.0);
        assert_almost_eq!(dy.x(), 0.0); assert_almost_eq!(dy.y(), 1.0); assert_almost_eq!(dy.z(), 0.0);
        assert_almost_eq!(dz.x(), 0.0); assert_almost_eq!(dz.y(), 0.0); assert_almost_eq!(dz.z(), 1.0);
        // Rotation matrix should be identity
        let rot_x = line_x.rotation_matrix().unwrap();
        assert_almost_eq!(rot_x[(0,0)], 1.0); assert_almost_eq!(rot_x[(1,0)], 0.0); assert_almost_eq!(rot_x[(2,0)], 0.0);
        assert_almost_eq!(rot_x[(0,1)], 0.0); assert_almost_eq!(rot_x[(1,1)], 1.0); assert_almost_eq!(rot_x[(2,1)], 0.0);
        assert_almost_eq!(rot_x[(0,2)], 0.0); assert_almost_eq!(rot_x[(1,2)], 0.0); assert_almost_eq!(rot_x[(2,2)], 1.0);

        // lineY = Line([0,0,0], [0,1,0])
    let line_y = Line::<Vector3d>::new(
            Vector3d::new(0.0, 0.0, 0.0),
            Vector3d::new(0.0, 1.0, 0.0),
        );
        let la_y = line_y.local_axis().expect("local axis defined");
        let dx = la_y.direction(Axis::AxisX);
        let dy = la_y.direction(Axis::AxisY);
        let dz = la_y.direction(Axis::AxisZ);
        // Reference directions from the provided Python output
        // X: [0,1,0], Y: [-1,0,0], Z: [0,0,1]
        assert_almost_eq!(dx.x(), 0.0); assert_almost_eq!(dx.y(), 1.0); assert_almost_eq!(dx.z(), 0.0);
        assert_almost_eq!(dy.x(), -1.0); assert_almost_eq!(dy.y(), 0.0); assert_almost_eq!(dy.z(), 0.0);
        assert_almost_eq!(dz.x(), 0.0); assert_almost_eq!(dz.y(), 0.0); assert_almost_eq!(dz.z(), 1.0);
        // Rotation matrix should be:
        //  0 -1  0
        //  1  0  0
        //  0  0  1
        let rot_y = line_y.rotation_matrix().unwrap();
        assert_almost_eq!(rot_y[(0,0)], 0.0);  assert_almost_eq!(rot_y[(1,0)], 1.0);  assert_almost_eq!(rot_y[(2,0)], 0.0);
        assert_almost_eq!(rot_y[(0,1)], -1.0); assert_almost_eq!(rot_y[(1,1)], 0.0);  assert_almost_eq!(rot_y[(2,1)], 0.0);
        assert_almost_eq!(rot_y[(0,2)], 0.0);  assert_almost_eq!(rot_y[(1,2)], 0.0);  assert_almost_eq!(rot_y[(2,2)], 1.0);

        // lineZ = Line([0,0,0], [0,0,1])
    let line_z = Line::<Vector3d>::new(
            Vector3d::new(0.0, 0.0, 0.0),
            Vector3d::new(0.0, 0.0, 1.0),
        );
        let la_z = line_z.local_axis().expect("local axis defined");
        let dx = la_z.direction(Axis::AxisX);
        let dy = la_z.direction(Axis::AxisY);
        let dz = la_z.direction(Axis::AxisZ);
        // Reference directions
        // X: [0,0,1], Y: [0,1,0], Z: [-1,0,0]
        assert_almost_eq!(dx.x(), 0.0); assert_almost_eq!(dx.y(), 0.0); assert_almost_eq!(dx.z(), 1.0);
        assert_almost_eq!(dy.x(), 0.0); assert_almost_eq!(dy.y(), 1.0); assert_almost_eq!(dy.z(), 0.0);
        assert_almost_eq!(dz.x(), -1.0); assert_almost_eq!(dz.y(), 0.0); assert_almost_eq!(dz.z(), 0.0);
        // Rotation matrix should be:
        //  0  0 -1
        //  0  1  0
        //  1  0  0
        let rot_z = line_z.rotation_matrix().unwrap();
        assert_almost_eq!(rot_z[(0,0)], 0.0);  assert_almost_eq!(rot_z[(1,0)], 0.0);  assert_almost_eq!(rot_z[(2,0)], 1.0);
        assert_almost_eq!(rot_z[(0,1)], 0.0);  assert_almost_eq!(rot_z[(1,1)], 1.0);  assert_almost_eq!(rot_z[(2,1)], 0.0);
        assert_almost_eq!(rot_z[(0,2)], -1.0); assert_almost_eq!(rot_z[(1,2)], 0.0);  assert_almost_eq!(rot_z[(2,2)], 0.0);
    }

}
