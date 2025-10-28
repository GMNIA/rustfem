use geometry::{Axis, Vector3d};
use nalgebra::{Matrix3, Matrix4, Rotation3, Unit, Vector3};

fn unit_z() -> Unit<Vector3<f64>> {
    Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0))
}

/// Axis-aligned bounding box helper used by structural entities.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox3d {
    min: Vector3d,
    max: Vector3d,
}

impl BoundingBox3d {
    pub fn new(min: Vector3d, max: Vector3d) -> Self {
        let min_vec = Vector3::new(
            min.x().min(max.x()),
            min.y().min(max.y()),
            min.z().min(max.z()),
        );
        let max_vec = Vector3::new(
            min.x().max(max.x()),
            min.y().max(max.y()),
            min.z().max(max.z()),
        );

        Self { min: Vector3d(min_vec), max: Vector3d(max_vec) }
    }

    pub fn from_point(point: Vector3d) -> Self {
        Self { min: point, max: point }
    }

    pub fn expand_with_point(&mut self, point: Vector3d) {
        self.min = Vector3d(Vector3::new(
            self.min.x().min(point.x()),
            self.min.y().min(point.y()),
            self.min.z().min(point.z()),
        ));
        self.max = Vector3d(Vector3::new(
            self.max.x().max(point.x()),
            self.max.y().max(point.y()),
            self.max.z().max(point.z()),
        ));
    }

    pub fn min(&self) -> Vector3d { self.min }
    pub fn max(&self) -> Vector3d { self.max }
}

/// 3D node combining a position and orientation.
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    name: Option<String>,
    center: Vector3d,
    rotation: Rotation3<f64>,
}

impl Node {
    pub fn new(center: Vector3d, name: Option<String>) -> Self {
        Self {
            name,
            center,
            rotation: Rotation3::identity(),
        }
    }

    pub fn name(&self) -> Option<&str> { self.name.as_deref() }

    pub fn center(&self) -> Vector3d { self.center }

    pub fn coord(&self, index: usize) -> f64 {
        match index {
            0 => self.center.x(),
            1 => self.center.y(),
            2 => self.center.z(),
            _ => panic!("coordinate index {} out of range", index),
        }
    }

    pub fn set_coord(&mut self, index: usize, value: f64) {
        let mut vec = self.center.0;
        match index {
            0 => vec.x = value,
            1 => vec.y = value,
            2 => vec.z = value,
            _ => panic!("coordinate index {} out of range", index),
        }
        self.center = Vector3d(vec);
    }

    /// Overwrite the node center. Intended for internal use by higher level objects.
    pub fn set_center(&mut self, center: Vector3d) {
        self.center = center;
    }

    /// Accumulate a rotation around the global Z axis.
    pub fn rotate(&mut self, angle: f64) {
        let delta = Rotation3::from_axis_angle(&unit_z(), angle);
        self.rotation = self.rotation * delta;
    }

    /// Translate the node by a vector expressed in the node local coordinates.
    pub fn move_by(&mut self, local_offset: Vector3d) {
        let rotated = self.rotation.matrix() * local_offset.0;
        self.center = Vector3d(self.center.0 + rotated);
    }

    /// Translate the node by a vector expressed in global coordinates.
    pub fn move_global(&mut self, global_offset: Vector3d) {
        self.center = Vector3d(self.center.0 + global_offset.0);
    }

    pub fn bounding_box(&self) -> BoundingBox3d {
        BoundingBox3d::from_point(self.center)
    }

    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        *self.rotation.matrix()
    }

    pub fn transformation_matrix(&self) -> Matrix4<f64> {
        let mut matrix = Matrix4::identity();
        matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(self.rotation.matrix());
        matrix[(0, 3)] = self.center.x();
        matrix[(1, 3)] = self.center.y();
        matrix[(2, 3)] = self.center.z();
        matrix
    }

    pub fn direction(&self, axis: Axis) -> Vector3d {
        let rotated = self.rotation * axis.to_vector3d().0;
        Vector3d(rotated)
    }

    pub fn to_global(&self, local: Vector3d) -> Vector3d {
        let rotated = self.rotation * local.0;
        Vector3d(rotated + self.center.0)
    }

    pub fn to_local(&self, global: Vector3d) -> Vector3d {
        let diff = global.0 - self.center.0;
        Vector3d(self.rotation.inverse() * diff)
    }

    pub fn center_offset(&self) -> Vector3d {
        Vector3d::new(0.0, 0.0, 0.0)
    }

    pub fn rotation_offset(&self) -> Vector3d {
        Vector3d::new(0.0, 0.0, 0.0)
    }

    pub fn transform(&self, local: Vector3d) -> Vector3d {
        self.to_global(local)
    }
}

#[cfg(test)]
mod tests {
    use geometry::{assert_vec3_almost_eq, Vector3d};

    use super::{Axis, Node};

    #[test]
    fn coordinate_access_and_update() {
        let mut node = Node::new(Vector3d::new(0.0, 0.0, 0.0), Some("origin".into()));
        assert_eq!(node.coord(0), 0.0);
        assert_eq!(node.coord(1), 0.0);
        node.set_coord(0, 1.0);
        node.set_coord(1, -2.0);
        assert_vec3_almost_eq!(node.center(), Vector3d::new(1.0, -2.0, 0.0));
    }

    #[test]
    fn local_global_roundtrip() {
        let mut node = Node::new(Vector3d::new(1.0, 0.0, 0.0), Some("pivot".into()));
        node.rotate(0.25);

        let local = Vector3d::new(0.5, -0.1, 0.0);
        let world = node.to_global(local);
        assert_vec3_almost_eq!(node.to_local(world), local);
    }

    #[test]
    fn rotate_and_move_updates_center() {
        let mut node = Node::new(Vector3d::new(0.0, 0.0, 0.0), Some("moving".into()));
        node.rotate(0.15);
        node.move_by(Vector3d::new(0.28415734530715797, -0.16915514374573093, 0.0));

        let expected = Vector3d::new(0.3062447934024681, -0.12479177080853521, 0.0);
        assert_vec3_almost_eq!(node.center(), expected);
    }

    #[test]
    fn direction_uses_rotation() {
        let mut node = Node::new(Vector3d::new(0.0, 0.0, 0.0), Some("dir".into()));
        node.rotate(0.1);
        let dir = node.direction(Axis::AxisX);
        assert_vec3_almost_eq!(dir, Vector3d::new(0.9950041652780258, 0.09983341664682815, 0.0));
    }
}
