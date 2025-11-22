use std::ops::{Deref, DerefMut};

use geometry::{Axis, Line3d, Vector3d};
use nalgebra::{Matrix3, Matrix4, Rotation3, Unit};
use utils::epsilon;

use crate::node::{BoundingBox3d, Node};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Fixity {
    translations: [bool; 3],
    rotations: [bool; 3],
}

impl Fixity {
    pub fn fixed() -> Self {
        Self { translations: [true; 3], rotations: [true; 3] }
    }

    pub fn pinned() -> Self {
        Self { translations: [true; 3], rotations: [false; 3] }
    }

    pub fn free() -> Self {
        Self { translations: [false; 3], rotations: [false; 3] }
    }
}

impl Default for Fixity {
    fn default() -> Self { Self::free() }
}

pub trait IntoVec3 {
    fn into_vec3(self) -> Vector3d;
}

impl IntoVec3 for Vector3d {
    fn into_vec3(self) -> Vector3d { self }
}

impl IntoVec3 for [f64; 3] {
    fn into_vec3(self) -> Vector3d { Vector3d::new(self[0], self[1], self[2]) }
}

impl IntoVec3 for (f64, f64, f64) {
    fn into_vec3(self) -> Vector3d { Vector3d::new(self.0, self.1, self.2) }
}

/// Minimal straight element described by two nodes.
#[derive(Debug, Clone)]
pub struct LinearElement {
    name: Option<String>,
    start_node: Node,
    end_node: Node,
    line: Line3d,
}

impl LinearElement {
    pub fn new(start_node: Node, end_node: Node) -> Self {
        let mut element = Self {
            name: None,
            start_node,
            end_node,
            line: Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 0.0)),
        };
        element.refresh_line();
        element
    }

    pub fn set_name<S: Into<String>>(&mut self, name: S) {
        self.name = Some(name.into());
    }

    pub fn clear_name(&mut self) {
        self.name = None;
    }

    pub fn get_name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    pub fn start_node(&self) -> &Node { &self.start_node }
    pub fn end_node(&self) -> &Node { &self.end_node }

    pub fn center(&self) -> Vector3d {
        Vector3d((self.start_node.center().0 + self.end_node.center().0) / 2.0)
    }

    pub fn length(&self) -> f64 {
        (self.end_node.center().0 - self.start_node.center().0).norm()
    }

    fn orientation(&self) -> Rotation3<f64> {
        self.line
            .rotation_matrix()
            .map(Rotation3::from_matrix_unchecked)
            .unwrap_or_else(Rotation3::identity)
    }

    pub fn direction(&self, axis: Axis) -> Vector3d {
        let rotated = self.orientation() * axis.to_vector3d().0;
        Vector3d(rotated)
    }

    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        *self.orientation().matrix()
    }

    pub fn transformation_matrix(&self) -> Matrix4<f64> {
        let mut matrix = Matrix4::identity();
        matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(self.orientation().matrix());
        let center = self.center();
        matrix[(0, 3)] = center.x();
        matrix[(1, 3)] = center.y();
        matrix[(2, 3)] = center.z();
        matrix
    }

    pub fn to_line(&self) -> Line3d { self.line }

    pub fn bounding_box(&self) -> BoundingBox3d {
        let mut bbox = BoundingBox3d::from_point(self.start_node.center());
        bbox.expand_with_point(self.end_node.center());
        bbox
    }

    pub fn rotate<A: IntoVec3>(&mut self, angle: f64, axis: A) {
        let axis_vec = axis.into_vec3().0;
        let unit_axis = match Unit::try_new(axis_vec, epsilon()) {
            Some(axis) => axis,
            None => return,
        };
        let incremental = Rotation3::from_axis_angle(&unit_axis, angle);

        self.line.rotate(angle, [axis_vec.x, axis_vec.y, axis_vec.z]);

        let center = self.center().0;
        for node in [&mut self.start_node, &mut self.end_node] {
            let relative = node.center().0 - center;
            node.set_center(Vector3d(incremental * relative + center));
            node.apply_rotation(&incremental);
        }
        self.refresh_line();
    }

    pub fn r#move<T: IntoVec3>(&mut self, offset: T) {
        let offset_vec = offset.into_vec3();
        self.line.r#move(offset_vec);
        for node in [&mut self.start_node, &mut self.end_node] {
            node.move_global(offset_vec);
        }
        self.refresh_line();
    }

    pub fn move_by(&mut self, local_offset: Vector3d) {
        let rotation = self.orientation();
        let global = rotation * local_offset.0;
        self.r#move(Vector3d(global));
    }

    pub fn move_global<T: IntoVec3>(&mut self, global_offset: T) {
        self.r#move(global_offset);
    }

    pub fn to_global(&self, local: Vector3d) -> Vector3d {
        let rotated = self.orientation() * local.0;
        Vector3d(rotated + self.center().0)
    }

    pub fn to_local(&self, global: Vector3d) -> Vector3d {
        let diff = global.0 - self.center().0;
        Vector3d(self.orientation().inverse() * diff)
    }

    fn refresh_line(&mut self) {
        self.line.set_endpoints(self.start_node.center(), self.end_node.center());
    }
}

impl Deref for LinearElement {
    type Target = Line3d;

    fn deref(&self) -> &Self::Target { &self.line }
}

impl DerefMut for LinearElement {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.line }
}

#[cfg(test)]
mod tests {
    use utils::{assert_almost_eq, assert_vec3_almost_eq};

    use crate::node::Node;

    use super::*;

    #[test]
    fn linear_element_tracks_rotation_and_translation() {
        let mut element = LinearElement::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(2.0, 0.0, 0.0)),
        );
        element.rotate(std::f64::consts::FRAC_PI_2, [0.0, 0.0, 1.0]);
        element.r#move([1.0, 0.0, 0.0]);

        assert_vec3_almost_eq!(element.start_node().center(), Vector3d::new(2.0, -1.0, 0.0));
        assert_vec3_almost_eq!(element.end_node().center(), Vector3d::new(2.0, 1.0, 0.0));
        assert_almost_eq!(element.length(), 2.0);
    }

    #[test]
    fn linear_element_to_local_inverts_to_global() {
        let element = LinearElement::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(2.0, 2.0, 0.0)),
        );
        let local = Vector3d::new(1.0, 0.0, 0.0);
        let global = element.to_global(local);
        let reverted = element.to_local(global);
        assert_vec3_almost_eq!(reverted, Vector3d::new(1.0, 0.0, 0.0));
    }
}
