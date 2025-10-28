use geometry::{Axis, Line3d, Vector3d};
use nalgebra::{Matrix3, Matrix4, Rotation3, Unit, Vector3};

use crate::{
    node::{BoundingBox3d, Node},
    section::Section,
};

fn unit_z() -> Unit<Vector3<f64>> {
    Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0))
}

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

/// Beam or cable member connecting two nodes.
#[derive(Debug, Clone)]
pub struct Member {
    name: Option<String>,
    start_node: Node,
    end_node: Node,
    section: Option<Section>,
    section_rotation: f64,
    init_tension: f64,
    is_cable: bool,
    device: Option<String>,
    start_fixity: Fixity,
    end_fixity: Fixity,
    mesh_nodes: Vec<Vector3d>,
    mesh: Vec<Line3d>,
}

impl Member {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        start_node: Node,
        end_node: Node,
        section: Option<Section>,
        section_rotation: Option<f64>,
        init_tension: Option<f64>,
        is_cable: Option<bool>,
        start_fixity: Option<Fixity>,
        end_fixity: Option<Fixity>,
        device: Option<String>,
        name: Option<String>,
    ) -> Self {
        let start_center = start_node.center();
        let end_center = end_node.center();

        Self {
            name,
            start_node,
            end_node,
            section,
            section_rotation: section_rotation.unwrap_or(0.0),
            init_tension: init_tension.unwrap_or(0.0),
            is_cable: is_cable.unwrap_or(false),
            device,
            start_fixity: start_fixity.unwrap_or_else(Fixity::free),
            end_fixity: end_fixity.unwrap_or_else(Fixity::free),
            mesh_nodes: vec![start_center, end_center],
            mesh: Vec::new(),
        }
    }

    fn refresh_endpoints(&mut self) {
        if self.mesh_nodes.is_empty() {
            self.mesh_nodes.push(self.start_node.center());
            self.mesh_nodes.push(self.end_node.center());
            return;
        }
        if let Some(first) = self.mesh_nodes.first_mut() {
            *first = self.start_node.center();
        }
        if let Some(last) = self.mesh_nodes.last_mut() {
            *last = self.end_node.center();
        }
    }

    pub fn name(&self) -> Option<&str> { self.name.as_deref() }
    pub fn start_node(&self) -> &Node { &self.start_node }
    pub fn end_node(&self) -> &Node { &self.end_node }
    pub fn section(&self) -> Option<&Section> { self.section.as_ref() }
    pub fn section_rotation(&self) -> f64 { self.section_rotation }
    pub fn init_tension(&self) -> f64 { self.init_tension }
    pub fn is_cable(&self) -> bool { self.is_cable }
    pub fn device(&self) -> Option<&str> { self.device.as_deref() }
    pub fn start_fixity(&self) -> &Fixity { &self.start_fixity }
    pub fn end_fixity(&self) -> &Fixity { &self.end_fixity }
    pub fn mesh_nodes(&self) -> &[Vector3d] { &self.mesh_nodes }
    pub fn mesh(&self) -> &[Line3d] { &self.mesh }

    pub fn add_mesh_node(&mut self, node: Vector3d) {
        self.mesh_nodes.push(node);
    }

    pub fn add_mesh_segment(&mut self, line: Line3d) {
        self.mesh.push(line);
    }

    pub fn center(&self) -> Vector3d {
        Vector3d((self.start_node.center().0 + self.end_node.center().0) / 2.0)
    }

    pub fn length(&self) -> f64 {
        (self.end_node.center().0 - self.start_node.center().0).norm()
    }

    fn orientation(&self) -> Rotation3<f64> {
        let delta = self.end_node.center().0 - self.start_node.center().0;
        if delta.norm() <= f64::EPSILON {
            return Rotation3::identity();
        }
        let angle = delta.y.atan2(delta.x) + self.section_rotation;
        Rotation3::from_axis_angle(&unit_z(), angle)
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

    pub fn to_line(&self) -> Line3d {
        Line3d::new(self.start_node.center(), self.end_node.center())
    }

    pub fn bounding_box(&self) -> BoundingBox3d {
        let mut bbox = BoundingBox3d::from_point(self.start_node.center());
        bbox.expand_with_point(self.end_node.center());
        bbox
    }

    pub fn rotate(&mut self, angle: f64) {
        let rotation = Rotation3::from_axis_angle(&unit_z(), angle);
        let center = self.center().0;

        for node in [&mut self.start_node, &mut self.end_node] {
            let relative = node.center().0 - center;
            node.set_center(Vector3d(rotation * relative + center));
            node.rotate(angle);
        }
        self.refresh_endpoints();
    }

    pub fn move_by(&mut self, local_offset: Vector3d) {
        let rotation = self.orientation();
        let global = rotation * local_offset.0;
        for node in [&mut self.start_node, &mut self.end_node] {
            node.move_global(Vector3d(global));
        }
        self.refresh_endpoints();
    }

    pub fn move_global(&mut self, global_offset: Vector3d) {
        for node in [&mut self.start_node, &mut self.end_node] {
            node.move_global(global_offset);
        }
        self.refresh_endpoints();
    }

    pub fn to_global(&self, local: Vector3d) -> Vector3d {
        let rotated = self.orientation() * local.0;
        Vector3d(rotated + self.center().0)
    }

    pub fn to_local(&self, global: Vector3d) -> Vector3d {
        let diff = global.0 - self.center().0;
        Vector3d(self.orientation().inverse() * diff)
    }
#[cfg(test)]
mod tests {
    use geometry::{assert_vec3_almost_eq, Vector3d};

    use super::*;
    use crate::{
        material::Material,
        section::Section,
    };

    #[test]
    fn member_direction_matches_node_difference() {
        let material = Material::new(
            210e9,
            0.3,
            8.004772271876737,
            78.5,
            1.2e-5,
            0.2,
            Some("S355".into()),
        );
        let section = Section::generic(material, Some("GenericSection".into()));
        let start = Node::new(Vector3d::new(1.3062447934024681, -0.12479177080853521, 0.0), Some("N1".into()));
        let end = Node::new(Vector3d::new(3.7937552065975324, 0.12479177080853521, 0.0), Some("N2".into()));
        let member = Member::new(
            start,
            end,
            Some(section),
            None,
            Some(1_000.0),
            Some(true),
            Some(Fixity::pinned()),
            Some(Fixity::pinned()),
            None,
            Some("CableMember".to_string()),
        );

        assert!((member.length() - 2.5).abs() < 1e-12);
        let dir = member.direction(Axis::AxisX);
        assert_vec3_almost_eq!(dir, Vector3d::new(0.9950041652780258, 0.09983341664682815, 0.0));
    }
}
