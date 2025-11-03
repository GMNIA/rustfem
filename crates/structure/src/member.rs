use geometry::{epsilon, Axis, Line3d, Vector3d};
use nalgebra::{Matrix3, Matrix4, Rotation3, Unit};

use crate::{
    node::{BoundingBox3d, Node},
    section::Section,
};

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
    fn default() -> Self {
        Self::free()
    }
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

/// Beam or cable member connecting two nodes.
#[derive(Debug, Clone)]
pub struct Member {
    name: Option<String>,
    start_node: Node,
    end_node: Node,
    line: Line3d,
    section: Option<Section>,
    section_rotation: Option<f64>,
    init_tension: Option<f64>,
    is_cable: Option<bool>,
    device: Option<String>,
    start_fixity: Option<Fixity>,
    end_fixity: Option<Fixity>,
    mesh_nodes: Vec<Vector3d>,
    mesh: Vec<Line3d>,
}

impl Member {
    pub fn new(start_node: Node, end_node: Node) -> Self {
        let start_center = start_node.center();
        let end_center = end_node.center();
        let line = Line3d::new(start_center, end_center);
        let mut member = Self {
            name: None,
            start_node,
            end_node,
            line,
            section: None,
            section_rotation: None,
            init_tension: None,
            is_cable: None,
            device: None,
            start_fixity: None,
            end_fixity: None,
            mesh_nodes: Vec::new(),
            mesh: Vec::new(),
        };
        member.refresh_endpoints();
        member
    }

    pub fn from_points<S, E>(start: S, end: E, section: Option<Section>) -> Self
    where
        S: Into<Vector3d>,
        E: Into<Vector3d>,
    {
        let start_node = Node::new(start.into());
        let end_node = Node::new(end.into());
        let mut member = Self::new(start_node, end_node);
        if let Some(section) = section {
            member.section = Some(section);
        }
        member
    }

    pub fn from_positions<S, E>(start: S, end: E) -> Self
    where
        S: Into<Vector3d>,
        E: Into<Vector3d>,
    {
        Self::from_points(start, end, None)
    }

    pub fn set_section(&mut self, section: Section) {
        self.section = Some(section);
    }

    pub fn clear_section(&mut self) {
        self.section = None;
        self.section_rotation = None;
    }

    pub fn get_section(&self) -> Option<&Section> {
        self.section.as_ref()
    }

    pub fn set_section_rotation(&mut self, section_rotation: f64) {
        if self.section.is_some() {
            self.section_rotation = Some(section_rotation);
        } else {
            self.section_rotation = None;
        }
    }

    pub fn clear_section_rotation(&mut self) {
        self.section_rotation = None;
    }

    pub fn get_section_rotation(&self) -> Option<f64> {
        self.section_rotation
    }

    pub fn set_init_tension(&mut self, init_tension: f64) {
        self.init_tension = Some(init_tension);
    }

    pub fn clear_init_tension(&mut self) {
        self.init_tension = None;
    }

    pub fn get_init_tension(&self) -> Option<f64> {
        self.init_tension
    }

    pub fn set_is_cable(&mut self, is_cable: bool) {
        self.is_cable = Some(is_cable);
    }

    pub fn clear_is_cable(&mut self) {
        self.is_cable = None;
    }

    pub fn get_is_cable(&self) -> Option<bool> {
        self.is_cable
    }

    pub fn set_device<S: Into<String>>(&mut self, device: S) {
        let value = device.into();
        if value.is_empty() {
            self.device = None;
        } else {
            self.device = Some(value);
        }
    }

    pub fn clear_device(&mut self) {
        self.device = None;
    }

    pub fn get_device(&self) -> Option<&str> {
        self.device.as_deref()
    }

    pub fn set_start_fixity(&mut self, start_fixity: Fixity) {
        self.start_fixity = Some(start_fixity);
    }

    pub fn clear_start_fixity(&mut self) {
        self.start_fixity = None;
    }

    pub fn get_start_fixity(&self) -> Option<&Fixity> {
        self.start_fixity.as_ref()
    }

    pub fn set_end_fixity(&mut self, end_fixity: Fixity) {
        self.end_fixity = Some(end_fixity);
    }

    pub fn clear_end_fixity(&mut self) {
        self.end_fixity = None;
    }

    pub fn get_end_fixity(&self) -> Option<&Fixity> {
        self.end_fixity.as_ref()
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

    fn refresh_endpoints(&mut self) {
        if self.mesh_nodes.is_empty() {
            self.mesh_nodes.push(self.start_node.center());
            self.mesh_nodes.push(self.end_node.center());
        } else {
            if let Some(first) = self.mesh_nodes.first_mut() {
                *first = self.start_node.center();
            }
            if let Some(last) = self.mesh_nodes.last_mut() {
                *last = self.end_node.center();
            }
        }
        self.line.set_endpoints(self.start_node.center(), self.end_node.center());
    }

    pub fn start_node(&self) -> &Node { &self.start_node }
    pub fn end_node(&self) -> &Node { &self.end_node }
    pub fn get_section_rotation_value(&self) -> f64 { self.section_rotation.unwrap_or(0.0) }
    pub fn get_init_tension_value(&self) -> f64 { self.init_tension.unwrap_or(0.0) }
    pub fn get_is_cable_value(&self) -> bool { self.is_cable.unwrap_or(false) }
    pub fn get_device_value(&self) -> &str { self.device.as_deref().unwrap_or("") }
    pub fn get_start_fixity_value(&self) -> Fixity { self.start_fixity.clone().unwrap_or_default() }
    pub fn get_end_fixity_value(&self) -> Fixity { self.end_fixity.clone().unwrap_or_default() }
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
        self.refresh_endpoints();
    }

    pub fn r#move<T: IntoVec3>(&mut self, offset: T) {
        let offset_vec = offset.into_vec3();
        self.line.r#move(offset_vec);
        for node in [&mut self.start_node, &mut self.end_node] {
            node.move_global(offset_vec);
        }
        self.refresh_endpoints();
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
}

impl<S, E> From<(S, E)> for Member
where
    S: Into<Vector3d>,
    E: Into<Vector3d>,
{
    fn from(value: (S, E)) -> Self {
        Self::from_points(value.0, value.1, None)
    }
}

impl From<(Node, Node, Section)> for Member {
    fn from((start, end, section): (Node, Node, Section)) -> Self {
        let mut member = Member::new(start, end);
        member.set_section(section);
        member
    }
}

impl<S, E> From<(S, E, Section)> for Member
where
    S: Into<Vector3d>,
    E: Into<Vector3d>,
{
    fn from(value: (S, E, Section)) -> Self {
        Self::from_points(value.0, value.1, Some(value.2))
    }
}

impl<S, E> From<(S, E, Option<Section>)> for Member
where
    S: Into<Vector3d>,
    E: Into<Vector3d>,
{
    fn from(value: (S, E, Option<Section>)) -> Self {
        Self::from_points(value.0, value.1, value.2)
    }
}

#[cfg(test)]
mod tests {
    use geometry::{assert_almost_eq, assert_vec3_almost_eq, Vector3d};

    use super::*;
    use crate::{
        material::Material,
        section::Section,
    };

    #[test]
    fn member_direction_matches_node_difference() {
        let start_node: Node = (Vector3d::new(0.0, 0.0, 0.0), "N1").into();
        let end_node: Node = (Vector3d::new(3.0, 4.0, 0.0), "N2").into();
        let member = Member::new(start_node, end_node);

        assert_almost_eq!(member.length(), 5.0);
        let direction = member.direction(Axis::AxisX);
        assert_vec3_almost_eq!(direction, Vector3d::new(3.0 / 5.0, 4.0 / 5.0, 0.0));

        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
        let section = Section::generic(material, Some("WithSection".into()));
        let mut member_with_section = Member::new(
            (Vector3d::new(0.0, 0.0, 0.0), "N1").into(),
            (Vector3d::new(3.0, 4.0, 0.0), "N2").into(),
        );
        member_with_section.set_section(section);
        member_with_section.set_name("member_with_section");

        assert_almost_eq!(member_with_section.length(), 5.0);
        assert!(member_with_section.get_section().is_some());
        let direction_with_section = member_with_section.direction(Axis::AxisX);
        assert_vec3_almost_eq!(direction_with_section, Vector3d::new(3.0 / 5.0, 4.0 / 5.0, 0.0));
        assert_eq!(member_with_section.get_name(), Some("member_with_section"));
    }

    #[test]
    fn member_rotates_with_orientation_tracking() {
        let mut member = Member::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(1.0, 0.0, 0.0)),
        );
        member.rotate(0.0, [0.0, 0.0, 1.0]);
        assert_vec3_almost_eq!(member.direction(Axis::AxisX), Vector3d::new(1.0, 0.0, 0.0));
        assert_vec3_almost_eq!(member.direction(Axis::AxisY), Vector3d::new(0.0, 1.0, 0.0));
        assert_vec3_almost_eq!(member.direction(Axis::AxisZ), Vector3d::new(0.0, 0.0, 1.0));
        assert_almost_eq!(member.get_section_rotation_value(), 0.0);

        let mut member = Member::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(1.0, 0.0, 0.0)),
        );
        let quarter_turn = std::f64::consts::FRAC_PI_2;
        member.rotate(quarter_turn, [0.0, 0.0, 1.0]);
        assert_vec3_almost_eq!(member.direction(Axis::AxisX), Vector3d::new(0.0, 1.0, 0.0));
        assert_vec3_almost_eq!(member.direction(Axis::AxisY), Vector3d::new(-1.0, 0.0, 0.0));
        assert_vec3_almost_eq!(member.direction(Axis::AxisZ), Vector3d::new(0.0, 0.0, 1.0));
        assert_almost_eq!(member.get_section_rotation_value(), 0.0);

        let mut member = Member::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(1.0, 0.0, 0.0)),
        );
        member.rotate(quarter_turn, [1.0, 0.0, 0.0]);
        assert_vec3_almost_eq!(member.direction(Axis::AxisX), Vector3d::new(1.0, 0.0, 0.0));
        assert_vec3_almost_eq!(member.direction(Axis::AxisY), Vector3d::new(0.0, 0.0, 1.0));
        assert_vec3_almost_eq!(member.direction(Axis::AxisZ), Vector3d::new(0.0, -1.0, 0.0));
        assert_almost_eq!(member.get_section_rotation_value(), 0.0);

        // With section present, section rotation can actually change.
        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
        let section = Section::generic(material, None);
        let start_node = Node::new(Vector3d::new(0.0, 0.0, 0.0));
        let end_node = Node::new(Vector3d::new(1.0, 0.0, 0.0));
        let mut member: Member = (start_node, end_node, section).into();
        member.set_section_rotation(std::f64::consts::FRAC_PI_4);
        assert_almost_eq!(member.get_section_rotation_value(), std::f64::consts::FRAC_PI_4);
    }
}
