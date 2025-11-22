use std::ops::{Deref, DerefMut};

use crate::{
    linearelement::{Fixity, LinearElement},
    node::Node,
    section::Section,
};

/// Beam formed by two nodes enriched with section related metadata.
#[derive(Debug, Clone)]
pub struct Beam {
    element: LinearElement,
    section: Option<Section>,
    section_rotation: Option<f64>,
    init_tension: Option<f64>,
    is_cable: Option<bool>,
    device: Option<String>,
    start_fixity: Option<Fixity>,
    end_fixity: Option<Fixity>,
}

impl Beam {
    pub fn new(start_node: Node, end_node: Node) -> Self {
        Self {
            element: LinearElement::new(start_node, end_node),
            section: None,
            section_rotation: None,
            init_tension: None,
            is_cable: None,
            device: None,
            start_fixity: None,
            end_fixity: None,
        }
    }

    pub fn linear_element(&self) -> &LinearElement { &self.element }
    pub fn linear_element_mut(&mut self) -> &mut LinearElement { &mut self.element }

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

    pub fn get_section_rotation_value(&self) -> f64 { self.section_rotation.unwrap_or(0.0) }
    pub fn get_init_tension_value(&self) -> f64 { self.init_tension.unwrap_or(0.0) }
    pub fn get_is_cable_value(&self) -> bool { self.is_cable.unwrap_or(false) }
    pub fn get_device_value(&self) -> &str { self.device.as_deref().unwrap_or("") }
    pub fn get_start_fixity_value(&self) -> Fixity { self.start_fixity.clone().unwrap_or_default() }
    pub fn get_end_fixity_value(&self) -> Fixity { self.end_fixity.clone().unwrap_or_default() }
}

impl From<(Node, Node, Section)> for Beam {
    fn from((start, end, section): (Node, Node, Section)) -> Self {
        let mut beam = Beam::new(start, end);
        beam.set_section(section);
        beam
    }
}

impl Deref for Beam {
    type Target = LinearElement;

    fn deref(&self) -> &Self::Target { &self.element }
}

impl DerefMut for Beam {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.element }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    use geometry::{Axis, Line3d, Vector3d};
    use nalgebra::{Matrix3, Rotation3};
    use utils::{approx_eq, assert_almost_eq, assert_vec3_almost_eq};

    use super::*;
    use crate::{material::Material, section::Section, BoundingBox3d};

    fn beam_from_coords(start: (f64, f64, f64), end: (f64, f64, f64)) -> Beam {
        Beam::new(Node::new(start), Node::new(end))
    }

    #[test]
    fn direction_returns_member_local_axes_in_global_space() {
        let beam = beam_from_coords((0.0, 0.0, 0.0), (2.0, 2.0, 0.0));

        let axis_x = beam.direction(Axis::AxisX);
        let magnitude = (2.0_f64).sqrt();
        assert_vec3_almost_eq!(axis_x, Vector3d::new(1.0 / magnitude, 1.0 / magnitude, 0.0));

        let axis_y = beam.direction(Axis::AxisY);
        assert_vec3_almost_eq!(axis_y, Vector3d::new(-1.0 / magnitude, 1.0 / magnitude, 0.0));

        let axis_z = beam.direction(Axis::AxisZ);
        assert_vec3_almost_eq!(axis_z, Vector3d::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn axis_basis_is_orthonormal() {
        let beam = beam_from_coords((0.0, 0.0, 0.0), (3.0, 0.5, 0.0));

        let x_axis = beam.direction(Axis::AxisX);
        let y_axis = beam.direction(Axis::AxisY);
        let z_axis = beam.direction(Axis::AxisZ);

        assert_almost_eq!(x_axis.dot(&y_axis), 0.0);
        assert_almost_eq!(x_axis.dot(&z_axis), 0.0);
        assert_almost_eq!(y_axis.dot(&z_axis), 0.0);

        assert_vec3_almost_eq!(x_axis.cross(&y_axis), z_axis);
    }

    #[test]
    fn bounding_box_spans_beam_endpoints() {
        let beam = beam_from_coords((-1.0, 2.0, -0.5), (4.0, -3.0, 1.5));

        let bbox: BoundingBox3d = beam.bounding_box();
        assert_vec3_almost_eq!(bbox.min(), Vector3d::new(-1.0, -3.0, -0.5));
        assert_vec3_almost_eq!(bbox.max(), Vector3d::new(4.0, 2.0, 1.5));
    }

    #[test]
    fn align_axis_respects_section_rotation() {
        let mut beam = beam_from_coords((0.0, 0.0, 0.0), (2.0, 0.0, 0.0));
        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
        let section = Section::generic(material, None);
        beam.set_section(section);
        beam.set_section_rotation(FRAC_PI_2);

        let axis_x = beam.direction(Axis::AxisX);
        assert_vec3_almost_eq!(axis_x, Vector3d::new(1.0, 0.0, 0.0));

        let axis_y = beam.direction(Axis::AxisY);
        assert_vec3_almost_eq!(axis_y, Vector3d::new(0.0, 1.0, 0.0));

        let axis_z = beam.direction(Axis::AxisZ);
        assert_vec3_almost_eq!(axis_z, Vector3d::new(0.0, 0.0, 1.0));
        assert_almost_eq!(beam.get_section_rotation_value(), FRAC_PI_2);
    }

    #[test]
    fn rotate_updates_node_positions_and_orientation() {
        let mut beam = beam_from_coords((0.0, 0.0, 0.0), (2.0, 0.0, 0.0));
        beam.rotate(FRAC_PI_2, [0.0, 0.0, 1.0]);

        assert_vec3_almost_eq!(beam.start_node().center(), Vector3d::new(1.0, -1.0, 0.0));
        assert_vec3_almost_eq!(beam.end_node().center(), Vector3d::new(1.0, 1.0, 0.0));

        let axis_x = beam.direction(Axis::AxisX);
        assert_vec3_almost_eq!(axis_x, Vector3d::new(0.0, 1.0, 0.0));
    }

    #[test]
    fn rotation_matrix_matches_expected_orientation() {
        let mut beam = beam_from_coords((0.0, 0.0, 0.0), (1.0, 1.0, 0.0));
        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
        let section = Section::generic(material, None);
        beam.set_section(section);
        beam.set_section_rotation(FRAC_PI_4);

        let rotation = beam.rotation_matrix();
        let base_line = Line3d::new(beam.start_node().center(), beam.end_node().center());
        let local_axis = base_line.local_axis().expect("local axis defined");
        let col_x = local_axis.direction(Axis::AxisX).0;
        let col_y = local_axis.direction(Axis::AxisY).0;
        let col_z = local_axis.direction(Axis::AxisZ).0;
        let base = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[col_x, col_y, col_z]));
        let expected = base;

        for row in 0..3 {
            for col in 0..3 {
                assert!(
                    approx_eq!(rotation[(row, col)], expected[(row, col)]),
                    "rotation entry ({}, {}) mismatch: {} vs {}",
                    row,
                    col,
                    rotation[(row, col)],
                    expected[(row, col)]
                );
            }
        }
    }

    #[test]
    fn move_updates_nodes_and_line() {
        let mut beam = Beam::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(1.0, 0.0, 0.0)),
        );

        beam.r#move([1.0, 0.0, 0.0]);
        assert_vec3_almost_eq!(beam.start_node().center(), Vector3d::new(1.0, 0.0, 0.0));
        assert_vec3_almost_eq!(beam.end_node().center(), Vector3d::new(2.0, 0.0, 0.0));

        beam.move_global(Vector3d::new(0.0, -2.0, 0.0));
        assert_vec3_almost_eq!(beam.start_node().center(), Vector3d::new(1.0, -2.0, 0.0));
        assert_vec3_almost_eq!(beam.end_node().center(), Vector3d::new(2.0, -2.0, 0.0));

        let line = beam.to_line();
        assert_vec3_almost_eq!(line.start(), beam.start_node().center());
        assert_vec3_almost_eq!(line.end(), beam.end_node().center());
    }

    #[test]
    fn beam_from_nodes_can_assign_section_directly() {
        let start = Node::new(Vector3d::new(0.0, 0.0, 0.0));
        let end = Node::new(Vector3d::new(2.0, 0.0, 0.0));
        let mut beam = Beam::new(start.clone(), end.clone());
        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
        let section = Section::generic(material, Some("Custom".into()));

        beam.section = Some(section.clone());

        assert_eq!(beam.start_node(), &start);
        assert_eq!(beam.end_node(), &end);
        assert_eq!(beam.get_section(), Some(&section));
    }

    #[test]
    fn beam_with_nodes_and_section_tracks_metadata() {
        let start = Node::new(Vector3d::new(0.0, 0.0, 0.0));
        let end = Node::new(Vector3d::new(1.0, 2.0, 0.0));
        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, Some("Steel".into()));
        let mut beam = Beam::new(start, end);
        let section = Section::generic(material, None);

        beam.set_section(section.clone());
        beam.set_init_tension(12.5);
        beam.set_section_rotation(FRAC_PI_4);
        beam.set_is_cable(true);

        assert_eq!(beam.get_section(), Some(&section));
        assert_almost_eq!(beam.get_init_tension_value(), 12.5);
        assert_almost_eq!(beam.get_section_rotation_value(), FRAC_PI_4);
        assert!(beam.get_is_cable().unwrap());
    }

    #[test]
    fn move_accepts_list_and_vector_inputs() {
        let original_start = Vector3d::new(0.0, 0.0, 0.0);
        let original_end = Vector3d::new(2.0, 0.0, 0.0);
        let mut beam = Beam::new(Node::new(original_start), Node::new(original_end));

        beam.r#move([0.5, 1.0, -0.5]);
        assert_vec3_almost_eq!(beam.start_node().center(), Vector3d::new(0.5, 1.0, -0.5));
        assert_vec3_almost_eq!(beam.end_node().center(), Vector3d::new(2.5, 1.0, -0.5));

        beam.r#move(Vector3d::new(-0.5, -1.0, 0.5));
        assert_vec3_almost_eq!(beam.start_node().center(), original_start);
        assert_vec3_almost_eq!(beam.end_node().center(), original_end);
    }

    #[test]
    fn to_global_transforms_local_point() {
        let beam = beam_from_coords((0.0, 0.0, 0.0), (2.0, 2.0, 0.0));
        let global = beam.to_global(Vector3d::new(1.0, 0.0, 0.0));
        let offset = (2.0_f64).sqrt() / 2.0;
        assert_vec3_almost_eq!(global, Vector3d::new(1.0 + offset, 1.0 + offset, 0.0));
    }

    #[test]
    fn to_local_is_inverse_of_to_global() {
        let beam = beam_from_coords((0.0, 0.0, 0.0), (2.0, 2.0, 0.0));
        let offset = (2.0_f64).sqrt() / 2.0;
        let point = Vector3d::new(1.0 + offset, 1.0 + offset, 0.0);

        let local = beam.to_local(point);
        assert_vec3_almost_eq!(local, Vector3d::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn to_line_returns_segment_between_nodes() {
        let beam = beam_from_coords((-1.0, 0.5, 0.0), (3.0, -0.5, 0.0));
        let line: Line3d = beam.to_line();

        assert_vec3_almost_eq!(line.start(), beam.start_node().center());
        assert_vec3_almost_eq!(line.end(), beam.end_node().center());
    }
}
