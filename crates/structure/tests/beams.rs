use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use geometry::{Axis, Line3d, Vector3d};
use nalgebra::{Matrix3, Rotation3};
use utils::{approx_eq, assert_almost_eq, assert_vec3_almost_eq};

use structure::{Beam, BoundingBox3d, Material, Node, Section};

fn beam_from_coords(start: (f64, f64, f64), end: (f64, f64, f64)) -> Beam {
    Beam::new(Node::new(start), Node::new(end))
}

#[test]
fn direction_returns_beam_local_axes_in_global_space() {
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

#[test]
fn linear_element_and_line_reflect_beam_geometry() {
    let start = Node::new(Vector3d::new(-1.0, 0.0, 0.0));
    let end = Node::new(Vector3d::new(2.0, 3.0, 1.0));
    let beam = Beam::new(start.clone(), end.clone());

    let element = beam.linear_element();
    assert_eq!(element.start_node(), &start);
    assert_eq!(element.end_node(), &end);
    assert_almost_eq!(element.length(), beam.length());

    let element_line = element.to_line();
    let beam_line = beam.to_line();
    assert_vec3_almost_eq!(element_line.start(), beam_line.start());
    assert_vec3_almost_eq!(element_line.end(), beam_line.end());
}
