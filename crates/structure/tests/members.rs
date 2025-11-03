use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use geometry::{assert_almost_eq, assert_vec3_almost_eq, Axis, Line3d, Vector3d};
use nalgebra::{Matrix3, Rotation3};
use structure::{BoundingBox3d, Material, Member, Node, Section};

#[test]
fn direction_returns_member_local_axes_in_global_space() {
    let member: Member = ((0.0, 0.0, 0.0), (2.0, 2.0, 0.0)).into();

    let axis_x = member.direction(Axis::AxisX);
    let magnitude = (2.0_f64).sqrt();
    assert_vec3_almost_eq!(axis_x, Vector3d::new(1.0 / magnitude, 1.0 / magnitude, 0.0));

    let axis_y = member.direction(Axis::AxisY);
    assert_vec3_almost_eq!(axis_y, Vector3d::new(-1.0 / magnitude, 1.0 / magnitude, 0.0));

    let axis_z = member.direction(Axis::AxisZ);
    assert_vec3_almost_eq!(axis_z, Vector3d::new(0.0, 0.0, 1.0));
}

#[test]
fn axis_basis_is_orthonormal() {
    let member: Member = ((0.0, 0.0, 0.0), (3.0, 0.5, 0.0)).into();

    let x_axis = member.direction(Axis::AxisX);
    let y_axis = member.direction(Axis::AxisY);
    let z_axis = member.direction(Axis::AxisZ);

    assert_almost_eq!(x_axis.dot(&y_axis), 0.0);
    assert_almost_eq!(x_axis.dot(&z_axis), 0.0);
    assert_almost_eq!(y_axis.dot(&z_axis), 0.0);

    assert_vec3_almost_eq!(x_axis.cross(&y_axis), z_axis);
}

#[test]
fn bounding_box_spans_member_endpoints() {
    let member: Member = ((-1.0, 2.0, -0.5), (4.0, -3.0, 1.5)).into();

    let bbox: BoundingBox3d = member.bounding_box();
    assert_vec3_almost_eq!(bbox.min(), Vector3d::new(-1.0, -3.0, -0.5));
    assert_vec3_almost_eq!(bbox.max(), Vector3d::new(4.0, 2.0, 1.5));
}

#[test]
fn align_axis_respects_section_rotation() {
    let mut member: Member = ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)).into();
    let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
    let section = Section::generic(material, None);
    member.set_section(section);
    member.set_section_rotation(FRAC_PI_2);

    let axis_x = member.direction(Axis::AxisX);
    assert_vec3_almost_eq!(axis_x, Vector3d::new(1.0, 0.0, 0.0));

    let axis_y = member.direction(Axis::AxisY);
    assert_vec3_almost_eq!(axis_y, Vector3d::new(0.0, 1.0, 0.0));

    let axis_z = member.direction(Axis::AxisZ);
    assert_vec3_almost_eq!(axis_z, Vector3d::new(0.0, 0.0, 1.0));
    assert_almost_eq!(member.get_section_rotation_value(), FRAC_PI_2);
}

#[test]
fn rotate_updates_node_positions_and_orientation() {
    let mut member: Member = ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)).into();
    member.rotate(FRAC_PI_2, [0.0, 0.0, 1.0]);

    assert_vec3_almost_eq!(member.start_node().center(), Vector3d::new(1.0, -1.0, 0.0));
    assert_vec3_almost_eq!(member.end_node().center(), Vector3d::new(1.0, 1.0, 0.0));

    let axis_x = member.direction(Axis::AxisX);
    assert_vec3_almost_eq!(axis_x, Vector3d::new(0.0, 1.0, 0.0));
}

#[test]
fn rotation_matrix_matches_expected_orientation() {
    let mut member: Member = ((0.0, 0.0, 0.0), (1.0, 1.0, 0.0)).into();
    let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
    let section = Section::generic(material, None);
    member.set_section(section);
    member.set_section_rotation(FRAC_PI_4);

    let rotation = member.rotation_matrix();
    let base_line = Line3d::new(member.start_node().center(), member.end_node().center());
    let local_axis = base_line.local_axis().expect("local axis defined");
    let col_x = local_axis.direction(Axis::AxisX).0;
    let col_y = local_axis.direction(Axis::AxisY).0;
    let col_z = local_axis.direction(Axis::AxisZ).0;
    let base = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[col_x, col_y, col_z]));
    let expected = base;

    for row in 0..3 {
        for col in 0..3 {
            assert_almost_eq!(
                rotation[(row, col)],
                expected[(row, col)],
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
    let mut member = Member::new(
        Node::new(Vector3d::new(0.0, 0.0, 0.0)),
        Node::new(Vector3d::new(1.0, 0.0, 0.0)),
    );

    member.r#move([1.0, 0.0, 0.0]);
    assert_vec3_almost_eq!(member.start_node().center(), Vector3d::new(1.0, 0.0, 0.0));
    assert_vec3_almost_eq!(member.end_node().center(), Vector3d::new(2.0, 0.0, 0.0));

    member.move_global(Vector3d::new(0.0, -2.0, 0.0));
    assert_vec3_almost_eq!(member.start_node().center(), Vector3d::new(1.0, -2.0, 0.0));
    assert_vec3_almost_eq!(member.end_node().center(), Vector3d::new(2.0, -2.0, 0.0));

    let line = member.to_line();
    assert_vec3_almost_eq!(line.start(), Vector3d::new(1.0, -2.0, 0.0));
    assert_vec3_almost_eq!(line.end(), Vector3d::new(2.0, -2.0, 0.0));
}

#[test]
fn move_accepts_list_and_vector_inputs() {
    let original_start = Vector3d::new(0.0, 0.0, 0.0);
    let original_end = Vector3d::new(2.0, 0.0, 0.0);
    let mut member = Member::new(
        Node::new(original_start),
        Node::new(original_end),
    );

    member.r#move([0.5, 1.0, -0.5]);
    assert_vec3_almost_eq!(member.start_node().center(), Vector3d::new(0.5, 1.0, -0.5));
    assert_vec3_almost_eq!(member.end_node().center(), Vector3d::new(2.5, 1.0, -0.5));

    member.r#move(Vector3d::new(-0.5, -1.0, 0.5));
    assert_vec3_almost_eq!(member.start_node().center(), original_start);
    assert_vec3_almost_eq!(member.end_node().center(), original_end);
}

#[test]
fn rotate_refreshes_mesh_nodes_for_all_input_forms() {
    let mut member = Member::new(
        Node::new(Vector3d::new(0.0, 0.0, 0.0)),
        Node::new(Vector3d::new(1.0, 0.0, 0.0)),
    );

    member.rotate(0.0, [0.0, 0.0, 1.0]);
    let mesh = member.mesh_nodes();
    assert_eq!(mesh.len(), 2);
    assert_vec3_almost_eq!(mesh[0], member.start_node().center());
    assert_vec3_almost_eq!(mesh[1], member.end_node().center());

    member.rotate(std::f64::consts::FRAC_PI_2, Vector3d::new(0.0, 0.0, 1.0));
    let mesh = member.mesh_nodes();
    assert_eq!(mesh.len(), 2);
    assert_vec3_almost_eq!(mesh[0], member.start_node().center());
    assert_vec3_almost_eq!(mesh[1], member.end_node().center());
}

#[test]
fn to_global_transforms_local_point() {
    let member: Member = ((0.0, 0.0, 0.0), (2.0, 2.0, 0.0)).into();
    let global = member.to_global(Vector3d::new(1.0, 0.0, 0.0));
    let offset = (2.0_f64).sqrt() / 2.0;
    assert_vec3_almost_eq!(global, Vector3d::new(1.0 + offset, 1.0 + offset, 0.0));
}

#[test]
fn to_local_is_inverse_of_to_global() {
    let member: Member = ((0.0, 0.0, 0.0), (2.0, 2.0, 0.0)).into();
    let offset = (2.0_f64).sqrt() / 2.0;
    let point = Vector3d::new(1.0 + offset, 1.0 + offset, 0.0);

    let local = member.to_local(point);
    assert_vec3_almost_eq!(local, Vector3d::new(1.0, 0.0, 0.0));
}

#[test]
fn to_line_returns_segment_between_nodes() {
    let member: Member = ((-1.0, 0.5, 0.0), (3.0, -0.5, 0.0)).into();
    let line: Line3d = member.to_line();

    assert_vec3_almost_eq!(line.start(), member.start_node().center());
    assert_vec3_almost_eq!(line.end(), member.end_node().center());
}
