use geometry::Vector3d;
use utils::assert_almost_eq;

use structure::{Beam, Member, Node};

#[test]
fn member_inherits_beam_kinematics() {
    let start = Node::new(Vector3d::new(0.0, 0.0, 0.0));
    let end = Node::new(Vector3d::new(2.0, 0.0, 0.0));
    let mut member: Member = (start, end).into();
    member.r#move([1.0, 0.0, 0.0]);

    assert_almost_eq!(member.length(), 2.0);
    assert_eq!(member.mesh().len(), 0);

    member.rotate(std::f64::consts::FRAC_PI_2, [0.0, 0.0, 1.0]);
    assert_almost_eq!(member.length(), 2.0);
}

#[test]
fn member_mesh_can_be_populated_later() {
    let mut member = Member::new(
        Node::new(Vector3d::new(0.0, 0.0, 0.0)),
        Node::new(Vector3d::new(2.0, 0.0, 0.0)),
    );
    let child = Beam::new(
        Node::new(Vector3d::new(0.0, 0.0, 0.0)),
        Node::new(Vector3d::new(1.0, 0.0, 0.0)),
    );
    member.add_mesh_beam(child);
    assert_eq!(member.mesh().len(), 1);
    assert_almost_eq!(member.mesh()[0].length(), 1.0);
}
