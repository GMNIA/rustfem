use geometry::{assert_almost_eq, Edge, Vector2d, Vector3d};

#[test]
fn edge2d_basic_behaviour() {
    let edge = Edge::new(Vector2d::new(0.0, 0.0), Vector2d::new(3.0, 0.0));
    assert_almost_eq!(edge.length(), 3.0);
    assert!(edge.contains(&Vector3d::from(Vector2d::new(1.5, 0.0))));
    let centroid = edge.centroid();
    assert_almost_eq!(centroid.x(), 1.5);
}

#[test]
fn edge_break_and_intersection() {
    let edge = Edge::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 4.0));
    let parts = edge.break_at(0.5);
    assert_eq!(parts.len(), 2);
    // Use a 3D edge to test intersection with a 3D line
    let edge3 = Edge::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(4.0, 4.0, 0.0));
    let diag = geometry::Line::new(Vector3d::new(0.0, 4.0, 0.0), Vector3d::new(4.0, 0.0, 0.0));
    let hit = edge3.intersection_with_line(&geometry::Line::new(Vector3d::new(0.0, 4.0, 0.0), Vector3d::new(4.0, 0.0, 0.0)), false).unwrap();
    assert_almost_eq!(hit.x(), 2.0);
}

#[test]
fn edge_tangents_swap_on_reverse() {
    let mut edge = Edge::with_tangents(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(0.0, 0.0, 5.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
    );
    edge.reverse();
    assert_eq!(edge.start_tangent().unwrap(), Vector3d::new(0.0, 1.0, 0.0));
    assert_eq!(edge.end_tangent().unwrap(), Vector3d::new(1.0, 0.0, 0.0));
}

#[test]
fn edge_ray_intersection_requires_forward_progress() {
    let edge = Edge::new(Vector2d::new(0.0, 0.0), Vector2d::new(5.0, 0.0));
    let other = Edge::new(Vector2d::new(-1.0, -1.0), Vector2d::new(-1.0, 1.0));
    assert!(edge.ray_intersection_with_edge(&other).is_none());

    let crossing = Edge::new(Vector2d::new(2.0, -1.0), Vector2d::new(2.0, 1.0));
    let hit = edge.ray_intersection_with_edge(&crossing).unwrap();
    assert_almost_eq!(hit.x(), 2.0);
    assert_almost_eq!(hit.y(), 0.0);
}

#[test]
fn edge_closest_point_matches_point_at() {
    let edge = Edge::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let point = Vector3d::new(0.0, 0.0, 3.0);
    let closest = edge.closest_point(&point);
    let expected = edge.point_at(0.3);
    assert_eq!(closest, expected);
    assert_almost_eq!(edge.length_at_point(&point), 3.0);
}
