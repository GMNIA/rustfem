use geometry::{assert_almost_eq, Arc2d, Arc3d, Line2d, Line3d, Vector2d, Vector3d};
use geometry::line::LineVector;
use std::f64::consts::PI;

#[test]
fn arc2d_basic_properties() {
    let arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
    assert_almost_eq!(arc.radius(), 1.0);
    assert!(arc.contains(&Vector2d::new((2.0f64).sqrt() / 2.0, (2.0f64).sqrt() / 2.0)));
    assert_almost_eq!(arc.length(), PI / 2.0);
}

#[test]
fn arc_break_and_reverse_behaviour() {
    let arc = Arc3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(1.0, 0.0, 0.0), Vector3d::new(0.0, 1.0, 0.0), false);
    let pieces = arc.break_at(0.5);
    assert_eq!(pieces.len(), 2);
    assert_almost_eq!(pieces[0].length(), arc.length() / 2.0);
    let reversed = arc.reversed();
    assert_eq!(reversed.start(), arc.end());
    assert_eq!(reversed.end(), arc.start());
}

#[test]
fn arc_point_at_and_tangents() {
    let arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
    let mid = arc.point_at(0.5);
    assert_almost_eq!(mid.x(), (2.0f64).sqrt() / 2.0);
    assert_almost_eq!(mid.y(), (2.0f64).sqrt() / 2.0);
    let tangent = arc.start_tangent();
    assert_almost_eq!(tangent.x(), 0.0);
    assert_almost_eq!(tangent.y(), 1.0);
}

#[test]
fn arc_line_intersection_filters_points_on_arc() {
    let arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
    let line = Line2d::new(Vector2d::new(0.0, 0.5), Vector2d::new(1.0, 0.5));
    let intersections = arc.intersection_with_line(&line, false);
    assert_eq!(intersections.len(), 1);
    assert!(arc.contains(&intersections[0]));
}

#[test]
fn arc_from_three_points_recovers_arc() {
    let p1 = Vector2d::new(1.0, 0.0);
    let p2 = Vector2d::new((2.0f64).sqrt() / 2.0, (2.0f64).sqrt() / 2.0);
    let p3 = Vector2d::new(0.0, 1.0);
    let arc = Arc2d::from_three_points(p1, p2, p3).expect("valid arc");
    assert!(arc.contains(&p2));
    assert_almost_eq!(arc.length(), PI / 2.0);
}

#[test]
fn arc_linearized_returns_segments() {
    let arc = Arc2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0), Vector2d::new(0.0, 1.0), false);
    let segments = arc.linearized(4);
    assert_eq!(segments.len(), 4);
    assert_almost_eq!(segments[0].start().x(), 1.0);
    assert_almost_eq!(segments.last().unwrap().end().y(), 1.0);
}

#[test]
fn arc3d_contains_points_on_z_plane() {
    let arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        false,
    );
    let mid = arc.point_at(0.5);
    assert!(mid.is_approx(&Vector3d::new(0.7071067811865476, 0.7071067811865475, 0.0), None));
    assert!(arc.contains(&arc.start()));
    assert!(arc.contains(&arc.end()));
    assert!(arc.contains(&mid));

    let outside_radius = Vector3d::new(mid.x(), mid.y(), 0.0).normalize().scale(1.2);
    assert!(!arc.contains(&outside_radius));

    let off_plane = Vector3d::new(mid.x(), mid.y(), 0.1);
    assert!(!arc.contains(&off_plane));
}

#[test]
fn arc3d_contains_points_on_3d_plane_manually_input() {
    let arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(-1., 0.0, -1.0),
        Vector3d::new(0.0, 1., 1.0),
        false,
    );
    let mid = arc.point_at(0.5);
    let manually_calculated_mid = Vector3d::new(-1., 1., 0.);
    assert!(mid.is_approx(&manually_calculated_mid, None));
    assert!(arc.contains(&arc.start()));
    assert!(arc.contains(&arc.end()));
    assert!(arc.contains(&manually_calculated_mid));
}

#[test]
fn arc3d_contains_respects_orientation() {
    let clockwise_arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        true,
    );
    let counter_arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        false,
    );

    let p = Vector3d::new((2.0f64).sqrt() / 2.0, (2.0f64).sqrt() / 2.0, 0.0);
    assert!(counter_arc.contains(&p));
    assert!(clockwise_arc.contains(&p));

    let beyond = Vector3d::new(-0.5, 0.5, 0.0);
    assert!(!counter_arc.contains(&beyond));
    assert!(!clockwise_arc.contains(&beyond));
}

#[test]
fn arc3d_length_and_angle_at_point() {
    let arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        false,
    );
    let point = Vector3d::new((2.0f64).sqrt() / 2.0, (2.0f64).sqrt() / 2.0, 0.0);
    let angle = arc.angle_from_point(&point);
    assert_almost_eq!(angle, PI / 4.0);
    assert_almost_eq!(arc.length_at_point(&point), arc.radius() * angle);
}

#[test]
fn arc3d_intersection_with_line() {
    let arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        false,
    );
    let line = Line3d::new(Vector3d::new(-1.0, 0.0, 0.0), Vector3d::new(2.0, 0.0, 0.0));
    let hits = arc.intersection_with_line(&line, false);
    assert_eq!(hits.len(), 1);
    assert!(arc.contains(&hits[0]));
    let ray_hits = arc.intersection_with_line(&line, true);
    assert_eq!(ray_hits.len(), 2);
}

#[test]
fn arc3d_intersection_with_arc() {
    let arc1 = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        false,
    );
    let arc2 = Arc3d::new(
        Vector3d::new(0.5, 0.0, 0.0),
        Vector3d::new(0.5, -0.5, 0.0),
        Vector3d::new(0.5, 0.5, 0.0),
        false,
    );
    let intersections = arc1.intersection_with_arc(&arc2);
    assert_eq!(intersections.len(), 1);
    assert!(arc1.contains(&intersections[0]));
    assert!(arc2.contains(&intersections[0]));
}

#[test]
fn arc3d_break_at_point() {
    let arc = Arc3d::new(
        Vector3d::new(0.0, 0.0, 0.0),
        Vector3d::new(1.0, 0.0, 0.0),
        Vector3d::new(0.0, 1.0, 0.0),
        false,
    );
    let point = arc.point_at(0.3);
    let segments = arc.break_at_point(&point);
    assert_eq!(segments.len(), 2);
    assert!(segments.iter().all(|seg| seg.contains(&point)));
    assert_almost_eq!(segments[0].length() + segments[1].length(), arc.length());
}
