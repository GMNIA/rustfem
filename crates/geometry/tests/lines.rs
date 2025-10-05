use geometry::{assert_almost_eq, epsilon, set_epsilon, Line2d, Line3d, Vector2d, Vector3d};

#[test]
fn line2d_basic_properties() {
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 0.0));
    assert_almost_eq!(line.length(), 4.0);
    let dir = line.direction().unwrap();
    assert_almost_eq!(dir.x(), 1.0);
    assert_almost_eq!(dir.y(), 0.0);
    let (min, max) = line.bounding_box();
    assert_almost_eq!(min.x(), 0.0);
    assert_almost_eq!(max.x(), 4.0);
}

#[test]
fn line3d_distance_and_projection() {
    let line = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let point = Vector3d::new(1.0, 0.0, 5.0);
    assert_almost_eq!(line.distance(&point), 1.0);
    let projection = line.projection(&point);
    assert_almost_eq!(projection.x(), 0.0);
    assert_almost_eq!(projection.z(), 5.0);
}

#[test]
fn line_break_and_reverse() {
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(2.0, 0.0));
    let segments = line.break_at(0.5);
    assert_eq!(segments.len(), 2);
    assert_almost_eq!(segments[0].length(), 1.0);
    assert_almost_eq!(segments[1].length(), 1.0);

    let mut mutable = line;
    mutable.reverse();
    assert_almost_eq!(mutable.start().x(), 2.0);
    assert_almost_eq!(mutable.end().x(), 0.0);
    let reversed = mutable.reversed();
    assert_almost_eq!(reversed.start().x(), 0.0);
}

#[test]
fn intersection_behaviour() {
    let a = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 4.0));
    let b = Line2d::new(Vector2d::new(4.0, 0.0), Vector2d::new(0.0, 4.0));
    let point = a.intersection(&b, false).unwrap();
    assert_almost_eq!(point.x(), 2.0);
    assert_almost_eq!(point.y(), 2.0);

    let c = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 5.0));
    let d = Line3d::new(Vector3d::new(1.0, 0.0, 0.0), Vector3d::new(1.0, 0.0, 5.0));
    assert!(c.intersection(&d, false).is_none());
}

#[test]
fn epsilon_respected_in_lines() {
    let original = epsilon();
    set_epsilon(1e-6);
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1e-7, 0.0));
    assert!(line.direction().is_none());
    set_epsilon(original);
}

#[test]
fn contains_and_point_parameter() {
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(10.0, 0.0));
    let point = Vector2d::new(3.0, 0.0);
    assert!(line.contains(&point));
    assert_almost_eq!(line.point_parameter(&point).unwrap(), 0.3);

    let outside = Vector2d::new(12.0, 0.0);
    assert!(line.point_parameter(&outside).is_none());
}

#[test]
fn length_at_point_matches_parameter() {
    let line = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let point = Vector3d::new(0.0, 0.0, 4.0);
    assert_almost_eq!(line.length_at_point(&point), 4.0);
}

#[test]
fn start_and_end_accessors() {
    let start = Vector3d::new(1.0, 2.0, 3.0);
    let end = Vector3d::new(4.0, 5.0, 6.0);
    let line = Line3d::new(start, end);
    assert_eq!(line.start(), start);
    assert_eq!(line.end(), end);
}

#[test]
fn point_at_supports_extrapolation() {
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(2.0, 0.0));
    let midpoint = line.point_at(0.5);
    assert_almost_eq!(midpoint.x(), 1.0);
    let beyond = line.point_at(1.5);
    assert_almost_eq!(beyond.x(), 3.0);
}

#[test]
fn break_at_point_splits_line() {
    let line = Line3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let split_point = Vector3d::new(0.0, 0.0, 2.5);
    let segments = line.break_at_point(&split_point);
    assert_eq!(segments.len(), 2);
    assert_almost_eq!(segments[0].end().z(), 2.5);
    assert_almost_eq!(segments[1].start().z(), 2.5);
}

#[test]
fn move_start_and_end_mutate_line() {
    let mut line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(1.0, 0.0));
    line.move_start(Vector2d::new(-1.0, 0.0));
    line.move_end(Vector2d::new(2.0, 0.0));
    assert_almost_eq!(line.start().x(), -1.0);
    assert_almost_eq!(line.end().x(), 2.0);
    assert_almost_eq!(line.length(), 3.0);
}

#[test]
fn reversed_returns_new_line_without_mutating_original() {
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(3.0, 0.0));
    let reversed = line.reversed();
    assert_almost_eq!(reversed.start().x(), 3.0);
    assert_almost_eq!(reversed.end().x(), 0.0);
    assert_almost_eq!(line.start().x(), 0.0);
}

#[test]
fn break_at_out_of_range_returns_original() {
    let line = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(5.0, 0.0));
    let segments = line.break_at(1.5);
    assert_eq!(segments.len(), 1);
    assert_almost_eq!(segments[0].length(), 5.0);
}

#[test]
fn ray_intersection_respects_direction() {
    let ray = Line2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(5.0, 0.0));
    let target = Line2d::new(Vector2d::new(-5.0, -5.0), Vector2d::new(-5.0, 5.0));
    assert!(ray.ray_intersection(&target).is_none());

    let target2 = Line2d::new(Vector2d::new(3.0, -5.0), Vector2d::new(3.0, 5.0));
    let hit = ray.ray_intersection(&target2).unwrap();
    assert_almost_eq!(hit.x(), 3.0);
    assert_almost_eq!(hit.y(), 0.0);
}
