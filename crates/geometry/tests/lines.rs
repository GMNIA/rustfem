use geometry::{assert_almost_eq, epsilon, set_epsilon, Axis, Line, Vector3d};

#[test]
fn line2d_basic_properties() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(4.0, 0.0, 0.0));
    assert_almost_eq!(line.length(), 4.0);
    let dir = line.direction().unwrap();
    assert_almost_eq!(dir.x(), 1.0);
    assert_almost_eq!(dir.y(), 0.0);
    assert_almost_eq!(dir.z(), 0.0);
    let (min, max) = line.bounding_box();
    assert_almost_eq!(min.x(), 0.0);
    assert_almost_eq!(max.x(), 4.0);
}

#[test]
fn line3d_distance_and_projection() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let point = Vector3d::new(1.0, 0.0, 5.0);
    assert_almost_eq!(line.distance(&point), 1.0);
    let projection = line.projection(&point);
    assert_almost_eq!(projection.x(), 0.0);
    assert_almost_eq!(projection.z(), 5.0);
}

#[test]
fn line_break_and_reverse() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(2.0, 0.0, 0.0));
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
    let a = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(4.0, 4.0, 0.0));
    let b = Line::new(Vector3d::new(4.0, 0.0, 0.0), Vector3d::new(0.0, 4.0, 0.0));
    let point = a.intersection(&b, false).unwrap();
    assert_almost_eq!(point.x(), 2.0);
    assert_almost_eq!(point.y(), 2.0);

    let c = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 5.0));
    let d = Line::new(Vector3d::new(1.0, 0.0, 0.0), Vector3d::new(1.0, 0.0, 5.0));
    assert!(c.intersection(&d, false).is_none());
}

#[test]
fn epsilon_respected_in_lines() {
    let original = epsilon();
    set_epsilon(1e-6);
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(1e-7, 0.0, 0.0));
    assert!(line.direction().is_none());
    set_epsilon(original);
}

#[test]
fn contains_and_point_parameter() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(10.0, 0.0, 0.0));
    let point = Vector3d::new(3.0, 0.0, 0.0);
    assert!(line.contains(&point));
    assert_almost_eq!(line.point_parameter(&point).unwrap(), 0.3);

    let outside = Vector3d::new(12.0, 0.0, 0.0);
    assert!(line.point_parameter(&outside).is_none());
}

#[test]
fn length_at_point_matches_parameter() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let point = Vector3d::new(0.0, 0.0, 4.0);
    assert_almost_eq!(line.length_at_point(&point), 4.0);
}

#[test]
fn start_and_end_accessors() {
    let start = Vector3d::new(1.0, 2.0, 3.0);
    let end = Vector3d::new(4.0, 5.0, 6.0);
    let line = Line::new(start, end);
    assert_eq!(line.start(), start);
    assert_eq!(line.end(), end);
}

#[test]
fn point_at_supports_extrapolation() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(2.0, 0.0, 0.0));
    let midpoint = line.point_at(0.5);
    assert_almost_eq!(midpoint.x(), 1.0);
    let beyond = line.point_at(1.5);
    assert_almost_eq!(beyond.x(), 3.0);
}

#[test]
fn break_at_point_splits_line() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
    let split_point = Vector3d::new(0.0, 0.0, 2.5);
    let segments = line.break_at_point(&split_point);
    assert_eq!(segments.len(), 2);
    assert_almost_eq!(segments[0].end().z(), 2.5);
    assert_almost_eq!(segments[1].start().z(), 2.5);
}

#[test]
fn move_start_and_end_mutate_line() {
    let mut line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(1.0, 0.0, 0.0));
    line.move_start(Vector3d::new(-1.0, 0.0, 0.0));
    line.move_end(Vector3d::new(2.0, 0.0, 0.0));
    assert_almost_eq!(line.start().x(), -1.0);
    assert_almost_eq!(line.end().x(), 2.0);
    assert_almost_eq!(line.length(), 3.0);
}

#[test]
fn reversed_returns_new_line_without_mutating_original() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(3.0, 0.0, 0.0));
    let reversed = line.reversed();
    assert_almost_eq!(reversed.start().x(), 3.0);
    assert_almost_eq!(reversed.end().x(), 0.0);
    assert_almost_eq!(line.start().x(), 0.0);
}

#[test]
fn break_at_out_of_range_returns_original() {
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(5.0, 0.0, 0.0));
    let segments = line.break_at(1.5);
    assert_eq!(segments.len(), 1);
    assert_almost_eq!(segments[0].length(), 5.0);
}

#[test]
fn ray_intersection_respects_direction() {
    let ray = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(5.0, 0.0, 0.0));
    let target = Line::new(Vector3d::new(-5.0, -5.0, 0.0), Vector3d::new(-5.0, 5.0, 0.0));
    assert!(ray.ray_intersection(&target).is_none());

    let target2 = Line::new(Vector3d::new(3.0, -5.0, 0.0), Vector3d::new(3.0, 5.0, 0.0));
    let hit = ray.ray_intersection(&target2).unwrap();
    assert_almost_eq!(hit.x(), 3.0);
    assert_almost_eq!(hit.y(), 0.0);
}

#[test]
fn local_axis_reference_line() {
    // --- Line 1 (high-precision reference) ---
    {
    let line = Line::new(
            Vector3d::new(8.8629468830558018, 0.6063292911286204, 3.2383809849393881),
            Vector3d::new(-8.8641849571395177, 8.3974564436718744, -7.9913375830590061),
        );

        let la = line.local_axis().expect("local axis defined");
        let dx = la.direction(Axis::AxisX);
        let dy = la.direction(Axis::AxisY);
        let dz = la.direction(Axis::AxisZ);

        // Use global precision for approximate comparisons

        // Expected axes (high precision)
        let ex = (-0.7919428611233178, 0.3480612421785141, -0.5016770638657532);
        let ey = (0.2940297688104120, 0.9374717978121527, 0.1862609013189455);
        let ez = (0.5351382996657117, 0.0000000000000000, -0.8447644643513900);

        // Axis X
        assert!((dx.x() - ex.0).abs() < epsilon());
        assert!((dx.y() - ex.1).abs() < epsilon());
        assert!((dx.z() - ex.2).abs() < epsilon());

        // Axis Y
        assert!((dy.x() - ey.0).abs() < epsilon());
        assert!((dy.y() - ey.1).abs() < epsilon());
        assert!((dy.z() - ey.2).abs() < epsilon());

        // Axis Z
        assert!((dz.x() - ez.0).abs() < epsilon());
        assert!(dz.y().abs() < epsilon()); // expected 0.0
        assert!((dz.z() - ez.2).abs() < epsilon());

        // Rotation matrix columns should match the axes [X, Y, Z]
        let rot = line.rotation_matrix().expect("rotation matrix");
        // Column 0 == Axis X
        assert!((rot[(0, 0)] - ex.0).abs() < epsilon());
        assert!((rot[(1, 0)] - ex.1).abs() < epsilon());
        assert!((rot[(2, 0)] - ex.2).abs() < epsilon());
        // Column 1 == Axis Y
        assert!((rot[(0, 1)] - ey.0).abs() < epsilon());
        assert!((rot[(1, 1)] - ey.1).abs() < epsilon());
        assert!((rot[(2, 1)] - ey.2).abs() < epsilon());
        // Column 2 == Axis Z
        assert!((rot[(0, 2)] - ez.0).abs() < epsilon());
        assert!(rot[(1, 2)].abs() < epsilon()); // expected 0.0
        assert!((rot[(2, 2)] - ez.2).abs() < epsilon());
    }

    // --- Line 2 ---
    {
    let line = Line::new(
            Vector3d::new(1.1189342017367565, -4.2728003376401702, 7.5108053223579638),
            Vector3d::new(8.3014985983509817, -4.9081710844533504, -3.5880289464723329),
        );
        let la = line.local_axis().expect("local axis defined");
        let dx = la.direction(Axis::AxisX);
        let dy = la.direction(Axis::AxisY);
        let dz = la.direction(Axis::AxisZ);
        let ex = (0.5426763869676948, -0.0480052363231268, -0.8385689216237682);
        let ey = (0.0260813778402616, 0.9988470840351694, -0.0403021642643762);
        let ez = (0.8395368370462623, 0.0, 0.5433027694042777);
        // Axes
        assert!((dx.x() - ex.0).abs() < epsilon());
        assert!((dx.y() - ex.1).abs() < epsilon());
        assert!((dx.z() - ex.2).abs() < epsilon());
        assert!((dy.x() - ey.0).abs() < epsilon());
        assert!((dy.y() - ey.1).abs() < epsilon());
        assert!((dy.z() - ey.2).abs() < epsilon());
        assert!((dz.x() - ez.0).abs() < epsilon());
        assert!(dz.y().abs() < epsilon());
        assert!((dz.z() - ez.2).abs() < epsilon());
        // Rotation columns == axes
        let rot = line.rotation_matrix().expect("rotation matrix");
        assert!((rot[(0, 0)] - ex.0).abs() < epsilon());
        assert!((rot[(1, 0)] - ex.1).abs() < epsilon());
        assert!((rot[(2, 0)] - ex.2).abs() < epsilon());
        assert!((rot[(0, 1)] - ey.0).abs() < epsilon());
        assert!((rot[(1, 1)] - ey.1).abs() < epsilon());
        assert!((rot[(2, 1)] - ey.2).abs() < epsilon());
        assert!((rot[(0, 2)] - ez.0).abs() < epsilon());
        assert!(rot[(1, 2)].abs() < epsilon());
        assert!((rot[(2, 2)] - ez.2).abs() < epsilon());
    }

    // --- Line 3 ---
    {
    let line = Line::new(
            Vector3d::new(3.7752716375722013, 2.8449661881000345, 0.8204574523201025),
            Vector3d::new(1.7781941993246075, 0.7192455642656554, -9.3092961795041234),
        );
        let la = line.local_axis().expect("local axis defined");
        let dx = la.direction(Axis::AxisX);
        let dy = la.direction(Axis::AxisY);
        let dz = la.direction(Axis::AxisZ);
        let ex = (-0.1894527479771249, -0.2016564835715411, -0.9609590620403539);
        let ey = (-0.0390056958960096, 0.9794563097114443, -0.1978481565597902);
        let ez = (0.9811147802227749, 0.0, -0.1934264408720172);
        // Axes
        assert!((dx.x() - ex.0).abs() < epsilon());
        assert!((dx.y() - ex.1).abs() < epsilon());
        assert!((dx.z() - ex.2).abs() < epsilon());
        assert!((dy.x() - ey.0).abs() < epsilon());
        assert!((dy.y() - ey.1).abs() < epsilon());
        assert!((dy.z() - ey.2).abs() < epsilon());
        assert!((dz.x() - ez.0).abs() < epsilon());
        assert!(dz.y().abs() < epsilon());
        assert!((dz.z() - ez.2).abs() < epsilon());
        // Rotation columns == axes
        let rot = line.rotation_matrix().expect("rotation matrix");
        assert!((rot[(0, 0)] - ex.0).abs() < epsilon());
        assert!((rot[(1, 0)] - ex.1).abs() < epsilon());
        assert!((rot[(2, 0)] - ex.2).abs() < epsilon());
        assert!((rot[(0, 1)] - ey.0).abs() < epsilon());
        assert!((rot[(1, 1)] - ey.1).abs() < epsilon());
        assert!((rot[(2, 1)] - ey.2).abs() < epsilon());
        assert!((rot[(0, 2)] - ez.0).abs() < epsilon());
        assert!(rot[(1, 2)].abs() < epsilon());
        assert!((rot[(2, 2)] - ez.2).abs() < epsilon());
    }

    // --- Line 4 ---
    {
    let line = Line::new(
            Vector3d::new(-3.5479024223149018, 6.6570262685284050, 8.6987877100447548),
            Vector3d::new(-1.4357545231405915, 7.7310136703640744, 2.8683323851624678),
        );
        let la = line.local_axis().expect("local axis defined");
        let dx = la.direction(Axis::AxisX);
        let dy = la.direction(Axis::AxisY);
        let dz = la.direction(Axis::AxisZ);
        let ex = (0.3356049324456320, 0.1706487834405075, -0.9264169266741824);
        let ey = (-0.0581231281275955, 0.9853319200707317, 0.1604453466648015);
        let ez = (0.9402079723629373, 0.0, 0.3406008935765937);
        // Axes
        assert!((dx.x() - ex.0).abs() < epsilon());
        assert!((dx.y() - ex.1).abs() < epsilon());
        assert!((dx.z() - ex.2).abs() < epsilon());
        assert!((dy.x() - ey.0).abs() < epsilon());
        assert!((dy.y() - ey.1).abs() < epsilon());
        assert!((dy.z() - ey.2).abs() < epsilon());
        assert!((dz.x() - ez.0).abs() < epsilon());
        assert!(dz.y().abs() < epsilon());
        assert!((dz.z() - ez.2).abs() < epsilon());
        // Rotation columns == axes
        let rot = line.rotation_matrix().expect("rotation matrix");
        assert!((rot[(0, 0)] - ex.0).abs() < epsilon());
        assert!((rot[(1, 0)] - ex.1).abs() < epsilon());
        assert!((rot[(2, 0)] - ex.2).abs() < epsilon());
        assert!((rot[(0, 1)] - ey.0).abs() < epsilon());
        assert!((rot[(1, 1)] - ey.1).abs() < epsilon());
        assert!((rot[(2, 1)] - ey.2).abs() < epsilon());
        assert!((rot[(0, 2)] - ez.0).abs() < epsilon());
        assert!(rot[(1, 2)].abs() < epsilon());
        assert!((rot[(2, 2)] - ez.2).abs() < epsilon());
    }

    // --- Line 5 ---
    {
    let line = Line::new(
            Vector3d::new(1.3383693014975826, -6.6151707468404330, -2.4873158375691196),
            Vector3d::new(0.3479043630687570, -9.5794875264221417, -8.5266996049192514),
        );
        let la = line.local_axis().expect("local axis defined");
        let dx = la.direction(Axis::AxisX);
        let dy = la.direction(Axis::AxisY);
        let dz = la.direction(Axis::AxisZ);
        let ex = (-0.1456529058092562, -0.4359178562848629, -0.8881220938594702);
        let ey = (-0.0705485087875248, 0.8999864568825520, -0.4301712279265629);
        let ez = (0.9868171760448723, 0.0, -0.1618389973486722);
        // Axes
        assert!((dx.x() - ex.0).abs() < epsilon());
        assert!((dx.y() - ex.1).abs() < epsilon());
        assert!((dx.z() - ex.2).abs() < epsilon());
        assert!((dy.x() - ey.0).abs() < epsilon());
        assert!((dy.y() - ey.1).abs() < epsilon());
        assert!((dy.z() - ey.2).abs() < epsilon());
        assert!((dz.x() - ez.0).abs() < epsilon());
        assert!(dz.y().abs() < epsilon());
        assert!((dz.z() - ez.2).abs() < epsilon());
        // Rotation columns == axes
        let rot = line.rotation_matrix().expect("rotation matrix");
        assert!((rot[(0, 0)] - ex.0).abs() < epsilon());
        assert!((rot[(1, 0)] - ex.1).abs() < epsilon());
        assert!((rot[(2, 0)] - ex.2).abs() < epsilon());
        assert!((rot[(0, 1)] - ey.0).abs() < epsilon());
        assert!((rot[(1, 1)] - ey.1).abs() < epsilon());
        assert!((rot[(2, 1)] - ey.2).abs() < epsilon());
        assert!((rot[(0, 2)] - ez.0).abs() < epsilon());
        assert!(rot[(1, 2)].abs() < epsilon());
        assert!((rot[(2, 2)] - ez.2).abs() < epsilon());
    }
}
