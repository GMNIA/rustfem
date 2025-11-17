use geometry::{Axis, Line, Vector3d};
use utils::{assert_almost_eq, assert_vec3_almost_eq};

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
    assert_almost_eq!(segments.len() as f64, 2.0);
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
    let line = Line::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(1e-7, 0.0, 0.0));
    // With fixed epsilon, this short line still has a well-defined direction.
    let dir = line.direction().expect("direction should be defined for short line");
    assert_almost_eq!(dir.x(), 1.0);
    assert_almost_eq!(dir.y(), 0.0);
    assert_almost_eq!(dir.z(), 0.0);
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
    assert_vec3_almost_eq!(line.start(), start);
    assert_vec3_almost_eq!(line.end(), end);
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
    assert_almost_eq!(segments.len() as f64, 2.0);
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
    assert_almost_eq!(segments.len() as f64, 1.0);
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
    for case in reference_line_cases().into_iter() {
        let line = Line::new(v3(case.start), v3(case.end));
        let la = line.local_axis().expect("local axis defined");
        let expected_axes = case.expected_axes.map(v3);
        let axis_labels = ["axisX", "axisY", "axisZ"];

        for (idx, axis_label) in axis_labels.iter().enumerate() {
            let axis = match idx {
                0 => Axis::AxisX,
                1 => Axis::AxisY,
                _ => Axis::AxisZ,
            };
            let actual = la.direction(axis);
            let expected = &expected_axes[idx];
            assert_vec3_almost_eq!(
                &format!("{} {}", case.name, axis_label),
                &actual,
                expected,
            );
        }

        let rot = line.rotation_matrix().expect("rotation matrix");
        for (idx, expected) in expected_axes.iter().enumerate() {
            let column = Vector3d::new(rot[(0, idx)], rot[(1, idx)], rot[(2, idx)]);
            assert_vec3_almost_eq!(&format!("{} rot col{}", case.name, idx), &column, expected);
        }
    }
}

#[test]
fn rotate_line() {
    for case in reference_line_cases().into_iter() {
        let mut line = Line::new(v3(case.start), v3(case.end));
        line.rotate(case.rotation.angle, case.rotation.axis);

        let rot_after = line.rotation_matrix().expect("rotation matrix after rotate");
        let expected_columns = case.rotation.expected_columns.map(v3);
        for (idx, expected) in expected_columns.iter().enumerate() {
            let column = Vector3d::new(rot_after[(0, idx)], rot_after[(1, idx)], rot_after[(2, idx)]);
            assert_vec3_almost_eq!(&format!("{} rot_after col{}", case.name, idx), &column, expected);
        }
    }
}

#[derive(Clone, Copy)]
struct RotationReference {
    angle: f64,
    axis: [f64; 3],
    expected_columns: [[f64; 3]; 3],
}

#[derive(Clone, Copy)]
struct LineReference {
    name: &'static str,
    start: [f64; 3],
    end: [f64; 3],
    expected_axes: [[f64; 3]; 3],
    rotation: RotationReference,
}

fn v3(values: [f64; 3]) -> Vector3d {
    Vector3d::new(values[0], values[1], values[2])
}

fn reference_line_cases() -> [LineReference; 5] {
    [
        LineReference {
            name: "line1",
            start: [8.8629468830558018, 0.6063292911286204, 3.2383809849393881],
            end: [-8.8641849571395177, 8.3974564436718744, -7.9913375830590061],
            expected_axes: [
                [-0.791942861123317798, 0.348061242178514141, -0.501677063865753170],
                [0.294029768810411984, 0.937471797812152730, 0.186260901318945543],
                [0.535138299665711692, 0.0, -0.844764464351389965],
            ],
            rotation: RotationReference {
                angle: 1.234567890123456690,
                axis: [-0.791942861123317798, 0.348061242178514141, -0.501677063865753170],
                expected_columns: [
                    [-0.791942861123317687, 0.348061242178514141, -0.501677063865753059],
                    [0.602182586684826160, 0.309299198191272495, -0.736009604755133551],
                    [-0.101008103681759664, -0.884978744203150702, -0.454543711098535752],
                ],
            },
        },
        LineReference {
            name: "line2",
            start: [1.118934201736756506, -4.272800337640170198, 7.510805322357963831],
            end: [8.301498598350981695, -4.908171084453350375, -3.588028946472332947],
            expected_axes: [
                [0.542676386967694824, -0.048005236323126810, -0.838568921623768171],
                [0.026081377840261620, 0.998847084035169419, -0.040302164264376224],
                [0.839536837046262252, 0.0, 0.543302769404277686],
            ],
            rotation: RotationReference {
                angle: -2.345678901234566904,
                axis: [0.026081377840261620, 0.998847084035169419, -0.040302164264376224],
                expected_columns: [
                    [0.220177888051432902, 0.033586008103318063, 0.974881365948131462],
                    [0.026081377840261627, 0.998847084035169530, -0.040302164264376245],
                    [-0.975110998473078583, 0.034299894666355871, 0.219048072081707068],
                ],
            },
        },
        LineReference {
            name: "line3",
            start: [3.775271637572201300, 2.844966188100034543, 0.820457452320102476],
            end: [1.778194199324607538, 0.719245564265655446, -9.309296179504123359],
            expected_axes: [
                [-0.189452747977124858, -0.201656483571541090, -0.960959062040353862],
                [-0.039005695896009607, 0.979456309711444262, -0.197848156559790184],
                [0.981114780222774874, 0.0, -0.193426440872017213],
            ],
            rotation: RotationReference {
                angle: 3.456789012345677925,
                axis: [0.981114780222774874, 0.0, -0.193426440872017213],
                expected_columns: [
                    [0.192211346969217378, -0.111912570629632085, 0.974951472962294008],
                    [-0.021646850225727973, -0.993718056862743948, -0.109799077137457249],
                    [0.981114780222774985, 0.0, -0.193426440872017213],
                ],
            },
        },
        LineReference {
            name: "line4",
            start: [-3.547902422314901827, 6.657026268528404955, 8.698787710044754817],
            end: [-1.435754523140591488, 7.731013670364074386, 2.868332385162467801],
            expected_axes: [
                [0.335604932445631998, 0.170648783440507462, -0.926416926674182406],
                [-0.058123128127595472, 0.985331920070731671, 0.160445346664801508],
                [0.940207972362937294, 0.0, 0.340600893576593733],
            ],
            rotation: RotationReference {
                angle: -4.567890123456789020,
                axis: [0.335604932445631998, 0.170648783440507462, -0.926416926674182406],
                expected_columns: [
                    [0.335604932445632054, 0.170648783440507545, -0.926416926674182517],
                    [0.938778834951623065, -0.141884374981772476, 0.313947644015535154],
                    [-0.077869303098436343, -0.975062980963951276, -0.207818562185039912],
                ],
            },
        },
        LineReference {
            name: "line5",
            start: [1.338369301497582597, -6.615170746840433047, -2.487315837569119559],
            end: [0.347904363068756994, -9.579487526422141741, -8.526699604919251385],
            expected_axes: [
                [-0.145652905809256217, -0.435917856284862903, -0.888122093859470163],
                [-0.070548508787524805, 0.899986456882551966, -0.430171227926562927],
                [0.986817176044872291, 0.0, -0.161838997348672214],
            ],
            rotation: RotationReference {
                angle: 5.678901234567890199,
                axis: [-0.070548508787524805, 0.899986456882551966, -0.430171227926562927],
                expected_columns: [
                    [0.440823854500862533, -0.358720761703848701, -0.822796295826380586],
                    [-0.070548508787524833, 0.899986456882551966, -0.430171227926562927],
                    [0.894816873561768977, 0.247676790496406840, 0.371428284112209461],
                ],
            },
        },
    ]
}
