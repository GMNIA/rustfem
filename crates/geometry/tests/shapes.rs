use geometry::{
    assert_almost_eq,
    assert_vec3_almost_eq,
    Disk,
    Rectangle,
    Shape,
    ShapeC,
    ShapeI,
    ShapeL,
    ShapeT,
    Vector3d,
};
use std::f64::consts::PI;

fn relative_error(expected: f64, actual: f64) -> f64 {
    if expected == 0.0 {
        actual.abs()
    } else {
        ((actual - expected) / expected).abs()
    }
}

fn assert_relative_eq(label: &str, expected: f64, actual: f64) {
    // Use a fixed tolerance here to avoid global epsilon side effects.
    let tol = 1e-10;
    let err = relative_error(expected, actual);
    assert!(
        err <= tol,
        "{label} relative error {err} exceeds tolerance {tol} (expected={expected}, actual={actual})"
    );
}

#[test]
fn rectangle_full_matches_reference_snapshot() {
    {
        let rect = Rectangle::new(220.0, 140.0, 100.0, 60.0);
        let poly = rect.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let centroidal_local = poly.centroidal_local_second_moment_of_area();
        let principal = poly.local_principal_axes();
        let global = poly.second_moment_of_area();
        let centroidal_global = poly.centroidal_second_moment_of_area();

        let expected_area = 30800.0;
        assert_relative_eq("rectangle full area", expected_area, area);
        assert_vec3_almost_eq!(centroid, Vector3d::new(0.0, 0.0, 0.0));

        let expected_i_xx_local = 50306666.66666666;
        let expected_i_yy_local = 124226666.66666669;
        assert_relative_eq("rectangle full I_xx local", expected_i_xx_local, local[(0, 0)]);
        assert_relative_eq("rectangle full I_yy local", expected_i_yy_local, local[(1, 1)]);
        assert_almost_eq!(local[(0, 1)], 0.0);

        let expected_i_xx_centroidal_local = 50306666.66666666;
        let expected_i_yy_centroidal_local = 124226666.66666669;
        assert_relative_eq(
            "rectangle full I_xx centroidal local",
            expected_i_xx_centroidal_local,
            centroidal_local[(0, 0)],
        );
        assert_relative_eq(
            "rectangle full I_yy centroidal local",
            expected_i_yy_centroidal_local,
            centroidal_local[(1, 1)],
        );
        assert_almost_eq!(centroidal_local[(0, 1)], 0.0);

        assert_relative_eq("rectangle full principal (0,0)", 1.0, principal[(0, 0)]);
        assert_relative_eq("rectangle full principal (0,1)", 0.0, principal[(0, 1)]);
        assert_relative_eq("rectangle full principal (1,0)", 0.0, principal[(1, 0)]);
        assert_relative_eq("rectangle full principal (1,1)", 1.0, principal[(1, 1)]);

        let expected_i_xx_global = 50306666.66666666;
        let expected_i_yy_global = 124226666.66666669;
        let expected_i_zz_global = 174533333.33333334;
        assert_relative_eq("rectangle full I_xx global", expected_i_xx_global, global[(0, 0)]);
        assert_relative_eq("rectangle full I_yy global", expected_i_yy_global, global[(1, 1)]);
        assert_relative_eq("rectangle full I_zz global", expected_i_zz_global, global[(2, 2)]);
        assert_almost_eq!(global[(0, 1)], 0.0);
        assert_almost_eq!(global[(0, 2)], 0.0);
        assert_almost_eq!(global[(1, 2)], 0.0);

        let expected_i_xx_centroidal_global = 50306666.66666666;
        let expected_i_yy_centroidal_global = 124226666.66666669;
        let expected_i_zz_centroidal_global = 174533333.33333334;
        assert_relative_eq(
            "rectangle full I_xx centroidal global",
            expected_i_xx_centroidal_global,
            centroidal_global[(0, 0)],
        );
        assert_relative_eq(
            "rectangle full I_yy centroidal global",
            expected_i_yy_centroidal_global,
            centroidal_global[(1, 1)],
        );
        assert_relative_eq(
            "rectangle full I_zz centroidal global",
            expected_i_zz_centroidal_global,
            centroidal_global[(2, 2)],
        );
    }
}

#[test]
fn disk_simple_matches_reference_snapshot() {
    {
        let disk = Disk::new(50.0, 0.0);

        let expected_area = PI * 50.0 * 50.0;
        assert_relative_eq("disk simple area", expected_area, disk.area());

        let inertia = disk.second_moment_of_area();
        let i_xx = inertia[(0, 0)];
        let i_yy = inertia[(1, 1)];
        let i_zz = inertia[(2, 2)];

        let expected_i_planar = PI * 50.0_f64.powi(4) / 4.0;
        assert_relative_eq("disk simple I_xx", expected_i_planar, i_xx);
        assert_relative_eq("disk simple I_yy", expected_i_planar, i_yy);

        let expected_i_zz = expected_i_planar * 2.0;
        assert_relative_eq("disk simple I_zz", expected_i_zz, i_zz);
    }
}

#[test]
fn disk_full_matches_reference_snapshot() {
    {
        let disk = Disk::new(80.0, 20.0);

        let expected_area = PI * (80.0 * 80.0 - 20.0 * 20.0);
        assert_relative_eq("disk full area", expected_area, disk.area());

        let inertia = disk.second_moment_of_area();
        let i_xx = inertia[(0, 0)];
        let i_yy = inertia[(1, 1)];
        let i_zz = inertia[(2, 2)];

        let expected_i_planar = PI * (80.0_f64.powi(4) - 20.0_f64.powi(4)) / 4.0;
        assert_relative_eq("disk full I_xx", expected_i_planar, i_xx);
        assert_relative_eq("disk full I_yy", expected_i_planar, i_yy);

        let expected_i_zz = expected_i_planar * 2.0;
        assert_relative_eq("disk full I_zz", expected_i_zz, i_zz);
    }
}

#[test]
fn shape_i_full_matches_reference_snapshot() {
    {
        let shape = ShapeI::new(
            150.0,
            150.0,
            360.0,
            16.0,
            16.0,
            10.0,
            12.0,
            5.0,
            6.0,
            2.0_f64.to_radians(),
            3.0_f64.to_radians(),
        );
        let poly = shape.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let global = poly.second_moment_of_area();

        let expected_area = 8080.0;
        assert_relative_eq("shape I full area", expected_area, area);
        assert_vec3_almost_eq!(centroid, Vector3d::new(0.0, 0.0, 0.0));

        let expected_local_xx = 171511893.3333333;
        let expected_local_yy = 9027333.333333336;
        assert_relative_eq("shape I full I_xx local", expected_local_xx, local[(0, 0)]);
        assert_relative_eq("shape I full I_yy local", expected_local_yy, local[(1, 1)]);

        let expected_global_xx = 171511893.3333333;
        let expected_global_yy = 9027333.333333336;
        let expected_global_zz = 180539226.66666666;
        assert_relative_eq("shape I full I_xx global", expected_global_xx, global[(0, 0)]);
        assert_relative_eq("shape I full I_yy global", expected_global_yy, global[(1, 1)]);
        assert_relative_eq("shape I full I_zz global", expected_global_zz, global[(2, 2)]);
    }
}

#[test]
fn shape_c_simple_matches_reference_snapshot() {
    {
        let shape = ShapeC::new(80.0, 80.0, 200.0, 10.0, 10.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let poly = shape.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let global = poly.second_moment_of_area();

        let expected_area = 2680.0;
        assert_relative_eq("shape C simple area", expected_area, area);
        assert_vec3_almost_eq!(
            centroid,
            Vector3d::new(25.08955223880597, 0.0, 0.0),
        );

        let expected_local_xx = 17369333.333333336;
        let expected_local_yy = 3426293.3333333335;
        assert_relative_eq("shape C simple I_xx local", expected_local_xx, local[(0, 0)]);
        assert_relative_eq("shape C simple I_yy local", expected_local_yy, local[(1, 1)]);

        let expected_global_xx = 17369333.333333336;
        let expected_global_yy = 3426293.3333333335;
        let expected_global_zz = 20795626.666666668;
        assert_relative_eq("shape C simple I_xx global", expected_global_xx, global[(0, 0)]);
        assert_relative_eq("shape C simple I_yy global", expected_global_yy, global[(1, 1)]);
        assert_relative_eq("shape C simple I_zz global", expected_global_zz, global[(2, 2)]);
    }
}

#[test]
fn shape_c_full_matches_reference_snapshot() {
    {
        let shape = ShapeC::new(
            110.0,
            90.0,
            240.0,
            14.0,
            12.0,
            8.0,
            8.0,
            4.0,
            5.0,
            3.0,
            3.5,
            1.5_f64.to_radians(),
            2.0_f64.to_radians(),
        );
        let poly = shape.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let global = poly.second_moment_of_area();

        let expected_area = 4332.0;
        assert_relative_eq("shape C full area", expected_area, area);
        assert_vec3_almost_eq!(
            centroid,
            Vector3d::new(32.35180055401662, -11.35457063711911, 0.0),
        );

        let expected_local_xx = 40273327.99999999;
        let expected_local_yy = 9163856.0;
        assert_relative_eq("shape C full I_xx local", expected_local_xx, local[(0, 0)]);
        assert_relative_eq("shape C full I_yy local", expected_local_yy, local[(1, 1)]);

        let expected_local_xy = -4023852.0000000005;
        assert_relative_eq("shape C full I_xy local", expected_local_xy, local[(0, 1)]);

        let expected_global_xx = 40273327.99999999;
        let expected_global_yy = 9163856.0;
        let expected_global_zz = 49437183.99999999;
        assert_relative_eq("shape C full I_xx global", expected_global_xx, global[(0, 0)]);
        assert_relative_eq("shape C full I_yy global", expected_global_yy, global[(1, 1)]);
        assert_relative_eq("shape C full I_xy global", 4023852.0000000005, global[(0, 1)]);
        assert_relative_eq("shape C full I_zz global", expected_global_zz, global[(2, 2)]);
    }
}

#[test]
fn shape_l_simple_matches_reference_snapshot() {
    {
        let shape = ShapeL::new(80.0, 80.0, 8.0, 8.0, 0.0, 0.0, 0.0, 0.0);
        let poly = shape.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let global = poly.second_moment_of_area();

        let expected_area = 1216.0;
        assert_relative_eq("shape L simple area", expected_area, area);
        assert_vec3_almost_eq!(
            centroid,
            Vector3d::new(18.94736842105263, -17.05263157894737, 0.0),
        );

        let expected_local_xx = 1090901.3333333333;
        let expected_local_yy = 1173845.3333333333;
        assert_relative_eq("shape L simple I_xx local", expected_local_xx, local[(0, 0)]);
        assert_relative_eq("shape L simple I_yy local", expected_local_yy, local[(1, 1)]);

        let expected_local_xy = -829440.0;
        assert_relative_eq("shape L simple I_xy local", expected_local_xy, local[(0, 1)]);

        let expected_global_xx = 1090901.3333333333;
        let expected_global_yy = 1173845.3333333333;
        let expected_global_zz = 2264746.6666666665;
        assert_relative_eq("shape L simple I_xx global", expected_global_xx, global[(0, 0)]);
        assert_relative_eq("shape L simple I_yy global", expected_global_yy, global[(1, 1)]);
        assert_relative_eq("shape L simple I_xy global", 829440.0, global[(0, 1)]);
        assert_relative_eq("shape L simple I_zz global", expected_global_zz, global[(2, 2)]);
    }
}

#[test]
fn shape_l_full_matches_reference_snapshot() {
    {
        let shape = ShapeL::new(
            120.0,
            100.0,
            12.0,
            10.0,
            6.0,
            3.0,
            2.0,
            2.5_f64.to_radians(),
        );
        let poly = shape.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let global = poly.second_moment_of_area();

        let expected_area = 2320.0;
        assert_relative_eq("shape L full area", expected_area, area);
        assert_vec3_almost_eq!(
            centroid,
            Vector3d::new(34.13793103448276, -25.03448275862069, 0.0),
        );

        let expected_local_xx = 3404693.3333333335;
        let expected_local_yy = 6091333.333333333;
        assert_relative_eq("shape L full I_xx local", expected_local_xx, local[(0, 0)]);
        assert_relative_eq("shape L full I_yy local", expected_local_yy, local[(1, 1)]);

        let expected_local_xy = -3484800.0;
        assert_relative_eq("shape L full I_xy local", expected_local_xy, local[(0, 1)]);

        let expected_global_xx = 3404693.3333333335;
        let expected_global_yy = 6091333.333333333;
        let expected_global_zz = 9496026.666666666;
        assert_relative_eq("shape L full I_xx global", expected_global_xx, global[(0, 0)]);
        assert_relative_eq("shape L full I_yy global", expected_global_yy, global[(1, 1)]);
        assert_relative_eq("shape L full I_xy global", 3484800.0, global[(0, 1)]);
        assert_relative_eq("shape L full I_zz global", expected_global_zz, global[(2, 2)]);
    }
}

#[test]
fn shape_t_full_matches_reference_snapshot() {
        let shape = ShapeT::new(
            160.0,
            140.0,
            16.0,
            10.0,
            10.0,
            4.0,
            1.5_f64.to_radians(),
        );
        let poly = shape.to_polygon();
        let area = poly.area();
        let centroid = poly.centroid();
        let local = poly.local_second_moment_of_area();
        let global = poly.second_moment_of_area();

        let expected_area = 3800.0;
        assert_relative_eq("shape T full area", expected_area, area);
        assert_vec3_almost_eq!(centroid, Vector3d::new(0.0, 39.15789473684211, 0.0));

        let expected_local_xx = 11563466.66666666;
        let expected_local_yy = 5471666.666666667;
        assert_relative_eq("shape T full I_xx local", expected_local_xx, local[(0, 0)]);
        assert_relative_eq("shape T full I_yy local", expected_local_yy, local[(1, 1)]);

        let expected_global_xx = 11563466.66666666;
        let expected_global_yy = 5471666.666666667;
        let expected_global_zz = 17035133.33333333;
        assert_relative_eq("shape T full I_xx global", expected_global_xx, global[(0, 0)]);
        assert_relative_eq("shape T full I_yy global", expected_global_yy, global[(1, 1)]);
        assert_relative_eq("shape T full I_zz global", expected_global_zz, global[(2, 2)]);
}

#[test]
fn all_shapes_full_have_valid_polygons() {
        let rect_full = Rectangle::new(220.0, 140.0, 100.0, 60.0);
        assert!(rect_full.area() > 0.0, "Rectangle_full should have positive area");

        let disk_full = Disk::new(80.0, 20.0);
        assert!(disk_full.area() > 0.0, "Disk_full should have positive area");

        let shape_i_full = ShapeI::new(
            150.0,
            150.0,
            360.0,
            16.0,
            16.0,
            10.0,
            12.0,
            5.0,
            6.0,
            2.0_f64.to_radians(),
            3.0_f64.to_radians(),
        );
        assert!(shape_i_full.area() > 0.0, "ShapeI_full should have positive area");

        let shape_c_full = ShapeC::new(
            110.0,
            90.0,
            240.0,
            14.0,
            12.0,
            8.0,
            8.0,
            4.0,
            5.0,
            3.0,
            3.5,
            1.5_f64.to_radians(),
            2.0_f64.to_radians(),
        );
        assert!(shape_c_full.area() > 0.0, "ShapeC_full should have positive area");

        let shape_l_full = ShapeL::new(
            120.0,
            100.0,
            12.0,
            10.0,
            6.0,
            3.0,
            2.0,
            2.5_f64.to_radians(),
        );
        assert!(shape_l_full.area() > 0.0, "ShapeL_full should have positive area");

        let shape_t_full = ShapeT::new(
            160.0,
            140.0,
            16.0,
            10.0,
            10.0,
            4.0,
            1.5_f64.to_radians(),
        );
        assert!(shape_t_full.area() > 0.0, "ShapeT_full should have positive area");
}
