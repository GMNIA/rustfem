use std::f64::consts::{PI, TAU};

use nalgebra::Matrix3;

use crate::polygon::Polygon as RawPolygon;
use crate::{epsilon, Vector3d};

/// Common interface shared by all cross-sectional shapes.
///
/// Shapes are thin planar regions that expose their area, centroid, and inertia
/// without mandating a particular underlying representation. Polygonal shapes
/// may delegate to a cached polygon, while analytic shapes (e.g. disks) can
/// provide closed-form results.
pub trait Shape {
    /// Planar area of the shape.
    fn area(&self) -> f64;

    /// Perimeter length of the shape.
    fn perimeter(&self) -> f64;

    /// Centroid expressed in global coordinates.
    fn centroid(&self) -> Vector3d;

    /// 3D inertia tensor of the shape about its centroid.
    fn second_moment_of_area(&self) -> Matrix3<f64>;

    /// Polygonal approximation of the boundary (useful for meshing or tests).
    fn linearized(&self, sides: usize) -> RawPolygon<Vector3d>;

    /// Circumference alias for shapes where that terminology is preferred.
    fn circumference(&self) -> f64 { self.perimeter() }
}

/// Helper: creates an axis-aligned rectangle centred at the origin.
fn rectangle_polygon(width: f64, height: f64) -> RawPolygon<Vector3d> {
    let hw = width / 2.0;
    let hh = height / 2.0;
    let verts = vec![
        Vector3d::new(-hw, -hh, 0.0),
        Vector3d::new(hw, -hh, 0.0),
        Vector3d::new(hw, hh, 0.0),
        Vector3d::new(-hw, hh, 0.0),
    ];
    RawPolygon::new(verts)
}

/// Helper: builds a regular N-gon approximation for a circle centred at the origin.
fn regular_ngon(radius: f64, sides: usize) -> RawPolygon<Vector3d> {
    assert!(radius > 0.0, "radius must be positive");
    assert!(sides >= 3, "need at least three sides to form a polygon");
    let step = TAU / sides as f64;
    let verts = (0..sides)
        .map(|i| {
            let angle = i as f64 * step;
            Vector3d::new(radius * angle.cos(), radius * angle.sin(), 0.0)
        })
        .collect::<Vec<_>>();
    RawPolygon::new(verts)
}

macro_rules! impl_polygon_shape {
    ($type:ty) => {
        impl $type {
            pub fn to_polygon(&self) -> RawPolygon<Vector3d> {
                self.polygon.clone()
            }
        }

        impl Shape for $type {
            fn area(&self) -> f64 { self.polygon.area() }
            fn perimeter(&self) -> f64 { self.polygon.perimeter() }
            fn centroid(&self) -> Vector3d { self.polygon.centroid() }
            fn second_moment_of_area(&self) -> Matrix3<f64> {
                self.polygon.second_moment_of_area()
            }
            fn linearized(&self, _sides: usize) -> RawPolygon<Vector3d> {
                self.polygon.clone()
            }
        }
    };
}

/// Solid rectangle defined by width and height. Optional hole dimensions are
/// retained for future development but currently ignored in the polygon.
#[derive(Debug, Clone)]
pub struct Rectangle {
    pub width: f64,
    pub height: f64,
    pub hole_width: f64,
    pub hole_height: f64,
    polygon: RawPolygon<Vector3d>,
}

impl Rectangle {
    pub fn new(width: f64, height: f64, hole_width: f64, hole_height: f64) -> Self {
        assert!(width > epsilon() && height > epsilon(), "rectangle dimensions must be positive");
        let polygon = rectangle_polygon(width, height);
        Self { width, height, hole_width, hole_height, polygon }
    }
}

impl_polygon_shape!(Rectangle);

/// Disk (solid circle) optionally with a concentric hole.
#[derive(Debug, Clone)]
pub struct Disk {
    pub radius: f64,
    pub hole_radius: f64,
}

impl Disk {
    const DEFAULT_LINEARIZATION_SIDES: usize = 256;

    pub fn new(radius: f64, hole_radius: f64) -> Self {
        assert!(radius > hole_radius, "outer radius must exceed hole radius");
        Self { radius, hole_radius }
    }

    pub fn circumference(&self) -> f64 { TAU * self.radius }

    fn solid_area(&self) -> f64 {
        let outer = self.radius * self.radius;
        let inner = self.hole_radius * self.hole_radius;
        PI * (outer - inner)
    }

    fn planar_inertia(&self) -> f64 {
        let outer = self.radius.powi(4);
        let inner = self.hole_radius.powi(4);
        PI * (outer - inner) / 4.0
    }
}

impl Shape for Disk {
    fn area(&self) -> f64 { self.solid_area() }

    fn perimeter(&self) -> f64 { self.circumference() }

    fn centroid(&self) -> Vector3d { Vector3d::new(0.0, 0.0, 0.0) }

    fn second_moment_of_area(&self) -> Matrix3<f64> {
        let ix = self.planar_inertia();
        let iy = ix;
        let iz = ix + iy;
        Matrix3::from_diagonal(&nalgebra::Vector3::new(ix, iy, iz))
    }

    fn linearized(&self, sides: usize) -> RawPolygon<Vector3d> {
        let sides = sides.max(Self::DEFAULT_LINEARIZATION_SIDES);
        regular_ngon(self.radius, sides)
    }
}

/// Doubly-symmetric I profile.
#[derive(Debug, Clone)]
pub struct ShapeI {
    pub bottom_width: f64,
    pub top_width: f64,
    pub height: f64,
    pub bottom_thickness: f64,
    pub top_thickness: f64,
    pub web_thickness: f64,
    pub fillet: f64,
    polygon: RawPolygon<Vector3d>,
}

impl ShapeI {
    pub fn new(
        bottom_width: f64,
        top_width: f64,
        height: f64,
        bottom_thickness: f64,
        top_thickness: f64,
        web_thickness: f64,
        fillet: f64,
    ) -> Self {
        assert!(height > bottom_thickness + top_thickness, "height must exceed flange thickness");
        let hw = height / 2.0;
        let bottom_half = bottom_width / 2.0;
        let top_half = top_width / 2.0;
        let web_half = web_thickness / 2.0;

        let verts = vec![
            Vector3d::new(-bottom_half, -hw, 0.0),
            Vector3d::new(bottom_half, -hw, 0.0),
            Vector3d::new(bottom_half, -hw + bottom_thickness, 0.0),
            Vector3d::new(web_half, -hw + bottom_thickness, 0.0),
            Vector3d::new(web_half, hw - top_thickness, 0.0),
            Vector3d::new(top_half, hw - top_thickness, 0.0),
            Vector3d::new(top_half, hw, 0.0),
            Vector3d::new(-top_half, hw, 0.0),
            Vector3d::new(-top_half, hw - top_thickness, 0.0),
            Vector3d::new(-web_half, hw - top_thickness, 0.0),
            Vector3d::new(-web_half, -hw + bottom_thickness, 0.0),
            Vector3d::new(-bottom_half, -hw + bottom_thickness, 0.0),
        ];

        let polygon = RawPolygon::new(verts);
        Self {
            bottom_width,
            top_width,
            height,
            bottom_thickness,
            top_thickness,
            web_thickness,
            fillet,
            polygon,
        }
    }
}

impl_polygon_shape!(ShapeI);

/// Channel (C) section.
#[derive(Debug, Clone)]
pub struct ShapeC {
    pub bottom_width: f64,
    pub top_width: f64,
    pub height: f64,
    pub bottom_thickness: f64,
    pub top_thickness: f64,
    pub web_thickness: f64,
    pub fillet: f64,
    polygon: RawPolygon<Vector3d>,
}

impl ShapeC {
    pub fn new(
        bottom_width: f64,
        top_width: f64,
        height: f64,
        bottom_thickness: f64,
        top_thickness: f64,
        web_thickness: f64,
        fillet: f64,
    ) -> Self {
        assert!(height > bottom_thickness + top_thickness, "height must exceed flange thickness");
        let half_h = height / 2.0;
        let verts = vec![
            Vector3d::new(0.0, -half_h, 0.0),
            Vector3d::new(bottom_width, -half_h, 0.0),
            Vector3d::new(bottom_width, -half_h + bottom_thickness, 0.0),
            Vector3d::new(web_thickness, -half_h + bottom_thickness, 0.0),
            Vector3d::new(web_thickness, half_h - top_thickness, 0.0),
            Vector3d::new(top_width, half_h - top_thickness, 0.0),
            Vector3d::new(top_width, half_h, 0.0),
            Vector3d::new(0.0, half_h, 0.0),
            Vector3d::new(0.0, half_h - top_thickness, 0.0),
            Vector3d::new(0.0, -half_h + bottom_thickness, 0.0),
        ];
        let polygon = RawPolygon::new(verts);
        Self {
            bottom_width,
            top_width,
            height,
            bottom_thickness,
            top_thickness,
            web_thickness,
            fillet,
            polygon,
        }
    }
}

impl_polygon_shape!(ShapeC);

/// Angle (L) section.
#[derive(Debug, Clone)]
pub struct ShapeL {
    pub width: f64,
    pub height: f64,
    pub flange_thickness: f64,
    pub web_thickness: f64,
    pub fillet: f64,
    polygon: RawPolygon<Vector3d>,
}

impl ShapeL {
    pub fn new(
        width: f64,
        height: f64,
        flange_thickness: f64,
        web_thickness: f64,
        fillet: f64,
    ) -> Self {
        assert!(width > web_thickness && height > flange_thickness, "invalid L-section dimensions");
        let verts = vec![
            Vector3d::new(0.0, 0.0, 0.0),
            Vector3d::new(width, 0.0, 0.0),
            Vector3d::new(width, flange_thickness, 0.0),
            Vector3d::new(web_thickness, flange_thickness, 0.0),
            Vector3d::new(web_thickness, height, 0.0),
            Vector3d::new(0.0, height, 0.0),
        ];
        let polygon = RawPolygon::new(verts);
        Self { width, height, flange_thickness, web_thickness, fillet, polygon }
    }
}

impl_polygon_shape!(ShapeL);

/// Tee (T) section.
#[derive(Debug, Clone)]
pub struct ShapeT {
    pub width: f64,
    pub height: f64,
    pub flange_thickness: f64,
    pub web_thickness: f64,
    pub fillet: f64,
    polygon: RawPolygon<Vector3d>,
}

impl ShapeT {
    pub fn new(
        width: f64,
        height: f64,
        flange_thickness: f64,
        web_thickness: f64,
        fillet: f64,
    ) -> Self {
        assert!(height > flange_thickness, "height must exceed flange thickness");
        let half_h = height / 2.0;
        let half_w = width / 2.0;
        let web_half = web_thickness / 2.0;
        let verts = vec![
            Vector3d::new(-half_w, half_h, 0.0),
            Vector3d::new(half_w, half_h, 0.0),
            Vector3d::new(half_w, half_h - flange_thickness, 0.0),
            Vector3d::new(web_half, half_h - flange_thickness, 0.0),
            Vector3d::new(web_half, -half_h, 0.0),
            Vector3d::new(-web_half, -half_h, 0.0),
            Vector3d::new(-web_half, half_h - flange_thickness, 0.0),
            Vector3d::new(-half_w, half_h - flange_thickness, 0.0),
        ];
        let polygon = RawPolygon::new(verts);
        Self { width, height, flange_thickness, web_thickness, fillet, polygon }
    }
}

impl_polygon_shape!(ShapeT);

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::{assert_almost_eq, assert_vec3_almost_eq, epsilon, DEFAULT_EPSILON};
    use crate::precision::with_epsilon;

    use super::*;

    fn relative_error(expected: f64, calculated: f64) -> f64 {
        if calculated == 0.0 {
            return expected.abs();
        }
        (expected / calculated - 1.0).abs()
    }

    fn assert_relative_eq(label: &str, expected: f64, calculated: f64) {
        assert_relative_eq_with(label, expected, calculated, epsilon());
    }

    fn assert_relative_eq_with(label: &str, expected: f64, calculated: f64, tol: f64) {
        let err = relative_error(expected, calculated);
        assert!(
            err < tol,
            "{label} relative error {err} exceeds tolerance {tol} (expected={expected}, actual={calculated})"
        );
    }

    #[test]
    fn rectangle_area_matches_dimensions() {
        with_epsilon(DEFAULT_EPSILON, || {
            let rect = Rectangle::new(0.3, 0.2, 0.0, 0.0);
            assert_relative_eq("rectangle area", 0.06, rect.area());
            let poly = rect.to_polygon();
            assert_eq!(poly.vertices().len(), 4);
        });
    }

    #[test]
    fn disk_area_close_to_circle() {
        with_epsilon(DEFAULT_EPSILON, || {
            let disk = Disk::new(0.15, 0.0);
            let area = disk.area();
            let expected = PI * 0.15 * 0.15;
            assert_relative_eq("disk area", expected, area);
        });
    }

    #[test]
    fn shape_i_area_matches_components() {
        with_epsilon(DEFAULT_EPSILON, || {
            let shape = ShapeI::new(0.18, 0.18, 0.3, 0.02, 0.02, 0.01, 0.0);
            assert_relative_eq("shape I area", 0.009799999999999986, shape.area());
        });
    }

    #[test]
    fn shape_c_area_matches_components() {
        with_epsilon(DEFAULT_EPSILON, || {
            let shape = ShapeC::new(0.12, 0.08, 0.25, 0.015, 0.012, 0.008, 0.0);
            assert_relative_eq("shape C area", 0.004544000000000003, shape.area());
        });
    }

    #[test]
    fn shape_l_area_matches_components() {
        with_epsilon(DEFAULT_EPSILON, || {
            let shape = ShapeL::new(0.1, 0.12, 0.02, 0.015, 0.0);
            assert_relative_eq("shape L area", 0.0035000000000000005, shape.area());
        });
    }

    #[test]
    fn shape_t_area_matches_components() {
        with_epsilon(DEFAULT_EPSILON, || {
            let shape = ShapeT::new(0.14, 0.28, 0.02, 0.01, 0.0);
            assert_relative_eq("shape T area", 0.005400000000000002, shape.area());
        });
    }

    #[test]
    fn shapes_linearized_match_area() {
        let shapes: Vec<(&str, f64, Box<dyn Shape>)> = vec![
            ("Rectangle", 0.020000000000000004, Box::new(Rectangle::new(0.2, 0.1, 0.0, 0.0))),
            ("Disk", 0.031415926535897934, Box::new(Disk::new(0.1, 0.0))),
            ("ShapeI", 0.008208000000000003, Box::new(ShapeI::new(0.16, 0.16, 0.24, 0.018, 0.018, 0.012, 0.0))),
            (
                "ShapeC",
                0.004100000000000004,
                Box::new(ShapeC::new(0.1, 0.08, 0.2, 0.014, 0.012, 0.01, 0.0)),
            ),
            ("ShapeL", 0.0021000000000000003, Box::new(ShapeL::new(0.08, 0.09, 0.015, 0.012, 0.0))),
            ("ShapeT", 0.004180000000000003, Box::new(ShapeT::new(0.12, 0.22, 0.018, 0.01, 0.0))),
        ];
        for (label, expected_area, shape) in shapes {
            assert_relative_eq(&format!("{label} area"), expected_area, shape.area());
            let polygon = shape.linearized(1024);
            assert_relative_eq_with(
                &format!("{label} linearized area"),
                shape.area(),
                polygon.area(),
                1e-4,
            );
        }
    }

    #[test]
    fn rectangle_simple_matches_reference_snapshot() {
        with_epsilon(DEFAULT_EPSILON, || {
            let rect = Rectangle::new(200.0, 100.0, 0.0, 0.0);
            let poly = rect.to_polygon();

            assert_relative_eq("rectangle area", 20000.0, poly.area());
            assert_vec3_almost_eq!(poly.centroid(), Vector3d::new(0.0, 0.0, 0.0));

            let local = poly.local_second_moment_of_area();
            assert_relative_eq("rectangle I_xx local", 16666666.666666666, local[(0, 0)]);
            assert_relative_eq("rectangle I_yy local", 66666666.666666664, local[(1, 1)]);
            assert_almost_eq!(local[(0, 1)], 0.0);
            let centroidal_local = poly.centroidal_local_second_moment_of_area();
            assert_relative_eq("rectangle I_xx centroidal local", 16666666.666666666, centroidal_local[(0, 0)]);
            assert_relative_eq("rectangle I_yy centroidal local", 66666666.666666664, centroidal_local[(1, 1)]);
            assert_almost_eq!(centroidal_local[(0, 1)], 0.0);

            let principal = poly.local_principal_axes();
            assert_relative_eq("rectangle principal (0,0)", 1.0, principal[(0, 0)]);
            assert_relative_eq("rectangle principal (0,1)", 0.0, principal[(0, 1)]);
            assert_relative_eq("rectangle principal (1,0)", 0.0, principal[(1, 0)]);
            assert_relative_eq("rectangle principal (1,1)", 1.0, principal[(1, 1)]);

            let global = poly.second_moment_of_area();
            assert_relative_eq("rectangle I_xx global", 16666666.666666666, global[(0, 0)]);
            assert_relative_eq("rectangle I_yy global", 66666666.666666664, global[(1, 1)]);
            assert_relative_eq("rectangle I_zz global", 83333333.33333333, global[(2, 2)]);
            assert_almost_eq!(global[(0, 1)], 0.0);
            assert_almost_eq!(global[(0, 2)], 0.0);
            assert_almost_eq!(global[(1, 2)], 0.0);
            let centroidal_global = poly.centroidal_second_moment_of_area();
            assert_relative_eq("rectangle I_xx centroidal global", 16666666.666666666, centroidal_global[(0, 0)]);
            assert_relative_eq("rectangle I_yy centroidal global", 66666666.666666664, centroidal_global[(1, 1)]);
            assert_relative_eq("rectangle I_zz centroidal global", 83333333.33333333, centroidal_global[(2, 2)]);
        });
    }

    #[test]
    fn shape_i_simple_matches_reference_snapshot() {
        with_epsilon(DEFAULT_EPSILON, || {
            let shape = ShapeI::new(120.0, 120.0, 300.0, 12.0, 12.0, 8.0, 0.0);
            let poly = shape.to_polygon();

            assert_relative_eq("shape I area", 5088.0, poly.area());
            assert_vec3_almost_eq!(poly.centroid(), Vector3d::new(0.0, 0.0, 0.0));

            let local = poly.local_second_moment_of_area();
            assert_relative_eq("shape I I_xx local", 73770624.0, local[(0, 0)]);
            assert_relative_eq("shape I I_yy local", 3467776.0, local[(1, 1)]);
            assert_almost_eq!(local[(0, 1)], 0.0);
            let centroidal_local = poly.centroidal_local_second_moment_of_area();
            assert_relative_eq("shape I I_xx centroidal local", 73770624.0, centroidal_local[(0, 0)]);
            assert_relative_eq("shape I I_yy centroidal local", 3467776.0, centroidal_local[(1, 1)]);
            assert_almost_eq!(centroidal_local[(0, 1)], 0.0);

            let principal = poly.local_principal_axes();
            assert_relative_eq("shape I principal (0,0)", 0.0, principal[(0, 0)]);
            assert_relative_eq("shape I principal (0,1)", 1.0, principal[(0, 1)]);
            assert_relative_eq("shape I principal (1,0)", 1.0, principal[(1, 0)]);
            assert_relative_eq("shape I principal (1,1)", 0.0, principal[(1, 1)]);

            let global = poly.second_moment_of_area();
            assert_relative_eq("shape I I_xx global", 73770624.0, global[(0, 0)]);
            assert_relative_eq("shape I I_yy global", 3467776.0, global[(1, 1)]);
            assert_relative_eq("shape I I_zz global", 77238400.0, global[(2, 2)]);
            let centroidal_global = poly.centroidal_second_moment_of_area();
            assert_relative_eq("shape I I_xx centroidal global", 73770624.0, centroidal_global[(0, 0)]);
            assert_relative_eq("shape I I_yy centroidal global", 3467776.0, centroidal_global[(1, 1)]);
            assert_relative_eq("shape I I_zz centroidal global", 77238400.0, centroidal_global[(2, 2)]);
        });
    }

    #[test]
    fn shape_t_simple_matches_reference_snapshot() {
        with_epsilon(DEFAULT_EPSILON, || {
            let shape = ShapeT::new(120.0, 100.0, 12.0, 8.0, 0.0);
            let poly = shape.to_polygon();

            assert_relative_eq("shape T area", 2144.0, poly.area());
            assert_vec3_almost_eq!(poly.centroid(), Vector3d::new(0.0, 27.582089552238806, 0.0));

            let local = poly.local_second_moment_of_area();
            assert_relative_eq("shape T I_xx local", 3284778.6666666665, local[(0, 0)]);
            assert_relative_eq("shape T I_yy local", 1731754.6666666667, local[(1, 1)]);
            assert_almost_eq!(local[(0, 1)], 0.0);
            let centroidal_local = poly.centroidal_local_second_moment_of_area();
            assert_relative_eq("shape T I_xx centroidal local", 1653684.2189055, centroidal_local[(0, 0)]);
            assert_relative_eq("shape T I_yy centroidal local", 1731754.6666666667, centroidal_local[(1, 1)]);
            assert_almost_eq!(centroidal_local[(0, 1)], 0.0);

            let principal = poly.local_principal_axes();
            assert_relative_eq("shape T principal (0,0)", 1.0, principal[(0, 0)]);
            assert_relative_eq("shape T principal (0,1)", 0.0, principal[(0, 1)]);
            assert_relative_eq("shape T principal (1,0)", 0.0, principal[(1, 0)]);
            assert_relative_eq("shape T principal (1,1)", 1.0, principal[(1, 1)]);

            let global = poly.second_moment_of_area();
            assert_relative_eq("shape T I_xx global", 3284778.6666666665, global[(0, 0)]);
            assert_relative_eq("shape T I_yy global", 1731754.6666666667, global[(1, 1)]);
            assert_relative_eq("shape T I_zz global", 5016533.333333333, global[(2, 2)]);
            let centroidal_global = poly.centroidal_second_moment_of_area();
            assert_relative_eq("shape T I_xx centroidal global", 1653684.2189055, centroidal_global[(0, 0)]);
            assert_relative_eq("shape T I_yy centroidal global", 1731754.6666666667, centroidal_global[(1, 1)]);
            assert_relative_eq("shape T I_zz centroidal global", 3385438.885572, centroidal_global[(2, 2)]);
        });
    }
}
