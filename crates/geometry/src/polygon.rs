use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};

use crate::arc::ArcVector;
use crate::line::{Axis, Line, LocalAxis};
use crate::{epsilon, Vector3d};
#[cfg(test)]
use crate::Vector2d;

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon<V>
where
    V: ArcVector,
{
    vertices: Vec<V>,
    // Per-polygon cached geometric properties
    normal: Vector3<f64>,
    rotation: Matrix3<f64>, // columns are [ex, ey, ez]
    centroid: V,
    area: f64,
    perimeter: f64,
}

// Local 2D/3D aliases removed; the crate root exports canonical 3D names.

impl<V> Polygon<V>
where
    V: ArcVector,
{
    /// Create a polygon from vertices (closed implicitly). At least three vertices are required.
    /// Accepts either 2D or 3D vectors; 2D inputs are promoted with z = 0.
    pub fn new<I, P>(vertices: I) -> Self
    where
        I: IntoIterator<Item = P>,
        P: Into<V>,
    {
        let mut verts: Vec<V> = vertices.into_iter().map(|p| p.into()).collect();
        assert!(verts.len() >= 3, "Polygon requires at least 3 vertices");

        // Remove consecutive duplicates
        verts.dedup_by(|a, b| a.is_approx(b, Some(epsilon())));
        assert!(verts.len() >= 3, "Polygon requires at least 3 distinct vertices");

        // Establish plane from first non-collinear triple (prefer the first three if possible)
        let (p0, normal) = {
            let mut base = verts[0].to_vec3();
            let mut normal = Vector3::new(0.0, 0.0, 0.0);
            'outer: for i in 0..verts.len() {
                for j in i + 1..verts.len() {
                    for k in j + 1..verts.len() {
                        let a = verts[i].to_vec3();
                        let b = verts[j].to_vec3();
                        let c = verts[k].to_vec3();
                        let u = b - a;
                        let v = c - a;
                        let n = u.cross(&v);
                        if n.norm() > epsilon() {
                            base = a;
                            normal = n.normalize();
                            break 'outer;
                        }
                    }
                }
            }
            if normal.norm() <= epsilon() {
                // All points collinear or identical; fallback to +Z
                (base, Vector3::new(0.0, 0.0, 1.0))
            } else {
                (base, normal)
            }
        };

        // Project all vertices onto the plane defined by (p0, normal)
        let verts: Vec<V> = verts
            .into_iter()
            .map(|v| {
                let pv = v.to_vec3();
                let proj = pv - normal * (pv - p0).dot(&normal);
                V::from_vec3(proj)
            })
            .collect();

        // Build local frame: ez = normal; ex = first edge projected to plane; ey = ez × ex
    let ez = normal;
    let ex = {
            // Find first non-degenerate edge
            let mut ex_opt: Option<Vector3<f64>> = None;
            for i in 0..verts.len() {
                let a = verts[i].to_vec3();
                let b = verts[(i + 1) % verts.len()].to_vec3();
                let edge = b - a;
                if edge.norm() > epsilon() {
                    ex_opt = Some(edge);
                    break;
                }
            }
            let mut ex = ex_opt.unwrap_or_else(|| Vector3::new(1.0, 0.0, 0.0));
            // Project onto plane to ensure orthogonality with ez
            ex = ex - ez * ex.dot(&ez);
            if ex.norm() <= epsilon() {
                // choose a stable axis perpendicular to ez
                let global_y = Vector3::new(0.0, 1.0, 0.0);
                let mut ex_raw = ez.cross(&global_y);
                if ex_raw.norm() <= epsilon() {
                    ex_raw = Vector3::new(1.0, 0.0, 0.0);
                }
                ex = ex_raw.normalize();
            } else {
                ex = ex.normalize();
            }
            ex
        };
        let ey = ez.cross(&ex);
        let rotation = Matrix3::from_columns(&[ex, ey, ez]);

    // Compute area, perimeter, and centroid in local coordinates (z ~ 0)
        let (area, centroid_local, perimeter) = {
            let mut area2 = 0.0; // 2 * area
            let mut cx_num = 0.0;
            let mut cy_num = 0.0;
            let mut perim = 0.0;

            // Transform vertices to local frame using temporary origin at first vertex
            let origin0 = verts[0].to_vec3();
            let r_t = rotation.transpose();
            let local: Vec<Vector3<f64>> = verts
                .iter()
                .map(|v| r_t * (v.to_vec3() - origin0))
                .collect();

            for i in 0..local.len() {
                let p = local[i];
                let q = local[(i + 1) % local.len()];
                let cross = p.x * q.y - q.x * p.y;
                area2 += cross;
                cx_num += (p.x + q.x) * cross;
                cy_num += (p.y + q.y) * cross;
                perim += (q - p).norm();
            }
            let area = 0.5 * area2;
            let (cx, cy) = if area.abs() > epsilon() {
                (cx_num / (3.0 * area2), cy_num / (3.0 * area2))
            } else {
                // Fallback: average of vertices
                let n = local.len() as f64;
                let sx = local.iter().map(|p| p.x).sum::<f64>() / n;
                let sy = local.iter().map(|p| p.y).sum::<f64>() / n;
                (sx, sy)
            };
            let centroid_local = Vector3::new(cx, cy, 0.0);
            (area, centroid_local, perim)
        };

        // Global centroid from local
        let centroid_vec = verts[0].to_vec3() + rotation * centroid_local;
        let centroid = V::from_vec3(centroid_vec);

        Self { vertices: verts, normal: ez, rotation, centroid, area, perimeter }
    }

    pub fn vertices(&self) -> &Vec<V> { &self.vertices }

    pub fn lines(&self) -> Vec<Line<V>> {
        let n = self.vertices.len();
        (0..n)
            .map(|i| Line::new(self.vertices[i], self.vertices[(i + 1) % n]))
            .collect()
    }

    pub fn area(&self) -> f64 { self.area.abs() }
    pub fn perimeter(&self) -> f64 { self.perimeter }
    pub fn centroid(&self) -> V { self.centroid }

    /// Center as used in reference outputs: the first input vertex.
    pub fn center(&self) -> V { self.vertices[0] }

    pub fn bounding_box(&self) -> (V, V) {
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];
        for v in &self.vertices {
            min = min.component_min(v);
            max = max.component_max(v);
        }
        (min, max)
    }

    pub fn local_axis(&self) -> LocalAxis {
        // Use centroid as origin
        LocalAxis::new(Vector3d(self.centroid.to_vec3()), self.rotation)
    }

    pub fn axis(&self, axis: Axis) -> Vector3d {
        match axis {
            Axis::AxisX => Vector3d::new(self.rotation.column(0)[0], self.rotation.column(0)[1], self.rotation.column(0)[2]),
            Axis::AxisY => Vector3d::new(self.rotation.column(1)[0], self.rotation.column(1)[1], self.rotation.column(1)[2]),
            Axis::AxisZ => Vector3d::new(self.rotation.column(2)[0], self.rotation.column(2)[1], self.rotation.column(2)[2]),
        }
    }

    /// Alias to match the external API style (Geometry::direction(Axis))
    pub fn direction(&self, axis: Axis) -> Vector3d { self.axis(axis) }

    pub fn to_local(&self, point: Vector3d) -> Vector3d {
        let offset = point.0 - self.centroid.to_vec3();
        let local = self.rotation.transpose() * offset;
        Vector3d::new(local.x, local.y, local.z)
    }

    pub fn to_global(&self, local: Vector3d) -> Vector3d {
        let global = self.centroid.to_vec3() + self.rotation * local.0;
        Vector3d::new(global.x, global.y, global.z)
    }

    /// Compute the 2D second moment of area matrix in the polygon's local plane
    /// about the centroid. Returns a 2x2 matrix [Ixx Ixy; Ixy Iyy] where axes are
    /// the polygon's local X,Y (columns of `self.rotation`).
    pub fn local_second_moment_of_area(&self) -> Matrix2<f64> {
        // Build local coordinates with an arbitrary origin (first vertex), then use
        // parallel axis theorem to shift to centroid.
        let r_t = self.rotation.transpose();
        let origin0 = self.vertices[0].to_vec3();
        let locals: Vec<Vector3<f64>> = self
            .vertices
            .iter()
            .map(|v| r_t * (v.to_vec3() - origin0))
            .collect();

        // Compute area and centroid in this local frame (reuse algorithm)
        let mut area2 = 0.0; // 2*A
        let mut cx_num = 0.0;
        let mut cy_num = 0.0;

        let mut ix0_sum = 0.0; // about origin0
        let mut iy0_sum = 0.0;
        let mut ixy0_sum = 0.0;

        for i in 0..locals.len() {
            let p = locals[i];
            let q = locals[(i + 1) % locals.len()];
            let cross = p.x * q.y - q.x * p.y; // xi*yj - xj*yi
            area2 += cross;
            cx_num += (p.x + q.x) * cross;
            cy_num += (p.y + q.y) * cross;

            // Origin-based second moments for polygon
            let yy = p.y * p.y + p.y * q.y + q.y * q.y;
            let xx = p.x * p.x + p.x * q.x + q.x * q.x;
            let xy = p.x * q.y + 2.0 * p.x * p.y + 2.0 * q.x * q.y + q.x * p.y;
            ix0_sum += yy * cross; // Ixx about origin (integral of y^2)
            iy0_sum += xx * cross; // Iyy about origin (integral of x^2)
            ixy0_sum += xy * cross; // Ixy about origin (integral of x*y)
        }

        let area = 0.5 * area2;
        // Guard against degenerate area
        if area.abs() <= epsilon() {
            return Matrix2::zeros();
        }

        let cx = cx_num / (3.0 * area2);
        let cy = cy_num / (3.0 * area2);

        // Scale sums to get moments about origin
        let ix0 = ix0_sum / 12.0;
        let iy0 = iy0_sum / 12.0;
        let ixy0 = ixy0_sum / 24.0;

        // Central (about centroid) using parallel axis theorem
        let ixx_c = ix0 - area * cy * cy;
        let iyy_c = iy0 - area * cx * cx;
        let ixy_c = ixy0 - area * cx * cy;

        Matrix2::new(ixx_c, ixy_c, ixy_c, iyy_c)
    }

    /// Local principal axes in the polygon plane as a 2x2 orthonormal matrix whose
    /// columns are eigenvectors of the local second moment matrix.
    pub fn local_principal_axes(&self) -> Matrix2<f64> {
        let s = self.local_second_moment_of_area();
        // Symmetric 2x2 -> closed-form eigenvectors
        let a = s[(0, 0)];
        let b = s[(0, 1)];
        let c = s[(1, 1)];
        // Compute eigenvalues
        let tr = a + c;
        let det = a * c - b * b;
        let disc = (tr * tr - 4.0 * det).max(0.0).sqrt();
    let l1 = 0.5 * (tr + disc);

        // Eigenvector for l1: (b, l1 - a) unless near-zero
        let mut v1 = if b.abs() > epsilon() || (l1 - a).abs() > epsilon() {
            Vector2::new(b, l1 - a)
        } else {
            // If nearly diagonal, choose axis with larger inertia as primary
            if a >= c { Vector2::new(1.0, 0.0) } else { Vector2::new(0.0, 1.0) }
        };
        if v1.norm() <= epsilon() {
            v1 = Vector2::new(1.0, 0.0);
        }
        v1 /= v1.norm();
        // v2 orthogonal to v1
        let v2 = Vector2::new(-v1.y, v1.x);
        Matrix2::from_columns(&[v1, v2])
    }

    /// Global 3D second moment of area tensor at the centroid. Embeds the local 2D
    /// tensor into 3D plate inertia form and rotates to global frame.
    pub fn second_moment_of_area(&self) -> Matrix3<f64> {
        let s2 = self.local_second_moment_of_area();
        let ixx = s2[(0, 0)];
        let iyy = s2[(1, 1)];
        let ixy = s2[(0, 1)];
        // Inertia tensor for a thin plate in its plane
        let mut j_local = Matrix3::zeros();
        j_local[(0, 0)] = ixx;
        j_local[(1, 1)] = iyy;
        j_local[(0, 1)] = -ixy;
        j_local[(1, 0)] = -ixy;
        j_local[(2, 2)] = ixx + iyy; // polar about normal
        // Rotate to global
        self.rotation * j_local * self.rotation.transpose()
    }

    /// Global 3D second moment of area tensor computed about the polygon "center"
    /// which we define as the first vertex (matching reference prints). This uses
    /// the local coordinates built with origin at the first vertex without shifting
    /// to the centroid.
    pub fn second_moment_of_area_at_center(&self) -> Matrix3<f64> {
        let r_t = self.rotation.transpose();
        let origin0 = self.vertices[0].to_vec3();
        let locals: Vec<Vector3<f64>> = self
            .vertices
            .iter()
            .map(|v| r_t * (v.to_vec3() - origin0))
            .collect();

        let mut ix0_sum = 0.0; // about origin0
        let mut iy0_sum = 0.0;
        let mut ixy0_sum = 0.0;

        for i in 0..locals.len() {
            let p = locals[i];
            let q = locals[(i + 1) % locals.len()];
            let cross = p.x * q.y - q.x * p.y;
            let yy = p.y * p.y + p.y * q.y + q.y * q.y;
            let xx = p.x * p.x + p.x * q.x + q.x * q.x;
            let xy = p.x * q.y + 2.0 * p.x * p.y + 2.0 * q.x * q.y + q.x * p.y;
            ix0_sum += yy * cross; // Ixx about origin (integral of y^2)
            iy0_sum += xx * cross; // Iyy about origin (integral of x^2)
            ixy0_sum += xy * cross; // Ixy about origin (integral of x*y)
        }

        let ixx0 = ix0_sum / 12.0;
        let iyy0 = iy0_sum / 12.0;
        let ixy0 = ixy0_sum / 24.0;

        let mut j_local = Matrix3::zeros();
        j_local[(0, 0)] = ixx0;
        j_local[(1, 1)] = iyy0;
        j_local[(0, 1)] = -ixy0;
        j_local[(1, 0)] = -ixy0;
        j_local[(2, 2)] = ixx0 + iyy0;
        self.rotation * j_local * self.rotation.transpose()
    }

    /// Global 3D principal axes as a 3x3 rotation matrix whose columns are the principal
    /// directions (two in-plane, one along the polygon normal).
    pub fn principal_axes(&self) -> Matrix3<f64> {
        let p2 = self.local_principal_axes();
        let u1_local = Vector3::new(p2[(0, 0)], p2[(1, 0)], 0.0);
        let u2_local = Vector3::new(p2[(0, 1)], p2[(1, 1)], 0.0);
        let u3_local = Vector3::new(0.0, 0.0, 1.0);
        let u1_global = self.rotation * u1_local;
        let u2_global = self.rotation * u2_local;
        let u3_global = self.rotation * u3_local;
        Matrix3::from_columns(&[u1_global, u2_global, u3_global])
    }

    pub fn contains(&self, point: &V) -> bool {
        // Project to local; ensure on plane; then 2D point-in-polygon (ray crossing)
        let p_local = self.to_local(Vector3d(point.to_vec3()));
        if p_local.z().abs() > epsilon() {
            return false;
        }
        let r_t = self.rotation.transpose();
        let origin = self.centroid.to_vec3();
        let locals: Vec<Vector3<f64>> = self
            .vertices
            .iter()
            .map(|v| r_t * (v.to_vec3() - origin))
            .collect();
        // Ray cast along +X
        let mut inside = false;
        for i in 0..locals.len() {
            let a = locals[i];
            let b = locals[(i + 1) % locals.len()];
            let yi = a.y;
            let yj = b.y;
            let xi = a.x;
            let xj = b.x;
            let intersects = ((yi > p_local.y() && yj <= p_local.y()) || (yj > p_local.y() && yi <= p_local.y()))
                && (xi + (p_local.y() - yi) * (xj - xi) / (yj - yi + 1e-30) > p_local.x());
            if intersects {
                inside = !inside;
            }
        }
        inside
    }

    pub fn border_contains(&self, point: &V) -> bool {
        let p_local = self.to_local(Vector3d(point.to_vec3()));
        if p_local.z().abs() > epsilon() {
            return false;
        }
        let r_t = self.rotation.transpose();
        let origin = self.centroid.to_vec3();
        let locals: Vec<Vector3<f64>> = self
            .vertices
            .iter()
            .map(|v| r_t * (v.to_vec3() - origin))
            .collect();

        for i in 0..locals.len() {
            let a = locals[i];
            let b = locals[(i + 1) % locals.len()];
            if point_on_segment_2d(p_local.0, a, b) {
                return true;
            }
        }
        false
    }

    pub fn closest_point(&self, point: &V) -> V {
        // Project to plane
        let p = point.to_vec3();
        let n = self.normal;
        let c = self.centroid.to_vec3();
        let to_point = p - c;
        let p_proj = p - n * to_point.dot(&n);
        let p_proj_v = V::from_vec3(p_proj);
        if self.contains(&p_proj_v) || self.border_contains(&p_proj_v) {
            return p_proj_v;
        }
        // Otherwise, closest among edges
        let mut best = self.vertices[0];
        let mut best_dist = f64::INFINITY;
        for line in self.lines() {
            let cp = line.closest_point(&p_proj_v);
            let d = cp.sub(&p_proj_v).norm();
            if d < best_dist {
                best_dist = d;
                best = cp;
            }
        }
        best
    }

    pub fn intersection_with_line(&self, line: &Line<V>, treat_as_ray: bool) -> Vec<V> {
        // Intersect infinite line with plane of polygon, then test containment
        // Line parametric: L(s) = s_start + s * dir
        let s0 = line.start().to_vec3();
        let s1 = line.end().to_vec3();
        let dir = s1 - s0;
        let n = self.normal;
        let denom = n.dot(&dir);
        if denom.abs() <= epsilon() {
            // Parallel: no intersection unless coplanar (not handled here)
            return Vec::new();
        }
        let c = self.centroid.to_vec3();
        let t = n.dot(&(c - s0)) / denom;
        if treat_as_ray && t < -epsilon() {
            return Vec::new();
        }
        let t = if treat_as_ray && t < 0.0 { 0.0 } else { t };
        let p = s0 + dir * t;
        let p_v = V::from_vec3(p);
        if self.contains(&p_v) || self.border_contains(&p_v) {
            vec![p_v]
        } else {
            Vec::new()
        }
    }

    pub fn reverse(&mut self) {
        self.vertices.reverse();
        // Flip normal and rebuild rotation
        self.normal = -self.normal;
        let ex = Vector3::new(self.rotation.column(0)[0], self.rotation.column(0)[1], self.rotation.column(0)[2]);
        let ez = self.normal;
        let ey = ez.cross(&ex);
        self.rotation = Matrix3::from_columns(&[ex, ey, ez]);
        // Recompute centroid sign if necessary (area sign flips)
        self.area = -self.area;
    }

    /// Whether the polygon is valid: non-degenerate area and no self-intersections.
    pub fn is_valid(&self) -> bool {
        self.area().abs() > epsilon() && !self.self_intersects()
    }

    fn self_intersects(&self) -> bool {
        let edges = self.lines();
        let n = edges.len();
        for i in 0..n {
            for j in i + 1..n {
                // Skip adjacent edges and the first-last pair
                let next_i = (i + 1) % n;
                let next_j = (j + 1) % n;
                if j == i || j == next_i || i == next_j || (i == 0 && j == n - 1) {
                    continue;
                }
                if let Some(_p) = edges[i].intersection(&edges[j], false) {
                    // Ignore intersections exactly at non-shared endpoints (handled by adjacency skip)
                    // If we got here, it's a proper crossing
                    return true;
                }
            }
        }
        false
    }
}

fn point_on_segment_2d(p: Vector3<f64>, a: Vector3<f64>, b: Vector3<f64>) -> bool {
    // Check if p is on segment ab in 2D (x,y)
    let ap = p - a;
    let ab = b - a;
    let cross = ap.x * ab.y - ap.y * ab.x;
    if cross.abs() > epsilon() {
        return false;
    }
    let dot = ap.x * ab.x + ap.y * ab.y;
    if dot < -epsilon() {
        return false;
    }
    let ab_len2 = ab.x * ab.x + ab.y * ab.y;
    if dot > ab_len2 + epsilon() {
        return false;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{assert_almost_eq, Line};
    use crate::Polygon as Polygon3d;

    #[test]
    fn square_xy_basic_metrics_and_axes() {
    let poly = Polygon3d::new([
            Vector2d::new(0.0, 0.0),
            Vector2d::new(1.0, 0.0),
            Vector2d::new(1.0, 1.0),
            Vector2d::new(0.0, 1.0),
        ]);
        assert_almost_eq!(poly.area(), 1.0);
        assert_almost_eq!(poly.perimeter(), 4.0);
        let c = poly.centroid();
        assert_almost_eq!(c.x(), 0.5);
        assert_almost_eq!(c.y(), 0.5);
        assert_almost_eq!(c.z(), 0.0);

        // Axes: ex along +X, ez along +Z, ey along +Y
        let ex = poly.axis(Axis::AxisX);
        let ey = poly.axis(Axis::AxisY);
        let ez = poly.axis(Axis::AxisZ);
        assert!(ex.is_approx(&Vector3d::new(1.0, 0.0, 0.0), None));
        assert!(ey.is_approx(&Vector3d::new(0.0, 1.0, 0.0), None));
        assert!(ez.is_approx(&Vector3d::new(0.0, 0.0, 1.0), None));
    }

    #[test]
    fn contains_border_and_closest_point() {
    let poly = Polygon3d::new([
            Vector2d::new(0.0, 0.0),
            Vector2d::new(2.0, 0.0),
            Vector2d::new(2.0, 1.0),
            Vector2d::new(0.0, 1.0),
        ]);
        let inside = Vector2d::new(1.0, 0.5);
        let edge_pt = Vector2d::new(0.0, 0.5);
        let outside = Vector2d::new(3.0, 0.5);
        assert!(poly.contains(&Vector3d::from(inside)));
        assert!(poly.border_contains(&Vector3d::from(edge_pt)));
        assert!(!poly.contains(&Vector3d::from(outside)));

        let p = Vector3d::new(3.0, 0.5, 2.0);
        let cp = poly.closest_point(&p);
        assert_almost_eq!(cp.y(), 0.5);
        assert_almost_eq!(cp.z(), 0.0);
        assert!(cp.x() <= 2.0 + epsilon() && cp.x() >= -epsilon());
    }

    #[test]
    fn line_intersection_through_center() {
    let poly = Polygon3d::new([
            Vector2d::new(0.0, 0.0),
            Vector2d::new(1.0, 0.0),
            Vector2d::new(1.0, 1.0),
            Vector2d::new(0.0, 1.0),
        ]);
        let line = Line::new(Vector3d::new(0.5, 0.5, -1.0), Vector3d::new(0.5, 0.5, 1.0));
        let hits = poly.intersection_with_line(&line, false);
        assert_eq!(hits.len(), 1);
        assert!(poly.contains(&hits[0]));
        assert!(hits[0].is_approx(&Vector3d::new(0.5, 0.5, 0.0), None));
    }
}
