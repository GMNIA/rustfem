use geometry::Vector3d;

use crate::material::Material;

/// Simplified cross-section entity capturing the metadata listed in the Python dump.
#[derive(Debug, Clone, PartialEq)]
pub struct Section {
    name: Option<String>,
    material: Material,
    area: f64,
    mass: f64,
    centroid: Vector3d,
    elastic_modulus: Vector3d,
    is_generic: bool,
    is_principal: bool,
    is_centroidal: bool,
    openings_area: f64,
    shear_area: Vector3d,
    shear_center: Vector3d,
    static_moment: Vector3d,
    second_moment_y: f64,
    second_moment_z: f64,
    second_moment_yz: f64,
    torsion_constant: f64,
    torsion_radius: f64,
    warping_constant: f64,
    radius_of_gyration: Vector3d,
    plastic_modulus: Vector3d,
    principal_axes: Option<(Vector3d, Vector3d)>,
    rotation_principal_axes: Option<f64>,
    parts: Vec<String>,
    section_values: Vec<f64>,
}

impl Section {
    pub fn generic(material: Material, name: Option<String>) -> Self {
        Self {
            name,
            material,
            area: 0.0,
            mass: 0.0,
            centroid: Vector3d::new(0.0, 0.0, 0.0),
            elastic_modulus: Vector3d::new(0.0, 0.0, 0.0),
            is_generic: true,
            is_principal: false,
            is_centroidal: true,
            openings_area: 0.0,
            shear_area: Vector3d::new(0.0, 0.0, 0.0),
            shear_center: Vector3d::new(0.0, 0.0, 0.0),
            static_moment: Vector3d::new(0.0, 0.0, 0.0),
            second_moment_y: 0.0,
            second_moment_z: 0.0,
            second_moment_yz: 0.0,
            torsion_constant: 0.0,
            torsion_radius: 0.0,
            warping_constant: 0.0,
            radius_of_gyration: Vector3d::new(0.0, 0.0, 0.0),
            plastic_modulus: Vector3d::new(0.0, 0.0, 0.0),
            principal_axes: None,
            rotation_principal_axes: None,
            parts: Vec::new(),
            section_values: Vec::new(),
        }
    }

    pub fn name(&self) -> Option<&str> { self.name.as_deref() }
    pub fn material(&self) -> &Material { &self.material }

    pub fn area(&self) -> f64 { self.area }
    pub fn mass(&self) -> f64 { self.mass }
    pub fn centroid(&self) -> Vector3d { self.centroid }
    pub fn elastic_modulus(&self) -> Vector3d { self.elastic_modulus }
    pub fn is_generic(&self) -> bool { self.is_generic }
    pub fn is_principal(&self) -> bool { self.is_principal }
    pub fn is_centroidal(&self) -> bool { self.is_centroidal }
    pub fn openings_area(&self) -> f64 { self.openings_area }
    pub fn shear_area(&self) -> Vector3d { self.shear_area }
    pub fn shear_center(&self) -> Vector3d { self.shear_center }
    pub fn static_moment_of_area(&self) -> Vector3d { self.static_moment }
    pub fn second_moment_of_area_y(&self) -> f64 { self.second_moment_y }
    pub fn second_moment_of_area_z(&self) -> f64 { self.second_moment_z }
    pub fn second_moment_of_area_yz(&self) -> f64 { self.second_moment_yz }
    pub fn torsion_constant(&self) -> f64 { self.torsion_constant }
    pub fn torsion_radius(&self) -> f64 { self.torsion_radius }
    pub fn warping_constant(&self) -> f64 { self.warping_constant }
    pub fn radius_of_gyration(&self) -> Vector3d { self.radius_of_gyration }
    pub fn plastic_modulus(&self) -> Vector3d { self.plastic_modulus }
    pub fn rotation_principal_axes(&self) -> Option<f64> { self.rotation_principal_axes }
    pub fn principal_axes(&self) -> Option<(Vector3d, Vector3d)> { self.principal_axes }
    pub fn parts(&self) -> &[String] { &self.parts }
    pub fn section_values(&self) -> &[f64] { &self.section_values }

    pub fn append_part(&mut self, part: impl Into<String>) {
        self.parts.push(part.into());
    }

    pub fn add_section_value(&mut self, value: f64) {
        self.section_values.push(value);
    }

    pub fn set_area(&mut self, area: f64) { self.area = area; }
    pub fn set_mass(&mut self, mass: f64) { self.mass = mass; }
    pub fn set_centroid(&mut self, centroid: Vector3d) { self.centroid = centroid; }

    pub fn set_elastic_modulus(&mut self, modulus: Vector3d) {
        self.elastic_modulus = modulus;
    }

    pub fn set_second_moment_components(&mut self, iy: f64, iz: f64, iyz: f64) {
        self.second_moment_y = iy;
        self.second_moment_z = iz;
        self.second_moment_yz = iyz;
    }

    pub fn set_radius_of_gyration(&mut self, radius: Vector3d) {
        self.radius_of_gyration = radius;
    }

    pub fn simplified(&self) -> Vec<String> {
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use geometry::Vector3d;
    use utils::{assert_almost_eq, assert_vec3_almost_eq};

    use super::*;
    use crate::material::Material;

    #[test]
    fn generic_section_default() {
        let material = Material::new(
            210e9,
            0.3,
            8.004772271876737,
            78.5,
            1.2e-5,
            0.2,
            Some("S355".into()),
        );
        let section = Section::generic(material, Some("GenericSection".into()));

        assert!(section.is_generic());
        assert!(section.is_centroidal());
        assert!(!section.is_principal());
        assert_almost_eq!(section.area(), 0.0);
        assert_almost_eq!(section.mass(), 0.0);
        assert!(section.parts().is_empty());
        assert!(section.section_values().is_empty());
        assert!(section.simplified().is_empty());
        assert_vec3_almost_eq!(section.centroid(), Vector3d::new(0.0, 0.0, 0.0));
    }
}
