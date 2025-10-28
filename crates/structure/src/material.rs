/// Simple isotropic material definition mirroring the Python demo.
#[derive(Debug, Clone, PartialEq)]
pub struct Material {
    name: Option<String>,
    young_modulus: f64,
    poisson_ratio: f64,
    density: f64,
    unit_weight: f64,
    thermal_coefficient: f64,
    friction_coefficient: f64,
    database_id: Option<String>,
}

impl Material {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        young_modulus: f64,
        poisson_ratio: f64,
        density: f64,
        unit_weight: f64,
        thermal_coefficient: f64,
        friction_coefficient: f64,
        name: Option<String>,
    ) -> Self {
        Self {
            name,
            young_modulus,
            poisson_ratio,
            density,
            unit_weight,
            thermal_coefficient,
            friction_coefficient,
            database_id: None,
        }
    }

    pub fn with_database_id(mut self, id: impl Into<String>) -> Self {
        self.database_id = Some(id.into());
        self
    }

    pub fn name(&self) -> Option<&str> { self.name.as_deref() }
    pub fn young_modulus(&self) -> f64 { self.young_modulus }
    pub fn poisson_ratio(&self) -> f64 { self.poisson_ratio }
    pub fn density(&self) -> f64 { self.density }
    pub fn unit_weight(&self) -> f64 { self.unit_weight }
    pub fn thermal_coefficient(&self) -> f64 { self.thermal_coefficient }
    pub fn friction_coefficient(&self) -> f64 { self.friction_coefficient }
    pub fn database_id(&self) -> Option<&str> { self.database_id.as_deref() }

    pub fn shear_modulus(&self) -> f64 {
        self.young_modulus / (2.0 * (1.0 + self.poisson_ratio))
    }

    pub fn stress(&self, strain: f64) -> f64 {
        self.young_modulus * strain
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shear_modulus_matches_isotropic_formula() {
        let material = Material::new(210e9, 0.3, 7850.0, 78.5, 1.2e-5, 0.2, Some("S355".into()));
        let shear = material.shear_modulus();
        assert!((shear - 80_769_230_769.23077).abs() < 1e-6);
    }

    #[test]
    fn stress_scales_with_strain() {
        let material = Material::new(210e9, 0.3, 7850.0, 78.5, 1.2e-5, 0.2, Some("S355".into()));
        assert_eq!(material.stress(1e-3), 210_000_000.0);
    }
}
