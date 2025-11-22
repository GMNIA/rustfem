use std::ops::{Deref, DerefMut};

use geometry::Vector3d;

use crate::{
    linearelement::LinearElement,
    node::Node,
    section::Section,
};

/// Spring abstraction sharing the same kinematics as any other linear element.
#[derive(Debug, Clone)]
pub struct Spring {
    element: LinearElement,
    section: Option<Section>,
    stiffness: Option<f64>,
}

impl Spring {
    pub fn new(start_node: Node, end_node: Node) -> Self {
        Self { element: LinearElement::new(start_node, end_node), section: None, stiffness: None }
    }

    pub fn from_points<S, E>(start: S, end: E, section: Option<Section>) -> Self
    where
        S: Into<Vector3d>,
        E: Into<Vector3d>,
    {
        let mut spring = Self::new(Node::new(start.into()), Node::new(end.into()));
        if let Some(section) = section {
            spring.section = Some(section);
        }
        spring
    }

    pub fn set_section(&mut self, section: Section) {
        self.section = Some(section);
    }

    pub fn clear_section(&mut self) {
        self.section = None;
    }

    pub fn section(&self) -> Option<&Section> {
        self.section.as_ref()
    }

    pub fn set_stiffness(&mut self, stiffness: f64) {
        self.stiffness = Some(stiffness);
    }

    pub fn clear_stiffness(&mut self) {
        self.stiffness = None;
    }

    pub fn stiffness(&self) -> Option<f64> {
        self.stiffness
    }
}

impl Deref for Spring {
    type Target = LinearElement;

    fn deref(&self) -> &Self::Target { &self.element }
}

impl DerefMut for Spring {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.element }
}

#[cfg(test)]
mod tests {
    use utils::assert_almost_eq;

    use super::*;
    use crate::{material::Material, section::Section};

    #[test]
    fn spring_defaults_to_zero_stiffness() {
        let spring = Spring::new(
            Node::new(Vector3d::new(0.0, 0.0, 0.0)),
            Node::new(Vector3d::new(1.0, 0.0, 0.0)),
        );

        assert!(spring.section().is_none());
        assert!(spring.stiffness().is_none());
        assert_almost_eq!(spring.length(), 1.0);
    }

    #[test]
    fn spring_accepts_section_and_stiffness() {
        let material = Material::new(210e9, 0.3, 8.0, 78.5, 1.2e-5, 0.2, None);
        let section = Section::generic(material, None);
        let mut spring = Spring::from_points((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), Some(section));

        spring.set_stiffness(42.0);
        assert!(spring.section().is_some());
        assert_almost_eq!(spring.stiffness().unwrap(), 42.0);
    }
}
