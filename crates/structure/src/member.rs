use std::ops::{Deref, DerefMut};

use crate::{beam::Beam, node::Node, section::Section};

/// Highest level linear element enriched with a list of child beams forming a mesh.
#[derive(Debug, Clone)]
pub struct Member {
    beam: Beam,
    mesh: Vec<Beam>,
}

impl Member {
    pub fn new(start_node: Node, end_node: Node) -> Self {
        Self { beam: Beam::new(start_node, end_node), mesh: Vec::new() }
    }

    pub fn mesh(&self) -> &[Beam] {
        &self.mesh
    }

    pub fn mesh_mut(&mut self) -> &mut Vec<Beam> {
        &mut self.mesh
    }

    pub fn add_mesh_beam(&mut self, beam: Beam) {
        self.mesh.push(beam);
    }

    pub fn clear_mesh(&mut self) {
        self.mesh.clear();
    }
}

impl From<(Node, Node)> for Member {
    fn from((start, end): (Node, Node)) -> Self {
        Member::new(start, end)
    }
}

impl From<(Node, Node, Option<Section>)> for Member {
    fn from((start, end, maybe_section): (Node, Node, Option<Section>)) -> Self {
        let mut member = Member::new(start, end);
        if let Some(section) = maybe_section {
            member.set_section(section);
        }
        member
    }
}

impl From<(Node, Node, Section)> for Member {
    fn from((start, end, section): (Node, Node, Section)) -> Self {
        let mut member = Member::new(start, end);
        member.set_section(section);
        member
    }
}

impl Deref for Member {
    type Target = Beam;

    fn deref(&self) -> &Self::Target { &self.beam }
}

impl DerefMut for Member {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.beam }
}

#[cfg(test)]
mod tests {
    use utils::assert_almost_eq;

    use super::*;

    #[test]
    fn member_mesh_defaults_to_empty() {
        let start = Node::new((0.0, 0.0, 0.0));
        let end = Node::new((1.0, 0.0, 0.0));
        let member: Member = (start, end).into();
        assert!(member.mesh().is_empty());
    }

    #[test]
    fn member_mesh_accepts_child_beams() {
        let parent_start = Node::new((0.0, 0.0, 0.0));
        let parent_end = Node::new((2.0, 0.0, 0.0));
        let mut member: Member = (parent_start, parent_end).into();
        let child: Beam = Beam::new(Node::new((0.0, 0.0, 0.0)), Node::new((1.0, 0.0, 0.0)));
        member.add_mesh_beam(child);
        assert_eq!(member.mesh().len(), 1);
        assert_almost_eq!(member.mesh()[0].length(), 1.0);
    }
}
