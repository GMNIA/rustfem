use crate::line::{Line, LineVector};
use crate::{epsilon, Vector2d, Vector3d};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge<V>
where
    V: LineVector,
{
    line: Line<V>,
    start_tangent: Option<V>,
    end_tangent: Option<V>,
}

pub type Edge2d = Edge<Vector2d>;
pub type Edge3d = Edge<Vector3d>;

impl<V> Edge<V>
where
    V: LineVector,
{
    pub fn new<S, E>(start: S, end: E) -> Self
    where
        S: Into<V>,
        E: Into<V>,
    {
        Self {
            line: Line::new(start.into(), end.into()),
            start_tangent: None,
            end_tangent: None,
        }
    }

    pub fn with_tangents<S, E, T1, T2>(start: S, end: E, start_tangent: T1, end_tangent: T2) -> Self
    where
        S: Into<V>,
        E: Into<V>,
        T1: Into<V>,
        T2: Into<V>,
    {
        Self {
            line: Line::new(start.into(), end.into()),
            start_tangent: Some(start_tangent.into()),
            end_tangent: Some(end_tangent.into()),
        }
    }

    pub fn start(&self) -> V {
        self.line.start()
    }

    pub fn end(&self) -> V {
        self.line.end()
    }

    pub fn length(&self) -> f64 {
        self.line.length()
    }

    pub fn centroid(&self) -> V {
        self.line.midpoint()
    }

    pub fn point_at(&self, t: f64) -> V {
        self.line.point_at(t)
    }

    pub fn closest_point(&self, point: &V) -> V {
        self.line.closest_point(point)
    }

    pub fn contains(&self, point: &V) -> bool {
        self.line.contains(point)
    }

    pub fn length_at_point(&self, point: &V) -> f64 {
        self.line.length_at_point(point)
    }

    pub fn break_at(&self, parameter: f64) -> Vec<Self> {
        self.line
            .break_at(parameter)
            .into_iter()
            .map(|segment| {
                let mut edge = Self::new(segment.start(), segment.end());
                if let Some(tangent) = self.start_tangent {
                    edge.start_tangent = Some(tangent);
                }
                if let Some(tangent) = self.end_tangent {
                    edge.end_tangent = Some(tangent);
                }
                edge
            })
            .collect()
    }

    pub fn break_at_point(&self, point: &V) -> Vec<Self> {
        self.line
            .break_at_point(point)
            .into_iter()
            .map(|segment| Self::new(segment.start(), segment.end()))
            .collect()
    }

    pub fn intersection_with_line(&self, other: &Line<V>, treat_as_ray: bool) -> Option<V> {
        self.line.intersection(other, treat_as_ray)
    }

    pub fn ray_intersection_with_edge(&self, other: &Self) -> Option<V> {
        self.line.ray_intersection(&other.line)
    }

    pub fn reverse(&mut self) {
        self.line.reverse();
        std::mem::swap(&mut self.start_tangent, &mut self.end_tangent);
    }

    pub fn reversed_edge(&self) -> Self {
        let reversed = Self {
            line: self.line.reversed(),
            start_tangent: self.end_tangent,
            end_tangent: self.start_tangent,
        };
        reversed
    }

    pub fn set_start_tangent(&mut self, tangent: V) {
        self.start_tangent = Some(tangent);
    }

    pub fn set_end_tangent(&mut self, tangent: V) {
        self.end_tangent = Some(tangent);
    }

    pub fn start_tangent(&self) -> Option<V> {
        self.start_tangent
    }

    pub fn end_tangent(&self) -> Option<V> {
        self.end_tangent
    }

    pub fn is_degenerate(&self) -> bool {
        self.length() <= epsilon()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_almost_eq;

    #[test]
    fn edge_length_and_point_at() {
        let edge = Edge2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 0.0));
        assert_almost_eq!(edge.length(), 4.0);
        let midpoint = edge.point_at(0.5);
        assert_almost_eq!(midpoint.x(), 2.0);
        assert_almost_eq!(midpoint.y(), 0.0);
    }

    #[test]
    fn edge_break_and_reverse() {
        let edge = Edge3d::new(Vector3d::new(0.0, 0.0, 0.0), Vector3d::new(0.0, 0.0, 10.0));
        let segments = edge.break_at(0.25);
        assert_eq!(segments.len(), 2);
        assert_almost_eq!(segments[0].length(), 2.5);
        assert_almost_eq!(segments[1].length(), 7.5);

        let mut mutable = edge;
        mutable.set_start_tangent(Vector3d::new(1.0, 0.0, 0.0));
        mutable.set_end_tangent(Vector3d::new(0.0, 1.0, 0.0));
        mutable.reverse();
        assert_eq!(mutable.start_tangent().unwrap(), Vector3d::new(0.0, 1.0, 0.0));
        assert_eq!(mutable.end_tangent().unwrap(), Vector3d::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn edge_intersection_with_line() {
        let edge = Edge2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(4.0, 4.0));
        let line = Line::new(Vector2d::new(0.0, 4.0), Vector2d::new(4.0, 0.0));
        let intersection = edge.intersection_with_line(&line, false).unwrap();
        assert_almost_eq!(intersection.x(), 2.0);
    }

    #[test]
    fn edge_contains_and_closest_point() {
        let edge = Edge2d::new(Vector2d::new(0.0, 0.0), Vector2d::new(10.0, 0.0));
        let point = Vector2d::new(5.0, 2.0);
        let closest = edge.closest_point(&point);
        assert_almost_eq!(closest.x(), 5.0);
        assert!(edge.contains(&Vector2d::new(5.0, 0.0)));
    }
}
