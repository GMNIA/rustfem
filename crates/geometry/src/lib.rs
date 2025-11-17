mod edge;
mod arc;
mod polygon;
pub mod line;
mod shape;
mod vector;

// Public API: expose 3D concrete type aliases as canonical names; 2D inputs
// to public constructors will still be accepted but the canonical exported
// types are the 3D-specialized aliases below.
pub type Arc = arc::Arc<Vector3d>;
pub type Edge = edge::Edge<Vector3d>;
pub type Polygon = polygon::Polygon<Vector3d>;
pub use shape::{Disk, Rectangle, Shape, ShapeC, ShapeI, ShapeL, ShapeT};
pub use vector::{Vector2d, Vector3d};
pub use line::{Axis, LocalAxis, Line3d};
pub use line::Line3d as Line;
