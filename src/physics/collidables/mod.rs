pub mod collidable;
pub mod collidable_description;
pub mod collidable_reference;
pub mod ray;
pub mod shape;
pub mod typed_index;

// Convex shape primitives
pub mod box_shape;
pub mod capsule;
pub mod convex_hull;
pub mod convex_hull_helper;
pub mod cylinder;
pub mod sphere;
pub mod triangle;

// Compound shapes
pub mod big_compound;
pub mod compound;
pub mod compound_builder;

// Mesh
pub mod mesh;
pub mod mesh_inertia_helper;

// Shape batch management
pub mod shapes;
