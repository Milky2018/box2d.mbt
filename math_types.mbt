// Core math types - ported from box2d/math_functions.h

// 2D vector
pub(all) struct Vec2 {
  x : Double
  y : Double
} derive(Eq, Show)

// 2D rotation (cosine and sine pair)
pub(all) struct Rot {
  c : Double  // cosine
  s : Double  // sine
} derive(Eq, Show)

// 2D rigid transform
pub(all) struct Transform {
  p : Vec2    // position
  q : Rot     // rotation
} derive(Eq, Show)

// 2-by-2 Matrix
pub(all) struct Mat22 {
  cx : Vec2   // column x
  cy : Vec2   // column y
} derive(Eq, Show)

// Axis-aligned bounding box
pub(all) struct AABB {
  lower_bound : Vec2
  upper_bound : Vec2
} derive(Eq, Show)

// Plane for collision
pub(all) struct Plane {
  normal : Vec2
  offset : Double
} derive(Eq, Show)

// Cosine and sine computation structure for determinism
pub(all) struct CosSin {
  cosine : Double
  sine : Double
} derive(Eq, Show)

// Constants
pub let pi : Double = 3.14159265359

// Zero and identity constants
pub let vec2_zero : Vec2 = { x: 0.0, y: 0.0 }
pub let rot_identity : Rot = { c: 1.0, s: 0.0 }
pub let transform_identity : Transform = { p: vec2_zero, q: rot_identity }
pub let mat22_zero : Mat22 = { cx: vec2_zero, cy: vec2_zero }

// Vector utility functions
pub fn min_vec2(a : Vec2, b : Vec2) -> Vec2 {
  { x: if a.x < b.x { a.x } else { b.x }, y: if a.y < b.y { a.y } else { b.y } }
}

pub fn max_vec2(a : Vec2, b : Vec2) -> Vec2 {
  { x: if a.x > b.x { a.x } else { b.x }, y: if a.y > b.y { a.y } else { b.y } }
}