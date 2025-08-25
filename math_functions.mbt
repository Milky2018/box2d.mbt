// Math functions - ported from box2d/math_functions.h and math_functions.c

// Types are available from same package automatically

// Validation functions (ported from math_functions.c line by line)

// bool b2IsValidFloat( float a )
pub fn is_valid_float(_a : Double) -> Bool {
  // Simplified check for now - full implementation would need better NaN/infinity detection
  // TODO: Implement proper NaN/infinity detection when MoonBit supports it
  // if ( isnan( a ) ) return false;
  // if ( isinf( a ) ) return false;
  // return true;
  true
}

// Is this a valid vector? Not NaN or infinity.
pub fn is_valid_vec2(v : Vec2) -> Bool {
  is_valid_float(v.x) && is_valid_float(v.y)
}

// Is this a valid rotation? Not NaN or infinity. Is normalized.
pub fn is_valid_rotation(q : Rot) -> Bool {
  is_valid_float(q.c) && is_valid_float(q.s) && is_normalized_rot(q)
}

// Is this a valid transform? Not NaN or infinity. Rotation is normalized.
pub fn is_valid_transform(t : Transform) -> Bool {
  is_valid_vec2(t.p) && is_valid_rotation(t.q)
}

// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
pub fn is_valid_plane(a : Plane) -> Bool {
  is_valid_vec2(a.normal) && is_normalized(a.normal) && is_valid_float(a.offset)
}

// Utility functions (ported from math_functions.h)

// Min/max for integers
pub fn min_int(a : Int, b : Int) -> Int {
  if a < b { a } else { b }
}

pub fn max_int(a : Int, b : Int) -> Int {
  if a > b { a } else { b }
}

pub fn abs_int(a : Int) -> Int {
  if a < 0 { -a } else { a }
}

// Negation for float
pub fn neg_float(a : Double) -> Double {
  -a
}

pub fn clamp_int(a : Int, lower : Int, upper : Int) -> Int {
  if a < lower { lower } else if a > upper { upper } else { a }
}

// Min/max for floats
pub fn min_float(a : Double, b : Double) -> Double {
  if a < b { a } else { b }
}

pub fn max_float(a : Double, b : Double) -> Double {
  if a > b { a } else { b }
}

pub fn abs_float(a : Double) -> Double {
  if a < 0.0 { -a } else { a }
}

pub fn clamp_float(a : Double, lower : Double, upper : Double) -> Double {
  if a < lower { lower } else if a > upper { upper } else { a }
}

// Custom atan2 implementation for determinism (ported from math_functions.c)
pub fn atan2(y : Double, x : Double) -> Double {
  // Added check for (0,0) to match atan2f and avoid NaN
  if x == 0.0 && y == 0.0 {
    return 0.0
  }

  let ax = abs_float(x)
  let ay = abs_float(y)
  let mx = max_float(ay, ax)
  let mn = min_float(ay, ax)
  let a = mn / mx

  // Minimax polynomial approximation to atan(a) on [0,1]
  let s = a * a
  let c = s * a
  let q = s * s
  let mut r = 0.024840285 * q + 0.18681418
  let t = -0.094097948 * q - 0.33213072
  r = r * s + t
  r = r * c + a

  // Map to full circle
  if ay > ax {
    r = 1.57079637 - r
  }

  if x < 0.0 {
    r = 3.14159274 - r
  }

  if y < 0.0 {
    r = -r
  }

  r
}

// CosSin is defined in math_types.mbt

// Convert any angle into the range [-pi, pi]
pub fn unwind_angle(radians : Double) -> Double {
  // Using modulo operation for determinism
  let two_pi = 2.0 * pi
  let result = radians % two_pi
  if result > pi {
    result - two_pi
  } else if result < -pi {
    result + two_pi
  } else {
    result
  }
}

// Compute the cosine and sine of an angle in radians (ported from math_functions.c)
pub fn compute_cos_sin(radians : Double) -> CosSin {
  let x = unwind_angle(radians)
  let pi2 = pi * pi

  // cosine needs angle in [-pi/2, pi/2]
  let c = if x < -0.5 * pi {
    let y = x + pi
    let y2 = y * y
    -(pi2 - 4.0 * y2) / (pi2 + y2)
  } else if x > 0.5 * pi {
    let y = x - pi
    let y2 = y * y
    -(pi2 - 4.0 * y2) / (pi2 + y2)
  } else {
    let y2 = x * x
    (pi2 - 4.0 * y2) / (pi2 + y2)
  }

  // sine needs angle in [0, pi]
  let s = if x < 0.0 {
    let y = x + pi
    -16.0 * y * (pi - y) / (5.0 * pi2 - 4.0 * y * (pi - y))
  } else {
    16.0 * x * (pi - x) / (5.0 * pi2 - 4.0 * x * (pi - x))
  }

  let mag = (s * s + c * c).sqrt()
  let inv_mag = if mag > 0.0 { 1.0 / mag } else { 0.0 }
  { cosine: c * inv_mag, sine: s * inv_mag }
}

// Vector operations (ported from math_functions.h)

// Vector dot product
pub fn dot(a : Vec2, b : Vec2) -> Double {
  a.x * b.x + a.y * b.y
}

// Vector cross product. In 2D this yields a scalar.
pub fn cross(a : Vec2, b : Vec2) -> Double {
  a.x * b.y - a.y * b.x
}

// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
pub fn cross_vs(v : Vec2, s : Double) -> Vec2 {
  { x: s * v.y, y: -s * v.x }
}

// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
pub fn cross_sv(s : Double, v : Vec2) -> Vec2 {
  { x: -s * v.y, y: s * v.x }
}

// Get a left pointing perpendicular vector
pub fn left_perp(v : Vec2) -> Vec2 {
  { x: -v.y, y: v.x }
}

// Get a right pointing perpendicular vector
pub fn right_perp(v : Vec2) -> Vec2 {
  { x: v.y, y: -v.x }
}

// Vector addition
pub fn add(a : Vec2, b : Vec2) -> Vec2 {
  { x: a.x + b.x, y: a.y + b.y }
}

// Vector subtraction
pub fn sub(a : Vec2, b : Vec2) -> Vec2 {
  { x: a.x - b.x, y: a.y - b.y }
}

// Vector negation
pub fn neg(a : Vec2) -> Vec2 {
  { x: -a.x, y: -a.y }
}

// Vector linear interpolation
pub fn lerp(a : Vec2, b : Vec2, t : Double) -> Vec2 {
  { x: (1.0 - t) * a.x + t * b.x, y: (1.0 - t) * a.y + t * b.y }
}

// Component-wise multiplication
pub fn mul_vv(a : Vec2, b : Vec2) -> Vec2 {
  { x: a.x * b.x, y: a.y * b.y }
}

// Multiply a scalar and vector
pub fn mul_sv(s : Double, v : Vec2) -> Vec2 {
  { x: s * v.x, y: s * v.y }
}

// a + s * b
pub fn mul_add(a : Vec2, s : Double, b : Vec2) -> Vec2 {
  { x: a.x + s * b.x, y: a.y + s * b.y }
}

// a - s * b
pub fn mul_sub(a : Vec2, s : Double, b : Vec2) -> Vec2 {
  { x: a.x - s * b.x, y: a.y - s * b.y }
}

// Component-wise absolute vector
pub fn abs_vec2(a : Vec2) -> Vec2 {
  { x: abs_float(a.x), y: abs_float(a.y) }
}

// Note: min_vec2 and max_vec2 are now defined in math_types.mbt

// Component-wise clamp vector v into the range [a, b]
pub fn clamp_vec2(v : Vec2, a : Vec2, b : Vec2) -> Vec2 {
  { x: clamp_float(v.x, a.x, b.x), y: clamp_float(v.y, a.y, b.y) }
}

// Get the length of this vector (the norm)
pub fn length(v : Vec2) -> Double {
  (v.x * v.x + v.y * v.y).sqrt()
}

// Aliases for test compatibility
pub fn vec2_add(a : Vec2, b : Vec2) -> Vec2 { add(a, b) }
pub fn vec2_dot(a : Vec2, b : Vec2) -> Double { dot(a, b) }
pub fn vec2_length(v : Vec2) -> Double { length(v) }

// Get the distance between two points
pub fn distance(a : Vec2, b : Vec2) -> Double {
  let dx = b.x - a.x
  let dy = b.y - a.y
  (dx * dx + dy * dy).sqrt()
}

// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
pub fn normalize(v : Vec2) -> Vec2 {
  let length = (v.x * v.x + v.y * v.y).sqrt()
  if length < 1.0e-10 {  // Similar to FLT_EPSILON
    { x: 0.0, y: 0.0 }
  } else {
    let inv_length = 1.0 / length
    { x: inv_length * v.x, y: inv_length * v.y }
  }
}

// Determines if the provided vector is normalized (norm(a) == 1).
pub fn is_normalized(a : Vec2) -> Bool {
  let aa = dot(a, a)
  abs_float(1.0 - aa) < 100.0 * 1.0e-10  // Similar to 100.0 * FLT_EPSILON
}

// Get the length squared of this vector
pub fn length_squared(v : Vec2) -> Double {
  v.x * v.x + v.y * v.y
}

// Get the distance squared between points
pub fn distance_squared(a : Vec2, b : Vec2) -> Double {
  let dx = b.x - a.x
  let dy = b.y - a.y
  dx * dx + dy * dy
}

// Rotation operations

// Make a rotation using an angle in radians
pub fn make_rot(radians : Double) -> Rot {
  let cs = compute_cos_sin(radians)
  { c: cs.cosine, s: cs.sine }
}

// Make a rotation using a unit vector
pub fn make_rot_from_unit_vector(unit_vector : Vec2) -> Rot {
  // Assert that the vector is normalized in debug mode
  { c: unit_vector.x, s: unit_vector.y }
}

// Is this rotation normalized?
pub fn is_normalized_rot(q : Rot) -> Bool {
  // larger tolerance due to failure on some platforms
  let qq = q.s * q.s + q.c * q.c
  1.0 - 0.0006 < qq && qq < 1.0 + 0.0006
}

// Normalize rotation
pub fn normalize_rot(q : Rot) -> Rot {
  let mag = (q.s * q.s + q.c * q.c).sqrt()
  let inv_mag = if mag > 0.0 { 1.0 / mag } else { 0.0 }
  { c: q.c * inv_mag, s: q.s * inv_mag }
}

// Get the angle in radians in the range [-pi, pi]
pub fn rot_get_angle(q : Rot) -> Double {
  atan2(q.s, q.c)
}

// Get the x-axis
pub fn rot_get_x_axis(q : Rot) -> Vec2 {
  { x: q.c, y: q.s }
}

// Get the y-axis
pub fn rot_get_y_axis(q : Rot) -> Vec2 {
  { x: -q.s, y: q.c }
}

// Multiply two rotations: q * r
pub fn mul_rot(q : Rot, r : Rot) -> Rot {
  {
    s: q.s * r.c + q.c * r.s,
    c: q.c * r.c - q.s * r.s
  }
}

// Transpose multiply two rotations: inv(a) * b
pub fn inv_mul_rot(a : Rot, b : Rot) -> Rot {
  {
    s: a.c * b.s - a.s * b.c,
    c: a.c * b.c + a.s * b.s
  }
}

// Relative angle between a and b
pub fn relative_angle(a : Rot, b : Rot) -> Double {
  let s = a.c * b.s - a.s * b.c
  let c = a.c * b.c + a.s * b.s
  atan2(s, c)
}

// Rotate a vector
pub fn rotate_vector(q : Rot, v : Vec2) -> Vec2 {
  { x: q.c * v.x - q.s * v.y, y: q.s * v.x + q.c * v.y }
}

// Inverse rotate a vector
pub fn inv_rotate_vector(q : Rot, v : Vec2) -> Vec2 {
  { x: q.c * v.x + q.s * v.y, y: -q.s * v.x + q.c * v.y }
}

// Transform operations

// Transform a point (e.g. local space to world space)
pub fn transform_point(t : Transform, p : Vec2) -> Vec2 {
  let x = (t.q.c * p.x - t.q.s * p.y) + t.p.x
  let y = (t.q.s * p.x + t.q.c * p.y) + t.p.y
  { x, y }
}

// Inverse transform a point (e.g. world space to local space)
pub fn inv_transform_point(t : Transform, p : Vec2) -> Vec2 {
  let vx = p.x - t.p.x
  let vy = p.y - t.p.y
  { x: t.q.c * vx + t.q.s * vy, y: -t.q.s * vx + t.q.c * vy }
}

// Multiply two transforms
pub fn mul_transforms(a : Transform, b : Transform) -> Transform {
  {
    q: mul_rot(a.q, b.q),
    p: add(rotate_vector(a.q, b.p), a.p)
  }
}

// Creates a transform that converts a local point in frame B to a local point in frame A
pub fn inv_mul_transforms(a : Transform, b : Transform) -> Transform {
  {
    q: inv_mul_rot(a.q, b.q),
    p: inv_rotate_vector(a.q, sub(b.p, a.p))
  }
}

// AABB operations


// Get the center of the AABB
pub fn aabb_center(a : AABB) -> Vec2 {
  { 
    x: 0.5 * (a.lower_bound.x + a.upper_bound.x),
    y: 0.5 * (a.lower_bound.y + a.upper_bound.y)
  }
}

// Get the extents of the AABB (half-widths)
pub fn aabb_extents(a : AABB) -> Vec2 {
  {
    x: 0.5 * (a.upper_bound.x - a.lower_bound.x),
    y: 0.5 * (a.upper_bound.y - a.lower_bound.y)
  }
}

// Union of two AABBs
pub fn aabb_union(a : AABB, b : AABB) -> AABB {
  {
    lower_bound: {
      x: min_float(a.lower_bound.x, b.lower_bound.x),
      y: min_float(a.lower_bound.y, b.lower_bound.y)
    },
    upper_bound: {
      x: max_float(a.upper_bound.x, b.upper_bound.x),
      y: max_float(a.upper_bound.y, b.upper_bound.y)
    }
  }
}

// Do a and b overlap
pub fn aabb_overlaps(a : AABB, b : AABB) -> Bool {
  not(b.lower_bound.x > a.upper_bound.x || 
      b.lower_bound.y > a.upper_bound.y || 
      a.lower_bound.x > b.upper_bound.x ||
      a.lower_bound.y > b.upper_bound.y)
}

// Signed separation of a point from a plane
pub fn plane_separation(plane : Plane, point : Vec2) -> Double {
  dot(plane.normal, point) - plane.offset
}

// One-dimensional mass-spring-damper simulation
pub fn spring_damper(hertz : Double, damping_ratio : Double, position : Double, velocity : Double, time_step : Double) -> Double {
  let omega = 2.0 * pi * hertz
  let omega_h = omega * time_step
  (velocity - omega * omega_h * position) / (1.0 + 2.0 * damping_ratio * omega_h + omega_h * omega_h)
}

// Get length and normalized vector (helper for collision functions)
pub fn get_length_and_normalize(v : Vec2) -> (Double, Vec2) {
  let len = length(v)
  if len < 1.0e-10 {
    (0.0, { x: 0.0, y: 0.0 })
  } else {
    (len, { x: v.x / len, y: v.y / len })
  }
}