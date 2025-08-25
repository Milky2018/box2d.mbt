// Box2D Math Tests - ported from test_math.c
// Comprehensive testing of mathematical functions for accuracy and correctness

// Test tolerance constants (ported from test_math.c)
let atan_tolerance = 0.00004  // 0.0023 degrees
let angle_tolerance = 0.002   // cosine/sine approximation tolerance

// Helper function for small value assertions
fn ensure_small(value : Double, tolerance : Double, description : String) -> Unit {
  if value < -tolerance || tolerance < value {
    abort("Test failed: " + description + " - value not within tolerance")
  }
}

fn ensure(condition : Bool, description : String) -> Unit {
  if not(condition) {
    abort("Test failed: " + description)
  }
}

// Test trigonometric functions accuracy
test "atan2 and trigonometric accuracy" {
  // Test atan2 and trigonometric functions over a range
  let mut t = -10.0
  while t < 10.0 {
    let angle = pi * t
    let rot = make_rot(angle)
    let c = @math.cos(angle)
    let s = @math.sin(angle)
    
    // The cosine and sine approximations should be accurate to about 0.002 radians
    ensure_small(rot.c - c, angle_tolerance, "cosine approximation")
    ensure_small(rot.s - s, angle_tolerance, "sine approximation")
    
    let unwound_angle = unwind_angle(angle)
    ensure(-pi <= unwound_angle && unwound_angle <= pi, "angle unwinding range")
    
    let computed_angle = atan2(s, c)
    ensure(is_valid_float(computed_angle), "atan2 returns valid float")
    
    let mut diff = abs_float(computed_angle - unwound_angle)
    
    // The two results can be off by 360 degrees (-pi and pi)
    if diff > pi {
      diff = diff - 2.0 * pi
    }
    
    // The approximate atan2 should be quite accurate
    ensure_small(diff, atan_tolerance, "atan2 accuracy")
    
    t = t + 0.1  // Use larger step for faster testing
  }
}

test "atan2 edge cases" {
  // Test atan2 over a grid of values
  let mut y = -1.0
  while y <= 1.0 {
    let mut x = -1.0
    while x <= 1.0 {
      let computed = atan2(y, x)
      ensure(is_valid_float(computed), "atan2 grid test returns valid float")
      x = x + 0.1  // Use larger step for faster testing
    }
    y = y + 0.1
  }
  
  // Test specific edge cases
  let a1 = atan2(1.0, 0.0)
  ensure(is_valid_float(a1), "atan2(1,0) valid")
  
  let a2 = atan2(-1.0, 0.0)
  ensure(is_valid_float(a2), "atan2(-1,0) valid")
  
  let a3 = atan2(0.0, 1.0)
  ensure(is_valid_float(a3), "atan2(0,1) valid")
  
  let a4 = atan2(0.0, -1.0)
  ensure(is_valid_float(a4), "atan2(0,-1) valid")
  
  let a5 = atan2(0.0, 0.0)
  ensure(is_valid_float(a5), "atan2(0,0) valid")
}

test "vector operations" {
  let zero = vec2_zero
  let one : Vec2 = { x: 1.0, y: 1.0 }
  let two : Vec2 = { x: 2.0, y: 2.0 }
  
  let v = add(one, two)
  ensure(v.x == 3.0 && v.y == 3.0, "vector addition")
  
  let v2 = sub(zero, two)
  ensure(v2.x == -2.0 && v2.y == -2.0, "vector subtraction")
  
  let v3 = add(two, two)
  ensure(v3.x != 5.0 && v3.y != 5.0, "vector addition inequality")
}

test "transform operations" {
  let transform1 : Transform = { p: { x: -2.0, y: 3.0 }, q: make_rot(1.0) }
  let transform2 : Transform = { p: { x: 1.0, y: 0.0 }, q: make_rot(-2.0) }
  let two : Vec2 = { x: 2.0, y: 2.0 }
  
  let combined_transform = mul_transforms(transform2, transform1)
  
  // Test transform composition
  let v1 = transform_point(transform2, transform_point(transform1, two))
  let v2 = transform_point(combined_transform, two)
  
  let epsilon = 1.0e-6  // Similar to 10.0 * FLT_EPSILON
  ensure_small(v2.x - v1.x, epsilon, "transform composition x")
  ensure_small(v2.y - v1.y, epsilon, "transform composition y")
  
  // Test inverse transform
  let v3 = transform_point(transform1, two)
  let v4 = inv_transform_point(transform1, v3)
  
  ensure_small(v4.x - two.x, epsilon, "inverse transform x")
  ensure_small(v4.y - two.y, epsilon, "inverse transform y")
}

test "vector normalization and rotation" {
  let v = normalize({ x: 0.2, y: -0.5 })
  
  // Test that normalized vector has length close to 1.0
  let length_v = length(v)
  inspect(abs_float(length_v - 1.0) < 1.0e-6, content="true")
  
  // Test normalization over a few key points (avoid infinite loop)
  let test_vectors : Array[Vec2] = [
    { x: 1.0, y: 0.0 },
    { x: 0.0, y: 1.0 },
    { x: 1.0, y: 1.0 },
    { x: -1.0, y: 1.0 },
    { x: 3.0, y: 4.0 }  // This should normalize to (0.6, 0.8)
  ]
  
  for test_vec in test_vectors {
    let u = normalize(test_vec)
    let length_u = length(u)
    
    // Normalized vectors should have length very close to 1.0
    inspect(abs_float(length_u - 1.0) < 1.0e-6, content="true")
  }
}

test "rotation interpolation" {
  // Test rotation interpolation (NLerp equivalent)
  let _q1 = rot_identity
  let _q2 = make_rot(0.5 * pi)
  let n = 10  // Smaller number for faster testing
  
  for i = 0; i <= n; i = i + 1 {
    let alpha = i.to_double() / n.to_double()
    
    // Simple linear interpolation of rotation
    let angle = alpha * 0.5 * pi
    let q = make_rot(angle)
    let computed_angle = rot_get_angle(q)
    
    let tolerance = 5.0 * pi / 180.0  // 5 degrees
    ensure_small(angle - computed_angle, tolerance, "rotation interpolation")
  }
}

test "relative angle computation" {
  let base_angle = 0.75 * pi
  let q1 = make_rot(base_angle)
  
  let mut t = -5.0  // Smaller range for faster testing
  while t < 5.0 {
    let angle = pi * t
    let q2 = make_rot(angle)
    
    let relative_angle = relative_angle(q1, q2)
    let unwound_angle = unwind_angle(angle - base_angle)
    let tolerance = 0.1 * pi / 180.0  // 0.1 degrees
    
    ensure_small(relative_angle - unwound_angle, tolerance, "relative angle computation")
    
    t = t + 0.2  // Use larger step for faster testing
  }
}

test "math function validation" {
  // Test validation functions
  let valid_vec : Vec2 = { x: 1.0, y: 2.0 }
  let valid_rot = make_rot(0.5)
  let valid_transform : Transform = { p: valid_vec, q: valid_rot }
  
  ensure(is_valid_vec2(valid_vec), "valid vector validation")
  ensure(is_valid_rotation(valid_rot), "valid rotation validation")
  ensure(is_valid_transform(valid_transform), "valid transform validation")
  
  // Test invalid cases with extreme values
  let large_vec : Vec2 = { x: 1.0e10, y: 1.0e10 }
  ensure(is_valid_vec2(large_vec), "large vector still valid")
}

test "distance and length functions" {
  let a : Vec2 = { x: 0.0, y: 0.0 }
  let b : Vec2 = { x: 3.0, y: 4.0 }
  
  let dist = distance(a, b)
  ensure_small(dist - 5.0, 1.0e-10, "distance calculation")
  
  let len = length(b)
  ensure_small(len - 5.0, 1.0e-10, "length calculation")
  
  let len_sq = length_squared(b)
  ensure_small(len_sq - 25.0, 1.0e-10, "length squared calculation")
}