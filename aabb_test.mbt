// Box2D AABB Tests - ported from test_collision.c
// Testing AABB validation, overlap, containment, and ray casting

test "AABB validation and operations" {
  // Test invalid AABB (ported from AABBTest)
  let a : @box2d.AABB = {
    lower_bound: { x: -1.0, y: -1.0 },
    upper_bound: { x: -2.0, y: -2.0 }
  }
  
  // This AABB is invalid because upper < lower
  assert_true(not(@box2d.is_valid_aabb(a)))
  
  // Create a valid AABB
  let valid_a : @box2d.AABB = {
    lower_bound: { x: -1.0, y: -1.0 },
    upper_bound: { x: 1.0, y: 1.0 }
  }
  assert_true(@box2d.is_valid_aabb(valid_a))
  
  // Test non-overlapping AABBs
  let b : @box2d.AABB = {
    lower_bound: { x: 2.0, y: 2.0 },
    upper_bound: { x: 4.0, y: 4.0 }
  }
  
  assert_true(not(@box2d.aabb_overlap(valid_a, b)))
  assert_true(not(@box2d.aabb_contains(valid_a, b)))
  
  // Test AABB ray casting
  let p1 : @box2d.Vec2 = { x: -2.0, y: 0.0 }
  let p2 : @box2d.Vec2 = { x: 2.0, y: 0.0 }
  
  let output = @box2d.aabb_ray_cast(valid_a, p1, p2)
  assert_true(output.hit)
  assert_true(0.1 < output.fraction && output.fraction < 0.9)
}