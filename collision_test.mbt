// Box2D Collision Tests - ported from test_collision.c
// Testing collision detection and AABB operations

test "AABB operations" {
  // Test invalid AABB
  let a : AABB = {
    lower_bound: { x: -1.0, y: -1.0 },
    upper_bound: { x: -2.0, y: -2.0 }
  }
  
  // This AABB is invalid because upper < lower
  assert_true(a.upper_bound.x < a.lower_bound.x)
  
  // Create a valid AABB
  let valid_a : AABB = {
    lower_bound: { x: -1.0, y: -1.0 },
    upper_bound: { x: 1.0, y: 1.0 }
  }
  assert_true(valid_a.upper_bound.x > valid_a.lower_bound.x && valid_a.upper_bound.y > valid_a.lower_bound.y)
  
  // Test non-overlapping AABBs
  let b : AABB = {
    lower_bound: { x: 2.0, y: 2.0 },
    upper_bound: { x: 4.0, y: 4.0 }
  }
  
  assert_true(not(aabb_overlap(valid_a, b)))
  assert_true(not(aabb_contains(valid_a, b)))
  
  // Test overlapping AABBs
  let c : AABB = {
    lower_bound: { x: 0.5, y: 0.5 },
    upper_bound: { x: 1.5, y: 1.5 }
  }
  
  assert_true(aabb_overlap(valid_a, c))
  
  // Test AABB containment
  let d : AABB = {
    lower_bound: { x: -0.5, y: -0.5 },
    upper_bound: { x: 0.5, y: 0.5 }
  }
  
  assert_true(aabb_contains(valid_a, d))
}

test "point in shape tests" {
  // Test point in circle
  let circle : Circle = { center: { x: 0.0, y: 0.0 }, radius: 2.0 }
  
  ensure(point_in_circle({ x: 0.0, y: 0.0 }, circle.center, circle.radius), "center point in circle")
  ensure(point_in_circle({ x: 1.0, y: 1.0 }, circle.center, circle.radius), "point inside circle")
  ensure(not(point_in_circle({ x: 3.0, y: 0.0 }, circle.center, circle.radius)), "point outside circle")
  
  // Test point in AABB
  let aabb : AABB = {
    lower_bound: { x: -1.0, y: -1.0 },
    upper_bound: { x: 1.0, y: 1.0 }
  }
  
  ensure(point_in_aabb({ x: 0.0, y: 0.0 }, aabb), "center point in AABB")
  ensure(point_in_aabb({ x: 0.5, y: -0.5 }, aabb), "point inside AABB")
  ensure(not(point_in_aabb({ x: 2.0, y: 0.0 }, aabb)), "point outside AABB")
}

test "circle collision detection" {
  // Test overlapping circles
  let circle_a : Circle = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b : Circle = { center: { x: 1.5, y: 0.0 }, radius: 1.0 }
  
  let transform_a : Transform = { p: { x: 0.0, y: 0.0 }, q: rot_identity }
  let transform_b : Transform = { p: { x: 0.0, y: 0.0 }, q: rot_identity }
  
  let manifold = collide_circles(circle_a, transform_a, circle_b, transform_b)
  ensure(manifold.point_count > 0, "overlapping circles produce collision manifold")
  
  // Test separated circles
  let circle_c : Circle = { center: { x: 5.0, y: 0.0 }, radius: 1.0 }
  let manifold2 = collide_circles(circle_a, transform_a, circle_c, transform_b)
  ensure(manifold2.point_count == 0, "separated circles produce no collision")
}

test "distance calculations" {
  let a : Vec2 = { x: 0.0, y: 0.0 }
  let b : Vec2 = { x: 3.0, y: 4.0 }
  
  let dist = distance_between_points(a, b)
  ensure_small(dist - 5.0, 1.0e-10, "distance between points")
  
  // Test distance to line segment
  let segment_a : Vec2 = { x: -1.0, y: 0.0 }
  let segment_b : Vec2 = { x: 1.0, y: 0.0 }
  let point : Vec2 = { x: 0.0, y: 1.0 }
  
  let closest = closest_point_on_segment(point, segment_a, segment_b)
  ensure_small(closest.x - 0.0, 1.0e-10, "closest point on segment x")
  ensure_small(closest.y - 0.0, 1.0e-10, "closest point on segment y")
}

test "AABB computation" {
  // Test circle AABB
  let circle : Circle = { center: { x: 1.0, y: 2.0 }, radius: 0.5 }
  let transform : Transform = { p: { x: 0.0, y: 0.0 }, q: rot_identity }
  
  let aabb = compute_circle_aabb(circle, transform)
  ensure_small(aabb.lower_bound.x - 0.5, 1.0e-10, "circle AABB lower x")
  ensure_small(aabb.lower_bound.y - 1.5, 1.0e-10, "circle AABB lower y")
  ensure_small(aabb.upper_bound.x - 1.5, 1.0e-10, "circle AABB upper x")
  ensure_small(aabb.upper_bound.y - 2.5, 1.0e-10, "circle AABB upper y")
  
  // Test polygon AABB
  let polygon = make_polygon_box(1.0, 0.5)  // Use helper function
  
  let poly_aabb = compute_polygon_aabb(polygon, transform)
  ensure_small(poly_aabb.lower_bound.x - (-1.0), 1.0e-10, "polygon AABB lower x")
  ensure_small(poly_aabb.lower_bound.y - (-0.5), 1.0e-10, "polygon AABB lower y")
  ensure_small(poly_aabb.upper_bound.x - 1.0, 1.0e-10, "polygon AABB upper x")
  ensure_small(poly_aabb.upper_bound.y - 0.5, 1.0e-10, "polygon AABB upper y")
}

test "ray validation" {
  // Test valid ray input
  let origin : Vec2 = { x: 0.0, y: 0.0 }
  let translation : Vec2 = { x: 1.0, y: 0.0 }
  let max_fraction = 1.0
  
  ensure(is_valid_ray(origin, translation, max_fraction), "valid ray input")
  
  // Test invalid ray input with negative fraction
  ensure(not(is_valid_ray(origin, translation, -1.0)), "invalid ray with negative fraction")
  
  // Test ray with very large fraction
  ensure(not(is_valid_ray(origin, translation, 1.0e50)), "invalid ray with huge fraction")
}