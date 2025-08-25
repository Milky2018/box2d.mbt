// Box2D Physics Tests - ported from test_world.c  
// Testing world simulation and physics stepping

///|
test "hello world simulation" {
  // Create world
  let world_def = default_world_def()
  let world_id = create_world(world_def)
  assert_true(world_is_valid(world_id))

  // Create ground body
  let ground_body_def = default_body_def()
  ground_body_def.position = { x: 0.0, y: -10.0 }
  let ground_id = create_body(world_id, ground_body_def)
  assert_true(body_is_valid(ground_id))

  // Create ground shape
  let ground_box = make_box(50.0, 10.0)
  let ground_shape_def = default_shape_def()
  let ground_shape_id = create_polygon_shape(
    ground_id, ground_shape_def, ground_box,
  )
  assert_true(shape_is_valid(ground_shape_id))

  // Create dynamic body
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.position = { x: 0.0, y: 4.0 }
  let body_id = create_body(world_id, body_def)
  assert_true(body_is_valid(body_id))

  // Create dynamic body shape
  let circle : @box2d.Circle = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let shape_def = default_shape_def()
  let shape_id = create_circle_shape(body_id, shape_def, circle)
  assert_true(shape_is_valid(shape_id))

  // Simulate some steps
  for _i = 0; _i < 60; _i = _i + 1 {
    world_step(world_id, 1.0 / 60.0, 8)
  }

  // The body should have moved due to gravity
  let position = body_get_position(body_id)
  assert_true(position.y < 4.0) // Should have fallen
}

///|
test "body simulation" {
  // Test body physics simulation using our simulation module
  let world_def = default_world_def()
  let _world = create_world_from_def(world_def)

  // Create a body simulation
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.position = { x: 0.0, y: 5.0 }
  let body_sim = create_body_sim(body_def)
  assert_true(body_sim.mass > 0.0)
  assert_true(body_sim.inv_mass > 0.0)

  // Test applying force
  let force : @box2d.Vec2 = { x: 0.0, y: -9.8 } // Gravity-like force
  let center = body_sim.center
  let body_with_force = body_apply_force(body_sim, force, center)
  assert_true(body_with_force.force.y < 0.0)

  // Test integration
  let dt = 1.0 / 60.0
  let integrated_body = integrate_body(body_with_force, dt)

  // Body should have moved down slightly
  assert_true(integrated_body.transform.p.y < body_sim.transform.p.y)
}

///|
test "mass properties" {
  // Test circle mass computation
  let circle : @box2d.Circle = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let density = 1.0
  let mass_data = compute_circle_mass(circle, density)
  let expected_mass = pi * 1.0 * 1.0 * density // π * r² * density
  assert_true(abs_float(mass_data.mass - expected_mass) < 0.001)
  assert_true(mass_data.center.x == 0.0)
  assert_true(mass_data.center.y == 0.0)

  // Test polygon mass computation (simplified)
  let polygon = make_polygon_box(0.5, 0.5)  // Use helper function
  let poly_mass_data = compute_polygon_mass(polygon, density)
  assert_true(poly_mass_data.mass > 0.0)
}

///|
test "impulse application" {
  // Create a dynamic body
  let body_def : @box2d.BodyDef = default_body_def()
  body_def.type_ = Dynamic
  let body_sim = create_body_sim(body_def)

  // Apply an impulse
  let impulse : @box2d.Vec2 = { x: 5.0, y: 0.0 }
  let point = body_sim.center
  let body_with_impulse = body_apply_impulse(body_sim, impulse, point)

  // Body should now have velocity
  assert_true(body_with_impulse.linear_velocity.x > 0.0)
  assert_true(abs_float(body_with_impulse.linear_velocity.x - 5.0) < 0.001)
}

///|
test "coordinate transformations" {
  // Create a body with a transform
  let body_def = default_body_def()
  body_def.position = { x: 2.0, y: 3.0 }
  body_def.rotation = make_rot(pi / 4.0) // 45 degrees
  let body_sim = create_body_sim(body_def)

  // Test world/local point conversions
  let local_point : @box2d.Vec2 = { x: 1.0, y: 0.0 }
  let world_point = body_get_world_point(body_sim, local_point)
  let back_to_local = body_get_local_point(body_sim, world_point)
  assert_true(abs_float(back_to_local.x - local_point.x) < 0.001)
  assert_true(abs_float(back_to_local.y - local_point.y) < 0.001)

  // Test world/local vector conversions
  let local_vector : @box2d.Vec2 = { x: 1.0, y: 0.0 }
  let world_vector = body_get_world_vector(body_sim, local_vector)
  let back_to_local_vec = body_get_local_vector(body_sim, world_vector)
  assert_true(abs_float(back_to_local_vec.x - local_vector.x) < 0.001)
  assert_true(abs_float(back_to_local_vec.y - local_vector.y) < 0.001)
}

///|
test "collision manifold" {
  // Test circle-circle collision
  let circle_a : @box2d.Circle = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b : @box2d.Circle = { center: { x: 1.5, y: 0.0 }, radius: 1.0 }
  let transform_a : @box2d.Transform = {
    p: { x: 0.0, y: 0.0 },
    q: rot_identity,
  }
  let transform_b : @box2d.Transform = {
    p: { x: 0.0, y: 0.0 },
    q: rot_identity,
  }
  let manifold = collide_circles(circle_a, transform_a, circle_b, transform_b)
  assert_true(manifold.point_count > 0)

  // Test separated circles
  let circle_c : @box2d.Circle = { center: { x: 5.0, y: 0.0 }, radius: 1.0 }
  let manifold2 = collide_circles(circle_a, transform_a, circle_c, transform_b)
  assert_true(manifold2.point_count == 0)
}
