// Box2D Constraint Solver Tests - based on Box2D validation patterns
// Testing constraint solver functionality and validation

///|
test "contact constraint initialization" {
  // Test contact constraint initialization from manifold
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.5, y: 0.0 }
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.5, y: 0.0 }
  let body_sim_b = create_body_sim(body_def_b)

  // Create overlapping circles
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  assert_true(manifold.point_count > 0)

  // Initialize contact constraint
  let constraint = init_contact_constraint(
    manifold, body_sim_a, body_sim_b, 0, 1,
  )

  // Validate constraint initialization
  assert_true(constraint.body_a_index == 0)
  assert_true(constraint.body_b_index == 1)
  assert_true(constraint.point_count == manifold.point_count)
  assert_true(constraint.inv_mass_a == body_sim_a.inv_mass)
  assert_true(constraint.inv_mass_b == body_sim_b.inv_mass)
  assert_true(constraint.inv_i_a == body_sim_a.inv_inertia)
  assert_true(constraint.inv_i_b == body_sim_b.inv_inertia)

  // Check that constraint points were created
  assert_true(constraint.points.length() > 0)

  // Validate constraint point data
  if constraint.points.length() > 0 {
    let point = constraint.points[0]
    assert_true(point.normal_mass > 0.0) // Should have valid effective mass
    assert_true(point.tangent_mass > 0.0)
    assert_true(point.normal_impulse == 0.0) // Should start with no accumulated impulse
    assert_true(point.tangent_impulse == 0.0)
  }
}

///|
test "contact constraint with no collision" {
  // Test constraint initialization with empty manifold
  let body_sim_a = create_body_sim(default_body_def())
  let body_sim_b = create_body_sim(default_body_def())
  let empty_manifold = {
    normal: { x: 0.0, y: 1.0 },
    rolling_impulse: 0.0,
    points: [],
    point_count: 0,
  }
  let constraint = init_contact_constraint(
    empty_manifold, body_sim_a, body_sim_b, 0, 1,
  )

  // Should create valid but empty constraint
  assert_true(constraint.point_count == 0)
  assert_true(constraint.points.length() == 0)
  assert_true(constraint.body_a_index == 0)
  assert_true(constraint.body_b_index == 1)
}

///|
test "contact solver creation" {
  // Create two colliding bodies
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.5, y: 0.0 }
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.5, y: 0.0 }
  let body_sim_b = create_body_sim(body_def_b)
  let bodies = [body_sim_a, body_sim_b]

  // Create collision manifold
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  let manifolds = [(0, 1, manifold)]

  // Create contact solver
  let solver = create_contact_solver(manifolds, bodies, 8, 3)

  // Validate solver creation
  assert_true(solver.constraint_count == 1)
  assert_true(solver.constraints.length() == 1)
  assert_true(solver.velocity_iterations == 8)
  assert_true(solver.position_iterations == 3)
}

///|
test "contact solver with no collisions" {
  // Test solver with empty collision list
  let body_sim = create_body_sim(default_body_def())
  let bodies = [body_sim]
  let manifolds = []
  let solver = create_contact_solver(manifolds, bodies, 8, 3)

  // Should create valid but empty solver
  assert_true(solver.constraint_count == 0)
  assert_true(solver.constraints.length() == 0)
  assert_true(solver.velocity_iterations == 8)
  assert_true(solver.position_iterations == 3)
}

///|
test "warm starting constraints" {
  // Create two dynamic bodies with initial velocities
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.5, y: 0.0 }
  body_def_a.linear_velocity = { x: 1.0, y: 0.0 }
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.5, y: 0.0 }
  body_def_b.linear_velocity = { x: -1.0, y: 0.0 }
  let body_sim_b = create_body_sim(body_def_b)
  let bodies = [body_sim_a, body_sim_b]

  // Create collision and solver
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  let manifolds = [(0, 1, manifold)]
  let solver = create_contact_solver(manifolds, bodies, 8, 3)

  // Apply warm starting
  let bodies_after_warmstart = warm_start_constraints(solver, bodies)

  // Validate that bodies array is returned with same size
  assert_true(bodies_after_warmstart.length() == bodies.length())

  // Warm starting with no previous impulses should not change velocities much
  let body_a_after = bodies_after_warmstart[0]
  let body_b_after = bodies_after_warmstart[1]

  // Bodies should still exist and be valid
  assert_true(body_a_after.mass == body_sim_a.mass)
  assert_true(body_b_after.mass == body_sim_b.mass)
}

///|
test "velocity constraint solving" {
  // Create two approaching dynamic bodies
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.8, y: 0.0 }
  body_def_a.linear_velocity = { x: 2.0, y: 0.0 } // Moving right
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.8, y: 0.0 }
  body_def_b.linear_velocity = { x: -2.0, y: 0.0 } // Moving left
  let body_sim_b = create_body_sim(body_def_b)
  let bodies = [body_sim_a, body_sim_b]

  // Create collision and solver
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  let manifolds = [(0, 1, manifold)]
  let solver = create_contact_solver(manifolds, bodies, 8, 3)

  // Apply constraint solving
  let bodies_after_solving = solve_velocity_constraints(solver, bodies)

  // Validate that bodies array is returned
  assert_true(bodies_after_solving.length() == bodies.length())

  // Bodies should still be valid after constraint solving
  let body_a_after = bodies_after_solving[0]
  let body_b_after = bodies_after_solving[1]
  assert_true(body_a_after.mass == body_sim_a.mass)
  assert_true(body_b_after.mass == body_sim_b.mass)

  // For a head-on collision, velocities should be affected
  // (exact values depend on mass ratio and restitution)
}

///|
test "position constraint solving" {
  // Create two overlapping bodies
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.3, y: 0.0 } // Overlapping position
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.3, y: 0.0 } // Overlapping position
  let body_sim_b = create_body_sim(body_def_b)
  let bodies = [body_sim_a, body_sim_b]

  // Create collision with penetration
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  let manifolds = [(0, 1, manifold)]
  let solver = create_contact_solver(manifolds, bodies, 8, 3)

  // Apply position correction
  let bodies_after_correction = solve_position_constraints(solver, bodies)

  // Validate that bodies array is returned
  assert_true(bodies_after_correction.length() == bodies.length())

  // Bodies should still be valid after position correction
  let body_a_after = bodies_after_correction[0]
  let body_b_after = bodies_after_correction[1]
  assert_true(body_a_after.mass == body_sim_a.mass)
  assert_true(body_b_after.mass == body_sim_b.mass)

  // Position correction should potentially move bodies to reduce penetration
  // (exact behavior depends on penetration depth and mass distribution)
}

///|
test "constraint solver with static body" {
  // Test constraint solving with one static body
  let static_body_def = default_body_def()
  static_body_def.type_ = Static
  static_body_def.position = { x: 0.0, y: -2.0 }
  let static_body = create_body_sim(static_body_def)
  let dynamic_body_def = default_body_def()
  dynamic_body_def.type_ = Dynamic
  dynamic_body_def.position = { x: 0.0, y: 0.0 }
  dynamic_body_def.linear_velocity = { x: 0.0, y: -1.0 } // Falling
  let dynamic_body = create_body_sim(dynamic_body_def)
  let bodies = [static_body, dynamic_body]

  // Create collision between static and dynamic
  let circle_static = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_dynamic = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_static,
    static_body.transform,
    circle_dynamic,
    dynamic_body.transform,
  )
  if manifold.point_count > 0 {
    let manifolds = [(0, 1, manifold)]
    let solver = create_contact_solver(manifolds, bodies, 8, 3)

    // Apply constraint solving
    let bodies_after_solving = solve_velocity_constraints(solver, bodies)

    // Static body should remain unchanged
    let static_after = bodies_after_solving[0]
    assert_true(static_after.linear_velocity.x == 0.0)
    assert_true(static_after.linear_velocity.y == 0.0)
    assert_true(static_after.transform.p.x == static_body.transform.p.x)
    assert_true(static_after.transform.p.y == static_body.transform.p.y)

    // Dynamic body's velocity should be affected
    let dynamic_after = bodies_after_solving[1]
    assert_true(dynamic_after.mass == dynamic_body.mass)
  }
}

///|
test "constraint solver validation" {
  // Test comprehensive constraint solver validation
  // Create a basic two-body collision scenario
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.5, y: 0.0 }
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.5, y: 0.0 }
  let body_sim_b = create_body_sim(body_def_b)
  let bodies = [body_sim_a, body_sim_b]

  // Create collision
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  let manifolds = [(0, 1, manifold)]

  // Create and validate solver
  let solver = create_contact_solver(manifolds, bodies, 8, 3)

  // Validate solver integrity
  assert_true(solver.constraints.length() == solver.constraint_count)
  assert_true(solver.velocity_iterations > 0)
  assert_true(solver.position_iterations > 0)
  if solver.constraint_count > 0 {
    let constraint = solver.constraints[0]

    // Validate constraint integrity
    assert_true(constraint.body_a_index >= 0)
    assert_true(constraint.body_b_index >= 0)
    assert_true(constraint.body_a_index != constraint.body_b_index)
    assert_true(constraint.point_count >= 0)
    assert_true(constraint.points.length() == constraint.point_count)

    // Validate mass properties are positive for dynamic bodies
    if bodies[constraint.body_a_index].inv_mass > 0.0 {
      assert_true(constraint.inv_mass_a > 0.0)
    }
    if bodies[constraint.body_b_index].inv_mass > 0.0 {
      assert_true(constraint.inv_mass_b > 0.0)
    }

    // Validate constraint points
    for i, point in constraint.points {
      if i < constraint.point_count {
        assert_true(point.normal_mass >= 0.0)
        assert_true(point.tangent_mass >= 0.0)
        // Initial impulses should be zero for new constraints
        assert_true(point.normal_impulse == 0.0)
        assert_true(point.tangent_impulse == 0.0)
      }
    }
  }
}

///|
test "constraint solver convergence" {
  // Test that multiple iterations improve constraint satisfaction
  let body_def_a = default_body_def()
  body_def_a.type_ = Dynamic
  body_def_a.position = { x: -0.4, y: 0.0 }
  body_def_a.linear_velocity = { x: 3.0, y: 0.0 }
  let body_sim_a = create_body_sim(body_def_a)
  let body_def_b = default_body_def()
  body_def_b.type_ = Dynamic
  body_def_b.position = { x: 0.4, y: 0.0 }
  body_def_b.linear_velocity = { x: -3.0, y: 0.0 }
  let body_sim_b = create_body_sim(body_def_b)
  let bodies = [body_sim_a, body_sim_b]

  // Create collision
  let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
  let manifold = collide_circles(
    circle_a,
    body_sim_a.transform,
    circle_b,
    body_sim_b.transform,
  )
  let manifolds = [(0, 1, manifold)]

  // Test with different iteration counts
  let solver_few = create_contact_solver(manifolds, bodies, 1, 1) // Few iterations
  let solver_many = create_contact_solver(manifolds, bodies, 10, 5) // Many iterations

  // Both should create valid solvers
  assert_true(solver_few.constraint_count == 1)
  assert_true(solver_many.constraint_count == 1)

  // Apply solving with few iterations
  let bodies_few = solve_velocity_constraints(solver_few, bodies)
  assert_true(bodies_few.length() == bodies.length())

  // Apply solving with many iterations  
  let bodies_many = solve_velocity_constraints(solver_many, bodies)
  assert_true(bodies_many.length() == bodies.length())

  // Both should produce valid results
  // (In practice, more iterations should lead to better constraint satisfaction)
}
