// Box2D Joint System Tests - testing distance joints and joint infrastructure
// Tests for joint creation, constraint solving, and spring behavior

// Test distance joint definition creation
///|
test "default distance joint definition" {
  let def = default_distance_joint_def()

  // Check base joint properties
  assert_true(def.base.body_id_a.id == -1)
  assert_true(def.base.body_id_b.id == -1)
  assert_false(def.base.collide_connected)

  // Check distance joint specific properties
  assert_true(def.length == 1.0)
  assert_false(def.enable_spring)
  assert_true(def.hertz == 0.0)
  assert_true(def.damping_ratio == 0.0)
  assert_false(def.enable_limit)
  assert_true(def.min_length == 0.0)
  assert_true(def.max_length == 1.0e6)
  assert_false(def.enable_motor)
  assert_true(def.max_motor_force == 0.0)
  assert_true(def.motor_speed == 0.0)
}

// Test distance joint creation

///|
test "create distance joint simulation" {
  let base_def = default_distance_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 },
    },
    length: 2.0,
    enable_spring: true,
    hertz: 5.0,
    damping_ratio: 0.7,
  }
  let anchor_a : Vec2 = { x: 1.0, y: 0.0 }
  let anchor_b : Vec2 = { x: -1.0, y: 0.0 }
  let joint_sim = create_distance_joint_sim(def, anchor_a, anchor_b)

  // Verify joint properties
  assert_true(joint_sim.joint_type == DistanceJoint)
  assert_true(joint_sim.body_id_a == 0)
  assert_true(joint_sim.body_id_b == 1)

  // Verify distance joint specific properties
  let distance_joint = joint_sim.distance_joint
  assert_true(distance_joint.length == 2.0)
  assert_true(distance_joint.enable_spring)
  assert_true(distance_joint.hertz == 5.0)
  assert_true(distance_joint.damping_ratio == 0.7)

  // Verify initial solver state
  assert_true(distance_joint.impulse == 0.0)
  assert_true(distance_joint.axial_mass == 0.0)
}

// Test softness computation for spring joints

///|
test "joint softness computation" {
  let hertz = 10.0
  let damping_ratio = 0.5
  let time_step = 1.0 / 60.0
  let softness = make_soft(hertz, damping_ratio, time_step)

  // Softness should produce valid constraint parameters
  assert_true(softness.bias_rate >= 0.0)
  assert_true(softness.mass_scale > 0.0 && softness.mass_scale <= 1.0)
  assert_true(softness.impulse_scale >= 0.0)

  // Zero frequency should produce rigid constraint
  let rigid_softness = make_soft(0.0, 0.0, time_step)
  assert_true(rigid_softness.bias_rate == 0.0)
  assert_true(rigid_softness.mass_scale == 1.0)
  assert_true(rigid_softness.impulse_scale == 0.0)
}


// Test distance joint constraint preparation

///|
test "distance joint preparation" {
  // Create two bodies
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 },
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 2.0, y: 0.0 },
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create distance joint
  let base_def = default_distance_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 },
    },
    length: 2.0,
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 }
  let joint_sim = create_distance_joint_sim(def, anchor_a, anchor_b)

  // Prepare joint for solving
  let time_step = 1.0 / 60.0
  let prepared_joint = prepare_distance_joint(joint_sim, bodies, time_step)

  // Verify preparation updated solver data
  let distance_joint = prepared_joint.distance_joint
  assert_true(distance_joint.index_a == 0)
  assert_true(distance_joint.index_b == 1)
  assert_true(distance_joint.axial_mass > 0.0) // Should compute effective mass
}

// Test distance joint warm starting

///|
test "distance joint warm starting" {
  // Create two bodies with some velocity
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 },
    linear_velocity: Vec2::{ x: 1.0, y: 0.0 }
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 2.0, y: 0.0 },
    linear_velocity: Vec2::{ x: -1.0, y: 0.0 }
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create and prepare joint
  let base_def = default_distance_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 }
    },
    length: 2.0
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 }
  let joint_sim = create_distance_joint_sim(def, anchor_a, anchor_b)
  let prepared_joint = prepare_distance_joint(joint_sim, bodies, 1.0 / 60.0)

  // Apply some impulse for warm starting
  let distance_joint = prepared_joint.distance_joint
  let distance_joint_with_impulse = { ..distance_joint, impulse: 10.0 } // Previous frame impulse  
  let joint_with_impulse = { ..prepared_joint, distance_joint: distance_joint_with_impulse }

  // Warm start should modify body velocities (use prepared joint with proper inverse masses)
  let warmed_bodies = warm_start_distance_joint(joint_with_impulse, bodies)

  // Bodies should have different velocities after warm starting (only if constraint violation exists)
  // Note: Warm starting applies accumulated impulse from previous frame
  // With 10.0 impulse and bodies at correct distance, some velocity change should occur
  let vel_change_a = warmed_bodies[0].linear_velocity.x - bodies[0].linear_velocity.x
  let vel_change_b = warmed_bodies[1].linear_velocity.x - bodies[1].linear_velocity.x
  let vel_change_abs_a = if vel_change_a >= 0.0 { vel_change_a } else { -vel_change_a }
  let vel_change_abs_b = if vel_change_b >= 0.0 { vel_change_b } else { -vel_change_b }
  
  // Should have some velocity change due to warm starting impulse
  assert_true(vel_change_abs_a > 1.0e-10 || vel_change_abs_b > 1.0e-10)
}

// Test distance joint velocity constraint solving

///|
test "distance joint constraint solving" {
  // Create two bodies with significant constraint violation and velocities that violate constraint
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 },
    linear_velocity: Vec2::{ x: -5.0, y: 0.0 } // Moving toward each other (violating distance constraint)
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 1.0, y: 0.0 }, // Much closer than desired 2.0 distance
    linear_velocity: Vec2::{ x: 5.0, y: 0.0 } // Moving toward each other (violating distance constraint)
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create rigid distance joint (no spring)
  let base_def = default_distance_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 }
    },
    length: 2.0, // Desired length
    enable_spring: false // Rigid constraint
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 }
  let joint_sim = create_distance_joint_sim(def, anchor_a, anchor_b)
  let prepared_joint = prepare_distance_joint(joint_sim, bodies, 1.0 / 60.0)

  // Solve velocity constraints
  let time_step = 1.0 / 60.0
  let (solved_joint, solved_bodies) = solve_distance_joint_velocity(
    prepared_joint, bodies, time_step, true,
  )

  // Constraint solver should have applied impulse to maintain distance
  let distance_joint = solved_joint.distance_joint
  let impulse_magnitude = if distance_joint.impulse >= 0.0 { distance_joint.impulse } else { -distance_joint.impulse }
  
  assert_true(impulse_magnitude > 1.0e-10) // Should have accumulated some impulse

  // Bodies should have modified velocities to maintain constraint
  let relative_velocity_before = sub(
    bodies[1].linear_velocity,
    bodies[0].linear_velocity,
  )
  let relative_velocity_after = sub(
    solved_bodies[1].linear_velocity,
    solved_bodies[0].linear_velocity,
  )

  // The constraint solver should reduce relative motion along constraint axis
  assert_true(
    length_squared(relative_velocity_after) <
    length_squared(relative_velocity_before),
  )
}

// Test distance joint length measurement

///|
test "distance joint length measurement" {
  // Create two bodies at known positions
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 }
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 3.0, y: 4.0 } // Distance = 5.0
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create joint with anchors at body centers
  let base_def = default_distance_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 }
    }
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 } // At body center
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 } // At body center
  let joint_sim = create_distance_joint_sim(def, anchor_a, anchor_b)

  // Measure current joint length
  let measured_length = get_distance_joint_length(joint_sim, bodies)

  // Should measure the distance between body positions (3,4,5 triangle)
  let expected_length = 5.0
  let tolerance = 1.0e-6
  let length_diff = measured_length - expected_length
  let length_diff_abs = if length_diff >= 0.0 { length_diff } else { -length_diff }
  assert_true(length_diff_abs < tolerance)
}

// Test spring joint behavior

///|
test "distance joint spring behavior" {
  // Create two bodies
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 }
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 1.0, y: 0.0 } // Compressed spring
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create spring joint
  let base_def = default_distance_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 }
    },
    length: 2.0, // Rest length
    enable_spring: true,
    hertz: 10.0, // Spring frequency
    damping_ratio: 0.3
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 }
  let joint_sim = create_distance_joint_sim(def, anchor_a, anchor_b)
  let prepared_joint = prepare_distance_joint(joint_sim, bodies, 1.0 / 60.0)

  // Spring constraint should be soft (not rigid)
  let distance_joint = prepared_joint.distance_joint
  assert_true(distance_joint.enable_spring)
  assert_true(distance_joint.distance_softness.bias_rate > 0.0)
  assert_true(distance_joint.distance_softness.mass_scale < 1.0) // Soft constraint
}

// Test revolute joint definition creation
///|
test "default revolute joint definition" {
  let def = default_revolute_joint_def()

  // Check base joint properties
  assert_true(def.base.body_id_a.id == -1)
  assert_true(def.base.body_id_b.id == -1)
  assert_false(def.base.collide_connected)

  // Check revolute joint specific properties
  assert_true(def.target_angle == 0.0)
  assert_false(def.enable_spring)
  assert_true(def.hertz == 0.0)
  assert_true(def.damping_ratio == 0.0)
  assert_false(def.enable_limit)
  assert_true(def.lower_angle == 0.0)
  assert_true(def.upper_angle == 0.0)
  assert_false(def.enable_motor)
  assert_true(def.max_motor_torque == 0.0)
  assert_true(def.motor_speed == 0.0)
}

// Test revolute joint creation
///|
test "create revolute joint simulation" {
  let base_def = default_revolute_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 },
    },
    enable_spring: true,
    hertz: 10.0,
    damping_ratio: 0.5,
    target_angle: 1.57, // π/2 radians
  }
  let anchor : Vec2 = { x: 0.5, y: 0.5 }
  let joint_sim = create_revolute_joint_sim(def, anchor)

  // Verify joint properties
  assert_true(joint_sim.joint_type == RevoluteJoint)
  assert_true(joint_sim.body_id_a == 0)
  assert_true(joint_sim.body_id_b == 1)

  // Verify revolute joint specific properties
  let revolute_joint = joint_sim.revolute_joint
  assert_true(revolute_joint.enable_spring)
  assert_true(revolute_joint.hertz == 10.0)
  assert_true(revolute_joint.damping_ratio == 0.5)
  assert_true(revolute_joint.target_angle == 1.57)

  // Verify initial solver state
  assert_true(revolute_joint.linear_impulse.x == 0.0)
  assert_true(revolute_joint.linear_impulse.y == 0.0)
  assert_true(revolute_joint.spring_impulse == 0.0)
  assert_true(revolute_joint.axial_mass == 0.0)
}

// Test revolute joint constraint preparation
///|
test "revolute joint preparation" {
  // Create two bodies
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 },
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 1.0, y: 0.0 },
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create revolute joint at origin
  let base_def = default_revolute_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 },
    },
  }
  let anchor : Vec2 = { x: 0.0, y: 0.0 }
  let joint_sim = create_revolute_joint_sim(def, anchor)

  // Prepare joint for solving
  let time_step = 1.0 / 60.0
  let prepared_joint = prepare_revolute_joint(joint_sim, bodies, time_step)

  // Verify preparation updated solver data
  let revolute_joint = prepared_joint.revolute_joint
  assert_true(revolute_joint.index_a == 0)
  assert_true(revolute_joint.index_b == 1)
  assert_true(revolute_joint.axial_mass > 0.0) // Should compute effective rotational mass
}

// Test prismatic joint definition creation
///|
test "default prismatic joint definition" {
  let def = default_prismatic_joint_def()

  // Check base joint properties
  assert_true(def.base.body_id_a.id == -1)
  assert_true(def.base.body_id_b.id == -1)
  assert_false(def.base.collide_connected)

  // Check prismatic joint specific properties
  assert_true(def.target_translation == 0.0)
  assert_false(def.enable_spring)
  assert_true(def.hertz == 0.0)
  assert_true(def.damping_ratio == 0.0)
  assert_false(def.enable_limit)
  assert_true(def.lower_translation == 0.0)
  assert_true(def.upper_translation == 0.0)
  assert_false(def.enable_motor)
  assert_true(def.max_motor_force == 0.0)
  assert_true(def.motor_speed == 0.0)
}

// Test prismatic joint creation
///|
test "create prismatic joint simulation" {
  let base_def = default_prismatic_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 },
    },
    enable_spring: true,
    hertz: 8.0,
    damping_ratio: 0.6,
    target_translation: 1.0,
  }
  let anchor_a : Vec2 = { x: 0.5, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.5, y: 0.0 }
  let axis_a : Vec2 = { x: 1.0, y: 0.0 }  // Horizontal sliding axis
  let joint_sim = create_prismatic_joint_sim(def, anchor_a, anchor_b, axis_a)

  // Verify joint properties
  assert_true(joint_sim.joint_type == PrismaticJoint)
  assert_true(joint_sim.body_id_a == 0)
  assert_true(joint_sim.body_id_b == 1)

  // Verify prismatic joint specific properties
  let prismatic_joint = joint_sim.prismatic_joint
  assert_true(prismatic_joint.enable_spring)
  assert_true(prismatic_joint.hertz == 8.0)
  assert_true(prismatic_joint.damping_ratio == 0.6)
  assert_true(prismatic_joint.target_translation == 1.0)

  // Verify initial solver state
  assert_true(prismatic_joint.impulse.x == 0.0)
  assert_true(prismatic_joint.impulse.y == 0.0)
  assert_true(prismatic_joint.spring_impulse == 0.0)
  assert_true(prismatic_joint.axial_mass == 0.0)
}

// Test prismatic joint constraint preparation
///|
test "prismatic joint preparation" {
  // Create two bodies
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 },
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 1.0, y: 0.0 },
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create prismatic joint with horizontal sliding axis
  let base_def = default_prismatic_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 },
    },
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 }
  let axis_a : Vec2 = { x: 1.0, y: 0.0 }  // Horizontal axis
  let joint_sim = create_prismatic_joint_sim(def, anchor_a, anchor_b, axis_a)

  // Prepare joint for solving
  let time_step = 1.0 / 60.0
  let prepared_joint = prepare_prismatic_joint(joint_sim, bodies, time_step)

  // Verify preparation updated solver data
  let prismatic_joint = prepared_joint.prismatic_joint
  assert_true(prismatic_joint.index_a == 0)
  assert_true(prismatic_joint.index_b == 1)
  assert_true(prismatic_joint.axial_mass > 0.0) // Should compute effective translational mass
}

// Test prismatic joint warm starting
///|
test "prismatic joint warm starting" {
  // Create two bodies with some velocity
  let base_def_a = default_body_def()
  let body_def_a = {
    ..base_def_a,
    type_: Dynamic,
    position: Vec2::{ x: 0.0, y: 0.0 },
    linear_velocity: Vec2::{ x: 0.0, y: 1.0 }  // Moving perpendicular to axis
  }
  let base_def_b = default_body_def()
  let body_def_b = {
    ..base_def_b,
    type_: Dynamic,
    position: Vec2::{ x: 1.0, y: 0.0 },
    linear_velocity: Vec2::{ x: 0.0, y: -1.0 } // Moving opposite direction
  }
  let body_a = create_body_sim(body_def_a)
  let body_b = create_body_sim(body_def_b)
  let bodies = [body_a, body_b]

  // Create and prepare joint with horizontal axis
  let base_def = default_prismatic_joint_def()
  let def = {
    ..base_def,
    base: {
      ..base_def.base,
      body_id_a: BodyId::{ id: 0 },
      body_id_b: BodyId::{ id: 1 }
    }
  }
  let anchor_a : Vec2 = { x: 0.0, y: 0.0 }
  let anchor_b : Vec2 = { x: 0.0, y: 0.0 }
  let axis_a : Vec2 = { x: 1.0, y: 0.0 }
  let joint_sim = create_prismatic_joint_sim(def, anchor_a, anchor_b, axis_a)
  let prepared_joint = prepare_prismatic_joint(joint_sim, bodies, 1.0 / 60.0)

  // Apply some accumulated impulse for warm starting
  let prismatic_joint = prepared_joint.prismatic_joint
  let prismatic_joint_with_impulse = {
    ..prismatic_joint,
    impulse: { x: 5.0, y: 2.0 },  // Previous frame impulse for perpendicular and angular constraints
    spring_impulse: 3.0
  }
  let joint_with_impulse = { ..prepared_joint, prismatic_joint: prismatic_joint_with_impulse }

  // Warm start should modify body velocities
  let warmed_bodies = warm_start_prismatic_joint(joint_with_impulse, bodies)

  // Bodies should have different velocities after warm starting
  let vel_change_a_x = warmed_bodies[0].linear_velocity.x - bodies[0].linear_velocity.x
  let vel_change_b_x = warmed_bodies[1].linear_velocity.x - bodies[1].linear_velocity.x
  let vel_change_a_y = warmed_bodies[0].linear_velocity.y - bodies[0].linear_velocity.y
  let vel_change_b_y = warmed_bodies[1].linear_velocity.y - bodies[1].linear_velocity.y
  
  // Should have some velocity change due to warm starting impulse
  let total_change = abs_float(vel_change_a_x) + abs_float(vel_change_b_x) + 
                     abs_float(vel_change_a_y) + abs_float(vel_change_b_y)
  assert_true(total_change > 1.0e-10)
}
