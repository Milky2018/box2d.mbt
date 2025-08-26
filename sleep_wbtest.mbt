// Box2D Body Sleeping System Tests - testing sleep states and transitions
// Tests for body sleep detection, island sleeping, and awakening

///|
test "body sleep time update - awake body" {
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.linear_velocity = { x: 2.0, y: 1.0 }  // Above sleep threshold
  body_def.angular_velocity = 1.0
  body_def.sleep_threshold = 0.5
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  let dt = 1.0 / 60.0  // 60 FPS
  
  let updated_body = update_body_sleep_time(body, dt, true)
  
  // High velocity body should have sleep time reset to 0
  assert_true(updated_body.sleep_time == 0.0)
}

///|
test "body sleep time update - sleepy body" {
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.linear_velocity = { x: 0.01, y: 0.01 }  // Below sleep threshold
  body_def.angular_velocity = 0.01
  body_def.sleep_threshold = 0.5
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  let dt = 1.0 / 60.0
  
  let updated_body = update_body_sleep_time(body, dt, true)
  
  // Low velocity body should accumulate sleep time
  assert_true(updated_body.sleep_time > 0.0)
  assert_true(updated_body.sleep_time == dt)
}

///|
test "body ready to sleep check" {
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  
  // Body with insufficient sleep time
  let not_ready_body = { ..body, sleep_time: 0.1 }
  assert_false(body_is_ready_to_sleep(not_ready_body))
  
  // Body with sufficient sleep time  
  let ready_body = { ..body, sleep_time: 1.0 }  // > time_to_sleep (0.5)
  assert_true(body_is_ready_to_sleep(ready_body))
}

///|
test "wake body function" {
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  let sleepy_body = { ..body, sleep_time: 1.0 }
  
  let awake_body = wake_body(sleepy_body)
  
  // Waking should reset sleep time
  assert_true(awake_body.sleep_time == 0.0)
}

///|
test "sleep body function" {
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.linear_velocity = { x: 5.0, y: 3.0 }
  body_def.angular_velocity = 2.0
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  let moving_body = body_apply_force(body, { x: 10.0, y: 5.0 }, body.center)
  
  let sleeping_body = sleep_body(moving_body)
  
  // Sleeping should zero velocities and forces
  assert_true(length_squared(sleeping_body.linear_velocity) == 0.0)
  assert_true(sleeping_body.angular_velocity == 0.0)
  assert_true(length_squared(sleeping_body.force) == 0.0)
  assert_true(sleeping_body.torque == 0.0)
}

///|
test "should wake on force" {
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.sleep_threshold = 0.5
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  
  // Small force - should not wake
  let small_force : Vec2 = { x: 0.1, y: 0.1 }
  assert_false(should_wake_on_force(body, small_force, 0.1))
  
  // Large force - should wake
  let large_force : Vec2 = { x: 10.0, y: 5.0 }
  assert_true(should_wake_on_force(body, large_force, 0.0))
}

///|
test "world step with sleep system" {
  // Create world with sleep enabled and no gravity
  let world_def = default_world_def()
  world_def.enable_sleep = true
  world_def.gravity = { x: 0.0, y: 0.0 }  // No gravity to avoid acceleration
  let world = create_world_from_def(world_def)
  
  // Add a slow-moving body that should become sleepy
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.linear_velocity = { x: 0.01, y: 0.01 }
  body_def.sleep_threshold = 0.5
  body_def.enable_sleep = true
  
  let body = create_body_sim(body_def)
  let world_with_body = world_add_body(world, body)
  
  // Step multiple times to accumulate sleep time
  let dt = 1.0 / 60.0
  let mut stepped_world = world_with_body
  
  for _i = 0; _i < 30; _i = _i + 1 {  // 0.5 second at 60 FPS
    stepped_world = step_world(stepped_world, dt)
  }
  
  // Body should have accumulated sleep time since it has low velocity and no gravity
  assert_true(stepped_world.bodies[0].sleep_time > 0.1)
  
  // World should still function correctly
  assert_true(world_get_body_count(stepped_world) == 1)
}