// Box2D Body Sleeping and Awakening System - ported from solver.c and solver_set.c
// Manages body sleep states for performance optimization

// Sleep constants (ported from Box2D constants.h)
let time_to_sleep = 0.5  // Time that a body must be still before it will go to sleep (seconds)

// Check if a body should be flagged as sleepy based on its current velocity (ported from b2FinalizeBodiesTask)
pub fn update_body_sleep_time(body : BodySim, dt : Double, enable_sleep : Bool) -> BodySim {
  if body.inv_mass == 0.0 {
    // Static bodies don't sleep
    return body
  }
  
  if !enable_sleep || !body.enable_sleep {
    // Sleep disabled - reset sleep time
    return { ..body, sleep_time: 0.0 }
  }
  
  // Calculate kinetic energy for sleep detection (ported from Box2D sleep logic)
  let linear_velocity_sq = length_squared(body.linear_velocity)
  
  // Sleep velocity calculation (includes both linear and angular motion)
  // Use simple velocity magnitude instead of complex math functions
  let linear_velocity_mag = if linear_velocity_sq > 0.0 { 
    // Simple approximation of sqrt using nested approximations
    let mut x = linear_velocity_sq
    x = 0.5 * (x + linear_velocity_sq / x)  // One Newton-Raphson iteration
    x = 0.5 * (x + linear_velocity_sq / x)  // Two iterations for better accuracy
    x
  } else { 
    0.0 
  }
  
  let angular_velocity_mag = if body.angular_velocity >= 0.0 { 
    body.angular_velocity 
  } else { 
    -body.angular_velocity 
  }
  
  let sleep_velocity = if linear_velocity_mag > angular_velocity_mag { 
    linear_velocity_mag 
  } else { 
    angular_velocity_mag 
  }
  
  if sleep_velocity > body.sleep_threshold {
    // Body is not sleepy - reset sleep time
    { ..body, sleep_time: 0.0 }
  } else {
    // Body is falling asleep - accumulate sleep time
    { ..body, sleep_time: body.sleep_time + dt }
  }
}

// Check if a body is ready to sleep (ported from Box2D TrySleepIsland logic)
pub fn body_is_ready_to_sleep(body : BodySim) -> Bool {
  if body.inv_mass == 0.0 {
    return false  // Static bodies don't sleep
  }
  
  if !body.enable_sleep {
    return false  // Sleep disabled
  }
  
  body.sleep_time >= time_to_sleep
}

// Wake up a sleeping body (ported from Box2D awakening logic)
pub fn wake_body(body : BodySim) -> BodySim {
  if body.inv_mass == 0.0 {
    return body  // Static bodies are never asleep
  }
  
  // Reset sleep time when awakening
  { ..body, sleep_time: 0.0 }
}

// Put a body to sleep (ported from Box2D TrySleepIsland)
pub fn sleep_body(body : BodySim) -> BodySim {
  if body.inv_mass == 0.0 {
    return body  // Static bodies don't need sleep state changes
  }
  
  // When sleeping, velocities are zeroed for stability
  {
    ..body,
    linear_velocity: { x: 0.0, y: 0.0 },
    angular_velocity: 0.0,
    force: { x: 0.0, y: 0.0 },
    torque: 0.0
  }
}

// Check if a body should wake up based on applied force (ported from Box2D wake-on-force logic)
pub fn should_wake_on_force(body : BodySim, force : Vec2, torque : Double) -> Bool {
  if body.inv_mass == 0.0 {
    return false  // Static bodies don't wake
  }
  
  if !body.enable_sleep {
    return false  // Already awake if sleep disabled
  }
  
  // Wake if significant force/torque is applied
  let force_magnitude_sq = length_squared(force)
  let torque_sq = torque * torque
  
  let force_threshold = body.sleep_threshold * body.mass
  let torque_threshold = body.sleep_threshold * body.inertia
  
  force_magnitude_sq > force_threshold * force_threshold ||
  torque_sq > torque_threshold * torque_threshold
}

// Check if a body should wake up based on applied impulse (ported from Box2D wake-on-impulse logic)
pub fn should_wake_on_impulse(body : BodySim, impulse : Vec2, angular_impulse : Double) -> Bool {
  if body.inv_mass == 0.0 {
    return false  // Static bodies don't wake
  }
  
  if !body.enable_sleep {
    return false  // Already awake if sleep disabled
  }
  
  // Calculate velocity change from impulse
  let linear_velocity_change = mul_sv(body.inv_mass, impulse)
  let angular_velocity_change = body.inv_inertia * angular_impulse
  
  // Wake if velocity change exceeds sleep threshold
  let linear_change_sq = length_squared(linear_velocity_change)
  
  // Simple magnitude calculations without complex math functions
  let linear_change_mag = if linear_change_sq > 0.0 { 
    let mut x = linear_change_sq
    x = 0.5 * (x + linear_change_sq / x)
    x = 0.5 * (x + linear_change_sq / x)
    x
  } else { 
    0.0 
  }
  
  let angular_change_mag = if angular_velocity_change >= 0.0 { 
    angular_velocity_change 
  } else { 
    -angular_velocity_change 
  }
  
  let velocity_change = if linear_change_mag > angular_change_mag { 
    linear_change_mag 
  } else { 
    angular_change_mag 
  }
  
  velocity_change > body.sleep_threshold
}

// Update sleep time for all bodies in an array (batch processing for performance)
pub fn update_bodies_sleep_time(bodies : Array[BodySim], dt : Double, enable_sleep : Bool) -> Array[BodySim] {
  let result = bodies.copy()
  for i, body in result {
    result[i] = update_body_sleep_time(body, dt, enable_sleep)
  }
  result
}

// Find bodies that are ready to sleep (for island management)
pub fn find_sleepy_bodies(bodies : Array[BodySim]) -> Array[Int] {
  let sleepy_indices = []
  for i, body in bodies {
    if body_is_ready_to_sleep(body) {
      sleepy_indices.push(i)
    }
  }
  sleepy_indices
}

// Wake up all bodies in an array (used when islands are disturbed)
pub fn wake_all_bodies(bodies : Array[BodySim]) -> Array[BodySim] {
  let result = bodies.copy()
  for i, body in result {
    result[i] = wake_body(body)
  }
  result
}

// Sleep all bodies in an array (used when islands go to sleep)
pub fn sleep_all_bodies(bodies : Array[BodySim]) -> Array[BodySim] {
  let result = bodies.copy()
  for i, body in result {
    result[i] = sleep_body(body)
  }
  result
}