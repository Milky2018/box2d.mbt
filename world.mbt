// Box2D World simulation - basic physics stepping
// World management and physics simulation loop

// Simple world representation with islands system and contact events
struct World {
  world_id : Int
  gravity : Vec2
  bodies : Array[BodySim]
  time_step : Double
  velocity_iterations : Int
  position_iterations : Int
  island_manager : IslandManager
  sleep_threshold : Double  // Energy threshold for body sleeping
  enable_sleep : Bool       // Whether sleep is enabled for this world
  contact_event_manager : ContactEventManager  // Contact event system
} derive(Eq, Show)

// World storage - currently unused, can be added back when needed
// let worlds : Ref[Array[World]] = { val: [] }

// Create a simple world from definition
pub fn create_world_from_def(def : WorldDef) -> World {
  {
    world_id: 0,  // Placeholder
    gravity: def.gravity,
    bodies: [],
    time_step: 1.0 / 60.0,  // 60 FPS default
    velocity_iterations: 8,
    position_iterations: 3,
    island_manager: create_island_manager(),
    sleep_threshold: 0.01,  // Default sleep energy threshold
    enable_sleep: def.enable_sleep,
    contact_event_manager: create_contact_event_manager()
  }
}

// Add body to world
fn world_add_body(world : World, body_sim : BodySim) -> World {
  let new_bodies = world.bodies.copy()
  new_bodies.push(body_sim)
  { ..world, bodies: new_bodies }
}

// Apply gravity to all dynamic bodies
fn apply_gravity(bodies : Array[BodySim], gravity : Vec2, _dt : Double) -> Array[BodySim] {
  let result = bodies.copy()
  for i, body in result {
    if body.inv_mass > 0.0 {  // Dynamic body
      let gravity_force = mul_sv(body.mass, gravity)
      let new_body = body_apply_force(body, gravity_force, body.center)
      result[i] = new_body
    }
  }
  result
}

// Integrate all bodies
fn integrate_bodies(bodies : Array[BodySim], dt : Double) -> Array[BodySim] {
  let result = bodies.copy()
  for i, body in result {
    result[i] = integrate_body(body, dt)
  }
  result
}

// Simple collision detection between all bodies
fn detect_collisions(bodies : Array[BodySim]) -> Array[(Int, Int, Manifold)] {
  let collisions = []
  
  for i, body_a in bodies {
    for j, body_b in bodies {
      if j <= i {  // Only check each pair once and skip self-collision
        continue
      }
      
      // Skip if both bodies are static
      if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
        continue
      }
      
      // Apply collision filtering
      if not(should_collide(body_a.collision_filter, body_b.collision_filter)) {
        continue
      }
      
      // For now, treat all bodies as unit circles for collision
      let circle_a = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
      let circle_b = { center: { x: 0.0, y: 0.0 }, radius: 1.0 }
      
      let manifold = collide_circles(circle_a, body_a.transform, circle_b, body_b.transform)
      
      if manifold.point_count > 0 {
        collisions.push((i, j, manifold))
      }
    }
  }
  
  collisions
}

// Contact constraint solving using proper sequential impulse method
fn solve_contacts_with_constraints(
  collisions : Array[(Int, Int, Manifold)],
  bodies : Array[BodySim],
  velocity_iterations : Int,
  position_iterations : Int
) -> Array[BodySim] {
  if collisions.length() == 0 {
    return bodies
  }
  
  // Create contact solver from collisions - use default 60fps timestep for now
  let timestep = 1.0 / 60.0
  let solver = create_contact_solver(collisions, bodies, velocity_iterations, position_iterations, timestep)
  
  if solver.constraint_count == 0 {
    return bodies
  }
  
  // Warm start constraints with previous frame impulses
  let bodies_after_warmstart = warm_start_constraints(solver, bodies)
  
  // Solve velocity constraints iteratively
  let mut current_bodies = bodies_after_warmstart
  for _iter = 0; _iter < velocity_iterations; _iter = _iter + 1 {
    current_bodies = solve_velocity_constraints(solver, current_bodies)
  }
  
  // Solve position constraints for stability
  for _iter = 0; _iter < position_iterations; _iter = _iter + 1 {
    current_bodies = solve_position_constraints(solver, current_bodies)
  }
  
  current_bodies
}

// Step world simulation with islands-based physics and sleeping
pub fn step_world(world : World, dt : Double) -> World {
  // Apply gravity
  let bodies_with_gravity = apply_gravity(world.bodies, world.gravity, dt)
  
  // Update body sleep times based on their current velocities
  let bodies_with_sleep_time = update_bodies_sleep_time(bodies_with_gravity, dt, world.enable_sleep)
  
  // Detect all collisions for this step
  let collisions = detect_collisions(bodies_with_sleep_time)
  
  // Process contact events (begin/end contact detection)
  let (updated_event_manager, contact_events) = process_contact_events(world.contact_event_manager, collisions)
  
  // Fire begin/end contact events
  fire_contact_events(updated_event_manager, contact_events)
  
  // Generate and fire pre-solve events
  let pre_solve_events = generate_pre_solve_events(updated_event_manager, collisions)
  fire_contact_events(updated_event_manager, pre_solve_events)
  
  // Build islands from current collisions and contacts
  let updated_island_manager = build_islands_from_collisions(collisions, bodies_with_sleep_time)
  
  // Solve constraints using islands system for better performance
  let bodies_after_constraints = solve_islands(
    updated_island_manager,
    bodies_with_sleep_time,
    collisions,
    world.velocity_iterations,
    world.position_iterations
  )
  
  // Integrate positions
  let bodies_after_integration = integrate_bodies(bodies_after_constraints, dt)
  
  // Process island sleeping (put sleepy islands to sleep)
  let (final_island_manager, final_bodies) = process_island_sleeping(
    updated_island_manager, 
    bodies_after_integration, 
    world.enable_sleep
  )
  
  { ..world, bodies: final_bodies, island_manager: final_island_manager, contact_event_manager: updated_event_manager }
}

// Get body count in world
pub fn world_get_body_count(world : World) -> Int {
  world.bodies.length()
}

// Get body by index
pub fn world_get_body(world : World, index : Int) -> BodySim {
  world.bodies[index]
}

// Get island statistics for the world
pub fn world_get_island_stats(world : World) -> (Int, Int, Int) {
  get_island_stats(world.island_manager)
}

// Check if world has any active islands
pub fn world_has_active_islands(world : World) -> Bool {
  let (_, _, awake_islands) = world_get_island_stats(world)
  awake_islands > 0
}

// Contact event callback management

// Set begin contact callback for world
pub fn world_set_begin_contact_callback(world : World, callback : ContactEventCallback) -> World {
  let updated_manager = set_begin_contact_callback(world.contact_event_manager, callback)
  { ..world, contact_event_manager: updated_manager }
}

// Set end contact callback for world
pub fn world_set_end_contact_callback(world : World, callback : ContactEventCallback) -> World {
  let updated_manager = set_end_contact_callback(world.contact_event_manager, callback)
  { ..world, contact_event_manager: updated_manager }
}

// Set pre-solve callback for world
pub fn world_set_pre_solve_callback(world : World, callback : ContactEventCallback) -> World {
  let updated_manager = set_pre_solve_callback(world.contact_event_manager, callback)
  { ..world, contact_event_manager: updated_manager }
}

// Set post-solve callback for world
pub fn world_set_post_solve_callback(world : World, callback : ContactEventCallback) -> World {
  let updated_manager = set_post_solve_callback(world.contact_event_manager, callback)
  { ..world, contact_event_manager: updated_manager }
}