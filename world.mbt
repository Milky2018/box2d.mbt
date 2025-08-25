// Box2D World simulation - basic physics stepping
// World management and physics simulation loop

// Simple world representation
pub(all) struct World {
  world_id : Int
  gravity : Vec2
  bodies : Array[BodySim]
  time_step : Double
  velocity_iterations : Int
  position_iterations : Int
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
    position_iterations: 3
  }
}

// Add body to world
pub fn world_add_body(world : World, body_sim : BodySim) -> World {
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

// Resolve collision between two bodies
fn resolve_collision(body_a : BodySim, body_b : BodySim, manifold : Manifold) -> (BodySim, BodySim) {
  if manifold.point_count == 0 {
    return (body_a, body_b)
  }
  
  let contact_point = manifold.points[0].point
  let normal = manifold.normal
  
  // Relative velocity
  let rel_velocity = sub(body_b.linear_velocity, body_a.linear_velocity)
  let normal_velocity = dot(rel_velocity, normal)
  
  // Do not resolve if velocities are separating
  if normal_velocity > 0.0 {
    return (body_a, body_b)
  }
  
  // Restitution (bounce)
  let restitution = 0.2
  let impulse_magnitude = -(1.0 + restitution) * normal_velocity / (body_a.inv_mass + body_b.inv_mass)
  let impulse = mul_sv(impulse_magnitude, normal)
  
  // Apply impulse
  let new_body_a = body_apply_impulse(body_a, mul_sv(-1.0, impulse), contact_point)
  let new_body_b = body_apply_impulse(body_b, impulse, contact_point)
  
  (new_body_a, new_body_b)
}

// Step world simulation
pub fn step_world(world : World, dt : Double) -> World {
  // Apply gravity
  let bodies_with_gravity = apply_gravity(world.bodies, world.gravity, dt)
  
  // Velocity iterations
  let mut current_bodies = bodies_with_gravity
  for _iter = 0; _iter < world.velocity_iterations; _iter = _iter + 1 {
    let collisions = detect_collisions(current_bodies)
    let new_bodies = current_bodies.copy()
    
    for collision in collisions {
      let (i, j, manifold) = collision
      let (new_body_a, new_body_b) = resolve_collision(new_bodies[i], new_bodies[j], manifold)
      new_bodies[i] = new_body_a
      new_bodies[j] = new_body_b
    }
    
    current_bodies = new_bodies
  }
  
  // Integrate positions
  let final_bodies = integrate_bodies(current_bodies, dt)
  
  { ..world, bodies: final_bodies }
}

// Get body count in world
pub fn world_get_body_count(world : World) -> Int {
  world.bodies.length()
}

// Get body by index
pub fn world_get_body(world : World, index : Int) -> BodySim {
  world.bodies[index]
}