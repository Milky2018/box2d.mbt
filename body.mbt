// Box2D Body management - ported from body.c
// Body creation, manipulation, and physics properties

// Internal body representation for simulation
pub(all) struct BodySim {
  transform : Transform
  center : Vec2
  linear_velocity : Vec2
  angular_velocity : Double
  force : Vec2
  torque : Double
  mass : Double
  inv_mass : Double
  inertia : Double
  inv_inertia : Double
  body_id : Int
  // Sleep-related fields (ported from Box2D body structure)
  sleep_threshold : Double  // Energy threshold for sleeping
  sleep_time : Double       // Time body has been below sleep threshold
  enable_sleep : Bool       // Whether this body can sleep
} derive(Eq, Show)

// Body state for dynamics
pub(all) struct BodyState {
  linear_velocity : Vec2
  angular_velocity : Double
  flags : Int
} derive(Eq, Show)

// MassData is now defined in types.mbt

// Note: body_sims and body_states storage removed as unused
// They can be added back when body simulation is implemented

// Get mass data for a circle (ported from b2ComputeCircleMass)
pub fn compute_circle_mass(circle : Circle, density : Double) -> MassData {
  let rr = circle.radius * circle.radius
  let mass = density * pi * rr
  // inertia about the center of mass
  let inertia = mass * 0.5 * rr
  { mass, center: circle.center, rotational_inertia: inertia }
}

// Get mass data for a polygon (ported from b2ComputePolygonMass)
pub fn compute_polygon_mass(polygon : Polygon, density : Double) -> MassData {
  if polygon.count < 3 {
    return { mass: 0.0, center: { x: 0.0, y: 0.0 }, rotational_inertia: 0.0 }
  }
  
  let mut center = { x: 0.0, y: 0.0 }
  let mut area = 0.0
  let mut rotational_inertia = 0.0
  
  // Get a reference point for forming triangles
  // Use the first vertex to reduce round-off errors
  let r = polygon.vertices[0]
  
  let inv3 = 1.0 / 3.0
  
  for i = 1; i < polygon.count - 1; i = i + 1 {
    // Triangle edges
    let e1 = sub(polygon.vertices[i], r)
    let e2 = sub(polygon.vertices[i + 1], r)
    
    let d = cross(e1, e2)
    
    let triangle_area = 0.5 * d
    area = area + triangle_area
    
    // Area weighted centroid, r at origin
    center = mul_add(center, triangle_area * inv3, add(e1, e2))
    
    let ex1 = e1.x
    let ey1 = e1.y
    let ex2 = e2.x  
    let ey2 = e2.y
    
    let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2
    let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2
    
    rotational_inertia = rotational_inertia + (0.25 * inv3 * d) * (intx2 + inty2)
  }
  
  // Total mass
  let mass = density * area
  
  // Center of mass, shift back from origin at r  
  let inv_area = 1.0 / area
  let final_center_rel = { x: center.x * inv_area, y: center.y * inv_area }
  let final_center = add(r, final_center_rel)
  
  // Inertia tensor relative to the local origin (point s)
  let mut final_inertia = density * rotational_inertia
  
  // Shift inertia to center of mass
  final_inertia = final_inertia - mass * dot(final_center_rel, final_center_rel)
  
  { mass, center: final_center, rotational_inertia: final_inertia }
}

// Get mass data for a capsule (ported from b2ComputeCapsuleMass)
pub fn compute_capsule_mass(capsule : Capsule, density : Double) -> MassData {
  let radius = capsule.radius
  let rr = radius * radius
  let p1 = capsule.center1
  let p2 = capsule.center2
  let length = distance(p2, p1)
  let ll = length * length
  
  let circle_mass = density * (pi * radius * radius)
  let box_mass = density * (2.0 * radius * length)
  
  let mass = circle_mass + box_mass
  let center = { x: 0.5 * (p1.x + p2.x), y: 0.5 * (p1.y + p2.y) }
  
  // two offset half circles, both halves add up to full circle and each half is offset by half length
  // semi-circle centroid = 4 r / 3 pi
  // Need to apply parallel-axis theorem twice:
  // 1. shift semi-circle centroid to origin  
  // 2. shift semi-circle to box end
  // m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
  // See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
  // I verified this formula by computing the convex hull of a 128 vertex capsule
  
  // half circle centroid
  let lc = 4.0 * radius / (3.0 * pi)
  
  // half length of rectangular portion of capsule
  let h = 0.5 * length
  
  let circle_inertia = circle_mass * (0.5 * rr + h * h + 2.0 * h * lc)
  let box_inertia = box_mass * (4.0 * rr + ll) / 12.0
  let rotational_inertia = circle_inertia + box_inertia
  
  { mass, center, rotational_inertia }
}

// Get mass data for a segment (no mass, since it's just a line)
pub fn compute_segment_mass(segment : Segment, _density : Double) -> MassData {
  let center = mul_sv(0.5, add(segment.point1, segment.point2))
  // Segments have no area, so no mass (density is unused since no area)
  { mass: 0.0, center, rotational_inertia: 0.0 }
}

// Apply force to body
pub fn body_apply_force(body_sim : BodySim, force : Vec2, point : Vec2) -> BodySim {
  if body_sim.inv_mass == 0.0 {
    return body_sim  // Static body
  }
  
  let new_force = add(body_sim.force, force)
  let r = sub(point, body_sim.center)
  let new_torque = body_sim.torque + cross(r, force)
  
  { ..body_sim, force: new_force, torque: new_torque }
}

// Apply impulse to body
pub fn body_apply_impulse(body_sim : BodySim, impulse : Vec2, point : Vec2) -> BodySim {
  if body_sim.inv_mass == 0.0 {
    return body_sim  // Static body
  }
  
  let new_linear_velocity = mul_add(body_sim.linear_velocity, body_sim.inv_mass, impulse)
  let r = sub(point, body_sim.center)
  let angular_impulse = cross(r, impulse)
  let new_angular_velocity = body_sim.angular_velocity + body_sim.inv_inertia * angular_impulse
  
  { ..body_sim, linear_velocity: new_linear_velocity, angular_velocity: new_angular_velocity }
}

// Integrate body position and rotation
pub fn integrate_body(body_sim : BodySim, dt : Double) -> BodySim {
  if body_sim.inv_mass == 0.0 {
    return body_sim  // Static body doesn't move
  }
  
  // Integrate velocity
  let acceleration = mul_sv(body_sim.inv_mass, body_sim.force)
  let new_linear_velocity = mul_add(body_sim.linear_velocity, dt, acceleration)
  let angular_acceleration = body_sim.inv_inertia * body_sim.torque
  let new_angular_velocity = body_sim.angular_velocity + dt * angular_acceleration
  
  // Integrate position
  let displacement = mul_sv(dt, new_linear_velocity)
  let new_position = add(body_sim.transform.p, displacement)
  let angle_change = dt * new_angular_velocity
  let new_rotation = mul_rot(body_sim.transform.q, make_rot(angle_change))
  
  let new_transform = { p: new_position, q: new_rotation }
  
  // Clear forces
  {
    ..body_sim,
    transform: new_transform,
    linear_velocity: new_linear_velocity,
    angular_velocity: new_angular_velocity,
    force: { x: 0.0, y: 0.0 },
    torque: 0.0
  }
}

// Set body mass data
pub fn body_set_mass_data(body_sim : BodySim, mass_data : MassData) -> BodySim {
  let inv_mass = if mass_data.mass > 0.0 { 1.0 / mass_data.mass } else { 0.0 }
  let inv_inertia = if mass_data.rotational_inertia > 0.0 { 1.0 / mass_data.rotational_inertia } else { 0.0 }
  
  {
    ..body_sim,
    mass: mass_data.mass,
    inv_mass,
    inertia: mass_data.rotational_inertia,
    inv_inertia,
    center: add(body_sim.transform.p, mass_data.center)
  }
}

// Get world point from local point
pub fn body_get_world_point(body_sim : BodySim, local_point : Vec2) -> Vec2 {
  transform_point(body_sim.transform, local_point)
}

// Get local point from world point
pub fn body_get_local_point(body_sim : BodySim, world_point : Vec2) -> Vec2 {
  inv_transform_point(body_sim.transform, world_point)
}

// Get world vector from local vector
pub fn body_get_world_vector(body_sim : BodySim, local_vector : Vec2) -> Vec2 {
  rotate_vector(body_sim.transform.q, local_vector)
}

// Get local vector from world vector
pub fn body_get_local_vector(body_sim : BodySim, world_vector : Vec2) -> Vec2 {
  inv_rotate_vector(body_sim.transform.q, world_vector)
}

// Create a default body simulation
pub fn create_body_sim(body_def : BodyDef) -> BodySim {
  let mass = if body_def.type_ == Dynamic { 1.0 } else { 0.0 }
  let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 }
  
  {
    transform: { p: body_def.position, q: body_def.rotation },
    center: body_def.position,
    linear_velocity: body_def.linear_velocity,
    angular_velocity: body_def.angular_velocity,
    force: { x: 0.0, y: 0.0 },
    torque: 0.0,
    mass,
    inv_mass,
    inertia: 1.0,
    inv_inertia: if mass > 0.0 { 1.0 } else { 0.0 },
    body_id: -1,  // Will be set when added to world
    // Sleep fields from BodyDef
    sleep_threshold: body_def.sleep_threshold,
    sleep_time: 0.0,  // Bodies start awake
    enable_sleep: body_def.enable_sleep
  }
}