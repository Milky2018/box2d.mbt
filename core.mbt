// Box2D Core API - ported from core.c and box2d.h

// Storage for worlds, bodies, and shapes
///|
let worlds_storage : Ref[Array[(WorldId, WorldDef)]] = { val: [] }

///|
let physics_worlds_storage : Ref[Array[(WorldId, World)]] = { val: [] }

///|
let bodies_storage : Ref[Array[(BodyId, WorldId, BodyDef)]] = { val: [] }

///|
let body_sims_storage : Ref[Array[(BodyId, BodySim)]] = { val: [] }

///|
let shapes_storage : Ref[Array[(ShapeId, BodyId, ShapeDef, ShapeType)]] = {
  val: [],
}

// ID generators

///|
let next_world_id : Ref[Int] = { val: 0 }

///|
let next_body_id : Ref[Int] = { val: 0 }

///|
let next_shape_id : Ref[Int] = { val: 0 }

// Default constructors (ported from types.c)

///|
pub fn default_world_def() -> WorldDef {
  {
    gravity: { x: 0.0, y: -10.0 },
    restitution_threshold: 1.0, // 1.0 * b2_lengthUnitsPerMeter
    hit_event_threshold: 1.0,
    contact_hertz: 30.0,
    contact_damping_ratio: 10.0,
    maximum_linear_speed: 400.0, // 400.0 * b2_lengthUnitsPerMeter
    enable_sleep: true,
    enable_continuous: true,
  }
}

///|
pub fn default_body_def() -> BodyDef {
  {
    type_: Static,
    position: { x: 0.0, y: 0.0 },
    rotation: rot_identity,
    linear_velocity: { x: 0.0, y: 0.0 },
    angular_velocity: 0.0,
    linear_damping: 0.0,
    angular_damping: 0.0,
    gravity_scale: 1.0,
    sleep_threshold: 0.05, // 0.05 * b2_lengthUnitsPerMeter
    enable_sleep: true,
    is_awake: true,
    is_enabled: true,
  }
}

///|
pub fn default_shape_def() -> ShapeDef {
  { density: 1.0, friction: 0.6, restitution: 0.0, is_sensor: false }
}

// Shape helpers

///|
pub fn make_box(half_width : Double, half_height : Double) -> Polygon {
  make_polygon_box(half_width, half_height)
}

// ID validation

///|
pub fn world_is_valid(id : WorldId) -> Bool {
  id.id >= 0
}

///|
pub fn body_is_valid(id : BodyId) -> Bool {
  id.id >= 0
}

///|
pub fn shape_is_valid(id : ShapeId) -> Bool {
  id.id >= 0
}

// World creation and management (ported from b2CreateWorld)

///|
pub fn create_world(def : WorldDef) -> WorldId {
  let id = next_world_id.val
  next_world_id.val = next_world_id.val + 1
  let world_id = WorldId::{ id, }

  // Store the world definition
  worlds_storage.val.push((world_id, def))
  
  // Create the physics world
  let physics_world = create_world_from_def(def)
  physics_worlds_storage.val.push((world_id, physics_world))
  
  world_id
}

// Body creation (ported from b2CreateBody)

///|
pub fn create_body(world_id : WorldId, def : BodyDef) -> BodyId {
  if world_is_valid(world_id) {
    let id = next_body_id.val
    next_body_id.val = next_body_id.val + 1
    let body_id = BodyId::{ id, }

    // Store the body definition
    bodies_storage.val.push((body_id, world_id, def))
    
    // Create body simulation and store it
    let body_sim = create_body_sim(def)
    body_sims_storage.val.push((body_id, body_sim))
    
    // Add body to the physics world
    let physics_worlds = physics_worlds_storage.val
    for i, entry in physics_worlds {
      let (world_id_stored, world) = entry
      if world_id_stored == world_id {
        let updated_world = world_add_body(world, body_sim)
        physics_worlds[i] = (world_id_stored, updated_world)
        break
      }
    }
    
    body_id
  } else {
    BodyId::{ id: -1 }
  }
}

// Shape creation (ported from b2CreateCircleShape)

///|
pub fn create_circle_shape(
  body_id : BodyId,
  def : ShapeDef,
  _circle : Circle,
) -> ShapeId {
  if body_is_valid(body_id) {
    let id = next_shape_id.val
    next_shape_id.val = next_shape_id.val + 1
    let shape_id = ShapeId::{ id, }

    // Store the shape with its definition and body association
    shapes_storage.val.push((shape_id, body_id, def, Circle))
    shape_id
  } else {
    ShapeId::{ id: -1 }
  }
}

///|
pub fn create_polygon_shape(
  body_id : BodyId,
  def : ShapeDef,
  _polygon : Polygon,
) -> ShapeId {
  if body_is_valid(body_id) {
    let id = next_shape_id.val
    next_shape_id.val = next_shape_id.val + 1
    let shape_id = ShapeId::{ id, }

    // Store the shape with its definition and body association
    shapes_storage.val.push((shape_id, body_id, def, Polygon))
    shape_id
  } else {
    ShapeId::{ id: -1 }
  }
}

// Create a capsule shape

///|
pub fn create_capsule_shape(
  body_id : BodyId,
  def : ShapeDef,
  _capsule : Capsule,
) -> ShapeId {
  if body_is_valid(body_id) {
    let id = next_shape_id.val
    next_shape_id.val = next_shape_id.val + 1
    let shape_id = ShapeId::{ id, }

    // Store the shape with its definition and body association
    shapes_storage.val.push((shape_id, body_id, def, Capsule))
    shape_id
  } else {
    ShapeId::{ id: -1 }
  }
}

// Create a segment shape

///|
pub fn create_segment_shape(
  body_id : BodyId,
  def : ShapeDef,
  _segment : Segment,
) -> ShapeId {
  if body_is_valid(body_id) {
    let id = next_shape_id.val
    next_shape_id.val = next_shape_id.val + 1
    let shape_id = ShapeId::{ id, }

    // Store the shape with its definition and body association
    shapes_storage.val.push((shape_id, body_id, def, Segment))
    shape_id
  } else {
    ShapeId::{ id: -1 }
  }
}

// Data retrieval functions

// Find world definition by ID

///|
pub fn get_world_def(world_id : WorldId) -> WorldDef? {
  let worlds = worlds_storage.val
  for entry in worlds {
    let (stored_id, def) = entry
    if stored_id.id == world_id.id {
      return Some(def)
    }
  }
  None
}

// Find body definition by ID

///|
pub fn get_body_def(body_id : BodyId) -> (WorldId, BodyDef)? {
  let bodies = bodies_storage.val
  for entry in bodies {
    let (stored_id, world_id, def) = entry
    if stored_id.id == body_id.id {
      return Some((world_id, def))
    }
  }
  None
}

// Find shape definition by ID

///|
pub fn get_shape_def(shape_id : ShapeId) -> (BodyId, ShapeDef, ShapeType)? {
  let shapes = shapes_storage.val
  for entry in shapes {
    let (stored_id, body_id, def, shape_type) = entry
    if stored_id.id == shape_id.id {
      return Some((body_id, def, shape_type))
    }
  }
  None
}

// Body queries using stored data

///|
pub fn body_get_position(body_id : BodyId) -> Vec2 {
  // Try to get the simulated position first
  let body_sims = body_sims_storage.val
  for entry in body_sims {
    let (stored_id, body_sim) = entry
    if stored_id == body_id {
      return body_sim.transform.p
    }
  }
  // Fallback to definition position if not found in simulation
  match get_body_def(body_id) {
    Some((_, def)) => def.position
    None => { x: 0.0, y: 0.0 }
  }
}

// Get body type

///|
pub fn body_get_type(body_id : BodyId) -> BodyType {
  match get_body_def(body_id) {
    Some((_, def)) => def.type_
    None => Static
  }
}

// Get body rotation

///|
pub fn body_get_rotation(body_id : BodyId) -> Rot {
  match get_body_def(body_id) {
    Some((_, def)) => def.rotation
    None => rot_identity
  }
}

// Get all bodies in a world

///|
pub fn world_get_bodies(world_id : WorldId) -> Array[BodyId] {
  let result : Array[BodyId] = []
  let bodies = bodies_storage.val
  for entry in bodies {
    let (body_id, stored_world_id, _) = entry
    if stored_world_id.id == world_id.id {
      result.push(body_id)
    }
  }
  result
}

// Get all shapes for a body

///|
pub fn body_get_shapes(body_id : BodyId) -> Array[ShapeId] {
  let result : Array[ShapeId] = []
  let shapes = shapes_storage.val
  for entry in shapes {
    let (shape_id, stored_body_id, _, _) = entry
    if stored_body_id.id == body_id.id {
      result.push(shape_id)
    }
  }
  result
}

// World simulation (simplified implementation)

///|
pub fn world_step(
  world_id : WorldId,
  time_step : Double,
  _sub_step_count : Int,
) -> Unit {
  // Find and update the physics world
  let physics_worlds = physics_worlds_storage.val
  for i, entry in physics_worlds {
    let (stored_world_id, world) = entry
    if stored_world_id == world_id {
      // Step the physics world
      let updated_world = step_world(world, time_step)
      physics_worlds[i] = (stored_world_id, updated_world)
      
      // Update the body simulations in storage
      let body_sims = body_sims_storage.val
      for j, updated_body_sim in updated_world.bodies {
        // Find and update the corresponding body in storage
        for k, sim_entry in body_sims {
          let (body_id, _stored_body_sim) = sim_entry
          // Match by position (simple heuristic for now)
          if j < body_sims.length() {
            body_sims[k] = (body_id, updated_body_sim)
          }
        }
      }
      break
    }
  }
}
