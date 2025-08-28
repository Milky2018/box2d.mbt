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
// Default collision filter - collides with everything
pub fn default_collision_filter() -> CollisionFilter {
  { category_bits: 0x0001, mask_bits: 0xFFFF, group_index: 0 }
}

// Test if two collision filters should allow collision (ported from b2ShouldCollide)
pub fn should_collide(filter_a : CollisionFilter, filter_b : CollisionFilter) -> Bool {
  // Different groups with negative indices never collide
  if filter_a.group_index != 0 && filter_a.group_index == filter_b.group_index {
    return filter_a.group_index > 0  // Same positive group = collide, same negative group = don't collide
  }
  
  // Check category/mask collision
  let collide_mask = (filter_a.mask_bits.land(filter_b.category_bits)) != 0 && 
                     (filter_b.mask_bits.land(filter_a.category_bits)) != 0
  return collide_mask
}

pub fn default_shape_def() -> ShapeDef {
  { density: 1.0, friction: 0.6, restitution: 0.0, rolling_friction: 0.2, filter: default_collision_filter(), is_sensor: false }
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

// Overlap queries for spatial testing

// Test if a point overlaps with a circle (ported from b2PointInCircle)
pub fn test_point_in_circle(point : Vec2, circle : Circle, transform : Transform) -> Bool {
  let center_world = transform_point(transform, circle.center)
  let d = sub(point, center_world)
  let distance_squared = dot(d, d)
  return distance_squared <= circle.radius * circle.radius
}

// Test if a point overlaps with a polygon (ported from b2PointInPolygon)
pub fn test_point_in_polygon(point : Vec2, polygon : Polygon, transform : Transform) -> Bool {
  let local_point = inv_transform_point(transform, point)
  
  for i = 0; i < polygon.count; i = i + 1 {
    let vertex = polygon.vertices[i]
    let normal = polygon.normals[i]
    let separation = dot(sub(local_point, vertex), normal)
    if separation > 0.0 {
      return false  // Point is outside this edge
    }
  }
  return true  // Point is inside all edges
}

// Test if a point overlaps with a capsule (ported from b2PointInCapsule)
pub fn test_point_in_capsule(point : Vec2, capsule : Capsule, transform : Transform) -> Bool {
  let local_point = inv_transform_point(transform, point)
  let p1 = capsule.center1
  let p2 = capsule.center2
  let radius = capsule.radius
  
  // Project point onto line segment
  let segment = sub(p2, p1)
  let segment_length_squared = dot(segment, segment)
  
  if segment_length_squared < epsilon {
    // Degenerate capsule - treat as circle
    let d = sub(local_point, p1)
    return dot(d, d) <= radius * radius
  }
  
  let t = clamp_float(dot(sub(local_point, p1), segment) / segment_length_squared, 0.0, 1.0)
  let projection = add(p1, mul_sv(t, segment))
  let d = sub(local_point, projection)
  return dot(d, d) <= radius * radius
}

// Test if two AABBs overlap (ported from b2AABB_Overlaps)
pub fn test_aabb_overlap(aabb_a : AABB, aabb_b : AABB) -> Bool {
  return aabb_a.lower_bound.x <= aabb_b.upper_bound.x && 
         aabb_a.upper_bound.x >= aabb_b.lower_bound.x &&
         aabb_a.lower_bound.y <= aabb_b.upper_bound.y &&
         aabb_a.upper_bound.y >= aabb_b.lower_bound.y
}

// Test if a point is inside an AABB (ported from b2AABB_Contains)
pub fn test_point_in_aabb(point : Vec2, aabb : AABB) -> Bool {
  return point.x >= aabb.lower_bound.x && point.x <= aabb.upper_bound.x &&
         point.y >= aabb.lower_bound.y && point.y <= aabb.upper_bound.y
}

// Shape casting and ray casting queries

// Cast a ray against a circle (ported from b2RayCastCircle)
pub fn raycast_circle(input : RayCastInput, circle : Circle, transform : Transform) -> RayResult {
  let center_world = transform_point(transform, circle.center)
  let s = sub(input.origin, center_world)
  let b = dot(s, input.translation)
  let c = dot(s, s) - circle.radius * circle.radius
  
  // Solve quadratic equation
  let discriminant = b * b - c
  if discriminant < 0.0 {
    return { shape_id: { id: -1 }, point: { x: 0.0, y: 0.0 }, normal: { x: 0.0, y: 0.0 }, fraction: 0.0, hit: false }
  }
  
  let fraction = -b - discriminant.sqrt()
  if fraction < 0.0 || fraction > input.max_fraction {
    return { shape_id: { id: -1 }, point: { x: 0.0, y: 0.0 }, normal: { x: 0.0, y: 0.0 }, fraction: 0.0, hit: false }
  }
  
  let hit_point = add(input.origin, mul_sv(fraction, input.translation))
  let normal = normalize(sub(hit_point, center_world))
  
  {
    shape_id: { id: -1 },  // Shape ID would be provided by caller
    point: hit_point,
    normal,
    fraction,
    hit: true
  }
}

// Cast a ray against a polygon (ported from b2RayCastPolygon)
pub fn raycast_polygon(input : RayCastInput, polygon : Polygon, transform : Transform) -> RayResult {
  // Transform ray to polygon local space
  let local_origin = inv_transform_point(transform, input.origin)
  let local_translation = inv_rotate_vector(transform.q, input.translation)
  
  let mut t_min = 0.0
  let mut t_max = input.max_fraction
  let mut hit_normal = { x: 0.0, y: 0.0 }
  let mut hit_edge = -1
  
  for i = 0; i < polygon.count; i = i + 1 {
    let vertex = polygon.vertices[i]
    let normal = polygon.normals[i]
    
    let numerator = dot(sub(vertex, local_origin), normal)
    let denominator = dot(local_translation, normal)
    
    if denominator == 0.0 {
      if numerator < 0.0 {
        // Ray is parallel and outside
        return { shape_id: { id: -1 }, point: { x: 0.0, y: 0.0 }, normal: { x: 0.0, y: 0.0 }, fraction: 0.0, hit: false }
      }
    } else {
      let t = numerator / denominator
      if denominator < 0.0 {
        // Ray entering
        if t > t_min {
          t_min = t
          hit_normal = normal
          hit_edge = i
        }
      } else {
        // Ray exiting
        if t < t_max {
          t_max = t
        }
      }
      
      if t_min > t_max {
        return { shape_id: { id: -1 }, point: { x: 0.0, y: 0.0 }, normal: { x: 0.0, y: 0.0 }, fraction: 0.0, hit: false }
      }
    }
  }
  
  if hit_edge >= 0 && t_min >= 0.0 && t_min <= input.max_fraction {
    let hit_point = add(input.origin, mul_sv(t_min, input.translation))
    let world_normal = rotate_vector(transform.q, hit_normal)
    
    {
      shape_id: { id: -1 },  // Shape ID would be provided by caller
      point: hit_point,
      normal: world_normal,
      fraction: t_min,
      hit: true
    }
  } else {
    { shape_id: { id: -1 }, point: { x: 0.0, y: 0.0 }, normal: { x: 0.0, y: 0.0 }, fraction: 0.0, hit: false }
  }
}

// Cast a ray against a capsule (ported from b2RayCastCapsule)
pub fn raycast_capsule(input : RayCastInput, capsule : Capsule, transform : Transform) -> RayResult {
  // Transform ray to capsule local space
  let local_origin = inv_transform_point(transform, input.origin)
  let local_translation = inv_rotate_vector(transform.q, input.translation)
  
  let p1 = capsule.center1
  let p2 = capsule.center2
  let radius = capsule.radius
  
  // Create a "thick line segment" by expanding endpoints with radius
  let segment = sub(p2, p1)
  let segment_length = length(segment)
  
  if segment_length < epsilon {
    // Degenerate capsule - treat as circle
    let circle = { center: p1, radius }
    let local_input = { origin: local_origin, translation: local_translation, max_fraction: input.max_fraction }
    let result = raycast_circle(local_input, circle, { p: { x: 0.0, y: 0.0 }, q: rot_identity })
    if result.hit {
      return {
        shape_id: { id: -1 },
        point: transform_point(transform, result.point),
        normal: rotate_vector(transform.q, result.normal),
        fraction: result.fraction,
        hit: true
      }
    } else {
      return result
    }
  }
  
  let axis = mul_sv(1.0 / segment_length, segment)
  let _perpendicular = { x: -axis.y, y: axis.x }  // For potential future use in full capsule raycasting
  
  // Test against the two rounded ends (circles)
  let circle1 = { center: p1, radius }
  let circle2 = { center: p2, radius }
  
  let local_input = { origin: local_origin, translation: local_translation, max_fraction: input.max_fraction }
  let result1 = raycast_circle(local_input, circle1, { p: { x: 0.0, y: 0.0 }, q: rot_identity })
  let result2 = raycast_circle(local_input, circle2, { p: { x: 0.0, y: 0.0 }, q: rot_identity })
  
  // Return the closest hit
  let mut best_result = { shape_id: { id: -1 }, point: { x: 0.0, y: 0.0 }, normal: { x: 0.0, y: 0.0 }, fraction: 0.0, hit: false }
  let mut best_fraction = input.max_fraction + 1.0
  
  if result1.hit && result1.fraction < best_fraction {
    best_result = {
      shape_id: { id: -1 },
      point: transform_point(transform, result1.point),
      normal: rotate_vector(transform.q, result1.normal),
      fraction: result1.fraction,
      hit: true
    }
    best_fraction = result1.fraction
  }
  
  if result2.hit && result2.fraction < best_fraction {
    best_result = {
      shape_id: { id: -1 },
      point: transform_point(transform, result2.point),
      normal: rotate_vector(transform.q, result2.normal),
      fraction: result2.fraction,
      hit: true
    }
    best_fraction = result2.fraction
  }
  
  return best_result
}

// Debug drawing utilities

// Create default debug drawing context
pub fn create_debug_draw_context() -> DebugDrawContext {
  { commands: [], flags: [DrawShapes] }
}

// Add debug drawing command to context
pub fn add_debug_command(context : DebugDrawContext, command : DebugDrawCommand) -> DebugDrawContext {
  let new_commands = context.commands.copy()
  new_commands.push(command)
  { ..context, commands: new_commands }
}

// Standard colors for debug drawing
pub let debug_color_red : Color = { r: 1.0, g: 0.0, b: 0.0, a: 1.0 }
pub let debug_color_green : Color = { r: 0.0, g: 1.0, b: 0.0, a: 1.0 }
pub let debug_color_blue : Color = { r: 0.0, g: 0.0, b: 1.0, a: 1.0 }
pub let debug_color_yellow : Color = { r: 1.0, g: 1.0, b: 0.0, a: 1.0 }
pub let debug_color_white : Color = { r: 1.0, g: 1.0, b: 1.0, a: 1.0 }
pub let debug_color_gray : Color = { r: 0.5, g: 0.5, b: 0.5, a: 1.0 }

// Draw a circle shape for debugging
pub fn debug_draw_circle(context : DebugDrawContext, circle : Circle, transform : Transform, color : Color) -> DebugDrawContext {
  let center_world = transform_point(transform, circle.center)
  add_debug_command(context, DrawCircle(center_world, circle.radius, color))
}

// Draw a polygon shape for debugging
pub fn debug_draw_polygon(context : DebugDrawContext, polygon : Polygon, transform : Transform, color : Color) -> DebugDrawContext {
  let world_vertices = []
  for i = 0; i < polygon.count; i = i + 1 {
    let world_vertex = transform_point(transform, polygon.vertices[i])
    world_vertices.push(world_vertex)
  }
  add_debug_command(context, DrawPolygon(world_vertices, color))
}

// Draw a capsule shape for debugging
pub fn debug_draw_capsule(context : DebugDrawContext, capsule : Capsule, transform : Transform, color : Color) -> DebugDrawContext {
  let center1_world = transform_point(transform, capsule.center1)
  let center2_world = transform_point(transform, capsule.center2)
  
  // Draw the two end circles
  let context1 = add_debug_command(context, DrawCircle(center1_world, capsule.radius, color))
  let context2 = add_debug_command(context1, DrawCircle(center2_world, capsule.radius, color))
  
  // Draw connecting lines (simplified representation)
  add_debug_command(context2, DrawLine(center1_world, center2_world, color))
}

// Draw a body's transform (coordinate frame)
pub fn debug_draw_transform(context : DebugDrawContext, transform : Transform) -> DebugDrawContext {
  add_debug_command(context, DrawTransform(transform))
}

// Draw an AABB for debugging
pub fn debug_draw_aabb(context : DebugDrawContext, aabb : AABB, color : Color) -> DebugDrawContext {
  let vertices = [
    { x: aabb.lower_bound.x, y: aabb.lower_bound.y },
    { x: aabb.upper_bound.x, y: aabb.lower_bound.y },
    { x: aabb.upper_bound.x, y: aabb.upper_bound.y },
    { x: aabb.lower_bound.x, y: aabb.upper_bound.y }
  ]
  add_debug_command(context, DrawPolygon(vertices, color))
}

// Clear debug drawing commands
pub fn clear_debug_commands(context : DebugDrawContext) -> DebugDrawContext {
  { ..context, commands: [] }
}
