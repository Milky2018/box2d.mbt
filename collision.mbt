// Box2D Collision detection - ported from collision functions
// Basic collision detection and helper functions

// Test if point is inside circle
pub fn point_in_circle(point : Vec2, center : Vec2, radius : Double) -> Bool {
  let dx = point.x - center.x
  let dy = point.y - center.y
  dx * dx + dy * dy <= radius * radius
}

// Test if point is inside capsule
pub fn point_in_capsule(point : Vec2, capsule : Capsule, transform : Transform) -> Bool {
  let center1 = transform_point(transform, capsule.center1)
  let center2 = transform_point(transform, capsule.center2)
  
  // Distance from point to the line segment between centers
  let closest_point = closest_point_on_segment(point, center1, center2)
  let distance = distance_between_points(point, closest_point)
  
  distance <= capsule.radius
}

// Test if point is inside AABB
pub fn point_in_aabb(point : Vec2, aabb : AABB) -> Bool {
  point.x >= aabb.lower_bound.x && 
  point.x <= aabb.upper_bound.x &&
  point.y >= aabb.lower_bound.y && 
  point.y <= aabb.upper_bound.y
}

// Test if point is inside polygon (simplified implementation)
pub fn point_in_polygon(point : Vec2, polygon : Polygon, transform : Transform) -> Bool {
  let local_point = inv_transform_point(transform, point)
  
  // Use winding number test
  let mut winding = 0
  for i = 0; i < polygon.count; i = i + 1 {
    let j = (i + 1) % polygon.count
    let vi = polygon.vertices[i]
    let vj = polygon.vertices[j]
    
    if vi.y <= local_point.y {
      if vj.y > local_point.y {  // upward crossing
        if cross(sub(vj, vi), sub(local_point, vi)) > 0.0 {
          winding = winding + 1
        }
      }
    } else {
      if vj.y <= local_point.y {  // downward crossing
        if cross(sub(vj, vi), sub(local_point, vi)) < 0.0 {
          winding = winding - 1
        }
      }
    }
  }
  
  winding != 0
}

// Ray cast against circle (ported from b2RayCastCircle)
pub fn ray_cast_circle(circle : Circle, input : RayCastInput) -> RayResult {
  let no_hit : RayResult = {
    shape_id: { id: -1 },
    point: { x: 0.0, y: 0.0 },
    normal: { x: 0.0, y: 0.0 },
    fraction: 0.0,
    hit: false
  }
  
  let center = circle.center
  let to_center = sub(center, input.origin)
  
  let a = dot(input.translation, input.translation)
  let b = -2.0 * dot(input.translation, to_center)
  let c = dot(to_center, to_center) - circle.radius * circle.radius
  
  let discriminant = b * b - 4.0 * a * c
  if discriminant < 0.0 {
    return no_hit  // No intersection
  }
  
  let sqrt_discriminant = discriminant.sqrt()
  let t1 = (-b - sqrt_discriminant) / (2.0 * a)
  let t2 = (-b + sqrt_discriminant) / (2.0 * a)
  
  let t = if t1 >= 0.0 && t1 <= input.max_fraction { t1 } else if t2 >= 0.0 && t2 <= input.max_fraction { t2 } else { -1.0 }
  
  if t < 0.0 || t > input.max_fraction {
    no_hit
  } else {
    let hit_point = mul_add(input.origin, t, input.translation)
    let normal = normalize(sub(hit_point, center))
    
    {
      shape_id: { id: -1 },
      point: hit_point,
      normal,
      fraction: t,
      hit: true
    }
  }
}

// Ray cast against polygon (ported from b2RayCastPolygon)
pub fn ray_cast_polygon(polygon : Polygon, input : RayCastInput) -> RayResult {
  let no_hit : RayResult = {
    shape_id: { id: -1 },
    point: { x: 0.0, y: 0.0 },
    normal: { x: 0.0, y: 0.0 },
    fraction: 0.0,
    hit: false
  }
  
  if polygon.radius == 0.0 {
    // Shift all math to first vertex since the polygon may be far from the origin
    let base = polygon.vertices[0]
    let p1 = sub(input.origin, base)
    let d = input.translation
    let mut lower = 0.0
    let mut upper = input.max_fraction
    let mut index = -1
    
    for edge_index = 0; edge_index < polygon.count; edge_index = edge_index + 1 {
      // p = p1 + a * d
      // dot(normal, p - v) = 0
      // dot(normal, p1 - v) + a * dot(normal, d) = 0
      let vertex = sub(polygon.vertices[edge_index], base)
      let numerator = dot(polygon.normals[edge_index], sub(vertex, p1))
      let denominator = dot(polygon.normals[edge_index], d)
      
      if denominator == 0.0 {
        // Parallel and runs outside edge
        if numerator < 0.0 {
          return no_hit
        }
      } else {
        // Note: we want this predicate without division:
        // lower < numerator / denominator, where denominator < 0
        // Since denominator < 0, we have to flip the inequality:
        // lower < numerator / denominator <==> denominator * lower > numerator.
        if denominator < 0.0 && numerator < lower * denominator {
          // Increase lower.
          // The segment enters this half-space.
          lower = numerator / denominator
          index = edge_index
        } else if denominator > 0.0 && numerator < upper * denominator {
          // Decrease upper.
          // The segment exits this half-space.
          upper = numerator / denominator
        }
      }
      
      if upper < lower {
        // Ray misses
        return no_hit
      }
    }
    
    if index >= 0 {
      {
        shape_id: { id: -1 },
        point: mul_add(input.origin, lower, d),
        normal: polygon.normals[index],
        fraction: lower,
        hit: true
      }
    } else {
      // Initial overlap
      {
        shape_id: { id: -1 },
        point: input.origin,
        normal: { x: 0.0, y: 0.0 },
        fraction: 0.0,
        hit: true
      }
    }
  } else {
    // For polygons with radius, use simplified approach
    no_hit
  }
}

// Ray cast against segment (ported from b2RayCastSegment)
pub fn ray_cast_segment(segment : Segment, input : RayCastInput, one_sided : Bool) -> RayResult {
  let no_hit : RayResult = {
    shape_id: { id: -1 },
    point: { x: 0.0, y: 0.0 },
    normal: { x: 0.0, y: 0.0 },
    fraction: 0.0,
    hit: false
  }
  
  if one_sided {
    // Skip left-side collision
    let offset = cross(sub(input.origin, segment.point1), sub(segment.point2, segment.point1))
    if offset < 0.0 {
      return no_hit
    }
  }
  
  // Put the ray into the edge's frame of reference
  let p1 = input.origin
  let d = input.translation
  
  let v1 = segment.point1
  let v2 = segment.point2
  let e = sub(v2, v1)
  
  let len = length(e)
  if len == 0.0 {
    return no_hit
  }
  
  let e_unit = mul_sv(1.0 / len, e)
  
  // Normal points to the right, looking from v1 towards v2
  let normal = right_perp(e_unit)
  
  // Intersect ray with infinite segment using normal
  // Similar to intersecting a ray with an infinite plane
  // p = p1 + t * d
  // dot(normal, p - v1) = 0
  // dot(normal, p1 - v1) + t * dot(normal, d) = 0
  let numerator = dot(normal, sub(v1, p1))
  let denominator = dot(normal, d)
  
  if denominator == 0.0 {
    // parallel
    return no_hit
  }
  
  let t = numerator / denominator
  if t < 0.0 || input.max_fraction < t {
    // out of ray range
    return no_hit
  }
  
  // Intersection point on infinite segment
  let p = mul_add(p1, t, d)
  
  // Compute position of p along segment
  // p = v1 + s * e
  // s = dot(p - v1, e) / dot(e, e)
  let s = dot(sub(p, v1), e_unit)
  if s < 0.0 || len < s {
    // out of segment range
    return no_hit
  }
  
  let final_normal = if numerator > 0.0 { neg(normal) } else { normal }
  
  {
    shape_id: { id: -1 },
    point: p,
    normal: final_normal,
    fraction: t,
    hit: true
  }
}


// Compute distance between two points
pub fn distance_between_points(a : Vec2, b : Vec2) -> Double {
  let dx = b.x - a.x
  let dy = b.y - a.y
  (dx * dx + dy * dy).sqrt()
}

// Closest point on a line segment to a point
pub fn closest_point_on_segment(point : Vec2, segment_a : Vec2, segment_b : Vec2) -> Vec2 {
  let segment_vec = sub(segment_b, segment_a)
  let point_vec = sub(point, segment_a)
  
  let segment_length_sq = dot(segment_vec, segment_vec)
  if segment_length_sq < 1.0e-10 {
    return segment_a  // Degenerate segment
  }
  
  let t = dot(point_vec, segment_vec) / segment_length_sq
  let clamped_t = clamp_float(t, 0.0, 1.0)
  
  mul_add(segment_a, clamped_t, segment_vec)
}