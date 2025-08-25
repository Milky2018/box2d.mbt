// Box2D Geometry functions - ported from geometry.c
// Core geometric calculations and polygon utilities

// Compute polygon centroid (ported from b2ComputePolygonCentroid)
pub fn compute_polygon_centroid(vertices : Array[Vec2]) -> Vec2 {
  let count = vertices.length()
  if count == 0 {
    return { x: 0.0, y: 0.0 }
  }
  
  let mut center = { x: 0.0, y: 0.0 }
  let mut area = 0.0
  
  // Get a reference point for forming triangles.
  // Use the first vertex to reduce round-off errors.
  let origin = vertices[0]
  
  let inv3 = 1.0 / 3.0
  
  for i = 1; i < count - 1; i = i + 1 {
    // Triangle edges
    let e1 = sub(vertices[i], origin)
    let e2 = sub(vertices[i + 1], origin)
    let a = 0.5 * cross(e1, e2)
    
    // Area weighted centroid
    center = mul_add(center, a * inv3, add(e1, e2))
    area = area + a
  }
  
  if area > 1.0e-10 {  // Similar to FLT_EPSILON
    let inv_area = 1.0 / area
    center = { x: center.x * inv_area, y: center.y * inv_area }
    
    // Restore offset
    add(origin, center)
  } else {
    origin
  }
}

// Create a box polygon (ported from b2MakeBox)
pub fn make_polygon_box(half_width : Double, half_height : Double) -> Polygon {
  let vertices = [
    { x: -half_width, y: -half_height },
    { x: half_width, y: -half_height },
    { x: half_width, y: half_height },
    { x: -half_width, y: half_height }
  ]
  let normals = [
    { x: 0.0, y: -1.0 },  // Bottom edge normal (down)
    { x: 1.0, y: 0.0 },   // Right edge normal (right)
    { x: 0.0, y: 1.0 },   // Top edge normal (up)
    { x: -1.0, y: 0.0 }   // Left edge normal (left)
  ]
  let centroid = { x: 0.0, y: 0.0 }
  { vertices, normals, centroid, radius: 0.0, count: 4 }
}

// Create a square polygon
pub fn make_polygon_square(half_size : Double) -> Polygon {
  make_polygon_box(half_size, half_size)
}

// Create a capsule shape (ported from Box2D capsule functions)
pub fn make_capsule(center1 : Vec2, center2 : Vec2, radius : Double) -> Capsule {
  { center1, center2, radius }
}

// Create a horizontal capsule
pub fn make_capsule_horizontal(length : Double, radius : Double) -> Capsule {
  let half_length = length * 0.5
  {
    center1: { x: -half_length, y: 0.0 },
    center2: { x: half_length, y: 0.0 },
    radius
  }
}

// Create a vertical capsule  
pub fn make_capsule_vertical(length : Double, radius : Double) -> Capsule {
  let half_length = length * 0.5
  {
    center1: { x: 0.0, y: -half_length },
    center2: { x: 0.0, y: half_length },
    radius
  }
}

// Create a line segment
pub fn make_segment(point1 : Vec2, point2 : Vec2) -> Segment {
  { point1, point2 }
}

// Create a horizontal segment
pub fn make_segment_horizontal(half_length : Double) -> Segment {
  {
    point1: { x: -half_length, y: 0.0 },
    point2: { x: half_length, y: 0.0 }
  }
}

// Create a vertical segment
pub fn make_segment_vertical(half_length : Double) -> Segment {
  {
    point1: { x: 0.0, y: -half_length },
    point2: { x: 0.0, y: half_length }
  }
}

// Validate if a ray cast input is valid (ported from b2IsValidRay)
pub fn is_valid_ray(origin : Vec2, translation : Vec2, max_fraction : Double) -> Bool {
  is_valid_vec2(origin) && 
  is_valid_vec2(translation) && 
  is_valid_float(max_fraction) && 
  0.0 <= max_fraction && 
  max_fraction < 1.0e30  // Similar to B2_HUGE
}

// Compute AABB for a circle shape
pub fn compute_circle_aabb(circle : Circle, transform : Transform) -> AABB {
  let center = transform_point(transform, circle.center)
  {
    lower_bound: { x: center.x - circle.radius, y: center.y - circle.radius },
    upper_bound: { x: center.x + circle.radius, y: center.y + circle.radius }
  }
}

// Compute AABB for a polygon shape  
pub fn compute_polygon_aabb(polygon : Polygon, transform : Transform) -> AABB {
  if polygon.vertices.length() == 0 {
    return { lower_bound: { x: 0.0, y: 0.0 }, upper_bound: { x: 0.0, y: 0.0 } }
  }
  
  let first_vertex = transform_point(transform, polygon.vertices[0])
  let mut lower = first_vertex
  let mut upper = first_vertex
  
  for i = 1; i < polygon.vertices.length(); i = i + 1 {
    let v = transform_point(transform, polygon.vertices[i])
    lower = min_vec2(lower, v)
    upper = max_vec2(upper, v)
  }
  
  // Add radius for rounded polygons
  let expansion = { x: polygon.radius, y: polygon.radius }
  { 
    lower_bound: sub(lower, expansion), 
    upper_bound: add(upper, expansion)
  }
}

// Compute AABB for a capsule shape
pub fn compute_capsule_aabb(capsule : Capsule, transform : Transform) -> AABB {
  let center1 = transform_point(transform, capsule.center1)
  let center2 = transform_point(transform, capsule.center2)
  
  let lower = min_vec2(center1, center2)
  let upper = max_vec2(center1, center2)
  
  let expansion = { x: capsule.radius, y: capsule.radius }
  {
    lower_bound: sub(lower, expansion),
    upper_bound: add(upper, expansion)
  }
}

// Compute AABB for a segment shape
pub fn compute_segment_aabb(segment : Segment, transform : Transform) -> AABB {
  let point1 = transform_point(transform, segment.point1)
  let point2 = transform_point(transform, segment.point2)
  
  let lower = min_vec2(point1, point2)
  let upper = max_vec2(point1, point2)
  
  { lower_bound: lower, upper_bound: upper }
}