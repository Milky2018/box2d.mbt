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

// ============ Box2D Polygon Creation Functions ============

// Create a polygon from a convex hull (ported from b2MakePolygon)
pub fn make_polygon(hull : Hull, radius : Double) -> Polygon {
  if hull.count < 3 {
    // Return empty polygon for degenerate hulls
    return {
      vertices: [],
      normals: [],
      centroid: { x: 0.0, y: 0.0 },
      radius: 0.0,
      count: 0
    }
  }
  
  // Copy vertices from hull
  let vertices = []
  for i = 0; i < hull.count; i = i + 1 {
    vertices.push(hull.points[i])
  }
  
  // Compute outward normals for each edge
  let normals = []
  for i = 0; i < hull.count; i = i + 1 {
    let i1 = i
    let i2 = (i + 1) % hull.count
    let edge = sub(vertices[i2], vertices[i1])
    
    // Normal points outward (perpendicular to edge, rotated 90° clockwise)
    let normal = normalize({ x: edge.y, y: -edge.x })
    normals.push(normal)
  }
  
  // Compute centroid
  let centroid = compute_polygon_centroid(vertices)
  
  {
    vertices,
    normals,
    centroid,
    radius,
    count: hull.count
  }
}

// Create a polygon from vertices array (ported from Box2D pattern)
pub fn make_polygon_from_vertices(vertices : Array[Vec2], radius : Double) -> Polygon {
  if vertices.length() < 3 {
    // Return empty polygon for insufficient vertices
    return {
      vertices: [],
      normals: [],
      centroid: { x: 0.0, y: 0.0 },
      radius: 0.0,
      count: 0
    }
  }
  
  // Compute convex hull from vertices to ensure validity
  let hull = compute_hull(vertices)
  
  // Create polygon from validated hull
  make_polygon(hull, radius)
}

// Create a transformed polygon (ported from b2MakeOffsetPolygon)
pub fn make_offset_polygon(hull : Hull, position : Vec2, rotation : Rot, radius : Double) -> Polygon {
  if hull.count < 3 {
    return {
      vertices: [],
      normals: [],
      centroid: { x: 0.0, y: 0.0 },
      radius: 0.0,
      count: 0
    }
  }
  
  // Transform vertices
  let vertices = []
  for i = 0; i < hull.count; i = i + 1 {
    let local_vertex = hull.points[i]
    let world_vertex = add(position, rotate_vector(rotation, local_vertex))
    vertices.push(world_vertex)
  }
  
  // Transform normals 
  let normals = []
  for i = 0; i < hull.count; i = i + 1 {
    let i1 = i
    let i2 = (i + 1) % hull.count
    let edge = sub(vertices[i2], vertices[i1])
    
    // Normal points outward
    let normal = normalize({ x: edge.y, y: -edge.x })
    normals.push(normal)
  }
  
  // Transform centroid
  let local_centroid = compute_polygon_centroid(hull.points)
  let world_centroid = add(position, rotate_vector(rotation, local_centroid))
  
  {
    vertices,
    normals,
    centroid: world_centroid,
    radius,
    count: hull.count
  }
}

// Create a transformed box polygon (ported from b2MakeOffsetBox)
pub fn make_offset_box(half_width : Double, half_height : Double, center : Vec2, rotation : Rot, radius : Double) -> Polygon {
  // Create local box vertices
  let local_vertices = [
    { x: -half_width, y: -half_height },
    { x: half_width, y: -half_height },
    { x: half_width, y: half_height },
    { x: -half_width, y: half_height }
  ]
  
  // Transform vertices to world space
  let vertices = []
  for vertex in local_vertices {
    let world_vertex = add(center, rotate_vector(rotation, vertex))
    vertices.push(world_vertex)
  }
  
  // Compute transformed normals
  let local_normals = [
    { x: 0.0, y: -1.0 },  // Bottom
    { x: 1.0, y: 0.0 },   // Right
    { x: 0.0, y: 1.0 },   // Top
    { x: -1.0, y: 0.0 }   // Left
  ]
  
  let normals = []
  for normal in local_normals {
    let world_normal = rotate_vector(rotation, normal)
    normals.push(world_normal)
  }
  
  // Transform centroid (box centroid is at origin in local space)
  let centroid = add(center, rotate_vector(rotation, { x: 0.0, y: 0.0 }))
  
  {
    vertices,
    normals,
    centroid,
    radius,
    count: 4
  }
}

// Create a rounded box (ported from b2MakeRoundedBox)
pub fn make_rounded_box(half_width : Double, half_height : Double, radius : Double) -> Polygon {
  let box = make_polygon_box(half_width, half_height)
  { ..box, radius }
}

// Transform an existing polygon (ported from b2TransformPolygon)
pub fn transform_polygon(transform : Transform, polygon : Polygon) -> Polygon {
  if polygon.count == 0 {
    return polygon
  }
  
  // Transform vertices
  let vertices = []
  for vertex in polygon.vertices {
    let world_vertex = transform_point(transform, vertex)
    vertices.push(world_vertex)
  }
  
  // Transform normals (only rotation, no translation)
  let normals = []
  for normal in polygon.normals {
    let world_normal = rotate_vector(transform.q, normal)
    normals.push(world_normal)
  }
  
  // Transform centroid
  let centroid = transform_point(transform, polygon.centroid)
  
  {
    vertices,
    normals,
    centroid,
    radius: polygon.radius,
    count: polygon.count
  }
}

// ============ Regular Polygon Creation Helpers ============

// Create a regular triangle
pub fn make_triangle(radius : Double) -> Polygon {
  let vertices = []
  for i = 0; i < 3; i = i + 1 {
    let angle = 2.0 * pi * i.to_double() / 3.0
    let cs = compute_cos_sin(angle)
    let vertex = {
      x: radius * cs.cosine,
      y: radius * cs.sine
    }
    vertices.push(vertex)
  }
  
  make_polygon_from_vertices(vertices, 0.0)
}

// Create a regular pentagon
pub fn make_pentagon(radius : Double) -> Polygon {
  let vertices = []
  for i = 0; i < 5; i = i + 1 {
    let angle = 2.0 * pi * i.to_double() / 5.0
    let cs = compute_cos_sin(angle)
    let vertex = {
      x: radius * cs.cosine,
      y: radius * cs.sine
    }
    vertices.push(vertex)
  }
  
  make_polygon_from_vertices(vertices, 0.0)
}

// Create a regular hexagon
pub fn make_hexagon(radius : Double) -> Polygon {
  let vertices = []
  for i = 0; i < 6; i = i + 1 {
    let angle = 2.0 * pi * i.to_double() / 6.0
    let cs = compute_cos_sin(angle)
    let vertex = {
      x: radius * cs.cosine,
      y: radius * cs.sine
    }
    vertices.push(vertex)
  }
  
  make_polygon_from_vertices(vertices, 0.0)
}

// Create a regular polygon with specified number of sides
pub fn make_regular_polygon(sides : Int, radius : Double) -> Polygon {
  if sides < 3 {
    // Return empty polygon for invalid sides
    return {
      vertices: [],
      normals: [],
      centroid: { x: 0.0, y: 0.0 },
      radius: 0.0,
      count: 0
    }
  }
  
  let vertices = []
  for i = 0; i < sides; i = i + 1 {
    let angle = 2.0 * pi * i.to_double() / sides.to_double()
    let cs = compute_cos_sin(angle)
    let vertex = {
      x: radius * cs.cosine,
      y: radius * cs.sine
    }
    vertices.push(vertex)
  }
  
  make_polygon_from_vertices(vertices, 0.0)
}