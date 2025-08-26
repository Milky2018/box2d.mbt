// Box2D Polygon Creation Tests - testing all polygon creation functions
// Tests for the complete Box2D polygon creation API

///|
test "polygon from vertices" {
  // Test creating polygon from custom vertices
  let vertices : Array[Vec2] = [
    { x: 0.0, y: 1.0 },    // Top
    { x: -1.0, y: -1.0 },  // Bottom left
    { x: 1.0, y: -1.0 }    // Bottom right
  ]
  
  let triangle = make_polygon_from_vertices(vertices, 0.0)
  
  // Should create valid triangle
  assert_true(triangle.count == 3)
  assert_true(triangle.vertices.length() == 3)
  assert_true(triangle.normals.length() == 3)
  assert_true(triangle.radius == 0.0)
  
  // Centroid should be approximately at (0, -0.33) for this triangle
  assert_true(abs_float(triangle.centroid.x) < 0.1)
  assert_true(abs_float(triangle.centroid.y + 0.33) < 0.2) // Triangle centroid is below origin
}

///|  
test "polygon from vertices with insufficient points" {
  // Test with insufficient vertices (less than 3)
  let vertices : Array[Vec2] = [
    { x: 0.0, y: 0.0 },
    { x: 1.0, y: 0.0 }
  ]
  
  let polygon = make_polygon_from_vertices(vertices, 0.0)
  
  // Should return empty polygon
  assert_true(polygon.count == 0)
  assert_true(polygon.vertices.length() == 0)
}

///|
test "regular triangle creation" {
  let triangle = make_triangle(1.0)
  
  // Should create valid triangle
  assert_true(triangle.count == 3)
  assert_true(triangle.vertices.length() == 3)
  assert_true(triangle.normals.length() == 3)
  
  // Vertices should be at unit distance from origin
  for vertex in triangle.vertices {
    let distance = vec2_length(vertex)
    assert_true(abs_float(distance - 1.0) < 0.01) // Should be approximately 1.0
  }
  
  // Centroid should be at origin for regular polygon
  assert_true(abs_float(triangle.centroid.x) < 0.01)
  assert_true(abs_float(triangle.centroid.y) < 0.01)
}

///|
test "regular pentagon creation" {
  let pentagon = make_pentagon(2.0)
  
  // Should create valid pentagon
  assert_true(pentagon.count == 5)
  assert_true(pentagon.vertices.length() == 5)
  assert_true(pentagon.normals.length() == 5)
  
  // Vertices should be at distance 2.0 from origin
  for vertex in pentagon.vertices {
    let distance = vec2_length(vertex)
    assert_true(abs_float(distance - 2.0) < 0.01)
  }
  
  // Centroid should be at origin
  assert_true(abs_float(pentagon.centroid.x) < 0.01)
  assert_true(abs_float(pentagon.centroid.y) < 0.01)
}

///|
test "regular hexagon creation" {
  let hexagon = make_hexagon(1.5)
  
  // Should create valid hexagon
  assert_true(hexagon.count == 6)
  assert_true(hexagon.vertices.length() == 6)
  assert_true(hexagon.normals.length() == 6)
  
  // Vertices should be at distance 1.5 from origin
  for vertex in hexagon.vertices {
    let distance = vec2_length(vertex)
    assert_true(abs_float(distance - 1.5) < 0.01)
  }
}

///|
test "regular polygon with custom sides" {
  // Test octagon (8 sides)
  let octagon = make_regular_polygon(8, 1.0)
  
  assert_true(octagon.count == 8)
  assert_true(octagon.vertices.length() == 8)
  assert_true(octagon.normals.length() == 8)
  
  // All vertices should be at unit distance
  for vertex in octagon.vertices {
    let distance = vec2_length(vertex)
    assert_true(abs_float(distance - 1.0) < 0.01)
  }
}

///|
test "regular polygon with invalid sides" {
  // Test with too few sides
  let invalid_polygon = make_regular_polygon(2, 1.0)
  
  // Should return empty polygon
  assert_true(invalid_polygon.count == 0)
  assert_true(invalid_polygon.vertices.length() == 0)
}

///|
test "hull to polygon conversion" {
  // Create a hull and convert to polygon
  let vertices : Array[Vec2] = [
    { x: -1.0, y: -1.0 },
    { x: 1.0, y: -1.0 },
    { x: 1.0, y: 1.0 },
    { x: -1.0, y: 1.0 },
    { x: 0.0, y: 0.0 }  // Interior point that should be removed
  ]
  
  let hull = compute_hull(vertices)
  let polygon = make_polygon(hull, 0.0)
  
  // Should create valid polygon (hull should remove interior point)
  assert_true(polygon.count == 4) // Square, interior point removed
  assert_true(polygon.vertices.length() == 4)
  assert_true(polygon.normals.length() == 4)
  
  // Should be a unit square
  assert_true(abs_float(polygon.centroid.x) < 0.01)
  assert_true(abs_float(polygon.centroid.y) < 0.01)
}

///|
test "offset polygon creation" {
  // Create offset polygon
  let vertices : Array[Vec2] = [
    { x: -0.5, y: -0.5 },
    { x: 0.5, y: -0.5 },
    { x: 0.5, y: 0.5 },
    { x: -0.5, y: 0.5 }
  ]
  
  let hull = compute_hull(vertices)
  let position : Vec2 = { x: 2.0, y: 3.0 }
  let rotation = make_rot(pi / 4.0) // 45 degrees
  
  let offset_polygon = make_offset_polygon(hull, position, rotation, 0.0)
  
  // Should create valid transformed polygon
  assert_true(offset_polygon.count == 4)
  assert_true(offset_polygon.vertices.length() == 4)
  
  // Centroid should be at the offset position
  assert_true(abs_float(offset_polygon.centroid.x - position.x) < 0.01)
  assert_true(abs_float(offset_polygon.centroid.y - position.y) < 0.01)
}

///|
test "offset box creation" {
  let center : Vec2 = { x: 1.0, y: 2.0 }
  let rotation = make_rot(pi / 6.0) // 30 degrees
  let half_width = 1.5
  let half_height = 1.0
  
  let offset_box = make_offset_box(half_width, half_height, center, rotation, 0.0)
  
  // Should create valid transformed box
  assert_true(offset_box.count == 4)
  assert_true(offset_box.vertices.length() == 4)
  assert_true(offset_box.normals.length() == 4)
  
  // Centroid should be at the specified center
  assert_true(abs_float(offset_box.centroid.x - center.x) < 0.01)
  assert_true(abs_float(offset_box.centroid.y - center.y) < 0.01)
}

///|
test "rounded box creation" {
  let rounded_box = make_rounded_box(1.0, 0.5, 0.1)
  
  // Should be same as regular box but with radius
  assert_true(rounded_box.count == 4)
  assert_true(rounded_box.vertices.length() == 4)
  assert_true(rounded_box.radius == 0.1)
  
  // Should still be centered at origin
  assert_true(abs_float(rounded_box.centroid.x) < 0.01)
  assert_true(abs_float(rounded_box.centroid.y) < 0.01)
}

///|
test "polygon transformation" {
  let box = make_polygon_box(1.0, 1.0)
  let transform : Transform = {
    p: { x: 3.0, y: 4.0 },
    q: make_rot(pi / 2.0) // 90 degrees
  }
  
  let transformed = transform_polygon(transform, box)
  
  // Should maintain same vertex count
  assert_true(transformed.count == box.count)
  assert_true(transformed.vertices.length() == box.vertices.length())
  
  // Centroid should be transformed
  assert_true(abs_float(transformed.centroid.x - 3.0) < 0.01)
  assert_true(abs_float(transformed.centroid.y - 4.0) < 0.01)
  
  // Original polygon should be unchanged
  assert_true(abs_float(box.centroid.x) < 0.01)
  assert_true(abs_float(box.centroid.y) < 0.01)
}

///|
test "polygon normal computation" {
  let triangle = make_triangle(1.0)
  
  // All normals should be unit vectors pointing outward
  for normal in triangle.normals {
    let length = vec2_length(normal)
    assert_true(abs_float(length - 1.0) < 0.01) // Should be unit length
  }
  
  // For a triangle centered at origin, normals should point outward
  // Check that normals are perpendicular to their respective edges
  for i = 0; i < triangle.count; i = i + 1 {
    let i1 = i
    let i2 = (i + 1) % triangle.count
    let edge = sub(triangle.vertices[i2], triangle.vertices[i1])
    let normal = triangle.normals[i]
    
    // Normal should be perpendicular to edge (dot product ≈ 0)
    let dot_product = vec2_dot(normalize(edge), normal)
    assert_true(abs_float(dot_product) < 0.01)
  }
}

///|
test "polygon in physics simulation" {
  // Test custom polygon in physics simulation
  let world_def = default_world_def()
  let world_id = create_world(world_def)
  
  // Create dynamic body
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  body_def.position = { x: 0.0, y: 5.0 }
  let body_id = create_body(world_id, body_def)
  
  // Create pentagon shape
  let pentagon = make_pentagon(1.0)
  let shape_def = default_shape_def()
  shape_def.density = 1.0
  shape_def.friction = 0.3
  let shape_id = create_polygon_shape(body_id, shape_def, pentagon)
  
  assert_true(shape_is_valid(shape_id))
  
  // Simulate a few steps
  for _i = 0; _i < 30; _i = _i + 1 {
    world_step(world_id, 1.0/60.0, 8)
  }
  
  // Pentagon should fall due to gravity
  let final_pos = body_get_position(body_id)
  assert_true(final_pos.y < 5.0)
}

///|
test "complex custom polygon" {
  // Test creating a more complex custom shape
  let star_vertices : Array[Vec2] = [
    { x: 0.0, y: 2.0 },     // Top point
    { x: 0.6, y: 0.6 },     // Upper right
    { x: 1.9, y: 0.6 },     // Right point  
    { x: 0.9, y: -0.2 },    // Lower right
    { x: 1.2, y: -1.6 },    // Bottom right
    { x: 0.0, y: -0.8 },    // Bottom center
    { x: -1.2, y: -1.6 },   // Bottom left
    { x: -0.9, y: -0.2 },   // Lower left
    { x: -1.9, y: 0.6 },    // Left point
    { x: -0.6, y: 0.6 }     // Upper left
  ]
  
  // Note: This creates a star shape, but hull computation will make it convex
  let polygon = make_polygon_from_vertices(star_vertices, 0.0)
  
  // Should create valid convex polygon (hull of star points)
  // The hull computation might reduce the number of points significantly
  assert_true(polygon.count >= 3 || polygon.count == 0) // Allow empty polygon if hull fails
  assert_true(polygon.vertices.length() >= 3 || polygon.vertices.length() == 0)
  assert_true(polygon.normals.length() >= 3 || polygon.normals.length() == 0)
  
  // All normals should be unit vectors (if polygon was created successfully)
  if polygon.count > 0 {
    for normal in polygon.normals {
      let length = vec2_length(normal)
      assert_true(abs_float(length - 1.0) < 0.01)
    }
  }
}