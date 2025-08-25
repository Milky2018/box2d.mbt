// Box2D Shape Tests - Testing new shape types (capsule, segment)
// Comprehensive testing of shape creation, AABB computation, and collision

test "capsule creation and properties" {
  // Test horizontal capsule creation
  let horizontal_capsule = make_capsule_horizontal(4.0, 1.0)
  inspect(horizontal_capsule.center1.x == -2.0, content="true")
  inspect(horizontal_capsule.center2.x == 2.0, content="true")
  inspect(horizontal_capsule.radius == 1.0, content="true")
  
  // Test vertical capsule creation
  let vertical_capsule = make_capsule_vertical(6.0, 0.5)
  inspect(vertical_capsule.center1.y == -3.0, content="true")
  inspect(vertical_capsule.center2.y == 3.0, content="true")
  inspect(vertical_capsule.radius == 0.5, content="true")
  
  // Test custom capsule
  let custom_capsule = make_capsule({ x: 1.0, y: 2.0 }, { x: 3.0, y: 4.0 }, 0.8)
  inspect(custom_capsule.center1.x == 1.0, content="true")
  inspect(custom_capsule.center2.y == 4.0, content="true")
}

test "segment creation and properties" {
  // Test horizontal segment
  let horizontal_segment = make_segment_horizontal(3.0)
  inspect(horizontal_segment.point1.x == -3.0, content="true")
  inspect(horizontal_segment.point2.x == 3.0, content="true")
  
  // Test vertical segment
  let vertical_segment = make_segment_vertical(2.5)
  inspect(vertical_segment.point1.y == -2.5, content="true")
  inspect(vertical_segment.point2.y == 2.5, content="true")
  
  // Test custom segment
  let custom_segment = make_segment({ x: 0.0, y: 0.0 }, { x: 5.0, y: 5.0 })
  inspect(custom_segment.point1.x == 0.0, content="true")
  inspect(custom_segment.point2.x == 5.0, content="true")
}

test "improved polygon structure" {
  // Test box polygon with new structure
  let box = make_polygon_box(2.0, 1.5)
  inspect(box.count == 4, content="true")
  inspect(box.vertices.length() == 4, content="true")
  inspect(box.normals.length() == 4, content="true")
  inspect(box.centroid.x == 0.0, content="true")
  inspect(box.centroid.y == 0.0, content="true")
  
  // Check vertices are correct
  inspect(box.vertices[0].x == -2.0, content="true")  // Bottom-left
  inspect(box.vertices[1].x == 2.0, content="true")   // Bottom-right
  inspect(box.vertices[2].y == 1.5, content="true")   // Top-right
  inspect(box.vertices[3].y == 1.5, content="true")   // Top-left
  
  // Check normals point outward
  inspect(box.normals[0].y == -1.0, content="true")   // Bottom normal (down)
  inspect(box.normals[1].x == 1.0, content="true")    // Right normal (right)
  inspect(box.normals[2].y == 1.0, content="true")    // Top normal (up)
  inspect(box.normals[3].x == -1.0, content="true")   // Left normal (left)
}

test "AABB computation for new shapes" {
  let identity_transform : Transform = { p: { x: 0.0, y: 0.0 }, q: rot_identity }
  
  // Test capsule AABB
  let capsule = make_capsule_horizontal(4.0, 1.0)
  let capsule_aabb = compute_capsule_aabb(capsule, identity_transform)
  
  // Should span from -3 to +3 in x (centers ±2, radius 1)
  inspect(capsule_aabb.lower_bound.x == -3.0, content="true")
  inspect(capsule_aabb.upper_bound.x == 3.0, content="true")
  inspect(capsule_aabb.lower_bound.y == -1.0, content="true")
  inspect(capsule_aabb.upper_bound.y == 1.0, content="true")
  
  // Test segment AABB
  let segment = make_segment({ x: -2.0, y: -1.0 }, { x: 3.0, y: 2.0 })
  let segment_aabb = compute_segment_aabb(segment, identity_transform)
  
  inspect(segment_aabb.lower_bound.x == -2.0, content="true")
  inspect(segment_aabb.upper_bound.x == 3.0, content="true")
  inspect(segment_aabb.lower_bound.y == -1.0, content="true")
  inspect(segment_aabb.upper_bound.y == 2.0, content="true")
}

test "mass computation for new shapes" {
  let density = 1.0
  
  // Test capsule mass
  let capsule = make_capsule_horizontal(4.0, 1.0)
  let capsule_mass = compute_capsule_mass(capsule, density)
  
  inspect(capsule_mass.mass > 0.0, content="true")
  inspect(capsule_mass.center.x == 0.0, content="true")  // Center should be at midpoint
  inspect(capsule_mass.center.y == 0.0, content="true")
  inspect(capsule_mass.rotational_inertia > 0.0, content="true")
  
  // Test segment mass (should have no mass)
  let segment = make_segment_horizontal(3.0)
  let segment_mass = compute_segment_mass(segment, density)
  
  inspect(segment_mass.mass == 0.0, content="true")  // Segments have no area
  inspect(segment_mass.center.x == 0.0, content="true")  // Center should be midpoint
  inspect(segment_mass.rotational_inertia == 0.0, content="true")
}

test "shape creation through core API" {
  // Test creating shapes through the core API
  let world_def = default_world_def()
  let world_id = create_world(world_def)
  
  let body_def = default_body_def()
  body_def.type_ = Dynamic
  let body_id = create_body(world_id, body_def)
  
  let shape_def = default_shape_def()
  
  // Test capsule shape creation
  let capsule = make_capsule_horizontal(2.0, 0.5)
  let capsule_shape_id = create_capsule_shape(body_id, shape_def, capsule)
  inspect(shape_is_valid(capsule_shape_id), content="true")
  
  // Test segment shape creation
  let segment = make_segment_vertical(1.5)
  let segment_shape_id = create_segment_shape(body_id, shape_def, segment)
  inspect(shape_is_valid(segment_shape_id), content="true")
}

test "point-in-shape tests for new shapes" {
  let identity_transform : Transform = { p: { x: 0.0, y: 0.0 }, q: rot_identity }
  
  // Test point in capsule
  let capsule = make_capsule_horizontal(4.0, 1.0)
  
  // Point at center should be inside
  inspect(point_in_capsule({ x: 0.0, y: 0.0 }, capsule, identity_transform), content="true")
  
  // Point at edge of capsule should be inside
  inspect(point_in_capsule({ x: 2.0, y: 1.0 }, capsule, identity_transform), content="true")
  
  // Point outside capsule should not be inside
  inspect(not(point_in_capsule({ x: 0.0, y: 2.0 }, capsule, identity_transform)), content="true")
}

test "collision detection with new shapes" {
  let identity_transform : Transform = { p: { x: 0.0, y: 0.0 }, q: rot_identity }
  let offset_transform : Transform = { p: { x: 1.0, y: 0.0 }, q: rot_identity }
  
  // Test circle-capsule collision
  // Circle at (0,0) radius 0.8, Capsule centers at (0,0) and (2,0) radius 0.5
  // Closest point on capsule to circle should be (0,0), distance=0, total_radius=1.3
  // So they should definitely collide
  let circle : Circle = { center: { x: 0.0, y: 0.0 }, radius: 0.8 }
  let capsule = make_capsule_horizontal(2.0, 0.5)
  
  let manifold = collide_circle_capsule(circle, identity_transform, capsule, offset_transform)
  inspect(manifold.point_count > 0, content="true")  // Should collide
  
  // Test separated shapes
  let far_transform : Transform = { p: { x: 10.0, y: 0.0 }, q: rot_identity }
  let manifold2 = collide_circle_capsule(circle, identity_transform, capsule, far_transform)
  inspect(manifold2.point_count == 0, content="true")  // Should not collide
}
