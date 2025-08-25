// Box2D Shape Tests - ported from test_shape.c
// Testing shape mass computation, AABB computation, point-in-shape, and ray casting

test "shape mass computation" {
  // Test circle mass computation (ported from ShapeMassTest)
  let circle : @box2d.Circle = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let density = 1.0
  let md = @box2d.compute_circle_mass(circle, density)
  let expected_mass = @box2d.pi * 1.0 * 1.0 * density  // π * r² * density
  ensure_small(md.mass - expected_mass, 1.0e-6, "circle mass")
  ensure(md.center.x == 1.0 && md.center.y == 0.0, "circle mass center")
  ensure_small(md.rotational_inertia - 0.5 * @box2d.pi, 1.0e-6, "circle rotational inertia")
  
  // Test capsule mass computation
  let capsule : @box2d.Capsule = { center1: { x: -1.0, y: 0.0 }, center2: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let capsule_md = @box2d.compute_capsule_mass(capsule, density)
  
  // Box that fully contains capsule for comparison
  let radius = capsule.radius
  let length = @box2d.distance_between_points(capsule.center1, capsule.center2)
  let box = @box2d.make_polygon_box(radius, radius + 0.5 * length)
  let box_md = @box2d.compute_polygon_mass(box, density)
  
  // Capsule mass should be between approximations
  ensure(capsule_md.mass > 0.0, "capsule has positive mass")
  ensure(capsule_md.mass < box_md.mass, "capsule mass less than containing box")
  
  // Test polygon mass computation
  let polygon = @box2d.make_polygon_box(1.0, 1.0)  // 2x2 box
  let poly_md = @box2d.compute_polygon_mass(polygon, density)
  ensure_small(poly_md.mass - 4.0, 1.0e-6, "polygon mass")
  ensure_small(poly_md.center.x, 1.0e-6, "polygon center x")
  ensure_small(poly_md.center.y, 1.0e-6, "polygon center y")
  ensure_small(poly_md.rotational_inertia - 8.0 / 3.0, 2.0e-6, "polygon rotational inertia")
}

test "shape AABB computation" {
  // Test circle AABB (ported from ShapeAABBTest)
  let circle : @box2d.Circle = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let identity_transform = @box2d.transform_identity
  let circle_aabb = @box2d.compute_circle_aabb(circle, identity_transform)
  
  ensure_small(circle_aabb.lower_bound.x - 0.0, 1.0e-6, "circle AABB lower x")
  ensure_small(circle_aabb.lower_bound.y - (-1.0), 1.0e-6, "circle AABB lower y")
  ensure_small(circle_aabb.upper_bound.x - 2.0, 1.0e-6, "circle AABB upper x")
  ensure_small(circle_aabb.upper_bound.y - 1.0, 1.0e-6, "circle AABB upper y")
  
  // Test polygon AABB
  let box = @box2d.make_polygon_box(1.0, 1.0)
  let box_aabb = @box2d.compute_polygon_aabb(box, identity_transform)
  
  ensure_small(box_aabb.lower_bound.x - (-1.0), 1.0e-6, "box AABB lower x")
  ensure_small(box_aabb.lower_bound.y - (-1.0), 1.0e-6, "box AABB lower y")
  ensure_small(box_aabb.upper_bound.x - 1.0, 1.0e-6, "box AABB upper x")
  ensure_small(box_aabb.upper_bound.y - 1.0, 1.0e-6, "box AABB upper y")
  
  // Test segment AABB
  let segment : @box2d.Segment = { point1: { x: 0.0, y: 1.0 }, point2: { x: 0.0, y: -1.0 } }
  let segment_aabb = @box2d.compute_segment_aabb(segment, identity_transform)
  
  ensure_small(segment_aabb.lower_bound.x - 0.0, 1.0e-6, "segment AABB lower x")
  ensure_small(segment_aabb.lower_bound.y - (-1.0), 1.0e-6, "segment AABB lower y")
  ensure_small(segment_aabb.upper_bound.x - 0.0, 1.0e-6, "segment AABB upper x")
  ensure_small(segment_aabb.upper_bound.y - 1.0, 1.0e-6, "segment AABB upper y")
}

test "point in shape tests" {
  // Test point in shape (ported from PointInShapeTest)
  let p1 : @box2d.Vec2 = { x: 0.5, y: 0.5 }
  let p2 : @box2d.Vec2 = { x: 4.0, y: -4.0 }
  
  // Test point in circle
  let circle : @box2d.Circle = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  let circle_center = circle.center
  let circle_radius = circle.radius
  
  ensure(@box2d.point_in_circle(p1, circle_center, circle_radius), "p1 in circle")
  ensure(not(@box2d.point_in_circle(p2, circle_center, circle_radius)), "p2 not in circle")
  
  // Test point in polygon
  let box = @box2d.make_polygon_box(1.0, 1.0)
  let identity_transform = @box2d.transform_identity
  
  ensure(@box2d.point_in_polygon(p1, box, identity_transform), "p1 in polygon")
  ensure(not(@box2d.point_in_polygon(p2, box, identity_transform)), "p2 not in polygon")
}

test "ray cast shape tests" {
  // Test ray casting against shapes (ported from RayCastShapeTest)
  let input : @box2d.RayCastInput = {
    origin: { x: -4.0, y: 0.0 },
    translation: { x: 8.0, y: 0.0 },
    max_fraction: 1.0
  }
  
  // Test ray cast against circle
  let circle : @box2d.Circle = { center: { x: 1.0, y: 0.0 }, radius: 1.0 }
  
  let circle_result = @box2d.ray_cast_circle(circle, input)
  ensure(circle_result.hit, "ray hits circle")
  ensure_small(circle_result.normal.x - (-1.0), 1.0e-6, "circle hit normal x")
  ensure_small(circle_result.normal.y - 0.0, 1.0e-6, "circle hit normal y")
  ensure_small(circle_result.fraction - 0.5, 1.0e-6, "circle hit fraction")
  
  // Test ray cast against polygon
  let box = @box2d.make_polygon_box(1.0, 1.0)
  
  let polygon_result = @box2d.ray_cast_polygon(box, input)
  ensure(polygon_result.hit, "ray hits polygon")
  ensure_small(polygon_result.normal.x - (-1.0), 1.0e-6, "polygon hit normal x")
  ensure_small(polygon_result.normal.y - 0.0, 1.0e-6, "polygon hit normal y")
  ensure_small(polygon_result.fraction - 3.0 / 8.0, 1.0e-6, "polygon hit fraction")
  
  // Test ray cast against segment
  let segment : @box2d.Segment = { point1: { x: 0.0, y: 1.0 }, point2: { x: 0.0, y: -1.0 } }
  
  let segment_result = @box2d.ray_cast_segment(segment, input, true)
  ensure(segment_result.hit, "ray hits segment")
  ensure_small(segment_result.normal.x - (-1.0), 1.0e-6, "segment hit normal x")
  ensure_small(segment_result.normal.y - 0.0, 1.0e-6, "segment hit normal y")
  ensure_small(segment_result.fraction - 0.5, 1.0e-6, "segment hit fraction")
}