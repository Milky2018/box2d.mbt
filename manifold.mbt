// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from manifold.c - Contact manifold generation

// Constants are imported from distance.mbt package

// Helper function to make an ID (ported from B2_MAKE_ID) - currently unused but may be needed later
pub fn make_id(a : Int, b : Int) -> Int {
  (a << 8) | b
}

// Helper to create empty manifold
fn empty_manifold() -> Manifold {
  { normal: vec2_zero, rolling_impulse: 0.0, points: [], point_count: 0 }
}

// Helper to make capsule polygon (ported from b2MakeCapsule)
fn make_capsule_polygon(p1 : Vec2, p2 : Vec2, radius : Double) -> Polygon {
  let vertices = [p1, p2]
  let d = sub(p2, p1)
  let axis = normalize(d)
  let normal = right_perp(axis)
  let normals = [normal, neg(normal)]
  let centroid = lerp(p1, p2, 0.5)
  
  { 
    vertices, normals, centroid, radius, 
    count: 2
  }
}

// Compute the collision manifold between two circles (ported from b2CollideCircles)
pub fn collide_circles(circle_a : Circle, xf_a : Transform, circle_b : Circle, xf_b : Transform) -> Manifold {
  let xf = inv_mul_transforms(xf_a, xf_b)
  
  let point_a = circle_a.center
  let point_b = transform_point(xf, circle_b.center)
  
  let (distance, normal) = get_length_and_normalize(sub(point_b, point_a))
  
  let radius_a = circle_a.radius
  let radius_b = circle_b.radius
  
  let separation = distance - radius_a - radius_b
  if separation > 0.02 {
    return empty_manifold()
  }
  
  let c_a = mul_add(point_a, radius_a, normal)
  let c_b = mul_add(point_b, -radius_b, normal)
  let contact_point_a = lerp(c_a, c_b, 0.5)
  
  let manifold_normal = rotate_vector(xf_a.q, normal)
  let mp = {
    point: add(xf_a.p, rotate_vector(xf_a.q, contact_point_a)),
    anchor_a: rotate_vector(xf_a.q, contact_point_a),
    anchor_b: add(rotate_vector(xf_a.q, contact_point_a), sub(xf_a.p, xf_b.p)),
    separation,
    normal_impulse: 0.0,
    tangent_impulse: 0.0,
    total_normal_impulse: 0.0,
    normal_velocity: 0.0,
    id: 0,
    persisted: false
  }
  
  { normal: manifold_normal, rolling_impulse: 0.0, points: [mp], point_count: 1 }
}

// Compute the collision manifold between a capsule and circle (ported from b2CollideCapsuleAndCircle)
pub fn collide_capsule_and_circle(capsule_a : Capsule, xf_a : Transform, circle_b : Circle, xf_b : Transform) -> Manifold {
  let xf = inv_mul_transforms(xf_a, xf_b)
  
  // Compute circle position in the frame of the capsule
  let p_b = transform_point(xf, circle_b.center)
  
  // Compute closest point
  let p1 = capsule_a.center1
  let p2 = capsule_a.center2
  
  let e = sub(p2, p1)
  
  // Find closest point on capsule segment
  let s1 = dot(sub(p_b, p1), e)
  let s2 = dot(sub(p2, p_b), e)
  
  let p_a = if s1 < 0.0 {
    // p1 region
    p1
  } else if s2 < 0.0 {
    // p2 region
    p2
  } else {
    // circle colliding with segment interior
    let s = s1 / dot(e, e)
    mul_add(p1, s, e)
  }
  
  let (distance, normal) = get_length_and_normalize(sub(p_b, p_a))
  
  let radius_a = capsule_a.radius
  let radius_b = circle_b.radius
  let separation = distance - radius_a - radius_b
  if separation > 0.02 {
    return empty_manifold()
  }
  
  let c_a = mul_add(p_a, radius_a, normal)
  let c_b = mul_add(p_b, -radius_b, normal)
  let contact_point_a = lerp(c_a, c_b, 0.5)
  
  let manifold_normal = rotate_vector(xf_a.q, normal)
  let mp = {
    point: add(xf_a.p, rotate_vector(xf_a.q, contact_point_a)),
    anchor_a: rotate_vector(xf_a.q, contact_point_a),
    anchor_b: add(rotate_vector(xf_a.q, contact_point_a), sub(xf_a.p, xf_b.p)),
    separation,
    normal_impulse: 0.0,
    tangent_impulse: 0.0,
    total_normal_impulse: 0.0,
    normal_velocity: 0.0,
    id: 0,
    persisted: false
  }
  
  { normal: manifold_normal, rolling_impulse: 0.0, points: [mp], point_count: 1 }
}

// Compute the collision manifold between a polygon and circle (ported from b2CollidePolygonAndCircle)
pub fn collide_polygon_and_circle(polygon_a : Polygon, xf_a : Transform, circle_b : Circle, xf_b : Transform) -> Manifold {
  let xf = inv_mul_transforms(xf_a, xf_b)
  
  // Compute circle position in the frame of the polygon
  let center = transform_point(xf, circle_b.center)
  let radius_a = polygon_a.radius
  let radius_b = circle_b.radius
  let radius = radius_a + radius_b
  
  // Find the min separating edge
  let mut normal_index = 0
  let mut separation = -1.0e30
  let vertex_count = polygon_a.count
  let vertices = polygon_a.vertices
  let normals = polygon_a.normals
  
  for i = 0; i < vertex_count; i = i + 1 {
    let s = dot(normals[i], sub(center, vertices[i]))
    if s > separation {
      separation = s
      normal_index = i
    }
  }
  
  if separation > radius + 0.02 {
    return empty_manifold()
  }
  
  // Vertices of the reference edge
  let vert_index1 = normal_index
  let vert_index2 = if vert_index1 + 1 < vertex_count { vert_index1 + 1 } else { 0 }
  let v1 = vertices[vert_index1]
  let v2 = vertices[vert_index2]
  
  // Compute barycentric coordinates
  let u1 = dot(sub(center, v1), sub(v2, v1))
  let u2 = dot(sub(center, v2), sub(v1, v2))
  
  if u1 < 0.0 && separation > 1.0e-10 {
    // Circle center is closest to v1 and safely outside the polygon
    let normal = normalize(sub(center, v1))
    let sep = dot(sub(center, v1), normal)
    if sep > radius + 0.02 {
      return empty_manifold()
    }
    
    let c_a = mul_add(v1, radius_a, normal)
    let c_b = mul_sub(center, radius_b, normal)
    let contact_point_a = lerp(c_a, c_b, 0.5)
    
    let manifold_normal = rotate_vector(xf_a.q, normal)
    let mp = {
      point: add(xf_a.p, rotate_vector(xf_a.q, contact_point_a)),
      anchor_a: rotate_vector(xf_a.q, contact_point_a),
      anchor_b: add(rotate_vector(xf_a.q, contact_point_a), sub(xf_a.p, xf_b.p)),
      separation: dot(sub(c_b, c_a), normal),
      normal_impulse: 0.0,
      tangent_impulse: 0.0,
      total_normal_impulse: 0.0,
      normal_velocity: 0.0,
      id: 0,
      persisted: false
    }
    { normal: manifold_normal, rolling_impulse: 0.0, points: [mp], point_count: 1 }
  } else if u2 < 0.0 && separation > 1.0e-10 {
    // Circle center is closest to v2 and safely outside the polygon
    let normal = normalize(sub(center, v2))
    let sep = dot(sub(center, v2), normal)
    if sep > radius + 0.02 {
      return empty_manifold()
    }
    
    let c_a = mul_add(v2, radius_a, normal)
    let c_b = mul_sub(center, radius_b, normal)
    let contact_point_a = lerp(c_a, c_b, 0.5)
    
    let manifold_normal = rotate_vector(xf_a.q, normal)
    let mp = {
      point: add(xf_a.p, rotate_vector(xf_a.q, contact_point_a)),
      anchor_a: rotate_vector(xf_a.q, contact_point_a),
      anchor_b: add(rotate_vector(xf_a.q, contact_point_a), sub(xf_a.p, xf_b.p)),
      separation: dot(sub(c_b, c_a), normal),
      normal_impulse: 0.0,
      tangent_impulse: 0.0,
      total_normal_impulse: 0.0,
      normal_velocity: 0.0,
      id: 0,
      persisted: false
    }
    { normal: manifold_normal, rolling_impulse: 0.0, points: [mp], point_count: 1 }
  } else {
    // Circle center is between v1 and v2. Center may be inside polygon
    let normal = normals[normal_index]
    let manifold_normal = rotate_vector(xf_a.q, normal)
    
    // c_a is the projection of the circle center onto the reference edge
    let c_a = mul_add(center, radius_a - dot(sub(center, v1), normal), normal)
    
    // c_b is the deepest point on the circle with respect to the reference edge
    let c_b = mul_sub(center, radius_b, normal)
    
    let contact_point_a = lerp(c_a, c_b, 0.5)
    
    let mp = {
      point: add(xf_a.p, rotate_vector(xf_a.q, contact_point_a)),
      anchor_a: rotate_vector(xf_a.q, contact_point_a),
      anchor_b: add(rotate_vector(xf_a.q, contact_point_a), sub(xf_a.p, xf_b.p)),
      separation: separation - radius,
      normal_impulse: 0.0,
      tangent_impulse: 0.0,
      total_normal_impulse: 0.0,
      normal_velocity: 0.0,
      id: 0,
      persisted: false
    }
    { normal: manifold_normal, rolling_impulse: 0.0, points: [mp], point_count: 1 }
  }
}

// Simplified segment collision functions (converting to capsules)
pub fn collide_segment_and_circle(segment_a : Segment, xf_a : Transform, circle_b : Circle, xf_b : Transform) -> Manifold {
  let capsule_a = { center1: segment_a.point1, center2: segment_a.point2, radius: 0.0 }
  collide_capsule_and_circle(capsule_a, xf_a, circle_b, xf_b)
}

pub fn collide_segment_and_capsule(segment_a : Segment, xf_a : Transform, capsule_b : Capsule, xf_b : Transform) -> Manifold {
  let capsule_a = { center1: segment_a.point1, center2: segment_a.point2, radius: 0.0 }
  collide_capsules(capsule_a, xf_a, capsule_b, xf_b)
}

pub fn collide_polygon_and_capsule(polygon_a : Polygon, xf_a : Transform, capsule_b : Capsule, xf_b : Transform) -> Manifold {
  let poly_b = make_capsule_polygon(capsule_b.center1, capsule_b.center2, capsule_b.radius)
  collide_polygons(polygon_a, xf_a, poly_b, xf_b)
}

pub fn collide_segment_and_polygon(segment_a : Segment, xf_a : Transform, polygon_b : Polygon, xf_b : Transform) -> Manifold {
  let polygon_a = make_capsule_polygon(segment_a.point1, segment_a.point2, 0.0)
  collide_polygons(polygon_a, xf_a, polygon_b, xf_b)
}

// Placeholder implementations for complex collision functions
// These would need full implementation from manifold.c for production use
pub fn collide_capsules(capsule_a : Capsule, xf_a : Transform, capsule_b : Capsule, xf_b : Transform) -> Manifold {
  // Simplified implementation - convert both to polygons
  let poly_a = make_capsule_polygon(capsule_a.center1, capsule_a.center2, capsule_a.radius)
  let poly_b = make_capsule_polygon(capsule_b.center1, capsule_b.center2, capsule_b.radius)
  collide_polygons(poly_a, xf_a, poly_b, xf_b)
}

pub fn collide_polygons(polygon_a : Polygon, xf_a : Transform, polygon_b : Polygon, xf_b : Transform) -> Manifold {
  // Simplified implementation - just check if centroids are close
  let xf = inv_mul_transforms(xf_a, xf_b)
  let centroid_b = transform_point(xf, polygon_b.centroid)
  let distance_to_centroid = distance(polygon_a.centroid, centroid_b)
  
  if distance_to_centroid < polygon_a.radius + polygon_b.radius + 0.02 {
    // Create a simple contact point
    let normal = normalize(sub(centroid_b, polygon_a.centroid))
    let separation = distance_to_centroid - polygon_a.radius - polygon_b.radius
    
    let manifold_normal = rotate_vector(xf_a.q, normal)
    let contact_point_a = mul_add(polygon_a.centroid, polygon_a.radius, normal)
    
    let mp = {
      point: add(xf_a.p, rotate_vector(xf_a.q, contact_point_a)),
      anchor_a: rotate_vector(xf_a.q, contact_point_a),
      anchor_b: add(rotate_vector(xf_a.q, contact_point_a), sub(xf_a.p, xf_b.p)),
      separation,
      normal_impulse: 0.0,
      tangent_impulse: 0.0,
      total_normal_impulse: 0.0,
      normal_velocity: 0.0,
      id: 0,
      persisted: false
    }
    { normal: manifold_normal, rolling_impulse: 0.0, points: [mp], point_count: 1 }
  } else {
    empty_manifold()
  }
}

// Chain segment collision functions (simplified implementations)
pub fn collide_chain_segment_and_circle(segment_a : ChainSegment, xf_a : Transform, circle_b : Circle, xf_b : Transform) -> Manifold {
  collide_segment_and_circle(segment_a.segment, xf_a, circle_b, xf_b)
}

pub fn collide_chain_segment_and_capsule(segment_a : ChainSegment, xf_a : Transform, capsule_b : Capsule, xf_b : Transform, _cache : SimplexCache) -> Manifold {
  collide_segment_and_capsule(segment_a.segment, xf_a, capsule_b, xf_b)
}

pub fn collide_chain_segment_and_polygon(segment_a : ChainSegment, xf_a : Transform, polygon_b : Polygon, xf_b : Transform, _cache : SimplexCache) -> Manifold {
  collide_segment_and_polygon(segment_a.segment, xf_a, polygon_b, xf_b)
}

// Alias function for test compatibility
pub fn collide_circle_capsule(circle : Circle, xf_circle : Transform, capsule : Capsule, xf_capsule : Transform) -> Manifold {
  collide_capsule_and_circle(capsule, xf_capsule, circle, xf_circle)
}