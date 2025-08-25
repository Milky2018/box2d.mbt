// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from distance.c

// Constants
let linear_slop = 0.005  // B2_LINEAR_SLOP in meters
pub let speculative_distance : Double = 4.0 * linear_slop  // B2_SPECULATIVE_DISTANCE
let max_polygon_vertices = 8  // B2_MAX_POLYGON_VERTICES

// Segment distance result (ported from b2SegmentDistanceResult)
pub(all) struct SegmentDistanceResult {
  closest1 : Vec2
  closest2 : Vec2
  distance_squared : Double
  fraction1 : Double  
  fraction2 : Double
} derive(Eq, Show)

// Get transform from sweep (ported from b2GetSweepTransform)
pub fn get_sweep_transform(sweep : Sweep, time : Double) -> Transform {
  // https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
  let p = add(mul_sv(1.0 - time, sweep.c1), mul_sv(time, sweep.c2))
  
  let q = {
    c: (1.0 - time) * sweep.q1.c + time * sweep.q2.c,
    s: (1.0 - time) * sweep.q1.s + time * sweep.q2.s
  }
  
  let normalized_q = normalize_rot(q)
  
  // Shift to origin
  let final_p = sub(p, rotate_vector(normalized_q, sweep.local_center))
  
  { p: final_p, q: normalized_q }
}

// Follows Ericson 5.1.9 Closest Points of Two Line Segments (ported from b2SegmentDistance)
pub fn segment_distance(p1 : Vec2, q1 : Vec2, p2 : Vec2, q2 : Vec2) -> SegmentDistanceResult {
  let d1 = sub(q1, p1)
  let d2 = sub(q2, p2)
  let r = sub(p1, p2)
  let dd1 = dot(d1, d1)
  let dd2 = dot(d2, d2)
  let rd1 = dot(r, d1)
  let rd2 = dot(r, d2)
  
  let eps_sqr = 1.0e-14  // FLT_EPSILON squared equivalent
  
  if dd1 < eps_sqr || dd2 < eps_sqr {
    // Handle all degeneracies
    if dd1 >= eps_sqr {
      // Segment 1 is valid, segment 2 is degenerate
      let s = clamp_float(-rd1 / dd1, 0.0, 1.0)
      let t = 0.0
      let closest1 = mul_add(p1, s, d1)
      let closest2 = p2
      let distance_vec = sub(closest2, closest1)
      {
        closest1,
        closest2,
        distance_squared: dot(distance_vec, distance_vec),
        fraction1: s,
        fraction2: t
      }
    } else if dd2 >= eps_sqr {
      // Segment 2 is valid, segment 1 is degenerate
      let s = 0.0
      let t = clamp_float(rd2 / dd2, 0.0, 1.0)
      let closest1 = p1
      let closest2 = mul_add(p2, t, d2)
      let distance_vec = sub(closest2, closest1)
      {
        closest1,
        closest2,
        distance_squared: dot(distance_vec, distance_vec),
        fraction1: s,
        fraction2: t
      }
    } else {
      // Both segments are degenerate
      let closest1 = p1
      let closest2 = p2
      let distance_vec = sub(closest2, closest1)
      {
        closest1,
        closest2,
        distance_squared: dot(distance_vec, distance_vec),
        fraction1: 0.0,
        fraction2: 0.0
      }
    }
  } else {
    // General case - both segments are valid
    let d12 = dot(d1, d2)
    let denom = dd1 * dd2 - d12 * d12
    
    if abs_float(denom) < eps_sqr {
      // Segments are parallel
      let s = 0.0
      let t = clamp_float(rd2 / dd2, 0.0, 1.0)
      let closest1 = p1
      let closest2 = mul_add(p2, t, d2)
      let distance_vec = sub(closest2, closest1)
      {
        closest1,
        closest2,
        distance_squared: dot(distance_vec, distance_vec),
        fraction1: s,
        fraction2: t
      }
    } else {
      // Segments are not parallel
      let mut s = (d12 * rd2 - dd2 * rd1) / denom
      let mut t = (dd1 * rd2 - d12 * rd1) / denom
      
      s = clamp_float(s, 0.0, 1.0)
      t = clamp_float(t, 0.0, 1.0)
      
      let closest1 = mul_add(p1, s, d1)
      let closest2 = mul_add(p2, t, d2)
      let distance_vec = sub(closest2, closest1)
      {
        closest1,
        closest2,
        distance_squared: dot(distance_vec, distance_vec),
        fraction1: s,
        fraction2: t
      }
    }
  }
}

// Make shape proxy (ported from b2MakeProxy)
pub fn make_proxy(points : Array[Vec2], radius : Double) -> ShapeProxy {
  let count = min_int(points.length(), max_polygon_vertices)
  let proxy_points : Array[Vec2] = []
  for i = 0; i < count; i = i + 1 {
    proxy_points.push(points[i])
  }
  { points: proxy_points, count, radius }
}

// Make offset proxy (ported from b2MakeOffsetProxy)
pub fn make_offset_proxy(points : Array[Vec2], radius : Double, position : Vec2, rotation : Rot) -> ShapeProxy {
  let count = min_int(points.length(), max_polygon_vertices)
  let transform = { p: position, q: rotation }
  let proxy_points : Array[Vec2] = []
  for i = 0; i < count; i = i + 1 {
    proxy_points.push(transform_point(transform, points[i]))
  }
  { points: proxy_points, count, radius }
}

// Weight two vectors (ported from b2Weight2)
fn weight2(a1 : Double, w1 : Vec2, a2 : Double, w2 : Vec2) -> Vec2 {
  { x: a1 * w1.x + a2 * w2.x, y: a1 * w1.y + a2 * w2.y }
}

// Weight three vectors (ported from b2Weight3)  
fn weight3(a1 : Double, w1 : Vec2, a2 : Double, w2 : Vec2, a3 : Double, w3 : Vec2) -> Vec2 {
  { x: a1 * w1.x + a2 * w2.x + a3 * w3.x, y: a1 * w1.y + a2 * w2.y + a3 * w3.y }
}

// Find support point (ported from b2FindSupport)
fn find_support(proxy : ShapeProxy, direction : Vec2) -> Int {
  let points = proxy.points
  let count = proxy.count
  
  let mut best_index = 0
  let mut best_value = dot(points[0], direction)
  for i = 1; i < count; i = i + 1 {
    let value = dot(points[i], direction)
    if value > best_value {
      best_index = i
      best_value = value
    }
  }
  
  best_index
}

// Make simplex from cache (ported from b2MakeSimplexFromCache)
fn make_simplex_from_cache(cache : SimplexCache, proxy_a : ShapeProxy, proxy_b : ShapeProxy) -> Simplex {
  let empty_vertex = { w_a: { x: 0.0, y: 0.0 }, w_b: { x: 0.0, y: 0.0 }, w: { x: 0.0, y: 0.0 }, a: -1.0, index_a: 0, index_b: 0 }
  
  // If the cache is empty or invalid, create a simple initial simplex
  if cache.count == 0 {
    let v1 = {
      index_a: 0,
      index_b: 0,
      w_a: proxy_a.points[0],
      w_b: proxy_b.points[0],
      w: sub(proxy_a.points[0], proxy_b.points[0]),
      a: 1.0
    }
    return { v1, v2: empty_vertex, v3: empty_vertex, count: 1 }
  }
  
  // Copy data from cache
  let v1 = if cache.count >= 1 {
    let ia = cache.index_a[0]
    let ib = cache.index_b[0]
    let wa = proxy_a.points[ia]
    let wb = proxy_b.points[ib]
    { index_a: ia, index_b: ib, w_a: wa, w_b: wb, w: sub(wa, wb), a: -1.0 }
  } else { empty_vertex }
  
  let v2 = if cache.count >= 2 {
    let ia = cache.index_a[1]
    let ib = cache.index_b[1]
    let wa = proxy_a.points[ia]
    let wb = proxy_b.points[ib]
    { index_a: ia, index_b: ib, w_a: wa, w_b: wb, w: sub(wa, wb), a: -1.0 }
  } else { empty_vertex }
  
  let v3 = if cache.count >= 3 {
    let ia = cache.index_a[2]
    let ib = cache.index_b[2]
    let wa = proxy_a.points[ia]
    let wb = proxy_b.points[ib]
    { index_a: ia, index_b: ib, w_a: wa, w_b: wb, w: sub(wa, wb), a: -1.0 }
  } else { empty_vertex }
  
  { v1, v2, v3, count: cache.count }
}

// Make simplex cache (ported from b2MakeSimplexCache)
fn make_simplex_cache(simplex : Simplex) -> SimplexCache {
  let index_a : Array[Int] = []
  let index_b : Array[Int] = []
  
  if simplex.count >= 1 {
    index_a.push(simplex.v1.index_a)
    index_b.push(simplex.v1.index_b)
  }
  if simplex.count >= 2 {
    index_a.push(simplex.v2.index_a)
    index_b.push(simplex.v2.index_b)
  }
  if simplex.count >= 3 {
    index_a.push(simplex.v3.index_a)
    index_b.push(simplex.v3.index_b)
  }
  
  { index_a, index_b, count: simplex.count }
}

// Compute simplex witness points (ported from b2ComputeSimplexWitnessPoints)
fn compute_simplex_witness_points(simplex : Simplex) -> (Vec2, Vec2) {
  match simplex.count {
    1 => (simplex.v1.w_a, simplex.v1.w_b)
    2 => {
      let a = weight2(simplex.v1.a, simplex.v1.w_a, simplex.v2.a, simplex.v2.w_a)
      let b = weight2(simplex.v1.a, simplex.v1.w_b, simplex.v2.a, simplex.v2.w_b)
      (a, b)
    }
    3 => {
      let a = weight3(simplex.v1.a, simplex.v1.w_a, simplex.v2.a, simplex.v2.w_a, simplex.v3.a, simplex.v3.w_a)
      // Note: b = a for 3-simplex (overlap case)
      (a, a)
    }
    _ => ({ x: 0.0, y: 0.0 }, { x: 0.0, y: 0.0 })
  }
}

// Solve 2-simplex (ported from b2SolveSimplex2)
fn solve_simplex2(simplex : Simplex) -> (Simplex, Vec2) {
  let w1 = simplex.v1.w
  let w2 = simplex.v2.w
  let e12 = sub(w2, w1)
  
  // w1 region
  let d12_2 = neg_float(dot(w1, e12))
  if d12_2 <= 0.0 {
    // a2 <= 0, so we clamp it to 0
    let new_v1 = { ..simplex.v1, a: 1.0 }
    let new_simplex = { ..simplex, v1: new_v1, count: 1 }
    (new_simplex, neg(w1))
  } else {
    // w2 region
    let d12_1 = dot(w2, e12)
    if d12_1 <= 0.0 {
      // a1 <= 0, so we clamp it to 0
      let new_v2 = { ..simplex.v2, a: 1.0 }
      let new_simplex = { ..simplex, v1: new_v2, count: 1 }
      (new_simplex, neg(w2))
    } else {
      // Must be in e12 region
      let inv_d12 = 1.0 / (d12_1 + d12_2)
      let new_v1 = { ..simplex.v1, a: d12_1 * inv_d12 }
      let new_v2 = { ..simplex.v2, a: d12_2 * inv_d12 }
      let new_simplex = { ..simplex, v1: new_v1, v2: new_v2, count: 2 }
      let combined = add(w1, w2)
      let cross_result = cross(combined, e12)
      (new_simplex, cross_sv(cross_result, e12))
    }
  }
}

// Solve 3-simplex (ported from b2SolveSimplex3) - simplified version
fn solve_simplex3(simplex : Simplex) -> (Simplex, Vec2) {
  let _w1 = simplex.v1.w
  let _w2 = simplex.v2.w  
  let _w3 = simplex.v3.w
  
  // For now, just handle the triangle case and return zero search direction
  // This indicates overlap/containment
  // TODO: Full GJK triangle case implementation would use w1, w2, w3
  let inv_d123 = 1.0 / 3.0  // Simple equal weighting
  let new_v1 = { ..simplex.v1, a: inv_d123 }
  let new_v2 = { ..simplex.v2, a: inv_d123 }
  let new_v3 = { ..simplex.v3, a: inv_d123 }
  let new_simplex = { v1: new_v1, v2: new_v2, v3: new_v3, count: 3 }
  
  // No search direction - origin is inside triangle
  (new_simplex, { x: 0.0, y: 0.0 })
}

// Simplified GJK distance algorithm
pub fn shape_distance(input : DistanceInput, cache : SimplexCache) -> (DistanceOutput, SimplexCache) {
  let proxy_a = input.proxy_a
  
  // Get proxyB in frame A to avoid further transforms in the main loop
  let transform = inv_mul_transforms(input.transform_a, input.transform_b)
  let local_points : Array[Vec2] = []
  for i = 0; i < input.proxy_b.count; i = i + 1 {
    local_points.push(transform_point(transform, input.proxy_b.points[i]))
  }
  let local_proxy_b = { points: local_points, count: input.proxy_b.count, radius: input.proxy_b.radius }
  
  // Initialize the simplex
  let mut simplex = make_simplex_from_cache(cache, proxy_a, local_proxy_b)
  let mut non_unit_normal = { x: 0.0, y: 0.0 }
  
  // Simple iteration - for basic functionality
  let max_iterations = 20
  let mut iteration = 0
  
  while iteration < max_iterations {
    let (new_simplex, d) = match simplex.count {
      1 => (simplex, neg(simplex.v1.w))
      2 => solve_simplex2(simplex)
      3 => solve_simplex3(simplex)
      _ => (simplex, { x: 0.0, y: 0.0 })
    }
    simplex = new_simplex
    
    // If we have 3 points or zero direction, we're done
    if simplex.count == 3 || (dot(d, d) < 1.0e-14) {
      let (local_point_a, local_point_b) = compute_simplex_witness_points(simplex)
      let output = {
        point_a: transform_point(input.transform_a, local_point_a),
        point_b: transform_point(input.transform_a, local_point_b),
        distance: 0.0,
        normal: { x: 0.0, y: 0.0 },
        iterations: iteration,
        simplex_count: 1
      }
      let new_cache = make_simplex_cache(simplex)
      return (output, new_cache)
    }
    
    non_unit_normal = d
    
    // Find support point
    let vertex_index_a = find_support(proxy_a, d)
    let vertex_w_a = proxy_a.points[vertex_index_a]
    let vertex_index_b = find_support(local_proxy_b, neg(d))
    let vertex_w_b = local_proxy_b.points[vertex_index_b]
    let vertex_w = sub(vertex_w_a, vertex_w_b)
    
    // Create new vertex
    let new_vertex = {
      w_a: vertex_w_a,
      w_b: vertex_w_b,
      w: vertex_w,
      a: -1.0,
      index_a: vertex_index_a,
      index_b: vertex_index_b
    }
    
    // Add vertex to simplex
    simplex = match simplex.count {
      1 => { ..simplex, v2: new_vertex, count: 2 }
      2 => { ..simplex, v3: new_vertex, count: 3 }
      _ => simplex
    }
    
    iteration = iteration + 1
  }
  
  // Prepare output
  let normal = if length(non_unit_normal) > 1.0e-8 {
    normalize(non_unit_normal)
  } else {
    { x: 1.0, y: 0.0 }
  }
  let world_normal = rotate_vector(input.transform_a.q, normal)
  
  let (local_point_a, local_point_b) = compute_simplex_witness_points(simplex)
  
  let output = {
    normal: world_normal,
    distance: distance_between_points(local_point_a, local_point_b),
    point_a: transform_point(input.transform_a, local_point_a),
    point_b: transform_point(input.transform_a, local_point_b),
    iterations: iteration,
    simplex_count: 1
  }
  
  let new_cache = make_simplex_cache(simplex)
  (output, new_cache)
}