// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from aabb.c and aabb.h

// Test if AABB is valid (ported from b2IsValidAABB)
pub fn is_valid_aabb(a : AABB) -> Bool {
  let d = sub(a.upper_bound, a.lower_bound)
  let valid = d.x >= 0.0 && d.y >= 0.0
  valid && is_valid_vec2(a.lower_bound) && is_valid_vec2(a.upper_bound)
}

// Do a and b overlap (ported from b2AABB_Overlaps)
pub fn aabb_overlap(a : AABB, b : AABB) -> Bool {
  not(b.lower_bound.x > a.upper_bound.x || 
      b.lower_bound.y > a.upper_bound.y || 
      a.lower_bound.x > b.upper_bound.x ||
      a.lower_bound.y > b.upper_bound.y)
}

// Does a fully contain b (ported from b2AABB_Contains)
pub fn aabb_contains(a : AABB, b : AABB) -> Bool {
  let mut s = true
  s = s && a.lower_bound.x <= b.lower_bound.x
  s = s && a.lower_bound.y <= b.lower_bound.y
  s = s && b.upper_bound.x <= a.upper_bound.x
  s = s && b.upper_bound.y <= a.upper_bound.y
  s
}

// Get surface area of an AABB (the perimeter length) (ported from b2Perimeter)
pub fn aabb_perimeter(a : AABB) -> Double {
  let wx = a.upper_bound.x - a.lower_bound.x
  let wy = a.upper_bound.y - a.lower_bound.y
  2.0 * (wx + wy)
}

// Enlarge a to contain b (ported from b2EnlargeAABB)
// Returns true if the AABB grew
pub fn enlarge_aabb(a : AABB, b : AABB) -> (AABB, Bool) {
  let mut changed = false
  let mut lower_x = a.lower_bound.x
  let mut lower_y = a.lower_bound.y
  let mut upper_x = a.upper_bound.x
  let mut upper_y = a.upper_bound.y
  
  if b.lower_bound.x < a.lower_bound.x {
    lower_x = b.lower_bound.x
    changed = true
  }
  
  if b.lower_bound.y < a.lower_bound.y {
    lower_y = b.lower_bound.y
    changed = true
  }
  
  if a.upper_bound.x < b.upper_bound.x {
    upper_x = b.upper_bound.x
    changed = true
  }
  
  if a.upper_bound.y < b.upper_bound.y {
    upper_y = b.upper_bound.y
    changed = true
  }
  
  let result : AABB = {
    lower_bound: { x: lower_x, y: lower_y },
    upper_bound: { x: upper_x, y: upper_y }
  }
  
  (result, changed)
}

// Ray cast against AABB (ported from b2AABB_RayCast)
// From Real-time Collision Detection, p179.
pub fn aabb_ray_cast(a : AABB, p1 : Vec2, p2 : Vec2) -> RayResult {
  // Radius not handled
  let no_hit : RayResult = { 
    shape_id: { id: -1 }, 
    point: { x: 0.0, y: 0.0 }, 
    normal: { x: 0.0, y: 0.0 }, 
    fraction: 0.0, 
    hit: false 
  }
  
  let mut tmin = -1.0e30  // -FLT_MAX equivalent
  let mut tmax = 1.0e30   // FLT_MAX equivalent
  
  let p = p1
  let d = sub(p2, p1)
  let abs_d = abs_vec2(d)
  
  let mut normal_x = 0.0
  let mut normal_y = 0.0
  
  // x-coordinate
  if abs_d.x < 1.0e-7 {  // FLT_EPSILON equivalent
    // parallel
    if p.x < a.lower_bound.x || a.upper_bound.x < p.x {
      return no_hit
    }
  } else {
    let inv_d = 1.0 / d.x
    let mut t1 = (a.lower_bound.x - p.x) * inv_d
    let mut t2 = (a.upper_bound.x - p.x) * inv_d
    
    // Sign of the normal vector
    let mut s = -1.0
    
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
      s = 1.0
    }
    
    // Push the min up
    if t1 > tmin {
      normal_y = 0.0
      normal_x = s
      tmin = t1
    }
    
    // Pull the max down
    tmax = min_float(tmax, t2)
    
    if tmin > tmax {
      return no_hit
    }
  }
  
  // y-coordinate
  if abs_d.y < 1.0e-7 {  // FLT_EPSILON equivalent
    // parallel
    if p.y < a.lower_bound.y || a.upper_bound.y < p.y {
      return no_hit
    }
  } else {
    let inv_d = 1.0 / d.y
    let mut t1 = (a.lower_bound.y - p.y) * inv_d
    let mut t2 = (a.upper_bound.y - p.y) * inv_d
    
    // Sign of the normal vector
    let mut s = -1.0
    
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
      s = 1.0
    }
    
    // Push the min up
    if t1 > tmin {
      normal_x = 0.0
      normal_y = s
      tmin = t1
    }
    
    // Pull the max down
    tmax = min_float(tmax, t2)
    
    if tmin > tmax {
      return no_hit
    }
  }
  
  // Does the ray start inside the box?
  // Does the ray intersect beyond the max fraction?
  if tmin < 0.0 || 1.0 < tmin {
    return no_hit
  }
  
  // Intersection
  { 
    shape_id: { id: -1 }, 
    point: lerp(p1, p2, tmin), 
    normal: { x: normal_x, y: normal_y }, 
    fraction: tmin, 
    hit: true 
  }
}