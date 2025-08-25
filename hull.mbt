// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from hull.c - Convex hull computation using quickhull algorithm

// Note: constants are already defined in distance.mbt

// Hull result (ported from b2Hull)
pub(all) struct Hull {
  points : Array[Vec2]
  count : Int
} derive(Eq, Show)

// Create empty hull
pub fn empty_hull() -> Hull {
  { points: [], count: 0 }
}

// Quickhull recursion (ported from b2RecurseHull)
fn recurse_hull(p1 : Vec2, p2 : Vec2, ps : Array[Vec2]) -> Hull {
  let count = ps.length()
  if count == 0 {
    return empty_hull()
  }
  
  // Create an edge vector pointing from p1 to p2
  let e = normalize(sub(p2, p1))
  
  // Discard points left of e and find point furthest to the right of e
  let right_points : Array[Vec2] = []
  
  let mut best_index = 0
  let mut best_distance = cross(sub(ps[0], p1), e)
  if best_distance > 0.0 {
    right_points.push(ps[0])
  }
  
  for i = 1; i < count; i = i + 1 {
    let distance = cross(sub(ps[i], p1), e)
    if distance > best_distance {
      best_index = i
      best_distance = distance
    }
    
    if distance > 0.0 {
      right_points.push(ps[i])
    }
  }
  
  if best_distance < 2.0 * linear_slop {
    return empty_hull()
  }
  
  let best_point = ps[best_index]
  
  // Compute hull to the right of p1-bestPoint
  let hull1 = recurse_hull(p1, best_point, right_points)
  
  // Compute hull to the right of bestPoint-p2
  let hull2 = recurse_hull(best_point, p2, right_points)
  
  // Stitch together hulls
  let hull_points : Array[Vec2] = []
  
  for i = 0; i < hull1.count; i = i + 1 {
    hull_points.push(hull1.points[i])
  }
  
  hull_points.push(best_point)
  
  for i = 0; i < hull2.count; i = i + 1 {
    hull_points.push(hull2.points[i])
  }
  
  { points: hull_points, count: hull_points.length() }
}

// Compute convex hull using quickhull algorithm (ported from b2ComputeHull)
pub fn compute_hull(points : Array[Vec2]) -> Hull {
  let count = points.length()
  if count < 3 || count > max_polygon_vertices {
    return empty_hull()
  }
  
  let actual_count = min_int(count, max_polygon_vertices)
  
  // Compute AABB for later use
  let mut aabb_lower = { x: 1.0e30, y: 1.0e30 }
  let mut aabb_upper = { x: -1.0e30, y: -1.0e30 }

  // Perform aggressive point welding. First point always remains.
  // Also compute the bounding box for later.
  let ps : Array[Vec2] = []
  let tol_sqr = 16.0 * linear_slop * linear_slop
  
  for i = 0; i < actual_count; i = i + 1 {
    aabb_lower = min_vec2(aabb_lower, points[i])
    aabb_upper = max_vec2(aabb_upper, points[i])
    
    let vi = points[i]
    
    let mut unique = true
    for j = 0; j < ps.length(); j = j + 1 {
      let vj = ps[j]
      let dist_sqr = distance_squared(vi, vj)
      if dist_sqr < tol_sqr {
        unique = false
        break
      }
    }
    
    if unique {
      ps.push(vi)
    }
  }
  
  let n = ps.length()
  if n < 3 {
    // All points very close together
    return empty_hull()
  }
  
  // Find an extreme point as the first point on the hull
  let c = mul_sv(0.5, add(aabb_lower, aabb_upper))  // AABB center
  let mut f1 = 0
  let mut dsq1 = distance_squared(c, ps[0])
  for i = 1; i < n; i = i + 1 {
    let dsq = distance_squared(c, ps[i])
    if dsq > dsq1 {
      f1 = i
      dsq1 = dsq
    }
  }
  
  // Remove p1 from working set
  let p1 = ps[f1]
  ps[f1] = ps[n - 1]
  ps.pop() |> ignore
  let remaining_n = n - 1
  
  let mut f2 = 0
  let mut dsq2 = distance_squared(p1, ps[0])
  for i = 1; i < remaining_n; i = i + 1 {
    let dsq = distance_squared(p1, ps[i])
    if dsq > dsq2 {
      f2 = i
      dsq2 = dsq
    }
  }
  
  // Remove p2 from working set
  let p2 = ps[f2]
  ps[f2] = ps[remaining_n - 1]
  ps.pop() |> ignore
  let final_n = remaining_n - 1
  
  // Split the points into points that are left and right of the line p1-p2
  let right_points : Array[Vec2] = []
  let left_points : Array[Vec2] = []
  
  let e = normalize(sub(p2, p1))
  
  for i = 0; i < final_n; i = i + 1 {
    let d = cross(sub(ps[i], p1), e)
    
    // Slop used here to skip points that are very close to the line p1-p2
    if d >= 2.0 * linear_slop {
      right_points.push(ps[i])
    } else if d <= -2.0 * linear_slop {
      left_points.push(ps[i])
    }
  }
  
  // Compute hulls on right and left
  let hull1 = recurse_hull(p1, p2, right_points)
  let hull2 = recurse_hull(p2, p1, left_points)
  
  if hull1.count == 0 && hull2.count == 0 {
    // All points collinear
    return empty_hull()
  }
  
  // Stitch hulls together, preserving CCW winding order
  let hull_points : Array[Vec2] = []
  hull_points.push(p1)
  
  for i = 0; i < hull1.count; i = i + 1 {
    hull_points.push(hull1.points[i])
  }
  
  hull_points.push(p2)
  
  for i = 0; i < hull2.count; i = i + 1 {
    hull_points.push(hull2.points[i])
  }
  
  let mut hull = { points: hull_points, count: hull_points.length() }
  
  // Merge collinear points
  let mut searching = true
  while searching && hull.count > 2 {
    searching = false
    
    let mut i = 0
    while i < hull.count {
      let i1 = i
      let i2 = (i + 1) % hull.count
      let i3 = (i + 2) % hull.count
      
      let s1 = hull.points[i1]
      let s2 = hull.points[i2]
      let s3 = hull.points[i3]
      
      // Unit edge vector for s1-s3
      let r = normalize(sub(s3, s1))
      
      let distance = cross(sub(s2, s1), r)
      if distance <= 2.0 * linear_slop {
        // Remove midpoint from hull
        let new_points : Array[Vec2] = []
        for j = 0; j < hull.count; j = j + 1 {
          if j != i2 {
            new_points.push(hull.points[j])
          }
        }
        hull = { points: new_points, count: new_points.length() }
        
        // Continue searching for collinear points
        searching = true
        break
      }
      i = i + 1
    }
  }
  
  if hull.count < 3 {
    // All points collinear, shouldn't be reached
    return empty_hull()
  }
  
  hull
}

// Validate hull (ported from b2ValidateHull)
pub fn validate_hull(hull : Hull) -> Bool {
  if hull.count < 3 || hull.count > max_polygon_vertices {
    return false
  }
  
  // Test that every point is behind every edge
  for i = 0; i < hull.count; i = i + 1 {
    // Create an edge vector
    let i1 = i
    let i2 = if i < hull.count - 1 { i1 + 1 } else { 0 }
    let p = hull.points[i1]
    let e = normalize(sub(hull.points[i2], p))
    
    for j = 0; j < hull.count; j = j + 1 {
      // Skip points that subtend the current edge
      if j == i1 || j == i2 {
        continue
      }
      
      let distance = cross(sub(hull.points[j], p), e)
      if distance >= 0.0 {
        return false
      }
    }
  }
  
  // Test for collinear points
  for i = 0; i < hull.count; i = i + 1 {
    let i1 = i
    let i2 = (i + 1) % hull.count
    let i3 = (i + 2) % hull.count
    
    let p1 = hull.points[i1]
    let p2 = hull.points[i2]
    let p3 = hull.points[i3]
    
    let e = normalize(sub(p3, p1))
    
    let distance = cross(sub(p2, p1), e)
    if distance <= linear_slop {
      // p1-p2-p3 are collinear
      return false
    }
  }
  
  true
}