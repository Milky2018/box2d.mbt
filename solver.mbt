// Box2D Constraint Solver - ported from solver.c
// Sequential impulse constraint solver for contacts and joints

// Contact constraint point (ported from b2ContactConstraintPoint)
pub(all) struct ContactConstraintPoint {
  // Contact point data
  r_a : Vec2               // Vector from body A center to contact point
  r_b : Vec2               // Vector from body B center to contact point
  mut normal_impulse : Double  // Accumulated normal impulse
  mut tangent_impulse : Double // Accumulated friction impulse
  
  // Constraint solving data
  normal_mass : Double     // Effective mass for normal constraint
  tangent_mass : Double    // Effective mass for tangent constraint
  velocity_bias : Double   // Bias for position correction
  
  // Warm starting
  warm_normal_impulse : Double  // Previous frame normal impulse
  warm_tangent_impulse : Double // Previous frame tangent impulse
} derive(Eq, Show)

// Contact constraint between two bodies (ported from b2ContactConstraint)  
pub(all) struct ContactConstraint {
  // Bodies involved in contact
  body_a_index : Int       // Index of body A in world bodies array
  body_b_index : Int       // Index of body B in world bodies array
  
  // Contact properties
  normal : Vec2            // Contact normal (from A to B)
  friction : Double        // Combined friction coefficient
  restitution : Double     // Combined restitution coefficient
  tangent : Vec2           // Contact tangent vector (perpendicular to normal)
  
  // Constraint points
  mut points : Array[ContactConstraintPoint] // Contact constraint points
  point_count : Int        // Number of valid contact points
  
  // Mass properties
  inv_mass_a : Double      // Inverse mass of body A  
  inv_mass_b : Double      // Inverse mass of body B
  inv_i_a : Double         // Inverse rotational inertia of body A
  inv_i_b : Double         // Inverse rotational inertia of body B
} derive(Eq, Show)

// Contact solver data for a physics step
pub(all) struct ContactSolver {
  constraints : Array[ContactConstraint]  // All contact constraints
  constraint_count : Int                  // Number of valid constraints
  velocity_iterations : Int               // Number of velocity iterations
  position_iterations : Int               // Number of position iterations  
} derive(Eq, Show)

// Initialize contact constraint from manifold
pub fn init_contact_constraint(
  manifold : Manifold,
  body_a : BodySim, 
  body_b : BodySim,
  body_a_index : Int,
  body_b_index : Int
) -> ContactConstraint {
  if manifold.point_count == 0 {
    return {
      body_a_index, body_b_index,
      normal: vec2_zero, friction: 0.0, restitution: 0.0, tangent: vec2_zero,
      points: [], point_count: 0,
      inv_mass_a: body_a.inv_mass, inv_mass_b: body_b.inv_mass,
      inv_i_a: body_a.inv_inertia, inv_i_b: body_b.inv_inertia
    }
  }
  
  let normal = manifold.normal
  let tangent = { x: normal.y, y: -normal.x } // Perpendicular to normal
  
  // Combine material properties
  let friction = 0.3 // TODO: Use actual shape friction values  
  let restitution = 0.2 // TODO: Use actual shape restitution values
  
  // Initialize constraint points
  let constraint_points = []
  for i, manifold_point in manifold.points {
    if i >= manifold.point_count { break }
    
    // Contact point relative to body centers
    let r_a = sub(manifold_point.point, body_a.transform.p)  
    let r_b = sub(manifold_point.point, body_b.transform.p)
    
    // Compute effective masses for constraints
    let rn_a = cross(r_a, normal)
    let rn_b = cross(r_b, normal)  
    let normal_mass = body_a.inv_mass + body_b.inv_mass + 
                     body_a.inv_inertia * rn_a * rn_a +
                     body_b.inv_inertia * rn_b * rn_b
    
    let rt_a = cross(r_a, tangent)
    let rt_b = cross(r_b, tangent)
    let tangent_mass = body_a.inv_mass + body_b.inv_mass +
                      body_a.inv_inertia * rt_a * rt_a +
                      body_b.inv_inertia * rt_b * rt_b
    
    // Velocity bias for position correction (Baumgarte stabilization)
    let velocity_bias = if manifold_point.separation < 0.0 {
      -0.2 * 60.0 * manifold_point.separation // TODO: Use actual timestep
    } else {
      0.0
    }
    
    let constraint_point = {
      r_a, r_b,
      normal_impulse: 0.0, tangent_impulse: 0.0,
      normal_mass: if normal_mass > 0.0 { 1.0 / normal_mass } else { 0.0 },
      tangent_mass: if tangent_mass > 0.0 { 1.0 / tangent_mass } else { 0.0 },
      velocity_bias,
      warm_normal_impulse: manifold_point.normal_impulse, // Warm start from previous frame
      warm_tangent_impulse: manifold_point.tangent_impulse
    }
    
    constraint_points.push(constraint_point)
  }
  
  {
    body_a_index, body_b_index, normal, friction, restitution, tangent,
    points: constraint_points, point_count: manifold.point_count,
    inv_mass_a: body_a.inv_mass, inv_mass_b: body_b.inv_mass,
    inv_i_a: body_a.inv_inertia, inv_i_b: body_b.inv_inertia
  }
}

// Create contact solver from collision manifolds
pub fn create_contact_solver(
  manifolds : Array[(Int, Int, Manifold)],
  bodies : Array[BodySim],
  velocity_iterations : Int,
  position_iterations : Int
) -> ContactSolver {
  let constraints = []
  
  for entry in manifolds {
    let (body_a_idx, body_b_idx, manifold) = entry
    if manifold.point_count > 0 {
      let body_a = bodies[body_a_idx]
      let body_b = bodies[body_b_idx]
      let constraint = init_contact_constraint(manifold, body_a, body_b, body_a_idx, body_b_idx)
      constraints.push(constraint)
    }
  }
  
  {
    constraints, 
    constraint_count: constraints.length(),
    velocity_iterations,
    position_iterations
  }
}

// Warm start constraints with previous frame impulses
pub fn warm_start_constraints(solver : ContactSolver, bodies : Array[BodySim]) -> Array[BodySim] {
  let result_bodies = bodies.copy()
  
  for constraint in solver.constraints {
    let body_a = result_bodies[constraint.body_a_index]
    let body_b = result_bodies[constraint.body_b_index]
    
    let mut v_a = body_a.linear_velocity
    let mut w_a = body_a.angular_velocity
    let mut v_b = body_b.linear_velocity  
    let mut w_b = body_b.angular_velocity
    
    for point in constraint.points {
      // Apply warm starting impulses
      let warm_P = add(mul_sv(point.warm_normal_impulse, constraint.normal),
                       mul_sv(point.warm_tangent_impulse, constraint.tangent))
      
      // Apply to body A (negative direction)
      v_a = sub(v_a, mul_sv(constraint.inv_mass_a, warm_P))
      w_a = w_a - constraint.inv_i_a * cross(point.r_a, warm_P)
      
      // Apply to body B (positive direction)  
      v_b = add(v_b, mul_sv(constraint.inv_mass_b, warm_P))
      w_b = w_b + constraint.inv_i_b * cross(point.r_b, warm_P)
    }
    
    // Update bodies with new velocities
    result_bodies[constraint.body_a_index] = { ..body_a, linear_velocity: v_a, angular_velocity: w_a }
    result_bodies[constraint.body_b_index] = { ..body_b, linear_velocity: v_b, angular_velocity: w_b }
  }
  
  result_bodies
}

// Solve velocity constraints (main constraint solver)
pub fn solve_velocity_constraints(solver : ContactSolver, bodies : Array[BodySim]) -> Array[BodySim] {
  let result_bodies = bodies.copy()
  
  for constraint in solver.constraints {
    let body_a = result_bodies[constraint.body_a_index]
    let body_b = result_bodies[constraint.body_b_index]
    
    let mut v_a = body_a.linear_velocity
    let mut w_a = body_a.angular_velocity
    let mut v_b = body_b.linear_velocity
    let mut w_b = body_b.angular_velocity
    
    for i, point in constraint.points {
      // Tangent impulse (friction)
      let dv = add(sub(v_b, v_a), 
                   sub(mul_sv(w_b, { x: -point.r_b.y, y: point.r_b.x }),
                       mul_sv(w_a, { x: -point.r_a.y, y: point.r_a.x })))
      let vt = dot(dv, constraint.tangent)
      let mut lambda = point.tangent_mass * (-vt)
      
      // Clamp friction impulse  
      let max_friction = constraint.friction * point.normal_impulse
      let new_impulse = clamp_float(constraint.points[i].tangent_impulse + lambda, -max_friction, max_friction)
      lambda = new_impulse - constraint.points[i].tangent_impulse
      constraint.points[i].tangent_impulse = new_impulse
      
      let tangent_P = mul_sv(lambda, constraint.tangent)
      
      v_a = sub(v_a, mul_sv(constraint.inv_mass_a, tangent_P))
      w_a = w_a - constraint.inv_i_a * cross(point.r_a, tangent_P)
      v_b = add(v_b, mul_sv(constraint.inv_mass_b, tangent_P))
      w_b = w_b + constraint.inv_i_b * cross(point.r_b, tangent_P)
      
      // Normal impulse
      let dv2 = add(sub(v_b, v_a),
                    sub(mul_sv(w_b, { x: -point.r_b.y, y: point.r_b.x }),
                        mul_sv(w_a, { x: -point.r_a.y, y: point.r_a.x })))
      let vn = dot(dv2, constraint.normal)
      let mut lambda2 = point.normal_mass * (-vn + point.velocity_bias)
      
      // Clamp normal impulse (only push, no pull)
      let new_impulse2 = if constraint.points[i].normal_impulse + lambda2 > 0.0 {
        constraint.points[i].normal_impulse + lambda2
      } else {
        0.0
      }
      lambda2 = new_impulse2 - constraint.points[i].normal_impulse
      constraint.points[i].normal_impulse = new_impulse2
      
      let normal_P = mul_sv(lambda2, constraint.normal)
      
      v_a = sub(v_a, mul_sv(constraint.inv_mass_a, normal_P))
      w_a = w_a - constraint.inv_i_a * cross(point.r_a, normal_P)
      v_b = add(v_b, mul_sv(constraint.inv_mass_b, normal_P))  
      w_b = w_b + constraint.inv_i_b * cross(point.r_b, normal_P)
    }
    
    // Update bodies
    result_bodies[constraint.body_a_index] = { ..body_a, linear_velocity: v_a, angular_velocity: w_a }
    result_bodies[constraint.body_b_index] = { ..body_b, linear_velocity: v_b, angular_velocity: w_b }
  }
  
  result_bodies
}

// Use the existing clamp_float function from math_functions.mbt

// Position correction to prevent drift (simplified Baumgarte stabilization)
pub fn solve_position_constraints(solver : ContactSolver, bodies : Array[BodySim]) -> Array[BodySim] {
  let result_bodies = bodies.copy()
  
  for constraint in solver.constraints {
    let body_a = result_bodies[constraint.body_a_index]
    let body_b = result_bodies[constraint.body_b_index]
    
    let mut c_a = body_a.transform.p
    let mut a_a = rot_get_angle(body_a.transform.q)
    let mut c_b = body_b.transform.p  
    let mut a_b = rot_get_angle(body_b.transform.q)
    
    // Only correct if there's significant penetration
    for point in constraint.points {
      let r_a = rotate_vector(make_rot(a_a), sub(point.r_a, body_a.transform.p))
      let r_b = rotate_vector(make_rot(a_b), sub(point.r_b, body_b.transform.p))
      
      let separation = dot(add(sub(c_b, c_a), sub(r_b, r_a)), constraint.normal)
      
      if separation < -0.005 { // Only correct significant penetration
        let correction_C = clamp_float(0.8 * (separation + 0.005), -0.2, 0.0)
        let impulse = -constraint.points[0].normal_mass * correction_C // Use first point's mass
        
        let position_P = mul_sv(impulse, constraint.normal)
        
        c_a = sub(c_a, mul_sv(constraint.inv_mass_a, position_P))
        a_a = a_a - constraint.inv_i_a * cross(r_a, position_P)
        c_b = add(c_b, mul_sv(constraint.inv_mass_b, position_P))
        a_b = a_b + constraint.inv_i_b * cross(r_b, position_P)
      }
    }
    
    // Update body positions
    result_bodies[constraint.body_a_index] = { 
      ..body_a, 
      transform: { p: c_a, q: make_rot(a_a) }
    }
    result_bodies[constraint.body_b_index] = {
      ..body_b,
      transform: { p: c_b, q: make_rot(a_b) }  
    }
  }
  
  result_bodies
}