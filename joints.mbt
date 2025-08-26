// Box2D Joints System - ported from joint.c and distance_joint.c
// Joint management and constraint solving for distance joints

// Joint edge for connectivity graph (ported from b2JointEdge)
pub(all) struct JointEdge {
  body_id : Int                   // Connected body ID
  prev_key : Int                  // Previous joint in this body's joint list
  next_key : Int                  // Next joint in this body's joint list
} derive(Eq, Show)

// Core joint structure (ported from b2Joint)
pub(all) struct Joint {
  joint_type : JointType          // Type of this joint
  set_index : Int                 // Index of simulation set
  color_index : Int               // Index in constraint graph color array
  local_index : Int               // Joint index within set or graph color
  edges : (JointEdge, JointEdge)  // Two edges, one for each connected body
  joint_id : Int                  // Unique joint identifier
  island_id : Int                 // Island this joint belongs to
  island_prev : Int               // Previous joint in island
  island_next : Int               // Next joint in island
  collide_connected : Bool        // Whether connected bodies should collide
} derive(Eq, Show)

// Distance joint simulation data (ported from b2DistanceJoint)
pub(all) struct DistanceJoint {
  // Configuration
  length : Double                 // Rest length
  hertz : Double                  // Spring frequency
  damping_ratio : Double          // Spring damping
  lower_spring_force : Double     // Maximum tension
  upper_spring_force : Double     // Maximum compression
  min_length : Double             // Minimum length limit
  max_length : Double             // Maximum length limit
  max_motor_force : Double        // Maximum motor force
  motor_speed : Double            // Desired motor speed
  
  // Solver data
  mut impulse : Double            // Accumulated impulse for distance constraint
  mut lower_impulse : Double      // Accumulated impulse for lower limit
  mut upper_impulse : Double      // Accumulated impulse for upper limit
  mut motor_impulse : Double      // Accumulated impulse for motor
  
  // Cached solver data
  mut index_a : Int               // Body A solver index
  mut index_b : Int               // Body B solver index
  mut anchor_a : Vec2             // Local anchor point on body A
  mut anchor_b : Vec2             // Local anchor point on body B
  mut delta_center : Vec2         // Center-to-center vector
  mut distance_softness : Softness // Soft constraint parameters
  mut axial_mass : Double         // Effective mass along constraint axis
  
  // Control flags
  enable_spring : Bool            // Whether spring behavior is enabled
  enable_limit : Bool             // Whether length limits are enabled
  enable_motor : Bool             // Whether motor is enabled
} derive(Eq, Show)

// Revolute joint simulation data (ported from b2RevoluteJoint)
pub(all) struct RevoluteJoint {
  // Constraint impulses
  mut linear_impulse : Vec2       // Accumulated linear impulse for position constraint
  mut spring_impulse : Double     // Accumulated impulse for spring constraint
  mut motor_impulse : Double      // Accumulated impulse for motor
  mut lower_impulse : Double      // Accumulated impulse for lower limit
  mut upper_impulse : Double      // Accumulated impulse for upper limit
  
  // Configuration
  hertz : Double                  // Spring frequency
  damping_ratio : Double          // Spring damping
  target_angle : Double           // Target angle for spring
  max_motor_torque : Double       // Maximum motor torque
  motor_speed : Double            // Desired motor speed
  lower_angle : Double            // Lower angle limit
  upper_angle : Double            // Upper angle limit
  
  // Cached solver data  
  mut index_a : Int               // Body A solver index
  mut index_b : Int               // Body B solver index
  mut frame_a : Transform         // Local frame on body A
  mut frame_b : Transform         // Local frame on body B
  mut delta_center : Vec2         // Center-to-center vector
  mut axial_mass : Double         // Effective mass for rotational constraint
  mut spring_softness : Softness  // Soft constraint parameters for spring
  
  // Control flags
  enable_spring : Bool            // Whether spring behavior is enabled
  enable_motor : Bool             // Whether motor is enabled
  enable_limit : Bool             // Whether angle limits are enabled
} derive(Eq, Show)

// Joint simulation structure (ported from b2JointSim)
pub(all) struct JointSim {
  joint_type : JointType          // Joint type identifier
  body_id_a : Int                 // First body ID
  body_id_b : Int                 // Second body ID
  local_frame_a : Transform       // Local frame on body A
  local_frame_b : Transform       // Local frame on body B
  
  // Solver cached data
  mut inv_mass_a : Double         // Inverse mass of body A
  mut inv_mass_b : Double         // Inverse mass of body B
  mut inv_ia : Double             // Inverse rotational inertia of body A
  mut inv_ib : Double             // Inverse rotational inertia of body B
  mut constraint_softness : Softness // Constraint softness parameters
  
  // Specific joint data (union-like structure)
  distance_joint : DistanceJoint  // Distance joint specific data
  revolute_joint : RevoluteJoint  // Revolute joint specific data
  prismatic_joint : PrismaticJoint // Prismatic joint specific data
} derive(Eq, Show)

// Create default distance joint definition
pub fn default_distance_joint_def() -> DistanceJointDef {
  {
    base: {
      body_id_a: { id: -1 },
      body_id_b: { id: -1 },
      collide_connected: false
    },
    length: 1.0,
    enable_spring: false,
    lower_spring_force: 0.0,
    upper_spring_force: 0.0,
    hertz: 0.0,
    damping_ratio: 0.0,
    enable_limit: false,
    min_length: 0.0,
    max_length: 1.0e6,  // Large default max length
    enable_motor: false,
    max_motor_force: 0.0,
    motor_speed: 0.0
  }
}

// Create default revolute joint definition
pub fn default_revolute_joint_def() -> RevoluteJointDef {
  {
    base: {
      body_id_a: { id: -1 },
      body_id_b: { id: -1 },
      collide_connected: false
    },
    target_angle: 0.0,
    enable_spring: false,
    hertz: 0.0,
    damping_ratio: 0.0,
    enable_limit: false,
    lower_angle: 0.0,
    upper_angle: 0.0,
    enable_motor: false,
    max_motor_torque: 0.0,
    motor_speed: 0.0
  }
}

// Create distance joint simulation structure
pub fn create_distance_joint_sim(
  def : DistanceJointDef, 
  anchor_a : Vec2, 
  anchor_b : Vec2
) -> JointSim {
  // Create local frames at the anchor points
  let local_frame_a = { p: anchor_a, q: { c: 1.0, s: 0.0 } }
  let local_frame_b = { p: anchor_b, q: { c: 1.0, s: 0.0 } }
  
  // Create the distance joint data
  let distance_joint = {
    length: def.length,
    hertz: def.hertz,
    damping_ratio: def.damping_ratio,
    lower_spring_force: def.lower_spring_force,
    upper_spring_force: def.upper_spring_force,
    min_length: def.min_length,
    max_length: def.max_length,
    max_motor_force: def.max_motor_force,
    motor_speed: def.motor_speed,
    impulse: 0.0,
    lower_impulse: 0.0,
    upper_impulse: 0.0,
    motor_impulse: 0.0,
    index_a: -1,
    index_b: -1,
    anchor_a: { x: 0.0, y: 0.0 },
    anchor_b: { x: 0.0, y: 0.0 },
    delta_center: { x: 0.0, y: 0.0 },
    distance_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 },
    axial_mass: 0.0,
    enable_spring: def.enable_spring,
    enable_limit: def.enable_limit,
    enable_motor: def.enable_motor
  }
  
  {
    joint_type: DistanceJoint,
    body_id_a: def.base.body_id_a.id,
    body_id_b: def.base.body_id_b.id,
    local_frame_a,
    local_frame_b,
    inv_mass_a: 0.0,
    inv_mass_b: 0.0,
    inv_ia: 0.0,
    inv_ib: 0.0,
    constraint_softness: { bias_rate: 0.2, mass_scale: 1.0, impulse_scale: 0.0 }, // Default bias for position correction
    distance_joint,
    revolute_joint: {  // Create dummy revolute joint data (not used for distance joints)
      linear_impulse: { x: 0.0, y: 0.0 },
      spring_impulse: 0.0,
      motor_impulse: 0.0,
      lower_impulse: 0.0,
      upper_impulse: 0.0,
      hertz: 0.0,
      damping_ratio: 0.0,
      target_angle: 0.0,
      max_motor_torque: 0.0,
      motor_speed: 0.0,
      lower_angle: 0.0,
      upper_angle: 0.0,
      index_a: -1,
      index_b: -1,
      frame_a: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
      frame_b: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
      delta_center: { x: 0.0, y: 0.0 },
      axial_mass: 0.0,
      spring_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 },
      enable_spring: false,
      enable_motor: false,
      enable_limit: false
    },
    prismatic_joint: {  // Create dummy prismatic joint data (not used for distance joints)
      target_translation: 0.0,
      hertz: 0.0,
      damping_ratio: 0.0,
      lower_translation: 0.0,
      upper_translation: 0.0,
      max_motor_force: 0.0,
      motor_speed: 0.0,
      enable_spring: false,
      enable_limit: false,
      enable_motor: false,
      impulse: { x: 0.0, y: 0.0 },
      spring_impulse: 0.0,
      motor_impulse: 0.0,
      lower_impulse: 0.0,
      upper_impulse: 0.0,
      index_a: -1,
      index_b: -1,
      frame_a: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
      frame_b: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
      delta_center: { x: 0.0, y: 0.0 },
      axial_mass: 0.0,
      spring_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 }
    }
  }
}

// Softness computation (ported from b2MakeSoft in Box2D)
pub fn make_soft(hertz : Double, damping_ratio : Double, time_step : Double) -> Softness {
  if hertz == 0.0 {
    return { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 }
  }
  
  let omega = 2.0 * pi * hertz
  let d = 2.0 * damping_ratio * omega
  let k = omega * omega
  
  let h = time_step
  let a1 = 1.0 + h * d + h * h * k
  let a2 = h * k
  let a3 = h * d + h * h * k
  
  {
    bias_rate: a2 / a1,
    mass_scale: 1.0 / a1,
    impulse_scale: a3 / a1
  }
}

// Create revolute joint simulation structure
pub fn create_revolute_joint_sim(
  def : RevoluteJointDef,
  anchor : Vec2    // Single anchor point (same for both bodies in local space)
) -> JointSim {
  // Create local frames at the anchor point
  let local_frame = { p: anchor, q: { c: 1.0, s: 0.0 } }
  
  // Create the revolute joint data
  let revolute_joint = {
    linear_impulse: { x: 0.0, y: 0.0 },
    spring_impulse: 0.0,
    motor_impulse: 0.0,
    lower_impulse: 0.0,
    upper_impulse: 0.0,
    hertz: def.hertz,
    damping_ratio: def.damping_ratio,
    target_angle: def.target_angle,
    max_motor_torque: def.max_motor_torque,
    motor_speed: def.motor_speed,
    lower_angle: def.lower_angle,
    upper_angle: def.upper_angle,
    index_a: -1,
    index_b: -1,
    frame_a: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
    frame_b: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
    delta_center: { x: 0.0, y: 0.0 },
    axial_mass: 0.0,
    spring_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 },
    enable_spring: def.enable_spring,
    enable_motor: def.enable_motor,
    enable_limit: def.enable_limit
  }
  
  {
    joint_type: RevoluteJoint,
    body_id_a: def.base.body_id_a.id,
    body_id_b: def.base.body_id_b.id,
    local_frame_a: local_frame,
    local_frame_b: local_frame,
    inv_mass_a: 0.0,
    inv_mass_b: 0.0,
    inv_ia: 0.0,
    inv_ib: 0.0,
    constraint_softness: { bias_rate: 0.2, mass_scale: 1.0, impulse_scale: 0.0 }, // Default bias for position correction
    distance_joint: {  // Create dummy distance joint data (not used for revolute joints)
      length: 1.0,
      hertz: 0.0,
      damping_ratio: 0.0,
      lower_spring_force: 0.0,
      upper_spring_force: 0.0,
      min_length: 0.0,
      max_length: 1.0e6,
      max_motor_force: 0.0,
      motor_speed: 0.0,
      impulse: 0.0,
      lower_impulse: 0.0,
      upper_impulse: 0.0,
      motor_impulse: 0.0,
      index_a: -1,
      index_b: -1,
      anchor_a: { x: 0.0, y: 0.0 },
      anchor_b: { x: 0.0, y: 0.0 },
      delta_center: { x: 0.0, y: 0.0 },
      distance_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 },
      axial_mass: 0.0,
      enable_spring: false,
      enable_limit: false,
      enable_motor: false
    },
    revolute_joint,
    prismatic_joint: {  // Create dummy prismatic joint data (not used for revolute joints)
      target_translation: 0.0,
      hertz: 0.0,
      damping_ratio: 0.0,
      lower_translation: 0.0,
      upper_translation: 0.0,
      max_motor_force: 0.0,
      motor_speed: 0.0,
      enable_spring: false,
      enable_limit: false,
      enable_motor: false,
      impulse: { x: 0.0, y: 0.0 },
      spring_impulse: 0.0,
      motor_impulse: 0.0,
      lower_impulse: 0.0,
      upper_impulse: 0.0,
      index_a: -1,
      index_b: -1,
      frame_a: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
      frame_b: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
      delta_center: { x: 0.0, y: 0.0 },
      axial_mass: 0.0,
      spring_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 }
    }
  }
}

// Prepare distance joint for solving (ported from b2PrepareDistanceJoint)
pub fn prepare_distance_joint(
  joint_sim : JointSim, 
  bodies : Array[BodySim], 
  time_step : Double
) -> JointSim {
  if joint_sim.joint_type != DistanceJoint {
    return joint_sim
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return joint_sim
  }
  
  let body_a = bodies[body_id_a]
  let body_b = bodies[body_id_b]
  
  // Update cached inverse mass data
  let updated_joint = { 
    ..joint_sim,
    inv_mass_a: body_a.inv_mass,
    inv_mass_b: body_b.inv_mass,
    inv_ia: body_a.inv_inertia,
    inv_ib: body_b.inv_inertia
  }
  
  // Update distance joint specific data
  let distance_joint = updated_joint.distance_joint
  
  // Compute anchors in world coordinates relative to body centers
  let anchor_a = rotate_vector(body_a.transform.q, 
    sub(joint_sim.local_frame_a.p, body_a.center))
  let anchor_b = rotate_vector(body_b.transform.q, 
    sub(joint_sim.local_frame_b.p, body_b.center))
  let delta_center = sub(body_b.center, body_a.center)
  
  // Compute constraint axis
  let separation = add(sub(anchor_b, anchor_a), delta_center)
  let axis = normalize(separation)
  
  // Compute effective mass along constraint axis
  let cr_a = cross(anchor_a, axis)
  let cr_b = cross(anchor_b, axis)
  let k = body_a.inv_mass + body_b.inv_mass + 
          body_a.inv_inertia * cr_a * cr_a + 
          body_b.inv_inertia * cr_b * cr_b
  let axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 }
  
  // Compute softness for spring behavior
  let distance_softness = make_soft(
    distance_joint.hertz, 
    distance_joint.damping_ratio, 
    time_step
  )
  
  // Create updated distance joint with computed values
  let updated_distance_joint = {
    ..distance_joint,
    index_a: body_id_a,
    index_b: body_id_b,
    anchor_a,
    anchor_b,
    delta_center,
    axial_mass,
    distance_softness
  }
  
  { ..updated_joint, distance_joint: updated_distance_joint }
}

// Warm start distance joint (ported from b2WarmStartDistanceJoint)
pub fn warm_start_distance_joint(
  joint_sim : JointSim,
  bodies : Array[BodySim]
) -> Array[BodySim] {
  
  if joint_sim.joint_type != DistanceJoint {
    return bodies
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return bodies
  }
  
  let distance_joint = joint_sim.distance_joint
  let result = bodies.copy()
  
  let body_a = result[body_id_a]
  let body_b = result[body_id_b]
  
  
  let m_a = joint_sim.inv_mass_a
  let m_b = joint_sim.inv_mass_b
  let i_a = joint_sim.inv_ia
  let i_b = joint_sim.inv_ib
  
  // Recompute current anchor positions (like Box2D does in warm start)
  let r_a = rotate_vector(body_a.transform.q, sub(joint_sim.local_frame_a.p, body_a.center))
  let r_b = rotate_vector(body_b.transform.q, sub(joint_sim.local_frame_b.p, body_b.center))
  
  // Current center-to-center vector
  let delta_center = sub(body_b.transform.p, body_a.transform.p)
  
  // Current separation between anchor points
  let separation = add(sub(r_b, r_a), delta_center)
  
  
  // Use separation vector if valid, otherwise use constraint direction
  let axis = if length(separation) > 0.001 {
    normalize(separation)
  } else {
    // When bodies are at constraint length, use body center direction
    let center_separation = sub(body_b.transform.p, body_a.transform.p)
    if length(center_separation) > 0.001 {
      normalize(center_separation)
    } else {
      // Fallback to unit vector along x-axis
      { x: 1.0, y: 0.0 }
    }
  }
  
  
  // Apply accumulated impulses (even if no constraint violation exists)
  let total_impulse = distance_joint.impulse + distance_joint.lower_impulse - 
                     distance_joint.upper_impulse + distance_joint.motor_impulse
  
                     
  // Only apply if there's accumulated impulse
  if total_impulse != 0.0 {
    let impulse_vector = mul_sv(total_impulse, axis)
    
    
    // Update body velocities (apply impulse changes bodies B positive, A negative)
    let new_vel_a = sub(body_a.linear_velocity, mul_sv(m_a, impulse_vector))
    let new_omega_a = body_a.angular_velocity - i_a * cross(r_a, impulse_vector)
    let new_vel_b = add(body_b.linear_velocity, mul_sv(m_b, impulse_vector))
    let new_omega_b = body_b.angular_velocity + i_b * cross(r_b, impulse_vector)
    
    result[body_id_a] = { ..body_a, linear_velocity: new_vel_a, angular_velocity: new_omega_a }
    result[body_id_b] = { ..body_b, linear_velocity: new_vel_b, angular_velocity: new_omega_b }
  }
  
  result
}

// Solve distance joint velocity constraints (ported from b2SolveDistanceJoint)
pub fn solve_distance_joint_velocity(
  joint_sim : JointSim,
  bodies : Array[BodySim],
  _time_step : Double,
  use_bias : Bool
) -> (JointSim, Array[BodySim]) {
  if joint_sim.joint_type != DistanceJoint {
    return (joint_sim, bodies)
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return (joint_sim, bodies)
  }
  
  let distance_joint = joint_sim.distance_joint
  let result = bodies.copy()
  
  let body_a = result[body_id_a]
  let body_b = result[body_id_b]
  
  let m_a = joint_sim.inv_mass_a
  let m_b = joint_sim.inv_mass_b
  let i_a = joint_sim.inv_ia
  let i_b = joint_sim.inv_ib
  
  // Compute world positions of anchor points
  let world_anchor_a = add(body_a.transform.p, rotate_vector(body_a.transform.q, joint_sim.local_frame_a.p))
  let world_anchor_b = add(body_b.transform.p, rotate_vector(body_b.transform.q, joint_sim.local_frame_b.p))
  
  // Current separation is simply the distance between world anchors
  let separation = sub(world_anchor_b, world_anchor_a)
  let current_length = length(separation)
  
  if current_length < 0.001 {
    return (joint_sim, bodies)
  }
  
  let axis = mul_sv(1.0 / current_length, separation)
  
  // Compute anchor positions relative to body centers (needed for velocity constraints)
  let r_a = rotate_vector(body_a.transform.q, joint_sim.local_frame_a.p)
  let r_b = rotate_vector(body_b.transform.q, joint_sim.local_frame_b.p)
  
  // Get current velocities
  let v_a = body_a.linear_velocity
  let w_a = body_a.angular_velocity
  let v_b = body_b.linear_velocity
  let w_b = body_b.angular_velocity
  
  // Solve main distance constraint (spring or rigid)  
  let v_r = add(sub(v_b, v_a), sub(cross_sv(w_b, r_b), cross_sv(w_a, r_a)))
  let c_dot = dot(axis, v_r)
  let constraint_error = current_length - distance_joint.length
  
  let (bias, mass_coeff, impulse_coeff) = if distance_joint.enable_spring {
    // Spring constraint parameters
    let spring_bias = distance_joint.distance_softness.bias_rate * constraint_error
    let spring_mass_coeff = distance_joint.distance_softness.mass_scale
    let spring_impulse_coeff = distance_joint.distance_softness.impulse_scale
    (spring_bias, spring_mass_coeff, spring_impulse_coeff)
  } else {
    // Rigid constraint parameters - Box2D approach
    let rigid_bias = if use_bias { joint_sim.constraint_softness.bias_rate * constraint_error } else { 0.0 }
    let rigid_mass_scale = if use_bias { joint_sim.constraint_softness.mass_scale } else { 1.0 }
    let rigid_impulse_scale = if use_bias { joint_sim.constraint_softness.impulse_scale } else { 0.0 }
    (rigid_bias, rigid_mass_scale, rigid_impulse_scale)
  }
  
  // Calculate impulse (following Box2D formula exactly)
  let impulse = -mass_coeff * distance_joint.axial_mass * (c_dot + bias) - 
                impulse_coeff * distance_joint.impulse
  let new_impulse = distance_joint.impulse + impulse
  let updated_distance_joint = { ..distance_joint, impulse: new_impulse }
  
  
  let constraint_impulse = mul_sv(impulse, axis)
  let new_v_a = sub(v_a, mul_sv(m_a, constraint_impulse))
  let new_w_a = w_a - i_a * cross(r_a, constraint_impulse)
  let new_v_b = add(v_b, mul_sv(m_b, constraint_impulse))
  let new_w_b = w_b + i_b * cross(r_b, constraint_impulse)
  
  // Update body velocities
  result[body_id_a] = { ..body_a, linear_velocity: new_v_a, angular_velocity: new_w_a }
  result[body_id_b] = { ..body_b, linear_velocity: new_v_b, angular_velocity: new_w_b }
  
  let updated_joint_sim = { ..joint_sim, distance_joint: updated_distance_joint }
  (updated_joint_sim, result)
}

// Get current length of distance joint
pub fn get_distance_joint_length(joint_sim : JointSim, bodies : Array[BodySim]) -> Double {
  if joint_sim.joint_type != DistanceJoint {
    return 0.0
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return 0.0
  }
  
  let body_a = bodies[body_id_a]
  let body_b = bodies[body_id_b]
  
  let p_a = transform_point(body_a.transform, joint_sim.local_frame_a.p)
  let p_b = transform_point(body_b.transform, joint_sim.local_frame_b.p)
  
  length(sub(p_b, p_a))
}

// Prepare revolute joint for solving (ported from b2PrepareRevoluteJoint)
pub fn prepare_revolute_joint(
  joint_sim : JointSim,
  bodies : Array[BodySim], 
  time_step : Double
) -> JointSim {
  if joint_sim.joint_type != RevoluteJoint {
    return joint_sim
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return joint_sim
  }
  
  let body_a = bodies[body_id_a]
  let body_b = bodies[body_id_b]
  
  // Update cached inverse mass data
  let updated_joint = { 
    ..joint_sim,
    inv_mass_a: body_a.inv_mass,
    inv_mass_b: body_b.inv_mass,
    inv_ia: body_a.inv_inertia,
    inv_ib: body_b.inv_inertia
  }
  
  // Update revolute joint specific data
  let revolute_joint = updated_joint.revolute_joint
  
  // Compute joint anchor frames with world space rotation, relative to center of mass
  let frame_a_q = mul_rot(body_a.transform.q, joint_sim.local_frame_a.q)
  let frame_a_p = rotate_vector(body_a.transform.q, sub(joint_sim.local_frame_a.p, body_a.center))
  let frame_b_q = mul_rot(body_b.transform.q, joint_sim.local_frame_b.q)  
  let frame_b_p = rotate_vector(body_b.transform.q, sub(joint_sim.local_frame_b.p, body_b.center))
  
  // Compute the initial center delta
  let delta_center = sub(body_b.center, body_a.center)
  
  // Compute effective mass for rotation constraint (I_A + I_B)
  let axial_mass = body_a.inv_inertia + body_b.inv_inertia
  let effective_axial_mass = if axial_mass > 0.0 { 1.0 / axial_mass } else { 0.0 }
  
  // Compute softness for spring behavior
  let spring_softness = make_soft(
    revolute_joint.hertz,
    revolute_joint.damping_ratio,
    time_step
  )
  
  // Create updated revolute joint with computed values
  let updated_revolute_joint = {
    ..revolute_joint,
    index_a: body_id_a,
    index_b: body_id_b,
    frame_a: { p: frame_a_p, q: frame_a_q },
    frame_b: { p: frame_b_p, q: frame_b_q },
    delta_center,
    axial_mass: effective_axial_mass,
    spring_softness
  }
  
  { ..updated_joint, revolute_joint: updated_revolute_joint }
}

// Warm start revolute joint (ported from b2WarmStartRevoluteJoint)
pub fn warm_start_revolute_joint(
  joint_sim : JointSim,
  bodies : Array[BodySim]
) -> Array[BodySim] {
  if joint_sim.joint_type != RevoluteJoint {
    return bodies
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return bodies
  }
  
  let revolute_joint = joint_sim.revolute_joint
  let result = bodies.copy()
  
  let body_a = result[body_id_a]
  let body_b = result[body_id_b]
  
  let m_a = joint_sim.inv_mass_a
  let m_b = joint_sim.inv_mass_b
  let i_a = joint_sim.inv_ia
  let i_b = joint_sim.inv_ib
  
  // Apply accumulated linear impulse (position constraint)
  let linear_impulse = revolute_joint.linear_impulse
  let new_v_a = sub(body_a.linear_velocity, mul_sv(m_a, linear_impulse))
  let new_v_b = add(body_b.linear_velocity, mul_sv(m_b, linear_impulse))
  
  // Apply accumulated angular impulses (rotation constraints)
  let total_angular_impulse = revolute_joint.spring_impulse + revolute_joint.motor_impulse + 
                             revolute_joint.lower_impulse - revolute_joint.upper_impulse
  let new_w_a = body_a.angular_velocity - i_a * total_angular_impulse
  let new_w_b = body_b.angular_velocity + i_b * total_angular_impulse
  
  result[body_id_a] = { ..body_a, linear_velocity: new_v_a, angular_velocity: new_w_a }
  result[body_id_b] = { ..body_b, linear_velocity: new_v_b, angular_velocity: new_w_b }
  
  result
}

// Solve revolute joint velocity constraints (ported from b2SolveRevoluteJoint)
pub fn solve_revolute_joint_velocity(
  joint_sim : JointSim,
  bodies : Array[BodySim],
  time_step : Double,
  use_bias : Bool
) -> (JointSim, Array[BodySim]) {
  if joint_sim.joint_type != RevoluteJoint {
    return (joint_sim, bodies)
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return (joint_sim, bodies)
  }
  
  let revolute_joint = joint_sim.revolute_joint
  let result = bodies.copy()
  
  let body_a = result[body_id_a]
  let body_b = result[body_id_b]
  
  let m_a = joint_sim.inv_mass_a
  let m_b = joint_sim.inv_mass_b
  let i_a = joint_sim.inv_ia
  let i_b = joint_sim.inv_ib
  
  // Get current velocities
  let mut v_a = body_a.linear_velocity
  let mut w_a = body_a.angular_velocity
  let mut v_b = body_b.linear_velocity
  let mut w_b = body_b.angular_velocity
  
  // Check if rotation is fixed (both bodies have zero rotational inertia)
  let fixed_rotation = (i_a + i_b == 0.0)
  
  // Current relative rotation
  let q_a = revolute_joint.frame_a.q
  let q_b = revolute_joint.frame_b.q
  let rel_q = inv_mul_rot(q_a, q_b)
  
  // Updated revolute joint data
  let mut updated_revolute_joint = revolute_joint
  
  // Solve spring constraint
  if revolute_joint.enable_spring && !fixed_rotation {
    let joint_angle = rot_get_angle(rel_q)
    let joint_angle_delta = unwind_angle(joint_angle - revolute_joint.target_angle)
    
    let constraint_error = joint_angle_delta
    let bias = revolute_joint.spring_softness.bias_rate * constraint_error
    let mass_scale = revolute_joint.spring_softness.mass_scale
    let impulse_scale = revolute_joint.spring_softness.impulse_scale
    
    let c_dot = w_b - w_a
    let impulse = -mass_scale * revolute_joint.axial_mass * (c_dot + bias) - 
                  impulse_scale * revolute_joint.spring_impulse
    updated_revolute_joint = { ..updated_revolute_joint, spring_impulse: updated_revolute_joint.spring_impulse + impulse }
    
    w_a = w_a - i_a * impulse
    w_b = w_b + i_b * impulse
  }
  
  // Solve motor constraint  
  if revolute_joint.enable_motor && !fixed_rotation {
    let c_dot = w_b - w_a - revolute_joint.motor_speed
    let impulse = -revolute_joint.axial_mass * c_dot
    let old_impulse = updated_revolute_joint.motor_impulse
    let max_impulse = time_step * revolute_joint.max_motor_torque  // Convert torque to impulse using time_step
    let new_motor_impulse = clamp_float(updated_revolute_joint.motor_impulse + impulse, -max_impulse, max_impulse)
    let final_impulse = new_motor_impulse - old_impulse
    updated_revolute_joint = { ..updated_revolute_joint, motor_impulse: new_motor_impulse }
    
    w_a = w_a - i_a * final_impulse
    w_b = w_b + i_b * final_impulse
  }
  
  // Solve position constraint (point-to-point)
  // This keeps the anchor points coincident
  let r_a = revolute_joint.frame_a.p
  let r_b = revolute_joint.frame_b.p
  
  // Compute relative velocity at anchor points
  let v_r = add(sub(v_b, v_a), sub(cross_sv(w_b, r_b), cross_sv(w_a, r_a)))
  
  // Compute position error for bias correction
  let bias = if use_bias {
    // Compute current separation between anchor points
    let separation = add(sub(r_b, r_a), revolute_joint.delta_center)
    mul_sv(joint_sim.constraint_softness.bias_rate, separation)
  } else {
    { x: 0.0, y: 0.0 }
  }
  
  let mass_scale = if use_bias { joint_sim.constraint_softness.mass_scale } else { 1.0 }
  let impulse_scale = if use_bias { joint_sim.constraint_softness.impulse_scale } else { 0.0 }
  
  // Solve for linear impulse to eliminate relative velocity and position error
  // Mass matrix for 2D point constraint
  let k_11 = m_a + m_b + i_a * r_a.y * r_a.y + i_b * r_b.y * r_b.y
  let k_12 = -i_a * r_a.x * r_a.y - i_b * r_b.x * r_b.y
  let k_22 = m_a + m_b + i_a * r_a.x * r_a.x + i_b * r_b.x * r_b.x
  
  let det = k_11 * k_22 - k_12 * k_12
  let impulse_x = if det > 0.0 { 
    -mass_scale * (k_22 * (v_r.x + bias.x) - k_12 * (v_r.y + bias.y)) / det - 
    impulse_scale * updated_revolute_joint.linear_impulse.x
  } else { 0.0 }
  let impulse_y = if det > 0.0 { 
    -mass_scale * (-k_12 * (v_r.x + bias.x) + k_11 * (v_r.y + bias.y)) / det - 
    impulse_scale * updated_revolute_joint.linear_impulse.y
  } else { 0.0 }
  let linear_impulse = { x: impulse_x, y: impulse_y }
  
  updated_revolute_joint = { ..updated_revolute_joint, linear_impulse: add(updated_revolute_joint.linear_impulse, linear_impulse) }
  
  // Apply linear impulse
  v_a = sub(v_a, mul_sv(m_a, linear_impulse))
  w_a = w_a - i_a * cross(r_a, linear_impulse)
  v_b = add(v_b, mul_sv(m_b, linear_impulse))
  w_b = w_b + i_b * cross(r_b, linear_impulse)
  
  // Update body velocities
  result[body_id_a] = { ..body_a, linear_velocity: v_a, angular_velocity: w_a }
  result[body_id_b] = { ..body_b, linear_velocity: v_b, angular_velocity: w_b }
  
  let updated_joint_sim = { ..joint_sim, revolute_joint: updated_revolute_joint }
  (updated_joint_sim, result)
}

// Prismatic joint simulation data (ported from b2PrismaticJoint)
pub(all) struct PrismaticJoint {
  // Configuration
  target_translation : Double     // Target translation for spring-damper
  hertz : Double                  // Spring frequency
  damping_ratio : Double          // Spring damping ratio
  lower_translation : Double      // Lower translation limit
  upper_translation : Double      // Upper translation limit
  max_motor_force : Double        // Maximum motor force
  motor_speed : Double            // Desired motor speed
  enable_spring : Bool            // Enable spring behavior
  enable_limit : Bool             // Enable translation limits
  enable_motor : Bool             // Enable motor
  
  // Solver data
  mut impulse : Vec2              // Accumulated impulse for perpendicular and angular constraints
  mut spring_impulse : Double     // Accumulated impulse for spring constraint
  mut motor_impulse : Double      // Accumulated impulse for motor
  mut lower_impulse : Double      // Accumulated impulse for lower limit
  mut upper_impulse : Double      // Accumulated impulse for upper limit
  
  // Cached solver data
  mut index_a : Int               // Body A solver index
  mut index_b : Int               // Body B solver index
  mut frame_a : Transform         // Local frame on body A
  mut frame_b : Transform         // Local frame on body B
  mut delta_center : Vec2         // Center-to-center vector
  mut axial_mass : Double         // Effective mass for axial constraint
  mut spring_softness : Softness  // Soft constraint parameters for spring
} derive(Eq, Show)

// Create default prismatic joint definition
pub fn default_prismatic_joint_def() -> PrismaticJointDef {
  {
    base: {
      body_id_a: { id: -1 },
      body_id_b: { id: -1 },
      collide_connected: false
    },
    target_translation: 0.0,
    enable_spring: false,
    hertz: 0.0,
    damping_ratio: 0.0,
    enable_limit: false,
    lower_translation: 0.0,
    upper_translation: 0.0,
    enable_motor: false,
    max_motor_force: 0.0,
    motor_speed: 0.0
  }
}

// Create prismatic joint simulation (ported from b2CreatePrismaticJoint)
pub fn create_prismatic_joint_sim(
  def : PrismaticJointDef,
  anchor_a : Vec2,    // Anchor point on body A in local space
  anchor_b : Vec2,    // Anchor point on body B in local space
  axis_a : Vec2       // Axis of translation on body A in local space (normalized)
) -> JointSim {
  // Create local frames: body A has the axis, body B has the anchor
  let axis_rotation = make_rot_from_unit_vector(normalize(axis_a))
  let local_frame_a = { p: anchor_a, q: axis_rotation }
  let local_frame_b = { p: anchor_b, q: { c: 1.0, s: 0.0 } }
  
  let default_distance_joint = {
    length: 1.0,
    hertz: 0.0,
    damping_ratio: 0.0,
    lower_spring_force: 0.0,
    upper_spring_force: 0.0,
    min_length: 0.0,
    max_length: 0.0,
    max_motor_force: 0.0,
    motor_speed: 0.0,
    impulse: 0.0,
    lower_impulse: 0.0,
    upper_impulse: 0.0,
    motor_impulse: 0.0,
    index_a: -1,
    index_b: -1,
    anchor_a: { x: 0.0, y: 0.0 },
    anchor_b: { x: 0.0, y: 0.0 },
    delta_center: { x: 0.0, y: 0.0 },
    distance_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 },
    axial_mass: 0.0,
    enable_spring: false,
    enable_limit: false,
    enable_motor: false
  }
  
  let default_revolute_joint = {
    linear_impulse: { x: 0.0, y: 0.0 },
    spring_impulse: 0.0,
    motor_impulse: 0.0,
    lower_impulse: 0.0,
    upper_impulse: 0.0,
    index_a: -1,
    index_b: -1,
    frame_a: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
    frame_b: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
    delta_center: { x: 0.0, y: 0.0 },
    axial_mass: 0.0,
    spring_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 },
    target_angle: 0.0,
    hertz: 0.0,
    damping_ratio: 0.0,
    lower_angle: 0.0,
    upper_angle: 0.0,
    max_motor_torque: 0.0,
    motor_speed: 0.0,
    enable_spring: false,
    enable_limit: false,
    enable_motor: false
  }
  
  let prismatic_joint = {
    target_translation: def.target_translation,
    hertz: def.hertz,
    damping_ratio: def.damping_ratio,
    lower_translation: def.lower_translation,
    upper_translation: def.upper_translation,
    max_motor_force: def.max_motor_force,
    motor_speed: def.motor_speed,
    enable_spring: def.enable_spring,
    enable_limit: def.enable_limit,
    enable_motor: def.enable_motor,
    impulse: { x: 0.0, y: 0.0 },
    spring_impulse: 0.0,
    motor_impulse: 0.0,
    lower_impulse: 0.0,
    upper_impulse: 0.0,
    index_a: -1,
    index_b: -1,
    frame_a: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
    frame_b: { p: { x: 0.0, y: 0.0 }, q: { c: 1.0, s: 0.0 } },
    delta_center: { x: 0.0, y: 0.0 },
    axial_mass: 0.0,
    spring_softness: { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 }
  }
  
  {
    joint_type: PrismaticJoint,
    body_id_a: def.base.body_id_a.id,
    body_id_b: def.base.body_id_b.id,
    local_frame_a: local_frame_a,
    local_frame_b: local_frame_b,
    inv_mass_a: 0.0,
    inv_mass_b: 0.0,
    inv_ia: 0.0,
    inv_ib: 0.0,
    constraint_softness: { bias_rate: 0.2, mass_scale: 1.0, impulse_scale: 0.0 }, // Default bias for position correction
    distance_joint: default_distance_joint,
    revolute_joint: default_revolute_joint,
    prismatic_joint: prismatic_joint
  }
}

// Prepare prismatic joint for solving (ported from b2PreparePrismaticJoint)
pub fn prepare_prismatic_joint(
  joint_sim : JointSim,
  bodies : Array[BodySim],
  time_step : Double
) -> JointSim {
  if joint_sim.joint_type != PrismaticJoint {
    return joint_sim
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return joint_sim
  }
  
  let body_a = bodies[body_id_a]
  let body_b = bodies[body_id_b]
  
  let m_a = body_a.inv_mass
  let m_b = body_b.inv_mass
  let i_a = body_a.inv_inertia
  let i_b = body_b.inv_inertia
  
  let prismatic_joint = joint_sim.prismatic_joint
  
  // Compute joint frame data
  let frame_a_q = mul_rot(body_a.transform.q, joint_sim.local_frame_a.q)
  let frame_a_p = rotate_vector(body_a.transform.q, joint_sim.local_frame_a.p)
  let frame_b_q = mul_rot(body_b.transform.q, joint_sim.local_frame_b.q)
  let frame_b_p = rotate_vector(body_b.transform.q, joint_sim.local_frame_b.p)
  
  let frame_a = { p: frame_a_p, q: frame_a_q }
  let frame_b = { p: frame_b_p, q: frame_b_q }
  
  let delta_center = sub(body_b.center, body_a.center)
  
  // Compute axis in world space (along frame A's x-axis)
  let axis_a = rotate_vector(frame_a.q, { x: 1.0, y: 0.0 })
  let d = add(delta_center, sub(frame_b.p, frame_a.p))
  
  // Compute effective mass for axial constraint
  let a1 = cross(add(d, frame_a.p), axis_a)
  let a2 = cross(frame_b.p, axis_a)
  let axial_mass_inv = m_a + m_b + i_a * a1 * a1 + i_b * a2 * a2
  let axial_mass = if axial_mass_inv > 0.0 { 1.0 / axial_mass_inv } else { 0.0 }
  
  // Compute spring softness
  let spring_softness = make_soft(prismatic_joint.hertz, prismatic_joint.damping_ratio, time_step)
  
  let updated_prismatic_joint = {
    ..prismatic_joint,
    index_a: body_id_a,
    index_b: body_id_b,
    frame_a: frame_a,
    frame_b: frame_b,
    delta_center: delta_center,
    axial_mass: axial_mass,
    spring_softness: spring_softness
  }
  
  let updated_joint_sim = {
    ..joint_sim,
    inv_mass_a: m_a,
    inv_mass_b: m_b,
    inv_ia: i_a,
    inv_ib: i_b,
    prismatic_joint: updated_prismatic_joint
  }
  
  updated_joint_sim
}

// Warm start prismatic joint (ported from b2WarmStartPrismaticJoint)
pub fn warm_start_prismatic_joint(
  joint_sim : JointSim,
  bodies : Array[BodySim]
) -> Array[BodySim] {
  if joint_sim.joint_type != PrismaticJoint {
    return bodies
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return bodies
  }
  
  let prismatic_joint = joint_sim.prismatic_joint
  let result = bodies.copy()
  
  let body_a = result[body_id_a]
  let body_b = result[body_id_b]
  
  let m_a = joint_sim.inv_mass_a
  let m_b = joint_sim.inv_mass_b
  let i_a = joint_sim.inv_ia
  let i_b = joint_sim.inv_ib
  
  let r_a = prismatic_joint.frame_a.p
  let r_b = prismatic_joint.frame_b.p
  
  // Compute axis and perpendicular direction
  let axis_a = rotate_vector(prismatic_joint.frame_a.q, { x: 1.0, y: 0.0 })
  let perp_a = left_perp(axis_a)
  
  let d = add(prismatic_joint.delta_center, sub(r_b, r_a))
  let a1 = cross(add(d, r_a), axis_a)
  let a2 = cross(r_b, axis_a)
  let s1 = cross(add(d, r_a), perp_a)
  let s2 = cross(r_b, perp_a)
  
  // Combined axial impulse
  let axial_impulse = prismatic_joint.spring_impulse + prismatic_joint.motor_impulse + 
                     prismatic_joint.lower_impulse - prismatic_joint.upper_impulse
  let perp_impulse = prismatic_joint.impulse.x
  let angle_impulse = prismatic_joint.impulse.y
  
  // Total force and torque
  let p = add(mul_sv(axial_impulse, axis_a), mul_sv(perp_impulse, perp_a))
  let l_a = axial_impulse * a1 + perp_impulse * s1 + angle_impulse
  let l_b = axial_impulse * a2 + perp_impulse * s2 + angle_impulse
  
  // Apply warm starting impulses
  let updated_body_a = {
    ..body_a,
    linear_velocity: sub(body_a.linear_velocity, mul_sv(m_a, p)),
    angular_velocity: body_a.angular_velocity - i_a * l_a
  }
  
  let updated_body_b = {
    ..body_b,
    linear_velocity: add(body_b.linear_velocity, mul_sv(m_b, p)),
    angular_velocity: body_b.angular_velocity + i_b * l_b
  }
  
  result[body_id_a] = updated_body_a
  result[body_id_b] = updated_body_b
  
  result
}

// Solve prismatic joint velocity constraints (ported from b2SolvePrismaticJoint)
pub fn solve_prismatic_joint_velocity(
  joint_sim : JointSim,
  bodies : Array[BodySim],
  time_step : Double,
  use_bias : Bool
) -> (JointSim, Array[BodySim]) {
  if joint_sim.joint_type != PrismaticJoint {
    return (joint_sim, bodies)
  }
  
  let body_id_a = joint_sim.body_id_a
  let body_id_b = joint_sim.body_id_b
  
  if body_id_a < 0 || body_id_a >= bodies.length() || 
     body_id_b < 0 || body_id_b >= bodies.length() {
    return (joint_sim, bodies)
  }
  
  let prismatic_joint = joint_sim.prismatic_joint
  let result = bodies.copy()
  
  let body_a = result[body_id_a]
  let body_b = result[body_id_b]
  
  let m_a = joint_sim.inv_mass_a
  let m_b = joint_sim.inv_mass_b
  let i_a = joint_sim.inv_ia
  let i_b = joint_sim.inv_ib
  
  // Get current velocities
  let mut v_a = body_a.linear_velocity
  let mut w_a = body_a.angular_velocity
  let mut v_b = body_b.linear_velocity
  let mut w_b = body_b.angular_velocity
  
  let r_a = prismatic_joint.frame_a.p
  let r_b = prismatic_joint.frame_b.p
  
  // Compute current axis and perpendicular direction
  let axis_a = rotate_vector(prismatic_joint.frame_a.q, { x: 1.0, y: 0.0 })
  let d = add(prismatic_joint.delta_center, sub(r_b, r_a))
  let translation = dot(axis_a, d)
  
  // These scalars are for torques generated by axial forces
  let a1 = cross(add(d, r_a), axis_a)
  let a2 = cross(r_b, axis_a)
  
  // Updated prismatic joint data
  let mut updated_prismatic_joint = prismatic_joint
  
  // Solve spring constraint (translation spring)
  if prismatic_joint.enable_spring {
    let constraint_error = translation - prismatic_joint.target_translation
    let bias = prismatic_joint.spring_softness.bias_rate * constraint_error
    let mass_scale = prismatic_joint.spring_softness.mass_scale
    let impulse_scale = prismatic_joint.spring_softness.impulse_scale
    
    let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a
    let delta_impulse = -mass_scale * prismatic_joint.axial_mass * (c_dot + bias) - 
                       impulse_scale * prismatic_joint.spring_impulse
    updated_prismatic_joint = { ..updated_prismatic_joint, spring_impulse: updated_prismatic_joint.spring_impulse + delta_impulse }
    
    let p = mul_sv(delta_impulse, axis_a)
    let l_a = delta_impulse * a1
    let l_b = delta_impulse * a2
    
    v_a = sub(v_a, mul_sv(m_a, p))
    w_a = w_a - i_a * l_a
    v_b = add(v_b, mul_sv(m_b, p))
    w_b = w_b + i_b * l_b
  }
  
  // Solve motor constraint
  if prismatic_joint.enable_motor {
    let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a
    let impulse = prismatic_joint.axial_mass * (prismatic_joint.motor_speed - c_dot)
    let old_impulse = updated_prismatic_joint.motor_impulse
    let max_impulse = time_step * prismatic_joint.max_motor_force
    let new_motor_impulse = clamp_float(updated_prismatic_joint.motor_impulse + impulse, -max_impulse, max_impulse)
    let final_impulse = new_motor_impulse - old_impulse
    updated_prismatic_joint = { ..updated_prismatic_joint, motor_impulse: new_motor_impulse }
    
    let p = mul_sv(final_impulse, axis_a)
    let l_a = final_impulse * a1
    let l_b = final_impulse * a2
    
    v_a = sub(v_a, mul_sv(m_a, p))
    w_a = w_a - i_a * l_a
    v_b = add(v_b, mul_sv(m_b, p))
    w_b = w_b + i_b * l_b
  }
  
  // Solve limit constraints
  if prismatic_joint.enable_limit {
    // Lower limit
    let constraint_error_lower = translation - prismatic_joint.lower_translation
    let bias_lower = if constraint_error_lower > 0.0 {
      // speculation - prevent constraint violation
      constraint_error_lower / time_step
    } else if use_bias {
      joint_sim.constraint_softness.bias_rate * constraint_error_lower
    } else {
      0.0
    }
    
    let mass_scale_lower = if constraint_error_lower > 0.0 {
      1.0
    } else if use_bias {
      joint_sim.constraint_softness.mass_scale
    } else {
      1.0
    }
    
    let impulse_scale_lower = if constraint_error_lower > 0.0 {
      0.0
    } else if use_bias {
      joint_sim.constraint_softness.impulse_scale
    } else {
      0.0
    }
    
    let old_impulse_lower = updated_prismatic_joint.lower_impulse
    let c_dot_lower = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a
    let impulse_lower = -prismatic_joint.axial_mass * mass_scale_lower * (c_dot_lower + bias_lower) - impulse_scale_lower * old_impulse_lower
    let new_lower_impulse = max_float(old_impulse_lower + impulse_lower, 0.0)
    let final_impulse_lower = new_lower_impulse - old_impulse_lower
    updated_prismatic_joint = { ..updated_prismatic_joint, lower_impulse: new_lower_impulse }
    
    let p_lower = mul_sv(final_impulse_lower, axis_a)
    let l_a_lower = final_impulse_lower * a1
    let l_b_lower = final_impulse_lower * a2
    
    v_a = sub(v_a, mul_sv(m_a, p_lower))
    w_a = w_a - i_a * l_a_lower
    v_b = add(v_b, mul_sv(m_b, p_lower))
    w_b = w_b + i_b * l_b_lower
    
    // Upper limit (signs flipped to keep constraint positive when satisfied)
    let constraint_error_upper = prismatic_joint.upper_translation - translation
    let bias_upper = if constraint_error_upper > 0.0 {
      // speculation
      constraint_error_upper / time_step
    } else if use_bias {
      joint_sim.constraint_softness.bias_rate * constraint_error_upper
    } else {
      0.0
    }
    
    let mass_scale_upper = if constraint_error_upper > 0.0 {
      1.0
    } else if use_bias {
      joint_sim.constraint_softness.mass_scale
    } else {
      1.0
    }
    
    let impulse_scale_upper = if constraint_error_upper > 0.0 {
      0.0
    } else if use_bias {
      joint_sim.constraint_softness.impulse_scale
    } else {
      0.0
    }
    
    let old_impulse_upper = updated_prismatic_joint.upper_impulse
    // Sign flipped on velocity constraint
    let c_dot_upper = dot(axis_a, sub(v_a, v_b)) + a1 * w_a - a2 * w_b
    let impulse_upper = -prismatic_joint.axial_mass * mass_scale_upper * (c_dot_upper + bias_upper) - impulse_scale_upper * old_impulse_upper
    let new_upper_impulse = max_float(old_impulse_upper + impulse_upper, 0.0)
    let final_impulse_upper = new_upper_impulse - old_impulse_upper
    updated_prismatic_joint = { ..updated_prismatic_joint, upper_impulse: new_upper_impulse }
    
    let p_upper = mul_sv(final_impulse_upper, axis_a)
    let l_a_upper = final_impulse_upper * a1
    let l_b_upper = final_impulse_upper * a2
    
    // Sign flipped on applied impulse
    v_a = add(v_a, mul_sv(m_a, p_upper))
    w_a = w_a + i_a * l_a_upper
    v_b = sub(v_b, mul_sv(m_b, p_upper))
    w_b = w_b - i_b * l_b_upper
  }
  
  // Solve the prismatic constraint in block form (perpendicular and angular constraints)
  let perp_a = left_perp(axis_a)
  
  // These scalars are for torques generated by the perpendicular constraint force
  let s1 = cross(add(d, r_a), perp_a)
  let s2 = cross(r_b, perp_a)
  
  let c_dot_perp = dot(perp_a, sub(v_b, v_a)) + s2 * w_b - s1 * w_a
  let c_dot_angle = w_b - w_a
  
  let bias_perp = if use_bias {
    joint_sim.constraint_softness.bias_rate * dot(perp_a, d)
  } else {
    0.0
  }
  
  let bias_angle = if use_bias {
    // Get relative rotation between frames
    let q_a = prismatic_joint.frame_a.q
    let q_b = prismatic_joint.frame_b.q
    let rel_q = inv_mul_rot(q_a, q_b)
    joint_sim.constraint_softness.bias_rate * rot_get_angle(rel_q)
  } else {
    0.0
  }
  
  let mass_scale = if use_bias { joint_sim.constraint_softness.mass_scale } else { 1.0 }
  let impulse_scale = if use_bias { joint_sim.constraint_softness.impulse_scale } else { 0.0 }
  
  // Mass matrix for 2D prismatic constraint
  let k11 = m_a + m_b + i_a * s1 * s1 + i_b * s2 * s2
  let k12 = i_a * s1 + i_b * s2
  let k22 = if i_a + i_b == 0.0 { 1.0 } else { i_a + i_b } // Avoid division by zero for fixed rotation
  
  let det = k11 * k22 - k12 * k12
  let impulse_perp = if det > 0.0 {
    -mass_scale * (k22 * (c_dot_perp + bias_perp) - k12 * (c_dot_angle + bias_angle)) / det - 
    impulse_scale * updated_prismatic_joint.impulse.x
  } else {
    0.0
  }
  
  let impulse_angle = if det > 0.0 {
    -mass_scale * (-k12 * (c_dot_perp + bias_perp) + k11 * (c_dot_angle + bias_angle)) / det - 
    impulse_scale * updated_prismatic_joint.impulse.y
  } else {
    0.0
  }
  
  updated_prismatic_joint = {
    ..updated_prismatic_joint,
    impulse: {
      x: updated_prismatic_joint.impulse.x + impulse_perp,
      y: updated_prismatic_joint.impulse.y + impulse_angle
    }
  }
  
  let p_constraint = mul_sv(impulse_perp, perp_a)
  let l_a_constraint = impulse_perp * s1 + impulse_angle
  let l_b_constraint = impulse_perp * s2 + impulse_angle
  
  v_a = sub(v_a, mul_sv(m_a, p_constraint))
  w_a = w_a - i_a * l_a_constraint
  v_b = add(v_b, mul_sv(m_b, p_constraint))
  w_b = w_b + i_b * l_b_constraint
  
  // Update body velocities
  result[body_id_a] = { ..body_a, linear_velocity: v_a, angular_velocity: w_a }
  result[body_id_b] = { ..body_b, linear_velocity: v_b, angular_velocity: w_b }
  
  let updated_joint_sim = { ..joint_sim, prismatic_joint: updated_prismatic_joint }
  (updated_joint_sim, result)
}