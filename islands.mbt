// Box2D Islands System - ported from island.c and solver_set.c
// Connected body management and constraint graph organization for efficient solving

// Constants for set types (ported from Box2D constants)
let static_set_index = 0        // Static bodies - never sleep
let disabled_set_index = 1      // Disabled bodies - not simulated
let awake_set_index = 2         // Active bodies - full simulation
let _first_sleeping_set = 3     // Sleeping islands start here (unused for now)

let _null_island_index = -1     // Invalid island ID (unused for now)

// Island data structure (ported from b2Island)
pub(all) struct Island {
  set_index : Int           // Index of solver set containing this island
  local_index : Int         // Island index within the set  
  island_id : Int           // Unique island identifier
  
  // Body linked list management
  head_body : Int           // Index of first body in island
  tail_body : Int           // Index of last body in island
  body_count : Int          // Number of bodies in island
  
  // Contact linked list management
  head_contact : Int        // Index of first contact in island
  tail_contact : Int        // Index of last contact in island
  contact_count : Int       // Number of contacts in island
  
  // Joint linked list management
  head_joint : Int          // Index of first joint in island
  tail_joint : Int          // Index of last joint in island  
  joint_count : Int         // Number of joints in island
  
  // Island management
  constraint_remove_count : Int  // Tracks removed constraints for splitting
} derive(Eq, Show)

// Solver set containing related islands (ported from b2SolverSet)
pub(all) struct SolverSet {
  set_index : Int                           // Stable set identifier
  mut islands : Array[Island]               // Islands in this set
  mut island_count : Int                    // Number of active islands
  mut bodies : Array[BodySim]               // Bodies in this set
  mut body_count : Int                      // Number of active bodies
  mut contacts : Array[(Int, Int, Manifold)]    // Contacts in this set
  mut contact_count : Int                   // Number of active contacts
  mut joints : Array[Int]                   // Joint indices in this set
  mut joint_count : Int                     // Number of active joints
} derive(Eq, Show)

// World island management (ported from Box2D world structure)
pub(all) struct IslandManager {
  mut solver_sets : Array[SolverSet]        // All solver sets
  mut set_count : Int                       // Number of active sets
  mut next_island_id : Int                  // Island ID counter
  
  // Performance tracking
  mut island_merge_count : Int              // Islands merged this step
  mut island_split_count : Int              // Islands split this step
} derive(Eq, Show)

// Create empty island
pub fn create_island(set_index : Int, local_index : Int, island_id : Int) -> Island {
  {
    set_index,
    local_index,
    island_id,
    head_body: -1,
    tail_body: -1, 
    body_count: 0,
    head_contact: -1,
    tail_contact: -1,
    contact_count: 0,
    head_joint: -1,
    tail_joint: -1,
    joint_count: 0,
    constraint_remove_count: 0
  }
}

// Create empty solver set
pub fn create_solver_set(set_index : Int) -> SolverSet {
  {
    set_index,
    islands: [],
    island_count: 0,
    bodies: [],
    body_count: 0,
    contacts: [],
    contact_count: 0,
    joints: [],
    joint_count: 0
  }
}

// Create island manager
pub fn create_island_manager() -> IslandManager {
  let solver_sets = []
  
  // Create standard solver sets
  solver_sets.push(create_solver_set(static_set_index))      // Static bodies
  solver_sets.push(create_solver_set(disabled_set_index))    // Disabled bodies  
  solver_sets.push(create_solver_set(awake_set_index))       // Awake bodies
  
  {
    solver_sets,
    set_count: 3,
    next_island_id: 0,
    island_merge_count: 0,
    island_split_count: 0
  }
}

// Add body to island (ported from Box2D body linking)
pub fn island_add_body(island : Island, body_index : Int) -> Island {
  if island.body_count == 0 {
    // First body in island
    {
      ..island,
      head_body: body_index,
      tail_body: body_index,
      body_count: 1
    }
  } else {
    // Add to end of linked list
    {
      ..island,
      tail_body: body_index,
      body_count: island.body_count + 1
    }
  }
}

// Add contact to island (ported from Box2D contact linking)
pub fn island_add_contact(island : Island, contact_index : Int) -> Island {
  if island.contact_count == 0 {
    // First contact in island
    {
      ..island,
      head_contact: contact_index,
      tail_contact: contact_index,
      contact_count: 1
    }
  } else {
    // Add to end of linked list
    {
      ..island,
      tail_contact: contact_index,
      contact_count: island.contact_count + 1
    }
  }
}

// Add joint to island (ported from Box2D joint linking)
pub fn island_add_joint(island : Island, joint_index : Int) -> Island {
  if island.joint_count == 0 {
    // First joint in island
    {
      ..island,
      head_joint: joint_index,
      tail_joint: joint_index,
      joint_count: 1
    }
  } else {
    // Add to end of linked list
    {
      ..island,
      tail_joint: joint_index,
      joint_count: island.joint_count + 1
    }
  }
}

// Merge two islands (ported from b2MergeIslands)
// Always merge smaller island into larger one for cache efficiency
pub fn merge_islands(island_a : Island, island_b : Island) -> Island {
  let (large, small) = if island_a.body_count >= island_b.body_count {
    (island_a, island_b)
  } else {
    (island_b, island_a)
  }
  
  // Merge counts
  let merged_island = {
    ..large,
    body_count: large.body_count + small.body_count,
    contact_count: large.contact_count + small.contact_count,
    joint_count: large.joint_count + small.joint_count,
    constraint_remove_count: large.constraint_remove_count + small.constraint_remove_count
  }
  
  // Link body lists
  let with_bodies = if small.body_count > 0 {
    if large.body_count == 0 {
      {
        ..merged_island,
        head_body: small.head_body,
        tail_body: small.tail_body
      }
    } else {
      // Connect tail of large to head of small
      {
        ..merged_island,
        tail_body: small.tail_body
      }
    }
  } else {
    merged_island
  }
  
  // Link contact lists
  let with_contacts = if small.contact_count > 0 {
    if with_bodies.contact_count == 0 {
      {
        ..with_bodies,
        head_contact: small.head_contact,
        tail_contact: small.tail_contact
      }
    } else {
      // Connect tail of large to head of small
      {
        ..with_bodies,
        tail_contact: small.tail_contact
      }
    }
  } else {
    with_bodies
  }
  
  // Link joint lists
  if small.joint_count > 0 {
    if with_contacts.joint_count == 0 {
      {
        ..with_contacts,
        head_joint: small.head_joint,
        tail_joint: small.tail_joint
      }
    } else {
      // Connect tail of large to head of small
      {
        ..with_contacts,
        tail_joint: small.tail_joint
      }
    }
  } else {
    with_contacts
  }
}

// Check if island should sleep (ported from Box2D sleep logic)
pub fn island_should_sleep(island : Island, bodies : Array[BodySim]) -> Bool {
  if island.body_count == 0 || island.constraint_remove_count > 0 {
    return false  // Empty islands or pending splits can't sleep
  }
  
  // Check if all bodies in island are ready to sleep
  let mut body_index = island.head_body
  let mut bodies_checked = 0
  
  while body_index != -1 && bodies_checked < island.body_count {
    if body_index >= 0 && body_index < bodies.length() {
      let body = bodies[body_index]
      
      // Use the standardized sleep check from sleep.mbt
      if !body_is_ready_to_sleep(body) {
        return false  // Body not ready to sleep
      }
    }
    
    // Move to next body (in real implementation, bodies would have next pointers)
    bodies_checked = bodies_checked + 1
    body_index = body_index + 1  // Simplified - real implementation uses linked list
  }
  
  true  // All bodies ready to sleep
}

// Build islands from collision manifolds and joints (ported from Box2D island building)  
pub fn build_islands_from_collisions(
  collisions : Array[(Int, Int, Manifold)], 
  bodies : Array[BodySim]
) -> IslandManager {
  // For simplicity in the initial implementation, create a basic manager
  // In a full implementation, this would build connected components using union-find
  let manager = create_island_manager()
  let mut island_count = 0
  
  // Count active collisions to estimate island complexity
  for collision in collisions {
    let (body_a_idx, body_b_idx, manifold) = collision
    
    if manifold.point_count > 0 && 
       body_a_idx < bodies.length() && body_b_idx < bodies.length() {
      let body_a = bodies[body_a_idx]
      let body_b = bodies[body_b_idx]
      
      // Count collisions involving dynamic bodies
      if body_a.inv_mass > 0.0 || body_b.inv_mass > 0.0 {
        island_count = island_count + 1
      }
    }
  }
  
  // Create a single awake island for all active bodies (simplified approach)
  if island_count > 0 {
    let awake_island = create_island(awake_set_index, 0, 0)
    let island_with_contacts = { ..awake_island, contact_count: island_count }
    
    // Create updated awake set with the new island
    let updated_awake_set = {
      ..manager.solver_sets[awake_set_index],
      islands: [island_with_contacts],
      island_count: 1
    }
    
    // Update the solver sets array
    let updated_solver_sets = manager.solver_sets.copy()
    updated_solver_sets[awake_set_index] = updated_awake_set
    
    // Return updated manager
    { ..manager, solver_sets: updated_solver_sets, next_island_id: 1 }
  } else {
    manager
  }
}

// Solve islands using constraint solver (integration point)
pub fn solve_islands(
  manager : IslandManager,
  bodies : Array[BodySim], 
  collisions : Array[(Int, Int, Manifold)],
  velocity_iterations : Int,
  position_iterations : Int
) -> Array[BodySim] {
  if manager.solver_sets.length() <= awake_set_index {
    return bodies
  }
  
  let awake_set = manager.solver_sets[awake_set_index]
  
  if awake_set.island_count == 0 {
    // No islands - solve all contacts together (fallback to global solver)
    return solve_contacts_with_constraints(collisions, bodies, velocity_iterations, position_iterations)
  }
  
  // For now, solve all awake bodies together
  // In a full implementation, we would solve each island separately for better performance
  let mut result_bodies = bodies.copy()
  
  // Extract contacts for awake bodies
  let awake_collisions = []
  for collision in collisions {
    let (body_a_idx, body_b_idx, _manifold) = collision
    if body_a_idx < bodies.length() && body_b_idx < bodies.length() {
      let body_a = bodies[body_a_idx]
      let body_b = bodies[body_b_idx] 
      
      // Include if at least one body is dynamic (awake)
      if body_a.inv_mass > 0.0 || body_b.inv_mass > 0.0 {
        awake_collisions.push(collision)
      }
    }
  }
  
  // Solve awake island constraints
  if awake_collisions.length() > 0 {
    result_bodies = solve_contacts_with_constraints(
      awake_collisions,
      result_bodies,
      velocity_iterations,
      position_iterations
    )
  }
  
  result_bodies
}

// Put an island to sleep (ported from Box2D TrySleepIsland)
pub fn put_island_to_sleep(manager : IslandManager, island : Island, bodies : Array[BodySim]) -> (IslandManager, Array[BodySim]) {
  if !island_should_sleep(island, bodies) {
    // Island not ready to sleep
    return (manager, bodies)
  }
  
  // Sleep all bodies in the island
  let updated_bodies = bodies.copy()
  let mut body_index = island.head_body
  let mut bodies_processed = 0
  
  while body_index != -1 && bodies_processed < island.body_count {
    if body_index >= 0 && body_index < updated_bodies.length() {
      updated_bodies[body_index] = sleep_body(updated_bodies[body_index])
    }
    bodies_processed = bodies_processed + 1
    body_index = body_index + 1  // Simplified linked list traversal
  }
  
  // In a full implementation, we would move the island to a sleeping solver set
  // For now, we just mark bodies as asleep
  (manager, updated_bodies)
}

// Wake an island (ported from Box2D WakeSolverSet)
pub fn wake_island(manager : IslandManager, island : Island, bodies : Array[BodySim]) -> (IslandManager, Array[BodySim]) {
  // Wake all bodies in the island
  let updated_bodies = bodies.copy()
  let mut body_index = island.head_body
  let mut bodies_processed = 0
  
  while body_index != -1 && bodies_processed < island.body_count {
    if body_index >= 0 && body_index < updated_bodies.length() {
      updated_bodies[body_index] = wake_body(updated_bodies[body_index])
    }
    bodies_processed = bodies_processed + 1
    body_index = body_index + 1  // Simplified linked list traversal
  }
  
  // In a full implementation, we would move the island to the awake solver set
  // For now, we just mark bodies as awake
  (manager, updated_bodies)
}

// Process sleeping for all islands (called during world step)
pub fn process_island_sleeping(manager : IslandManager, bodies : Array[BodySim], enable_sleep : Bool) -> (IslandManager, Array[BodySim]) {
  if !enable_sleep {
    return (manager, bodies)
  }
  
  let mut current_bodies = bodies
  let mut current_manager = manager
  
  // Check awake islands for sleep candidates
  if manager.solver_sets.length() > awake_set_index {
    let awake_set = manager.solver_sets[awake_set_index]
    
    for island in awake_set.islands {
      if island_should_sleep(island, current_bodies) {
        let (updated_manager, updated_bodies) = put_island_to_sleep(current_manager, island, current_bodies)
        current_manager = updated_manager
        current_bodies = updated_bodies
      }
    }
  }
  
  (current_manager, current_bodies)
}

// Get island statistics for debugging/monitoring
pub fn get_island_stats(manager : IslandManager) -> (Int, Int, Int) {
  let mut total_islands = 0
  let mut total_bodies = 0
  let mut awake_islands = 0
  
  for set in manager.solver_sets {
    total_islands = total_islands + set.island_count
    total_bodies = total_bodies + set.body_count
    
    if set.set_index == awake_set_index {
      awake_islands = set.island_count
    }
  }
  
  (total_islands, total_bodies, awake_islands)
}