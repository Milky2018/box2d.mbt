// Box2D Contact Event System - for game logic integration
// Tracks contact lifecycle and provides callbacks for begin/end/pre-solve/post-solve events

// Contact event callback function type
pub struct ContactEventCallback((ContactEvent) -> Unit)

// Create a contact event callback from a function
pub fn create_contact_callback(func : (ContactEvent) -> Unit) -> ContactEventCallback {
  ContactEventCallback(func)
}

// Contact event manager for tracking contact lifecycle and callbacks
pub struct ContactEventManager {
  // Contact tracking cache
  contact_cache : Array[ContactCacheEntry]
  
  // Enable/disable specific event types
  enable_begin_end_events : Bool
  enable_solve_events : Bool
} derive(Eq, Show)

// Global storage for callbacks (since functions can't be stored in derive structs)
let begin_contact_callbacks : Ref[Array[ContactEventCallback]] = { val: [] }
let end_contact_callbacks : Ref[Array[ContactEventCallback]] = { val: [] }
let pre_solve_callbacks : Ref[Array[ContactEventCallback]] = { val: [] }
let post_solve_callbacks : Ref[Array[ContactEventCallback]] = { val: [] }

// Create default contact event manager
pub fn create_contact_event_manager() -> ContactEventManager {
  {
    contact_cache: [],
    enable_begin_end_events: false,
    enable_solve_events: false
  }
}

// Register begin contact callback
pub fn set_begin_contact_callback(
  manager : ContactEventManager, 
  callback : ContactEventCallback
) -> ContactEventManager {
  begin_contact_callbacks.val.push(callback)
  { 
    ..manager, 
    enable_begin_end_events: true
  }
}

// Register end contact callback  
pub fn set_end_contact_callback(
  manager : ContactEventManager,
  callback : ContactEventCallback
) -> ContactEventManager {
  end_contact_callbacks.val.push(callback)
  { 
    ..manager, 
    enable_begin_end_events: true
  }
}

// Register pre-solve callback
pub fn set_pre_solve_callback(
  manager : ContactEventManager,
  callback : ContactEventCallback
) -> ContactEventManager {
  pre_solve_callbacks.val.push(callback)
  { 
    ..manager, 
    enable_solve_events: true
  }
}

// Register post-solve callback
pub fn set_post_solve_callback(
  manager : ContactEventManager,
  callback : ContactEventCallback
) -> ContactEventManager {
  post_solve_callbacks.val.push(callback)
  { 
    ..manager, 
    enable_solve_events: true
  }
}

// Create contact ID from collision data
fn make_contact_id(body_a_idx : Int, body_b_idx : Int, shape_a_id : Int, shape_b_id : Int) -> ContactId {
  // Ensure consistent ordering (smaller body index first)
  if body_a_idx <= body_b_idx {
    { shape_a: shape_a_id, shape_b: shape_b_id, body_a: body_a_idx, body_b: body_b_idx }
  } else {
    { shape_a: shape_b_id, shape_b: shape_a_id, body_a: body_b_idx, body_b: body_a_idx }
  }
}

// Find contact cache entry by ID
fn find_contact_cache_entry(cache : Array[ContactCacheEntry], contact_id : ContactId) -> Int {
  for i, entry in cache {
    if entry.contact_id == contact_id {
      return i
    }
  }
  -1 // Not found
}

// Process contacts and generate begin/end events
pub fn process_contact_events(
  manager : ContactEventManager,
  collisions : Array[(Int, Int, Manifold)]
) -> (ContactEventManager, Array[ContactEvent]) {
  if not(manager.enable_begin_end_events) {
    return (manager, [])
  }
  
  let events = []
  let updated_cache = manager.contact_cache.copy()
  
  // Mark all existing contacts as not touching this frame
  for i, entry in updated_cache {
    updated_cache[i] = { ..entry, was_touching: entry.is_touching, is_touching: false }
  }
  
  // Process current frame collisions
  for collision in collisions {
    let (body_a_idx, body_b_idx, manifold) = collision
    
    // For now, assume default shape IDs (this would need proper shape tracking)
    let contact_id = make_contact_id(body_a_idx, body_b_idx, 0, 0)
    let cache_index = find_contact_cache_entry(updated_cache, contact_id)
    
    if manifold.point_count > 0 {
      // Contact is active this frame
      if cache_index >= 0 {
        // Update existing contact
        let entry = updated_cache[cache_index]
        updated_cache[cache_index] = {
          ..entry,
          is_touching: true,
          manifold,
          // Keep accumulated impulses for now
        }
        
        // No begin event needed (already touching)
      } else {
        // New contact - add to cache and generate begin event
        let new_entry = {
          contact_id,
          was_touching: false,
          is_touching: true,
          manifold,
          normal_impulse: 0.0,
          tangent_impulse: 0.0
        }
        updated_cache.push(new_entry)
        
        // Generate begin contact event
        let begin_event = {
          event_type: BeginContact,
          shape_id_a: { id: contact_id.shape_a },
          shape_id_b: { id: contact_id.shape_b },
          body_id_a: { id: contact_id.body_a },
          body_id_b: { id: contact_id.body_b },
          manifold,
          normal_impulse: 0.0,
          tangent_impulse: 0.0
        }
        events.push(begin_event)
      }
    }
  }
  
  // Check for ended contacts and generate end events
  let final_cache = []
  for entry in updated_cache {
    if entry.was_touching && not(entry.is_touching) {
      // Contact ended - generate end event
      let end_event = {
        event_type: EndContact,
        shape_id_a: { id: entry.contact_id.shape_a },
        shape_id_b: { id: entry.contact_id.shape_b },
        body_id_a: { id: entry.contact_id.body_a },
        body_id_b: { id: entry.contact_id.body_b },
        manifold: entry.manifold,
        normal_impulse: entry.normal_impulse,
        tangent_impulse: entry.tangent_impulse
      }
      events.push(end_event)
      // Don't keep ended contacts in cache
    } else if entry.is_touching {
      // Keep active contacts
      final_cache.push(entry)
    }
  }
  
  let updated_manager = { ..manager, contact_cache: final_cache }
  (updated_manager, events)
}

// Fire contact events through registered callbacks
pub fn fire_contact_events(_manager : ContactEventManager, events : Array[ContactEvent]) -> Unit {
  for event in events {
    match event.event_type {
      BeginContact => {
        for callback in begin_contact_callbacks.val {
          let ContactEventCallback(func) = callback
          func(event)
        }
      }
      EndContact => {
        for callback in end_contact_callbacks.val {
          let ContactEventCallback(func) = callback
          func(event)
        }
      }
      PreSolve => {
        for callback in pre_solve_callbacks.val {
          let ContactEventCallback(func) = callback
          func(event)
        }
      }
      PostSolve => {
        for callback in post_solve_callbacks.val {
          let ContactEventCallback(func) = callback
          func(event)
        }
      }
    }
  }
}

// Generate pre-solve events for all active contacts
pub fn generate_pre_solve_events(
  manager : ContactEventManager,
  collisions : Array[(Int, Int, Manifold)]
) -> Array[ContactEvent] {
  if not(manager.enable_solve_events) {
    return []
  }
  
  let events = []
  for collision in collisions {
    let (body_a_idx, body_b_idx, manifold) = collision
    if manifold.point_count > 0 {
      let contact_id = make_contact_id(body_a_idx, body_b_idx, 0, 0)
      let pre_solve_event = {
        event_type: PreSolve,
        shape_id_a: { id: contact_id.shape_a },
        shape_id_b: { id: contact_id.shape_b },
        body_id_a: { id: contact_id.body_a },
        body_id_b: { id: contact_id.body_b },
        manifold,
        normal_impulse: 0.0,
        tangent_impulse: 0.0
      }
      events.push(pre_solve_event)
    }
  }
  events
}

// Generate post-solve events with impulse information
pub fn generate_post_solve_events(
  manager : ContactEventManager,
  solver : ContactSolver
) -> Array[ContactEvent] {
  if not(manager.enable_solve_events) {
    return []
  }
  
  let events = []
  for constraint in solver.constraints {
    // Calculate total impulses from constraint points
    let mut total_normal_impulse = 0.0
    let mut total_tangent_impulse = 0.0
    
    for point in constraint.points {
      total_normal_impulse = total_normal_impulse + point.normal_impulse
      total_tangent_impulse = total_tangent_impulse + point.tangent_impulse
    }
    
    let post_solve_event = {
      event_type: PostSolve,
      shape_id_a: { id: 0 }, // Would need proper shape tracking
      shape_id_b: { id: 0 }, // Would need proper shape tracking  
      body_id_a: { id: constraint.body_a_index },
      body_id_b: { id: constraint.body_b_index },
      manifold: { 
        normal: constraint.normal,
        rolling_impulse: 0.0,
        points: [], // Simplified for now
        point_count: constraint.point_count
      },
      normal_impulse: total_normal_impulse,
      tangent_impulse: total_tangent_impulse
    }
    events.push(post_solve_event)
  }
  events
}