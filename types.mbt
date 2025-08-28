// Box2D API types - ported from box2d/types.h

// Body and shape types
pub(all) enum BodyType {
  Static
  Kinematic  
  Dynamic
} derive(Eq, Show)

pub(all) enum ShapeType {
  Circle
  Capsule
  Polygon
  Segment
  ChainSegment
} derive(Eq, Show)

// ID types  
pub(all) struct WorldId {
  id : Int
} derive(Eq, Show)

pub(all) struct BodyId {
  id : Int
} derive(Eq, Show)

pub(all) struct ShapeId {
  id : Int
} derive(Eq, Show)

pub(all) struct JointId {
  id : Int
} derive(Eq, Show)

// Shape geometry
pub(all) struct Circle {
  center : Vec2
  radius : Double
} derive(Eq, Show)

// A solid capsule - two semicircles connected by a rectangle
pub(all) struct Capsule {
  center1 : Vec2  // Local center of first semicircle
  center2 : Vec2  // Local center of second semicircle  
  radius : Double // Radius of the semicircles
} derive(Eq, Show)

// Line segment with two-sided collision
pub(all) struct Segment {
  point1 : Vec2
  point2 : Vec2
} derive(Eq, Show)

// Convex polygon with proper Box2D structure
pub(all) struct Polygon {
  vertices : Array[Vec2]      // Polygon vertices
  normals : Array[Vec2]       // Outward normal vectors  
  centroid : Vec2             // Centroid of the polygon
  radius : Double             // External radius for rounded polygons
  count : Int                 // Number of vertices
} derive(Eq, Show)

// Chain segment for one-sided collision (used in chain shapes)
pub(all) struct ChainSegment {
  ghost1 : Vec2     // Tail ghost vertex
  segment : Segment // The line segment
  ghost2 : Vec2     // Head ghost vertex
  chain_id : Int    // Owning chain shape index
} derive(Eq, Show)

// Ray casting input (ported from b2RayCastInput)
pub(all) struct RayCastInput {
  origin : Vec2         // Start point of the ray cast
  translation : Vec2    // Translation of the ray cast
  max_fraction : Double // The maximum fraction of the translation to consider, typically 1
} derive(Eq, Show)

// Definition types (ported from Box2D types.h)
pub(all) struct WorldDef {
  mut gravity : Vec2
  restitution_threshold : Double
  hit_event_threshold : Double
  contact_hertz : Double
  contact_damping_ratio : Double
  maximum_linear_speed : Double
  mut enable_sleep : Bool
  mut enable_continuous : Bool
} derive(Eq, Show)

// Softness type for constraint softening (ported from b2Softness in Box2D)
pub(all) struct Softness {
  bias_rate : Double              // Bias correction rate
  mass_scale : Double             // Mass scaling for soft constraints
  impulse_scale : Double          // Impulse scaling for soft constraints
} derive(Eq, Show)

// Joint types (ported from Box2D b2JointType)
pub(all) enum JointType {
  DistanceJoint
  MotorJoint
  PrismaticJoint
  RevoluteJoint
  WeldJoint
  WheelJoint
} derive(Eq, Show)

// Base joint definition (ported from Box2D b2JointDef)
pub(all) struct JointDef {
  body_id_a : BodyId              // First attached body
  body_id_b : BodyId              // Second attached body
  collide_connected : Bool        // Set this flag to true if the attached bodies should collide
} derive(Eq, Show)

// Distance joint definition (ported from Box2D b2DistanceJointDef)
pub(all) struct DistanceJointDef {
  base : JointDef                 // Base joint definition
  mut length : Double             // The rest length of this joint
  mut enable_spring : Bool        // Enable the distance constraint to behave like a spring
  mut lower_spring_force : Double // The lower spring force controls how much tension it can sustain
  mut upper_spring_force : Double // The upper spring force controls how much compression it can sustain
  mut hertz : Double              // The spring linear stiffness Hertz, cycles per second
  mut damping_ratio : Double      // The spring damping ratio (non-dimensional)
  mut enable_limit : Bool         // Enable/disable the joint limit
  mut min_length : Double         // Minimum length. Clamped to a stable minimum value
  mut max_length : Double         // Maximum length. Must be greater than or equal to the minimum length
  mut enable_motor : Bool         // Enable/disable the joint motor
  mut max_motor_force : Double    // The maximum motor force in newtons
  mut motor_speed : Double        // The desired motor speed, usually in meters per second
} derive(Eq, Show)

// Revolute joint definition (ported from b2RevoluteJointDef)
pub(all) struct RevoluteJointDef {
  base : JointDef                     // Base joint definition
  mut target_angle : Double          // Target angle for spring-damper (radians)
  mut enable_spring : Bool           // Enable rotational spring
  mut hertz : Double                 // Spring frequency (Hz)
  mut damping_ratio : Double         // Spring damping ratio
  mut enable_limit : Bool            // Enable joint limits
  mut lower_angle : Double           // Lower angle limit (radians)
  mut upper_angle : Double           // Upper angle limit (radians)  
  mut enable_motor : Bool            // Enable joint motor
  mut max_motor_torque : Double      // Maximum motor torque (N⋅m)
  mut motor_speed : Double           // Desired motor speed (rad/s)
} derive(Eq, Show)

// Prismatic joint definition (ported from b2PrismaticJointDef)
pub(all) struct PrismaticJointDef {
  base : JointDef                     // Base joint definition
  mut target_translation : Double    // Target translation for spring-damper (meters)
  mut enable_spring : Bool           // Enable translational spring
  mut hertz : Double                 // Spring frequency (Hz)
  mut damping_ratio : Double         // Spring damping ratio
  mut enable_limit : Bool            // Enable joint limits
  mut lower_translation : Double     // Lower translation limit (meters)
  mut upper_translation : Double     // Upper translation limit (meters)
  mut enable_motor : Bool            // Enable joint motor
  mut max_motor_force : Double       // Maximum motor force (N)
  mut motor_speed : Double           // Desired motor speed (m/s)
} derive(Eq, Show)

// Motor joint definition (ported from b2MotorJointDef)
pub(all) struct MotorJointDef {
  base : JointDef                     // Base joint definition
  mut linear_offset : Vec2            // Position of bodyB minus position of bodyA (in bodyA frame)
  mut angular_offset : Double         // Angle of bodyB minus angle of bodyA (radians)
  mut max_force : Double              // Maximum motor force (N)
  mut max_torque : Double             // Maximum motor torque (N⋅m)  
  mut correction_factor : Double      // Position correction factor [0,1]
} derive(Eq, Show)

pub(all) struct BodyDef {
  mut type_ : BodyType
  mut position : Vec2
  mut rotation : Rot
  mut linear_velocity : Vec2
  mut angular_velocity : Double
  mut linear_damping : Double
  mut angular_damping : Double
  mut gravity_scale : Double
  mut sleep_threshold : Double
  mut enable_sleep : Bool
  mut is_awake : Bool
  mut is_enabled : Bool
} derive(Eq, Show)

// Collision filtering (ported from b2Filter in Box2D)
pub(all) struct CollisionFilter {
  category_bits : Int    // What category this shape belongs to (bitmask)
  mask_bits : Int        // What categories this shape collides with (bitmask)
  group_index : Int      // Group index for collision filtering (0 = no group)
} derive(Eq, Show)

pub(all) struct ShapeDef {
  mut density : Double
  mut friction : Double
  mut restitution : Double
  mut rolling_friction : Double
  mut filter : CollisionFilter
  mut is_sensor : Bool
} derive(Eq, Show)


pub(all) struct RayResult {
  shape_id : ShapeId
  point : Vec2
  normal : Vec2
  fraction : Double
  hit : Bool
} derive(Eq, Show)

// Sweep for continuous collision (ported from b2Sweep)
pub(all) struct Sweep {
  local_center : Vec2  // Local center of mass position
  c1 : Vec2           // Center world position at time 0
  c2 : Vec2           // Center world position at time 1
  q1 : Rot            // World rotation at time 0
  q2 : Rot            // World rotation at time 1
} derive(Eq, Show)

// Shape proxy for GJK distance algorithm (ported from b2ShapeProxy)
pub(all) struct ShapeProxy {
  points : Array[Vec2]  // Support points
  count : Int           // Number of support points
  radius : Double       // Shape radius
} derive(Eq, Show)

// Simplex vertex for GJK (ported from b2SimplexVertex)
pub(all) struct SimplexVertex {
  w_a : Vec2     // Support point in proxyA
  w_b : Vec2     // Support point in proxyB
  w : Vec2       // w_a - w_b
  a : Double     // Barycentric coordinate for closest point
  index_a : Int  // w_a index
  index_b : Int  // w_b index
} derive(Eq, Show)

// Simplex for GJK (ported from b2Simplex)
pub(all) struct Simplex {
  v1 : SimplexVertex
  v2 : SimplexVertex
  v3 : SimplexVertex
  count : Int
} derive(Eq, Show)

// Simplex cache (ported from b2SimplexCache)
pub(all) struct SimplexCache {
  index_a : Array[Int]  // indices of points on shape A
  index_b : Array[Int]  // indices of points on shape B
  count : Int           // length of indices
} derive(Eq, Show)

// Distance algorithm input (ported from b2DistanceInput)
pub(all) struct DistanceInput {
  proxy_a : ShapeProxy
  proxy_b : ShapeProxy
  transform_a : Transform
  transform_b : Transform
  use_radii : Bool
} derive(Eq, Show)

// Distance algorithm output (ported from b2DistanceOutput)
pub(all) struct DistanceOutput {
  point_a : Vec2     // Closest point on shapeA
  point_b : Vec2     // Closest point on shapeB
  distance : Double  // Distance between closest points
  normal : Vec2      // Unit normal from A to B
  iterations : Int   // Number of GJK iterations used
  simplex_count : Int // Number of simplexes processed
} derive(Eq, Show)

// Mass data computed for a shape (ported from b2MassData)
pub(all) struct MassData {
  mass : Double                 // The mass of the shape, usually in kilograms
  center : Vec2                 // The position of the shape's centroid relative to the shape's origin
  rotational_inertia : Double   // The rotational inertia of the shape about the shape center
} derive(Eq, Show)

// A manifold point is a contact point belonging to a contact manifold (ported from b2ManifoldPoint)
pub(all) struct ManifoldPoint {
  point : Vec2         // Location of the contact point in world space
  anchor_a : Vec2      // Location of the contact point relative to shapeA's origin in world space
  anchor_b : Vec2      // Location of the contact point relative to shapeB's origin in world space  
  separation : Double  // The separation of the contact point, negative if penetrating
  normal_impulse : Double      // The impulse along the manifold normal vector
  tangent_impulse : Double     // The friction impulse
  total_normal_impulse : Double // The total normal impulse applied across sub-stepping and restitution
  normal_velocity : Double     // Relative normal velocity pre-solve
  id : Int             // Uniquely identifies a contact point between two shapes
  persisted : Bool     // Did this contact point exist the previous step?
} derive(Eq, Show)

// A contact manifold describes the contact points between colliding shapes (ported from b2Manifold)
pub(all) struct Manifold {
  normal : Vec2                 // The unit normal vector in world space, points from shape A to bodyB
  rolling_impulse : Double      // Angular impulse applied for rolling resistance
  points : Array[ManifoldPoint] // The manifold points, up to two are possible in 2D
  point_count : Int             // The number of contacts points, will be 0, 1, or 2
} derive(Eq, Show)

// Contact event types (ported from Box2D contact events)
pub(all) enum ContactEventType {
  BeginContact    // Contact started this frame
  EndContact      // Contact ended this frame
  PreSolve        // Before contact constraint solving
  PostSolve       // After contact constraint solving  
} derive(Eq, Show)

// Contact event data passed to callbacks
pub(all) struct ContactEvent {
  event_type : ContactEventType    // Type of contact event
  shape_id_a : ShapeId            // First shape in contact
  shape_id_b : ShapeId            // Second shape in contact  
  body_id_a : BodyId              // First body in contact
  body_id_b : BodyId              // Second body in contact
  manifold : Manifold             // Contact manifold data
  normal_impulse : Double         // Total normal impulse (for post-solve events)
  tangent_impulse : Double        // Total tangent impulse (for post-solve events)
} derive(Eq, Show)

// Contact ID for tracking contact persistence
pub(all) struct ContactId {
  shape_a : Int     // Shape A ID
  shape_b : Int     // Shape B ID
  body_a : Int      // Body A index  
  body_b : Int      // Body B index
} derive(Eq, Show)

// Contact cache entry for tracking contact lifecycle
struct ContactCacheEntry {
  contact_id : ContactId          // Unique contact identifier
  was_touching : Bool             // Was touching in previous frame
  is_touching : Bool              // Is touching in current frame  
  manifold : Manifold             // Current contact manifold
  normal_impulse : Double         // Accumulated normal impulse
  tangent_impulse : Double        // Accumulated tangent impulse
} derive(Eq, Show)

// Debug drawing interface (ported from Box2D b2DebugDraw)
pub(all) enum DebugDrawFlags {
  DrawShapes       // Draw shapes
  DrawJoints       // Draw joint connections  
  DrawAABBs        // Draw axis aligned bounding boxes
  DrawPairs        // Draw broad-phase pairs
  DrawCenterOfMass // Draw center of mass frames
  DrawStats        // Draw performance statistics
} derive(Eq, Show)

// Color for debug drawing (RGBA)
pub(all) struct Color {
  r : Double  // Red component [0,1]
  g : Double  // Green component [0,1] 
  b : Double  // Blue component [0,1]
  a : Double  // Alpha component [0,1]
} derive(Eq, Show)

// Debug drawing command (for rendering system to process)
pub(all) enum DebugDrawCommand {
  DrawLine(Vec2, Vec2, Color)          // Draw line from p1 to p2
  DrawCircle(Vec2, Double, Color)      // Draw circle at center with radius
  DrawSolidCircle(Vec2, Double, Color) // Draw filled circle
  DrawPolygon(Array[Vec2], Color)      // Draw polygon outline
  DrawSolidPolygon(Array[Vec2], Color) // Draw filled polygon
  DrawSegment(Vec2, Vec2, Color)       // Draw line segment
  DrawTransform(Transform)             // Draw coordinate frame
} derive(Eq, Show)

// Debug drawing context
pub(all) struct DebugDrawContext {
  commands : Array[DebugDrawCommand]   // Commands to execute
  flags : Array[DebugDrawFlags]       // What to draw
} derive(Eq, Show)