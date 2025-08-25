# Box2D MoonBit Port

A MoonBit port of the Box2D physics engine for 2D rigid body simulation. This library provides real-time physics simulation with collision detection, rigid body dynamics, and constraint solving.

## Features

- **Rigid Body Dynamics**: Full support for static, kinematic, and dynamic bodies
- **Collision Detection**: Circle, capsule, polygon, and segment shapes with accurate manifold generation
- **Physics Simulation**: Gravity, forces, impulses, and realistic physics stepping
- **Shape Support**: Circles, capsules, polygons, segments, and chain segments
- **Math Library**: Comprehensive 2D vector and transform operations
- **Memory Efficient**: Optimized data structures and algorithms

## Installation

Add this to your `moon.pkg.json`:

```json
{
  "deps": {
    "Milky2018/box2d": "latest"
  }
}
```

## Quick Start

### Creating a Simple Falling Ball

```moonbit
test "falling ball physics simulation" {
  // Create a physics world with gravity
  let world_def = @box2d.default_world_def()
  // Note: WorldDef fields are immutable, so we use the default gravity of (0.0, -10.0)
  let world = @box2d.create_world(world_def)
  inspect(@box2d.world_is_valid(world), content="true")
  
  // Create a static ground body
  let ground_def = @box2d.default_body_def()
  ground_def.position = @box2d.Vec2::{ x: 0.0, y: -10.0 }
  let ground = @box2d.create_body(world, ground_def)
  inspect(@box2d.body_is_valid(ground), content="true")
  
  // Add ground shape
  let ground_shape_def = @box2d.default_shape_def()
  let ground_box = @box2d.make_box(50.0, 1.0)
  let ground_shape = @box2d.create_polygon_shape(ground, ground_shape_def, ground_box)
  inspect(@box2d.shape_is_valid(ground_shape), content="true")
  
  // Create a dynamic ball
  let ball_def = @box2d.default_body_def()
  ball_def.type_ = @box2d.BodyType::Dynamic
  ball_def.position = @box2d.Vec2::{ x: 0.0, y: 10.0 }
  let ball = @box2d.create_body(world, ball_def)
  
  // Add ball shape with physics properties
  let ball_shape_def = @box2d.default_shape_def()
  ball_shape_def.density = 1.0
  ball_shape_def.friction = 0.3
  ball_shape_def.restitution = 0.6 // Bounce factor
  let circle = @box2d.Circle::{ center: @box2d.Vec2::{ x: 0.0, y: 0.0 }, radius: 0.5 }
  let ball_shape = @box2d.create_circle_shape(ball, ball_shape_def, circle)
  inspect(@box2d.shape_is_valid(ball_shape), content="true")
  
  // Get initial position
  let initial_position = @box2d.body_get_position(ball)
  inspect(initial_position.y, content="10")
  
  // Simulate for a few steps
  for _i = 0; _i < 10; _i = _i + 1 {
    @box2d.world_step(world, 1.0/60.0, 8)
  }
  
  // Ball should have fallen due to gravity
  let final_position = @box2d.body_get_position(ball)
  inspect(final_position.y < initial_position.y, content="true")
}
```

### Working with Different Shapes

```moonbit
test "shape creation and validation" {
  let world_def = @box2d.default_world_def()
  let world = @box2d.create_world(world_def)
  
  let body_def = @box2d.default_body_def()
  body_def.type_ = @box2d.BodyType::Dynamic
  let body = @box2d.create_body(world, body_def)
  
  let shape_def = @box2d.default_shape_def()
  shape_def.density = 1.0
  
  // Circle shape
  let circle = @box2d.Circle::{ center: @box2d.Vec2::{ x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_shape = @box2d.create_circle_shape(body, shape_def, circle)
  inspect(@box2d.shape_is_valid(circle_shape), content="true")
  
  // Capsule shape (rounded rectangle)
  let capsule = @box2d.make_capsule_horizontal(2.0, 0.5) // length=2.0, radius=0.5
  let capsule_shape = @box2d.create_capsule_shape(body, shape_def, capsule)
  inspect(@box2d.shape_is_valid(capsule_shape), content="true")
  inspect(capsule.center1.x, content="-1")
  inspect(capsule.center2.x, content="1")
  inspect(capsule.radius, content="0.5")
  
  // Box polygon
  let box = @box2d.make_box(1.0, 2.0) // width=1.0, height=2.0
  let box_shape = @box2d.create_polygon_shape(body, shape_def, box)
  inspect(@box2d.shape_is_valid(box_shape), content="true")
  inspect(box.count, content="4")
  
  // Line segment
  let segment = @box2d.make_segment(
    @box2d.Vec2::{ x: -1.0, y: 0.0 }, 
    @box2d.Vec2::{ x: 1.0, y: 0.0 }
  )
  let segment_shape = @box2d.create_segment_shape(body, shape_def, segment)
  inspect(@box2d.shape_is_valid(segment_shape), content="true")
  inspect(segment.point1.x, content="-1")
  inspect(segment.point2.x, content="1")
}
```

### Collision Detection

```moonbit
test "circle collision detection" {
  // Test overlapping circles
  let circle_a = @box2d.Circle::{ center: @box2d.Vec2::{ x: 0.0, y: 0.0 }, radius: 1.0 }
  let circle_b = @box2d.Circle::{ center: @box2d.Vec2::{ x: 1.5, y: 0.0 }, radius: 1.0 }
  
  let transform_a = @box2d.Transform::{ 
    p: @box2d.Vec2::{ x: 0.0, y: 0.0 }, 
    q: @box2d.rot_identity 
  }
  let transform_b = @box2d.Transform::{ 
    p: @box2d.Vec2::{ x: 0.0, y: 0.0 }, 
    q: @box2d.rot_identity 
  }
  
  let manifold = @box2d.collide_circles(circle_a, transform_a, circle_b, transform_b)
  
  // Circles should collide (distance = 1.5, combined radius = 2.0)
  inspect(manifold.point_count > 0, content="true")
  inspect(manifold.points.length() > 0, content="true")
  
  // Test separated circles
  let circle_c = @box2d.Circle::{ center: @box2d.Vec2::{ x: 5.0, y: 0.0 }, radius: 1.0 }
  let manifold2 = @box2d.collide_circles(circle_a, transform_a, circle_c, transform_b)
  
  // Circles should not collide (distance = 5.0, combined radius = 2.0)
  inspect(manifold2.point_count == 0, content="true")
}
```

### Vector Math Operations

```moonbit
test "vector math operations" {
  let v1 = @box2d.Vec2::{ x: 3.0, y: 4.0 }
  let v2 = @box2d.Vec2::{ x: 1.0, y: 2.0 }
  
  // Basic operations
  let sum = @box2d.vec2_add(v1, v2)        // Addition
  inspect(sum.x, content="4")
  inspect(sum.y, content="6")
  
  let diff = @box2d.sub(v1, v2)            // Subtraction  
  inspect(diff.x, content="2")
  inspect(diff.y, content="2")
  
  let scaled = @box2d.mul_sv(2.0, v1)      // Scalar multiplication
  inspect(scaled.x, content="6")
  inspect(scaled.y, content="8")
  
  // Vector properties
  let length = @box2d.vec2_length(v1)      // Length: 5.0
  inspect(length, content="5")
  
  let length_sq = @box2d.length_squared(v1) // Squared length: 25.0
  inspect(length_sq, content="25")
  
  let normalized = @box2d.normalize(v1)     // Unit vector
  let normalized_length = @box2d.vec2_length(normalized)
  inspect(normalized_length > 0.99 && normalized_length < 1.01, content="true")
  
  // Vector operations
  let dot_product = @box2d.vec2_dot(v1, v2) // Dot product: 11.0
  inspect(dot_product, content="11")
  
  let cross_product = @box2d.cross(v1, v2)  // Cross product: 2.0
  inspect(cross_product, content="2")
  
  // Distance between points
  let distance = @box2d.distance(v1, v2)    // Distance: sqrt(8) ≈ 2.83
  inspect(distance > 2.8 && distance < 2.9, content="true")
}
```

### Transform Operations

```moonbit
test "transform operations" {
  // Create a transform (position + rotation)
  let position = @box2d.Vec2::{ x: 5.0, y: 3.0 }
  let rotation = @box2d.make_rot(@box2d.pi / 4.0) // 45 degrees
  let transform = @box2d.Transform::{ p: position, q: rotation }
  
  // Verify rotation is correct (45 degrees)
  inspect(rotation.c > 0.7 && rotation.c < 0.8, content="true") // cos(45°) ≈ 0.707
  inspect(rotation.s > 0.7 && rotation.s < 0.8, content="true") // sin(45°) ≈ 0.707
  
  // Transform points from local to world coordinates
  let local_point = @box2d.Vec2::{ x: 1.0, y: 0.0 }
  let world_point = @box2d.transform_point(transform, local_point)
  
  // Transformed point should be rotated and translated
  inspect(world_point.x > 5.7 && world_point.x < 5.8, content="true") // ≈ 5.707
  inspect(world_point.y > 3.7 && world_point.y < 3.8, content="true") // ≈ 3.707
  
  // Inverse transform (world to local)
  let back_to_local = @box2d.inv_transform_point(transform, world_point)
  
  // Should get back original local point
  inspect(back_to_local.x > 0.99 && back_to_local.x < 1.01, content="true")
  inspect(back_to_local.y > -0.01 && back_to_local.y < 0.01, content="true")
  
  // Transform vectors (no translation) 
  let local_vector = @box2d.Vec2::{ x: 2.0, y: 0.0 }
  let world_vector = @box2d.rotate_vector(transform.q, local_vector)
  
  // Vector should be rotated by 45 degrees
  inspect(world_vector.x > 1.4 && world_vector.x < 1.5, content="true") // ≈ 1.414
  inspect(world_vector.y > 1.4 && world_vector.y < 1.5, content="true") // ≈ 1.414
}
```

### AABB (Axis-Aligned Bounding Box) Operations

```moonbit
test "AABB operations" {
  let circle = @box2d.Circle::{ center: @box2d.Vec2::{ x: 1.0, y: 2.0 }, radius: 0.5 }
  let transform = @box2d.Transform::{ 
    p: @box2d.Vec2::{ x: 0.0, y: 0.0 }, 
    q: @box2d.rot_identity 
  }
  
  // Compute AABB for circle
  let aabb = @box2d.compute_circle_aabb(circle, transform)
  
  // Circle center (1,2) with radius 0.5 should have AABB from (0.5,1.5) to (1.5,2.5)
  inspect(aabb.lower_bound.x, content="0.5")
  inspect(aabb.lower_bound.y, content="1.5")
  inspect(aabb.upper_bound.x, content="1.5")
  inspect(aabb.upper_bound.y, content="2.5")
  
  // AABB operations
  let aabb2 = @box2d.AABB::{
    lower_bound: @box2d.Vec2::{ x: 0.0, y: 0.0 },
    upper_bound: @box2d.Vec2::{ x: 2.0, y: 3.0 }
  }
  
  // Test overlap - aabb (0.5,1.5)-(1.5,2.5) should overlap with aabb2 (0,0)-(2,3)
  let overlaps = @box2d.aabb_overlap(aabb, aabb2)
  inspect(overlaps, content="true")
  
  // Test containment - aabb2 should contain aabb
  let contains = @box2d.aabb_contains(aabb2, aabb)
  inspect(contains, content="true")
  
  // Test union
  let union = @box2d.aabb_union(aabb, aabb2)
  inspect(union.lower_bound.x, content="0")
  inspect(union.lower_bound.y, content="0")
  inspect(union.upper_bound.x, content="2")
  inspect(union.upper_bound.y, content="3")
}
```

## API Reference

### Core Types

- **`Vec2`**: 2D vector with x, y components
- **`Transform`**: Position and rotation (p: Vec2, q: Rot)
- **`WorldId`**, **`BodyId`**, **`ShapeId`**: Unique identifiers for physics objects
- **`Circle`**, **`Capsule`**, **`Polygon`**, **`Segment`**: Shape types
- **`Manifold`**: Collision contact information
- **`AABB`**: Axis-aligned bounding box

### World Management

- **`create_world(def: WorldDef)`**: Create physics world
- **`world_step(world: WorldId, dt: Double, iterations: Int)`**: Step simulation
- **`world_is_valid(world: WorldId)`**: Check world validity

### Body Management

- **`create_body(world: WorldId, def: BodyDef)`**: Create rigid body
- **`body_get_position(body: BodyId)`**: Get body position
- **`body_get_type(body: BodyId)`**: Get body type (Static/Dynamic/Kinematic)

### Shape Creation

- **`create_circle_shape(body: BodyId, def: ShapeDef, circle: Circle)`**
- **`create_capsule_shape(body: BodyId, def: ShapeDef, capsule: Capsule)`**
- **`create_polygon_shape(body: BodyId, def: ShapeDef, polygon: Polygon)`**
- **`create_segment_shape(body: BodyId, def: ShapeDef, segment: Segment)`**

### Math Operations

- Vector arithmetic: `vec2_add`, `sub`, `mul_sv`, `vec2_dot`, `cross`
- Transform operations: `transform_point`, `inv_transform_point`, `mul_transforms`
- Rotation utilities: `make_rot`, `rot_get_angle`, `rotate_vector`

## Status

This is a work-in-progress port of Box2D. Currently implemented:

- ✅ Core math library (vectors, transforms, rotations)
- ✅ Basic shapes (circle, capsule, polygon, segment)
- ✅ Collision detection and manifold generation
- ✅ Physics simulation with gravity and integration
- ✅ AABB operations and spatial queries
- ✅ Memory management and entity storage
- 🚧 Joint system (planned)
- 🚧 Broad phase collision detection (planned)
- 🚧 Advanced constraints (planned)

## License

This port maintains compatibility with the original Box2D license (MIT).

## Contributing

Contributions welcome! This port aims to stay faithful to the original Box2D while leveraging MoonBit's type system and performance characteristics.