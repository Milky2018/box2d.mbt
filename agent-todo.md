# Box2D MoonBit Port - TODO List

This file tracks remaining tasks for completing the Box2D physics engine port to MoonBit. Tasks are organized by priority and complexity.

## 🔥 High Priority (Essential Features)

### Contact System & Material Properties
- [x] **Fix friction constraint solving** (solver.mbt:127) - Replace hardcoded 0.3 with actual shape friction values
- [x] **Fix restitution handling** (solver.mbt:128) - Replace hardcoded 0.2 with actual shape restitution values  
- [x] **Fix timestep in bias calculation** (solver.mbt:130) - Use actual timestep instead of hardcoded 60.0
- [x] **Implement contact event callbacks** - Begin/end contact events for game logic
- [ ] **Add rolling friction support** - Prevent unrealistic rolling behavior

### Shape Filtering System
- [ ] **Add collision filtering** - Category bits, mask bits, group indices for collision layers
- [ ] **Implement shape sensors** - Shapes that detect overlap without collision response
- [ ] **Add collision callbacks** - Pre-solve, post-solve contact modification

### Missing Joint Types
- [ ] **Motor Joint** - Direct velocity control joint for kinematic driving
- [ ] **Weld Joint** - Rigid connection joint (removes all degrees of freedom)
- [ ] **Wheel Joint** - Vehicle wheel simulation with suspension
- [ ] **Joint breaking** - Joints that break under excessive force

## 🎯 Medium Priority (Commonly Needed)

### Query & Testing Systems
- [ ] **Overlap queries** - Test shapes against world or other shapes for triggers
- [ ] **Point-in-shape queries** - Which shapes contain a given point
- [ ] **Shape casting** - Moving shape collision detection
- [ ] **Advanced ray casting** - Ray vs world with filtering options

### Shape System Enhancements
- [ ] **Chain shapes** - Multi-segment connected chains (extend ChainSegment)
- [ ] **Compound shapes** - Multiple shapes as single collision object
- [ ] **Shape modification** - Runtime shape vertex/radius changes
- [ ] **Shape destruction callbacks** - Cleanup when shapes are removed

### World Features
- [ ] **Debug drawing interface** - Visual debugging of physics objects
- [ ] **World ray casting** - Cast rays through entire world with filtering
- [ ] **World explosion effects** - Apply radial impulses to bodies in radius
- [ ] **World bounds** - Optional world boundaries with callbacks

## 📊 Low Priority (Advanced Features)

### Performance & Advanced Physics
- [ ] **Continuous collision detection** - Prevent high-speed object tunneling
- [ ] **Multi-threading support** - Parallel constraint solving
- [ ] **Profile data collection** - Performance timing and statistics
- [ ] **Memory pool optimization** - Reduce allocation overhead

### Math & Utility Improvements  
- [ ] **NaN/infinity detection** (math_functions.mbt:88) - When MoonBit supports it
- [ ] **Full GJK triangle case** (distance.mbt:146) - Complete GJK implementation
- [ ] **SIMD math operations** - Vectorized math when available
- [ ] **Fixed-point math option** - For deterministic simulation

### Advanced Constraint Features
- [ ] **One-way platforms** - Shapes passable from one direction
- [ ] **Joint motors with PID** - More sophisticated motor control
- [ ] **Joint limits with softness** - Soft joint limits with spring-damper
- [ ] **Joint motors with gear ratios** - Coupled joint movements

## 🧪 Testing & Quality

### Test Coverage
- [ ] **Joint stress tests** - Verify joint stability under extreme forces
- [ ] **Performance benchmarks** - Compare with original Box2D performance  
- [ ] **Contact manifold accuracy tests** - Verify collision detection precision
- [ ] **Determinism tests** - Ensure reproducible simulation results

### Documentation & Examples
- [ ] **API documentation** - Complete function documentation
- [ ] **Physics tutorial examples** - Car physics, platformer, particle systems
- [ ] **Performance guide** - Optimization best practices
- [ ] **Migration guide** - From other physics engines

## 🔧 Code Quality & Maintenance

### Code Improvements
- [ ] **Error handling** - Proper error types instead of invalid IDs
- [ ] **Logging system** - Debug and performance logging
- [ ] **Assert macros** - Debug-mode validation checks
- [ ] **Code formatting** - Ensure consistent style throughout

### API Enhancements
- [ ] **Builder pattern APIs** - Fluent interfaces for complex object creation
- [ ] **Event listener registration** - Type-safe callback registration
- [ ] **Resource management** - Automatic cleanup of physics objects
- [ ] **Serialization support** - Save/load world state

## 📋 Implementation Notes

### Quick Wins (Easy for AI agents)
These tasks are well-defined and can be completed independently:
- ✅ Replace hardcoded values in solver.mbt with actual material properties
- ✅ Implement Motor Joint (similar pattern to existing joints)
- ✅ Add basic overlap query functions
- ✅ Create debug drawing interface structure

### Complex Tasks (Require careful design)
These need more planning and integration work:
- Continuous collision detection (affects core integration loop)
- Multi-threading (requires thread-safe data structures)
- Contact event system (needs callback management)
- Advanced constraint features (affects solver architecture)

### Dependencies
Some tasks depend on MoonBit language features:
- NaN/infinity detection awaits language support
- SIMD operations depend on MoonBit SIMD capabilities
- Multi-threading needs MoonBit threading primitives

---

**Overall Completion Status: ~70%**
**Core Physics: ✅ Excellent**  
**Game Integration: 🔶 Needs work**
**Advanced Features: ❌ Missing**

*Last updated: Based on analysis of commit 58fc1c8*