# Kronos Physics Engine

> *"Master of time. Simulator of worlds."*

A modular, high-precision 2D physics simulation engine built from scratch in Python. Kronos simulates real-world physical systems — from classical particle dynamics to rigid body interactions — with the long-term goal of becoming a full-scale world simulation engine with an LLM interface layer.

---

## Current State

Kronos is in active development. Phase 1 (particle simulation) is complete. Phase 2 (rigid bodies) is in progress.

---

## Architecture

```
Physics-engine/
    core/
        vectors.py       # Vector2D math — add, sub, multiply, magnitude, normalize, dot, distance, angle
        particles.py     # Particle class with Verlet integration
        rigidbody.py     # RigidBody class with SAT collision and rotation
        springs.py       # Spring forces with Hooke's law and damping
    simulation/
        world.py         # World state — manages all objects, forces, collisions
    visualization/
        renderer.py      # Pygame renderer with fixed timestep loop
    main.py              # Entry point
```

---

## Features

### Vector Math (`vectors.py`)
- `Vector2D` class with full 2D math operations
- Addition, subtraction, scalar multiplication
- Magnitude, normalization, dot product
- Distance and angle between vectors

### Particle System (`particles.py`)
- Point mass particles with variable mass and radius
- Verlet integration for accurate position and velocity updates
- Force accumulation and reset per frame
- Velocity capping to prevent numerical explosion
- Visual radius scales with mass

### Spring Forces (`springs.py`)
- Hooke's Law: `F = -k * (current_length - rest_length)`
- Configurable stiffness `k` and damping coefficient
- Damping opposes relative velocity along spring axis
- Equal and opposite forces on both connected particles
- Stable with `k=0.1`, `damp=0.5`, `dt=0.016`

### World Simulation (`world.py`)
- Gravity force applied per frame scaled by mass
- Drag force opposing velocity
- Configurable restitution for particles and rigid bodies separately
- Particle boundary collision with radius-aware correction
- Particle-particle collision with proper 2D impulse resolution
- Spring force application before particle update
- Rigid body boundary collision with penetration correction and sleep thresholds
- Rigid body collision detection via SAT
- Rigid body collision response with friction impulse

### Rigid Bodies (`rigidbody.py`)
- General convex polygon shape defined by vertex list
- Moment of inertia calculated from polygon geometry using shoelace method
- Verlet integration for both linear and rotational motion
- `get_world_vertices()` — transforms local vertices to world space using rotation matrix
- `get_axes()` — SAT face normals for collision detection
- `project_onto_axis()` — projects polygon onto axis, returns min/max
- `sat_collision()` — full SAT with minimum translation vector (MTV) output
- Angular velocity damping framerate independent

### Collision System
**Particle-Particle:**
- Circle vs circle distance check
- Position correction to prevent overlap
- Impulse-based momentum conservation
- Proper 2D collision normal

**Rigid Body-Rigid Body:**
- SAT (Separating Axis Theorem) collision detection
- MTV (Minimum Translation Vector) for penetration depth and normal
- Normal direction correction ensuring correct push direction
- Mass-weighted position correction with slop threshold
- Linear impulse resolution
- Friction impulse with Coulomb friction model (`mu=0.3`)
- Angular impulse from friction at contact

### Renderer (`renderer.py`)
- Fixed timestep physics loop decoupled from rendering
- `physics_dt=0.008` for stable simulation
- Frame time capped at 50ms to prevent spiral of death
- 60fps rendering via `clock.tick(60)`
- Grid visualization with coordinate labels
- Particle rendering with mass-proportional radius
- Spring rendering as colored line between particles
- Rigid body rendering as wireframe polygon
- SAT debug visualization (yellow axes) — toggle as needed
- Mouse click spawns particles (left=mass 1, right=mass 5)

---

## Known Limitations

- Rigid bodies rest at arbitrary angles — requires friction-based contact torque to fix fully
- Stacking multiple rigid bodies is unstable — needs multiple solver iterations
- No concave shape support — convex polygons only
- No particle-rigid body collision
- No joint or constraint system yet

---

## Roadmap

### Phase 1 — Classical Particle Simulation ✅
- Vector math
- Verlet integration
- Gravity and drag
- Variable mass particles
- Spring forces
- Particle-particle collision
- Pygame visualization

### Phase 2 — Rigid Bodies (In Progress)
- ✅ Polygon rigid body
- ✅ Moment of inertia
- ✅ Rotation and angular dynamics
- ✅ SAT collision detection
- ✅ Impulse collision response
- ✅ Friction impulse
- ⬜ Proper contact point calculation
- ⬜ Multiple solver iterations
- ⬜ Stable stacking
- ⬜ Particle-rigid body collision

### Phase 3 — Constraints
- Distance constraints
- Joints and hinges
- Rope and chain simulation

### Phase 4 — Optimization
- Spatial partitioning (quadtree)
- Broad phase collision detection
- Performance profiling

### Phase 5 — LLM Interface
- `get_state()` API returning structured JSON
- Event logging (collisions, boundary hits)
- Energy tracking (kinetic, potential, total)
- Natural language scenario description → simulation parameters
- Designed for integration with a custom LLM (in development separately)

---

## Usage

```python
from core.particles import Particle
from core.springs import Spring
from core.rigidbody import RigidBody
from simulation.world import World
from visualization.renderer import Renderer

world = World(800, 600)
renderer = Renderer(world)

# particle simulation
p1 = Particle(300, 400, 1.0)
p2 = Particle(500, 400, 5.0)
spring = Spring(p1, p2, length=100, k=0.1, damp=0.5)
world.add_particle(p1)
world.add_particle(p2)
world.add_spring(spring)

# rigid body simulation
body = RigidBody(400, 300, 1.0, 0.0)
body.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
body.angular_velocity = 1.0
world.add_rigid_bodies(body)

renderer.run()
```

---

## Requirements

```
python >= 3.10
pygame >= 2.0
numpy
```

---

## Built By

Kronos is being built from scratch as a foundation for a larger world simulation platform with LLM integration. Every system — vectors, integration, collision, rendering — written and debugged by hand.