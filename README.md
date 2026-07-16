# Kronos Physics Engine

> A multi-domain physics simulation engine built entirely from scratch in Python.
> No physics libraries. No game engine. Every formula implemented manually.

![Python](https://img.shields.io/badge/Python-3.10+-blue)
![Pygame](https://img.shields.io/badge/Renderer-Pygame-green)
![Status](https://img.shields.io/badge/Status-Active%20Development-orange)
![Phase](https://img.shields.io/badge/Phase-3%20Constraints-yellow)

---

## Table of Contents

- [What Is Kronos](#what-is-kronos)
- [Why Build This](#why-build-this)
- [Current Features](#current-features)
- [Architecture](#architecture)
- [How It Works](#how-it-works)
  - [Particles](#particles)
  - [Rigid Bodies](#rigid-bodies)
  - [Constraints](#constraints)
  - [Control Panel](#control-panel)
- [Getting Started](#getting-started)
- [Usage Examples](#usage-examples)
- [Roadmap](#roadmap)
- [Known Limitations](#known-limitations)
- [Contributing](#contributing)
- [Long-Term Vision](#long-term-vision)

---

## What Is Kronos

Kronos is a 2D physics simulation engine written in Python with zero physics library dependencies. Every system — integration, collision detection, constraint solving, friction, rigid body dynamics — is implemented from first principles using real physics formulas and mathematical derivations.

It is not a game. It is not a wrapper around Box2D or any other physics library. It is a ground-up implementation of classical mechanics that will eventually expand into a full multi-domain physics software suite covering soft body dynamics, fluid simulation, 3D physics, quantum mechanics simulation, and optics.

The current focus is classical 2D mechanics: particles, rigid bodies, springs, and a growing constraint system.

---

## Why Build This

Most physics simulations in software call a library and never understand what happens underneath. Kronos exists to close that gap completely.

Every bug fixed here represents a physics concept deeply understood — not just called from an API. SAT collision, PBD constraint solving, impulse resolution, moment of inertia, Verlet integration, Coulomb friction — all implemented by hand, verified against real conservation laws, and reviewed via CodeRabbit AI on every pull request.

It is also the technical foundation for a larger personal project roadmap:

```
Kronos (physics engine)
    → Cybersecurity Agent (authorized pentest/red-team tool)
        → Multi-Agent Orchestration System
            → Jarvis-style Personal Assistant
```

---

## Current Features

### Phase 1 — Particles ✅
- Vector2D math primitives (add, subtract, multiply, dot product, normalize, distance, angle)
- Verlet integration for position and velocity
- Gravity and per-particle drag
- Variable mass and radius
- Particle-particle collision with proper 2D impulse resolution
- Boundary collision with restitution
- Springs with Hooke's law and velocity damping
- Pinned particle support (infinite effective mass)
- Grid visualization overlay

### Phase 2 — Rigid Bodies ✅
- General convex polygon shapes
- Moment of inertia calculation via shoelace method
- Verlet integration for both linear and rotational motion
- SAT (Separating Axis Theorem) collision detection with normalized axes
- MTV (Minimum Translation Vector) based collision response
- Normal impulse with restitution
- Coulomb friction model (static and dynamic)
- Contact point calculation (deepest penetrating vertex)
- Particle-rigid body collision handling
- Fixed timestep physics decoupled from render framerate
- Multiple solver iterations for stacking stability
- Jitter reduction via penetration slop and correction factor

### Phase 3 — Constraints (In Progress) 🔄
- **DistanceConstraint** — maintains fixed distance between anchor points on any two bodies. Stiffness controls behavior from rigid rod (1.0) to rubber band (0.01)
- **HingeConstraint** — pins two anchor points together with free relative rotation. Uses generalized inverse mass with rotational lever-arm term for correct rigid body behavior
- **ChainConstraint** — multi-link rope or chain connecting two bodies via intermediate link nodes, with Coulomb inter-link friction *(in progress)*

### Control Panel ✅
- Gravity, restitution, drag sliders
- Pause/resume and clear world buttons
- Live simulation stats (FPS, object counts, kinetic/potential energy)
- Selected object inspector (position, velocity, acceleration)
- Live velocity graph (200 frame rolling window)

---

## Architecture

```
kronos/
├── core/
│   ├── vectors.py          # Vector2D — all 2D math primitives
│   ├── particles.py        # Particle — point mass, Verlet integration
│   ├── rigidbody.py        # RigidBody — convex polygon, SAT, rotation
│   ├── springs.py          # Spring — Hooke's law + damping
│   ├── constraints.py      # DistanceConstraint, HingeConstraint, ChainConstraint
│   └── link.py             # Link — chain node, inherits Particle, adds friction
├── simulation/
│   └── world.py            # World — owns all objects, runs physics loop
├── visualization/
│   └── renderer.py         # Pygame renderer, fixed timestep accumulator
├── ui/
│   └── control_panel.py    # Dear PyGui control panel, runs on separate thread
├── main.py                 # Entry point and scene construction
└── README.md
```

### Design Principles

**Fixed timestep:** Physics runs at a fixed 0.008s step regardless of framerate. The renderer accumulates real time and steps physics as many times as needed to catch up, then interpolates rendering. This ensures deterministic, stable simulation.

**Thread safety:** The Dear PyGui control panel runs on a separate thread. All shared world state is protected by `threading.Lock()`. The physics loop holds the lock for its entire duration; the control panel acquires it only during reads and writes.

**PBD constraints:** Position Based Dynamics — constraints directly correct positions and derive velocity corrections from those corrections. Simple, stable, and easy to iterate on. Not as accurate as impulse-based solvers for stiff systems, but sufficient for the current phase.

**CodeRabbit AI review:** Every pull request is reviewed by CodeRabbit before merge. It has caught real bugs including sign errors in impulse calculations, unnormalized collision axes, race conditions in the threading model, and division-by-zero in edge cases.

---

## How It Works

### Particles

Particles are point masses integrated with the Velocity Verlet method:

```
position += velocity * dt + 0.5 * acceleration * dt²
velocity += acceleration * dt
acceleration = F / m  (reset each frame)
```

Particle-particle collision resolves using the 1D impulse formula extended to 2D along the collision normal:

```
J = -(1 + e) * (v_rel · n) / (1/m1 + 1/m2)
v1 += (J / m1) * n
v2 -= (J / m2) * n
```

Springs apply Hooke's law with velocity damping along the spring axis each frame, before integration.

### Rigid Bodies

Rigid bodies extend particle physics with rotation and shape.

**Moment of inertia** is calculated analytically from the polygon vertices using the shoelace formula — the same method used in structural engineering:

```
I = (m/6) * Σ |cross_i| * (x1² + x1*x2 + x2² + y1² + y1*y2 + y2²)
              ─────────────────────────────────────────────────────────
                                    Σ |cross_i|
```

**Collision detection** uses SAT. Every edge of both polygons contributes a test axis (its outward normal). The bodies are projected onto each axis. If any projection doesn't overlap, they don't collide. The axis with minimum overlap gives the collision normal and penetration depth.

**Collision response** applies a normal impulse to both linear and angular velocity:

```
J = -(1 + e) * (v_rel · n) / (1/m1 + 1/m2)

Δv = J/m * n
Δω = (r × J*n) / I
```

**Friction** is applied as a tangential impulse, clamped by the Coulomb limit:

```
|Jt| ≤ μ * |Jn|
```

### Constraints

**DistanceConstraint** uses PBD position correction:

```
error = current_length - rest_length
correction = delta * (error / length) * stiffness
```

Correction is split between bodies by mass ratio. Velocity is updated to match the position correction in the same step (once, not twice — duplicate application was an early critical bug).

**HingeConstraint** uses generalized inverse mass that accounts for both linear and rotational resistance:

```
w = 1/m + (r × n)² / I
λ = |error| / (w_a + w_b)

Δposition = n * inv_mass * λ
Δangle = (r × n) * λ / I
```

This is the correct lever-arm term missing from naive distance constraints applied to rigid bodies. Without it, the constraint can only translate bodies, not properly rotate them to satisfy the joint.

**ChainConstraint** internally creates n_links - 1 Link nodes and n_links DistanceConstraint segments. Its own solve loop iterates all segments, then applies Coulomb friction between consecutive link pairs:

```
friction_impulse = clamp(-v_tangential * reduced_mass, -μ*m*g, μ*m*g)
```

### Control Panel

The control panel runs entirely on a separate thread using Dear PyGui. It reads and writes world state only while holding `world.lock`. The physics thread also holds this lock during `step()`. This prevents any torn reads of position/velocity data while the panel is rendering graphs.

---

## Getting Started

### Requirements

```
python >= 3.10
pygame
dearpygui
numpy
```

### Installation

```bash
git clone https://github.com/KushalJain-00/Kronos-Physics_Engine-.git
cd Kronos-Physics_Engine-
pip install pygame dearpygui numpy
python main.py
```

### Controls

| Input | Action |
|---|---|
| Left click | Select object (shows in control panel) |
| Right click | Spawn particle at cursor |
| Control panel | Adjust gravity, restitution, drag |
| Pause/Resume | Freeze/unfreeze simulation |
| Clear World | Remove all objects |

---

## Usage Examples

### Basic Pendulum
```python
world = World(800, 600)

anchor = Particle(400, 500, 1.0)
anchor.pinned = True

bob = Particle(400, 350, 2.0)

constraint = DistanceConstraint(anchor, bob, (0,0), (0,0), 150, stiffness=1.0)

world.add_particle(anchor)
world.add_particle(bob)
world.add_constraint(constraint)
```

### Hinge Joint Between Two Rigid Bodies
```python
world = World(800, 600)

body_a = RigidBody(400, 400, 10.0, 0.0)
body_a.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])

body_b = RigidBody(400, 300, 1.0, 0.0)
body_b.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])

hinge = HingeConstraint(body_a, body_b, (0, 25), (0, -25))

world.add_rigid_bodies(body_a)
world.add_rigid_bodies(body_b)
world.add_constraint(hinge)
```

### Rope Between Two Bodies
```python
world = World(800, 600)

anchor = RigidBody(400, 500, 100.0, 0.0)  # heavy, barely moves
anchor.set_shape([(-30,-10),(30,-10),(30,10),(-30,10)])

payload = RigidBody(400, 300, 2.0, 0.0)
payload.set_shape([(-20,-20),(20,-20),(20,20),(-20,20)])

rope = ChainConstraint(
    world, anchor, payload,
    anchor_a=(0, -10), anchor_b=(0, 20),
    n_links=8, stiffness=0.8, friction=0.2
)

world.add_rigid_bodies(anchor)
world.add_rigid_bodies(payload)
world.add_constraint(rope)
```

---

## Roadmap

### Current — Phase 3: Constraints
- [x] DistanceConstraint
- [x] HingeConstraint
- [ ] ChainConstraint / Rope *(in progress)*
- [ ] AngleConstraint — limits relative rotation between bodies
- [ ] MotorConstraint — drives hinge at target angular velocity
- [ ] WeldConstraint — zero relative DOF, locks position and angle

### Phase 4: Solver Quality
- [ ] Multi-point contact manifolds (2 contact points per polygon pair)
- [ ] Sequential impulse solver with warm starting
- [ ] Sleeping bodies (stop simulating objects at rest)

This phase eliminates the root causes of current jitter, penetration, and stacking instability.

### Phase 5: Broad Phase Optimization
- [ ] Spatial partitioning (uniform grid or BVH)
- [ ] Eliminate O(n²) collision checks
- [ ] Required prerequisite for fluids and soft body at scale

### Phase 6: Continuous Collision Detection
- [ ] Swept SAT or conservative advancement
- [ ] Prevent fast objects from tunnelling between frames

### Phase 7: Soft Body / Deformable
- [ ] Cloth (mass-spring grid with self-collision)
- [ ] Soft body blobs (shape matching or FEM-lite)

### Phase 8: Fluid Simulation
- [ ] SPH (Smoothed Particle Hydrodynamics)
- [ ] Density, pressure, viscosity kernels
- [ ] Fluid-rigid body coupling
- [ ] Separate particle system from existing Particle class

### Phase 9: Ragdoll / Articulated Systems
- [ ] Composed constraint chains for joints
- [ ] Human/creature skeletons
- [ ] Mechanical linkages and robotic arms

### Phase 10: Polish
- [ ] Buoyancy and fluid coupling
- [ ] Force fields (directional, radial, vortex)
- [ ] Breakable constraints
- [ ] Raycasting queries
- [ ] Scene save/load

### Phase 11: 3D (Full Rewrite)
A complete rewrite, not an extension. Key changes:
- Renderer: Pygame → OpenGL/Vulkan
- Collision: SAT → GJK + EPA
- Rotation: angle → quaternions
- Inertia: scalar → 3×3 tensor
- All constraint math extended to 3D cross products

### Phase 12: Quantum Simulation *(separate module)*
- Schrödinger equation solver: `iℏ ∂ψ/∂t = Ĥψ`
- Finite difference / split-operator numerical methods
- Probability density visualization
- Scenarios: particle in a box, quantum tunnelling, double slit, harmonic oscillator

### Phase 13: Optics *(separate module)*
- **Ray optics:** reflection, refraction (Snell's law), lens simulation, dispersion
- **Wave optics:** Huygens wavelets, interference, diffraction, FDTD wave propagation

---

## Known Limitations

| Issue | Root Cause | Fix Planned |
|---|---|---|
| Rigid body balanced on corner indefinitely | No resting-contact torque solver | Phase 4 |
| Single contact point per collision | No manifold generation | Phase 4 |
| Stacking instability under many objects | No warm starting | Phase 4 |
| O(n²) collision detection | No spatial partitioning | Phase 5 |
| Fast objects can tunnel through thin surfaces | No CCD | Phase 6 |
| Hinged bodies still overlap slightly | Constraint vs collision position correction fight | Phase 4 |

---

## Contributing

Contributions are welcome. If you would like to improve the engine, open an issue or submit a pull request with a focused change and a short explanation.

## Long-Term Vision

Kronos is the first step toward an open, multi-domain physics software suite — an alternative to closed commercial tools like COMSOL, ANSYS, and MATLAB Simulink. The goal is software that is genuinely useful across scientific and engineering fields, built on a foundation where every formula is understood, not imported.

The expansion path:

```
2D Classical Mechanics (current)
    → 3D Classical Mechanics (full rewrite)
        → Soft Body / Fluids (added to 3D engine)
            → Quantum Simulation (separate module, same suite)
                → Optics (separate module, same suite)
                    → Unified multi-domain simulation software
```

---

## Working Principles

- **No physics libraries** — every formula from first principles
- **No skipping phases** — each phase completes before the next begins
- **PR for every feature** — CodeRabbit AI review before every merge
- **One bug at a time** — diagnose from actual code, not descriptions
- **Verify against physics** — conservation laws, not just "it looks right"

---

*Built by Kushal Jain — CS student, co-founder of TriSangum Labs.*
*Part of a longer personal roadmap: Kronos → Cybersecurity Agent → Multi-Agent System → Jarvis.*