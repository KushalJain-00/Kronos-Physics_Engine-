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
- [Current Features](#current-features)
- [Architecture](#architecture)
- [How It Works](#how-it-works)
- [Getting Started](#getting-started)
- [Usage Examples](#usage-examples)
- [Roadmap](#roadmap)
- [Known Limitations](#known-limitations)

---

## What Is Kronos

Kronos is a 2D physics simulation engine written in Python with zero physics library dependencies. Every system — integration, collision detection, constraint solving, friction, rigid body dynamics — is implemented from first principles using real physics formulas and mathematical derivations.

It is not a game and not a wrapper around Box2D or any other physics library. It is a ground-up implementation of classical mechanics.

---

## Current Features

### Phase 1 — Particles ✅
- `Vector2D` math primitives (add, subtract, multiply, dot product, normalize, distance, angle)
- Velocity Verlet integration, gravity, per-particle drag, variable mass/radius
- Particle-particle and boundary collision with restitution
- Springs (Hooke's law + velocity damping), pinned particles

### Phase 2 — Rigid Bodies ✅
- General convex polygons, moment of inertia via the shoelace method
- Verlet integration for linear and rotational motion
- SAT collision detection + MTV response, Coulomb friction, contact points
- Particle–rigid-body collision, fixed timestep decoupled from render framerate

### Phase 3 — Constraints ✅
- **DistanceConstraint** — fixed distance between anchor points on any two bodies. Uses generalized inverse mass (linear + rotational) so off-center anchors on rigid bodies correctly apply torque.
- **HingeConstraint** — pins two anchor points with free relative rotation. A subclass of `DistanceConstraint` with `rest_length = 0`.
- **ChainConstraint** — multi-link rope/chain between two bodies with inter-link Coulomb friction.
- **AngleConstraint** — clamps relative rotation to a `[min, max]` range (degrees at construction, wrapped to `[-π, π]` internally so full rotations never misfire).
- **MotorConstraint** — drives relative angular velocity `ω_b − ω_a` toward a target.
- **WeldConstraint** — zero relative DOF: locks relative position *and* angle, composing a hinge + angle lock.

---

## Architecture

```
kronos/
├── core/
│   ├── vectors.py          # Vector2D — all 2D math primitives
│   ├── particles.py        # Particle — point mass, Verlet integration
│   ├── rigidbody.py        # RigidBody — convex polygon, SAT, rotation
│   ├── springs.py          # Spring — Hooke's law + damping
│   ├── chains_and_ropes.py # Link — chain node (Particle + friction)
│   └── constraints.py      # Distance, Hinge, Chain, Angle, Motor, Weld
├── simulation/
│   └── world.py            # World — owns all objects, runs physics loop
├── visualization/
│   └── renderer.py         # Pygame renderer, fixed timestep accumulator
├── ui/
│   └── control_panel.py    # Dear PyGui panel (currently disconnected — see main.py)
├── scenes/                 # One module per demo scene
├── main.py                 # Entry point and scene selection
└── tests/                  # pytest suite
```

### Design Principles

**Fixed timestep:** physics runs at a fixed `0.008s` step regardless of framerate; the renderer accumulates real time and steps as needed, then interpolates.

**PBD constraints:** Position Based Dynamics — constraints correct positions directly and derive velocity corrections from the same correction. Applied exactly once per solve (duplicate application was an early critical bug).

**Generalized inverse mass:** for rigid-body constraints, each body's effective inverse mass is `w = 1/m + (r × n)² / I`, where `r` is the anchor's lever arm. This is the term that lets a constraint rotate a body instead of only translating it. It is shared by `DistanceConstraint` and (via subclass) `HingeConstraint`.

---

## How It Works

### Particles

Velocity Verlet:

```
position += velocity * dt + 0.5 * acceleration * dt²
velocity += acceleration * dt
```

### Rigid Bodies

- **Inertia** from the shoelace formula: `I = (m/6) · Σ|cross_i|·Q / Σ|cross_i|`
- **Collision** via SAT; impulse response `J = -(1+e)·(v_rel·n) / (1/m1+1/m2)`, with `Δv = J/m·n` and `Δω = (r×J·n)/I`
- **Friction** clamped by the Coulomb limit `|Jt| ≤ μ·|Jn|`

### Constraints

All constraints expose `solve()` and are solved generically by the world's 5-pass loop.

**DistanceConstraint / HingeConstraint** — generalized inverse mass:

```
w_i = 1/m_i + (r_i × n)² / I_i
λ = (current_length − rest_length) · stiffness / (w_a + w_b)

Δposition_i = ± n · inv_mass_i · λ
Δangle_i     = ± (r_i × n) · λ / I_i
```

**AngleConstraint** — wraps `b.angle − a.angle` to `[-π, π]` before comparing to `[min, max]`, splits the correction by inverse inertia, and mirrors the correction into angular velocity.

**MotorConstraint** — velocity-level: `Δω = (target − (ω_b − ω_a)) · (inv_i / Σinv)`, split by inverse inertia, pinned bodies excluded.

**WeldConstraint** — composes a `HingeConstraint` (point coincidence) and an `AngleConstraint` with `min == max == current relative angle`.

**ChainConstraint** — creates `n_links − 1` Link nodes and `n_links` `DistanceConstraint` segments, then applies inter-node friction (including at the anchor joints) using the world's gravity:

```
friction_impulse = clamp(−v_tangential · reduced_mass, −μ·m·|g|, μ·m·|g|)
```

---

## Getting Started

### Requirements

```
python >= 3.10
pygame
numpy
```

### Installation

```bash
git clone https://github.com/KushalJain-00/Kronos-Physics_Engine-.git
cd Kronos-Physics_Engine-
pip install pygame numpy
python main.py <scene>
```

### Available Scenes

| Scene | Description |
|---|---|
| `pendulum` | A pinned particle swinging a bob on a rigid rod |
| `hinge_joint` | Two rigid bodies pinned at a hinge |
| `rope` | A multi-link chain holding a payload |
| `stacking_test` | A vertical box stack (solver stress test) |
| `weld_joint` | Two bodies welded into one rigid unit |
| `motor` | A pinned stator driving a rotor at constant speed |

### Controls

| Input | Action |
|---|---|
| Left click | Select object |
| Right click | Spawn particle at cursor |

The Dear PyGui control panel is currently disconnected (see `main.py` for the reconnect point).

---

## Usage Examples

### Pendulum

```python
anchor = Particle(400, 500, 1.0); anchor.pinned = True
bob = Particle(400, 350, 2.0)
constraint = DistanceConstraint(anchor, bob, (0, 0), (0, 0), 150, stiffness=1.0)
world.add_particle(anchor); world.add_particle(bob); world.add_constraint(constraint)
```

### Hinge

```python
a = RigidBody(400, 400, 10.0, 0.0); a.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
b = RigidBody(400, 300, 1.0, 0.0);  b.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
hinge = HingeConstraint(a, b, (0, 25), (0, -25))
world.add_rigid_bodies(a); world.add_rigid_bodies(b); world.add_constraint(hinge)
```

### Motor

```python
stator = RigidBody(400, 400, 100.0, 0.0); stator.pinned = True
rotor = RigidBody(400, 400, 2.0, 0.0)
hinge = HingeConstraint(stator, rotor, (0, 0), (0, 0))
motor = MotorConstraint(stator, rotor, target_angular_velocity=2.0)
world.add_constraint(hinge); world.add_constraint(motor)
```

---

## Roadmap

### Current — Phase 3: Constraints ✅
- [x] DistanceConstraint
- [x] HingeConstraint
- [x] ChainConstraint / Rope
- [x] AngleConstraint — limits relative rotation
- [x] MotorConstraint — drives a joint at target angular velocity
- [x] WeldConstraint — locks relative position and angle

### Phase 4: Solver Quality
- [ ] Multi-point contact manifolds
- [ ] Sequential impulse solver with warm starting
- [ ] Sleeping bodies

### Phase 5: Broad Phase Optimization
- [ ] Spatial partitioning (uniform grid or BVH), eliminating O(n²) collision checks

### Phase 6: Continuous Collision Detection
- [ ] Swept SAT / conservative advancement to stop tunnelling

### Phase 7: Soft Body / Deformable
- [ ] Cloth (mass-spring grid), soft-body blobs

### Phase 8: Fluid Simulation
- [ ] SPH with density/pressure/viscosity kernels, fluid–rigid coupling

### Phase 9: Ragdoll / Articulated Systems
- [ ] Composed constraint chains, skeletons, linkages

### Phase 10: Polish
- [ ] Buoyancy, force fields, breakable constraints, raycasting, scene save/load

### Phase 11: 3D (Full Rewrite)
- [ ] OpenGL/Vulkan renderer, GJK+EPA, quaternions, 3×3 inertia tensor

---

## Known Limitations

| Issue | Root Cause | Fix Planned |
|---|---|---|
| Rigid body balanced on corner indefinitely | No resting-contact torque solver | Phase 4 |
| Single contact point per collision | No manifold generation | Phase 4 |
| Stacking instability under many objects | No warm starting | Phase 4 |
| O(n²) collision detection | No spatial partitioning | Phase 5 |
| Fast objects tunnel through thin surfaces | No CCD | Phase 6 |
