# Kronos Codebase Context

2D physics engine written from scratch in Python (numpy only for math primitives). No physics libraries. Y-up world coordinates, pygame renderer, DearPyGui control panel.

## Layout

| Path | Contents |
|---|---|
| `core/vectors.py` | `Vector2D(x, y)` — `add`, `sub`, `multiply(scalar)`, `magnitude`, `dot_product`, `normalize` (zero-safe → returns `(0,0)`), `distance`, `angle` (degrees, clamped cos). Caveat: `angle` divides by zero on a zero vector. |
| `core/particles.py` | `Particle(x, y, mass, color)` — mass must be > 0 (`ValueError`), `radius = max(5, int(mass * 5))`, `pinned` flag, velocity Verlet in `update(dt)` (position then velocity, MAX_VELOCITY=1000 clamp, acceleration reset, `time += dt` even when pinned). |
| `core/rigidbody.py` | `RigidBody(x, y, mass, angle, color)` — `set_shape(vertices)` (polygon, centered on position), `_calculate_inertia` via shoelace second-moment formula `(m/6)·Σabs(cross)·Q / Σabs(cross)` (moment about origin), `get_world_vertices` (rotate+translate), `get_axes`/`project_onto_axis`, `sat_collision` (min-overlap axis, first tie wins, returns `{"normal", "depth"}` or `None`; returns depth-0 dict at exact touch), `find_contact_point` (deepest vertex inside other), `particle_collision` (closest point on each edge), `area()` / `center_of_mass()` (shoelace, added for tests — flagged API additions). |
| `core/springs.py` | `Spring(p1, p2, length, k=0.1, damp=0.02)` — Hooke + damping along axis, equal-and-opposite, zero-length guard. |
| `core/chains_and_ropes.py` | `Link(Particle)` — adds `friction`, `radius=5`. (README calls this `core/link.py`; the real file is `chains_and_ropes.py`.) |
| `core/constraints.py` | `DistanceConstraint` — generalized PBD with inverse mass `w = 1/m + (r×n)²/I` (rotational lever-arm), angle/angular_velocity corrected for rigid bodies. `HingeConstraint` — subclass of DistanceConstraint with `rest_length=0`. `AngleConstraint` — clamps relative rotation to `[min,max]`, degrees at construction, wraps rel angle to `[-π,π]`, pinned-aware, velocity-corrected once. `MotorConstraint` — drives `ω_b−ω_a` to a target, inertia-split, pinned-aware. `WeldConstraint` — composes Hinge + Angle(min==max==current rel angle). `ChainConstraint(world, a, b, anchor_a, anchor_b, n_links, stiffness, friction, iterations=4, visible_links)` — `n_links-1` Links + `n_links` DistanceConstraint segments, 4 (configurable) solve iterations, per-segment Coulomb friction incl. anchor joints using `world.gravity`; validates n_links first. |
| `simulation/world.py` | `World(width, height)` — lists for particles/springs/rigid_bodies/constraints/links, `gravity=(0,-9.8)`, `restitution=0.9`, `rigid_body_restitution=0.5`, `drag_coefficient=0`, `mu=0.1`, `dt=0.1`, `paused`, `lock` (threading). `step(dt)`: rigid bodies (gravity, update, angular damping `0.99**(dt/0.016)`, boundaries), springs, particles (gravity+drag, boundaries), SAT position correction (slop=0.5, correction=0.2), links, 8 collision passes (particle-particle impulse `dot <= 0 → skip`, rigid-body impulse + Coulomb friction + torques, particle-rigidbody), 5 constraint passes. `_handle_boundaries` bounces at 4 walls; `clear()` empties everything (no per-object remove API). |
| `visualization/renderer.py` | `Renderer(world)` — `run()` loop with fixed physics_dt=0.008 accumulator decoupled from render fps (frame_time clamped to 0.05), selection (point-in-polygon), right-click spawns particles. |
| `ui/control_panel.py` | DearPyGui on a daemon thread, all world reads/writes under `world.lock`, live stats + velocity plot. |
| `scenes/` | `pendulum`, `hinge_joint`, `rope`, `stacking_test`, `weld_joint`, `motor` — each `build(world)` populates a World. |
| `main.py` | argparse → `World` → scene build → renderer (+ optional panel). |

## Physics approach

- Integration: velocity Verlet (position with ½a·dt² then velocity); exactly symplectic for position-dependent forces (harmonic energy stays bounded).
- Collision: SAT for polygons, impulse response `J = -(1+e)·(v_rel·n)/(1/m1+1/m2)`, position correction with slop, Coulomb-clamped friction for non-hinged pairs.
- Constraints: PBD — positions corrected directly, velocities derived from the same correction.

## Known quirks / non-conservative terms

- Angular damping (world.py:57) breaks strict angular-momentum conservation across steps.
- Slop position correction is non-conservative for angular momentum unless momenta are along the collision normal.
- Rigid-body boundaries only zero small velocities; no restitution nuance on top wall handling.

## Running

```bash
.venv/bin/python main.py [scene] [--no-panel]
```

## Tests

```bash
.venv/bin/python -m pytest tests/ -v
```

pytest (stdlib `unittest` was considered; pytest chosen per repo direction). Tests use fixtures from `tests/conftest.py`, never bare float equality (always `pytest.approx` with justified tolerance), one invariant per test, sentence-style test names. GUI modules (renderer loop, control panel) are only exercised headlessly via dummy SDL driver in `test_world.py::test_fixed_timestep_decoupled_from_variable_render_dt`.

## Test inventory

- `test_vectors.py` — math primitives vs closed forms.
- `test_conservation.py` — momentum/KE conservation (elastic + inelastic), rigid-body angular momentum, 200-step isolated-system momentum drift, spring energy with damping=0 (integrator-error detector).
- `test_inertia.py` / `test_shapes.py` — shoelace vs closed form: rectangle `m(w²+h²)/12`, mass scaling, square area, symmetric centroid.
- `test_collision.py` — SAT detection/separation/depth/normal-convention, particle-vs-body, zero-penetration edge case.
- `test_constraints.py` — Distance convergence/no-op/one-step, pinned immobility, Hinge coincidence, Chain link counts, velocity-correction-exactly-once regression.
- `test_world.py` — clock, gravity kinematics, fixed-timestep accumulator, add/clear.
- `test_regressions.py` — CodeRabbit-caught bugs: impulse sign, axis normalization, zero-length/zero-velocity division, lock race.
