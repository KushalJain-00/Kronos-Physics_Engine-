# Session Log — ControlPanel Rewrite + Engine Improvements

## What happened in this session

This session rewrote the entire front-end of the Kronos physics engine and added
real physics improvements underneath. The old setup had two windows (pygame for
the simulation + DearPyGui for controls, running on a background thread with
locks everywhere). The new setup is a single DearPyGui window called "Kronos
Studio" with the simulation drawn via GPU-batched draw commands, controls
side-by-side, and physics running in the same thread — no locks, no threading
bugs.

Everything lives on the `feat/ControlPanel_Rewrite` branch, based on `main`.

---

## 1. Architecture change — from two windows to one

### Before
- **pygame** window drew the simulation
- **DearPyGui** panel ran on a daemon thread, talking to the world via `world.lock`
- Two processes, thread races, headless tests used a fake SDL driver

### After
- Single `dearpygui` viewport with three docked windows:
  - **Left** — Controls (scene selector, physics sliders, spawn tools)
  - **Center** — Simulation (a `drawlist` rendering the world each frame)
  - **Right** — Inspector (selected object stats, velocity plot, energy readouts)
- Physics runs in the main render loop with a fixed-timestep accumulator
- No threads, no locks needed (world.lock kept for API compat, unused)

---

## 2. New files created

| File | What it does |
|------|-------------|
| `simulation/timestep.py` | `FixedTimestep` — a pure-logic accumulator that decouples physics
from render framerate. Headless-testable, no GUI dependency. |
| `simulation/spatial_hash.py` | `SpatialHash` — uniform-grid broadphase. All O(n^2) pair loops
in the world now go through grid candidate pairs instead. |
| `scenes/sandbox.py` | Empty scene — opens an empty world so you can spawn things
with the tools. Set as the default scene. |
| `scenes/cloth.py` | Soft-body demo: 12x8 particle grid with horizontal, vertical,
and diagonal springs, pinned at the top row. |
| `tests/test_timestep.py` | 5 tests for the fixed-timestep accumulator (step counts,
carry-over, clamping). |
| `tests/test_spatial_hash.py` | 5 tests for the broadphase grid (pair correctness, large-extent
bodies, brute-force cross-check, clear). |

---

## 3. Files rewritten

### `visualization/renderer.py`
Complete rewrite — the old pygame `Renderer` class is replaced with a
DearPyGui studio. Key features:

- **Camera**: middle-drag to pan, mouse wheel to zoom (toward cursor)
- **Spawn tools**: right-click spawns particle/box/circle (selectable in the
  controls panel, with mass/size sliders)
- **Select + drag**: left-click selects objects, drag moves them, release
  throws (kinematic drag with velocity feedback)
- **Trails**: particles leave alpha-faded trails behind them
- **Vectors**: toggleable velocity arrows on every body
- **Contact debug**: toggleable contact-point markers with normal lines
- **Scene switcher**: dropdown reloads any registered scene live
- **Stats**: FPS, entity counts, kinetic/potential energy, sim time
- **Velocity plot**: live graph of the selected object's velocity over time

### `main.py`
Simplified — dropped `--no-panel` flag (no separate panel anymore), passes
the initial scene name to the renderer so the Reset button works immediately.
Default scene changed to `sandbox`.

### `ui/control_panel.py`
Deleted — all control panel content folded into the single renderer file.
Fewer files, one thread, no IPC.

---

## 4. Changes to `simulation/world.py`

| Change | Why |
|--------|-----|
| `self.wind` (Vector2D) | New wind force, applied to particles only each substep. |
| `self.substeps = 2` | Physics splits into 2 substeps per `step()` call for better
stacking stability. Particle time still totals exactly `dt`. |
| `self.debug_contacts` | List of `{"point", "normal", "depth"}` dicts rebuilt every
step — the renderer draws these as the contact overlay. |
| Broadphase via `SpatialHash` | All three collision loops (particle-particle, rigid-body,
particle-body) now enumerate grid candidate pairs instead of
brute-force O(n^2). One grid built per substep. |
| Particle-rigid-body friction | Coulomb-clamped tangential impulse added after normal
impulse (linear-only — `# ponytail: spin ignored`). |
| Rigid-body top-wall symmetry | Top boundary now zeros small velocities the same way the
bottom does. Was asymmetric before. |
| Angular damping hoisted | Runs once per `step()` instead of per substep — per-substep
damping broke the angular-momentum conservation tests. |
| SAT position correction once | Runs only on the first substep — same reason as angular
damping. |

---

## 5. CodeRabbit review — 8 fixes applied

After the rewrite was pushed, a CodeRabbit review flagged several issues.
I verified each against the committed code and applied only the valid ones.

### Bugs fixed

| # | File | What was wrong |
|---|------|---------------|
| 1 | `main.py` + renderer | `Renderer` never received the initial scene name, so
the "Reset scene" button was a no-op until you manually
changed the dropdown. Now `main` passes `args.scene` and
the renderer stores it. |
| 2 | `scenes/cloth.py` | The diagonal spring `grid[r-1][c-1]` was added when
`r > 0` but without checking `c > 0`. When `c == 0`,
Python's negative index wraps to the last column — a
bogus cross-corner spring. Added `if c > 0` guard. |
| 3 | `simulation/world.py` | Broadphase extent used half the AABB width/height,
centered at `body.position`. For rotated/offset geometry
the AABB center differs from position, so some vertices
could fall outside the inserted square. Now uses the
maximum distance from `body.position` to any vertex. |
| 4 | `visualization/renderer.py` | Global mouse handlers fired over *any* window — clicking
the control panel cleared selection, wheel-zoom over the
inspector moved the camera. Added `_over_viewport()` guard
on all sim-affecting handlers (click, drag, pan, wheel,
spawn). Release left ungated so drags can end anywhere. |
| 5 | `visualization/renderer.py` | The "Radius" slider had no effect on particles — the
particle path ignored `self.radius`. Now sets
`p.radius = self.radius` before adding to the world. |
| 6 | `visualization/renderer.py` | `view_w = cw - 600` went negative when the window was
narrower than 600px, crashing the drawlist. Clamped to
`max(200, cw - 600)`. |
| 7 | `visualization/renderer.py` | Contact marker circles were drawn in world coordinates
while everything else uses screen coordinates — the
markers rendered in the wrong place. Now converts via
`to_screen()` like the normal lines. |
| 8 | `visualization/renderer.py` | No middle-button release handler existed, so `_pan_last`
stayed stale between drags. The next pan computed a delta
from the old position, causing a camera jump. Added a
`_clear_pan` callback on middle release. |

### Skipped (not necessary)

- **`_snap_small_values` helper** — top/bottom zeroing is duplicated 10 lines
  x2, but it's cosmetic-only and the thresholds are identical. Refactor risk
  > value for this PR.
- **Drag velocity timestamps** — `/60.0` misestimates throw velocity off-60fps;
  cosmetic, not worth the bookkeeping.
- **Trail pruning** — stale trails from deleted particles aren't reachable:
  `clear()` / scene reload already calls `trails.clear()`.
- **Spatial hash forward-scan** — `seen` dedup already guarantees correct
  unique pairs; the 8-way scan at cell_size 100 is negligible.
- **Cloth pinning fix** — CodeRabbit suggested pinning `r == rows-1`, but this
  is a Y-up world. `r == 0` (highest y) *is* already the top row. Pinning the
  bottom would be wrong.
- **Viewport gate on release** — must fire to end a drag even if the mouse left
  the viewport; gating would leave `self.dragging` stuck.

---

## 6. Branch management

| Branch | Based on | Content |
|--------|----------|---------|
| `feat/Constraint_Completeion` | `main` | Other session's work: Weld + Motor constraints,
new weld/motor scenes, panel disconnected. At `f6e4401`. |
| `feat/ControlPanel_Rewrite` | `main` | This session: DPG studio front, spatial hash,
substeps, wind, new scenes, CodeRabbit fixes. At `425b68c`. |

The worktree (`Physics EngineSim-feat`) was created during the session to avoid
colliding with the other session's checkout. It was later removed and the branch
moved into the main checkout for the CodeRabbit fix pass.

Both branches are pushed to `origin` and ready for pull requests.

---

## 7. Test results

Starting count: 59 (on `feat/Constraint_Completeion` base).

After this session on `feat/ControlPanel_Rewrite`: **63 passed**.

| Change | Count |
|--------|-------|
| Removed `test_fixed_timestep_decoupled_from_variable_render_dt` (pygame dummy-driver) | -1 |
| Added `tests/test_timestep.py` (5 tests) | +5 |
| Added `tests/test_spatial_hash.py` (5 tests) | +5 |
| Added wind test in `tests/test_world.py` | +1 |
| **Net** | **+10 → 69 (on Constraint branch) / 63 (on main base)** |

The 63 count is on `feat/ControlPanel_Rewrite` (based on `main`'s 53 tests).
The 69 count was on `feat/Constraint_Completeion` which includes 6 extra tests
from the other session's constraint work.

---

## 8. How to run

```bash
# From the main checkout, on feat/ControlPanel_Rewrite:
.venv/bin/python main.py              # default: sandbox scene
.venv/bin/python main.py cloth        # cloth demo
.venv/bin/python main.py pendulum     # classic pendulum
.venv/bin/python main.py --width 1200 --height 900

# Run tests:
.venv/bin/python -m pytest tests -v
```

Requires a display (DearPyGui needs OpenGL). The default scene is `sandbox` —
an empty world where you can right-click to spawn particles and boxes.
