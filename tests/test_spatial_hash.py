"""Unit tests for the spatial hash broadphase: deterministic pair enumeration, multi-cell extents, a brute-force cross-check, and clearing."""

import random

from simulation.spatial_hash import SpatialHash


def _seeded_entries(seed=1234, n=40, grid=None):
    """Inserts n random objects (fixed seed) into a fresh grid and returns (grid, [(obj, x, y, extent), ...]) for brute-force reference."""
    random.seed(seed)
    grid = grid or SpatialHash(cell_size=100.0)
    entries = []
    for _ in range(n):
        obj = object()
        x = random.uniform(0.0, 1000.0)
        y = random.uniform(0.0, 1000.0)
        extent = random.uniform(0.0, 150.0)
        grid.insert(obj, x, y, extent=extent)
        entries.append((obj, x, y, extent))
    return grid, entries


def test_pairs_contains_exactly_the_expected_unordered_pairs():
    """Objects sharing or bordering a cell must pair once and only once — no self-pairs, no duplicates, and symmetric pairs not repeated."""
    grid = SpatialHash(cell_size=100.0)
    a = object()
    b = object()
    c = object()
    d = object()
    e = object()
    grid.insert(a, 0, 0)  # cell (0,0)
    grid.insert(b, 99, 0)  # cell (0,0) — same cell as a
    grid.insert(c, 150, 0)  # cell (1,0) — neighbor of (0,0)
    grid.insert(d, 150, 150)  # cell (1,1) — diagonal neighbor of (0,0)
    grid.insert(e, 400, 400)  # cell (4,4) — far from all others
    expected = {
        frozenset((a, b)),
        frozenset((a, c)),
        frozenset((a, d)),
        frozenset((b, c)),
        frozenset((b, d)),
        frozenset((c, d)),
    }
    assert {frozenset(p) for p in grid.pairs()} == expected


def test_large_extent_body_pairs_with_objects_far_from_its_center():
    """A body with a 250-unit extent spans cells -2..2, so it must pair with an object 240 units from its center (same bbox) but not one 600 units away."""
    grid = SpatialHash(cell_size=100.0)
    body = object()
    near = object()
    far = object()
    grid.insert(body, 0, 0, extent=250.0)
    grid.insert(near, 240, 0)  # 240 units from center, inside the body's bbox
    grid.insert(far, 600, 0)  # 600 units from center, outside the body's bbox
    pairs = {frozenset(p) for p in grid.pairs()}
    assert frozenset((body, near)) in pairs
    assert frozenset((body, far)) not in pairs


def test_grid_pairs_include_every_brute_force_overlapping_pair():
    """Every pair whose bounding boxes overlap must be reported by the grid — the broadphase may add candidates but must never miss a collision."""
    grid, entries = _seeded_entries()
    pairs = {frozenset(p) for p in grid.pairs()}
    for i, (obj_i, xi, yi, ei) in enumerate(entries):
        for obj_j, xj, yj, ej in entries[i + 1:]:
            if abs(xi - xj) <= ei + ej and abs(yi - yj) <= ei + ej:
                assert frozenset((obj_i, obj_j)) in pairs


def test_grid_pairs_exclude_boxes_with_a_gap_larger_than_two_cells():
    """When both bounding-box gaps exceed 2*cell_size the objects' cells are at least two apart, so the neighbor check must not report them."""
    grid, entries = _seeded_entries()
    pairs = {frozenset(p) for p in grid.pairs()}
    for i, (obj_i, xi, yi, ei) in enumerate(entries):
        for obj_j, xj, yj, ej in entries[i + 1:]:
            gap = max(abs(xi - xj) - (ei + ej), abs(yi - yj) - (ei + ej))
            if gap > 2 * grid.cell_size:
                assert frozenset((obj_i, obj_j)) not in pairs


def test_clear_empties_the_grid():
    """clear() must drop every cell so a re-populated grid starts from an empty slate."""
    grid, _ = _seeded_entries(n=10)
    assert len(grid.pairs()) > 0
    grid.clear()
    assert grid.pairs() == []
