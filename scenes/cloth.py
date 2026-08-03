from core.particles import Particle
from core.springs import Spring
from simulation.world import World

def build(world: World) -> None:
    cols, rows, spacing = 12, 8, 12.0
    x0 = world.width / 2 - (cols - 1) * spacing / 2
    y0 = world.height - 150
    grid = [[Particle(x0 + c * spacing, y0 + r * spacing, 1.0) for c in range(cols)] for r in range(rows)]
    for r in range(rows):
        for c in range(cols):
            p = grid[r][c]
            if r == 0:
                p.pinned = True
            world.add_particle(p)
            if c > 0:
                world.add_spring(Spring(p, grid[r][c - 1], spacing, k=0.5, damp=0.02))
            if r > 0:
                world.add_spring(Spring(p, grid[r - 1][c], spacing, k=0.5, damp=0.02))
                if c > 0:
                    world.add_spring(Spring(p, grid[r - 1][c - 1], spacing, k=0.5, damp=0.02))
