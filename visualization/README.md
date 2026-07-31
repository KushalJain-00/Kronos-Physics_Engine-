# visualization

This package handles the visual display of the physics world.

## What is here
- renderer.py: renders the grid, particles, springs, rigid bodies, constraints, and handles mouse-based selection and spawning.

## Purpose
The visualization package turns the simulation state into a visible 2D scene using Pygame.

## How it connects to the engine
The renderer reads the current world state from the simulation and draws what is happening in real time, making the physics results visible to the user.
