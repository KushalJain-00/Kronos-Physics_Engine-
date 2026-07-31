# simulation

This package manages the active simulation world.

## What is here
- world.py: owns particles, springs, rigid bodies, constraints, and links.
- Handles stepping the physics loop, applying gravity, collisions, boundaries, and constraint solving.

## Purpose
The simulation package is the runtime controller that updates the world every frame and keeps the objects interacting with each other.

## How it connects to the engine
This package sits in the middle of the system: it collects objects from the core package, runs their physics updates, and passes the resulting state to the renderer and the UI for display and control.
