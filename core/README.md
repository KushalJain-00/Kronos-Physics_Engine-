# core

This package contains the physics engine's fundamental building blocks.

## What is here
- vectors.py: 2D vector math utilities used across the engine.
- particles.py: point-mass particles with integration, gravity, drag, and pinned support.
- rigidbody.py: convex rigid bodies with inertia, rotation, and collision handling.
- springs.py: spring forces with damping.
- constraints.py: joint-like constraints such as distance, hinge, and chain systems.
- chains_and_ropes.py: link objects used by rope-like constraints.

## Purpose
The core package is the mathematical and physical foundation of the project. Most simulation behavior starts here.

## How it connects to the engine
This package provides the objects and rules that the simulation world updates each frame. The world uses particles, rigid bodies, springs, and constraints from here to build the behavior you see in the renderer and control panel.
