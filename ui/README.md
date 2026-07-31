# ui

This package contains the interactive control interface for the engine.

## What is here
- control_panel.py: a Dear PyGui window with controls for pause/resume, clearing the world, adjusting gravity, restitution, and drag, plus live simulation statistics.

## Purpose
The ui package lets you inspect and tune the simulation while it runs.

## How it connects to the engine
The control panel reads and writes the same world state used by the simulation loop, so it can pause the engine, change world settings, and display live information about the current objects.
