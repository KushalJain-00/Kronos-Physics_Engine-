"""Unit tests for the fixed-timestep accumulator that decouples physics from the render framerate."""

import pytest
from simulation.timestep import FixedTimestep


def test_fifty_millisecond_frame_yields_exactly_six_eight_millisecond_steps():
    """A 50 ms frame must produce exactly 6 steps of 0.008 s — 0.05 / 0.008 = 6.25, so the leftover 0.25 of a step stays in the accumulator."""
    ts = FixedTimestep(physics_dt=0.008)
    assert ts.advance(0.05) == 6


def test_frame_smaller_than_physics_dt_yields_zero_steps():
    """A 4 ms frame is smaller than the 8 ms physics step, so it must be banked without stepping."""
    ts = FixedTimestep(physics_dt=0.008)
    assert ts.advance(0.004) == 0


def test_fractional_accumulation_carries_over_across_frames():
    """Two 4 ms frames must yield zero steps on the first and exactly one 8 ms step on the second — the partial step carries over instead of being dropped."""
    ts = FixedTimestep(physics_dt=0.008)
    assert ts.advance(0.004) == 0
    assert ts.advance(0.004) == 1


def test_frame_dt_is_clamped_to_max_frame_dt():
    """A 1.0 s frame must be clamped to max_frame_dt = 0.05, yielding at most floor(0.05 / 0.008) = 6 steps instead of a 125-step burst."""
    ts = FixedTimestep(physics_dt=0.008, max_frame_dt=0.05)
    assert ts.advance(1.0) == 6


def test_accumulator_keeps_sub_step_residual_after_advance():
    """After a 50 ms frame the accumulator must hold the 2 ms residual (0.05 - 6*0.008), so no physics time is lost to integer division."""
    ts = FixedTimestep(physics_dt=0.008)
    ts.advance(0.05)
    assert ts.accumulator == pytest.approx(0.05 - 6 * 0.008, abs=1e-12)
