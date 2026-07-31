import pytest

from core.constraints import AngleConstraint, MotorConstraint, WeldConstraint


class FakeBody:
    """Minimal stand-in for RigidBody, exposing only the attributes that
    AngleConstraint.solve() reads/writes."""

    def __init__(self, angle=0.0, angular_velocity=0.0, moment_of_inertia=None):
        self.angle = angle
        self.angular_velocity = angular_velocity
        if moment_of_inertia is not None:
            self.moment_of_inertia = moment_of_inertia


class TestAngleConstraintInit:
    def test_stores_bodies_and_limits(self):
        body_a = FakeBody()
        body_b = FakeBody()
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0)

        assert constraint.body_a is body_a
        assert constraint.body_b is body_b
        assert constraint.max_angle == 1.0
        assert constraint.min_angle == -1.0

    def test_default_stiffness(self):
        constraint = AngleConstraint(FakeBody(), FakeBody(), max_angle=1.0, min_angle=-1.0)
        assert constraint.stiffness == 0.5

    def test_custom_stiffness(self):
        constraint = AngleConstraint(
            FakeBody(), FakeBody(), max_angle=1.0, min_angle=-1.0, stiffness=0.8
        )
        assert constraint.stiffness == 0.8


class TestAngleConstraintSolveWithinBounds:
    def test_no_change_when_relative_angle_within_bounds(self):
        body_a = FakeBody(angle=0.0, angular_velocity=0.0)
        body_b = FakeBody(angle=0.5, angular_velocity=0.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0)

        constraint.solve()

        assert body_a.angle == 0.0
        assert body_b.angle == 0.5
        assert body_a.angular_velocity == 0.0
        assert body_b.angular_velocity == 0.0

    def test_no_change_when_relative_angle_negative_within_bounds(self):
        body_a = FakeBody(angle=3.0, angular_velocity=0.0)
        body_b = FakeBody(angle=1.0, angular_velocity=0.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=5.0, min_angle=-5.0)

        constraint.solve()

        assert body_a.angle == 3.0
        assert body_b.angle == 1.0
        assert body_a.angular_velocity == 0.0
        assert body_b.angular_velocity == 0.0

    def test_no_change_at_exact_max_boundary(self):
        # relative_angle == max_angle should NOT trigger correction (strict '>')
        body_a = FakeBody(angle=0.0)
        body_b = FakeBody(angle=1.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0)

        constraint.solve()

        assert body_a.angle == 0.0
        assert body_b.angle == 1.0
        assert body_a.angular_velocity == 0.0
        assert body_b.angular_velocity == 0.0

    def test_no_change_at_exact_min_boundary(self):
        # relative_angle == min_angle should NOT trigger correction (strict '<')
        body_a = FakeBody(angle=0.0)
        body_b = FakeBody(angle=-1.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0)

        constraint.solve()

        assert body_a.angle == 0.0
        assert body_b.angle == -1.0
        assert body_a.angular_velocity == 0.0
        assert body_b.angular_velocity == 0.0


class TestAngleConstraintSolveExceedsMax:
    def test_correction_applied_when_above_max(self):
        body_a = FakeBody(angle=0.0, angular_velocity=0.0)
        body_b = FakeBody(angle=2.0, angular_velocity=0.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=0.5)

        constraint.solve()

        # correction = (relative_angle - max_angle) * stiffness = (2.0 - 1.0) * 0.5 = 0.5
        assert body_a.angle == pytest.approx(0.5)
        assert body_b.angle == pytest.approx(1.5)
        assert body_a.angular_velocity == pytest.approx(-0.5)
        assert body_b.angular_velocity == pytest.approx(0.5)

    def test_correction_scales_with_stiffness(self):
        body_a = FakeBody(angle=0.0)
        body_b = FakeBody(angle=2.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=1.0)

        constraint.solve()

        # full stiffness -> full correction of the excess (1.0)
        assert body_a.angle == pytest.approx(1.0)
        assert body_b.angle == pytest.approx(1.0)
        assert body_a.angular_velocity == pytest.approx(-1.0)
        assert body_b.angular_velocity == pytest.approx(1.0)

    def test_zero_stiffness_produces_no_effective_correction(self):
        body_a = FakeBody(angle=0.0, angular_velocity=0.0)
        body_b = FakeBody(angle=2.0, angular_velocity=0.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=0.0)

        constraint.solve()

        assert body_a.angle == pytest.approx(0.0)
        assert body_b.angle == pytest.approx(2.0)
        assert body_a.angular_velocity == pytest.approx(0.0)
        assert body_b.angular_velocity == pytest.approx(0.0)

    def test_repeated_solve_calls_converge_toward_max_angle(self):
        body_a = FakeBody(angle=0.0)
        body_b = FakeBody(angle=2.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=0.5)

        previous_gap = body_b.angle - body_a.angle - constraint.max_angle
        for _ in range(20):
            constraint.solve()
            gap = body_b.angle - body_a.angle - constraint.max_angle
            assert gap >= -1e-9
            assert gap <= previous_gap + 1e-9
            previous_gap = gap

        # After many iterations the relative angle should have converged close to max_angle
        assert (body_b.angle - body_a.angle) == pytest.approx(1.0, abs=1e-3)


class TestAngleConstraintSolveBelowMin:
    def test_correction_applied_when_below_min(self):
        body_a = FakeBody(angle=0.0, angular_velocity=0.0)
        body_b = FakeBody(angle=-2.0, angular_velocity=0.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=0.5)

        constraint.solve()

        # correction = (min_angle - relative_angle) * stiffness = (-1.0 - (-2.0)) * 0.5 = 0.5
        assert body_a.angle == pytest.approx(-0.5)
        assert body_b.angle == pytest.approx(-1.5)
        assert body_a.angular_velocity == pytest.approx(0.5)
        assert body_b.angular_velocity == pytest.approx(-0.5)

    def test_correction_scales_with_stiffness(self):
        body_a = FakeBody(angle=0.0)
        body_b = FakeBody(angle=-2.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=1.0)

        constraint.solve()

        assert body_a.angle == pytest.approx(-1.0)
        assert body_b.angle == pytest.approx(-1.0)
        assert body_a.angular_velocity == pytest.approx(1.0)
        assert body_b.angular_velocity == pytest.approx(-1.0)


class TestAngleConstraintMomentOfInertiaHandling:
    def test_solve_ignores_missing_moment_of_inertia(self):
        # moment_of_inertia defaults to 0 via getattr; solve() must not raise
        body_a = FakeBody(angle=0.0)
        body_b = FakeBody(angle=2.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0)

        constraint.solve()  # should not raise AttributeError

        assert body_a.angle == pytest.approx(0.5)
        assert body_b.angle == pytest.approx(1.5)

    def test_solve_behaves_same_regardless_of_moment_of_inertia_value(self):
        # `w` is computed from moment_of_inertia but never used to affect the
        # correction, so varying it should not change the resulting angles.
        body_a = FakeBody(angle=0.0, moment_of_inertia=10.0)
        body_b = FakeBody(angle=2.0, moment_of_inertia=25.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=1.0, min_angle=-1.0, stiffness=0.5)

        constraint.solve()

        assert body_a.angle == pytest.approx(0.5)
        assert body_b.angle == pytest.approx(1.5)


class TestAngleConstraintAsymmetricLimits:
    def test_asymmetric_limits_max_only_violation(self):
        body_a = FakeBody(angle=1.0)
        body_b = FakeBody(angle=4.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=2.0, min_angle=-10.0, stiffness=0.25)

        constraint.solve()

        # relative_angle = 3.0, correction = (3.0 - 2.0) * 0.25 = 0.25
        assert body_a.angle == pytest.approx(1.25)
        assert body_b.angle == pytest.approx(3.75)

    def test_asymmetric_limits_min_only_violation(self):
        body_a = FakeBody(angle=1.0)
        body_b = FakeBody(angle=-4.0)
        constraint = AngleConstraint(body_a, body_b, max_angle=10.0, min_angle=-2.0, stiffness=0.25)

        constraint.solve()

        # relative_angle = -5.0, correction = (-2.0 - (-5.0)) * 0.25 = 0.75
        assert body_a.angle == pytest.approx(0.25)
        assert body_b.angle == pytest.approx(-3.25)


class TestConstraintStubClasses:
    """MotorConstraint and WeldConstraint are added as unimplemented stubs in
    this PR. Their __init__/solve methods are missing the `self` parameter,
    so instantiating them currently fails. These tests pin down that current
    (incomplete) behavior so future changes to these classes are intentional."""

    def test_motor_constraint_instantiation_raises_type_error(self):
        with pytest.raises(TypeError):
            MotorConstraint()

    def test_weld_constraint_instantiation_raises_type_error(self):
        with pytest.raises(TypeError):
            WeldConstraint()