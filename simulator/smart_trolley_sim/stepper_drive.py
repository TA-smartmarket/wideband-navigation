"""Optional stepper drivetrain for the simulated trolley.

When enabled, the plant drives its wheels through the same C++ step-rate
conversion and acceleration profile the A4988 firmware uses, so:

* the simulated trolley cannot accelerate faster than a NEMA 17 allows,
* `wheel_odometry_distance_m` becomes a genuine STEP-pulse measurement,
* the step/position API can be exercised from the simulator.

This is opt-in (`physics.use_stepper_model`) because it makes the plant stiffer
than the simple first-order model and is only needed when studying the stepper
behaviour specifically.
"""

from __future__ import annotations

from dataclasses import dataclass

from .native_core import StepperAxis


@dataclass
class StepperPair:
    """Two stepper axes plus the conversion constants the plant needs."""

    left: StepperAxis
    right: StepperAxis
    steps_per_meter: float

    def reset(self) -> None:
        self.left.reset()
        self.right.reset()

    def close(self) -> None:
        self.left.close()
        self.right.close()

    def odometry_distance_m(self) -> float:
        """Mean ground distance derived from the STEP pulse counts."""
        return 0.5 * (self.left.distance_m + self.right.distance_m)

    def odometry_yaw_rad(self, wheel_base_m: float) -> float:
        if wheel_base_m <= 1e-6:
            return 0.0
        return (self.right.distance_m - self.left.distance_m) / wheel_base_m

    def revolutions(self) -> tuple[float, float]:
        return self.left.revolutions, self.right.revolutions


def create_stepper_pair(steps_per_revolution: int = 200, microsteps: int = 16,
                        wheel_radius_m: float = 0.05, max_step_rate_hz: float = 20000.0,
                        max_step_accel_hz_per_s: float = 20000.0,
                        invert_left: bool = False, invert_right: bool = False) -> StepperPair:
    """Build both axes with the NEMA 17 / A4988 defaults."""
    left = StepperAxis(steps_per_revolution, microsteps, wheel_radius_m,
                       max_step_rate_hz, max_step_accel_hz_per_s, invert_left)
    right = StepperAxis(steps_per_revolution, microsteps, wheel_radius_m,
                        max_step_rate_hz, max_step_accel_hz_per_s, invert_right)
    steps_per_meter = left.steps_per_meter
    return StepperPair(left=left, right=right, steps_per_meter=steps_per_meter)
