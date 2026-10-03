"""Trolley plant simulation (differential drive with motor dynamics).

The plant consumes the normalised wheel commands produced by the C++ path
follower and integrates the chassis pose.  Nothing here knows about navigation:
the simulator must not teleport the trolley along the planned route, it must
react to motor commands exactly like the real hardware would.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import PhysicsConfig
from .native_core import StepperAxis


@dataclass
class TrolleyState:
    """Ground-truth pose and wheel state (never fed to the navigation core)."""

    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0
    left_wheel_mps: float = 0.0
    right_wheel_mps: float = 0.0
    commanded_left: float = 0.0
    commanded_right: float = 0.0
    distance_travelled_m: float = 0.0

    @property
    def linear_velocity_mps(self) -> float:
        return 0.5 * (self.left_wheel_mps + self.right_wheel_mps)

    @property
    def angular_velocity_radps(self) -> float:
        return 0.0  # replaced by the plant, which knows the wheel base


class TrolleyPlant:
    """First-order motor model + exact unicycle integration.

    Motor response uses a discrete first-order lag with time constant
    ``motor_time_constant_s``; the wheel command deadband models static friction
    so the PID cannot park the trolley in a half-powered buzzing state.
    """

    def _stepper_wheel_speed(self, axis) -> float:
        """Wheel speed the axis is actually emitting (from its STEP rate).

        The stepper's acceleration limit means the commanded speed is not reached
        instantly, so the plant must use the *emitted* rate: that is what makes
        the simulated trolley honour the same motion profile as the hardware.
        """
        steps_per_meter = self._stepper.steps_per_meter if self._stepper else 0.0
        if steps_per_meter <= 1e-6:
            return 0.0
        return axis.current_rate_hz / steps_per_meter

    def __init__(self, physics: PhysicsConfig, rng=None, stepper=None):
        self.physics = physics
        self.rng = rng
        self.state = TrolleyState()
        self._omega_radps = 0.0
        # Optional stepper axes: when supplied, the wheels are driven through the
        # same STEP-rate conversion the A4988 firmware uses, and the odometry is
        # the pulse count rather than an integral of velocity.
        self._stepper = stepper
        self._last_step_distance = 0.0

    # -- lifecycle ---------------------------------------------------------
    def set_stepper(self, stepper) -> None:
        """Attach (or detach with None) the optional stepper drivetrain."""
        self._stepper = stepper
        self._last_step_distance = 0.0

    def reset(self, x_m: float, y_m: float, theta_rad: float = 0.0) -> None:
        self.state = TrolleyState(x_m=x_m, y_m=y_m, theta_rad=theta_rad)
        self._omega_radps = 0.0
        self._last_step_distance = 0.0
        if self._stepper is not None:
            self._stepper.reset()

    # -- properties --------------------------------------------------------
    @property
    def angular_velocity_radps(self) -> float:
        return self._omega_radps

    @property
    def wheel_base_m(self) -> float:
        return self.physics.wheel_base_m

    # -- simulation --------------------------------------------------------
    def _command_to_target_speed(self, command: float, gain: float) -> float:
        """Normalised command [-1, 1] -> target wheel linear speed (m/s)."""
        command = max(-1.0, min(1.0, command))
        # Deadband: a command that cannot overcome static friction produces no
        # motion, but once moving the full command is applied (no discontinuity
        # in the middle of the range).
        if abs(command) < self.physics.motor_deadband_command:
            return 0.0
        return command * self.physics.max_wheel_speed_mps * gain

    def step(self, left_command: float, right_command: float, dt_s: float) -> TrolleyState:
        """Advance the plant by ``dt_s`` seconds under the given commands."""
        physics = self.physics
        if dt_s <= 0.0:
            return self.state

        state = self.state
        state.commanded_left = max(-1.0, min(1.0, left_command))
        state.commanded_right = max(-1.0, min(1.0, right_command))

        target_left = self._command_to_target_speed(state.commanded_left, physics.left_gain)
        target_right = self._command_to_target_speed(state.commanded_right, physics.right_gain)

        # Discrete first-order lag: alpha = 1 - exp(-dt/tau), unconditionally
        # stable for any dt > 0 (an explicit Euler form would overshoot for
        # dt > 2*tau).
        tau = max(physics.motor_time_constant_s, 1e-4)
        alpha = 1.0 - math.exp(-dt_s / tau)

        state.left_wheel_mps += (target_left - state.left_wheel_mps) * alpha
        state.right_wheel_mps += (target_right - state.right_wheel_mps) * alpha

        if self._stepper is not None:
            # Drive the simulated wheels through the real STEP-rate conversion so
            # the step profile (including its acceleration limit) is the one the
            # firmware would produce.
            for axis, target in ((self._stepper.left, state.left_wheel_mps),
                                 (self._stepper.right, state.right_wheel_mps)):
                axis.set_velocity(target)
                axis.advance(dt_s)
            state.left_wheel_mps = self._stepper_wheel_speed(self._stepper.left)
            state.right_wheel_mps = self._stepper_wheel_speed(self._stepper.right)

        if physics.wheel_noise_std_mps > 0.0 and self.rng is not None:
            state.left_wheel_mps += self.rng.gauss(0.0, physics.wheel_noise_std_mps)
            state.right_wheel_mps += self.rng.gauss(0.0, physics.wheel_noise_std_mps)

        # Differential drive forward kinematics.
        v = 0.5 * (state.left_wheel_mps + state.right_wheel_mps)
        omega = (state.right_wheel_mps - state.left_wheel_mps) / physics.wheel_base_m
        self._omega_radps = omega

        # Midpoint (second-order Runge-Kutta) integration of the unicycle model:
        # exact for constant twist and far more accurate than Euler for the
        # turning arcs the trolley actually performs.
        theta_mid = state.theta_rad + 0.5 * omega * dt_s
        state.x_m += v * math.cos(theta_mid) * dt_s
        state.y_m += v * math.sin(theta_mid) * dt_s
        state.theta_rad = _normalize_angle(state.theta_rad + omega * dt_s)
        state.distance_travelled_m += abs(v) * dt_s
        return state

    def brake(self, dt_s: float) -> TrolleyState:
        """Coast to a stop with zero commands (used after E-stop / arrival)."""
        return self.step(0.0, 0.0, dt_s)

    def stop_immediately(self) -> None:
        """Hard stop: zero the wheel speeds without integrating motion.

        The firmware cuts motor power instantly on an emergency stop, so the
        simulated plant must be able to model that too.
        """
        self.state.left_wheel_mps = 0.0
        self.state.right_wheel_mps = 0.0
        self.state.commanded_left = 0.0
        self.state.commanded_right = 0.0
        self._omega_radps = 0.0


def _normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, +pi]."""
    wrapped = math.fmod(angle, 2.0 * math.pi)
    if wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    elif wrapped < -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def true_heading_deg(state: TrolleyState) -> float:
    return math.degrees(state.theta_rad)
