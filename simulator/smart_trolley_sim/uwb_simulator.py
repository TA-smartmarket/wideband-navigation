"""Simulated UWB positioning subsystem.

Models what the navigation core receives from the real positioning subsystem
(Yusuf's UWB + EKF): a processed ``(x, y, quality, valid)`` sample at a fixed
update rate, with Gaussian noise, dropouts, invalid samples, low quality,
position jumps and a frozen output.

Ground truth never reaches the navigation core: this module is the only bridge
between the plant and the controller.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .config import UwbDefaults


@dataclass
class UwbSample:
    """One measurement as it would arrive over the wire."""

    x_m: float
    y_m: float
    quality: float
    valid: bool
    timestamp_ms: int
    #: True when this sample was produced by a scheduled update (as opposed to
    #: being a repeat of the previous one).  Used for logging/metrics only.
    fresh: bool = True
    #: Name of the fault active when the sample was produced ("" when nominal).
    fault: str = ""


@dataclass
class UwbStatistics:
    """Counters used by the experiment metrics."""

    generated: int = 0
    delivered: int = 0
    dropped: int = 0
    invalid: int = 0
    low_quality: int = 0
    jumps: int = 0

    def as_dict(self) -> dict:
        return {
            "generated": self.generated,
            "delivered": self.delivered,
            "dropped": self.dropped,
            "invalid": self.invalid,
            "low_quality": self.low_quality,
            "jumps": self.jumps,
        }


@dataclass
class UwbFaultState:
    """Runtime fault switches, controlled by the scenario and by the keyboard."""

    noise_enabled: bool = True
    noise_std_m: float = 0.05
    dropout_enabled: bool = False
    dropout_probability: float = 0.0
    dropout_burst_s: float = 0.0
    invalid_enabled: bool = False
    invalid_probability: float = 0.0
    low_quality_enabled: bool = False
    frozen_enabled: bool = False
    jump_offset_m: tuple[float, float] = (0.0, 0.0)
    #: Minimum quality reported while the low-quality fault is active.
    low_quality_value: float = 0.35


class UwbSimulator:
    """Generates UWB-like measurements from the true pose at a fixed rate."""

    def __init__(self, config: UwbDefaults, seed: int = 12345):
        self.config = config
        self.rng = random.Random(seed)
        self.faults = UwbFaultState(
            noise_std_m=config.noise_std_m,
            dropout_probability=config.dropout_probability,
            invalid_probability=config.invalid_probability,
        )
        self.statistics = UwbStatistics()
        self._next_update_ms = 0
        self._dropout_until_ms = 0
        self._last_sample: UwbSample | None = None
        self._jump_offset = (0.0, 0.0)
        # EKF-equivalent state: the smoothed estimate published to navigation.
        self._estimate: tuple[float, float] | None = None

    # -- configuration -----------------------------------------------------
    @property
    def update_period_ms(self) -> float:
        rate = max(self.config.update_rate_hz, 0.1)
        return 1000.0 / rate

    def reset(self, now_ms: int = 0) -> None:
        self.statistics = UwbStatistics()
        self._next_update_ms = now_ms
        self._dropout_until_ms = 0
        self._last_sample = None
        self._jump_offset = (0.0, 0.0)
        self.faults.jump_offset_m = (0.0, 0.0)
        self._estimate = None

    # -- fault injection ---------------------------------------------------
    def set_noise(self, enabled: bool, std_m: float | None = None) -> None:
        self.faults.noise_enabled = enabled
        if std_m is not None:
            self.faults.noise_std_m = std_m

    def set_dropout(self, enabled: bool, probability: float | None = None,
                    burst_s: float | None = None) -> None:
        self.faults.dropout_enabled = enabled
        if probability is not None:
            self.faults.dropout_probability = probability
        if burst_s is not None:
            self.faults.dropout_burst_s = burst_s

    def trigger_dropout(self, duration_s: float, now_ms: int) -> None:
        """Force a dropout window of ``duration_s`` starting now."""
        self._dropout_until_ms = now_ms + int(duration_s * 1000.0)

    def set_invalid(self, enabled: bool, probability: float | None = None) -> None:
        self.faults.invalid_enabled = enabled
        if probability is not None:
            self.faults.invalid_probability = probability

    def set_low_quality(self, enabled: bool) -> None:
        self.faults.low_quality_enabled = enabled

    def set_frozen(self, enabled: bool) -> None:
        self.faults.frozen_enabled = enabled

    def trigger_jump(self, magnitude_m: float = 1.5) -> tuple[float, float]:
        """Shift the measured position by a fixed offset (1-2 m by default)."""
        angle = self.rng.uniform(0.0, 2.0 * math.pi)
        offset = (magnitude_m * math.cos(angle), magnitude_m * math.sin(angle))
        self._jump_offset = offset
        self.faults.jump_offset_m = offset
        self.statistics.jumps += 1
        return offset

    def clear_jump(self) -> None:
        self._jump_offset = (0.0, 0.0)
        self.faults.jump_offset_m = (0.0, 0.0)

    def clear_all_faults(self) -> None:
        self.set_noise(True)
        self.set_dropout(False, probability=0.0)
        self.set_invalid(False, probability=0.0)
        self.set_low_quality(False)
        self.set_frozen(False)
        self.clear_jump()
        self._dropout_until_ms = 0

    # -- measurement generation -------------------------------------------
    def poll(self, true_x_m: float, true_y_m: float, now_ms: int) -> UwbSample | None:
        """Return a new sample when the update interval elapsed, else None.

        Returning ``None`` models the normal case of "no new UWB frame yet";
        navigation must keep using the last accepted position until the timeout.
        """
        if now_ms < self._next_update_ms:
            return None
        self._next_update_ms = now_ms + self.update_period_ms

        faults = self.faults
        self.statistics.generated += 1

        # 1. Dropout: no sample at all (either a forced window or a probabilistic
        #    loss).  The navigation core sees the position age out.
        in_forced_dropout = now_ms < self._dropout_until_ms
        probabilistic_dropout = (
            faults.dropout_enabled and self.rng.random() < faults.dropout_probability
        )
        if in_forced_dropout or probabilistic_dropout:
            self.statistics.dropped += 1
            if faults.dropout_burst_s > 0.0 and probabilistic_dropout and not in_forced_dropout:
                self.trigger_dropout(faults.dropout_burst_s, now_ms)
            return None

        # 2. Frozen output: the sensor repeats its previous value while the
        #    trolley keeps moving.  Navigation must not drive on stale data.
        if faults.frozen_enabled and self._last_sample is not None:
            frozen = UwbSample(
                x_m=self._last_sample.x_m,
                y_m=self._last_sample.y_m,
                quality=self._last_sample.quality,
                valid=True,
                timestamp_ms=now_ms,
                fresh=False,
                fault="frozen",
            )
            self._last_sample = frozen
            self.statistics.delivered += 1
            return frozen

        # 3. Noise model: independent Gaussian error per axis.
        noise_x = 0.0
        noise_y = 0.0
        if faults.noise_enabled and faults.noise_std_m > 0.0:
            noise_x = self.rng.gauss(0.0, faults.noise_std_m)
            noise_y = self.rng.gauss(0.0, faults.noise_std_m)

        # The positioning subsystem publishes an *estimate*: smooth the raw
        # ranging noise the way the EKF does.  The estimate is tracked on the
        # noise-free position, then the jump offset is applied on top as a step,
        # because a position jump is an estimator divergence - not something a
        # filter is supposed to hide, and navigation must react to it.
        raw_x = true_x_m + noise_x
        raw_y = true_y_m + noise_y
        alpha = self.effective_filter_alpha()
        if alpha >= 1.0 or self._estimate is None:
            self._estimate = (raw_x, raw_y)
        else:
            alpha = max(0.01, min(1.0, alpha))
            prev_x, prev_y = self._estimate
            self._estimate = (alpha * raw_x + (1.0 - alpha) * prev_x,
                              alpha * raw_y + (1.0 - alpha) * prev_y)
        measured_x = self._estimate[0] + self._jump_offset[0]
        measured_y = self._estimate[1] + self._jump_offset[1]

        # 4. Quality model: degrades with noise level and can be forced low.
        quality = self.config.quality_base
        if faults.noise_enabled and faults.noise_std_m > 0.0:
            quality -= self.config.quality_noise_scale * faults.noise_std_m
        quality -= abs(self.rng.gauss(0.0, self.config.quality_noise))
        fault_name = ""
        if self._jump_offset != (0.0, 0.0):
            fault_name = "position_jump"
        if faults.low_quality_enabled:
            quality = min(quality, faults.low_quality_value)
            fault_name = "low_quality"
        quality = max(0.0, min(1.0, quality))

        # 5. Invalid samples: the positioning subsystem flags the measurement as
        #    unusable (e.g. insufficient anchors in view).
        valid = True
        if faults.invalid_enabled and self.rng.random() < faults.invalid_probability:
            valid = False
            fault_name = "invalid"
            self.statistics.invalid += 1
        if faults.low_quality_enabled:
            self.statistics.low_quality += 1

        sample = UwbSample(
            x_m=measured_x,
            y_m=measured_y,
            quality=quality,
            valid=valid,
            timestamp_ms=now_ms,
            fresh=True,
            fault=fault_name,
        )
        self._last_sample = sample
        self.statistics.delivered += 1
        return sample

    def effective_filter_alpha(self) -> float:
        """Smoothing gain for the current noise level (1.0 = no smoothing).

        Mirrors a Kalman gain: the noisier the sensor, the more the estimate
        leans on its previous value.  A clean sensor keeps the filter open so the
        estimate does not lag behind the trolley during turns.
        """
        config = self.config
        base = config.filter_alpha
        if not self.faults.noise_enabled or self.faults.noise_std_m <= 0.0:
            return 1.0  # nothing to filter: emit the raw value
        reference = max(config.filter_alpha_reference_sigma, 1e-6)
        span = max(1.0 - config.filter_alpha_min, 1e-6)
        alpha = 1.0 - span * min(self.faults.noise_std_m / reference, 1.0)
        return max(config.filter_alpha_min, min(base, alpha))

    # -- introspection -----------------------------------------------------
    @property
    def last_sample(self) -> UwbSample | None:
        return self._last_sample

    def is_dropout_active(self, now_ms: int) -> bool:
        return now_ms < self._dropout_until_ms

    def active_fault_names(self, now_ms: int) -> list[str]:
        """Human readable list of active faults (for the on-screen overlay)."""
        names = []
        if self.is_dropout_active(now_ms):
            names.append("dropout(forced)")
        elif self.faults.dropout_enabled:
            names.append("dropout")
        if self.faults.noise_enabled and self.faults.noise_std_m > 0.0:
            names.append(f"noise {self.faults.noise_std_m:.2f} m")
        if self.faults.invalid_enabled:
            names.append("invalid")
        if self.faults.low_quality_enabled:
            names.append("low quality")
        if self.faults.frozen_enabled:
            names.append("frozen")
        if self._jump_offset != (0.0, 0.0):
            names.append("position jump")
        return names
