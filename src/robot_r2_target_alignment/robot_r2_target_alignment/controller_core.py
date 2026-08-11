"""ROS-independent PID logic for horizontal image alignment."""

from __future__ import annotations

from dataclasses import dataclass
import math


def command_for_mode(lateral_velocity: float, test_mode: bool) -> float | None:
    """Return no command in test mode, otherwise the requested velocity."""
    if test_mode:
        return None
    if not math.isfinite(lateral_velocity):
        raise ValueError("lateral velocity must be finite")
    return lateral_velocity


@dataclass(frozen=True)
class PidConfig:
    """Validated lateral velocity controller settings."""

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    minimum_output: float
    invert_output: bool

    def validate(self) -> None:
        values = (
            self.kp,
            self.ki,
            self.kd,
            self.integral_limit,
            self.output_limit,
            self.minimum_output,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("PID configuration must contain finite values")
        if self.kp < 0.0 or self.ki < 0.0 or self.kd < 0.0:
            raise ValueError("PID gains must be non-negative")
        if self.integral_limit < 0.0:
            raise ValueError("control.integral_limit must be non-negative")
        if self.output_limit <= 0.0:
            raise ValueError("control.output_limit must be positive")
        if not 0.0 <= self.minimum_output <= self.output_limit:
            raise ValueError(
                "control.minimum_output must be between zero and output_limit"
            )


class LateralPid:
    """PID controller whose input is normalized horizontal image error."""

    def __init__(self, config: PidConfig) -> None:
        config.validate()
        self._config = config
        self.reset()

    def set_config(self, config: PidConfig) -> None:
        config.validate()
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._has_last_error = False

    def update(self, error: float, dt: float) -> float:
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0.0:
            return 0.0
        config = self._config
        self._integral += error * dt
        if config.integral_limit > 0.0:
            self._integral = max(
                -config.integral_limit,
                min(config.integral_limit, self._integral),
            )

        derivative = 0.0
        if self._has_last_error:
            derivative = (error - self._last_error) / dt
        output = (
            config.kp * error
            + config.ki * self._integral
            + config.kd * derivative
        )
        self._last_error = error
        self._has_last_error = True

        if config.invert_output:
            output = -output
        output = max(-config.output_limit, min(config.output_limit, output))
        if 0.0 < abs(output) < config.minimum_output:
            output = math.copysign(config.minimum_output, output)
        return output
