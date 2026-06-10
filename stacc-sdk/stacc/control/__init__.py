"""stacc.control — Controllers for robotics."""

from stacc.control.feedforward import Feedforward, FrictionModel, GravityCompensation
from stacc.control.filters import (
    ComplementaryFilter,
    KalmanFilter1D,
    LowPassFilter,
    MovingAverageFilter,
    NotchFilter,
)
from stacc.control.limiter import AccelerationLimiter, RateLimiter, SCurveProfile
from stacc.control.pid import CascadedPID, PIDController
from stacc.control.pipeline import ControllerPipeline, ControllerStage
from stacc.control.wrapper import (
    FilteredController,
    LimitedController,
    PIDWithFeedforward,
)

__all__ = [
    "PIDController",
    "CascadedPID",
    "Feedforward",
    "GravityCompensation",
    "FrictionModel",
    "PIDWithFeedforward",
    "FilteredController",
    "LimitedController",
    "ControllerPipeline",
    "ControllerStage",
    "LowPassFilter",
    "ComplementaryFilter",
    "MovingAverageFilter",
    "KalmanFilter1D",
    "NotchFilter",
    "RateLimiter",
    "AccelerationLimiter",
    "SCurveProfile",
]
