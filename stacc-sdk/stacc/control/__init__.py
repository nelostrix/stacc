"""stacc.control — Modern Robotics Control Suite."""

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
from stacc.control.mpc import LinearMPC
from stacc.control.impedance import CartesianImpedanceController
from stacc.control.operational_space import OperationalSpaceController
from stacc.control.lqr import lqr, LQRController, solve_discrete_are
from stacc.control.cbf import ControlBarrierFilter
from stacc.control.wbc import HierarchicalWBC, TaskObjective

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
    "LinearMPC",
    "CartesianImpedanceController",
    "OperationalSpaceController",
    "lqr",
    "LQRController",
    "solve_discrete_are",
    "ControlBarrierFilter",
    "HierarchicalWBC",
    "TaskObjective",
]
