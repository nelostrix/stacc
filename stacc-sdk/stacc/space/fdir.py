"""STACC Space: Fault Detection, Isolation & Recovery (FDIR) and Redundancy Voting."""

import numpy as np
from typing import Dict, List, Any, Tuple
from dataclasses import dataclass


@dataclass
class FaultEvent:
    timestamp: float
    subsystem: str
    severity: str  # 'WARNING', 'CRITICAL', 'FATAL'
    description: str
    mitigation_action: str


class FDIRSystem:
    """
    Fault Detection, Isolation and Recovery (FDIR) system.
    Implements Triple Modular Redundancy (TMR) voting, motor stall recovery, and autonomous safe-mode fallback.
    """

    def __init__(self, torque_limit_N_m: float = 45.0, temp_limit_C: float = 75.0):
        self.torque_limit = torque_limit_N_m
        self.temp_limit = temp_limit_C
        self.system_state = "NOMINAL"  # NOMINAL, DEGRADED, SAFE_MODE, EMERGENCY_STOP
        self.fault_log: List[FaultEvent] = []

    def tmr_sensor_vote(self, sensor_a: float, sensor_b: float, sensor_c: float, tolerance: float = 0.05) -> Tuple[float, bool]:
        """
        Triple Modular Redundancy (TMR) Majority Voting.
        Detects anomalous sensor drift and isolates faulty channel.
        """
        readings = [sensor_a, sensor_b, sensor_c]
        # Check pairwise differences
        diff_ab = abs(sensor_a - sensor_b)
        diff_bc = abs(sensor_b - sensor_c)
        diff_ac = abs(sensor_a - sensor_c)

        if diff_ab <= tolerance and diff_bc <= tolerance:
            # All 3 agree
            return float(np.median(readings)), True
        elif diff_ab <= tolerance:
            # C is faulty outlier
            return (sensor_a + sensor_b) / 2.0, False
        elif diff_ac <= tolerance:
            # B is faulty outlier
            return (sensor_a + sensor_c) / 2.0, False
        elif diff_bc <= tolerance:
            # A is faulty outlier
            return (sensor_b + sensor_c) / 2.0, False
        else:
            # Inconclusive (divergence across all 3)
            self.system_state = "DEGRADED"
            return float(np.median(readings)), False

    def check_actuator_health(self, torques: np.ndarray, temperatures: np.ndarray) -> Dict[str, Any]:
        """Verify actuator torque limits and thermal envelopes."""
        max_torque = float(np.max(np.abs(torques)))
        max_temp = float(np.max(temperatures))

        has_overtorque = max_torque > self.torque_limit
        has_overtemp = max_temp > self.temp_limit

        if has_overtorque or has_overtemp:
            if max_torque > self.torque_limit * 1.25 or max_temp > self.temp_limit + 15.0:
                self.system_state = "SAFE_MODE"
                action = "SAFE_MODE: Motor current cut to 20%, brakes engaged."
            else:
                self.system_state = "DEGRADED"
                action = "DEGRADED: Slew-rate limiter tightened, velocity reduced 50%."
            
            return {
                "healthy": False,
                "system_state": self.system_state,
                "max_torque_N_m": round(max_torque, 1),
                "max_temp_C": round(max_temp, 1),
                "action": action,
            }

        return {
            "healthy": True,
            "system_state": self.system_state,
            "max_torque_N_m": round(max_torque, 1),
            "max_temp_C": round(max_temp, 1),
            "action": "NOMINAL: Closed-loop control active.",
        }
