"""stacc — The Universal Robotics Development Platform.

Build robots from line followers to Mars rovers.

Usage:
    import stacc
    from stacc.robot import DifferentialDrive
    stacc.simulate(robot, duration=30)
"""
__version__ = "0.1.0"

from stacc.sim.engine import simulate
import time
wait = time.sleep

__all__ = ["simulate", "wait", "__version__"]
