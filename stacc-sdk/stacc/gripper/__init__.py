"""Universal gripper/end-effector system.

Supports any end effector type: parallel jaw, vacuum, soft, magnetic,
dexterous multi-finger. Each gripper type defines open/close/grasp
behavior and integrates with the simulation via force control.

Usage:
    from stacc.gripper import ParallelGripper, VacuumGripper, SoftGripper

    gripper = ParallelGripper(max_width=0.08, max_force=20.0)
    robot.attach_end_effector(gripper)

    @robot.on_update
    def control(dt):
        gripper.close(force=15)
        if gripper.is_grasping:
            # object is held
            pass
"""

from stacc.gripper.base import Gripper
from stacc.gripper.parallel import ParallelGripper
from stacc.gripper.vacuum import VacuumGripper
from stacc.gripper.soft import SoftGripper
from stacc.gripper.magnetic import MagneticGripper
from stacc.gripper.dexterous import DexterousHand

__all__ = [
    "Gripper",
    "ParallelGripper",
    "VacuumGripper",
    "SoftGripper",
    "MagneticGripper",
    "DexterousHand",
]
