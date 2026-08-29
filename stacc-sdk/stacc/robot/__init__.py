"""stacc.robot — Comprehensive Robot Topologies and Kinematic Systems."""

from stacc.robot.differential_drive import DifferentialDrive
from stacc.robot.serial_manipulator import SerialManipulator
from stacc.robot.quadruped import QuadrupedRobot
from stacc.robot.humanoid import HumanoidRobot
from stacc.robot.drone import MultirotorDrone
from stacc.robot.mecanum import MecanumDrive
from stacc.robot.ackermann import AckermannVehicle
from stacc.robot.delta import DeltaRobot

__all__ = [
    "DifferentialDrive",
    "SerialManipulator",
    "QuadrupedRobot",
    "HumanoidRobot",
    "MultirotorDrone",
    "MecanumDrive",
    "AckermannVehicle",
    "DeltaRobot",
]
