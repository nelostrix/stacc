"""stacc.robot — Robot type implementations."""
from stacc.robot.differential_drive import DifferentialDrive
from stacc.robot.serial_manipulator import SerialManipulator

__all__ = ["DifferentialDrive", "SerialManipulator"]
