"""
Robotic arm pick-and-place example.

Demonstrates:
- Defining a 6-DOF arm using DH parameters
- Inverse kinematics solving
- Gripper control
- Simulation with objects

Run:
    stacc init my_arm --template=arm_6dof
    cd my_arm
    stacc simulate --duration=30
"""
import stacc
import numpy as np
from stacc.robot import SerialManipulator
from stacc.kinematics import DH, InverseKinematics
from stacc.gripper import ParallelGripper

# Define arm geometry
dh_params = [
    DH(a=0,     alpha=-np.pi/2, d=0.089, theta_offset=0),
    DH(a=0.425, alpha=0,        d=0,     theta_offset=-np.pi/2),
    DH(a=0.392, alpha=0,        d=0,     theta_offset=0),
    DH(a=0,     alpha=-np.pi/2, d=0.109, theta_offset=0),
    DH(a=0,     alpha=np.pi/2,  d=0.095, theta_offset=0),
    DH(a=0,     alpha=0,        d=0.082, theta_offset=0),
]

robot = SerialManipulator(name="picker", dh_parameters=dh_params)
gripper = ParallelGripper(max_width=0.08, force=20)
robot.attach_end_effector(gripper)

ik = InverseKinematics(robot, method="damped_least_squares")

@robot.on_start
def pick_and_place():
    # Move to pick position
    angles = ik.solve([0.4, 0.2, 0.05], orientation="top_down")
    robot.move_joints(angles, speed=0.5)
    
    gripper.close(force=15)
    stacc.wait(0.5)
    
    # Move to place position
    angles = ik.solve([0.4, -0.2, 0.15], orientation="top_down")
    robot.move_joints(angles, speed=0.3)
    
    gripper.open()

stacc.simulate(robot, duration=30, render=True,
               objects=["table", "cube_red", "cube_blue"])
