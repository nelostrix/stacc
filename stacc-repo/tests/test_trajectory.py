"""Test: smooth arm motion with quintic trajectory."""

import math

import stacc
from stacc.control.pid import PIDController
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot
from stacc.kinematics.trajectory import TrajectoryExecutor, WaypointPath

robot = Robot(name="smooth_arm")
robot.add_link(Link("base", mass=5.0, visual=Geometry("box", size=[0.2, 0.2, 0.1])))
robot.add_joint(Joint("shoulder", type="revolute", axis=[0, 1, 0], damping=2.0))
robot.add_link(Link("upper", mass=1.5, visual=Geometry("box", size=[0.06, 0.06, 0.35])))
robot.add_joint(Joint("elbow", type="revolute", axis=[0, 1, 0], damping=1.5))
robot.add_link(Link("lower", mass=1.0, visual=Geometry("box", size=[0.05, 0.05, 0.3])))

path = WaypointPath(
    waypoints=[[0, 0], [0.8, -1.2], [-0.5, 0.8], [0.3, -0.5], [0, 0]],
    durations=[2.0, 1.5, 2.0, 2.5],
    method="minjerk",
)

pids = [PIDController(kp=200.0, kd=20.0) for _ in range(2)]
executor = TrajectoryExecutor(robot, path, pids)


@robot.on_update
def control(dt):
    executor.step(dt)


stacc.simulate(robot, duration=10, render=True, physics="mujoco")
