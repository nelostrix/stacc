"""Test: robot_to_mjcf generates valid MJCF that Mujoco can load."""

import test_mujoco_raw
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link

# After you write robot_to_mjcf, test it:
from stacc.core.robot_base import Robot
from stacc.sim.engine import SimulateConfig
from stacc.sim.mujoco_backend import robot_to_mjcf

robot = Robot(name="test_arm")
robot.add_link(Link("base", mass=2.0, visual=Geometry("box", size=[0.1, 0.1, 0.05])))
robot.add_link(Link("upper", mass=1.0, visual=Geometry("box", size=[0.04, 0.04, 0.3])))
robot.add_joint(Joint("shoulder", type="revolute", axis=[0, 1, 0]))
robot.add_link(Link("lower", mass=0.5, visual=Geometry("box", size=[0.03, 0.03, 0.25])))
robot.add_joint(Joint("elbow", type="revolute", axis=[0, 1, 0]))

config = SimulationConfig(gravity=[0, 0, -9.81], timestep=0.001)
mjcf_xml = robot_to_mjcf(robot, config)


print("Generated MJCF: ")
print(mjcf_xml)
print()


model = mujoco.MjModel.from_xml_string(mjcf_xml)
data = mujoco.MjData(model)
print(f"Model loaded: {model.nq} position, {model.nv} velocities, {model.nu} actuators")
print("SUCCESS: MJCF generation works")
