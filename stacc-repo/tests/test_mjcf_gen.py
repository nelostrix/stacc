import mujoco

from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot
from stacc.sim.engine import SimulationConfig
from stacc.sim.mjcf_builder import robot_to_mjcf

robot = Robot(name="Test_arm_001")
robot.add_link(Link("base", mass=2.0, visual=Geometry("box", size=[0.1, 0.1, 0.05])))
robot.add_joint(Joint("shoulder", type="revolute", axis=[0, 1, 0]))
robot.add_link(
    Link("upper_arm", mass=1.0, visual=Geometry("box", size=[0.04, 0.04, 0.3]))
)
robot.add_joint(Joint("elbow", type="revolute", axis=[0, 1, 0]))
robot.add_link(
    Link("lower_arm", mass=0.5, visual=Geometry("box", size=[0.03, 0.03, 0.25]))
)


config = SimulationConfig(gravity=[0, 0, -9.81], timestep=0.002)
xml = robot_to_mjcf(robot, config)

print("Generated MJCF:")
print(xml)
print()

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
print(f"loaded: {model.nq} positions, {model.nv} velocities, {model.nu} actuators")

# quick physics test
data.qpos[0] = 0.5
mujoco.mj_forward(model, data)
for i in range(500):
    mujoco.mj_step(model, data)
print(f"After 500 steps: joint0={data.qpos[0]:.4f} joint1={data.qpos[1]:.4f}")
print("SUCCESS")
