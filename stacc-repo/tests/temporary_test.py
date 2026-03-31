# import stacc
# from stacc.core.link import Geometry, Link
# from stacc.core.robot_base import Robot

# robot = Robot(name="Falling_box")
# robot.fixed_base = False
# robot.add_link(Link("body", mass=1.0, visual=Geometry("box", size=[0.2, 0.2, 0.2])))

# stacc.simulate(robot, duration=50, render=True, physics="mujoco")

# import stacc
# from stacc.core.link import Geometry, Link
# from stacc.core.robot_base import Robot
# from stacc.sim.engine import SimulationConfig
# from stacc.sim.mjcf_builder import robot_to_mjcf

# robot = Robot(name="Falling_box")
# robot.fixed_base = False
# robot.add_link(Link("body", mass=1.0, visual=Geometry("box", size=[0.2, 0.2, 0.2])))

# config = SimulationConfig(gravity=[0, 0, -9.81], timestep=0.002)
# xml = robot_to_mjcf(robot, config)
# print(xml)
import stacc
from stacc.control.pid import PIDController
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot

robot = Robot(name="demo_arm")
robot.add_link(Link("base", mass=5.0, visual=Geometry("box", size=[0.2, 0.2, 0.1])))
robot.add_joint(Joint("shoulder", type="revolute", axis=[0, 1, 0], damping=2.0))
robot.add_link(
    Link("upper_arm", mass=1.0, visual=Geometry("box", size=[0.06, 0.06, 0.4]))
)
robot.add_joint(Joint("elbow", type="revolute", axis=[0, 1, 0], damping=1.0))
robot.add_link(
    Link("forearm", mass=0.5, visual=Geometry("box", size=[0.05, 0.05, 0.3]))
)

pid1 = PIDController(kp=200.0, kd=20.0)
pid2 = PIDController(kp=100.0, kd=10.0)


@robot.on_update
def control(dt):
    pos = robot.get_joint_positions()
    t1 = pid1.compute(0.5 - pos[0], dt)
    t2 = pid2.compute(-0.8 - pos[1], dt)
    robot.set_joint_torques([t1, t2])


stacc.simulate(robot, duration=15, render=True, physics="mujoco")
