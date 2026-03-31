import stacc
from stacc.control.pid import PIDController
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot

robot = Robot(name="demo_arm")
robot.add_link(Link("base", mass=2.0, visual=Geometry("box", size=[0.15, 0.15, 0.05])))
robot.add_joint(Joint("shoulder", type="revolute", axis=[0, 1, 0]))
robot.add_link(
    Link("upper_arm", mass=1.0, visual=Geometry("box", size=[0.06, 0.06, 0.4]))
)
robot.add_joint(Joint("elbow", type="revolute", axis=[0, 1, 0]))
robot.add_link(
    Link("forearm", mass=0.5, visual=Geometry("box", size=[0.05, 0.05, 0.3]))
)

pid1 = PIDController(kp=100.0, kd=10.0)
pid2 = PIDController(kp=50.0, kd=5.0)


@robot.on_update
def control(dt):
    pos = robot.get_joint_positions()
    t1 = pid1.compute(0.8 - pos[0], dt)
    t2 = pid2.compute(-1.2 - pos[1], dt)
    robot.set_joint_torques([t1, t2])


stacc.simulate(robot, duration=15, render=True, physics="mujoco")
