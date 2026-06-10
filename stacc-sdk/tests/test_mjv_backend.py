import stacc
from stacc.control.pid import PIDController
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot

robot = Robot(name="test_arm")
robot.add_link(Link("base", mass=2.0, visual=Geometry("box", size=[0.1, 0.1, 0.5])))
robot.add_joint(
    Joint("shoulder", type="revolute", axis=[0, 1, 0], limits=(-3.14, 3.14))
)
robot.add_link(
    Link("upper_arm", mass=1.0, visual=Geometry("box", size=[0.04, 0.04, 0.3]))
)
robot.add_joint(Joint("elbow", type="revolute", axis=[0, 1, 0], limits=(-2.0, 2.0)))
robot.add_link(
    Link("lower_arm", mass=0.5, visual=Geometry("box", size=[0.03, 0.03, 0.25]))
)
pid1 = PIDController(kp=50.0, kd=5.0)
pid2 = PIDController(kp=30.0, kd=3.0)


@robot.on_update
def control(dt):
    position = robot.get_joint_positions()

    t1 = pid1.compute(0.5 - position[0], dt)
    t2 = pid2.compute(-1.0 - position[1], dt)

    robot.set_joint_torques([t1, t2])


stacc.simulate(robot, duration=10, render=True, physics="mujoco")
