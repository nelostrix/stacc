"""
Minimal stacc example — a robot in 10 lines.

This demonstrates the simplest possible robot you can build with stacc.
A 2-wheel differential drive bot that follows a line using IR sensors.

Run:
    cd examples/
    stacc init minimal_bot --template=line_follower
    cd minimal_bot
    stacc simulate
"""
import stacc
from stacc.robot import DifferentialDrive
from stacc.sensors import IRArray
from stacc.control import PIDController

robot = DifferentialDrive(name="mini", wheel_radius=0.033, wheel_base=0.15)
ir = IRArray(count=5, spacing=0.01)
robot.attach_sensor(ir, position="front_bottom")
pid = PIDController(kp=1.0, ki=0.01, kd=0.5)

@robot.on_update
def loop(dt):
    correction = pid.compute(ir.line_position(), dt)
    robot.set_speed(left=0.5 - correction, right=0.5 + correction)

stacc.simulate(robot, duration=60, render=True)
