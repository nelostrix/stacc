import stacc
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot
from stacc.sensors.imu import IMU
from stacc.sensors.lidar import LiDAR

robot = Robot(name="sensor_bot")
robot.add_link(Link("base", mass=3.0, visual=Geometry("box", size=[0.2, 0.2, 0.1])))
robot.add_joint(Joint("spin", type="revolute", axis=[0, 0, 1], damping=0.5))
robot.add_link(Link("arm", mass=1.0, visual=Geometry("box", size=[0.05, 0.05, 0.3])))

imu = IMU(rate=100)
lidar = LiDAR(channels=1, points_per_channel=36, range_max=10.0, fov_h=360)

robot.attach_sensor(imu, position="base")
robot.attach_sensor(lidar, position="top")


@robot.on_update
def control(dt):
    # Spin the arm
    robot.set_joint_torques([2.0])

    # Read sensors every 0.5 seconds
    if int(robot.time * 2) != int((robot.time - dt) * 2):
        imu_data = imu.read()
        lidar_data = lidar.read()
        print(
            f"  t={robot.time:.1f}s | "
            f"IMU accel=[{imu_data['acceleration'][0]:.2f}, "
            f"{imu_data['acceleration'][1]:.2f}, "
            f"{imu_data['acceleration'][2]:.2f}] | "
            f"LiDAR hits={lidar_data['num_points']}, "
            f"min_range={min(lidar_data['ranges']):.2f}m"
        )


stacc.simulate(robot, duration=10, render=True, physics="mujoco")
