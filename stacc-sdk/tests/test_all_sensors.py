"""Universal sensor test — tests every sensor in stacc.

Run: python tests/test_all_sensors.py

Reports pass/fail for each sensor in both standalone and MuJoCo modes.
"""

import sys
import traceback

# ── Collect results ──
results = []


def _run_legacy_check(name, func):
    """Run a test function, catch errors, report result."""
    if __name__ != "__main__":
        return
    try:
        func()
        results.append((name, "PASS", ""))
        print(f"  \033[92m✓\033[0m {name}")
    except Exception as e:
        results.append((name, "FAIL", str(e)))
        print(f"  \033[91m✗\033[0m {name}: {e}")


print("\n" + "=" * 60)
print("  stacc sensor test suite")
print("=" * 60 + "\n")


# ══════════════════════════════════════════
# TEST: BaseSensor
# ══════════════════════════════════════════


def test_base_sensor():
    from stacc.sensors.base import BaseSensor

    s = BaseSensor(name="test", rate=100.0, noise_stddev=0.1)
    assert s.name == "test"
    assert s.rate == 100.0
    assert abs(s.period - 0.01) < 1e-6
    noisy = s.add_noise(1.0)
    assert isinstance(noisy, float)
    vec = s.add_noise_vector([1.0, 2.0, 3.0])
    assert len(vec) == 3


_run_legacy_check("BaseSensor", test_base_sensor)


# ══════════════════════════════════════════
# TEST: IMU
# ══════════════════════════════════════════


def test_imu():
    from stacc.sensors.imu import IMU

    imu = IMU(rate=400)
    assert imu.rate == 400
    data = imu.read()
    assert "acceleration" in data
    assert "angular_velocity" in data or "orientation" in data
    assert len(data["acceleration"]) == 3
    accel = imu.read_acceleration()
    assert len(accel) == 3


_run_legacy_check("IMU", test_imu)


# ══════════════════════════════════════════
# TEST: IRArray
# ══════════════════════════════════════════


def test_ir_array():
    from stacc.sensors.ir import IRArray

    ir = IRArray(count=5, spacing=0.01)
    assert ir.count == 5
    data = ir.read()
    assert len(data) == 5
    pos = ir.line_position()
    assert -1.0 <= pos <= 1.0


_run_legacy_check("IRArray", test_ir_array)


# ══════════════════════════════════════════
# TEST: Encoder
# ══════════════════════════════════════════


def test_encoder():
    from stacc.sensors.encoder import Encoder

    enc = Encoder(ticks_per_rev=1024)
    assert enc.ticks_per_rev == 1024
    data = enc.read()
    assert "angle" in data
    assert "velocity" in data
    assert "ticks" in data


_run_legacy_check("Encoder", test_encoder)


# ══════════════════════════════════════════
# TEST: LiDAR
# ══════════════════════════════════════════


def test_lidar():
    from stacc.sensors.lidar import LiDAR

    lidar = LiDAR(channels=1, points_per_channel=36, range_max=10.0, fov_h=360)
    assert lidar.num_rays == 36
    data = lidar.read()
    assert "ranges" in data
    assert "points" in data
    assert len(data["ranges"]) == 36
    ranges = lidar.get_ranges()
    assert len(ranges) == 36
    assert all(r >= 0 for r in ranges)


_run_legacy_check("LiDAR (standalone)", test_lidar)


def test_lidar_3d():
    from stacc.sensors.lidar import LiDAR

    lidar = LiDAR(
        channels=16, points_per_channel=90, range_max=30.0, fov_h=360, fov_v=30
    )
    assert lidar.num_rays == 16 * 90
    data = lidar.read()
    assert len(data["ranges"]) == 16 * 90


_run_legacy_check("LiDAR 3D (16ch)", test_lidar_3d)


# ══════════════════════════════════════════
# TEST: Camera
# ══════════════════════════════════════════


def test_camera():
    from stacc.sensors.camera import Camera

    cam = Camera(width=320, height=240, fov=60)
    assert cam.resolution == (320, 240)
    data = cam.read()
    assert "image" in data
    assert data["width"] == 320
    assert data["height"] == 240
    img = cam.get_image()
    # Should be numpy array or None if numpy not available
    if img is not None:
        assert img.shape == (240, 320, 3)


_run_legacy_check("Camera (standalone)", test_camera)


# ══════════════════════════════════════════
# TEST: DepthCamera
# ══════════════════════════════════════════


def test_depth_camera():
    from stacc.sensors.depth_camera import DepthCamera

    cam = DepthCamera(width=160, height=120, range_max=10.0)
    data = cam.read()
    assert "rgb" in data
    assert "depth" in data
    rgb = cam.get_rgb()
    depth = cam.get_depth()
    if rgb is not None:
        assert rgb.shape == (120, 160, 3)
    if depth is not None:
        assert depth.shape == (120, 160)


_run_legacy_check("DepthCamera (standalone)", test_depth_camera)


# ══════════════════════════════════════════
# TEST: StereoCamera
# ══════════════════════════════════════════


def test_stereo():
    from stacc.sensors.stereo_camera import StereoCamera

    stereo = StereoCamera(baseline=0.12, width=320, height=240)
    assert stereo.baseline == 0.12
    data = stereo.read()
    assert "left" in data
    assert "right" in data
    assert "baseline" in data


_run_legacy_check("StereoCamera", test_stereo)


# ══════════════════════════════════════════
# TEST: ForceTorqueSensor
# ══════════════════════════════════════════


def test_force_torque():
    from stacc.sensors.force_torque import ForceTorqueSensor

    ft = ForceTorqueSensor(range_force=100, range_torque=10)
    data = ft.read()
    assert "force" in data
    assert "torque" in data
    assert "wrench" in data
    assert len(data["force"]) == 3
    assert len(data["torque"]) == 3
    assert len(data["wrench"]) == 6
    f = ft.get_force()
    assert len(f) == 3


_run_legacy_check("ForceTorqueSensor", test_force_torque)


# ══════════════════════════════════════════
# TEST: GPS
# ══════════════════════════════════════════


def test_gps():
    from stacc.sensors.gps import GPS

    gps = GPS(origin_lat=26.8, origin_lon=80.9, rate=10)
    data = gps.read()
    assert "latitude" in data
    assert "longitude" in data
    assert "altitude" in data
    assert abs(data["latitude"] - 26.8) < 1.0  # should be near origin
    assert abs(data["longitude"] - 80.9) < 1.0


_run_legacy_check("GPS", test_gps)


# ══════════════════════════════════════════
# TEST: Ultrasonic
# ══════════════════════════════════════════


def test_ultrasonic():
    from stacc.sensors.ultrasonic import Ultrasonic

    us = Ultrasonic(range_max=4.0, range_min=0.02)
    data = us.read()
    assert "distance" in data
    assert "valid" in data
    dist = us.get_distance()
    assert isinstance(dist, float)


_run_legacy_check("Ultrasonic", test_ultrasonic)


# ══════════════════════════════════════════
# TEST: ContactSensor
# ══════════════════════════════════════════


def test_contact():
    from stacc.sensors.contact import ContactSensor

    cs = ContactSensor(threshold=0.1)
    data = cs.read()
    assert "in_contact" in data
    assert "force" in data
    assert cs.is_touching == False  # no force applied


_run_legacy_check("ContactSensor", test_contact)


# ══════════════════════════════════════════
# TEST: Barometer
# ══════════════════════════════════════════


def test_barometer():
    from stacc.sensors.barometer import Barometer

    baro = Barometer(rate=50)
    data = baro.read()
    assert "pressure" in data
    assert "altitude_estimate" in data
    assert "temperature" in data
    # At sea level, pressure should be ~101325 Pa
    assert 90000 < data["pressure"] < 110000


_run_legacy_check("Barometer", test_barometer)


# ══════════════════════════════════════════
# TEST: TemperatureSensor
# ══════════════════════════════════════════


def test_temperature():
    from stacc.sensors.temperature import TemperatureSensor

    temp = TemperatureSensor()
    data = temp.read()
    assert "temperature" in data
    assert "unit" in data
    assert data["unit"] == "celsius"
    assert -50 < data["temperature"] < 150


_run_legacy_check("TemperatureSensor", test_temperature)


# ══════════════════════════════════════════
# TEST: LiDAR with MuJoCo raycasting
# ══════════════════════════════════════════


def test_lidar_mujoco():
    import mujoco
    import numpy as np

    from stacc.sensors.lidar import LiDAR

    # Create a scene with a box obstacle
    xml = """
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <geom type="plane" size="5 5 0.1"/>
        <body name="obstacle" pos="2 0 0.5">
          <geom type="box" size="0.5 0.5 0.5" rgba="1 0 0 1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    lidar = LiDAR(channels=1, points_per_channel=72, range_max=10.0, fov_h=360)
    lidar.set_mujoco_backend(model, data)
    result = lidar.read()

    # Should detect the box in some directions
    ranges = result["ranges"]
    assert len(ranges) == 72
    min_range = min(ranges)
    # At least one ray should hit the box (closer than max range)
    assert min_range < 10.0, f"LiDAR didn't detect obstacle: min range = {min_range}"


_run_legacy_check("LiDAR + MuJoCo raycasting", test_lidar_mujoco)


# ══════════════════════════════════════════
# TEST: Camera with MuJoCo rendering
# ══════════════════════════════════════════


def test_camera_mujoco():
    import mujoco

    from stacc.sensors.camera import Camera

    xml = """
    <mujoco>
      <worldbody>
        <light pos="0 0 3"/>
        <geom type="plane" size="5 5 0.1"/>
        <body pos="1 0 0.5">
          <geom type="sphere" size="0.3" rgba="1 0 0 1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam = Camera(width=320, height=240)
    cam.set_mujoco_backend(model, data)
    result = cam.read()

    img = result["image"]
    assert img is not None
    assert img.shape == (240, 320, 3)
    # Image should not be all black (scene has objects)
    assert img.max() > 0, "Camera rendered all-black image"


_run_legacy_check("Camera + MuJoCo rendering", test_camera_mujoco)


# ══════════════════════════════════════════
# TEST: Full robot with sensors attached
# ══════════════════════════════════════════


def test_robot_with_sensors():
    from stacc.core.joint import Joint
    from stacc.core.link import Geometry, Link
    from stacc.core.robot_base import Robot
    from stacc.sensors.gps import GPS
    from stacc.sensors.imu import IMU
    from stacc.sensors.lidar import LiDAR

    robot = Robot(name="sensor_bot")
    robot.add_link(Link("base", mass=2.0, visual=Geometry("box", size=[0.2, 0.2, 0.1])))

    imu = IMU(rate=100)
    lidar = LiDAR(channels=1, points_per_channel=36)
    gps = GPS(origin_lat=26.8, origin_lon=80.9)

    robot.attach_sensor(imu, position="body_center")
    robot.attach_sensor(lidar, position="top")
    robot.attach_sensor(gps, position="top")

    assert len(robot.sensors) == 3
    assert robot.sensors[0]._mount_position == "body_center"
    assert robot.sensors[1]._mount_position == "top"

    # All sensors should be readable
    for sensor in robot.sensors:
        data = sensor.read()
        assert isinstance(data, dict)


_run_legacy_check("Robot with multiple sensors", test_robot_with_sensors)


# ══════════════════════════════════════════
# SUMMARY
# ══════════════════════════════════════════

print("\n" + "=" * 60)
passed = sum(1 for _, status, _ in results if status == "PASS")
failed = sum(1 for _, status, _ in results if status == "FAIL")
total = len(results)

if failed == 0:
    print(f"  \033[92mALL {total} TESTS PASSED\033[0m")
else:
    print(
        f"  \033[92m{passed} passed\033[0m, \033[91m{failed} failed\033[0m out of {total}"
    )
    print()
    for name, status, err in results:
        if status == "FAIL":
            print(f"  \033[91m✗ {name}\033[0m")
            print(f"    {err}")

print("=" * 60 + "\n")
if __name__ == "__main__":
    sys.exit(1 if failed > 0 else 0)
