"""Tests for stacc core modules — robot, joints, links, kinematics, PID."""
import math
import pytest


class TestJoint:
    def test_revolute_defaults(self):
        from stacc.core.joint import Joint
        j = Joint("test", type="revolute")
        assert j.type == "revolute"
        assert j.limits == (-math.pi, math.pi)
        assert j.is_actuated

    def test_fixed_joint(self):
        from stacc.core.joint import Joint
        j = Joint("fixed", type="fixed")
        assert not j.is_actuated
        assert j.clamp(5.0) == 5.0  # fixed joints don't clamp

    def test_clamp(self):
        from stacc.core.joint import Joint
        j = Joint("j", type="revolute", limits=(-1.0, 1.0))
        assert j.clamp(0.5) == 0.5
        assert j.clamp(2.0) == 1.0
        assert j.clamp(-3.0) == -1.0

    def test_invalid_type(self):
        from stacc.core.joint import Joint
        with pytest.raises(ValueError):
            Joint("bad", type="nonexistent")

    def test_serialization(self):
        from stacc.core.joint import Joint
        j = Joint("j1", type="revolute", limits=(-1, 1))
        d = j.to_dict()
        assert d["name"] == "j1"
        assert d["type"] == "revolute"


class TestLink:
    def test_link_creation(self):
        from stacc.core.link import Link
        l = Link("base", mass=2.0)
        assert l.name == "base"
        assert l.mass == 2.0

    def test_inertia_box(self):
        from stacc.core.link import Inertia
        i = Inertia.from_box(mass=1.0, x=0.1, y=0.1, z=0.1)
        assert i.ixx > 0
        assert i.ixy == 0  # symmetric box

    def test_inertia_sphere(self):
        from stacc.core.link import Inertia
        i = Inertia.from_sphere(mass=1.0, radius=0.05)
        assert abs(i.ixx - i.iyy) < 1e-10  # sphere is symmetric


class TestRobot:
    def test_robot_creation(self):
        from stacc.core.robot_base import Robot
        from stacc.core.joint import Joint
        from stacc.core.link import Link

        r = Robot(name="test_bot")
        r.add_link(Link("base"))
        r.add_link(Link("arm"))
        r.add_joint(Joint("j1", type="revolute"))

        assert r.num_links == 2
        assert r.num_joints == 1
        assert r.dof == 1

    def test_callbacks(self):
        from stacc.core.robot_base import Robot
        r = Robot("cb_test")
        called = []

        @r.on_update
        def loop(dt):
            called.append("update")

        @r.on_start
        def start():
            called.append("start")

        r._startup()
        r._tick(0.01)
        assert "start" in called
        assert "update" in called


class TestTransforms:
    def test_rotation_identity(self):
        from stacc.kinematics.transforms import rot_x, rot_y, rot_z
        import numpy as np
        np.testing.assert_allclose(rot_x(0), np.eye(3), atol=1e-10)
        np.testing.assert_allclose(rot_y(0), np.eye(3), atol=1e-10)
        np.testing.assert_allclose(rot_z(0), np.eye(3), atol=1e-10)

    def test_quaternion_roundtrip(self):
        from stacc.kinematics.transforms import euler_to_quaternion, quaternion_to_euler
        roll, pitch, yaw = 0.1, 0.2, 0.3
        q = euler_to_quaternion(roll, pitch, yaw)
        r2, p2, y2 = quaternion_to_euler(*q)
        assert abs(roll - r2) < 1e-10
        assert abs(pitch - p2) < 1e-10
        assert abs(yaw - y2) < 1e-10

    def test_dh_identity(self):
        from stacc.kinematics.transforms import DH
        import numpy as np
        dh = DH(a=0, alpha=0, d=0, theta_offset=0)
        T = dh.transform(0)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-10)


class TestFK:
    def test_single_joint(self):
        from stacc.kinematics.transforms import DH
        from stacc.kinematics.forward import end_effector_position
        import numpy as np

        # Single link of length 1.0
        dh = [DH(a=1.0, alpha=0, d=0, theta_offset=0)]
        pos = end_effector_position(dh, [0.0])
        np.testing.assert_allclose(pos, [1.0, 0.0, 0.0], atol=1e-10)

        # Rotate 90 degrees
        pos90 = end_effector_position(dh, [math.pi / 2])
        np.testing.assert_allclose(pos90, [0.0, 1.0, 0.0], atol=1e-10)

    def test_two_joint_arm(self):
        from stacc.kinematics.transforms import DH
        from stacc.kinematics.forward import end_effector_position
        import numpy as np

        # Two links, each 1.0m long
        dh = [
            DH(a=1.0, alpha=0, d=0),
            DH(a=1.0, alpha=0, d=0),
        ]
        # Both at zero: end effector at (2, 0, 0)
        pos = end_effector_position(dh, [0.0, 0.0])
        np.testing.assert_allclose(pos, [2.0, 0.0, 0.0], atol=1e-10)


class TestIK:
    def test_ik_simple(self):
        from stacc.kinematics.transforms import DH
        from stacc.kinematics.inverse import InverseKinematics
        from stacc.kinematics.forward import end_effector_position
        import numpy as np

        dh = [DH(a=1.0, alpha=0, d=0), DH(a=1.0, alpha=0, d=0)]
        ik = InverseKinematics(dh_params=dh, tolerance=1e-3, max_iterations=200)

        target = [1.5, 0.5, 0.0]
        angles = ik.solve(target)
        result = end_effector_position(dh, angles)
        error = np.linalg.norm(np.array(target) - result)
        assert error < 0.01, f"IK error too large: {error}"


class TestPID:
    def test_proportional(self):
        from stacc.control.pid import PIDController
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.compute(1.0, dt=0.01)  # error = 1.0
        assert output == 2.0

    def test_integral(self):
        from stacc.control.pid import PIDController
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        pid.compute(1.0, dt=0.1)
        output = pid.compute(1.0, dt=0.1)
        assert abs(output - 0.2) < 1e-10  # 2 steps * 1.0 * 0.1 = 0.2

    def test_output_limits(self):
        from stacc.control.pid import PIDController
        pid = PIDController(kp=100.0, output_limits=(-5, 5))
        output = pid.compute(1.0, dt=0.01)
        assert output == 5.0

    def test_reset(self):
        from stacc.control.pid import PIDController
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.compute(1.0, dt=0.1)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error is None


class TestDifferentialDrive:
    def test_creation(self):
        from stacc.robot.differential_drive import DifferentialDrive
        bot = DifferentialDrive(name="test", wheel_radius=0.033, wheel_base=0.15)
        assert bot.dof == 2
        assert bot.num_links == 3

    def test_set_speed(self):
        from stacc.robot.differential_drive import DifferentialDrive
        bot = DifferentialDrive()
        bot.set_speed(left=0.5, right=0.5)
        assert bot.linear_velocity > 0
        assert abs(bot.angular_velocity) < 1e-10


class TestSerialManipulator:
    def test_creation(self):
        from stacc.robot.serial_manipulator import SerialManipulator
        from stacc.kinematics.transforms import DH
        dh = [DH(a=0.5, alpha=0, d=0), DH(a=0.5, alpha=0, d=0)]
        arm = SerialManipulator(name="test_arm", dh_parameters=dh)
        assert arm.n_joints == 2
        assert arm.dof == 2

    def test_fk(self):
        from stacc.robot.serial_manipulator import SerialManipulator
        from stacc.kinematics.transforms import DH
        import numpy as np
        dh = [DH(a=1.0, alpha=0, d=0), DH(a=1.0, alpha=0, d=0)]
        arm = SerialManipulator(dh_parameters=dh)
        pos = arm.end_effector_position()
        np.testing.assert_allclose(pos, [2.0, 0.0, 0.0], atol=1e-10)
