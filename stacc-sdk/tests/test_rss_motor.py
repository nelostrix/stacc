"""Motor model from RSS: effort (stall torque) + speed (no-load speed) on a limb
makes the joint a DC motor whose torque falls with speed."""

import math
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stacc.rss import parse, compile_document  # noqa: E402

ROVER = """
robot rover {
    chassis { shape: differential-drive; dimensions: [400mm, 300mm, 120mm]; mass: 4kg; }
    wheel front_left  { radius: 60mm; width: 30mm; mass: 0.3kg; MOTOR }
    wheel front_right { radius: 60mm; width: 30mm; mass: 0.3kg; MOTOR }
    wheel rear_left   { radius: 60mm; width: 30mm; mass: 0.3kg; MOTOR }
    wheel rear_right  { radius: 60mm; width: 30mm; mass: 0.3kg; MOTOR }
}
"""


def _rover(motor: str):
    return compile_document(parse(ROVER.replace("MOTOR", motor)))


def test_units_and_joint_properties():
    robot = compile_document(parse(
        "robot a { body { shape: box; } arm main { joints: 2; effort: 2.5; speed: 60rpm; "
        "friction: 0.1; limits: [-90deg, 45deg]; } }"
    ))
    for joint in robot.joints:
        assert joint.max_effort == pytest.approx(2.5)
        assert joint.max_velocity == pytest.approx(2 * math.pi)  # 60 rpm
        assert joint.damping == pytest.approx(2.5 / (2 * math.pi))  # back-EMF
        assert joint.friction == pytest.approx(0.1)
        assert joint.limits == pytest.approx((-math.pi / 2, math.pi / 4))
    assert compile_document(parse("robot b { body { shape: box; } arm m { speed: 180deg/s; } }")).joints[0].max_velocity == pytest.approx(math.pi)


def test_speed_without_effort_warns_once_per_limb():
    robot = compile_document(parse("robot a { body { shape: box; } arm main { joints: 3; speed: 30rpm; } }"))
    assert sum("speed" in w for w in robot.rss_warnings) == 1
    assert all(j.damping == 0 for j in robot.joints)


def test_mjcf_carries_damping_friction_and_force_limit():
    from stacc.sim.engine import SimulationConfig
    from stacc.sim.mjcf_builder import robot_to_mjcf

    xml = robot_to_mjcf(_rover("effort: 1.2; speed: 300rpm; friction: 0.02;"), SimulationConfig(physics="mujoco"))
    assert 'frictionloss="0.02"' in xml
    assert 'forcerange="-1.2 1.2"' in xml
    assert f'damping="{1.2 / (300 * 2 * math.pi / 60)}"' in xml


def _drive(robot, torque: float, seconds: float) -> float:
    """Apply equal wheel torque and return the rover's forward speed (m/s)."""
    from stacc.server.runtime import StaccRuntime

    rt = StaccRuntime(physics="mujoco", hz=500.0)
    rt.set_robot(robot)
    rt.start()
    try:
        time.sleep(0.5)
        rt.set_joint_torques([torque] * robot.dof)
        time.sleep(seconds)
        x0, t0 = rt.snapshot()["pose"][0], rt.snapshot()["sim_time"]
        time.sleep(0.5)
        x1, t1 = rt.snapshot()["pose"][0], rt.snapshot()["sim_time"]
        return (x1 - x0) / (t1 - t0)
    finally:
        rt.stop()


def test_rover_reaches_motor_top_speed_instead_of_accelerating_forever():
    pytest.importorskip("mujoco")
    robot = _rover("effort: 1.2; speed: 300rpm;")
    speed = _drive(robot, torque=1.0, seconds=3.0)
    # DC motor: ω = τ / (effort/speed) = 1.0 / (1.2 / 31.42) ≈ 26.2 rad/s -> 1.57 m/s on 60 mm wheels.
    assert speed == pytest.approx(1.57, rel=0.15)
