"""Rotors from RSS: thrust actuators that make a drone actually fly in MuJoCo."""

import math
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stacc.rss import load_file  # noqa: E402

SDK = Path(__file__).resolve().parents[1]
G = 9.81


def _drone():
    return load_file(SDK / "examples" / "drone.rss")


def test_rotors_compile_to_thrusters_on_a_free_base():
    robot = _drone()
    assert robot.fixed_base is False
    assert robot.thrusters == ["rotor_front_left", "rotor_front_right", "rotor_rear_left", "rotor_rear_right"]
    spins = [robot._links_by_name[n].thruster["spin"] for n in robot.thrusters]
    assert spins == ["ccw", "cw", "cw", "ccw"]  # X-quad: diagonal pairs match
    assert all(robot._links_by_name[n].thruster["max_thrust"] == 6.0 for n in robot.thrusters)


def test_mjcf_has_one_site_actuator_per_rotor():
    from stacc.sim.engine import SimulationConfig
    from stacc.sim.mjcf_builder import robot_to_mjcf

    xml = robot_to_mjcf(_drone(), SimulationConfig(physics="mujoco"))
    assert xml.count('<site name="rotor_') == 4
    assert xml.count('ctrlrange="0 6.0"') == 4
    assert 'gear="0 0 1 0 0 -0.016"' in xml and 'gear="0 0 1 0 0 0.016"' in xml


def _fly(thrusts, seconds):
    """Hold rotor thrusts and return (z gain in m, yaw change in rad)."""
    from stacc.server.runtime import StaccRuntime

    robot = _drone()
    rt = StaccRuntime(physics="mujoco", hz=500.0)
    rt.set_robot(robot)
    rt.start()
    try:
        time.sleep(0.5)  # settle on the floor
        start = rt.snapshot()["pose"]
        rt.set_thrusts(thrusts)
        time.sleep(seconds)
        end = rt.snapshot()["pose"]
        return end[2] - start[2], math.atan2(math.sin(end[5] - start[5]), math.cos(end[5] - start[5]))
    finally:
        rt.stop()


def _hover_thrust():
    robot = _drone()
    return sum(link.mass for link in robot.links) * G / 4


def test_drone_climbs_above_hover_thrust_without_yawing():
    pytest.importorskip("mujoco")
    climb, yaw = _fly([_hover_thrust() * 1.3] * 4, 1.0)
    assert climb > 0.3  # ~0.3 g net upward for about a second
    assert abs(yaw) < math.radians(2)


def test_drone_stays_down_below_hover_thrust():
    pytest.importorskip("mujoco")
    climb, _ = _fly([_hover_thrust() * 0.7] * 4, 1.0)
    assert climb < 0.01


def test_stronger_ccw_pair_yaws_the_drone_clockwise():
    pytest.importorskip("mujoco")
    h = _hover_thrust() * 1.2
    # Front-left and rear-right spin counter-clockwise; their reaction torque is clockwise.
    _, yaw = _fly([h * 1.15, h * 0.85, h * 0.85, h * 1.15], 1.0)
    assert yaw < -math.radians(5)
