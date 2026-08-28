"""Tests for the Robotics Intermediate Representation (stacc.core.rir).

Proves the core RIR bet: a Robot graph built any way (URDF, hand-written
connect() calls, serial chains) survives a full JSON round-trip and still
drives the same downstream consumers (here: joint/link/actuator data a
simulator or HAL layer would read).
"""
import json

from stacc.core.joint import Joint
from stacc.core.link import Link, Geometry, Inertia
from stacc.core.robot_base import Robot
from stacc.core.rir import Actuator, robot_to_rir, rir_from_dict, save_rir, load_rir


def _build_branching_arm() -> Robot:
    """A small tree (not a serial chain) — two shoulders off one torso.

    Deliberately not a humanoid-shaped hardcode: this exercises connect()
    branching, which is the thing RIR has to preserve losslessly.
    """
    robot = Robot(name="test_arm")

    torso = Link("torso", mass=5.0, inertia=Inertia.from_box(5.0, 0.3, 0.2, 0.4))
    l_arm = Link("l_upper_arm", mass=1.5, visual=Geometry(shape="cylinder", radius=0.03, length=0.25))
    r_arm = Link("r_upper_arm", mass=1.5, visual=Geometry(shape="cylinder", radius=0.03, length=0.25))
    l_fore = Link("l_forearm", mass=1.0)

    robot.add_link(torso)
    robot.add_link(l_arm)
    robot.add_link(r_arm)
    robot.add_link(l_fore)

    robot.connect("torso", "l_upper_arm", Joint("l_shoulder", type="revolute", axis=[1, 0, 0], limits=(-1.5, 1.5)))
    robot.connect("torso", "r_upper_arm", Joint("r_shoulder", type="revolute", axis=[1, 0, 0], limits=(-1.5, 1.5)))
    robot.connect("l_upper_arm", "l_forearm", Joint("l_elbow", type="revolute", limits=(0, 2.3)))

    return robot


def test_round_trip_preserves_topology():
    robot = _build_branching_arm()
    data = robot_to_rir(robot)
    rebuilt = rir_from_dict(data)

    assert rebuilt.num_links == robot.num_links
    assert rebuilt.num_joints == robot.num_joints
    assert rebuilt.dof == robot.dof
    assert {l.name for l in rebuilt.links} == {l.name for l in robot.links}
    assert {j.name for j in rebuilt.joints} == {j.name for j in robot.joints}


def test_round_trip_preserves_branching():
    robot = _build_branching_arm()
    rebuilt = rir_from_dict(robot_to_rir(robot))

    # l_upper_arm has one child (l_forearm); r_upper_arm has none — this is
    # exactly the shape a serial-chain-only representation can't express.
    l_children = {c.name for _, c, _ in rebuilt.get_all_children("l_upper_arm")}
    r_children = {c.name for _, c, _ in rebuilt.get_all_children("r_upper_arm")}
    assert l_children == {"l_forearm"}
    assert r_children == set()


def test_round_trip_preserves_joint_limits_and_kinematics():
    robot = _build_branching_arm()
    rebuilt = rir_from_dict(robot_to_rir(robot))

    original = {j.name: j for j in robot.joints}
    restored = {j.name: j for j in rebuilt.joints}
    for name, joint in original.items():
        r = restored[name]
        assert r.type == joint.type
        assert r.limits == joint.limits
        assert r.axis == joint.axis


def test_round_trip_preserves_mass_and_inertia():
    robot = _build_branching_arm()
    rebuilt = rir_from_dict(robot_to_rir(robot))

    torso = next(l for l in rebuilt.links if l.name == "torso")
    assert torso.mass == 5.0
    assert torso.inertia.ixx > 0


def test_actuators_round_trip():
    robot = _build_branching_arm()
    actuators = [
        Actuator(joint_name="l_shoulder", type="servo", interface="can", channel="j1"),
        Actuator(joint_name="r_shoulder", type="servo", interface="can", channel="j2"),
    ]
    data = robot_to_rir(robot, actuators=actuators)
    rebuilt = rir_from_dict(data)

    assert len(rebuilt.actuators) == 2
    assert {a.joint_name for a in rebuilt.actuators} == {"l_shoulder", "r_shoulder"}
    assert all(a.interface == "can" for a in rebuilt.actuators)


def test_rir_is_json_serializable():
    robot = _build_branching_arm()
    data = robot_to_rir(robot)
    # If this raises, RIR isn't actually a portable interchange format.
    text = json.dumps(data)
    assert json.loads(text)["name"] == "test_arm"


def test_save_and_load_rir_file(tmp_path):
    robot = _build_branching_arm()
    path = str(tmp_path / "test_arm.rir.json")
    save_rir(robot, path)
    rebuilt = load_rir(path)

    assert rebuilt.name == robot.name
    assert rebuilt.dof == robot.dof
