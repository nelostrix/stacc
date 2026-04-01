"""Full humanoid — torso, 2 arms, 2 legs, head. 14 joints, all PID controlled."""

import math

import stacc
from stacc.control.pid import PIDController
from stacc.core.joint import Joint
from stacc.core.link import Geometry, Link
from stacc.core.robot_base import Robot

robot = Robot(name="humanoid")

# ══════════════════════════════════════
# BODY STRUCTURE — define all links first
# ══════════════════════════════════════

# Spine (serial chain: pelvis → torso → upper_chest → head)
robot.add_link(
    Link("pelvis", mass=8.0, visual=Geometry("box", size=[0.28, 0.18, 0.12]))
)
robot.add_link(Link("torso", mass=6.0, visual=Geometry("box", size=[0.24, 0.16, 0.30])))
robot.add_link(
    Link("upper_chest", mass=4.0, visual=Geometry("box", size=[0.30, 0.18, 0.20]))
)
robot.add_link(Link("head", mass=2.0, visual=Geometry("sphere", radius=0.10)))

# Arms
robot.add_link(
    Link("l_upper_arm", mass=1.5, visual=Geometry("box", size=[0.06, 0.06, 0.28]))
)
robot.add_link(
    Link("l_forearm", mass=1.0, visual=Geometry("box", size=[0.05, 0.05, 0.24]))
)
robot.add_link(
    Link("r_upper_arm", mass=1.5, visual=Geometry("box", size=[0.06, 0.06, 0.28]))
)
robot.add_link(
    Link("r_forearm", mass=1.0, visual=Geometry("box", size=[0.05, 0.05, 0.24]))
)

# Legs
robot.add_link(
    Link("l_thigh", mass=3.0, visual=Geometry("box", size=[0.08, 0.08, 0.35]))
)
robot.add_link(
    Link("l_shin", mass=2.0, visual=Geometry("box", size=[0.06, 0.06, 0.32]))
)
robot.add_link(
    Link("r_thigh", mass=3.0, visual=Geometry("box", size=[0.08, 0.08, 0.35]))
)
robot.add_link(
    Link("r_shin", mass=2.0, visual=Geometry("box", size=[0.06, 0.06, 0.32]))
)

# ══════════════════════════════════════
# CONNECTIONS — define the topology
# ══════════════════════════════════════

# Spine (vertical chain)
robot.connect(
    "pelvis", "torso", Joint("waist", type="revolute", axis=[0, 1, 0], damping=5.0)
)

robot.connect(
    "torso", "upper_chest", Joint("chest", type="revolute", axis=[0, 1, 0], damping=3.0)
)

robot.connect(
    "upper_chest", "head", Joint("neck", type="revolute", axis=[0, 1, 0], damping=1.0)
)

# Left arm — attached to side of upper_chest
robot.connect(
    "upper_chest",
    "l_upper_arm",
    Joint("l_shoulder", type="revolute", axis=[1, 0, 0], damping=2.0),
    offset=[-0.20, 0, 0.08],
)

robot.connect(
    "l_upper_arm",
    "l_forearm",
    Joint("l_elbow", type="revolute", axis=[0, 1, 0], damping=1.0, limits=(0, 2.5)),
)

# Right arm — attached to other side
robot.connect(
    "upper_chest",
    "r_upper_arm",
    Joint("r_shoulder", type="revolute", axis=[1, 0, 0], damping=2.0),
    offset=[0.20, 0, 0.08],
)

robot.connect(
    "r_upper_arm",
    "r_forearm",
    Joint("r_elbow", type="revolute", axis=[0, 1, 0], damping=1.0, limits=(0, 2.5)),
)

# Left leg — attached to bottom of pelvis
robot.connect(
    "pelvis",
    "l_thigh",
    Joint("l_hip", type="revolute", axis=[0, 1, 0], damping=4.0),
    offset=[-0.10, 0, -0.06],
)

robot.connect(
    "l_thigh",
    "l_shin",
    Joint("l_knee", type="revolute", axis=[0, 1, 0], damping=3.0, limits=(0, 2.5)),
)

# Right leg
robot.connect(
    "pelvis",
    "r_thigh",
    Joint("r_hip", type="revolute", axis=[0, 1, 0], damping=4.0),
    offset=[0.10, 0, -0.06],
)

robot.connect(
    "r_thigh",
    "r_shin",
    Joint("r_knee", type="revolute", axis=[0, 1, 0], damping=3.0, limits=(0, 2.5)),
)


# ══════════════════════════════════════
# CONTROL — 11 PID controllers
# ══════════════════════════════════════

joint_names = [
    "waist",
    "chest",
    "neck",
    "l_shoulder",
    "l_elbow",
    "r_shoulder",
    "r_elbow",
    "l_hip",
    "l_knee",
    "r_hip",
    "r_knee",
]

pids = {name: PIDController(kp=300.0, kd=30.0) for name in joint_names}


@robot.on_update
def control(dt):
    pos = robot.get_joint_positions()
    t = robot.time

    targets = {
        "waist": 0.15 * math.sin(t * 0.5),
        "chest": 0.10 * math.sin(t * 0.7),
        "neck": 0.25 * math.sin(t * 1.0),
        "l_shoulder": 0.6 * math.sin(t * 1.2),
        "l_elbow": 1.0 + 0.5 * math.sin(t * 1.5),
        "r_shoulder": 0.6 * math.sin(t * 1.2 + math.pi),
        "r_elbow": 1.0 + 0.5 * math.sin(t * 1.5 + math.pi),
        "l_hip": 0.3 * math.sin(t * 0.8),
        "l_knee": 0.8 + 0.3 * math.sin(t * 0.8),
        "r_hip": 0.3 * math.sin(t * 0.8 + math.pi),
        "r_knee": 0.8 + 0.3 * math.sin(t * 0.8 + math.pi),
    }

    torques = []
    for i, jname in enumerate(joint_names):
        if i < len(pos):
            target = targets.get(jname, 0.0)
            torque = pids[jname].compute(target - pos[i], dt)
            torques.append(torque)
    robot.set_joint_torques(torques)


stacc.simulate(robot, duration=30, render=True, physics="mujoco")
