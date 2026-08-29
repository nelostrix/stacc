"""Humanoid Bipedal Robot Topology.

Full-body 20-30 DOF humanoid robot model (Atlas / Digit / Optimus style):
- Pelvis floating base
- Left & Right Legs (6 DOF each: hip yaw, hip roll, hip pitch, knee pitch, ankle pitch, ankle roll)
- Torso & Waist (2-3 DOF: waist yaw, waist pitch)
- Left & Right Arms (4-7 DOF each: shoulder pitch, roll, yaw, elbow pitch, wrist roll/pitch/yaw)
- Neck / Head (2 DOF: pitch, yaw)
"""

from __future__ import annotations
from typing import Dict, List, Optional
import math
import numpy as np

from stacc.core.robot_base import Robot
from stacc.core.joint import Joint
from stacc.core.link import Link, Geometry


class HumanoidRobot(Robot):
    """Full-Body Bipedal Humanoid Robot Model.

    Args:
        name: Robot name
        total_height: Total humanoid height in meters (default 1.65m)
        total_mass: Total humanoid mass in kg (default 55.0kg)
    """

    def __init__(
        self,
        name: str = "humanoid",
        total_height: float = 1.65,
        total_mass: float = 55.0
    ):
        super().__init__(name=name)
        self.total_height = float(total_height)
        self.total_mass = float(total_mass)

        # 1. Floating Base / Pelvis
        self.add_link(Link(name="pelvis", mass=12.0, visual=Geometry(shape="box", size=[0.25, 0.30, 0.15])))

        # 2. Torso / Upper Body
        self.add_joint(Joint(name="waist_yaw", type="revolute", axis=[0, 0, 1], origin=[0, 0, 0.15], parent_link="pelvis", child_link="torso"))
        self.add_link(Link(name="torso", mass=18.0, visual=Geometry(shape="box", size=[0.28, 0.36, 0.45])))

        # 3. Head & Neck
        self.add_joint(Joint(name="neck_yaw", type="revolute", axis=[0, 0, 1], origin=[0, 0, 0.45], parent_link="torso", child_link="neck"))
        self.add_link(Link(name="neck", mass=1.0, visual=Geometry(shape="cylinder", radius=0.05, length=0.08)))
        self.add_joint(Joint(name="head_pitch", type="revolute", axis=[0, 1, 0], origin=[0, 0, 0.08], parent_link="neck", child_link="head"))
        self.add_link(Link(name="head", mass=3.5, visual=Geometry(shape="sphere", radius=0.10)))

        # 4. Legs (Left & Right - 6 DOF each)
        for side, is_left in [("left", 1), ("right", -1)]:
            y_offset = is_left * 0.10

            # Hip Yaw
            self.add_joint(Joint(name=f"{side}_hip_yaw", type="revolute", axis=[0, 0, 1], origin=[0, y_offset, -0.05], parent_link="pelvis", child_link=f"{side}_hip_yaw_link"))
            self.add_link(Link(name=f"{side}_hip_yaw_link", mass=1.5))

            # Hip Roll
            self.add_joint(Joint(name=f"{side}_hip_roll", type="revolute", axis=[1, 0, 0], origin=[0, 0, 0], parent_link=f"{side}_hip_yaw_link", child_link=f"{side}_hip_roll_link"))
            self.add_link(Link(name=f"{side}_hip_roll_link", mass=1.8))

            # Hip Pitch (Thigh)
            self.add_joint(Joint(name=f"{side}_hip_pitch", type="revolute", axis=[0, 1, 0], origin=[0, 0, 0], parent_link=f"{side}_hip_roll_link", child_link=f"{side}_thigh"))
            self.add_link(Link(name=f"{side}_thigh", mass=4.5, visual=Geometry(shape="cylinder", radius=0.06, length=0.40)))

            # Knee Pitch (Shank)
            self.add_joint(Joint(name=f"{side}_knee", type="revolute", axis=[0, 1, 0], origin=[0, 0, -0.40], parent_link=f"{side}_thigh", child_link=f"{side}_shank"))
            self.add_link(Link(name=f"{side}_shank", mass=3.2, visual=Geometry(shape="cylinder", radius=0.05, length=0.40)))

            # Ankle Pitch
            self.add_joint(Joint(name=f"{side}_ankle_pitch", type="revolute", axis=[0, 1, 0], origin=[0, 0, -0.40], parent_link=f"{side}_shank", child_link=f"{side}_ankle"))
            self.add_link(Link(name=f"{side}_ankle", mass=0.8))

            # Ankle Roll (Foot sole contact plane)
            self.add_joint(Joint(name=f"{side}_ankle_roll", type="revolute", axis=[1, 0, 0], origin=[0, 0, 0], parent_link=f"{side}_ankle", child_link=f"{side}_foot"))
            self.add_link(Link(name=f"{side}_foot", mass=1.2, visual=Geometry(shape="box", size=[0.22, 0.10, 0.04])))

        # 5. Arms (Left & Right - 4 DOF each)
        for side, is_left in [("left", 1), ("right", -1)]:
            y_offset = is_left * 0.22

            # Shoulder Pitch
            self.add_joint(Joint(name=f"{side}_shoulder_pitch", type="revolute", axis=[0, 1, 0], origin=[0, y_offset, 0.40], parent_link="torso", child_link=f"{side}_shoulder"))
            self.add_link(Link(name=f"{side}_shoulder", mass=1.8))

            # Shoulder Roll
            self.add_joint(Joint(name=f"{side}_shoulder_roll", type="revolute", axis=[1, 0, 0], origin=[0, 0, 0], parent_link=f"{side}_shoulder", child_link=f"{side}_upper_arm"))
            self.add_link(Link(name=f"{side}_upper_arm", mass=2.2, visual=Geometry(shape="cylinder", radius=0.04, length=0.30)))

            # Elbow Pitch
            self.add_joint(Joint(name=f"{side}_elbow", type="revolute", axis=[0, 1, 0], origin=[0, 0, -0.30], parent_link=f"{side}_upper_arm", child_link=f"{side}_forearm"))
            self.add_link(Link(name=f"{side}_forearm", mass=1.6, visual=Geometry(shape="cylinder", radius=0.04, length=0.28)))

            # Wrist Roll / Hand
            self.add_joint(Joint(name=f"{side}_wrist_roll", type="revolute", axis=[0, 0, 1], origin=[0, 0, -0.28], parent_link=f"{side}_forearm", child_link=f"{side}_hand"))
            self.add_link(Link(name=f"{side}_hand", mass=0.6, visual=Geometry(shape="box", size=[0.10, 0.06, 0.04])))
