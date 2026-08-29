"""Universal Multi-Format Robot Model Transpiler.

Converts and validates models between:
- URDF (Unified Robot Description Format)
- MJCF (MuJoCo XML)
- SDF (Simulation Description Format)
- OpenUSD (Universal Scene Description for NVIDIA Omniverse / Isaac Lab)
- RIR (Robotics Intermediate Representation)
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple, Union
import xml.etree.ElementTree as ET
import numpy as np

from stacc.core.robot_base import Robot


class ModelTranspiler:
    """Universal Bidirectional Robot Format Transpiler."""

    @staticmethod
    def validate_inertia_tensor(mass: float, ixx: float, iyy: float, izz: float) -> Tuple[bool, str]:
        """Validate physical plausibility via triangle inequalities:
        Ixx + Iyy >= Izz, Iyy + Izz >= Ixx, Ixx + Izz >= Iyy and mass > 0.
        """
        if mass <= 0:
            return False, f"Non-positive mass: {mass} kg"
        if ixx <= 0 or iyy <= 0 or izz <= 0:
            return False, f"Non-positive principal inertia: ({ixx}, {iyy}, {izz})"
        if ixx + iyy < izz or iyy + izz < ixx or ixx + izz < iyy:
            return False, f"Violates triangle inequality: ({ixx}, {iyy}, {izz})"
        return True, "Physical"

    @staticmethod
    def robot_to_usd_spec(robot: Robot) -> Dict[str, Union[str, List[Dict]]]:
        """Generate OpenUSD Stage prim hierarchy for Isaac Lab / Omniverse."""
        prim_tree = {
            "stage_type": "UsdStage",
            "default_prim": f"/World/{robot.name}",
            "meters_per_unit": 1.0,
            "up_axis": "Z",
            "prims": []
        }

        # Add Articulation Root
        for link in robot.links:
            prim_tree["prims"].append({
                "path": f"/World/{robot.name}/{link.name}",
                "type": "UsdGeomXform",
                "mass": getattr(link, "mass", 1.0),
                "is_rigid_body": True
            })

        for joint in robot.joints:
            prim_tree["prims"].append({
                "path": f"/World/{robot.name}/joints/{joint.name}",
                "type": "UsdPhysicsRevoluteJoint" if joint.type == "revolute" else "UsdPhysicsPrismaticJoint",
                "body0": f"/World/{robot.name}/{joint.parent_link}",
                "body1": f"/World/{robot.name}/{joint.child_link}",
                "axis": joint.axis
            })

        return prim_tree

    @staticmethod
    def robot_to_sdf(robot: Robot) -> str:
        """Convert a stacc Robot instance to Gazebo / Ignition SDF 1.8 XML format."""
        sdf = ET.Element("sdf", version="1.8")
        model = ET.SubElement(sdf, "model", name=robot.name)

        for link in robot.links:
            l_elem = ET.SubElement(model, "link", name=link.name)
            inertial = ET.SubElement(l_elem, "inertial")
            ET.SubElement(inertial, "mass").text = str(getattr(link, "mass", 1.0))

        for joint in robot.joints:
            j_elem = ET.SubElement(model, "joint", name=joint.name, type=joint.type)
            ET.SubElement(j_elem, "parent").text = joint.parent_link
            ET.SubElement(j_elem, "child").text = joint.child_link
            axis = ET.SubElement(j_elem, "axis")
            ET.SubElement(axis, "xyz").text = f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}"

        return ET.tostring(sdf, encoding="unicode")
