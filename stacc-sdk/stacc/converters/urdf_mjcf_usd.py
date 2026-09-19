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
    def robot_to_urdf(robot: Robot) -> str:
        """Convert a stacc Robot instance to URDF XML.

        Produces a flat <robot><link/>...<joint/>...</robot> document that
        round-trips through stacc.core.urdf_parser.load_urdf_string.
        """
        # stacc Joint types -> URDF joint types. URDF has no "spherical" type;
        # "floating" is the closest equivalent (a 6-DOF free joint).
        joint_type_map = {
            "revolute": "revolute",
            "prismatic": "prismatic",
            "continuous": "continuous",
            "fixed": "fixed",
            "spherical": "floating",
        }

        urdf = ET.Element("robot", name=robot.name)

        for link in robot.links:
            l_elem = ET.SubElement(urdf, "link", name=link.name)

            inertial = ET.SubElement(l_elem, "inertial")
            ox, oy, oz = link.origin[:3]
            ET.SubElement(inertial, "origin", xyz=f"{ox} {oy} {oz}", rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=str(link.mass))
            i = link.inertia
            ET.SubElement(
                inertial,
                "inertia",
                ixx=str(i.ixx), ixy=str(i.ixy), ixz=str(i.ixz),
                iyy=str(i.iyy), iyz=str(i.iyz), izz=str(i.izz),
            )

            for tag, geom in (("visual", link.visual), ("collision", link.collision)):
                if geom is None:
                    continue
                g_parent = ET.SubElement(l_elem, tag)
                gx, gy, gz = geom.origin[:3]
                ET.SubElement(g_parent, "origin", xyz=f"{gx} {gy} {gz}", rpy="0 0 0")
                geometry_el = ET.SubElement(g_parent, "geometry")
                if geom.shape == "box":
                    sx, sy, sz = geom.size
                    ET.SubElement(geometry_el, "box", size=f"{sx} {sy} {sz}")
                elif geom.shape == "cylinder":
                    ET.SubElement(
                        geometry_el, "cylinder",
                        radius=str(geom.radius), length=str(geom.length),
                    )
                elif geom.shape == "sphere":
                    ET.SubElement(geometry_el, "sphere", radius=str(geom.radius))
                elif geom.shape == "mesh":
                    sx, sy, sz = geom.mesh_scale
                    ET.SubElement(
                        geometry_el, "mesh",
                        filename=geom.mesh_path or "", scale=f"{sx} {sy} {sz}",
                    )
                if tag == "visual":
                    material = ET.SubElement(g_parent, "material", name=f"{link.name}_material")
                    r, g, b, a = geom.color
                    ET.SubElement(material, "color", rgba=f"{r} {g} {b} {a}")

        for joint in robot.joints:
            urdf_type = joint_type_map.get(joint.type, "fixed")
            j_elem = ET.SubElement(urdf, "joint", name=joint.name, type=urdf_type)
            ET.SubElement(j_elem, "parent", link=joint.parent_link or "")
            ET.SubElement(j_elem, "child", link=joint.child_link or "")

            ox, oy, oz, rr, rp, ry = joint.origin
            ET.SubElement(j_elem, "origin", xyz=f"{ox} {oy} {oz}", rpy=f"{rr} {rp} {ry}")
            ax, ay, az = joint.axis
            ET.SubElement(j_elem, "axis", xyz=f"{ax} {ay} {az}")

            if joint.type not in ("fixed", "continuous"):
                lower, upper = joint.limits
                ET.SubElement(
                    j_elem, "limit",
                    lower=str(lower), upper=str(upper),
                    effort=str(joint.max_effort), velocity=str(joint.max_velocity),
                )
            if joint.damping or joint.friction:
                ET.SubElement(
                    j_elem, "dynamics",
                    damping=str(joint.damping), friction=str(joint.friction),
                )

        return '<?xml version="1.0"?>\n' + ET.tostring(urdf, encoding="unicode")

    @staticmethod
    def robot_to_sdf(robot: Robot) -> str:
        """Convert a stacc Robot instance to Gazebo / Ignition SDF 1.8 XML format.

        Produces a <sdf><model>...</model></sdf> document with full
        inertial/visual/collision/joint-limit data (not just mass + axis)
        that round-trips through stacc.core.urdf_parser.load_sdf.
        """
        # stacc Joint types -> SDF joint types (mirror of the type_map in
        # urdf_parser._parse_sdf_model, which maps SDF "ball"/"screw" back
        # onto stacc's "spherical"/"revolute").
        joint_type_map = {
            "revolute": "revolute",
            "prismatic": "prismatic",
            "continuous": "continuous",
            "fixed": "fixed",
            "spherical": "ball",
        }

        sdf = ET.Element("sdf", version="1.8")
        model = ET.SubElement(sdf, "model", name=robot.name)

        for link in robot.links:
            l_elem = ET.SubElement(model, "link", name=link.name)
            ox, oy, oz = link.origin[:3]
            ET.SubElement(l_elem, "pose").text = f"{ox} {oy} {oz} 0 0 0"

            inertial = ET.SubElement(l_elem, "inertial")
            ET.SubElement(inertial, "mass").text = str(link.mass)
            inertia_el = ET.SubElement(inertial, "inertia")
            i = link.inertia
            for tag, val in (("ixx", i.ixx), ("ixy", i.ixy), ("ixz", i.ixz),
                              ("iyy", i.iyy), ("iyz", i.iyz), ("izz", i.izz)):
                ET.SubElement(inertia_el, tag).text = str(val)

            for tag, geom in (("visual", link.visual), ("collision", link.collision)):
                if geom is None:
                    continue
                g_parent = ET.SubElement(l_elem, tag, name=f"{link.name}_{tag}")
                gx, gy, gz = geom.origin[:3]
                ET.SubElement(g_parent, "pose").text = f"{gx} {gy} {gz} 0 0 0"
                geometry_el = ET.SubElement(g_parent, "geometry")
                if geom.shape == "box":
                    sx, sy, sz = geom.size
                    ET.SubElement(ET.SubElement(geometry_el, "box"), "size").text = f"{sx} {sy} {sz}"
                elif geom.shape == "cylinder":
                    cyl = ET.SubElement(geometry_el, "cylinder")
                    ET.SubElement(cyl, "radius").text = str(geom.radius)
                    ET.SubElement(cyl, "length").text = str(geom.length)
                elif geom.shape == "sphere":
                    ET.SubElement(ET.SubElement(geometry_el, "sphere"), "radius").text = str(geom.radius)
                elif geom.shape == "mesh":
                    mesh_el = ET.SubElement(geometry_el, "mesh")
                    ET.SubElement(mesh_el, "uri").text = geom.mesh_path or ""
                    sx, sy, sz = geom.mesh_scale
                    ET.SubElement(mesh_el, "scale").text = f"{sx} {sy} {sz}"
                if tag == "visual":
                    material = ET.SubElement(g_parent, "material")
                    r, g, b, a = geom.color
                    ET.SubElement(material, "diffuse").text = f"{r} {g} {b} {a}"

        for joint in robot.joints:
            sdf_type = joint_type_map.get(joint.type, "fixed")
            j_elem = ET.SubElement(model, "joint", name=joint.name, type=sdf_type)
            ET.SubElement(j_elem, "parent").text = joint.parent_link or ""
            ET.SubElement(j_elem, "child").text = joint.child_link or ""

            ox, oy, oz, rr, rp, ry = joint.origin
            ET.SubElement(j_elem, "pose").text = f"{ox} {oy} {oz} {rr} {rp} {ry}"

            axis_el = ET.SubElement(j_elem, "axis")
            ax, ay, az = joint.axis
            ET.SubElement(axis_el, "xyz").text = f"{ax} {ay} {az}"

            if joint.type not in ("fixed", "continuous"):
                lower, upper = joint.limits
                limit_el = ET.SubElement(axis_el, "limit")
                ET.SubElement(limit_el, "lower").text = str(lower)
                ET.SubElement(limit_el, "upper").text = str(upper)
                ET.SubElement(limit_el, "effort").text = str(joint.max_effort)
                ET.SubElement(limit_el, "velocity").text = str(joint.max_velocity)
            if joint.damping or joint.friction:
                dynamics_el = ET.SubElement(axis_el, "dynamics")
                ET.SubElement(dynamics_el, "damping").text = str(joint.damping)
                ET.SubElement(dynamics_el, "friction").text = str(joint.friction)

        return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode")
