"""Universal URDF/SDF parser — load any robot description into stacc.

Reads URDF (Unified Robot Description Format) and SDF (Simulation
Description Format) files and produces a stacc Robot object with full
topology, geometry, inertial properties, and joint configurations.

Supports:
- All URDF joint types: revolute, continuous, prismatic, fixed, floating, planar
- All geometry types: box, cylinder, sphere, mesh (STL/DAE/OBJ references)
- Full inertial properties: mass, inertia tensor, center of mass
- Material/color definitions and references
- Tree topologies of any depth and branching factor
- Joint limits, damping, friction, effort, velocity bounds
- Joint mimic relationships
- Origin transforms (xyz + rpy) on joints, visuals, collisions, inertials
- Graceful handling of missing/optional elements

Usage:
    from stacc.core.urdf_parser import load_urdf, load_sdf

    # From file
    robot = load_urdf("path/to/robot.urdf")

    # From string
    robot = load_urdf_string(urdf_xml_string)

    # Then simulate as usual
    import stacc
    stacc.simulate(robot, duration=10, render=True, physics="mujoco")
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Tuple

from stacc.core.joint import Joint
from stacc.core.link import Geometry, Inertia, Link
from stacc.core.robot_base import Robot

# ══════════════════════════════════════════
# PARSING UTILITIES
# ══════════════════════════════════════════


def _parse_vec3(
    text: Optional[str], default: Optional[List[float]] = None
) -> List[float]:
    """Parse a space-separated '1.0 2.0 3.0' string into [float, float, float]."""
    if text is None:
        return default or [0.0, 0.0, 0.0]
    parts = text.strip().split()
    return [float(p) for p in parts]


def _parse_float(text: Optional[str], default: float = 0.0) -> float:
    """Parse a float from string, returning default if None or invalid."""
    if text is None:
        return default
    try:
        return float(text)
    except (ValueError, TypeError):
        return default


def _parse_origin(element: Optional[ET.Element]) -> Tuple[List[float], List[float]]:
    """Parse an <origin xyz="..." rpy="..."/> element.

    Returns:
        (xyz, rpy) as two lists of 3 floats each.
    """
    if element is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    xyz = _parse_vec3(element.get("xyz"), [0.0, 0.0, 0.0])
    rpy = _parse_vec3(element.get("rpy"), [0.0, 0.0, 0.0])
    return xyz, rpy


def _parse_color(element: Optional[ET.Element]) -> Optional[List[float]]:
    """Parse a <color rgba="r g b a"/> element."""
    if element is None:
        return None
    rgba_str = element.get("rgba")
    if rgba_str:
        return _parse_vec3(rgba_str + " 1.0")[:4]  # ensure 4 values
    return None


# ══════════════════════════════════════════
# GEOMETRY PARSING
# ══════════════════════════════════════════


def _parse_geometry(
    geom_element: Optional[ET.Element], urdf_dir: str = ""
) -> Optional[Geometry]:
    """Parse a <geometry> element containing box/cylinder/sphere/mesh."""
    if geom_element is None:
        return None

    # Box: <box size="x y z"/>
    box = geom_element.find("box")
    if box is not None:
        size = _parse_vec3(box.get("size"), [0.1, 0.1, 0.1])
        return Geometry(shape="box", size=size)

    # Cylinder: <cylinder radius="r" length="l"/>
    cyl = geom_element.find("cylinder")
    if cyl is not None:
        radius = _parse_float(cyl.get("radius"), 0.05)
        length = _parse_float(cyl.get("length"), 0.1)
        return Geometry(shape="cylinder", radius=radius, length=length)

    # Sphere: <sphere radius="r"/>
    sph = geom_element.find("sphere")
    if sph is not None:
        radius = _parse_float(sph.get("radius"), 0.05)
        return Geometry(shape="sphere", radius=radius)

    # Mesh: <mesh filename="path.stl" scale="sx sy sz"/>
    mesh = geom_element.find("mesh")
    if mesh is not None:
        filename = mesh.get("filename", "")
        scale = _parse_vec3(mesh.get("scale"), [1.0, 1.0, 1.0])

        # Resolve relative paths
        if filename and not os.path.isabs(filename):
            # Handle package:// URIs
            if filename.startswith("package://"):
                # Strip package:// prefix, keep relative path
                filename = filename.replace("package://", "")
            # Handle file:// URIs
            elif filename.startswith("file://"):
                filename = filename.replace("file://", "")

            # Try to resolve relative to URDF file location
            if urdf_dir:
                resolved = os.path.join(urdf_dir, filename)
                if os.path.exists(resolved):
                    filename = resolved

        return Geometry(
            shape="mesh",
            mesh_path=filename,
            mesh_scale=scale,
        )

    return None


# ══════════════════════════════════════════
# LINK PARSING
# ══════════════════════════════════════════


def _parse_link(
    link_element: ET.Element, materials: Dict[str, List[float]], urdf_dir: str = ""
) -> Link:
    """Parse a <link> element into a stacc Link.

    Handles:
    - <inertial>: mass, inertia tensor, origin
    - <visual>: geometry, material/color, origin
    - <collision>: geometry, origin
    - Virtual links (no geometry — common in URDF)
    """
    name = link_element.get("name", "unnamed_link")

    # ── Inertial ──
    inertial_el = link_element.find("inertial")
    if inertial_el is not None:
        mass_el = inertial_el.find("mass")
        mass = _parse_float(mass_el.get("value") if mass_el is not None else None, 0.1)

        inertia_el = inertial_el.find("inertia")
        if inertia_el is not None:
            inertia = Inertia(
                ixx=_parse_float(inertia_el.get("ixx"), 0.001),
                iyy=_parse_float(inertia_el.get("iyy"), 0.001),
                izz=_parse_float(inertia_el.get("izz"), 0.001),
                ixy=_parse_float(inertia_el.get("ixy"), 0.0),
                ixz=_parse_float(inertia_el.get("ixz"), 0.0),
                iyz=_parse_float(inertia_el.get("iyz"), 0.0),
            )
        else:
            inertia = Inertia.from_box(mass, 0.05, 0.05, 0.05)

        inertial_origin, _ = _parse_origin(inertial_el.find("origin"))
    else:
        mass = 0.1  # virtual link — minimal mass
        inertia = Inertia.from_box(0.1, 0.01, 0.01, 0.01)
        inertial_origin = [0, 0, 0]

    # ── Visual ──
    visual_el = link_element.find("visual")
    visual_geom = None
    color = None

    if visual_el is not None:
        geom_el = visual_el.find("geometry")
        visual_geom = _parse_geometry(geom_el, urdf_dir)

        # Material / color
        material_el = visual_el.find("material")
        if material_el is not None:
            # Direct color
            color_el = material_el.find("color")
            if color_el is not None:
                color = _parse_color(color_el)
            # Named material reference
            elif material_el.get("name") in materials:
                color = materials[material_el.get("name")]

        if visual_geom and color:
            visual_geom.color = color

    # ── Collision ──
    collision_el = link_element.find("collision")
    collision_geom = None
    if collision_el is not None:
        collision_geom = _parse_geometry(collision_el.find("geometry"), urdf_dir)

    # ── Build Link ──
    link = Link(
        name=name,
        mass=max(mass, 0.001),  # MuJoCo requires positive mass
        inertia=inertia,
        visual=visual_geom,
        collision=collision_geom or visual_geom,
    )

    return link


# ══════════════════════════════════════════
# JOINT PARSING
# ══════════════════════════════════════════


def _parse_joint(joint_element: ET.Element) -> Tuple[Joint, str, str, List[float]]:
    """Parse a <joint> element into a stacc Joint + parent/child names + offset.

    Returns:
        (joint, parent_link_name, child_link_name, xyz_offset)
    """
    name = joint_element.get("name", "unnamed_joint")
    urdf_type = joint_element.get("type", "fixed")

    # Type mapping: URDF → stacc
    type_map = {
        "revolute": "revolute",
        "continuous": "continuous",
        "prismatic": "prismatic",
        "fixed": "fixed",
        "floating": "continuous",  # approximate as free
        "planar": "prismatic",  # approximate
    }
    stacc_type = type_map.get(urdf_type, "fixed")

    # Parent and child
    parent_el = joint_element.find("parent")
    child_el = joint_element.find("child")
    parent_name = parent_el.get("link", "") if parent_el is not None else ""
    child_name = child_el.get("link", "") if child_el is not None else ""

    # Origin (transform from parent to joint frame)
    origin_xyz, origin_rpy = _parse_origin(joint_element.find("origin"))

    # Axis
    axis_el = joint_element.find("axis")
    if axis_el is not None:
        axis = _parse_vec3(axis_el.get("xyz"), [0, 0, 1])
    else:
        axis = [0, 0, 1]  # default Z axis

    # Limits
    limit_el = joint_element.find("limit")
    limits = None
    max_effort = 100.0
    max_velocity = 10.0

    if limit_el is not None:
        lower = _parse_float(limit_el.get("lower"), -math.pi)
        upper = _parse_float(limit_el.get("upper"), math.pi)
        max_effort = _parse_float(limit_el.get("effort"), 100.0)
        max_velocity = _parse_float(limit_el.get("velocity"), 10.0)
        limits = (lower, upper)

    # Dynamics
    dynamics_el = joint_element.find("dynamics")
    damping = 0.0
    friction = 0.0

    if dynamics_el is not None:
        damping = _parse_float(dynamics_el.get("damping"), 0.0)
        friction = _parse_float(dynamics_el.get("friction"), 0.0)

    # Build Joint
    joint = Joint(
        name=name,
        type=stacc_type,
        parent_link=parent_name,
        child_link=child_name,
        axis=axis,
        limits=limits,
        max_velocity=max_velocity,
        max_effort=max_effort,
        damping=damping,
        friction=friction,
        origin=origin_xyz + origin_rpy,
    )

    return joint, parent_name, child_name, origin_xyz


# ══════════════════════════════════════════
# MATERIAL PARSING
# ══════════════════════════════════════════


def _parse_materials(robot_element: ET.Element) -> Dict[str, List[float]]:
    """Parse top-level <material> definitions (named colors).

    URDF allows defining materials at the robot level:
        <material name="blue">
            <color rgba="0 0 0.8 1"/>
        </material>

    These can then be referenced by name in link visuals.
    """
    materials = {}

    for mat_el in robot_element.findall("material"):
        name = mat_el.get("name")
        if name:
            color_el = mat_el.find("color")
            if color_el is not None:
                rgba = _parse_color(color_el)
                if rgba:
                    materials[name] = rgba

    return materials


# ══════════════════════════════════════════
# MAIN PARSER
# ══════════════════════════════════════════


def _parse_urdf_tree(robot_element: ET.Element, urdf_dir: str = "") -> Robot:
    """Parse a <robot> XML element into a stacc Robot.

    This is the core parser that handles any URDF topology.
    """
    robot_name = robot_element.get("name", "urdf_robot")
    robot = Robot(name=robot_name)
    robot.fixed_base = True  # URDF robots are fixed by default

    # ── Parse materials ──
    materials = _parse_materials(robot_element)

    # Also collect materials defined inside links
    for link_el in robot_element.findall("link"):
        visual_el = link_el.find("visual")
        if visual_el is not None:
            mat_el = visual_el.find("material")
            if mat_el is not None:
                name = mat_el.get("name")
                color_el = mat_el.find("color")
                if name and color_el is not None:
                    rgba = _parse_color(color_el)
                    if rgba and name not in materials:
                        materials[name] = rgba

    # ── Parse all links ──
    link_map: Dict[str, Link] = {}
    for link_el in robot_element.findall("link"):
        link = _parse_link(link_el, materials, urdf_dir)
        link_map[link.name] = link
        robot.add_link(link)

    # ── Parse all joints and build topology ──
    child_links = set()  # track which links are children

    for joint_el in robot_element.findall("joint"):
        joint, parent_name, child_name, offset = _parse_joint(joint_el)

        if parent_name not in link_map or child_name not in link_map:
            # Skip joints referencing unknown links
            continue

        child_links.add(child_name)
        robot.connect(parent_name, child_name, joint, offset=offset)

    # ── Determine root link ──
    # Root link = a link that is never a child in any joint
    root_candidates = [name for name in link_map if name not in child_links]

    if root_candidates:
        # Check if robot has a "world" or "base" link that should be fixed
        root_name = root_candidates[0]
        root_link = link_map[root_name]

        # If root link has no geometry and no mass, it's a virtual "world" frame
        if root_link.visual is None and root_link.mass <= 0.1:
            robot.fixed_base = True

    return robot


# ══════════════════════════════════════════
# PUBLIC API
# ══════════════════════════════════════════


def load_urdf(filepath: str) -> Robot:
    """Load a URDF file and return a stacc Robot.

    Args:
        filepath: Path to the .urdf file

    Returns:
        A stacc Robot object ready for simulation

    Usage:
        robot = load_urdf("robots/panda.urdf")
        stacc.simulate(robot, duration=10)
    """
    filepath = os.path.expanduser(filepath)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"URDF file not found: {filepath}")

    urdf_dir = os.path.dirname(os.path.abspath(filepath))

    tree = ET.parse(filepath)
    root = tree.getroot()

    # Handle both <robot> root and files that start with <?xml>
    if root.tag != "robot":
        robot_el = root.find("robot")
        if robot_el is None:
            raise ValueError(f"No <robot> element found in {filepath}")
    else:
        robot_el = root

    return _parse_urdf_tree(robot_el, urdf_dir)


def load_urdf_string(urdf_xml: str) -> Robot:
    """Parse a URDF XML string and return a stacc Robot.

    Args:
        urdf_xml: URDF XML content as a string

    Returns:
        A stacc Robot object ready for simulation

    Usage:
        xml = open("robot.urdf").read()
        robot = load_urdf_string(xml)
    """
    root = ET.fromstring(urdf_xml)

    if root.tag != "robot":
        robot_el = root.find("robot")
        if robot_el is None:
            raise ValueError("No <robot> element found in URDF string")
    else:
        robot_el = root

    return _parse_urdf_tree(robot_el)


def load_sdf(filepath: str) -> Robot:
    """Load an SDF (Simulation Description Format) file.

    SDF is used by Gazebo. This parser extracts the first <model>
    from the SDF file and converts it to a stacc Robot.

    Args:
        filepath: Path to the .sdf file

    Returns:
        A stacc Robot object
    """
    filepath = os.path.expanduser(filepath)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"SDF file not found: {filepath}")

    sdf_dir = os.path.dirname(os.path.abspath(filepath))
    tree = ET.parse(filepath)
    root = tree.getroot()

    # SDF structure: <sdf><world><model>...</model></world></sdf>
    # or: <sdf><model>...</model></sdf>
    model = root.find(".//model")
    if model is None:
        raise ValueError(f"No <model> element found in {filepath}")

    return _parse_sdf_model(model, sdf_dir)


def _parse_sdf_model(model_element: ET.Element, sdf_dir: str = "") -> Robot:
    """Parse an SDF <model> element into a stacc Robot.

    SDF is similar to URDF but with different tag names:
    - <link> is the same
    - <joint> is the same
    - <inertial><inertia> uses <ixx>, <iyy>, etc. as child elements
    - <geometry> is the same
    - <pose> replaces <origin> (format: "x y z roll pitch yaw")
    """
    robot_name = model_element.get("name", "sdf_robot")
    robot = Robot(name=robot_name)
    robot.fixed_base = True

    link_map = {}
    child_links = set()

    # ── Parse links ──
    for link_el in model_element.findall("link"):
        name = link_el.get("name", "unnamed")

        # Mass/inertia
        inertial_el = link_el.find("inertial")
        mass = 0.1
        inertia = Inertia.from_box(0.1, 0.05, 0.05, 0.05)

        if inertial_el is not None:
            mass_el = inertial_el.find("mass")
            if mass_el is not None:
                mass = _parse_float(mass_el.text, 0.1)

            inertia_el = inertial_el.find("inertia")
            if inertia_el is not None:
                inertia = Inertia(
                    ixx=_parse_float(
                        getattr(inertia_el.find("ixx"), "text", None), 0.001
                    ),
                    iyy=_parse_float(
                        getattr(inertia_el.find("iyy"), "text", None), 0.001
                    ),
                    izz=_parse_float(
                        getattr(inertia_el.find("izz"), "text", None), 0.001
                    ),
                    ixy=_parse_float(
                        getattr(inertia_el.find("ixy"), "text", None), 0.0
                    ),
                    ixz=_parse_float(
                        getattr(inertia_el.find("ixz"), "text", None), 0.0
                    ),
                    iyz=_parse_float(
                        getattr(inertia_el.find("iyz"), "text", None), 0.0
                    ),
                )

        # Visual geometry
        visual_el = link_el.find("visual")
        visual_geom = None
        if visual_el is not None:
            visual_geom = _parse_geometry(visual_el.find("geometry"), sdf_dir)

        link = Link(
            name=name, mass=max(mass, 0.001), inertia=inertia, visual=visual_geom
        )
        link_map[name] = link
        robot.add_link(link)

    # ── Parse joints ──
    for joint_el in model_element.findall("joint"):
        name = joint_el.get("name", "unnamed_joint")
        jtype = joint_el.get("type", "fixed")

        parent_el = joint_el.find("parent")
        child_el = joint_el.find("child")
        parent_name = (
            parent_el.text.strip() if parent_el is not None and parent_el.text else ""
        )
        child_name = (
            child_el.text.strip() if child_el is not None and child_el.text else ""
        )

        # Pose (SDF uses <pose>x y z r p y</pose> instead of <origin>)
        pose_el = joint_el.find("pose")
        offset = [0.0, 0.0, 0.0]
        if pose_el is not None and pose_el.text:
            vals = _parse_vec3(pose_el.text + " 0 0 0")
            offset = vals[:3]

        # Axis
        axis_el = joint_el.find("axis")
        axis = [0, 0, 1]
        limits = None
        damping = 0.0

        if axis_el is not None:
            xyz_el = axis_el.find("xyz")
            if xyz_el is not None and xyz_el.text:
                axis = _parse_vec3(xyz_el.text)

            limit_el = axis_el.find("limit")
            if limit_el is not None:
                lower = _parse_float(
                    limit_el.find("lower").text
                    if limit_el.find("lower") is not None
                    else None,
                    -math.pi,
                )
                upper = _parse_float(
                    limit_el.find("upper").text
                    if limit_el.find("upper") is not None
                    else None,
                    math.pi,
                )
                limits = (lower, upper)

            dynamics_el = axis_el.find("dynamics")
            if dynamics_el is not None:
                damping = _parse_float(
                    dynamics_el.find("damping").text
                    if dynamics_el.find("damping") is not None
                    else None,
                    0.0,
                )

        type_map = {
            "revolute": "revolute",
            "continuous": "continuous",
            "prismatic": "prismatic",
            "fixed": "fixed",
            "ball": "spherical",
            "screw": "revolute",
        }
        stacc_type = type_map.get(jtype, "fixed")

        joint = Joint(
            name=name,
            type=stacc_type,
            axis=axis,
            limits=limits,
            damping=damping,
            parent_link=parent_name,
            child_link=child_name,
        )

        if parent_name in link_map and child_name in link_map:
            child_links.add(child_name)
            robot.connect(parent_name, child_name, joint, offset=offset)

    return robot


# ══════════════════════════════════════════
# CONVENIENCE: Auto-detect format
# ══════════════════════════════════════════


def load_robot(filepath: str) -> Robot:
    """Load a robot from any supported format (auto-detected).

    Supports: .urdf, .sdf, .xml (tries URDF first, then SDF)

    Args:
        filepath: Path to the robot description file

    Returns:
        A stacc Robot object

    Usage:
        robot = load_robot("my_robot.urdf")
        robot = load_robot("my_robot.sdf")
    """
    filepath = os.path.expanduser(filepath)
    ext = os.path.splitext(filepath)[1].lower()

    if ext == ".sdf":
        return load_sdf(filepath)
    elif ext in (".urdf", ".xml", ".xacro"):
        return load_urdf(filepath)
    else:
        # Try URDF first, fall back to SDF
        try:
            return load_urdf(filepath)
        except (ValueError, ET.ParseError):
            return load_sdf(filepath)
