"""Universal MJCF builder — converts any stacc Robot to MuJoCo XML.

Handles:
- Serial chains (arms, snakes)
- Branching trees (humanoids, multi-arm robots)
- Free-floating bases (mobile robots, drones)
- Any geometry type (box, sphere, cylinder)
- Auto-positioning from geometry dimensions
- Manual offset overrides for precise placement
- Correct actuator ordering matching joint state indices

This file should NEVER need modification for new robot types.
All robot-specific logic lives in the Robot definition, not here.
"""

import xml.etree.ElementTree as ET
from typing import List, Optional

# ── Visual colors for distinguishing links ──
LINK_COLORS = [
    "0.3 0.7 0.5 1",  # green
    "0.3 0.5 0.8 1",  # blue
    "0.8 0.4 0.3 1",  # orange
    "0.6 0.3 0.7 1",  # purple
    "0.8 0.7 0.3 1",  # yellow
    "0.3 0.8 0.8 1",  # cyan
    "0.8 0.3 0.5 1",  # pink
    "0.5 0.8 0.3 1",  # lime
    "0.4 0.4 0.7 1",  # indigo
    "0.7 0.5 0.3 1",  # brown
    "0.3 0.6 0.6 1",  # teal
    "0.7 0.3 0.3 1",  # red
]


def _get_half_height(link) -> float:
    """Get half the height of a link along its primary axis (Z)."""
    if link.visual is None:
        return 0.03
    if link.visual.shape == "box":
        return link.visual.size[2] / 2
    elif link.visual.shape == "cylinder":
        return link.visual.length / 2
    elif link.visual.shape == "sphere":
        return link.visual.radius
    return 0.03


def _get_half_width(link) -> float:
    """Get half the width of a link along X axis."""
    if link.visual is None:
        return 0.03
    if link.visual.shape == "box":
        return link.visual.size[0] / 2
    elif link.visual.shape == "sphere":
        return link.visual.radius
    return 0.03


def _geom_attrs(link, color_index: int) -> dict:
    """Build MuJoCo geom attributes from a Link's visual geometry."""
    attrs = {
        "rgba": LINK_COLORS[color_index % len(LINK_COLORS)],
        "mass": str(link.mass),
    }

    if link.visual is None:
        attrs["type"] = "box"
        attrs["size"] = "0.03 0.03 0.03"
        return attrs

    shape = link.visual.shape

    if shape == "box":
        sx = link.visual.size[0] / 2  # MuJoCo uses half-extents
        sy = link.visual.size[1] / 2
        sz = link.visual.size[2] / 2
        attrs["type"] = "box"
        attrs["size"] = f"{sx} {sy} {sz}"

    elif shape == "sphere":
        attrs["type"] = "sphere"
        attrs["size"] = str(link.visual.radius)

    elif shape == "cylinder":
        half_len = link.visual.length / 2
        attrs["type"] = "cylinder"
        attrs["size"] = f"{link.visual.radius} {half_len}"

    elif shape == "capsule":
        half_len = getattr(link.visual, "length", 0.1) / 2
        attrs["type"] = "capsule"
        attrs["size"] = f"{getattr(link.visual, 'radius', 0.02)} {half_len}"

    else:
        attrs["type"] = "box"
        attrs["size"] = "0.03 0.03 0.03"

    return attrs


def _joint_attrs(joint) -> Optional[dict]:
    """Build MuJoCo joint attributes from a stacc Joint."""
    if joint.type == "fixed":
        return None

    attrs = {"name": joint.name}

    # Type mapping
    type_map = {
        "revolute": "hinge",
        "continuous": "hinge",
        "prismatic": "slide",
        "spherical": "ball",
    }
    attrs["type"] = type_map.get(joint.type, "hinge")

    # Limits
    if joint.type == "revolute" or joint.type == "prismatic":
        attrs["limited"] = "true"
        attrs["range"] = f"{joint.limits[0]} {joint.limits[1]}"
    elif joint.type == "continuous":
        attrs["limited"] = "false"

    # Axis (not used for ball joints)
    if joint.type != "spherical":
        ax = joint.axis
        attrs["axis"] = f"{ax[0]} {ax[1]} {ax[2]}"

    # Damping
    if joint.damping > 0:
        attrs["damping"] = str(joint.damping)

    return attrs


def _compute_auto_offset(parent_link, child_link, joint) -> str:
    """Automatically compute child body position relative to parent.

    The offset places the child at the edge of the parent along the
    joint's primary axis direction.
    """
    parent_half_h = _get_half_height(parent_link)
    child_half_h = _get_half_height(child_link)
    parent_half_w = _get_half_width(parent_link)
    child_half_w = _get_half_width(child_link)

    ax = joint.axis if hasattr(joint, "axis") else [0, 0, 1]

    # Determine primary attachment direction from joint axis
    # Joint axis is rotation axis, so body extends perpendicular to it
    # Default: stack along Z (vertical)
    # If joint axis is X → limb hangs along Z from side of parent
    # If joint axis is Y → limb hangs along Z (standard arm/leg)
    # If joint axis is Z → limb extends along X or Y

    abs_ax = [abs(a) for a in ax]
    max_axis = abs_ax.index(max(abs_ax))

    if max_axis == 0:
        # X-axis rotation → arm/leg attached to side, extends down
        # Place child at side of parent (along X), displaced down in Z
        side = 1.0 if ax[0] >= 0 else -1.0
        x_off = side * (parent_half_w + 0.02)
        z_off = parent_half_h  # hang from top of parent area
        return f"{x_off} 0 {z_off}"

    elif max_axis == 2:
        # Z-axis rotation → rotate around vertical
        # Place child on top
        z_off = parent_half_h + child_half_h
        return f"0 0 {z_off}"

    else:
        # Y-axis rotation → standard vertical stacking
        z_off = parent_half_h + child_half_h
        return f"0 0 {z_off}"


def robot_to_mjcf(robot, config) -> str:
    """Convert any stacc Robot into MuJoCo MJCF XML.

    Handles serial chains, branching trees, and mixed topologies.
    Processes the robot's link tree recursively, generating nested
    <body> elements with proper joints, inertials, and geometry.
    """
    root = ET.Element("mujoco", model=robot.name)
    ET.SubElement(root, "compiler", angle="radian")

    gx, gy, gz = config.gravity
    ET.SubElement(
        root, "option", timestep=str(config.timestep), gravity=f"{gx} {gy} {gz}"
    )

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 0 3", dir="0 0 -1")
    ET.SubElement(worldbody, "geom", type="plane", size="5 5 0.1", rgba="0.9 0.9 0.9 1")

    if not robot.links:
        return ET.tostring(root, encoding="unicode")

    # ── Track state ──
    color_idx = [0]
    joint_order = []  # ordered list of joint names for actuator generation
    visited = set()  # prevent infinite loops

    def _build_body(parent_element, link, joint_for_this, pos_str, is_root=False):
        """Recursively build a body and all its children."""
        if link.name in visited:
            return
        visited.add(link.name)

        # Create body element
        body = ET.SubElement(parent_element, "body", name=link.name, pos=pos_str)

        # Freejoint for mobile robot root
        if is_root and not robot.fixed_base:
            ET.SubElement(body, "freejoint", name="root")

        # Joint
        if joint_for_this is not None and joint_for_this.type != "fixed":
            j_attrs = _joint_attrs(joint_for_this)
            if j_attrs:
                ET.SubElement(body, "joint", **j_attrs)
                joint_order.append(joint_for_this.name)

        # Inertial
        ET.SubElement(
            body,
            "inertial",
            pos="0 0 0",
            mass=str(link.mass),
            diaginertia=(f"{link.inertia.ixx} {link.inertia.iyy} {link.inertia.izz}"),
        )

        # Geometry
        geom_attrs = _geom_attrs(link, color_idx[0])
        color_idx[0] += 1
        ET.SubElement(body, "geom", **geom_attrs)

        # ── Recurse into children ──
        children = robot.get_all_children(link.name)
        for child_joint, child_link, explicit_offset in children:
            if child_link.name in visited:
                continue

            # Compute position
            if explicit_offset is not None:
                # User provided exact offset
                ox, oy, oz = explicit_offset
                child_pos = f"{ox} {oy} {oz}"
            else:
                # Auto-compute from geometry
                child_pos = _compute_auto_offset(link, child_link, child_joint)

            _build_body(body, child_link, child_joint, child_pos)

    # ── Find root and start building ──
    root_link = robot.get_root_link()
    z_offset = 0.5 if robot.fixed_base else 3.0

    # Adjust z_offset for robots with legs (need more clearance)
    if not robot.fixed_base:
        total_height = sum(_get_half_height(l) * 2 for l in robot.links) / max(
            len(robot.links), 1
        )
        z_offset = max(3.0, total_height * 2)

    _build_body(worldbody, root_link, None, f"0 0 {z_offset}", is_root=True)

    # ── Actuators (in joint_order to match MuJoCo's ctrl array) ──
    actuator_el = ET.SubElement(root, "actuator")
    for jname in joint_order:
        jobj = next((j for j in robot.joints if j.name == jname), None)
        effort = jobj.max_effort if jobj else 100.0
        ET.SubElement(
            actuator_el, "motor", joint=jname, ctrlrange=f"{-effort} {effort}"
        )

    return ET.tostring(root, encoding="unicode")
