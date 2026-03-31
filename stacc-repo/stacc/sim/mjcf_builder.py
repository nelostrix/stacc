"""Convert stacc Robot objects to MuJoCo MJCF XML."""

import xml.etree.ElementTree as ET

# Colors for different links so you can tell them apart in the viewer
LINK_COLORS = [
    "0.3 0.7 0.5 1",
    "0.3 0.5 0.8 1",
    "0.8 0.4 0.3 1",
    "0.6 0.3 0.7 1",
    "0.8 0.7 0.3 1",
    "0.3 0.8 0.8 1",
]


def _geom_attrs(link, color_index):
    """Build geom attributes from a Link's visual geometry."""
    attrs = {"rgba": LINK_COLORS[color_index % len(LINK_COLORS)]}

    if link.visual is None:
        # No geometry defined — make a default small box
        attrs["type"] = "box"
        attrs["size"] = "0.03 0.03 0.03"
        attrs["mass"] = str(link.mass)
        return attrs

    shape = link.visual.shape
    attrs["mass"] = str(link.mass)

    if shape == "box":
        # IMPORTANT: MuJoCo uses HALF-sizes for boxes
        sx = link.visual.size[0] / 2
        sy = link.visual.size[1] / 2
        sz = link.visual.size[2] / 2
        attrs["type"] = "box"
        attrs["size"] = f"{sx} {sy} {sz}"

    elif shape == "sphere":
        attrs["type"] = "sphere"
        attrs["size"] = str(link.visual.radius)

    elif shape == "cylinder":
        attrs["type"] = "cylinder"
        half_len = link.visual.length / 2
        attrs["size"] = f"{link.visual.radius} {half_len}"

    else:
        # Fallback to box
        attrs["type"] = "box"
        attrs["size"] = "0.03 0.03 0.03"

    return attrs


def _joint_attrs(joint):
    """Build joint attributes from a stacc Joint."""
    attrs = {"name": joint.name}

    # Map stacc joint types to MuJoCo types
    if joint.type == "revolute":
        attrs["type"] = "hinge"
        attrs["limited"] = "true"
        attrs["range"] = f"{joint.limits[0]} {joint.limits[1]}"
    elif joint.type == "continuous":
        attrs["type"] = "hinge"
        attrs["limited"] = "false"
    elif joint.type == "prismatic":
        attrs["type"] = "slide"
        attrs["limited"] = "true"
        attrs["range"] = f"{joint.limits[0]} {joint.limits[1]}"
    else:
        return None  # fixed joints = no joint element

    # Axis
    ax = joint.axis
    attrs["axis"] = f"{ax[0]} {ax[1]} {ax[2]}"

    # Damping
    if joint.damping > 0:
        attrs["damping"] = str(joint.damping)

    return attrs


def robot_to_mjcf(robot, config) -> str:
    """Convert a stacc Robot into MJCF XML string.

    Builds a nested body chain from the robot's links and joints.
    Links[0] is the root body, each subsequent link is nested inside
    the previous one, with joints[i] attached to links[i+1].
    """
    root = ET.Element("mujoco", model=robot.name)

    # Compiler
    ET.SubElement(root, "compiler", angle="radian")

    # Physics options
    gx, gy, gz = config.gravity
    ET.SubElement(
        root, "option", timestep=str(config.timestep), gravity=f"{gx} {gy} {gz}"
    )

    # World
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 0 3", dir="0 0 -1")
    ET.SubElement(worldbody, "geom", type="plane", size="5 5 0.1", rgba="0.9 0.9 0.9 1")

    # Build kinematic chain
    # Link 0 = root body (attached to world)
    # Link i+1 = child of link i, with joint i between them
    links = robot.links
    joints = robot.joints

    if len(links) == 0:
        # Empty robot — just return minimal MJCF
        return ET.tostring(root, encoding="unicode")

    # Calculate body positions along Z axis based on geometry
    # Each link is stacked above the previous one
    current_parent = worldbody
    z_offset = 0.5  # Start above ground

    for i, link in enumerate(links):
        # Compute height of this link for positioning
        if link.visual and link.visual.shape == "box":
            link_height = link.visual.size[2]
        elif link.visual and link.visual.shape == "cylinder":
            link_height = link.visual.length
        else:
            link_height = 0.06  # default

        # Position: first link at z_offset, subsequent links stacked
        if i == 0:
            pos = f"0 0 {z_offset}"
        else:
            pos = f"0 0 {link_height}"

        # Create body
        body = ET.SubElement(current_parent, "body", name=link.name, pos=pos)

        # Add joint (joint i connects link i to link i+1)
        # So link 0 has no joint, link 1 has joint 0, link 2 has joint 1, etc.
        if i > 0 and (i - 1) < len(joints):
            joint = joints[i - 1]
            if joint.type != "fixed":
                j_attrs = _joint_attrs(joint)
                if j_attrs:
                    ET.SubElement(body, "joint", **j_attrs)

        # Add inertial
        ET.SubElement(
            body,
            "inertial",
            pos="0 0 0",
            mass=str(link.mass),
            diaginertia=f"{link.inertia.ixx} {link.inertia.iyy} {link.inertia.izz}",
        )

        # Add geometry
        geom_attrs = _geom_attrs(link, i)
        ET.SubElement(body, "geom", **geom_attrs)

        # Next link is nested inside this one
        current_parent = body

    # Actuators — one motor per actuated joint
    actuator_el = ET.SubElement(root, "actuator")
    for joint in joints:
        if joint.type not in ("fixed",):
            ET.SubElement(
                actuator_el,
                "motor",
                joint=joint.name,
                ctrlrange=f"{-joint.max_effort} {joint.max_effort}",
            )

    return ET.tostring(root, encoding="unicode")
