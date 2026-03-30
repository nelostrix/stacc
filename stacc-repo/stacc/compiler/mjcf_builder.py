import xml.etree.ElementTree as ET


def robot_to_mjcf(robot, config) -> str:
    root = ET.Element("mujoco", model=robot.name)

    ET.SubElement(root, "option", gravity="0 0 -9.81", timestamp=str(config.timestep))
    worldbody = ET.SubElement(root, "worldbody")

    # minimal body (required!)
    body = ET.SubElement(worldbody, "body", name="base", pos="0 0 0")
    ET.SubElement(body, "geom", type="sphere", size="0.1", rgba="0.8 0.2 0.2 1")
    return ET.tostring(root, encoding="unicode")
