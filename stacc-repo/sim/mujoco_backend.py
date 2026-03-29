import xml.etree.ElementTree as ET


def robot_to_mjcf(robot: Robot, config: SimulationConfig) -> str:
    """convert a stacc Robot into MJCF XML string"""


root = ET.Element("mujoco", model="Robot.name")
option = ET.SubElement(root, "option", gravity="0 0 -9.81", timestep="0.001")
worldbody = ET.SubElement(root, "worldbody")
# ...build tree
return ET.tostring(root, encoding="unicode")
