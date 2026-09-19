"""RSS — a small declarative language for describing robots.

    robot "FactoryBot" {
        chassis { shape: differential-drive; dimensions: [600mm, 450mm, 250mm]; }
        arm left { joints: 6; actuator: servo; reach: 650mm; }
        wheel left { radius: 120mm; }
    }

Pipeline: `parse()` (text -> AST) -> `compile_document()` / `compile_robot()`
(AST -> `stacc.core.robot_base.Robot`) -> existing stacc exporters/simulator
(`stacc.converters.urdf_mjcf_usd.ModelTranspiler.robot_to_urdf`, `stacc.simulate`).
"""

from __future__ import annotations

from stacc.rss.ast_nodes import Block, ComponentDef, Document, NumberValue, RobotDef
from stacc.rss.compiler import SensorSpec, compile_document, compile_robot
from stacc.rss.errors import RSSCompileError, RSSError, RSSSyntaxError
from stacc.rss.parser import parse
from stacc.rss.ros2_skeleton import write_ros2_package

__all__ = [
    "parse",
    "compile_document",
    "compile_robot",
    "write_ros2_package",
    "SensorSpec",
    "Document",
    "RobotDef",
    "ComponentDef",
    "Block",
    "NumberValue",
    "RSSError",
    "RSSSyntaxError",
    "RSSCompileError",
]
