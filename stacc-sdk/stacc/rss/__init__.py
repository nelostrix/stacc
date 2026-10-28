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
from stacc.rss.embed import (
    CLOSE_MARKER,
    OPEN_MARKER,
    EmbeddedRSS,
    compile_embedded,
    extract_rss_blocks,
    has_embedded_rss,
    load_file,
    rss,
    this_file,
)
from stacc.rss.errors import RSSCompileError, RSSError, RSSSyntaxError
from stacc.rss.importer import install as _install_importer, module
from stacc.rss.parser import parse
from stacc.rss.ros2_skeleton import write_ros2_package

__all__ = [
    "parse",
    "compile_document",
    "compile_robot",
    "write_ros2_package",
    "module",
    "rss",
    "this_file",
    "load_file",
    "compile_embedded",
    "extract_rss_blocks",
    "has_embedded_rss",
    "EmbeddedRSS",
    "OPEN_MARKER",
    "CLOSE_MARKER",
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

# `import stacc.rss` makes .rss files importable: `from design import robot, joints`.
_install_importer()
