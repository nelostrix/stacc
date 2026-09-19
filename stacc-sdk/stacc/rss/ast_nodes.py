"""AST node types produced by the RSS parser."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union

# A parsed value: a Python str (string or bareword ident), a float (number,
# with `.unit` set when the source had a unit suffix), or a list of Values
# (array literal). Kept as plain Python containers rather than a wrapper
# class so downstream code can treat it like ordinary data.
Value = Union[str, float, List["Value"]]


@dataclass
class NumberValue:
    """A number literal, optionally carrying its source unit (e.g. "650mm")."""

    value: float
    unit: Optional[str] = None


@dataclass
class Block:
    """A named, optionally labeled `{ ... }` body — the one recursive shape
    every RSS construct (robot bodies, `arm left { ... }`, `sensors { ... }`,
    etc.) is built from.
    """

    name: str
    label: Optional[str] = None
    properties: Dict[str, Value] = field(default_factory=dict)
    children: List["Block"] = field(default_factory=list)
    uses: List[str] = field(default_factory=list)

    def child(self, name: str) -> Optional["Block"]:
        for c in self.children:
            if c.name == name:
                return c
        return None

    def children_named(self, *names: str) -> List["Block"]:
        return [c for c in self.children if c.name in names]


@dataclass
class ComponentDef:
    name: str
    block: Block


@dataclass
class RobotDef:
    name: str
    block: Block


@dataclass
class Document:
    components: Dict[str, ComponentDef] = field(default_factory=dict)
    robots: List[RobotDef] = field(default_factory=list)
