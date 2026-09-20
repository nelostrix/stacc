"""AST node types produced by the RSS parser."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union

# A parsed value: a Python str (string or bareword ident), a NumberValue
# (number, with `.unit` set when the source had a unit suffix), a VarRef
# (a `var(--name)` call, resolved against var_decls at compile time), or a
# list of Values (array literal). Kept as plain Python containers rather
# than a wrapper class so downstream code can treat it like ordinary data.
Value = Union[str, "NumberValue", "VarRef", List["Value"]]


@dataclass
class NumberValue:
    """A number literal, optionally carrying its source unit (e.g. "650mm")."""

    value: float
    unit: Optional[str] = None


@dataclass
class VarRef:
    """A `var(--name)` reference inside a property value — resolved against
    the enclosing style/appearance block's `var_decls` at compile time.
    """

    name: str  # includes the leading "--", e.g. "--primary"


@dataclass
class Selector:
    """One compound CSS-like selector inside a `style`/`appearance` block:
    `arm`, `#left_arm`, `.warning`, or `sensor[type="camera"]`. Only one of
    `element_type`-driving-a-lookup / `id` / `classes` / `attr` combination
    is typically populated per real-world rule, but nothing stops stacking
    e.g. an attribute predicate onto an element type — both are checked.
    """

    element: Optional[str] = None          # e.g. "arm", "sensor", "chassis"
    id: Optional[str] = None               # e.g. "left_arm" (from "#left_arm")
    classes: List[str] = field(default_factory=list)  # e.g. ["warning"]
    attr: Optional[Tuple[str, str]] = None  # e.g. ("type", "camera")

    def specificity(self) -> int:
        # Deliberately simple CSS-like ordering: id beats class/attr beats
        # element type. Not CSS's exact (a,b,c) tuple scheme — RSS selectors
        # never combine enough parts for that distinction to matter yet.
        if self.id:
            return 100
        if self.classes or self.attr:
            return 10
        return 1


@dataclass
class Block:
    """A named, optionally labeled `{ ... }` body — the one recursive shape
    every RSS construct (robot bodies, `arm left { ... }`, `sensors { ... }`,
    etc.) is built from.

    A block found inside a `style`/`appearance` container may additionally
    carry a `selector` (set when it was written as `#id { }`, `.class { }`,
    or `elem[attr="val"] { }` rather than the plain `name [label] { }` shape)
    — see `stacc/rss/compiler.py`'s style-application pass.
    """

    name: str
    label: Optional[str] = None
    properties: Dict[str, Value] = field(default_factory=dict)
    children: List["Block"] = field(default_factory=list)
    uses: List[str] = field(default_factory=list)
    selector: Optional[Selector] = None
    var_decls: Dict[str, Value] = field(default_factory=dict)

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
