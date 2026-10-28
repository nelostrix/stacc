"""Import `.rss` files as Python modules.

    import stacc.rss                       # installs the importer
    from design import robot, joints, classes

    robot                                  # the compiled stacc Robot
    joints.arm_main_1                      # a Joint, also joints["arm_main_1"]
    [j.name for j in classes.lead_leg.joints]
    components["industrial_arm"]           # a component's properties, unit-normalized

`design.rss` is found like any module: next to the running script, on
`sys.path`, or inside a package (`from robots.design import robot`). Regular
`.py` modules of the same name win. `importlib.reload(design)` recompiles the
file after an edit.

A path that isn't importable by name (`./parts/arm-v2.rss`) loads with
`stacc.rss.module("./parts/arm-v2.rss")`, resolved against the calling file.

Module contents:
    robot        the robot, when the file defines exactly one
    <name>       every robot, by its own name (when that's a valid identifier)
    robots       {name: Robot}
    joints, links, sensors
                 the single robot's parts, in compile order, by name
    classes      RSS `class:` names -> .links / .joints tagged with that class
    components   {name: {property: value}} for every `component` definition
    warnings     compile warnings (also robot.rss_warnings)
    source       the file's text
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import importlib.util
import inspect
import keyword
import sys
import types
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Union

from stacc.rss.compiler import compile_robot, normalize
from stacc.rss.parser import parse

SUFFIX = ".rss"


class Parts:
    """Named parts in compile order: `parts.name`, `parts["name"]`, iteration, `len()`."""

    def __init__(self, kind: str, items: Sequence[Any]):
        self._kind = kind
        self._items: Dict[str, Any] = {getattr(item, "name", str(i)): item for i, item in enumerate(items)}

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self._items[name]
        except KeyError:
            raise AttributeError(f"no {self._kind} named '{name}' (have: {', '.join(self._items) or 'none'})") from None

    def __getitem__(self, name: str) -> Any:
        try:
            return self._items[name]
        except KeyError:
            raise KeyError(f"no {self._kind} named '{name}'") from None

    def __contains__(self, name: object) -> bool:
        return name in self._items

    def __iter__(self) -> Iterator[Any]:
        return iter(self._items.values())

    def __len__(self) -> int:
        return len(self._items)

    def names(self) -> List[str]:
        return list(self._items)

    def __dir__(self) -> List[str]:
        return [n for n in self._items if n.isidentifier()]

    def __repr__(self) -> str:
        return f"<{self._kind}s: {', '.join(self._items)}>"


class ClassGroup:
    """Everything one RSS `class:` name was applied to."""

    def __init__(self, name: str, links: List[Any], joints: List[Any]):
        self.name = name
        self.links = links
        self.joints = joints

    def __repr__(self) -> str:
        return f"<class .{self.name}: {len(self.links)} links, {len(self.joints)} joints>"


def _classes(robot: Any) -> Parts:
    link_classes: Dict[str, List[str]] = getattr(robot, "rss_classes", {}) or {}
    by_class: Dict[str, List[str]] = {}
    for link_name, names in link_classes.items():
        for c in names:
            by_class.setdefault(c, []).append(link_name)
    groups = []
    for c, link_names in by_class.items():
        tagged = set(link_names)
        links = [l for l in robot.links if l.name in tagged]
        joints = [j for j in robot.joints if j.child_link in tagged]
        groups.append(ClassGroup(c, links, joints))
    return Parts("class", groups)


def _component_properties(block: Any) -> Dict[str, Any]:
    return {k: normalize(v) for k, v in block.properties.items()}


def populate(module: types.ModuleType, path: Path, source: str) -> None:
    """Compile `source` (the text of `path`) and expose its robots and parts on `module`."""
    doc = parse(source)
    search = [path.resolve().parent]
    robots = {r.name: compile_robot(r, doc.components, search) for r in doc.robots}
    ns = module.__dict__
    ns["source"] = source
    ns["robots"] = robots
    ns["components"] = {name: _component_properties(c.block) for name, c in doc.components.items()}
    ns["warnings"] = [w for r in robots.values() for w in getattr(r, "rss_warnings", [])]
    for name, robot in robots.items():
        if name.isidentifier() and not keyword.iskeyword(name) and name not in ns:
            ns[name] = robot
    if len(robots) == 1:
        robot = next(iter(robots.values()))
        ns["robot"] = robot
        ns["joints"] = Parts("joint", robot.joints)
        ns["links"] = Parts("link", robot.links)
        ns["sensors"] = Parts("sensor", robot.sensors)
        ns["classes"] = _classes(robot)
    else:
        many = ", ".join(robots) or "none"

        def __getattr__(attr: str) -> Any:
            if attr in ("robot", "joints", "links", "sensors", "classes"):
                raise AttributeError(
                    f"{path.name} defines {len(robots)} robots ({many}); use robots['name'] or the robot's own name instead of '{attr}'"
                )
            raise AttributeError(f"module '{module.__name__}' has no attribute '{attr}'")

        ns["__getattr__"] = __getattr__


class RssLoader(importlib.abc.Loader):
    def __init__(self, path: Path):
        self.path = path

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> Optional[types.ModuleType]:
        return None  # default module creation

    def exec_module(self, module: types.ModuleType) -> None:
        populate(module, self.path, self.path.read_text())


class RssFinder(importlib.abc.MetaPathFinder):
    """Finds `<name>.rss` wherever `<name>.py` would be looked for."""

    def find_spec(self, fullname: str, path: Optional[Sequence[str]], target: Any = None):
        leaf = fullname.rpartition(".")[2]
        for entry in path if path is not None else sys.path:
            candidate = Path(entry or ".") / f"{leaf}{SUFFIX}"
            if candidate.is_file():
                return importlib.util.spec_from_file_location(fullname, candidate, loader=RssLoader(candidate))
        return None


_FINDER = RssFinder()


def install() -> None:
    """Make `.rss` files importable. Runs when `stacc.rss` is imported; safe to call again."""
    if _FINDER not in sys.meta_path:
        # After the standard finders, so a .py module of the same name always wins.
        sys.meta_path.append(_FINDER)


def module(path: Union[str, Path], name: Optional[str] = None) -> types.ModuleType:
    """Load an `.rss` file by path (relative to the calling file) as a module."""
    p = Path(path)
    if not p.is_absolute():
        caller = inspect.stack()[1].filename
        base = Path(caller).resolve().parent if caller and not caller.startswith("<") else Path.cwd()
        p = (base / p).resolve()
    if p.suffix != SUFFIX:
        raise ValueError(f"expected a {SUFFIX} file, got {p}")
    loader = RssLoader(p)
    spec = importlib.util.spec_from_file_location(name or p.stem.replace("-", "_"), p, loader=loader)
    assert spec is not None
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod
