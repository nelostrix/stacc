"""Compiles a parsed RSS `Document`/`RobotDef` into a `stacc.core.robot_base.Robot`.

Semantic mapping (deliberately simple for the MVP — see the plan for the
documented fast-follows):

- `body` / `chassis` block  -> the robot's root `Link`.
- `arm` / `leg` / `wheel` / `gripper` / `head` blocks (whether directly under
  the robot, or nested one level inside an `actuators { ... }` container)
  -> one child `Link` + `Joint` each, attached to the root link. A named
  limb block produces a single Link+Joint pair, not an N-DOF serial chain,
  even when a `joints: N` property is given — that's stored as metadata
  instead. `wheel` gets a `continuous` joint; everything else `revolute`.
- `sensors { ... }` block -> each child becomes a `SensorSpec` attached via
  `Robot.attach_sensor`.
- `appearance` / `style` block -> `color` on a child whose name matches the
  root block's name (or an existing limb's link name) is applied to that
  link's `Geometry.color`.
- Anything else (payload, actuator/motor names, sensor models, fov, ...)
  that doesn't map onto a stacc dataclass field is preserved verbatim on
  `robot.rss_metadata[link_name]` so it isn't silently dropped.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, FrozenSet, List, Optional, Tuple

from stacc.core.link import Geometry, Inertia, Link
from stacc.core.joint import Joint
from stacc.core.robot_base import Robot
from stacc.rss.ast_nodes import Block, ComponentDef, Document, NumberValue, RobotDef, Value
from stacc.rss.errors import RSSCompileError
from stacc.rss.parser import parse

ROOT_BLOCK_NAMES = {"body", "chassis"}
LIMB_BLOCK_NAMES = {"arm", "leg", "wheel", "gripper", "head"}
CONTAINER_BLOCK_NAMES = {"actuators"}
SENSOR_CONTAINER_NAMES = {"sensors"}
APPEARANCE_CONTAINER_NAMES = {"appearance", "style"}
KNOWN_GEOMETRY_SHAPES = {"box", "cylinder", "sphere", "mesh"}
MOBILE_SHAPES = {"differential-drive", "ackermann", "mecanum", "omni", "omnidirectional"}

_LENGTH_TO_M = {"mm": 0.001, "cm": 0.01, "m": 1.0}
_MASS_TO_KG = {"g": 0.001, "kg": 1.0}
_ANGLE_TO_RAD = {"deg": math.pi / 180.0, "rad": 1.0}


@dataclass
class SensorSpec:
    """A sensor declared in RSS, kept generic since RSS doesn't (yet) map
    model strings like "rplidar-a2" onto stacc's concrete sensor driver
    classes in stacc.sensors.*.

    `enabled = False` (with no-op `should_update`/`read`) is a deliberate
    MVP simplification: there's no real driver behind this sensor, so the
    sim loop (`stacc/sim/engine.py`'s `Robot.sensors` iteration, which
    expects `.enabled`/`.should_update(t)`/`.read()`) must treat it as
    inert rather than crash on it.
    """

    kind: str
    label: Optional[str]
    properties: Dict[str, Any] = field(default_factory=dict)
    enabled: bool = False

    def should_update(self, t: float) -> bool:  # pragma: no cover - inert stub
        return False

    def read(self) -> None:  # pragma: no cover - inert stub
        return None

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return f"SensorSpec(kind={self.kind!r}, label={self.label!r})"


def normalize(value: Value) -> Any:
    """Convert a parsed Value into plain Python data, normalizing units to SI."""
    if isinstance(value, NumberValue):
        if value.unit is None:
            return value.value
        if value.unit in _LENGTH_TO_M:
            return value.value * _LENGTH_TO_M[value.unit]
        if value.unit in _MASS_TO_KG:
            return value.value * _MASS_TO_KG[value.unit]
        if value.unit in _ANGLE_TO_RAD:
            return value.value * _ANGLE_TO_RAD[value.unit]
        # m/s, deg/s, rad/s, or an unrecognized unit — pass the number through
        return value.value
    if isinstance(value, list):
        return [normalize(v) for v in value]
    return value  # already a plain str


def _hex_to_rgba(color: str) -> Optional[List[float]]:
    s = color.lstrip("#")
    if len(s) not in (6, 8):
        return None
    try:
        r = int(s[0:2], 16) / 255.0
        g = int(s[2:4], 16) / 255.0
        b = int(s[4:6], 16) / 255.0
        a = int(s[6:8], 16) / 255.0 if len(s) == 8 else 1.0
        return [r, g, b, a]
    except ValueError:
        return None


# ══════════════════════════════════════════
# `use` resolution
# ══════════════════════════════════════════


def _load_external_component(
    ref: str, search_paths: List[Path], file_cache: Dict[str, Document]
) -> Tuple[ComponentDef, Dict[str, ComponentDef]]:
    """Resolve a cross-file `use "path/to/library.rss:component-name";`
    reference. `path/to/library.rss` is looked up relative to each of
    `search_paths` in order (typically: the directory of the .rss file
    currently being compiled, plus any --rss-path directories) — this is
    the actual "import a stylesheet from elsewhere" behavior; a bare `use
    name;` only ever sees components defined in the same file.
    """
    if ":" not in ref:
        raise RSSCompileError(
            f"unknown component '{ref}' (referenced via 'use'); for a cross-file "
            f"reference use 'use \"path/to/file.rss:component-name\";'"
        )
    file_part, comp_name = ref.rsplit(":", 1)

    resolved_path = None
    for base in search_paths:
        candidate = Path(base) / file_part
        if candidate.exists():
            resolved_path = candidate
            break
    if resolved_path is None:
        searched = ", ".join(str(p) for p in search_paths) or "(none)"
        raise RSSCompileError(
            f"could not find '{file_part}' for 'use \"{ref}\";' — searched: {searched}"
        )

    cache_key = str(resolved_path.resolve())
    if cache_key not in file_cache:
        file_cache[cache_key] = parse(resolved_path.read_text())
    doc = file_cache[cache_key]

    if comp_name not in doc.components:
        raise RSSCompileError(f"component '{comp_name}' not found in {resolved_path}")
    return doc.components[comp_name], doc.components


def _resolve_component(
    name: str,
    components: Dict[str, ComponentDef],
    seen: FrozenSet[str],
    search_paths: List[Path],
    file_cache: Dict[str, Document],
) -> Block:
    if name in components:
        if name in seen:
            raise RSSCompileError(f"cyclic component reference involving '{name}'")
        return _resolve_block_uses(
            components[name].block, components, seen | {name}, search_paths, file_cache
        )

    if ":" in name and search_paths:
        comp_def, ext_components = _load_external_component(name, search_paths, file_cache)
        if name in seen:
            raise RSSCompileError(f"cyclic component reference involving '{name}'")
        return _resolve_block_uses(
            comp_def.block, ext_components, seen | {name}, search_paths, file_cache
        )

    raise RSSCompileError(f"unknown component '{name}' (referenced via 'use')")


def _resolve_block_uses(
    block: Block,
    components: Dict[str, ComponentDef],
    seen: FrozenSet[str],
    search_paths: List[Path],
    file_cache: Dict[str, Document],
) -> Block:
    """Merge `use` directives: a used component's properties become defaults
    (the block's own properties win), and its child blocks are prepended.
    Applied recursively at every nesting depth.
    """
    merged_properties = dict(block.properties)
    merged_children = list(block.children)

    for used_name in block.uses:
        used_block = _resolve_component(used_name, components, seen, search_paths, file_cache)
        for k, v in used_block.properties.items():
            merged_properties.setdefault(k, v)
        merged_children = list(used_block.children) + merged_children

    resolved_children = [
        _resolve_block_uses(c, components, seen, search_paths, file_cache)
        for c in merged_children
    ]
    return Block(
        name=block.name,
        label=block.label,
        properties=merged_properties,
        children=resolved_children,
        uses=[],
    )


# ══════════════════════════════════════════
# Block -> Link / Geometry
# ══════════════════════════════════════════


def _prop(block: Block, key: str, default: Any = None) -> Any:
    if key in block.properties:
        return normalize(block.properties[key])
    return default


def _block_metadata(block: Block) -> Dict[str, Any]:
    return {k: normalize(v) for k, v in block.properties.items()}


def _block_to_geometry(block: Block, default_length: Optional[float] = None) -> Geometry:
    """`default_length` is used to size an otherwise-unspecified segment of a
    multi-joint limb chain (see `_limb_segment_lengths`) — ignored whenever
    the block gives its own shape/dimensions/radius/length, so single-joint
    limbs (the common case) are unaffected.
    """
    shape = _prop(block, "shape")
    dims = _prop(block, "dimensions")
    radius = _prop(block, "radius")
    length = _prop(block, "length")
    color_val = _prop(block, "color")

    if shape in KNOWN_GEOMETRY_SHAPES:
        geom_shape = shape
    elif radius is not None:
        geom_shape = "cylinder"
    else:
        geom_shape = "box"

    kwargs: Dict[str, Any] = {}
    if geom_shape == "box":
        if isinstance(dims, list) and len(dims) == 3:
            kwargs["size"] = dims
        elif default_length is not None:
            kwargs["size"] = [0.05, 0.05, default_length]
    if geom_shape == "cylinder":
        kwargs["radius"] = radius if radius is not None else 0.03
        kwargs["length"] = length if length is not None else (default_length or 0.1)
    if geom_shape == "sphere" and radius is not None:
        kwargs["radius"] = radius

    geometry = Geometry(shape=geom_shape, **kwargs)
    if isinstance(color_val, str):
        rgba = _hex_to_rgba(color_val)
        if rgba:
            geometry.color = rgba
    return geometry


def _block_to_link(block: Block, name: str, default_length: Optional[float] = None) -> Link:
    mass = _prop(block, "mass", 1.0)
    if not isinstance(mass, (int, float)):
        mass = 1.0
    geometry = _block_to_geometry(block, default_length=default_length)
    inertia = Inertia.from_box(float(mass), 0.1, 0.1, 0.1)
    return Link(name=name, mass=float(mass), inertia=inertia, visual=geometry, collision=geometry)


def _limb_joint_count(limb: Block) -> int:
    n = _prop(limb, "joints")
    if isinstance(n, (int, float)) and n >= 1:
        return int(n)
    return 1


def _limb_segment_lengths(limb: Block, n: int) -> List[float]:
    """Split a limb's `reach` (already normalized to meters) evenly across
    its `joints: N` segments; fall back to a generic per-segment length
    when no reach is given.
    """
    reach = _prop(limb, "reach")
    if isinstance(reach, (int, float)) and reach > 0:
        return [reach / n] * n
    return [0.15] * n


def _degrees_value(block: Block, key: str, default: float) -> float:
    """Read a property as degrees regardless of how it was written — RSS
    normalizes `deg`-suffixed numbers to radians for Joint/Link consistency,
    but stacc's sensor drivers (LiDAR/Camera fov) take plain degrees.
    """
    raw = block.properties.get(key)
    if isinstance(raw, NumberValue):
        if raw.unit == "rad":
            return math.degrees(raw.value)
        return raw.value  # "deg" or unitless — already degrees
    return default


def _try_build_real_sensor(sub: Block):
    """Instantiate a real stacc.sensors driver for a recognized sensor
    `kind` (block name), or return None to fall back to the inert
    SensorSpec. Keeping this narrow and explicit — only wire kinds whose
    constructor shape we've actually checked — rather than guessing at
    every `model:` string in the wild.
    """
    try:
        from stacc.sensors.camera import Camera
        from stacc.sensors.imu import IMU
        from stacc.sensors.lidar import LiDAR
    except ImportError:
        return None

    kind = sub.name
    if kind == "lidar":
        return LiDAR(
            range_max=_prop(sub, "range", 30.0),
            fov_h=_degrees_value(sub, "fov", 360.0),
        )
    if kind == "camera":
        return Camera(fov=_degrees_value(sub, "fov", 60.0))
    if kind == "imu":
        return IMU()
    return None


def _build_sensor(sub: Block):
    """Build the sensor object attached for an RSS sensor sub-block: a real
    stacc.sensors driver when `sub.name` is a kind we know how to map
    (so `stacc rss run --sim` produces real MuJoCo-backed sensor data),
    else the generic inert SensorSpec so nothing is silently dropped.
    """
    real = _try_build_real_sensor(sub)
    if real is not None:
        # Give real drivers the same `.kind`/`.label`/`.properties` surface
        # SensorSpec has, so callers (stacc rss check, BOM, tests) don't need
        # to special-case type, and metadata like `model:` that didn't map
        # onto a constructor kwarg (e.g. "rplidar-a2") isn't lost.
        real.kind = sub.name
        real.label = sub.label
        real.properties = _block_metadata(sub)
        return real
    return SensorSpec(kind=sub.name, label=sub.label, properties=_block_metadata(sub))


def _unique_name(base: str, taken: set) -> str:
    name = base
    i = 2
    while name in taken:
        name = f"{base}_{i}"
        i += 1
    return name


# ══════════════════════════════════════════
# Robot compilation
# ══════════════════════════════════════════


def compile_robot(
    robot_def: RobotDef,
    components: Dict[str, ComponentDef],
    search_paths: Optional[List[Path]] = None,
) -> Robot:
    """`search_paths` enables cross-file `use "path/to/lib.rss:name";` —
    typically [the .rss file's own directory] plus any --rss-path
    directories; omit it (or pass []) to only allow same-file components.
    """
    file_cache: Dict[str, Document] = {}
    resolved = _resolve_block_uses(
        robot_def.block, components, frozenset(), search_paths or [], file_cache
    )

    robot = Robot(name=robot_def.name)
    # Properties that land directly on the robot's own root block (as opposed
    # to a body/chassis/limb child) — this is where a top-level `use` pulls in
    # component defaults like `joints: 6` or `payload: 10kg` that don't map
    # onto a specific stacc dataclass field.
    metadata: Dict[str, Dict[str, Any]] = {"__robot__": _block_metadata(resolved)}
    all_links: Dict[str, Link] = {}
    used_names: set = set()

    # ── root link (body / chassis) ──
    root_block: Optional[Block] = None
    for candidate in ROOT_BLOCK_NAMES:
        root_block = resolved.child(candidate)
        if root_block is not None:
            break

    root_name = "base"
    used_names.add(root_name)
    if root_block is not None:
        root_link = _block_to_link(root_block, root_name)
        shape = _prop(root_block, "shape")
        metadata[root_name] = _block_metadata(root_block)
    else:
        root_link = Link(name=root_name, mass=2.0)
        shape = None

    robot.add_link(root_link)
    robot.fixed_base = shape not in MOBILE_SHAPES
    all_links[root_name] = root_link

    # ── limbs: direct named blocks, plus one level inside `actuators { }` ──
    limb_blocks: List[Block] = list(resolved.children_named(*LIMB_BLOCK_NAMES))
    for container in resolved.children_named(*CONTAINER_BLOCK_NAMES):
        limb_blocks += list(container.children_named(*LIMB_BLOCK_NAMES))

    for limb in limb_blocks:
        base_name = f"{limb.name}_{limb.label}" if limb.label else limb.name
        joint_type = "continuous" if limb.name == "wheel" else "revolute"
        n_joints = _limb_joint_count(limb)
        segment_lengths = _limb_segment_lengths(limb, n_joints)
        limb_meta = _block_metadata(limb)

        parent_name = root_name
        prev_length = 0.0
        chain_link_names: List[str] = []
        for seg_idx in range(n_joints):
            # A single-joint limb keeps the plain "arm_left" name for
            # backward compatibility; a multi-joint chain numbers its
            # segments "arm_left_1", "arm_left_2", ...
            seg_base = f"{base_name}_{seg_idx + 1}" if n_joints > 1 else base_name
            link_name = _unique_name(seg_base, used_names)
            used_names.add(link_name)

            seg_length = segment_lengths[seg_idx]
            link = _block_to_link(limb, link_name, default_length=seg_length)
            # Alternate rotation axes down the chain so a multi-joint limb
            # doesn't collapse into redundant parallel hinges.
            axis = [0.0, 1.0, 0.0] if seg_idx % 2 == 0 else [0.0, 0.0, 1.0]
            joint = Joint(
                name=f"{link_name}_joint",
                type=joint_type,
                parent_link=parent_name,
                child_link=link_name,
                axis=axis,
            )
            # First segment attaches wherever the parent link's own geometry
            # implies (offset=None, same as the old single-joint behavior);
            # each later segment sits `prev_length` along Z from the one
            # before it, forming an actual serial chain instead of every
            # segment stacking at the root.
            offset = None if seg_idx == 0 else [0.0, 0.0, prev_length]
            robot.attach(parent_name, joint, link, offset=offset)

            all_links[link_name] = link
            metadata[link_name] = limb_meta
            chain_link_names.append(link_name)
            parent_name = link_name
            prev_length = seg_length

        # Sub-blocks of a limb that aren't structural (e.g. head { display{} camera{} })
        # become sensors mounted on the last (distal) segment of the chain.
        end_link_name = chain_link_names[-1]
        for sub in limb.children:
            robot.attach_sensor(_build_sensor(sub), position=end_link_name)

    # ── sensors { lidar front { ... } camera front { ... } } ──
    for sensors_block in resolved.children_named(*SENSOR_CONTAINER_NAMES):
        for s in sensors_block.children:
            robot.attach_sensor(_build_sensor(s), position=s.label or "body")

    # ── appearance / style: apply color to a matching link, keep the rest as metadata ──
    for appearance_block in resolved.children_named(*APPEARANCE_CONTAINER_NAMES):
        appearance_meta: Dict[str, Any] = metadata.setdefault("__appearance__", {})
        for sub in appearance_block.children:
            target_name = None
            if root_block is not None and sub.name == root_block.name:
                target_name = root_name
            elif sub.name in all_links:
                target_name = sub.name

            color_val = _prop(sub, "color")
            if target_name and isinstance(color_val, str):
                rgba = _hex_to_rgba(color_val)
                if rgba:
                    link_obj = all_links[target_name]
                    if link_obj.visual:
                        link_obj.visual.color = rgba
                    if link_obj.collision:
                        link_obj.collision.color = rgba
            appearance_meta[sub.name] = _block_metadata(sub)

    # ── anything else at the top level (e.g. `navigation { ... }`) isn't a
    # recognized block type, so nothing gets built from it — but its
    # properties are still captured as metadata (never silently dropped),
    # and it's surfaced as a warning so a typo'd/unsupported block name is
    # visible instead of just quietly doing nothing.
    recognized_top_level = (
        ({root_block.name} if root_block is not None else set())
        | LIMB_BLOCK_NAMES
        | CONTAINER_BLOCK_NAMES
        | SENSOR_CONTAINER_NAMES
        | APPEARANCE_CONTAINER_NAMES
    )
    warnings: List[str] = []
    for child in resolved.children:
        if child.name in recognized_top_level:
            continue
        metadata[child.name] = _block_metadata(child)
        label_part = f" {child.label}" if child.label else ""
        warnings.append(
            f"block '{child.name}{label_part}' isn't a recognized block type "
            f"(body/chassis/arm/leg/wheel/gripper/head/actuators/sensors/"
            f"appearance/style) — nothing was built from it, but its properties "
            f"were kept in robot.rss_metadata['{child.name}']"
        )

    robot.rss_metadata = metadata
    robot.rss_warnings = warnings
    return robot


def compile_document(
    doc: Document,
    robot_name: Optional[str] = None,
    search_paths: Optional[List[Path]] = None,
) -> Robot:
    """Compile one robot out of a parsed Document.

    Args:
        robot_name: which `robot "X" { ... }` to compile, when the document
            defines more than one. Required in that case.
        search_paths: directories to look in for cross-file
            `use "path/to/lib.rss:name";` references (see `compile_robot`).
    """
    if not doc.robots:
        raise RSSCompileError("document contains no 'robot' definitions")

    if robot_name is not None:
        for r in doc.robots:
            if r.name == robot_name:
                return compile_robot(r, doc.components, search_paths)
        raise RSSCompileError(f"no robot named '{robot_name}' in document")

    if len(doc.robots) > 1:
        names = ", ".join(r.name for r in doc.robots)
        raise RSSCompileError(
            f"document defines multiple robots ({names}); pass robot_name= to pick one"
        )

    return compile_robot(doc.robots[0], doc.components, search_paths)
