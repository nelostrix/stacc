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
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, FrozenSet, List, Optional, Tuple

from stacc.core.link import Geometry, Inertia, Link
from stacc.core.joint import Joint
from stacc.core.robot_base import Robot
from stacc.rss.ast_nodes import Block, ComponentDef, Document, NumberValue, RobotDef, Value, VarRef
from stacc.rss.errors import RSSCompileError
from stacc.rss.parser import parse

ROOT_BLOCK_NAMES = {"body", "chassis"}
LIMB_BLOCK_NAMES = {"arm", "leg", "wheel", "gripper", "head", "rotor"}
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

    @property
    def name(self) -> str:
        """Same naming the real drivers use, so telemetry, capabilities and the
        sensor list treat this inert sensor like any other instead of crashing."""
        return f"{self.kind}_{self.label}" if self.label else self.kind

    def should_update(self, t: float) -> bool:  # pragma: no cover - inert stub
        return False

    def reset(self) -> None:  # pragma: no cover - inert stub
        return None

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


# A small, deliberately non-exhaustive set of CSS-style named colors —
# common enough (black/white/red/...) that requiring hex for them would be
# an odd papercut for a language explicitly modeled on CSS.
_NAMED_COLORS = {
    "black": "#000000", "white": "#ffffff", "red": "#ff0000",
    "green": "#008000", "blue": "#0000ff", "yellow": "#ffff00",
    "orange": "#ffa500", "purple": "#800080", "gray": "#808080",
    "grey": "#808080", "cyan": "#00ffff", "magenta": "#ff00ff",
    "silver": "#c0c0c0", "gold": "#ffd700", "brown": "#a52a2a",
    "transparent": "#00000000",
}


def _hex_to_rgba(color: str) -> Optional[List[float]]:
    named = _NAMED_COLORS.get(color.lower())
    s = (named or color).lstrip("#")
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


# Named `material:` presets — real metallic/roughness/tint defaults, not
# just a label kept in metadata. An explicit `color:`/`metallic:`/
# `roughness:` alongside `material:` on the same block still wins (applied
# after the preset in `_apply_visual_properties`), so the preset is only
# ever a starting point, never a silent override of what you actually wrote.
MATERIAL_PRESETS: Dict[str, Dict[str, Any]] = {
    "aluminum":     {"color": [0.75, 0.76, 0.78, 1.0], "metallic": 0.9, "roughness": 0.35},
    "steel":        {"color": [0.56, 0.57, 0.58, 1.0], "metallic": 0.85, "roughness": 0.3},
    "brushed_steel": {"color": [0.6, 0.6, 0.62, 1.0], "metallic": 0.8, "roughness": 0.5},
    "brass":        {"color": [0.71, 0.65, 0.26, 1.0], "metallic": 0.85, "roughness": 0.3},
    "copper":       {"color": [0.72, 0.45, 0.2, 1.0], "metallic": 0.9, "roughness": 0.25},
    "carbon_fiber": {"color": [0.08, 0.08, 0.09, 1.0], "metallic": 0.2, "roughness": 0.4},
    "plastic":      {"color": [0.8, 0.8, 0.8, 1.0], "metallic": 0.0, "roughness": 0.6},
    "rubber":       {"color": [0.1, 0.1, 0.1, 1.0], "metallic": 0.0, "roughness": 0.9},
    "wood":         {"color": [0.55, 0.37, 0.2, 1.0], "metallic": 0.0, "roughness": 0.7},
    "glass":        {"color": [0.85, 0.9, 0.95, 0.25], "metallic": 0.0, "roughness": 0.05},
    "brushless":    {"color": [0.15, 0.15, 0.16, 1.0], "metallic": 0.7, "roughness": 0.4},
    "servo":        {"color": [0.2, 0.2, 0.22, 1.0], "metallic": 0.3, "roughness": 0.5},
}


def _apply_visual_properties(geo: Geometry, props: Dict[str, Any]) -> None:
    """Mutate a Geometry's real visual fields from resolved RSS properties.
    Shared by initial construction (`_block_to_geometry`) and the style
    cascade (`_apply_style_rules`) so `material:`/`color:`/`metallic:`/
    `roughness:`/`emission:`/`transparency:`/`opacity:` behave identically
    whether declared directly on a structural block or via a selector rule.
    """
    material_name = props.get("material")
    if isinstance(material_name, str) and material_name in MATERIAL_PRESETS:
        preset = MATERIAL_PRESETS[material_name]
        geo.color = list(preset["color"])
        geo.metallic = preset["metallic"]
        geo.roughness = preset["roughness"]
        geo.material_name = material_name

    color_val = props.get("color")
    if isinstance(color_val, str):
        rgba = _hex_to_rgba(color_val)
        if rgba:
            geo.color = rgba

    metallic_val = props.get("metallic")
    if isinstance(metallic_val, (int, float)):
        geo.metallic = max(0.0, min(1.0, float(metallic_val)))

    roughness_val = props.get("roughness")
    if isinstance(roughness_val, (int, float)):
        geo.roughness = max(0.0, min(1.0, float(roughness_val)))

    emission_val = props.get("emission")
    if isinstance(emission_val, str):
        rgba = _hex_to_rgba(emission_val)
        if rgba:
            geo.emission = rgba[:3]

    # transparency: 0 (opaque) .. 1 (fully transparent); opacity is its
    # inverse. Either can adjust the alpha channel of whatever color is
    # already set (from this same call or an earlier one).
    transparency_val = props.get("transparency")
    if isinstance(transparency_val, (int, float)):
        geo.color = [geo.color[0], geo.color[1], geo.color[2], max(0.0, min(1.0, 1.0 - float(transparency_val)))]
    opacity_val = props.get("opacity")
    if isinstance(opacity_val, (int, float)):
        geo.color = [geo.color[0], geo.color[1], geo.color[2], max(0.0, min(1.0, float(opacity_val)))]


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
        selector=block.selector,
        var_decls=dict(block.var_decls),
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


def _classes_of(block: Block) -> List[str]:
    """CSS-style classes for selector matching (`.warning { ... }`) — declared
    on the structural block itself as `class: name;` or `classes: [a, b];`.
    Both spellings are accepted since either reads naturally depending on
    whether there's one or several.
    """
    classes: List[str] = []
    single = _prop(block, "class")
    if isinstance(single, str):
        classes.append(single)
    multi = _prop(block, "classes")
    if isinstance(multi, list):
        classes.extend(str(c) for c in multi if isinstance(c, str))
    return classes


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
    _apply_visual_properties(geometry, _block_metadata(block))
    return geometry


def _inertia_for_geometry(mass: float, geometry: Geometry) -> Inertia:
    """Solid-body inertia derived from the link's own geometry.

    AC4 (docs/PLATFORM_SPEC.md §9): a dimension edit must move the mass
    properties, not just the drawn mesh — so the tensor is derived from the
    geometry we just built rather than a fixed 0.1 m placeholder cube (which
    made every link's inertia independent of its actual size).
    """
    if geometry.shape == "cylinder":
        return Inertia.from_cylinder(mass, float(geometry.radius), float(geometry.length))
    if geometry.shape == "sphere":
        return Inertia.from_sphere(mass, float(geometry.radius))
    if geometry.shape == "mesh":
        scale = geometry.mesh_scale or [0.1, 0.1, 0.1]
        return Inertia.from_box(mass, float(scale[0]), float(scale[1]), float(scale[2]))
    size = geometry.size or [0.1, 0.1, 0.1]
    return Inertia.from_box(mass, float(size[0]), float(size[1]), float(size[2]))


def _block_to_link(block: Block, name: str, default_length: Optional[float] = None) -> Link:
    mass = _prop(block, "mass", 1.0)
    if not isinstance(mass, (int, float)):
        mass = 1.0
    geometry = _block_to_geometry(block, default_length=default_length)
    inertia = _inertia_for_geometry(float(mass), geometry)
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


def _resolve_vars(value: Any, variables: Dict[str, Any]) -> Any:
    """Substitute any `var(--name)` in a parsed value against already-resolved
    custom properties. Declaration order matters: a `--x` can reference an
    earlier `--y` in the same style block, not a later one (same as CSS
    custom properties evaluate top-to-bottom).
    """
    if isinstance(value, VarRef):
        if value.name not in variables:
            raise RSSCompileError(
                f"undefined custom property '{value.name}' (referenced via var()) "
                f"— declare it first with '{value.name}: ...;'"
            )
        return variables[value.name]
    if isinstance(value, list):
        return [_resolve_vars(v, variables) for v in value]
    return value


def _apply_style_rules(
    appearance_blocks: List[Block],
    root_block: Optional[Block],
    root_name: str,
    all_links: Dict[str, Link],
    link_kind: Dict[str, str],
    link_id: Dict[str, str],
    link_classes: Dict[str, List[str]],
    robot: Robot,
    metadata: Dict[str, Dict[str, Any]],
) -> None:
    """Apply every `appearance`/`style` rule as a real cascade: resolve
    `--vars`, match each rule's selector against the compiled robot's
    links/joints/sensors, then apply in (specificity, source-order) so
    higher-specificity and later rules correctly win — the same idea `color:`
    already had for one exact-name match, generalized to real selectors
    (#id / .class / elem[attr="value"] / bare element-type) plus variables.

    Only `color` has a real effect today (the one visual property RSS's
    Geometry actually carries through to MJCF/URDF) — every other declared
    property is still captured faithfully in `robot.rss_metadata`, exactly
    like the original single-match version, rather than pretending a
    `material:`/`emission:` property does something the renderer doesn't
    support yet.
    """
    appearance_meta: Dict[str, Any] = metadata.setdefault("__appearance__", {})
    element_types = ROOT_BLOCK_NAMES | LIMB_BLOCK_NAMES

    # (specificity, source_order, target_kind, target, resolved_properties)
    applications: List[Tuple[int, int, str, Any, Dict[str, Any]]] = []
    order = 0

    for appearance_block in appearance_blocks:
        variables: Dict[str, Any] = {}
        for var_name, raw_value in appearance_block.var_decls.items():
            variables[var_name] = normalize(_resolve_vars(raw_value, variables))

        for sub in appearance_block.children:
            resolved_props = {
                k: normalize(_resolve_vars(v, variables)) for k, v in sub.properties.items()
            }
            appearance_meta[sub.name] = resolved_props
            selector = sub.selector

            if selector is None:
                # Legacy shape: a bare `name { ... }` child, checked from most
                # to least specific so a real match is never shadowed by a
                # coincidentally-named element type:
                #   1. names an exact compiled link (spelling out "arm_left"
                #      directly — the original, only way to target one part).
                #   2. names the root block's own name ("chassis"/"body").
                #   3. is a recognized element-type keyword -> every element
                #      of that type (new: previously these were inert).
                if sub.name in all_links:
                    applications.append((100, order, "link", sub.name, resolved_props))
                elif root_block is not None and sub.name == root_block.name:
                    applications.append((100, order, "link", root_name, resolved_props))
                elif sub.name in element_types:
                    for link_name, kind in link_kind.items():
                        if kind == sub.name:
                            applications.append((1, order, "link", link_name, resolved_props))
                elif sub.name == "joint":
                    for j in robot.joints:
                        applications.append((1, order, "joint", j.name, resolved_props))
                elif sub.name == "sensor":
                    for s in robot.sensors:
                        applications.append((1, order, "sensor", s, resolved_props))
                # else: an unrecognized name with nothing to match — already
                # captured above in appearance_meta, never silently dropped.
                order += 1
                continue

            # New selector-rule shape: #id, .class, and/or elem[attr="value"].
            specificity = selector.specificity()

            if selector.id is not None:
                for link_name, lid in link_id.items():
                    if lid == selector.id:
                        applications.append((specificity, order, "link", link_name, resolved_props))

            if selector.classes:
                for link_name, classes in link_classes.items():
                    if any(c in classes for c in selector.classes):
                        applications.append((specificity, order, "link", link_name, resolved_props))
                for s in robot.sensors:
                    s_classes = s.properties.get("classes") or []
                    single = s.properties.get("class")
                    if isinstance(single, str):
                        s_classes = [*s_classes, single]
                    if any(c in s_classes for c in selector.classes):
                        applications.append((specificity, order, "sensor", s, resolved_props))

            if selector.element == "sensor":
                for s in robot.sensors:
                    if selector.attr is None:
                        matches = True
                    elif selector.attr[0] == "type":
                        matches = s.kind == selector.attr[1]
                    else:
                        matches = s.properties.get(selector.attr[0]) == selector.attr[1]
                    if matches:
                        applications.append((specificity, order, "sensor", s, resolved_props))
            elif selector.element == "joint":
                for j in robot.joints:
                    applications.append((specificity, order, "joint", j.name, resolved_props))
            elif selector.element in element_types:
                for link_name, kind in link_kind.items():
                    if kind == selector.element:
                        applications.append((specificity, order, "link", link_name, resolved_props))

            order += 1

    # Stable sort: equal-specificity rules apply in source order, so a later
    # rule correctly wins the tie — real CSS cascade semantics.
    applications.sort(key=lambda a: (a[0], a[1]))

    for _, _, kind, target, props in applications:
        if kind == "link":
            link_obj = all_links.get(target)
            if link_obj is not None:
                if link_obj.visual:
                    _apply_visual_properties(link_obj.visual, props)
                if link_obj.collision:
                    _apply_visual_properties(link_obj.collision, props)
        elif kind == "sensor":
            target.properties.update(props)
        elif kind == "joint":
            metadata.setdefault(f"joint:{target}", {}).update(props)


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


# ─────────────────────────────────────────────────────────────────────────────
# Limb mounting. A limb's first joint sits on the surface of the root body
# (legs underneath, arms/heads on top, wheels on the sides), and each segment
# hangs *off* its joint along the limb's direction instead of being centred on
# it — so joints pivot at the segment's end, the way real hardware does. All
# positions are explicit connection offsets, so MuJoCo (mjcf_builder) and the
# Studio viewport (serialize_robot) place every part identically.
# ─────────────────────────────────────────────────────────────────────────────

_AXIS_VECTORS = {"x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0]}

# Default joint axes per limb kind, repeated/truncated to the limb's joint count.
_DEFAULT_LIMB_AXES = {
    "arm": ["z", "y", "y", "x", "y", "x"],  # base yaw, shoulder, elbow, wrist roll/pitch/roll
    "leg": ["x", "y", "y", "y"],  # hip abduction, hip flexion, knee, ankle
    "head": ["z", "y"],  # pan, tilt
    "gripper": ["x"],
}

# Limbs grow up from the top of the body, except legs, which hang down.
_LIMB_DIRECTION = {"arm": 1.0, "head": 1.0, "gripper": 1.0, "leg": -1.0}


def _limb_axes(limb: Block, n: int) -> List[List[float]]:
    """`axes: [x, y, y];` (or `axis: z;` for a one-joint limb) overrides the
    per-kind default pattern; unknown entries fall back to the default."""
    default = _DEFAULT_LIMB_AXES.get(limb.name, ["y", "z"])
    raw = _prop(limb, "axes")
    if raw is None and _prop(limb, "axis") is not None:
        raw = [_prop(limb, "axis")]
    names: List[str] = []
    for i in range(n):
        name = None
        if isinstance(raw, list) and i < len(raw) and isinstance(raw[i], str):
            name = raw[i].strip().lower()
        if name not in _AXIS_VECTORS:
            name = default[i % len(default)]
        names.append(name)
    return [list(_AXIS_VECTORS[a]) for a in names]


def _half_extents(geometry: Optional[Geometry]) -> List[float]:
    if geometry is None:
        return [0.1, 0.1, 0.1]
    if geometry.shape == "box":
        return [float(v) / 2 for v in (geometry.size or [0.2, 0.2, 0.2])]
    if geometry.shape == "cylinder":
        r = float(geometry.radius)
        return [r, r, float(geometry.length) / 2]
    if geometry.shape == "sphere":
        r = float(geometry.radius)
        return [r, r, r]
    return [0.1, 0.1, 0.1]


def _segment_extent(geometry: Geometry) -> float:
    """Length of a limb segment along its own Z axis."""
    return 2 * _half_extents(geometry)[2]


_SIDE_CODES = {
    "fl": (1, 1), "fr": (1, -1), "rl": (-1, 1), "rr": (-1, -1),
    "bl": (-1, 1), "br": (-1, -1), "lf": (1, 1), "rf": (1, -1), "lh": (-1, 1), "rh": (-1, -1),
}


def _label_sides(label: Optional[str]) -> Tuple[int, int]:
    """(front/rear, left/right) from a limb label like `front_left`, `rear`,
    `left`, or a short code (`fl`, `rr`, ...). +X is forward and +Y is left
    (ROS REP 103). 0 means the label doesn't say."""
    text = (label or "").lower().strip()
    if text in _SIDE_CODES:
        return _SIDE_CODES[text]
    words = set(re.split(r"[_\-\s]+", text))
    for word in words:  # e.g. `rotor_fl`
        if word in _SIDE_CODES:
            return _SIDE_CODES[word]
    fx = 1 if words & {"front", "fore"} else -1 if words & {"rear", "back", "hind"} else 0
    fy = 1 if "left" in words else -1 if "right" in words else 0
    return fx, fy


_CORNER_ORDER = [(1, 1), (1, -1), (-1, 1), (-1, -1), (0, 1), (0, -1)]


def _vector_prop(block: Block, key: str) -> Optional[List[float]]:
    raw = _prop(block, key)
    if isinstance(raw, list) and len(raw) == 3 and all(isinstance(v, (int, float)) for v in raw):
        return [float(v) for v in raw]
    return None


_SPEED_TO_RAD_S = {"rpm": 2 * math.pi / 60.0, "deg/s": math.pi / 180.0, "rad/s": 1.0, None: 1.0}


def _angular_speed(block: Block, *keys: str) -> Optional[float]:
    """An angular speed property in rad/s (`300rpm`, `180deg/s`, `6rad/s`)."""
    for key in keys:
        raw = block.properties.get(key)
        if isinstance(raw, NumberValue) and raw.unit in _SPEED_TO_RAD_S:
            return raw.value * _SPEED_TO_RAD_S[raw.unit]
    return None


def _apply_motor(joint: Joint, limb: Block, warnings: List[str]) -> None:
    """Joint/motor properties from a limb block.

    `effort:` is the motor's stall torque (N·m) and `speed:` its no-load speed.
    Given both, the joint behaves like a DC motor: available torque falls
    linearly with speed, τ = τ_cmd − (effort / speed)·ω, so it can't spin up
    without bound. That speed-dependent term is MuJoCo joint damping, which is
    integrated implicitly (stable at any value)."""
    effort = _prop(limb, "effort")
    if isinstance(effort, (int, float)) and effort > 0:
        joint.max_effort = float(effort)
    speed = _angular_speed(limb, "speed", "velocity")
    if speed is not None and speed > 0:
        joint.max_velocity = speed
    damping = _prop(limb, "damping")
    if isinstance(damping, (int, float)) and damping >= 0:
        joint.damping = float(damping)
    friction = _prop(limb, "friction")
    if isinstance(friction, (int, float)) and friction >= 0:
        joint.friction = float(friction)
    limits = _prop(limb, "limits")
    if (
        joint.type == "revolute"
        and isinstance(limits, list)
        and len(limits) == 2
        and all(isinstance(v, (int, float)) for v in limits)
        and limits[0] < limits[1]
    ):
        joint.limits = (float(limits[0]), float(limits[1]))

    if speed is not None and speed > 0:
        if isinstance(effort, (int, float)) and effort > 0:
            joint.damping += float(effort) / speed  # motor back-EMF
        else:
            label = f" {limb.label}" if limb.label else ""
            message = (
                f"'{limb.name}{label}' declares speed: but no effort: — the motor's torque-speed "
                f"curve needs both (stall torque and no-load speed), so speed is only recorded."
            )
            if message not in warnings:  # once per limb, not once per segment
                warnings.append(message)


def _limb_mount(limb: Block, index_in_kind: int, root_half: List[float], wheel_half_width: float = 0.0) -> List[float]:
    """Where a limb's first joint sits, relative to the root body's centre.
    An explicit `at: [x, y, z];` always wins."""
    explicit = _vector_prop(limb, "at")
    if explicit is not None:
        return explicit
    hx, hy, hz = root_half
    fx, fy = _label_sides(limb.label)
    if limb.name == "leg":
        if (fx, fy) == (0, 0):
            fx, fy = _CORNER_ORDER[index_in_kind % len(_CORNER_ORDER)]
        return [fx * hx * 0.8, fy * hy * 0.8, -hz]
    if limb.name == "rotor":
        # On top of the body at its corners (an X layout for four rotors).
        if (fx, fy) == (0, 0):
            fx, fy = _CORNER_ORDER[index_in_kind % len(_CORNER_ORDER)]
        return [fx * hx, fy * hy, hz + 0.01]
    if limb.name == "wheel":
        if fy == 0:
            fy = 1 if index_in_kind % 2 == 0 else -1
        return [fx * hx * 0.7, fy * (hy + wheel_half_width + 0.005), -hz]
    # arms, heads, grippers on the body: on top, spread by label
    return [fx * hx * 0.6, fy * hy * 0.6, hz]


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
    # Selector bookkeeping for the style/appearance pass below — which
    # element type, id (from a block's label), and CSS-style classes each
    # compiled link came from.
    link_kind: Dict[str, str] = {}
    link_id: Dict[str, str] = {}
    link_classes: Dict[str, List[str]] = {}

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
    # Anything that walks or rolls is free to move; `base: fixed;` / `base: free;`
    # on the body (or the robot) says so explicitly.
    declared_base = (root_block is not None and _prop(root_block, "base")) or _prop(resolved, "base")
    has_locomotion = any(True for _ in resolved.children_named("leg", "wheel", "rotor")) or any(
        True for c in resolved.children_named(*CONTAINER_BLOCK_NAMES) for _ in c.children_named("leg", "wheel", "rotor")
    )
    if declared_base in ("fixed", "free"):
        robot.fixed_base = declared_base == "fixed"
    else:
        robot.fixed_base = shape not in MOBILE_SHAPES and not has_locomotion
    all_links[root_name] = root_link
    link_kind[root_name] = root_block.name if root_block is not None else "chassis"
    if root_block is not None:
        if root_block.label:
            link_id[root_name] = root_block.label
        link_classes[root_name] = _classes_of(root_block)

    # ── limbs: direct named blocks, plus one level inside `actuators { }` ──
    limb_blocks: List[Block] = list(resolved.children_named(*LIMB_BLOCK_NAMES))
    for container in resolved.children_named(*CONTAINER_BLOCK_NAMES):
        limb_blocks += list(container.children_named(*LIMB_BLOCK_NAMES))

    root_half = _half_extents(root_link.visual)
    pending_warnings: List[str] = []
    kind_counts: Dict[str, int] = {}
    # Distal link of each arm chain, so a gripper can mount on the arm's tip.
    arm_tips: List[Tuple[Optional[str], str, float]] = []
    # Arms first, so grippers declared before them can still find a tip.
    limb_blocks.sort(key=lambda b: 1 if b.name == "gripper" else 0)
    wheel_labels: List[Optional[str]] = []

    for limb in limb_blocks:
        base_name = f"{limb.name}_{limb.label}" if limb.label else limb.name
        index_in_kind = kind_counts.get(limb.name, 0)
        kind_counts[limb.name] = index_in_kind + 1
        limb_meta = _block_metadata(limb)
        parent_name = root_name

        if limb.name == "rotor":
            # A thin disc on a fixed joint carrying a thrust actuator (see
            # mjcf_builder): force along the disc's axis, plus the rotor's yaw
            # reaction torque, whose sign follows its spin direction.
            link_name = _unique_name(base_name, used_names)
            used_names.add(link_name)
            radius = float(_prop(limb, "radius", 0.06))
            link = _block_to_link(limb, link_name)
            if _prop(limb, "shape") is None:
                geometry = Geometry(shape="cylinder", radius=radius, length=0.01)
                _apply_visual_properties(geometry, _block_metadata(limb))
                link.visual = link.collision = geometry
            if not isinstance(_prop(limb, "mass"), (int, float)):
                link.mass = 0.02
            if link.visual is not None:
                link.inertia = _inertia_for_geometry(link.mass, link.visual)
            fx, fy = _label_sides(limb.label)
            if (fx, fy) == (0, 0):
                fx, fy = _CORNER_ORDER[index_in_kind % len(_CORNER_ORDER)]
            spin = _prop(limb, "spin")
            if spin not in ("cw", "ccw"):
                # X-quad convention: front-left and rear-right spin counter-clockwise.
                spin = "ccw" if fx * fy > 0 else "cw"
            max_thrust = _prop(limb, "thrust", 5.0)
            link.thruster = {
                "max_thrust": float(max_thrust) if isinstance(max_thrust, (int, float)) and max_thrust > 0 else 5.0,
                "torque_ratio": float(_prop(limb, "torque_ratio", 0.016)),
                "spin": spin,
            }
            joint = Joint(name=f"{link_name}_joint", type="fixed", parent_link=root_name, child_link=link_name)
            robot.attach(root_name, joint, link, offset=_limb_mount(limb, index_in_kind, root_half))
            robot.thrusters = list(getattr(robot, "thrusters", [])) + [link_name]
            chain_link_names = [link_name]
        elif limb.name == "wheel":
            # One continuous joint spinning about the lateral (Y) axis; the
            # cylinder is turned so its own axis lines up with the axle.
            link_name = _unique_name(base_name, used_names)
            used_names.add(link_name)
            radius = float(_prop(limb, "radius", 0.05))
            width = float(_prop(limb, "width", _prop(limb, "length", 0.04)))
            link = _block_to_link(limb, link_name)
            if _prop(limb, "shape") is None:
                # A wheel is a cylinder unless the file says otherwise.
                geometry = Geometry(shape="cylinder", radius=radius, length=width)
                _apply_visual_properties(geometry, _block_metadata(limb))
                link.visual = link.collision = geometry
            if link.visual is not None and link.visual.shape == "cylinder":
                link.visual.origin = [0.0, 0.0, 0.0, math.pi / 2, 0.0, 0.0]
            link.inertia = _inertia_for_geometry(link.mass, link.visual) if link.visual is not None else link.inertia
            joint = Joint(
                name=f"{link_name}_joint",
                type="continuous",
                parent_link=root_name,
                child_link=link_name,
                axis=[0.0, 1.0, 0.0],
            )
            _apply_motor(joint, limb, pending_warnings)
            mount = _limb_mount(limb, index_in_kind, root_half, wheel_half_width=width / 2)
            robot.attach(root_name, joint, link, offset=mount)
            chain_link_names = [link_name]
            wheel_labels.append(limb.label)
        else:
            n_joints = _limb_joint_count(limb)
            segment_lengths = _limb_segment_lengths(limb, n_joints)
            axes = _limb_axes(limb, n_joints)
            direction = _LIMB_DIRECTION.get(limb.name, 1.0)

            if limb.name == "gripper" and arm_tips and _vector_prop(limb, "at") is None:
                tip = next((t for t in arm_tips if t[0] == limb.label), arm_tips[0])
                parent_name, offset = tip[1], [0.0, 0.0, tip[2]]
            else:
                offset = _limb_mount(limb, index_in_kind, root_half)

            chain_link_names = []
            for seg_idx in range(n_joints):
                # A single-joint limb keeps the plain "arm_left" name for
                # backward compatibility; a multi-joint chain numbers its
                # segments "arm_left_1", "arm_left_2", ...
                seg_base = f"{base_name}_{seg_idx + 1}" if n_joints > 1 else base_name
                link_name = _unique_name(seg_base, used_names)
                used_names.add(link_name)

                link = _block_to_link(limb, link_name, default_length=segment_lengths[seg_idx])
                extent = _segment_extent(link.visual) if link.visual is not None else segment_lengths[seg_idx]
                if link.visual is not None:
                    # The segment hangs off its joint along the limb direction.
                    link.visual.origin = [0.0, 0.0, direction * extent / 2, 0.0, 0.0, 0.0]
                joint = Joint(
                    name=f"{link_name}_joint",
                    type="revolute",
                    parent_link=parent_name,
                    child_link=link_name,
                    axis=axes[seg_idx],
                )
                _apply_motor(joint, limb, pending_warnings)
                robot.attach(parent_name, joint, link, offset=offset)
                chain_link_names.append(link_name)
                parent_name = link_name
                offset = [0.0, 0.0, direction * extent]

            if limb.name == "arm":
                arm_tips.append((limb.label, chain_link_names[-1], offset[2]))

        for link_name in chain_link_names:
            all_links[link_name] = robot._links_by_name[link_name]
            metadata[link_name] = limb_meta
            link_kind[link_name] = limb.name
            if limb.label:
                link_id[link_name] = limb.label
            link_classes[link_name] = _classes_of(limb)

        # Sub-blocks of a limb that aren't structural (e.g. head { display{} camera{} })
        # become sensors mounted on the last (distal) segment of the chain.
        end_link_name = chain_link_names[-1]
        for sub in limb.children:
            robot.attach_sensor(_build_sensor(sub), position=end_link_name)

    # A body resting on two side wheels and nothing else tips onto its edge.
    if len(wheel_labels) == 2 and not any(_label_sides(l)[0] for l in wheel_labels) and "leg" not in kind_counts:
        pending_warnings.append(
            "only two wheels, both on the centre line: the body will tip onto its front or rear edge. "
            "Add front/rear wheels (e.g. `wheel front_left { ... }`) or casters."
        )

    # Fixed robots stand on the floor; mobile robots spawn just above it instead
    # of being dropped from height.
    if robot.fixed_base:
        robot.spawn_height = root_half[2]
    else:
        reach_down = 0.0
        for limb in limb_blocks:
            if limb.name == "leg":
                n = _limb_joint_count(limb)
                reach_down = max(reach_down, sum(_limb_segment_lengths(limb, n)))
            elif limb.name == "wheel":
                reach_down = max(reach_down, float(_prop(limb, "radius", 0.05)))
        robot.spawn_height = root_half[2] + reach_down + 0.02

    # ── sensors { lidar front { ... } camera front { ... } } ──
    for sensors_block in resolved.children_named(*SENSOR_CONTAINER_NAMES):
        for s in sensors_block.children:
            robot.attach_sensor(_build_sensor(s), position=s.label or "body")

    # ── appearance / style: real CSS-like selectors, variables, and cascade ──
    _apply_style_rules(
        appearance_blocks=resolved.children_named(*APPEARANCE_CONTAINER_NAMES),
        root_block=root_block,
        root_name=root_name,
        all_links=all_links,
        link_kind=link_kind,
        link_id=link_id,
        link_classes=link_classes,
        robot=robot,
        metadata=metadata,
    )

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
    warnings: List[str] = list(pending_warnings)
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
