"""Export a compiled RSS/stacc Robot to real, printable meshes via trimesh.

Honest scope: RSS's `Geometry` only models box/cylinder/sphere/mesh
primitives — no fillets, chamfers, or CSG booleans (holes, unions,
subtractions). Real parametric CAD features like that need an actual
B-Rep solid-modeling kernel (OpenCASCADE), which isn't a dependency here.
This module produces genuinely watertight, correctly-dimensioned meshes
for what RSS *can* describe today; it does not pretend to do more.
"""

from __future__ import annotations

import os
from typing import Dict, List

from stacc.core.link import Geometry
from stacc.core.robot_base import Robot


def _mesh_for_geometry(geom: Geometry, link_name: str, warnings: List[str]):
    import trimesh

    if geom.shape == "box":
        sx, sy, sz = geom.size
        return trimesh.creation.box(extents=[sx, sy, sz])
    if geom.shape == "cylinder":
        return trimesh.creation.cylinder(radius=geom.radius, height=geom.length)
    if geom.shape == "sphere":
        return trimesh.creation.icosphere(radius=geom.radius)
    if geom.shape == "mesh":
        if geom.mesh_path and os.path.exists(geom.mesh_path):
            mesh = trimesh.load(geom.mesh_path, force="mesh")
            sx, sy, sz = geom.mesh_scale
            mesh.apply_scale([sx, sy, sz])
            return mesh
        warnings.append(
            f"link '{link_name}': mesh_path '{geom.mesh_path}' not found — skipped, "
            f"no fake geometry generated"
        )
        return None

    warnings.append(f"link '{link_name}': unknown geometry shape '{geom.shape}' — skipped")
    return None


def robot_to_meshes(robot: Robot) -> Dict[str, "trimesh.Trimesh"]:
    """One real trimesh.Trimesh per link with visual geometry.

    Links with no `visual` geometry (rare — RSS always sets one) are
    omitted rather than given a fake placeholder shape.
    """
    warnings: List[str] = []
    meshes: Dict[str, "trimesh.Trimesh"] = {}

    for link in robot.links:
        if link.visual is None:
            continue
        mesh = _mesh_for_geometry(link.visual, link.name, warnings)
        if mesh is not None:
            meshes[link.name] = mesh

    robot.rss_mesh_warnings = warnings
    return meshes


def _link_world_positions(robot: Robot) -> Dict[str, List[float]]:
    """Best-effort translation-only placement for the merged assembly
    preview: walk the tree from the root, accumulating each connection's
    [x, y, z] offset. Joint rotation (rpy) and joint angle are NOT applied
    — this is a rest-pose layout preview, not a kinematic solve.
    """
    root = robot.get_root_link()
    if root is None:
        return {}

    positions: Dict[str, List[float]] = {root.name: [0.0, 0.0, 0.0]}
    stack = [root.name]
    while stack:
        parent_name = stack.pop()
        px, py, pz = positions[parent_name]
        for _joint, child_link, offset in robot.get_all_children(parent_name):
            ox, oy, oz = (offset or [0.0, 0.0, 0.0])
            positions[child_link.name] = [px + ox, py + oy, pz + oz]
            stack.append(child_link.name)

    return positions


def write_stl_parts(robot: Robot, out_dir: str) -> List[str]:
    """Write one <link_name>.stl per part, plus a best-effort merged
    <robot.name>_assembly.stl. Returns the list of files written.
    """
    import trimesh

    os.makedirs(out_dir, exist_ok=True)
    meshes = robot_to_meshes(robot)
    written: List[str] = []

    for name, mesh in meshes.items():
        path = os.path.join(out_dir, f"{name}.stl")
        mesh.export(path)
        written.append(path)

    positions = _link_world_positions(robot)
    assembly = trimesh.Scene()
    for name, mesh in meshes.items():
        pos = positions.get(name, [0.0, 0.0, 0.0])
        transform = trimesh.transformations.translation_matrix(pos)
        assembly.add_geometry(mesh, transform=transform, node_name=name)

    if len(assembly.geometry) > 0:
        assembly_path = os.path.join(out_dir, f"{robot.name}_assembly.stl")
        assembly.export(assembly_path)
        written.append(assembly_path)

    return written
