"""Generate a Bill of Materials from a compiled RSS/stacc Robot.

Honest scope: this lists parts *by the name you gave them in the .rss
file* (e.g. `motor: "BLDC-01"`, `model: rplidar-a2`) — it does not resolve
those to a real distributor SKU, price, or datasheet. There's no parts
database or internet access here to do that lookup for real, and
fabricating plausible-looking SKUs/prices would be exactly the kind of
fake output this project has spent this whole session removing elsewhere.
Treat this as a procurement checklist to go shopping with, not a quote.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from typing import Any, Dict, List

from stacc.core.robot_base import Robot


@dataclass
class BomRow:
    category: str  # "printed_part" | "actuator" | "sensor"
    name: str
    spec: str
    source: str  # "3D-printed (see build/parts/)" | "purchased — see notes"
    notes: str = ""

    def as_dict(self) -> Dict[str, str]:
        return {
            "category": self.category,
            "name": self.name,
            "spec": self.spec,
            "source": self.source,
            "notes": self.notes,
        }


def _geometry_spec(link) -> str:
    geom = link.visual
    if geom is None:
        return f"mass={link.mass}kg"
    if geom.shape == "box":
        sx, sy, sz = geom.size
        return f"box {sx * 1000:.0f}x{sy * 1000:.0f}x{sz * 1000:.0f}mm, mass={link.mass}kg"
    if geom.shape == "cylinder":
        return (
            f"cylinder r={geom.radius * 1000:.0f}mm l={geom.length * 1000:.0f}mm, "
            f"mass={link.mass}kg"
        )
    if geom.shape == "sphere":
        return f"sphere r={geom.radius * 1000:.0f}mm, mass={link.mass}kg"
    return f"{geom.shape}, mass={link.mass}kg"


def generate_bom(robot: Robot) -> List[BomRow]:
    rows: List[BomRow] = []
    metadata: Dict[str, Dict[str, Any]] = getattr(robot, "rss_metadata", {})

    for link in robot.links:
        rows.append(
            BomRow(
                category="printed_part",
                name=link.name,
                spec=_geometry_spec(link),
                source="3D-printed (see build/parts/)",
                notes="",
            )
        )

    for joint in robot.joints:
        if joint.type == "fixed":
            continue
        link_meta = metadata.get(joint.child_link, {})
        actuator = link_meta.get("actuator") or link_meta.get("motor")
        rows.append(
            BomRow(
                category="actuator",
                name=joint.name,
                spec=f"{joint.type}, max_effort={joint.max_effort}, "
                     f"max_velocity={joint.max_velocity}",
                source="purchased — see notes",
                notes=(
                    f"actuator/motor as specified in .rss: {actuator!r}"
                    if actuator
                    else "no actuator/motor: specified in .rss — pick one rated for "
                         f"max_effort={joint.max_effort}"
                ),
            )
        )

    for sensor in robot.sensors:
        kind = getattr(sensor, "kind", type(sensor).__name__.lower())
        props = getattr(sensor, "properties", None) or {}
        model = props.get("model")
        driver = "real stacc.sensors driver" if type(sensor).__name__ != "SensorSpec" else "unrecognized kind — no driver wired yet"
        rows.append(
            BomRow(
                category="sensor",
                name=f"{kind}_{getattr(sensor, 'label', None) or ''}".rstrip("_"),
                spec=f"kind={kind}" + (f", model={model}" if model else ""),
                source="purchased — see notes",
                notes=f"{driver}; model name as specified in .rss, not a resolved SKU",
            )
        )

    return rows


def write_bom(robot: Robot, out_dir) -> Dict[str, str]:
    """Write build/bom.csv and build/bom.md. Returns {"csv": path, "md": path}."""
    import os

    os.makedirs(out_dir, exist_ok=True)
    rows = generate_bom(robot)

    csv_path = os.path.join(out_dir, "bom.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["category", "name", "spec", "source", "notes"])
        writer.writeheader()
        for row in rows:
            writer.writerow(row.as_dict())

    md_path = os.path.join(out_dir, "bom.md")
    with open(md_path, "w") as f:
        f.write(f"# Bill of materials — {robot.name}\n\n")
        f.write(
            "Part/model names are exactly what was written in the .rss file — "
            "not resolved to a real distributor SKU, price, or datasheet. "
            "Use this as a shopping checklist, not a quote.\n\n"
        )
        f.write("| Category | Name | Spec | Source | Notes |\n")
        f.write("|---|---|---|---|---|\n")
        for row in rows:
            f.write(f"| {row.category} | {row.name} | {row.spec} | {row.source} | {row.notes} |\n")

    return {"csv": csv_path, "md": md_path}
