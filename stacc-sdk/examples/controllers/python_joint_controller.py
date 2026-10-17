#!/usr/bin/env python3
"""Example Python controller: drive a joint through the STACC runtime API.

This is the shape every Python controller takes (spec §4): it never imports the
physics engine, it speaks the versioned ``/v1/runtime/*`` contract through
``stacc.server.runtime_client``. Run it against a live daemon:

    python stacc-sdk/examples/controllers/python_joint_controller.py

Exit code 0 when the commanded joint actually moved, 2 when it did not, 1 on a
runtime error. ``--joint``, ``--angle`` (radians) and ``--steps`` tune the run.
"""

from __future__ import annotations

import argparse
import sys

from stacc.server.runtime_client import RuntimeApiError, RuntimeClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="http://127.0.0.1:8766")
    parser.add_argument("--joint", default="J2", help="joint id (J2) or raw name")
    parser.add_argument("--angle", type=float, default=0.5, help="target angle in radians")
    parser.add_argument("--steps", type=int, default=400)
    args = parser.parse_args()

    rt = RuntimeClient(base=args.base)
    try:
        caps = rt.capabilities()
    except RuntimeApiError as e:
        print(f"[controller] runtime unavailable: {e}", file=sys.stderr)
        return 1

    print(f"[controller] engine={caps['engine']} dof={caps['joints']} api=v{caps['api_version']}")
    if caps["joints"] < 2:
        print("[controller] need at least two joints for this demo", file=sys.stderr)
        return 1

    # Command position and let the servo settle over a number of timesteps.
    rt.command_joint(args.joint, args.angle)
    for _ in range(args.steps):
        rt.step()

    joint = rt.joint_by_name(args.joint) or {}
    position = float(joint.get("position", 0.0))
    telemetry = rt.telemetry()
    print(f"[controller] {args.joint} position={position:.4f} rad "
          f"sim_time={telemetry.get('sim_time')} step={telemetry.get('step')}")

    moved = abs(position) > 1e-3
    print(f"[controller] {'OK — joint moved' if moved else 'FAILED — joint did not move'}")
    return 0 if moved else 2


if __name__ == "__main__":
    raise SystemExit(main())
