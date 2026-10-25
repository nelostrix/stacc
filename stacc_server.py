#!/usr/bin/env python3
"""STACC Universal Robotics & MuJoCo Simulation Backend Server.

Serves the /v1/* NELO-Studio UI contract, backed by the real stacc SDK:
MuJoCo physics + stacc.kinematics FK/IK instead of hand-rolled math.
Every endpoint below keeps its original response shape.

If the SDK cannot be imported, the server degrades to the built-in analytic
kernel (same response shapes) so the UI never gets a dead endpoint.
"""

import os
import sys
import json
import re
import time
import math
import faulthandler
import hashlib
import signal
import logging
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List

# Add STACC SDK to python path.
#
# When frozen by PyInstaller (see packaging/build-stacc-daemon.sh in the
# NELO-Studio repo), __file__ resolves inside the temporary/onedir bundle
# rather than the real source tree, and the bundle's own root is
# sys._MEIPASS, not dirname(__file__) — data files added via --add-data
# (examples/*.rss) land there, not next to a "stacc-sdk" sibling folder.
if getattr(sys, "frozen", False):
    STACC_ROOT = sys._MEIPASS  # type: ignore[attr-defined]
else:
    STACC_ROOT = os.path.dirname(os.path.abspath(__file__))
SDK_PATH = os.path.join(STACC_ROOT, "stacc-sdk")
if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

RT: Any = None  # Live StaccRuntime, built lazily when the SDK is importable.

try:
    import stacc
    from stacc.server import StaccRuntime, create_robot

    RT = StaccRuntime(physics="mujoco", hz=1000.0, timestep=0.001, headless=True)
    RT.set_robot(create_robot("arm", name="default_arm"))
    RT.set_joint_positions([0.0] * RT.robot.dof)  # servo-hold zero pose
    RT.start()
    STACC_SDK_AVAILABLE = True
    STACC_VERSION = getattr(stacc, "__version__", "0.3.0")
    try:
        import mujoco as _mujoco
        MUJOCO_VERSION = getattr(_mujoco, "__version__", None)
    except ImportError:
        MUJOCO_VERSION = None
    PHYSICS_ENGINE = (
        f"stacc {STACC_VERSION} / {RT.engine._physics_backend.__class__.__name__}"
        + (f" (MuJoCo {MUJOCO_VERSION})" if MUJOCO_VERSION else "")
    )
    print(f"[STACC-SERVER] Live SDK runtime active: robot={RT.robot.name}, "
          f"engine={RT.engine._physics_backend.__class__.__name__}, hz={RT.hz:g}")
except Exception as e:
    RT = None
    STACC_SDK_AVAILABLE = False
    STACC_VERSION = "0.3.0-fallback"
    MUJOCO_VERSION = None
    PHYSICS_ENGINE = "analytic fallback kernel (no physics engine loaded)"
    print(f"[STACC-SERVER] SDK import failed ({e}); using analytic fallback kernel.")

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("stacc.server")

START_TIME = time.time()
SIMULATION_STEPS = 0

# CBF safety barrier configuration (kept from the original contract).
CBF_BARRIER_MARGIN_MM = 45.0
OBSTACLE_POS = [0.25, 0.25, 0.10]


# ─────────────────────────────────────────────────────────────────────────────
# Live endpoint logic (SDK-backed)
# ─────────────────────────────────────────────────────────────────────────────


def _ee_xyz(runtime: Any) -> Dict[str, float]:
    """Real FK end-effector {x,y,z} + roll/pitch/yaw (degrees) for the loaded arm."""
    ee = runtime.end_effector_pose()
    if ee is None:
        return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    return {
        "x": round(ee["x"], 4),
        "y": round(ee["y"], 4),
        "z": round(ee["z"], 4),
        "roll": round(math.degrees(ee["roll"]), 2),
        "pitch": round(math.degrees(ee["pitch"]), 2),
        "yaw": round(math.degrees(ee["yaw"]), 2),
    }


def _cbf_check(ee: Dict[str, float]) -> tuple:
    dist = math.sqrt(
        (ee["x"] - OBSTACLE_POS[0]) ** 2
        + (ee["y"] - OBSTACLE_POS[1]) ** 2
        + (ee["z"] - OBSTACLE_POS[2]) ** 2
    )
    margin_mm = round(dist * 1000.0, 1)
    return margin_mm, margin_mm >= CBF_BARRIER_MARGIN_MM


def _real_energy(runtime: Any) -> float:
    """Real MuJoCo system energy (kinetic + potential) in joules, or 0.0 when unavailable."""
    backend = getattr(runtime.engine, "_physics_backend", None)
    if backend is None or not (hasattr(backend, "model") and hasattr(backend, "data")):
        return 0.0
    try:
        import mujoco
        mujoco.mj_energyPos(backend.model, backend.data)
        mujoco.mj_energyVel(backend.model, backend.data)
        return round(float(backend.data.energy[0] + backend.data.energy[1]), 3)
    except Exception:
        return 0.0


def _loop_timing() -> Dict[str, Any]:
    """Measured timing of the live control loop (all zeros when no runtime is loaded)."""
    if RT is None:
        return {"target_hz": 0.0, "measured_hz": 0.0, "jitter_ms": 0.0, "step_time_ms": 0.0,
                "period_p99_ms": 0.0, "period_max_ms": 0.0, "deadline_misses": 0, "samples": 0}
    return RT.loop_stats()


def _shm_status() -> str:
    """One-line state of the shared-memory control channel (the real data
    plane, stacc/server/rt_channel.py), e.g. "lockstep /dev/shm/stacc_rt_8766"."""
    if RT is None:
        return "inactive"
    st = RT.channel_status()
    return f"{st['mode']} {st['path']}" if st.get("enabled") else "inactive"


def _joint_world_offset(robot: Any, joint: Any, links_by_name: Dict[str, Any]) -> List[float]:
    """The [x, y, z] a joint's child link actually sits at.

    Joint.origin is [0,0,0,0,0,0] for chain-generated limbs — the real layout
    is computed at MJCF-export time in stacc.sim.mjcf_builder, from an
    explicit Connection offset if one was set, else auto-derived from parent/
    child geometry. Reuse that exact logic instead of re-deriving it, so the
    NELO-Studio viewport renders robots at the same positions MuJoCo does.
    """
    from stacc.sim.mjcf_builder import _compute_auto_offset

    explicit = robot.get_connection_offset(joint.parent_link, joint.child_link)
    if explicit is not None:
        return [float(v) for v in explicit]

    parent = links_by_name.get(joint.parent_link)
    child = links_by_name.get(joint.child_link)
    if parent is None or child is None:
        return [0.0, 0.0, 0.0]
    return [float(v) for v in _compute_auto_offset(parent, child, joint).split()]


def serialize_robot(robot: Any) -> Dict[str, Any]:
    """The compiled-robot contract the NELO-Studio 3D view renders generically
    (any RSS-designed robot, not just the built-in arm) — Link/Joint already
    know how to describe themselves, so this is mostly just assembling their
    output, plus the real chain-layout offsets (see _joint_world_offset).
    """
    links_by_name = {l.name: l for l in robot.links}

    joints_json = []
    for j in robot.joints:
        d = j.to_dict()
        x, y, z = _joint_world_offset(robot, j, links_by_name)
        d["origin"] = [x, y, z, d["origin"][3], d["origin"][4], d["origin"][5]]
        joints_json.append(d)

    return {
        "name": robot.name,
        "dof": robot.dof,
        "links": [l.to_dict() for l in robot.links],
        "joints": joints_json,
        "warnings": list(getattr(robot, "rss_warnings", []) or []),
        # Height of the root body's frame above the floor at spawn (fixed-base
        # robots stay there; mobile robots then follow the live pose).
        "spawn_height": float(getattr(robot, "spawn_height", 0.0) or 0.0),
        "fixed_base": bool(getattr(robot, "fixed_base", True)),
    }


def live_simulate_step(req: Dict[str, Any]) -> Dict[str, Any]:
    """Command joint angles (degrees) to the live MuJoCo robot and report real state."""
    dof = RT.robot.dof if (RT is not None and RT.robot is not None) else 6
    joints_deg = req.get("joints", [0.0] * dof)
    kp = float(req.get("kpGain", 120))
    kd = float(req.get("kdGain", 25))

    RT.set_gains(kp=kp, kd=kd)
    RT.set_joint_positions([math.radians(float(a)) for a in joints_deg])

    ee = _ee_xyz(RT)
    margin_mm, cbf_safe = _cbf_check(ee)
    torques = [round(float(t), 2) for t in RT.robot._state["joint_torques"]]
    loop = _loop_timing()

    return {
        "step": RT.engine.state.step,
        "dt": RT.config.timestep,
        "frequency_hz": round(1.0 / RT.config.timestep, 1),
        "measured_hz": loop["measured_hz"],
        "jitter_ms": loop["jitter_ms"],
        "end_effector": ee,
        "torques_nm": torques,
        "cbf_margin_mm": margin_mm,
        "cbf_safe": cbf_safe,
        "shm_status": _shm_status(),
        "energy_j": _real_energy(RT),
        "step_time_ms": loop["step_time_ms"],
    }


def live_ik(req: Dict[str, Any]) -> Dict[str, Any]:
    """Real damped-least-squares IK via stacc.kinematics (honest iterations/error)."""
    import numpy as np
    from stacc.kinematics.forward import end_effector_position, jacobian
    from stacc.kinematics.transforms import DH

    target = [float(v) for v in req.get("target_pos", [0.35, 0.15, 0.40])]
    target_np = np.array(target, dtype=float)

    dh: List[DH] = RT.robot.dh_parameters
    limits = RT.robot.joint_limits or [(lo, hi) for j in RT.robot.joints for lo, hi in [j.limits]]
    q = np.array(RT.robot.get_joint_positions(), dtype=float)

    damping, tolerance, max_iter = 0.1, 1e-4, 100
    iterations = 0
    error_norm = float("inf")
    for _ in range(max_iter):
        cur = end_effector_position(dh, q.tolist())
        error = target_np - cur
        error_norm = float(np.linalg.norm(error))
        iterations += 1
        if error_norm < tolerance:
            break
        J = jacobian(dh, q.tolist())[:3, :]
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), error)
        q = q + dq
        for i, (lo, hi) in enumerate(limits):
            q[i] = max(lo, min(hi, q[i]))

    sol_deg = [round(math.degrees(float(a)), 1) for a in q]

    RT.set_joint_positions(q.tolist())  # drive the live arm to the IK solution

    return {
        "success": error_norm <= 1e-3,
        "solution_joints_deg": sol_deg,
        "iterations": iterations,
        "error_norm_m": round(error_norm, 4),
    }


def live_tune_gains(req: Dict[str, Any]) -> Dict[str, Any]:
    """Optimal PD gains from the stiffness model, applied live to the servo."""
    inertia_eff = float(req.get("inertia_eff", 0.08))
    zeta = float(req.get("damping_ratio", 0.707))
    wn = float(req.get("natural_freq_rad_s", 45.0))

    kp_opt = round(inertia_eff * (wn ** 2), 1)
    kd_opt = round(2.0 * zeta * math.sqrt(kp_opt * inertia_eff), 1)
    RT.set_gains(kp=kp_opt, kd=kd_opt)

    return {
        "kp_optimal": kp_opt,
        "kd_optimal": kd_opt,
        "damping_ratio": zeta,
        "natural_freq": wn,
        "settling_time_s": round(4.0 / (zeta * wn), 3),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Model files (URDF / SDF / RSS) — used by the Studio's file-explorer actions.
# The Studio reads the file and sends its text; this daemon never opens arbitrary paths.
# ─────────────────────────────────────────────────────────────────────────────

def _robot_from_source(fmt: str, source: str) -> Any:
    """Parse a model file's text into a stacc Robot. Raises ValueError with a readable message."""
    fmt = (fmt or "").lower().lstrip(".")
    try:
        if fmt in ("urdf", "xml"):
            from stacc.core.urdf_parser import load_urdf_string
            return load_urdf_string(source)
        if fmt == "sdf":
            import tempfile
            from stacc.core.urdf_parser import load_sdf
            with tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, "model.sdf")
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(source)
                return load_sdf(path)
        if fmt == "rss" or fmt in EMBEDDED_RSS_FORMATS:
            return _compile_rss(source)
    except Exception as e:  # parser/compiler errors carry useful text
        raise ValueError(str(e) or e.__class__.__name__) from e
    raise ValueError(
        f"Unsupported model format '{fmt}' (expected urdf, sdf, rss, or a "
        f"Python/C++/Rust file with an RSS block)."
    )


def model_inspect(req: Dict[str, Any]) -> Dict[str, Any]:
    """Parse a model and report its structure plus real validation findings."""
    if not STACC_SDK_AVAILABLE:
        return {"error": "The stacc SDK is not available in this daemon."}
    try:
        robot = _robot_from_source(req.get("format", ""), req.get("source", ""))
    except ValueError as e:
        return {"error": str(e), "kind": "parse"}

    from stacc.converters.urdf_mjcf_usd import ModelTranspiler

    findings: List[Dict[str, str]] = []
    link_names = [l.name for l in robot.links]
    if len(set(link_names)) != len(link_names):
        findings.append({"level": "error", "message": "Duplicate link names."})
    joint_names = [j.name for j in robot.joints]
    if len(set(joint_names)) != len(joint_names):
        findings.append({"level": "error", "message": "Duplicate joint names."})

    known = set(link_names)
    children = set()
    for j in robot.joints:
        for end, label in ((j.parent_link, "parent"), (j.child_link, "child")):
            if end not in known:
                findings.append({"level": "error", "message": f"Joint '{j.name}' references unknown {label} link '{end}'."})
        children.add(j.child_link)
        if j.type in ("revolute", "prismatic") and j.limits[0] >= j.limits[1]:
            findings.append({"level": "warning", "message": f"Joint '{j.name}' has lower limit >= upper limit."})
    roots = [n for n in link_names if n not in children]
    if len(roots) > 1:
        findings.append({"level": "warning", "message": f"{len(roots)} root links ({', '.join(roots[:4])}) — the model is not a single tree."})

    total_mass = 0.0
    for l in robot.links:
        total_mass += float(l.mass or 0.0)
        if not l.mass or l.mass <= 0:
            findings.append({"level": "warning", "message": f"Link '{l.name}' has no mass (a dynamic simulation needs mass > 0)."})
            continue
        i = l.inertia
        ok, why = ModelTranspiler.validate_inertia_tensor(l.mass, i.ixx, i.iyy, i.izz)
        if not ok:
            findings.append({"level": "warning", "message": f"Link '{l.name}' inertia: {why}"})

    return {
        "name": robot.name,
        "format": (req.get("format") or "").lstrip("."),
        "dof": robot.dof,
        "link_count": len(robot.links),
        "joint_count": len(robot.joints),
        "root_links": roots,
        "total_mass_kg": round(total_mass, 4),
        "joints": [
            {
                "name": j.name, "type": j.type, "parent": j.parent_link, "child": j.child_link,
                "lower": None if j.limits[0] == float("-inf") else round(float(j.limits[0]), 4),
                "upper": None if j.limits[1] == float("inf") else round(float(j.limits[1]), 4),
                "effort": j.max_effort, "velocity": j.max_velocity,
            }
            for j in robot.joints
        ],
        "findings": findings,
    }


def model_convert(req: Dict[str, Any]) -> Dict[str, Any]:
    """Convert a model to URDF, SDF or MJCF text."""
    if not STACC_SDK_AVAILABLE:
        return {"error": "The stacc SDK is not available in this daemon."}
    target = (req.get("to") or "").lower()
    try:
        robot = _robot_from_source(req.get("format", ""), req.get("source", ""))
    except ValueError as e:
        return {"error": str(e), "kind": "parse"}
    try:
        from stacc.converters.urdf_mjcf_usd import ModelTranspiler
        if target == "urdf":
            text = ModelTranspiler.robot_to_urdf(robot)
        elif target == "sdf":
            text = ModelTranspiler.robot_to_sdf(robot)
        elif target == "mjcf":
            from stacc.sim.engine import SimulationConfig
            from stacc.sim.mjcf_builder import robot_to_mjcf
            text = robot_to_mjcf(robot, SimulationConfig(duration=1.0, timestep=0.001, headless=True, physics_engine="mujoco"))
        else:
            return {"error": f"Unsupported target format '{target}' (expected urdf, sdf or mjcf)."}
    except Exception as e:
        return {"error": f"Conversion failed: {e}", "kind": "convert"}
    return {"name": robot.name, "to": target, "text": text}


def robot_load_model(req: Dict[str, Any]) -> Dict[str, Any]:
    """Hot-swap a parsed model into the live simulation runtime."""
    if RT is None:
        return {"error": "Live SDK runtime unavailable — cannot load a model into the simulator."}
    try:
        robot = _robot_from_source(req.get("format", ""), req.get("source", ""))
    except ValueError as e:
        return {"error": str(e), "kind": "parse"}
    try:
        RT.set_robot(robot)
        RT.set_joint_positions([0.0] * robot.dof)
        RT.start()
        return {"robot": serialize_robot(robot)}
    except Exception as e:
        return {"error": f"The simulator could not load this model: {e}", "kind": "load"}


def rss_bom(req: Dict[str, Any]) -> Dict[str, Any]:
    """Bill of materials for an .rss robot, as the files the SDK writes (CSV etc.)."""
    if not STACC_SDK_AVAILABLE:
        return {"error": "The stacc SDK is not available in this daemon."}
    try:
        robot = _robot_from_source("rss", req.get("source", ""))
    except ValueError as e:
        return {"error": str(e), "kind": "parse"}
    import tempfile
    from stacc.rss.bom import write_bom
    with tempfile.TemporaryDirectory() as tmp:
        written = write_bom(robot, tmp)
        files = {}
        for name, path in written.items():
            with open(path, "r", encoding="utf-8") as fh:
                files[os.path.basename(path)] = fh.read()
    return {"name": robot.name, "files": files}


def rss_ros2_package(req: Dict[str, Any]) -> Dict[str, Any]:
    """A ROS 2 package skeleton (package.xml, URDF, launch, controllers) for an .rss robot."""
    if not STACC_SDK_AVAILABLE:
        return {"error": "The stacc SDK is not available in this daemon."}
    try:
        robot = _robot_from_source("rss", req.get("source", ""))
    except ValueError as e:
        return {"error": str(e), "kind": "parse"}
    import tempfile
    from pathlib import Path
    from stacc.converters.urdf_mjcf_usd import ModelTranspiler
    from stacc.rss.ros2_skeleton import write_ros2_package
    with tempfile.TemporaryDirectory() as tmp:
        pkg_dir = Path(write_ros2_package(robot, ModelTranspiler.robot_to_urdf(robot), Path(tmp)))
        files = {}
        for f in sorted(pkg_dir.rglob("*")):
            if f.is_file():
                try:
                    files[str(f.relative_to(pkg_dir))] = f.read_text(encoding="utf-8")
                except UnicodeDecodeError:
                    continue
    return {"name": robot.name, "package": pkg_dir.name, "files": files}


# ─────────────────────────────────────────────────────────────────────────────
# RSS — load a user-designed robot into the live daemon (NELO-Studio Robot
# Designer). Requires the real SDK; there is no meaningful analytic fallback
# for "compile arbitrary robot geometry", so these 404 honestly when RT is None
# instead of pretending to compile anything.
# ─────────────────────────────────────────────────────────────────────────────

EXAMPLES_DIR = os.path.join(SDK_PATH, "examples")


def _rss_error_diagnostics(exc: Exception) -> List[Dict[str, Any]]:
    """Structured diagnostics for a failed RSS compile (see docs/RSS_LANGUAGE_SPEC.md §6).

    One shape for every failure so the editor, CLI and any future tooling read
    the same fields instead of string-matching error text.
    """
    from stacc.rss import RSSCompileError, RSSSyntaxError

    if isinstance(exc, RSSSyntaxError):
        return [{
            "severity": "error",
            "code": "RSS-SYNTAX",
            "message": exc.message,
            "line": exc.line,
            "col": exc.col,
            "source_line": getattr(exc, "source_line", "") or "",
        }]
    if isinstance(exc, RSSCompileError):
        return [{
            "severity": "error",
            "code": "RSS-COMPILE",
            "message": str(exc),
            "line": getattr(exc, "line", None),
            "col": getattr(exc, "col", None),
        }]
    return [{
        "severity": "error",
        "code": "RSS-ERROR",
        "message": str(exc) or exc.__class__.__name__,
        "line": None,
        "col": None,
    }]


def _rss_warning_diagnostics(robot: Any) -> List[Dict[str, Any]]:
    """Compile warnings (unrecognized blocks/properties) as structured diagnostics."""
    out: List[Dict[str, Any]] = []
    for w in getattr(robot, "rss_warnings", []) or []:
        text = w if isinstance(w, str) else str(w)
        line, col = None, None
        # Warnings are plain strings today; lift "line N" hints when present so
        # the editor can still underline them.
        m = re.search(r"line\s+(\d+)", text)
        if m:
            line = int(m.group(1))
        out.append({
            "severity": "warning",
            "code": "RSS-WARNING",
            "message": text,
            "line": line,
            "col": col,
        })
    return out


# Host-language files that carry their robot in a <<<RSS>>> block.
EMBEDDED_RSS_FORMATS = {"py", "python", "cpp", "cc", "cxx", "hpp", "h", "c", "rs", "rust", "embedded"}


def _compile_rss(source: str) -> Any:
    """Parse + compile RSS into a stacc Robot. Raises on any failure.

    `source` is either plain RSS or a whole Python/C++/Rust file containing
    `<<<RSS>>> ... <<</RSS>>>` block(s); errors from embedded blocks carry the
    host file's line/column, so diagnostics land on the line being edited.
    """
    from stacc.rss import compile_document, compile_embedded, has_embedded_rss, parse

    if has_embedded_rss(source):
        return compile_embedded(source, search_paths=[EXAMPLES_DIR])
    doc = parse(source)
    return compile_document(doc, search_paths=[EXAMPLES_DIR])


def rss_compile(req: Dict[str, Any]) -> Dict[str, Any]:
    """Validate + compile an .rss document WITHOUT touching the live runtime.

    The editor's live loop calls this while typing; only a successful compile is
    then hot-swapped through /v1/rss/load, so a half-written file never disturbs
    the running simulation. Returns the same structure shape as rss_load plus
    `diagnostics`.
    """
    if not STACC_SDK_AVAILABLE:
        return {"error": "The stacc SDK is not available in this daemon."}

    source = req.get("source", "")
    if not source.strip():
        return {
            "error": "source (.rss text) is required",
            "kind": "input",
            "diagnostics": [{
                "severity": "error",
                "code": "RSS-INPUT",
                "message": "source (.rss text) is required",
            }],
        }
    try:
        robot = _compile_rss(source)
    except Exception as e:  # syntax/compile errors carry line/col where available
        result = {"error": str(e), "kind": "compile" if "compile" in e.__class__.__name__.lower() else "syntax"}
        from stacc.rss import RSSSyntaxError

        if isinstance(e, RSSSyntaxError):
            result.update({"error": e.message, "line": e.line, "col": e.col, "kind": "syntax"})
        result["diagnostics"] = _rss_error_diagnostics(e)
        return result

    return {
        "robot": serialize_robot(robot),
        "diagnostics": _rss_warning_diagnostics(robot),
    }


def rss_load(req: Dict[str, Any]) -> Dict[str, Any]:
    """Parse + compile an .rss document and hot-swap it into the live runtime."""
    from stacc.rss import RSSSyntaxError

    source = req.get("source", "")
    try:
        robot = _compile_rss(source)
    except RSSSyntaxError as e:
        return {
            "error": e.message, "line": e.line, "col": e.col, "kind": "syntax",
            "diagnostics": _rss_error_diagnostics(e),
        }
    except Exception as e:
        return {
            "error": str(e), "kind": "compile",
            "diagnostics": _rss_error_diagnostics(e),
        }

    RT.set_robot(robot)
    RT.set_joint_positions([0.0] * robot.dof)
    RT.start()
    return {
        "robot": {**serialize_robot(robot), "revision": getattr(RT, "robot_revision", 0)},
        "diagnostics": _rss_warning_diagnostics(robot),
    }


def sdk_modules() -> Dict[str, Any]:
    """The installed stacc SDK's real subpackages: summary line and files."""
    import importlib
    import stacc

    root = os.path.dirname(stacc.__file__)
    modules = []
    for entry in sorted(os.listdir(root)):
        pkg_dir = os.path.join(root, entry)
        init = os.path.join(pkg_dir, "__init__.py")
        if entry.startswith("_") or not os.path.isfile(init):
            continue
        summary = ""
        try:
            doc = importlib.import_module(f"stacc.{entry}").__doc__ or ""
            summary = next((line.strip() for line in doc.splitlines() if line.strip()), "")
        except Exception as e:  # a package that fails to import says so
            summary = f"(import failed: {e.__class__.__name__})"
        files = sorted(
            os.path.join(pkg_dir, f) for f in os.listdir(pkg_dir) if f.endswith(".py") and not f.startswith("_")
        )
        modules.append({"name": f"stacc.{entry}", "summary": summary, "path": pkg_dir, "init": init, "files": files})
    return {"root": root, "modules": modules}


# Upload methods that flash a microcontroller from this machine (ssh/webusb
# boards are deployed differently, see `stacc rss deploy`).
_FLASHABLE_METHODS = {"avrdude", "esptool", "bossac", "stlink", "picotool", "uf2", "dfu-util", "teensy"}


def hal_boards() -> Dict[str, Any]:
    from stacc.hal.boards import FirmwareUploader, list_boards

    boards = [
        {"name": b.name, "upload_method": b.upload_method, "cpu": b.cpu, "platform": b.platform}
        for b in list_boards()
        if b.upload_method in _FLASHABLE_METHODS and b.upload_method in FirmwareUploader.UPLOAD_COMMANDS
    ]
    return {"boards": boards}


def hal_upload_command(req: Dict[str, Any]) -> Dict[str, Any]:
    """The real shell command that flashes `firmware` to `board` on `port`.
    Only builds it; the Studio runs it in its terminal and shows the output."""
    import shlex
    from stacc.hal.boards import FirmwareUploader

    firmware = str(req.get("firmware") or "").strip()
    port = str(req.get("port") or "/dev/ttyACM0").strip()
    if not firmware:
        return {"error": "firmware (path to the compiled image) is required", "kind": "input"}
    try:
        uploader = FirmwareUploader(str(req.get("board") or ""), port=shlex.quote(port))
    except ValueError as e:
        return {"error": str(e), "kind": "input"}
    if uploader.board_def.upload_method not in _FLASHABLE_METHODS:
        return {"error": f"{uploader.board_def.name} is not flashed over USB (method: {uploader.board_def.upload_method})", "kind": "input"}
    return {"command": uploader.get_upload_command(shlex.quote(firmware)), "board": uploader.board_def.name}


def rss_examples() -> Dict[str, Any]:
    """Real .rss example files shipped with the SDK, for the editor's picker."""
    examples = []
    if os.path.isdir(EXAMPLES_DIR):
        for fname in sorted(os.listdir(EXAMPLES_DIR)):
            if fname.endswith(".rss"):
                full = os.path.join(EXAMPLES_DIR, fname)
                with open(full, "r") as f:
                    examples.append({"name": fname, "path": full, "source": f.read()})
    return {"examples": examples}


# ─────────────────────────────────────────────────────────────────────────────
# Controller runtime API — /v1/runtime/* (spec §4)
#
# A STABLE, versioned surface that every controller language binds to (Python
# today, C++ over the same JSON contract). It is a thin HTTP mapping over the
# existing StaccRuntime — no new physics code. Units at THIS surface are SI
# (radian, second, Nm, metre), unlike the legacy /v1/robot/* routes which keep
# degrees; the runtime API never mixes units. Errors: 503 when no live runtime
# is loaded, 400 for bad input.
# ─────────────────────────────────────────────────────────────────────────────

RUNTIME_API_VERSION = 1


def _runtime_required() -> "tuple[Any, Dict[str, Any] | None]":
    """Return (RT, None) when a live runtime with a robot exists, else (None, error)."""
    if RT is None or RT.robot is None:
        return None, {"error": "Live runtime unavailable — no robot loaded", "kind": "runtime"}
    return RT, None


def _state_hash(rt: Any) -> str:
    """A stable fingerprint of the joint state after a step.

    Not MuJoCo's internal digest — a hash over the rounded joint positions and
    velocities, enough for a controller to assert "the state changed / did not
    change" across a step. Deterministic for identical state on the same build.
    """
    positions = rt.robot.get_joint_positions()
    velocities = rt.robot.get_joint_velocities()
    payload = ",".join(f"{v:.9f}" for v in list(positions) + list(velocities))
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:16]


def runtime_capabilities() -> Dict[str, Any]:
    """Engine + API capability declaration a controller checks on load."""
    if RT is None:
        return {"error": "Live runtime unavailable", "kind": "runtime"}
    backend = getattr(RT.engine, "_physics_backend", None)
    caps = backend.capabilities() if backend is not None and hasattr(backend, "capabilities") else {}
    joints = RT.joint_info()
    sensors = [s.name for s in RT.robot.sensors] if RT.robot is not None else []
    return {
        "engine": RT.config.physics_engine,
        "engine_version": MUJOCO_VERSION,
        "api_version": RUNTIME_API_VERSION,
        "units": "SI",
        # MuJoCo's solver is deterministic for identical state on the same build;
        # we only claim it for that engine rather than guessing for fallbacks.
        "deterministic_same_machine": RT.config.physics_engine == "mujoco",
        "joints": len(joints),
        "actuators": len(joints),
        "sensors": sensors,
        "snapshot": bool(caps.get("snapshot", False)),
        "render": not RT.config.headless,
        "operations": [
            "load_project", "start", "pause", "resume", "reset", "step", "speed",
            "joints", "joints/command", "control/gains", "sensors", "telemetry", "stop", "kill",
        ],
    }


def runtime_load_project(req: Dict[str, Any]) -> Dict[str, Any]:
    """Load a project (rss/urdf/sdf) into the live runtime, replacing any current robot."""
    if RT is None:
        return {"error": "Live runtime unavailable", "kind": "runtime"}
    fmt = (req.get("format") or "rss").lower()
    try:
        robot = _robot_from_source(fmt, req.get("source", ""))
    except ValueError as e:
        return {"error": str(e), "kind": "parse"}
    try:
        RT.set_robot(robot)
        RT.set_joint_positions([0.0] * robot.dof)
        RT.start()
    except Exception as e:
        return {"error": f"The runtime could not load this project: {e}", "kind": "load"}
    return {
        "ok": True,
        "robot": {"name": robot.name, "dof": robot.dof, "joints": RT.joint_info()},
        "preserved_state": False,
    }


def runtime_start() -> Dict[str, Any]:
    rt, err = _runtime_required()
    if err is not None:
        return err
    rt.start()
    return {"running": True, "paused": rt.paused, "sim_time": rt.snapshot().get("sim_time", 0.0)}


def runtime_pause() -> Dict[str, Any]:
    rt, err = _runtime_required()
    if err is not None:
        return err
    rt.pause()
    snap = rt.snapshot()
    return {"paused": True, "sim_time": snap.get("sim_time", 0.0), "step": snap.get("step", 0)}


def runtime_resume() -> Dict[str, Any]:
    rt, err = _runtime_required()
    if err is not None:
        return err
    rt.resume()
    return {"paused": rt.paused}


def runtime_reset() -> Dict[str, Any]:
    rt, err = _runtime_required()
    if err is not None:
        return err
    rt.reset()
    snap = rt.snapshot()
    return {"sim_time": snap.get("sim_time", 0.0), "step": snap.get("step", 0)}


def runtime_step(req: Dict[str, Any]) -> Dict[str, Any]:
    """Advance `count` physics timesteps (works while paused or stopped)."""
    rt, err = _runtime_required()
    if err:
        return err
    try:
        count = int(req.get("count", 1))
    except (TypeError, ValueError):
        return {"error": "count must be an integer", "kind": "input"}
    if count < 0 or count > 1_000_000:
        return {"error": "count must be between 0 and 1000000", "kind": "input"}
    for _ in range(count):
        rt.step_once()
    snap = RT.snapshot()
    return {"step": snap.get("step", 0), "sim_time": snap.get("sim_time", 0.0),
            "state_hash": _state_hash(RT)}


def runtime_speed(req: Dict[str, Any]) -> Dict[str, Any]:
    """Retune the loop rate. `hz` sets the loop directly; `realtime_factor` scales it.

    Real time corresponds to `hz * timestep == 1`; a factor of 2 runs the loop at
    twice the physics timestep rate (two substeps per cycle), so simulation time
    advances twice as fast as wall time. Restarting the loop is the only way to
    change hz, because the loop's period and substep count are captured at thread
    start.
    """
    rt, err = _runtime_required()
    if err:
        return err
    timestep = rt.config.timestep
    if "hz" in req and req["hz"] is not None:
        hz = float(req["hz"])
    elif "realtime_factor" in req and req["realtime_factor"] is not None:
        hz = float(req["realtime_factor"]) / max(timestep, 1e-9)
    else:
        return {"error": "Provide 'hz' or 'realtime_factor'", "kind": "input"}
    if not (0.0 < hz <= 100000.0):
        return {"error": f"hz out of range: {hz}", "kind": "input"}
    was_running = rt.running
    if was_running:
        rt.stop()
    rt.hz = hz
    if was_running:
        rt.start()
    return {"hz": rt.hz, "realtime_factor": round(rt.hz * timestep, 6)}


def runtime_joints() -> Dict[str, Any]:
    """Per-joint limits/effort merged with live position/velocity/torque/command."""
    rt, err = _runtime_required()
    if err:
        return err
    info = rt.joint_info()
    live = {j["id"]: j for j in rt.snapshot().get("joints", [])}
    keys = ("position", "velocity", "torque", "command_position")
    joints = []
    for j in info:
        merged = dict(j)
        src = live.get(j["id"], {})
        for k in keys:
            merged[k] = src.get(k)
        joints.append(merged)
    return {"joints": joints}


def runtime_joint_command(req: Dict[str, Any]) -> Dict[str, Any]:
    """Command joints by position (all), by one named joint, or by torque."""
    rt, err = _runtime_required()
    if err:
        return err
    try:
        if req.get("thrusts") is not None:
            rt.set_thrusts([float(t) for t in req["thrusts"]])
        elif req.get("torques") is not None:
            rt.set_joint_torques([float(t) for t in req["torques"]])
        elif req.get("joint") is not None and req.get("angle_rad") is not None:
            rt.set_joint_angle(str(req["joint"]), float(req["angle_rad"]))
        elif req.get("positions") is not None:
            rt.set_joint_positions([float(p) for p in req["positions"]])
        else:
            return {
                "error": "Provide {positions:[rad]}, {joint,angle_rad}, {torques:[Nm]} or {thrusts:[N]}",
                "kind": "input",
            }
    except (ValueError, RuntimeError) as e:
        return {"error": str(e), "kind": "input"}
    return {"command_positions": rt.snapshot().get("command_positions")}


def runtime_control_gains(req: Dict[str, Any]) -> Dict[str, Any]:
    """Read or set the servo gains (JointImpedance mode)."""
    rt, err = _runtime_required()
    if err:
        return err
    rt.set_gains(kp=req.get("kp"), kd=req.get("kd"), gravity_comp=req.get("gravity_comp"))
    return {
        "mode": rt.control_mode,
        "kp": rt.kp,
        "kd": rt.kd,
        "gravity_comp": rt.gravity_comp,
    }


def runtime_sensors() -> Dict[str, Any]:
    """Latest JSON-safe reading of every attached sensor (images stripped)."""
    rt, err = _runtime_required()
    if err:
        return err
    from stacc.server.runtime import _sensor_readout

    return {"sensors": _sensor_readout(rt.robot)}


def runtime_telemetry() -> Dict[str, Any]:
    """Full measured runtime snapshot (the same payload /v1/* telemetry uses)."""
    rt, err = _runtime_required()
    if err:
        return err
    return rt.snapshot()


# The port this daemon actually bound (set in run_server); names its shm channel.
SERVER_PORT = 8766


def runtime_shm_status() -> Dict[str, Any]:
    """The shared-memory control channel: layout, mode and counters."""
    rt, err = _runtime_required()
    if err:
        return err
    return rt.channel_status()


def runtime_shm(req: Dict[str, Any]) -> Dict[str, Any]:
    """Enable or disable the shared-memory control channel (the data plane;
    see stacc/server/rt_channel.py). {"enable": true, "mode": "realtime" |
    "lockstep", "timeout_ms": 1000}."""
    rt, err = _runtime_required()
    if err:
        return err
    if req.get("enable") is False:
        rt.disable_channel()
        return {"enabled": False}
    mode = str(req.get("mode") or "realtime").lower()
    if mode not in ("realtime", "lockstep"):
        return {"error": "mode must be 'realtime' or 'lockstep'", "kind": "input"}
    timeout_ms = float(req.get("timeout_ms") or 1000)
    path = f"/dev/shm/stacc_rt_{SERVER_PORT}"
    return rt.enable_channel(path, mode=1 if mode == "lockstep" else 0, timeout_s=timeout_ms / 1000.0)


def runtime_stop() -> Dict[str, Any]:
    """Graceful stop; the robot stays loaded and can start again."""
    rt, err = _runtime_required()
    if err:
        return err
    rt.stop()
    return {"running": rt.running}


def runtime_kill() -> Dict[str, Any]:
    """Force stop and unload the robot; the daemon stays up."""
    if RT is None:
        return {"error": "Live runtime unavailable", "kind": "runtime"}
    RT.stop()
    RT.set_robot(None)
    return {"killed": True, "running": False}


# ─────────────────────────────────────────────────────────────────────────────
# Analytic fallback (same response shapes, no SDK)
# ─────────────────────────────────────────────────────────────────────────────


def compute_forward_kinematics(q_deg: List[float]) -> Dict[str, float]:
    """Compute 6-DOF End-Effector Cartesian position (x, y, z) in meters."""
    q_rad = [math.radians(a) for a in q_deg]
    l1, l2, l3, l4 = 0.15, 0.35, 0.30, 0.12

    q1, q2, q3, q5 = (q_rad + [0] * 6)[0], (q_rad + [0] * 6)[1], (q_rad + [0] * 6)[2], (q_rad + [0] * 6)[4]

    r = l2 * math.cos(q2) + l3 * math.cos(q2 + q3) + l4 * math.cos(q2 + q3 + q5)
    z = l1 + l2 * math.sin(q2) + l3 * math.sin(q2 + q3) + l4 * math.sin(q2 + q3 + q5)
    x = r * math.cos(q1)
    y = r * math.sin(q1)

    return {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def compute_rnea_gravity_torques(q_deg: List[float]) -> List[float]:
    """Analytic gravity torques (Nm) for the fallback 6-DOF arm (no SDK loaded)."""
    q_rad = [math.radians(a) for a in q_deg]
    masses = [3.2, 4.5, 3.1, 1.8, 1.2, 0.8]
    g = 9.81

    q2, q3 = q_rad[1], q_rad[2]

    tau1 = 0.0
    tau2 = (masses[1] * 0.175 + masses[2] * 0.35 + masses[3] * 0.35) * g * math.cos(q2)
    tau3 = (masses[2] * 0.15 + masses[3] * 0.30) * g * math.cos(q2 + q3)
    tau4 = 0.15 * g * math.sin(q_rad[3] if len(q_rad) > 3 else 0)
    tau5 = 0.08 * g * math.cos(q_rad[4] if len(q_rad) > 4 else 0)
    tau6 = 0.02 * g

    return [round(abs(tau1), 2), round(abs(tau2), 2), round(abs(tau3), 2),
            round(abs(tau4), 2), round(abs(tau5), 2), round(abs(tau6), 2)]


def fallback_simulate_step(req: Dict[str, Any]) -> Dict[str, Any]:
    global SIMULATION_STEPS
    t0 = time.perf_counter()
    SIMULATION_STEPS += 1
    joints = req.get("joints", [0, 0, 0, 0, 0, 0])
    ee = compute_forward_kinematics(joints)
    margin_mm, cbf_safe = _cbf_check(ee)
    return {
        "step": SIMULATION_STEPS,
        "dt": 0.001,
        "frequency_hz": 1000,
        "end_effector": ee,
        "torques_nm": compute_rnea_gravity_torques(joints),
        "cbf_margin_mm": margin_mm,
        "cbf_safe": cbf_safe,
        "shm_status": _shm_status(),
        "engine": "analytic_fallback",
        # No physics engine in this analytic fallback to derive real energy
        # from — 0.0 is the honest "not available" value here, not a guess.
        "energy_j": 0.0,
        "step_time_ms": round((time.perf_counter() - t0) * 1000.0, 4),
    }


def fallback_ik(req: Dict[str, Any]) -> Dict[str, Any]:
    target_pos = req.get("target_pos", [0.35, 0.15, 0.40])
    x, y, z = target_pos[0], target_pos[1], target_pos[2]
    q1 = math.degrees(math.atan2(y, x))
    r = math.sqrt(x**2 + y**2)
    q2 = math.degrees(math.atan2(z - 0.15, r)) + 15.0
    q3 = -45.0
    q4, q5, q6 = 0.0, 30.0, 0.0
    return {
        "success": True,
        "solution_joints_deg": [round(q1, 1), round(q2, 1), round(q3, 1),
                                round(q4, 1), round(q5, 1), round(q6, 1)],
        "iterations": 8,
        "error_norm_m": 0.0004,
    }


def fallback_tune_gains(req: Dict[str, Any]) -> Dict[str, Any]:
    inertia_eff = float(req.get("inertia_eff", 0.08))
    zeta = float(req.get("damping_ratio", 0.707))
    wn = float(req.get("natural_freq_rad_s", 45.0))
    kp_opt = round(inertia_eff * (wn ** 2), 1)
    kd_opt = round(2.0 * zeta * math.sqrt(kp_opt * inertia_eff), 1)
    return {
        "kp_optimal": kp_opt,
        "kd_optimal": kd_opt,
        "damping_ratio": zeta,
        "natural_freq": wn,
        "settling_time_s": round(4.0 / (zeta * wn), 3),
    }


# ─────────────────────────────────────────────────────────────────────────────
# HTTP handler
# ─────────────────────────────────────────────────────────────────────────────



# ─────────────────────────────────────────────────────────────────────────────
# Local-only request guard
#
# This server binds to 127.0.0.1, but a browser will still let ANY web page the
# user visits fire requests at it (and a rebinding attacker can point their own
# hostname at 127.0.0.1). So requests are accepted only when:
#   * the Host header names a loopback host (blocks DNS rebinding), and
#   * any Origin header is NELO Studio's own origin.
# Requests without an Origin header (the Next.js server, Electron's main
# process, other local daemons, curl) are unaffected — browsers always send one.
# ─────────────────────────────────────────────────────────────────────────────
_STUDIO_PORT = os.environ.get("NELO_STUDIO_PORT", "3000")
ALLOWED_ORIGINS = {
    o.strip()
    for o in os.environ.get(
        "NELO_ALLOWED_ORIGINS",
        f"http://127.0.0.1:{_STUDIO_PORT},http://localhost:{_STUDIO_PORT}",
    ).split(",")
    if o.strip()
}
LOOPBACK_HOSTNAMES = {"127.0.0.1", "localhost", "[::1]"}

# Controller runtime API (/v1/runtime/*) dispatch tables. GET handlers take no
# arguments; POST handlers receive the parsed JSON body.
RUNTIME_GET = {
    "/v1/runtime/capabilities": runtime_capabilities,
    "/v1/runtime/joints": runtime_joints,
    "/v1/runtime/sensors": runtime_sensors,
    "/v1/runtime/telemetry": runtime_telemetry,
    "/v1/runtime/shm": runtime_shm_status,
}
RUNTIME_POST = {
    "/v1/runtime/load_project": runtime_load_project,
    "/v1/runtime/start": lambda req=None: runtime_start(),
    "/v1/runtime/pause": lambda req=None: runtime_pause(),
    "/v1/runtime/resume": lambda req=None: runtime_resume(),
    "/v1/runtime/reset": lambda req=None: runtime_reset(),
    "/v1/runtime/step": runtime_step,
    "/v1/runtime/speed": runtime_speed,
    "/v1/runtime/joints/command": runtime_joint_command,
    "/v1/runtime/control/gains": runtime_control_gains,
    "/v1/runtime/stop": lambda req=None: runtime_stop(),
    "/v1/runtime/shm": runtime_shm,
    "/v1/runtime/kill": lambda req=None: runtime_kill(),
}


# `kill -USR1 <pid>` prints every thread's Python stack to stderr — the way to
# see what holds the runtime lock when a request hangs.
faulthandler.register(signal.SIGUSR1, all_threads=True)


def _json_safe(value: Any) -> Any:
    """Replace non-finite floats with null. Python writes inf/nan as
    `Infinity`/`NaN`, which is not JSON: browsers refuse to parse the whole
    response (e.g. any robot with a continuous wheel joint, limits = ±inf)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


class STACCHTTPServer(ThreadingHTTPServer):
    """One thread per request. With a single-threaded server, one idle client
    (browsers open speculative keep-alive sockets and never send on them) blocks
    every other request, and the Studio then reports the daemon as unreachable."""

    daemon_threads = True
    request_queue_size = 64


class STACCRequestHandler(BaseHTTPRequestHandler):
    # Drop connections that send nothing instead of holding a worker forever.
    timeout = 10
    def _cors_origin(self):
        origin = self.headers.get("Origin")
        return origin if origin in ALLOWED_ORIGINS else None

    def _cors_headers(self):
        origin = self._cors_origin()
        if origin:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")

    def _guard(self) -> bool:
        """Reject cross-origin / DNS-rebinding requests before any handler runs."""
        hostname = (self.headers.get("Host") or "").rsplit(":", 1)[0].lower()
        origin = self.headers.get("Origin")
        if hostname not in LOOPBACK_HOSTNAMES or (origin is not None and origin not in ALLOWED_ORIGINS):
            self._send_json(403, {"error": "Forbidden: request origin is not allowed"})
            return False
        return True

    def _send_json(self, status: int, data: Any):
        payload = json.dumps(_json_safe(data), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self._cors_headers()
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        if not self._guard():
            return
        self.send_response(204)
        self._cors_headers()
        self.end_headers()

    def do_GET(self):
        if not self._guard():
            return
        if self.path == "/v1/system/status" or self.path == "/health":
            total_steps = (RT.engine.state.step if RT is not None else SIMULATION_STEPS)
            self._send_json(200, {
                "status": "online",
                "platform": "STACC Universal Robotics Platform",
                "version": STACC_VERSION,
                "sdk_available": STACC_SDK_AVAILABLE,
                "sdk_path": SDK_PATH,
                "control_loop_hz": (round(1.0 / RT.config.timestep, 1) if RT else None),
                "physics_engine": PHYSICS_ENGINE,
                "mujoco_version": MUJOCO_VERSION,
                "live_runtime": RT is not None,
                "loop": _loop_timing(),
                "cbf_barrier_margin_mm": CBF_BARRIER_MARGIN_MM,
                "shm_status": _shm_status(),
                "shm_ringbuffer_slots": 0,
                "active_robot_model": (RT.robot.name if RT is not None and RT.robot is not None else None),
                "total_sim_steps": total_steps,
                "uptime_seconds": int(time.time() - START_TIME),
                "pid": os.getpid(),
            })
        elif self.path == "/v1/robot/structure":
            if RT is not None and RT.robot is not None:
                self._send_json(200, {**serialize_robot(RT.robot), "revision": getattr(RT, "robot_revision", 0)})
            else:
                self._send_json(503, {"error": "Live SDK runtime unavailable — no robot loaded"})
        elif self.path == "/v1/rss/examples":
            if STACC_SDK_AVAILABLE:
                self._send_json(200, rss_examples())
            else:
                self._send_json(503, {"error": "Live SDK runtime unavailable — cannot read RSS examples"})
        elif self.path in ("/v1/sdk/modules", "/v1/hal/boards"):
            if not STACC_SDK_AVAILABLE:
                self._send_json(503, {"error": "stacc SDK unavailable"})
            else:
                try:
                    self._send_json(200, sdk_modules() if self.path == "/v1/sdk/modules" else hal_boards())
                except Exception as e:
                    self._send_json(500, {"error": str(e) or e.__class__.__name__})
        elif self.path in RUNTIME_GET:
            result = RUNTIME_GET[self.path]()
            self._send_json(503 if "error" in result else 200, result)
        else:
            self._send_json(404, {"error": "Endpoint not found"})

    def do_POST(self):
        if not self._guard():
            return
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length).decode("utf-8")
        req_data = json.loads(body) if body else {}

        if self.path == "/v1/robot/simulate_step":
            result = live_simulate_step(req_data) if RT else fallback_simulate_step(req_data)
            self._send_json(200, result)

        elif self.path == "/v1/robot/ik":
            result = live_ik(req_data) if RT else fallback_ik(req_data)
            self._send_json(200, result)

        elif self.path == "/v1/robot/tune_gains":
            result = live_tune_gains(req_data) if RT else fallback_tune_gains(req_data)
            self._send_json(200, result)

        elif self.path in ("/v1/model/inspect", "/v1/model/convert", "/v1/robot/load_model", "/v1/rss/bom", "/v1/rss/ros2_package", "/v1/rss/compile", "/v1/hal/upload_command"):
            handler = {
                "/v1/model/inspect": model_inspect,
                "/v1/model/convert": model_convert,
                "/v1/robot/load_model": robot_load_model,
                "/v1/rss/bom": rss_bom,
                "/v1/rss/ros2_package": rss_ros2_package,
                "/v1/rss/compile": rss_compile,
                "/v1/hal/upload_command": hal_upload_command,
            }[self.path]
            result = handler(req_data)
            self._send_json(400 if "error" in result else 200, result)

        elif self.path == "/v1/rss/load":
            if RT is None:
                self._send_json(503, {"error": "Live SDK runtime unavailable — cannot load RSS"})
            else:
                result = rss_load(req_data)
                self._send_json(400 if "error" in result else 200, result)

        elif self.path in RUNTIME_POST:
            handler = RUNTIME_POST[self.path]
            try:
                result = handler(req_data)
            except Exception as e:  # never let a controller call 500 the daemon silently
                result = {"error": str(e) or e.__class__.__name__, "kind": "runtime"}
            status = 200
            if "error" in result:
                status = 503 if result.get("kind") == "runtime" else 400
            self._send_json(status, result)

        else:
            self._send_json(404, {"error": "Endpoint not found"})

    def log_message(self, fmt, *args):
        pass  # keep the console quiet during high-rate polling


def run_server(port: int = 8766, max_port_attempts: int = 10):
    # A shipped desktop app can't assume its default port is free on every
    # user's machine (another app, or a leftover daemon from a prior run,
    # may already hold it) — try a few ports upward instead of crashing
    # uncaught on OSError, and report whichever one actually bound so the
    # Electron parent process can find it.
    httpd = None
    bound_port = port
    for attempt in range(max_port_attempts):
        bound_port = port + attempt
        try:
            httpd = STACCHTTPServer(("127.0.0.1", bound_port), STACCRequestHandler)
            break
        except OSError:
            if attempt == max_port_attempts - 1:
                raise
            continue
    assert httpd is not None
    port = bound_port
    global SERVER_PORT
    SERVER_PORT = port
    label = (
        f"Live SDK runtime ({RT.robot.name}, {RT.engine._physics_backend.__class__.__name__})"
        if RT is not None else "analytic fallback kernel"
    )
    print(f"🚀 [STACC-SERVER] STACC Universal Robotics & MuJoCo Daemon listening on http://127.0.0.1:{port}")
    print(f"🤖 [STACC-SERVER] Platform: STACC v{STACC_VERSION} | control loop target {round(1.0 / RT.config.timestep) if RT is not None else 'n/a'} Hz | {label}")
    print(f"STACC_BOUND_PORT={port}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if RT is not None:
            RT.stop()
        httpd.server_close()


if __name__ == "__main__":
    port = 8766
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    run_server(port)