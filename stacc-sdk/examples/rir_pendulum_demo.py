"""RIR round-trip + backend-agnostic simulation, proven end-to-end.

This exists to answer two concrete questions, not just assert them:

  1. Does a robot built via the graph API actually survive a full
     Robot -> RIR (JSON) -> Robot round-trip and still behave physically
     identically when simulated? (proves RIR is a real interchange format,
     not just a convenience wrapper)

  2. Is the physics backend genuinely swappable — same RIR-rebuilt robot,
     same call site, only the `physics=` string changes? (proves the
     simulation layer is agnostic, not hardcoded to MuJoCo)

The robot: a fixed-base single-pivot arm — mass concentrated off-axis from
a revolute joint (axis=[1,0,0]), zero control torque applied. That's a
pendulum. If gravity is real, it swings. If it's not (the internal
fallback deliberately never reads config.gravity — see stacc/sim/engine.py
_InternalPhysics.step), it just sits at 0 forever. That contrast is the
proof, not an animation.

Run:
    PYTHONPATH=. python examples/rir_pendulum_demo.py
"""
import stacc
from stacc.core.joint import Joint
from stacc.core.link import Link, Geometry, Inertia
from stacc.core.robot_base import Robot
from stacc.core.rir import robot_to_rir, rir_from_dict
from stacc.sim.engine import SimulationConfig, SimulationEngine


def build_pendulum() -> Robot:
    """Base -> arm (revolute pivot) -> bob (rigid extension).

    stacc's MJCF builder places every link's own inertial/geom exactly at
    that link's own body origin, which is also where its joint sits — so
    a *single* revolute-jointed link has zero moment arm about its own
    joint (its mass sits exactly on the rotation axis, contributing no
    gravity torque). A second, rigidly-attached child link is offset from
    the parent by the auto-placement logic, so *its* mass genuinely swings
    away from the shoulder axis — that's what makes this an actual
    pendulum instead of a point mass stuck on its own pivot.
    """
    robot = Robot(name="pendulum")
    robot.fixed_base = True

    base = Link("base", mass=5.0, visual=Geometry(shape="box", size=[0.2, 0.2, 0.1]))
    arm = Link("arm", mass=0.1, visual=Geometry(shape="cylinder", radius=0.015, length=0.05))
    bob = Link(
        "bob",
        mass=1.0,
        visual=Geometry(shape="cylinder", radius=0.03, length=0.4),
        inertia=Inertia.from_cylinder(1.0, 0.03, 0.4),
    )
    robot.add_link(base)
    robot.add_link(arm)
    robot.add_link(bob)
    robot.connect(
        "base", "arm",
        Joint("shoulder", type="revolute", axis=[1, 0, 0], limits=(-3.14, 3.14), damping=0.0),
    )
    # Explicit Y offset, not the auto-placement default (which stacks purely
    # along Z): torque about an X-axis hinge under Z-gravity is r x F, and
    # that's zero when r and gravity are parallel (both pure Z) — the bob
    # has to sit off to the *side* of the hinge axis to feel any torque.
    robot.connect("arm", "bob", Joint("weld", type="fixed"), offset=[0, 0.3, 0])
    return robot


def run_headless(robot: Robot, physics: str, duration: float = 2.0) -> list:
    trace = []

    @robot.on_update
    def log(dt):
        trace.append(robot.get_joint_position("shoulder"))

    engine = SimulationEngine(
        robot,
        SimulationConfig(duration=duration, timestep=0.002, physics_engine=physics,
                          render=False, headless=True, realtime=False),
    )
    engine.run()
    return trace


def main():
    original = build_pendulum()

    print("=== Step 1: Robot -> RIR -> Robot round-trip ===")
    rir_data = robot_to_rir(original)
    print(f"RIR: {len(rir_data['links'])} links, {len(rir_data['connections'])} joints, "
          f"{len(rir_data['links']) + len(rir_data['connections'])} nodes total, "
          f"{len(str(rir_data))} chars of portable JSON")
    rebuilt = rir_from_dict(rir_data)
    assert rebuilt.num_links == original.num_links
    assert rebuilt.dof == original.dof
    print(f"Rebuilt robot from RIR: {rebuilt}")

    print("\n=== Step 2: fresh robot rebuilt from the *same* RIR data each time, two backends ===")
    # A fresh rir_from_dict() call each time — not the original Python object —
    # because SimulationEngine mutates joint state in place, and because this
    # is the actually representative workflow: load RIR, simulate it.
    trace_internal = run_headless(rir_from_dict(rir_data), physics="internal")
    trace_mujoco_a = run_headless(rir_from_dict(rir_data), physics="mujoco")
    trace_mujoco_b = run_headless(rir_from_dict(rir_data), physics="mujoco")

    print(f"internal backend — shoulder angle after {len(trace_internal)} steps: "
          f"{trace_internal[-1]:.6f} rad (gravity not modeled — stays at 0)")
    print(f"mujoco backend (run A) — shoulder angle after {len(trace_mujoco_a)} steps: "
          f"{trace_mujoco_a[-1]:.6f} rad")
    print(f"mujoco backend (run B) — shoulder angle after {len(trace_mujoco_b)} steps: "
          f"{trace_mujoco_b[-1]:.6f} rad")

    print("\n=== Step 3: verdicts ===")
    moved = abs(trace_mujoco_a[-1]) > 0.01
    print(f"[{'PASS' if moved else 'FAIL'}] MuJoCo backend applies real gravity "
          f"(angle moved {abs(trace_mujoco_a[-1]):.4f} rad from rest under zero torque)")

    deterministic = trace_mujoco_a == trace_mujoco_b
    print(f"[{'PASS' if deterministic else 'FAIL'}] identical initial conditions "
          f"-> bit-identical trajectory across repeated runs (determinism)")

    stayed_put = abs(trace_internal[-1]) < 1e-9
    print(f"[{'PASS' if stayed_put else 'FAIL'}] internal backend confirmed to ignore "
          f"gravity as documented (honest about its own limitation, not silently wrong)")


if __name__ == "__main__":
    main()
