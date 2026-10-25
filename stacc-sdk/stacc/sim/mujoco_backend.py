"""MuJoCo physics backend for the stacc simulation engine.

Converts a stacc Robot into an MJCF model via mjcf_builder, then
steps MuJoCo physics and synchronises state between stacc and MuJoCo.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Dict, Optional

import mujoco
import numpy as np
import mujoco
import mujoco.viewer

from stacc.sim.backend import PhysicsBackend, register_backend
from stacc.sim.mjcf_builder import robot_to_mjcf

if TYPE_CHECKING:
    from stacc.core.robot_base import Robot
    from stacc.sim.engine import SimulationConfig


class MuJoCoBackend(PhysicsBackend):
    """MuJoCo physics backend wrapping a robot simulation.

    Args:
        robot: A stacc Robot instance
        config: Simulation configuration
    """

    @classmethod
    def is_available(cls) -> bool:
        try:
            import mujoco  # noqa: F401
            import numpy as np  # noqa: F401
            return True
        except ImportError:
            return False

    def __init__(self, robot: "Robot", config: "SimulationConfig"):
        self.robot = robot
        self.config = config
        self.viewer = None
        self._step_count = 0

        mjcf_xml = robot_to_mjcf(robot, config)
        self.model = mujoco.MjModel.from_xml_string(mjcf_xml)
        self.data = mujoco.MjData(self.model)

        mujoco.mj_forward(self.model, self.data)

        # qpos and qvel use different address spaces whenever a free/ball
        # joint appears anywhere earlier in the model (e.g. a non-fixed
        # base): a free joint occupies 7 qpos slots (xyz + quaternion) but
        # only 6 qvel slots (xyz + angular velocity), so every joint after
        # it has qposadr != dofadr. Track both instead of reusing qposadr
        # for qvel indexing too.
        self._joint_map: Dict[int, int] = {}
        self._joint_dof_map: Dict[int, int] = {}
        for i, joint in enumerate(robot.joints):
            if joint.type == "fixed":
                continue
            mj_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint.name)
            if mj_id >= 0:
                self._joint_map[i] = self.model.jnt_qposadr[mj_id]
                self._joint_dof_map[i] = self.model.jnt_dofadr[mj_id]

        # stacc joint index -> actuator index, matched by the joint each actuator
        # drives (MJCF actuator order follows the body tree, not robot.joints).
        self._actuator_of: Dict[int, int] = {}
        for stacc_idx in self._joint_map:
            mj_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, robot.joints[stacc_idx].name)
            for a in range(self.model.nu):
                if self.model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and self.model.actuator_trnid[a, 0] == mj_joint:
                    self._actuator_of[stacc_idx] = a
                    break

        # Rotor thrusters, in robot.thrusters order -> their actuator index.
        self._thruster_actuators = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_thrust")
            for name in getattr(robot, "thrusters", [])
        ]

        # A mobile (non-fixed-base) robot's root body carries a freejoint named
        # "root" (see mjcf_builder); its 7 qpos slots are the base pose.
        root_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self._root_qpos_adr = int(self.model.jnt_qposadr[root_id]) if root_id >= 0 else None

        if config.render:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                print("   [mujoco] Viewer opened")
            except Exception as e:
                print(f"   [mujoco] Could not open viewer: {e}")
                self.viewer = None

        self._render_every = max(1, int(1.0 / (60 * config.timestep)))

        for sensor in robot.sensors:
            if hasattr(sensor, "set_mujoco_backend"):
                sensor.set_mujoco_backend(self.model, self.data)

    def reset(self):
        """Reset MuJoCo model state to initial configuration."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0

    def step(self, dt: float):
        """Step physics, sync state, and update viewer.

        `robot._state["servo"]` = {"kp", "kd", "targets"} runs every actuator as
        a joint servo inside MuJoCo (stable: its damping is integrated
        implicitly); without it, `joint_torques` are applied directly.
        """
        servo = self.robot._state.get("servo")
        torques = self.robot._state["joint_torques"]
        for stacc_idx, a in self._actuator_of.items():
            if servo is not None:
                kp, kd = float(servo["kp"]), float(servo["kd"])
                self.model.actuator_gainprm[a, 0] = kp
                self.model.actuator_biasprm[a, 0:3] = (0.0, -kp, -kd)
                self.data.ctrl[a] = servo["targets"][stacc_idx]
            else:
                self.model.actuator_gainprm[a, 0] = 1.0
                self.model.actuator_biasprm[a, 0:3] = (0.0, 0.0, 0.0)
                self.data.ctrl[a] = torques[stacc_idx]

        thrusts = self.robot._state.get("thrusts") or []
        for k, a in enumerate(self._thruster_actuators):
            if a >= 0:
                self.data.ctrl[a] = float(thrusts[k]) if k < len(thrusts) else 0.0

        mujoco.mj_step(self.model, self.data)

        # Report the torque each actuator actually applied (after force limits).
        if servo is not None:
            for stacc_idx, a in self._actuator_of.items():
                torques[stacc_idx] = float(self.data.actuator_force[a])

        for stacc_idx, qpos_adr in self._joint_map.items():
            self.robot._state["joint_positions"][stacc_idx] = self.data.qpos[qpos_adr]
            dof_adr = self._joint_dof_map[stacc_idx]
            self.robot._state["joint_velocities"][stacc_idx] = self.data.qvel[dof_adr]

        if self._root_qpos_adr is not None:
            self._sync_base_pose()

        self._step_count += 1
        if self.viewer and self._step_count % self._render_every == 0:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer = None

    def _sync_base_pose(self):
        """Copy the free base's world pose into robot state: `pose` as
        [x, y, z, roll, pitch, yaw] and `base_quat` as MuJoCo's [w, x, y, z]
        (renderers should prefer the quaternion — no Euler singularities)."""
        a = self._root_qpos_adr
        x, y, z, qw, qx, qy, qz = (float(v) for v in self.data.qpos[a:a + 7])
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.robot._state["pose"] = [x, y, z, roll, pitch, yaw]
        self.robot._state["base_quat"] = [qw, qx, qy, qz]

    def close(self):
        """Close the MuJoCo viewer."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None

    def capabilities(self) -> Dict[str, bool]:
        """SimEngine capability declaration conforming to Phase 4 Step 4.5."""
        return {
            "step": True,
            "reset": True,
            "snapshot": True,
            "restore": True,
            "read_state": True,
            "write_controls": True,
            "real_time_loop": True,
        }

    def snapshot(self) -> bytes:
        """Capture bit-identical MuJoCo state snapshot using mj_getState."""
        spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        sz = mujoco.mj_stateSize(self.model, spec)
        buf = np.zeros(sz, dtype=np.float64)
        mujoco.mj_getState(self.model, self.data, buf, spec)
        return buf.tobytes()

    def restore(self, snapshot_bytes: bytes) -> None:
        """Restore bit-identical MuJoCo state from binary snapshot using mj_setState."""
        spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        sz = mujoco.mj_stateSize(self.model, spec)
        buf = np.frombuffer(snapshot_bytes, dtype=np.float64)
        if len(buf) != sz:
            raise ValueError(f"Snapshot size mismatch: expected {sz} floats, got {len(buf)}")
        mujoco.mj_setState(self.model, self.data, buf, spec)
        mujoco.mj_forward(self.model, self.data)

    def read_state(self) -> Dict[str, Any]:
        """Read high-resolution engine state vectors."""
        return {
            "time": float(self.data.time),
            "step_count": self._step_count,
            "qpos": self.data.qpos.copy().tolist(),
            "qvel": self.data.qvel.copy().tolist(),
        }

    def write_controls(self, controls: Any) -> None:
        """Write motor/actuator controls to MuJoCo data.ctrl."""
        if isinstance(controls, (list, tuple)):
            for i, val in enumerate(controls):
                if i < self.model.nu:
                    self.data.ctrl[i] = float(val)
        elif isinstance(controls, dict):
            for k, val in controls.items():
                idx = int(k)
                if idx < self.model.nu:
                    self.data.ctrl[idx] = float(val)

    def __repr__(self) -> str:
        return (
            f"MuJoCoBackend(robot='{self.robot.name}', "
            f"joints={len(self._joint_map)}, viewer={'open' if self.viewer else 'closed'})"
        )


register_backend("mujoco", MuJoCoBackend)
