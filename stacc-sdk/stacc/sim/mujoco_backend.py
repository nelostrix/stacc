"""MuJoCo physics backend for the stacc simulation engine.

Converts a stacc Robot into an MJCF model via mjcf_builder, then
steps MuJoCo physics and synchronises state between stacc and MuJoCo.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, Optional

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
        """Step physics, sync state, and update viewer."""
        for stacc_idx, qpos_adr in self._joint_map.items():
            if stacc_idx < self.model.nu:
                self.data.ctrl[stacc_idx] = self.robot._state["joint_torques"][
                    stacc_idx
                ]

        mujoco.mj_step(self.model, self.data)

        for stacc_idx, qpos_adr in self._joint_map.items():
            self.robot._state["joint_positions"][stacc_idx] = self.data.qpos[qpos_adr]
            dof_adr = self._joint_dof_map[stacc_idx]
            self.robot._state["joint_velocities"][stacc_idx] = self.data.qvel[dof_adr]

        self._step_count += 1
        if self.viewer and self._step_count % self._render_every == 0:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer = None

    def close(self):
        """Close the MuJoCo viewer."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None

    def __repr__(self) -> str:
        return (
            f"MuJoCoBackend(robot='{self.robot.name}', "
            f"joints={len(self._joint_map)}, viewer={'open' if self.viewer else 'closed'})"
        )


register_backend("mujoco", MuJoCoBackend)
