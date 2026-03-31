import mujoco
import mujoco.viewer

from stacc.sim.mjcf_builder import robot_to_mjcf

# mujoco physics backend for stacc simulation engine.


class MuJoCoBackend:
    def __init__(self, robot, config):
        self.robot = robot
        self.config = config
        self.viewer = None
        # 1. Convert robot -> MJCF
        mjcf_xml = robot_to_mjcf(robot, config)
        self.model = mujoco.MjModel.from_xml_string(mjcf_xml)
        self.data = mujoco.MjData(self.model)

        # Initialize internal state:
        mujoco.mj_forward(self.model, self.data)

        self._joint_map = {}
        for i, joint in enumerate(robot.joints):
            if joint.type == "fixed":
                continue
            mj_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint.name)

            if mj_id >= 0:
                self._joint_map[i] = self.model.jnt_qposadr[mj_id]

        if config.render:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                print(f"   [mujoco]     Viewer Opened")

            except Exception as e:
                print(f"   [mujoco]    Could not open viewer: {e}")
                self.viewer = None

        self._step_count = 0
        self._render_every = max(1, int(1.0 / (60 * config.timestep)))

    def step(self, dt: float):
        """step physics, sync state ,update viewer."""
        for stacc_idx, qpos_adr in self._joint_map.items():
            self.robot._state["joint_positions"][stacc_idx] = self.data.qpos[qpos_adr]
            self.robot._state["joint_velocities"][stacc_idx] = self.data.qvel[qpos_adr]

            self._step_count += 1
            if self.viewer and self._step_count % self._render_every == 0:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    self.viewer = None  # User closed the window

        def close(self):
            if self.viewer:
                self.viewer.close()
