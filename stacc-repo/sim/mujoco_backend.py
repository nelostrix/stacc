import mujoco as mj
import mujoco_viewer as mjv

from stacc.compiler.mjcf_builder import robot_to_mjcf


class MuJoCoBackend:
    def __init__(self, robot, config):
        # 1. Convert robot -> MJCF
        xml = robot_to_mjcf(robot, config)

        # 2. Load Model
        self.model = mj.MjModel.from_xml_string(xml)

        # 3. Create data
        self.data = mj.MjData(self.model)

        # 4. Viewer
        self.viewer = None
        if config.render:
            self.viewer = mjv.launch_passive(self.model, self.data)
        # 5. Store robot
        self.robot = robot

        def step(self, dt: float):
            self.model.opt.timestep = dt
            mj.mj_step(self.model, self.data)
            if self.viewer:
                self.viewer.sync()
            for stacc_idx, qpos_adr in self.joint_map.items():
                actuator_idx = stacc_idx
                if actuator_idx < self.model.nu:
                    self.data.ctrl[actuator_idx] = self.robot._state["joint_torques"][
                        stacc_idx
                    ]
            mj.mj_step(self.model, self.data)

            for stacc_idx, qpos_adr in self._joint_map.items():
                self.robot._state["joint_positions"][stacc_idx] = self.data.qpos[
                    qpos_adr
                ]
                self.robot._state["joint_velocities"][stacc_idx] = self.data.qvel[
                    qpos_adr
                ]

        def close(self):
            if self.viewer:
                self.viewer.close()
