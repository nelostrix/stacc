"""Test: verify the 3D viewer opens and renders."""

import time

import mujoco
import mujoco.viewer

PENDULUM_XML = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="pendulum" pos="0 0 1.5">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.04 0.5" rgba="0.2 0.6 0.9 1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(PENDULUM_XML)
data = mujoco.MjData(model)
data.qpos[0] = 1.5  # Start tilted

# This opens a 3D window — drag to rotate, scroll to zoom
print("Opening viewer... (close window or Ctrl+C to stop)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 10:
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)  # ~500 FPS physics, viewer syncs when it can

print("SUCCESS: Viewer opened and rendered")
