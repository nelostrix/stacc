"""Test: 3D viewer with swinging pendulum."""

import time

import mujoco
import mujoco.viewer

XML = """
<mujoco model="pendulum">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>

    <body name="arm" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.3" mass="1" diaginertia="0.01 0.01 0.001"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.6" size="0.03" rgba="0.2 0.8 0.4 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)
data.qpos[0] = 1.5
mujoco.mj_forward(model, data)

print("Opening viewer... (close window or Ctrl+C to stop)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 15:
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)

print("Viewer closed")
