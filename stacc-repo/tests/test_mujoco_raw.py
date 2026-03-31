"""Test MuJoCo physics — guaranteed working pendulum."""

import mujoco
import numpy as np

XML = """
<mujoco model="pendulum">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1"/>

    <body name="arm" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.3" mass="1" diaginertia="0.01 0.01 0.001"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.6" size="0.02" rgba="0.2 0.8 0.4 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

# Set initial angle
data.qpos[0] = 1.0

# IMPORTANT: call mj_forward first to initialize internal state
mujoco.mj_forward(model, data)

print(f"DOF: {model.nv}, Bodies: {model.nbody}, Timestep: {model.opt.timestep}")
print(f"Initial angle: {data.qpos[0]:.4f}\n")

for i in range(1001):
    mujoco.mj_step(model, data)
    if i % 100 == 0:
        print(f"  step {i:4d}: angle={data.qpos[0]:+.4f}  velocity={data.qvel[0]:+.4f}")

print(f"\nFinal angle: {data.qpos[0]:.4f}")
if abs(data.qpos[0] - 1.0) > 0.01:
    print("SUCCESS: Pendulum is swinging (angle changed)")
else:
    print("PROBLEM: Angle didn't change — check MuJoCo version")
    print(f"MuJoCo version: {mujoco.__version__}")
