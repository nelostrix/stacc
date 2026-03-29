"""test: verify Mujoco works standalone before Integration with stacc."""

import time

import mujoco

# A simple pendulum defined in MJCF XML
PENDULUM_XML = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <light pos="0 0 3"/>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02 0.3" rgba="0.2 0.7 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# Load model and create data
model = mujoco.MjModel.from_xml_string(PENDULUM_XML)
data = mujoco.MjData(model)

# Give the pendulum an initial push
data.qpos[0] = 1.0  # Start at 1 radian

# Simulate 1000 steps
print("Stepping physics...")
for i in range(1000):
    mujoco.mj_step(model, data)
    if i % 200 == 0:
        print(
            f"  step {i}: angle={data.qpos[0]:.4f} rad, velocity={data.qvel[0]:.4f} rad/s"
        )

print(f"\nFinal angle: {data.qpos[0]:.4f} rad")
print("SUCCESS: MuJoCo physics is working")
