"""Test: load a URDF file and simulate it in MuJoCo.

This test creates a standard URDF file for a 7-DOF robotic arm,
parses it with the stacc URDF parser, and runs it in MuJoCo.
"""

import math
import os

import stacc
from stacc.control.pid import PIDController
from stacc.core.urdf_parser import load_urdf, load_urdf_string

# ═══════════════════════════════════════════
# A real URDF: 7-DOF robotic arm
# ═══════════════════════════════════════════

ARM_URDF = """<?xml version="1.0"?>
<robot name="stacc_7dof_arm">

  <material name="gray">
    <color rgba="0.6 0.6 0.6 1"/>
  </material>
  <material name="blue">
    <color rgba="0.2 0.4 0.8 1"/>
  </material>
  <material name="orange">
    <color rgba="0.9 0.5 0.2 1"/>
  </material>

  <!-- Base (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.08" length="0.05"/></geometry>
      <material name="gray"/>
    </visual>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry><cylinder radius="0.05" length="0.15"/></geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.005" iyy="0.005" izz="0.002" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9" effort="87" velocity="2.175"/>
    <dynamics damping="1.0"/>
  </joint>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <geometry><cylinder radius="0.05" length="0.2"/></geometry>
      <material name="gray"/>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.008" iyy="0.008" izz="0.002" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.76" upper="1.76" effort="87" velocity="2.175"/>
    <dynamics damping="1.0"/>
  </joint>

  <!-- Link 3 -->
  <link name="link3">
    <visual>
      <geometry><cylinder radius="0.04" length="0.2"/></geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.006" iyy="0.006" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9" effort="87" velocity="2.175"/>
    <dynamics damping="0.8"/>
  </joint>

  <!-- Link 4 -->
  <link name="link4">
    <visual>
      <geometry><cylinder radius="0.04" length="0.2"/></geometry>
      <material name="orange"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.006" iyy="0.006" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.07" upper="-0.07" effort="87" velocity="2.175"/>
    <dynamics damping="0.8"/>
  </joint>

  <!-- Link 5 -->
  <link name="link5">
    <visual>
      <geometry><cylinder radius="0.035" length="0.15"/></geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.003" iyy="0.003" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9" effort="12" velocity="2.61"/>
    <dynamics damping="0.5"/>
  </joint>

  <!-- Link 6 -->
  <link name="link6">
    <visual>
      <geometry><cylinder radius="0.035" length="0.1"/></geometry>
      <material name="orange"/>
    </visual>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.017" upper="3.75" effort="12" velocity="2.61"/>
    <dynamics damping="0.5"/>
  </joint>

  <!-- Link 7 (end effector mount) -->
  <link name="link7">
    <visual>
      <geometry><sphere radius="0.04"/></geometry>
      <material name="gray"/>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="joint7" type="revolute">
    <parent link="link6"/>
    <child link="link7"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9" effort="12" velocity="2.61"/>
    <dynamics damping="0.3"/>
  </joint>

</robot>
"""

# ═══════════════════════════════════════════
# Parse and simulate
# ═══════════════════════════════════════════

print("Parsing URDF...")
robot = load_urdf_string(ARM_URDF)
print(f"Loaded: {robot}")
print(f"  Links: {[l.name for l in robot.links]}")
print(f"  Joints: {[j.name for j in robot.joints]}")
print(f"  DOF: {robot.dof}")
print()

# PID controllers for all 7 joints
pids = [PIDController(kp=200.0, kd=20.0) for _ in range(robot.dof)]


@robot.on_update
def control(dt):
    pos = robot.get_joint_positions()
    t = robot.time

    # Smooth wave motion through all joints
    targets = [
        0.5 * math.sin(t * 0.3),  # joint1: base rotation
        0.4 * math.sin(t * 0.5 + 0.5),  # joint2: shoulder
        0.3 * math.sin(t * 0.4 + 1.0),  # joint3: upper arm rotation
        -1.5 + 0.3 * math.sin(t * 0.6),  # joint4: elbow (stays bent)
        0.5 * math.sin(t * 0.7 + 1.5),  # joint5: forearm rotation
        1.5 + 0.3 * math.sin(t * 0.8),  # joint6: wrist
        0.4 * math.sin(t * 1.0 + 2.0),  # joint7: end effector rotation
    ]

    torques = []
    for i in range(min(len(pids), len(pos))):
        torques.append(pids[i].compute(targets[i] - pos[i], dt))
    robot.set_joint_torques(torques)


print("Starting simulation...")
stacc.simulate(robot, duration=20, render=True, physics="mujoco")
