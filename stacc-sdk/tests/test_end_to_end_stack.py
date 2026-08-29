"""Comprehensive validation suite for STACC End-to-End Robotics OS Stack."""

import math
import numpy as np
import pytest

from stacc.estimation import ErrorStateEKF, MomentumObserver
from stacc.control import HierarchicalWBC, TaskObjective
from stacc.converters import ModelTranspiler
from stacc.bridge import ROS2Bridge, ZenohTelemetryStreamer
from stacc.planning import CHOMPPlanner, VoxelOctoMap, DualArmCoordinator
from stacc.ai import VLATaskGrounder, VisuomotorPolicyRuntime
from stacc.executive import AutonomousMissionExecutive, MissionPhase
from stacc.robot import QuadrupedRobot


def test_estimation_esekf_and_momentum_observer():
    """Verify 15-state ES-EKF and Momentum Disturbance Observer."""
    # 1. ES-EKF
    ekf = ErrorStateEKF()
    for _ in range(10):
        ekf.predict(a_m=np.array([0, 0, 9.81]), w_m=np.array([0, 0, 0]), dt=0.01)
    
    ekf.update_position(p_meas=np.array([0.0, 0.0, 0.0]))
    p, v, R = ekf.get_pose()
    assert np.allclose(p, [0, 0, 0], atol=1e-2)

    # 2. Momentum Disturbance Observer
    obs = MomentumObserver(num_joints=2, gain_diag=30.0)
    q = np.array([0.5, 0.2])
    q_dot = np.array([0.1, 0.0])
    M_mat = np.eye(2) * 1.5
    C_qdot = np.zeros(2)
    G_vec = np.array([0.0, 9.81])
    tau = np.array([0.0, 9.81])

    obs.update(q, q_dot, tau, M_mat, C_qdot, G_vec, dt=0.001)
    r_ext = obs.update(q, q_dot, tau + np.array([12.0, 0.0]), M_mat, C_qdot, G_vec, dt=0.001)
    assert not obs.is_collision(threshold_nm=50.0)


def test_whole_body_controller():
    """Verify Hierarchical WBC sequential null-space projection."""
    wbc = HierarchicalWBC(num_dof=4)

    J1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    task1 = TaskObjective(name="COM", J=J1, x_ddot_des=np.array([0.5, -0.2]))

    J2 = np.array([[0, 0, 1, 1]])
    task2 = TaskObjective(name="EE", J=J2, x_ddot_des=np.array([1.0]))

    M_mat = np.eye(4) * 2.0
    C_qdot = np.zeros(4)
    G_vec = np.zeros(4)

    q_ddot, tau = wbc.solve([task1, task2], M_mat, C_qdot, G_vec)
    assert len(q_ddot) == 4
    assert len(tau) == 4
    assert np.allclose(J1 @ q_ddot, [0.5, -0.2], atol=1e-3)


def test_model_transpilers():
    """Verify ModelTranspiler validation and USD/SDF generators."""
    is_phys, reason = ModelTranspiler.validate_inertia_tensor(mass=2.5, ixx=0.05, iyy=0.05, izz=0.08)
    assert is_phys

    is_phys_bad, _ = ModelTranspiler.validate_inertia_tensor(mass=2.5, ixx=0.01, iyy=0.01, izz=0.10)
    assert not is_phys_bad

    # Transpile QuadrupedRobot to OpenUSD spec and SDF XML
    quad = QuadrupedRobot(name="transpile_quad")
    usd_spec = ModelTranspiler.robot_to_usd_spec(quad)
    assert usd_spec["stage_type"] == "UsdStage"
    assert len(usd_spec["prims"]) > 0

    sdf_xml = ModelTranspiler.robot_to_sdf(quad)
    assert "<sdf" in sdf_xml and "transpile_quad" in sdf_xml


def test_bridges_ros2_and_zenoh():
    """Verify ROS 2 message serialization and Zenoh publisher."""
    js = ROS2Bridge.pack_joint_state(joint_names=["j1", "j2"], positions=[0.5, -0.2])
    assert js["name"] == ["j1", "j2"]
    assert js["position"] == [0.5, -0.2]

    twist = ROS2Bridge.pack_twist(linear=(1.0, 0.0, 0.0), angular=(0.0, 0.0, 0.5))
    assert twist["linear"]["x"] == 1.0

    zenoh = ZenohTelemetryStreamer(robot_id="STACC_BOT_01")
    assert zenoh.publish_telemetry("imu", {"acc_x": 0.02, "gyro_z": 0.01})


def test_planning_chomp_and_octomap():
    """Verify CHOMP trajectory optimizer and 3D Voxel OctoMap."""
    chomp = CHOMPPlanner(num_waypoints=15, num_dof=3)
    traj = chomp.optimize(q_start=[0, 0, 0], q_goal=[1, 1, 1], iterations=10)
    assert traj.shape == (15, 3)
    assert np.allclose(traj[0], [0, 0, 0])
    assert np.allclose(traj[-1], [1, 1, 1])

    octomap = VoxelOctoMap(min_bound=(-1, -1, 0), max_bound=(1, 1, 1), resolution=0.1)
    pts = np.array([[0.5, 0.5, 0.5], [0.0, 0.0, 0.2]])
    octomap.insert_point_cloud(pts, is_occupied=True)
    assert octomap.is_occupied(np.array([0.5, 0.5, 0.5]))
    assert not octomap.is_occupied(np.array([0.8, -0.8, 0.5]))

    dual = DualArmCoordinator(relative_grasp_offset=np.array([0.0, 0.40, 0.0]))
    slave_target = dual.compute_slave_target(master_pose=np.array([1.0, 0.0, 0.5]))
    assert np.allclose(slave_target, [1.0, 0.40, 0.5])


def test_vla_grounder_and_learning():
    """Verify VLA natural language task grounder and Visuomotor Policy Runtime."""
    vla = VLATaskGrounder()
    plan = vla.decompose_instruction("Pick up the titanium bracket from bin A and place it on table B")
    assert len(plan) >= 2
    actions = [step["action"] for step in plan]
    assert "DETECT_3D" in actions
    assert "NAVIGATE_TO_POSE" in actions

    policy = VisuomotorPolicyRuntime(action_dim=7, chunk_horizon=8)
    dummy_img = np.zeros((224, 224, 3), dtype=np.uint8)
    action = policy.predict_action(dummy_img, q_current=[0, 0, 0, 0, 0, 0])
    assert len(action) == 7


def test_mission_executive_fsm():
    """Verify Autonomous Mission Executive State Machine progression."""
    exec_fsm = AutonomousMissionExecutive(mission_name="test_mission")
    assert exec_fsm.phase == MissionPhase.IDLE
    exec_fsm.start_mission()
    assert exec_fsm.phase == MissionPhase.INITIALIZE

    for _ in range(8):
        exec_fsm.step()

    assert exec_fsm.phase == MissionPhase.COMPLETE
    assert len(exec_fsm.log) >= 8
