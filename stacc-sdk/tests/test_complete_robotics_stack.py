"""Comprehensive validation suite for STACC Complete Robotics Stack.

Validates:
- Lie Group SE(3)/SO(3) Spatial Vector Algebra & PoE Kinematics
- Rigid Body Dynamics (RNEA, CRBA, ABA, Manipulability)
- Modern Control (Linear MPC, Cartesian Impedance, OSC, LQR, CBF)
- Comprehensive Robot Topologies (Quadruped, Humanoid, Drone, Mecanum, Ackermann, Delta)
- Industrial Fieldbuses & 1000Hz SHM RingBuffer
- Legged Locomotion (Convex MPC, LIPM, ZMP)
- Grasp Synthesis & Force Closure
"""

import math
import numpy as np
import pytest

# 1. Lie Group & Spatial Kinematics
from stacc.kinematics.spatial import (
    skew, unskew,
    exp_so3, log_so3,
    exp_se3, log_se3,
    adjoint_transform,
    product_of_exponentials_spatial,
    product_of_exponentials_body,
    spatial_jacobian,
    body_jacobian,
)

# 2. Dynamics
from stacc.dynamics import (
    SpatialInertia,
    rnea_inverse_dynamics,
    compute_gravity_vector,
    compute_coriolis_vector,
    crba_mass_matrix,
    aba_forward_dynamics,
    yoshikawa_manipulability,
    manipulability_ellipsoid,
    singularity_robust_inverse,
)

# 3. Control
from stacc.control import (
    LinearMPC,
    CartesianImpedanceController,
    OperationalSpaceController,
    lqr, LQRController,
    ControlBarrierFilter,
)

# 4. Robot Topologies
from stacc.robot import (
    QuadrupedRobot,
    HumanoidRobot,
    MultirotorDrone,
    MecanumDrive,
    AckermannVehicle,
    DeltaRobot,
)

# 5. HAL & Fieldbuses
from stacc.hal import (
    CANBus, CANFrame, ODriveCAN, UnitreeMotorCAN,
    SharedMemoryRingBuffer,
    EtherCATSlaveCiA402, CiA402State,
)

# 6. Locomotion & Manipulation
from stacc.locomotion import QuadrupedConvexMPC, LinearInvertedPendulum
from stacc.manipulation import GraspCandidate


def test_lie_algebra_so3():
    """Verify so(3) skew, exp_so3, and log_so3 roundtrip."""
    axis = np.array([0.0, 1.0, 0.0])
    angle = math.pi / 3.0
    R = exp_so3(axis, angle)
    assert R.shape == (3, 3)
    assert np.allclose(R @ R.T, np.eye(3))

    recovered_axis, recovered_angle = log_so3(R)
    assert math.isclose(recovered_angle, angle, abs_tol=1e-5)
    assert np.allclose(recovered_axis, axis, atol=1e-4)


def test_lie_algebra_se3_poe():
    """Verify SE(3) twists, exponential mapping, and Product of Exponentials (PoE)."""
    # 2-DOF planar revolute arm
    S1 = np.array([0, 0, 1, 0, 0, 0], dtype=np.float64)
    S2 = np.array([0, 0, 1, 0, -1.0, 0], dtype=np.float64)
    M = np.array([
        [1, 0, 0, 2.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1.0]
    ], dtype=np.float64)

    # q = [0, 0] -> end effector at (2, 0, 0)
    T0 = product_of_exponentials_spatial([S1, S2], [0.0, 0.0], M)
    assert np.allclose(T0[:3, 3], [2.0, 0.0, 0.0])

    # q = [pi/2, 0] -> arm rotated 90 degrees -> end effector at (0, 2, 0)
    T90 = product_of_exponentials_spatial([S1, S2], [math.pi / 2.0, 0.0], M)
    assert np.allclose(T90[:3, 3], [0.0, 2.0, 0.0], atol=1e-5)


def test_rigid_body_dynamics_rnea_crba_aba():
    """Verify consistency between RNEA, CRBA, and ABA."""
    # 2-link pendulum model
    B1 = np.array([0, 0, 1, 0, 0, 0], dtype=np.float64)
    B2 = np.array([0, 0, 1, 0, 0, 0], dtype=np.float64)
    M1 = np.eye(4)
    M2 = np.array([[1, 0, 0, 1.0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)

    inertia1 = SpatialInertia(mass=1.0, com=[0.5, 0, 0], inertia_tensor=np.diag([0.01, 0.08, 0.08]))
    inertia2 = SpatialInertia(mass=1.0, com=[0.5, 0, 0], inertia_tensor=np.diag([0.01, 0.08, 0.08]))

    q = np.array([0.5, -0.3])
    q_dot = np.array([0.1, -0.2])
    q_ddot = np.array([1.5, -0.5])

    # Compute M(q) via CRBA
    M_mat = crba_mass_matrix(q, [B1, B2], [M1, M2], [inertia1, inertia2])
    assert M_mat.shape == (2, 2)
    # Check symmetric positive-definite
    assert np.allclose(M_mat, M_mat.T)
    eigenvalues = np.linalg.eigvals(M_mat)
    assert np.all(eigenvalues > 0)

    # Compute torques via RNEA
    tau_rnea = rnea_inverse_dynamics(q, q_dot, q_ddot, [B1, B2], [M1, M2], [inertia1, inertia2])
    assert len(tau_rnea) == 2

    # Forward dynamics via ABA using the computed torques should yield back original q_ddot
    q_ddot_aba = aba_forward_dynamics(q, q_dot, tau_rnea, [B1, B2], [M1, M2], [inertia1, inertia2])
    assert np.allclose(q_ddot_aba, q_ddot, atol=1e-4)


def test_modern_controllers():
    """Verify Linear MPC, Cartesian Impedance, and CBF Safety Filters."""
    # 1. Linear MPC
    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    B = np.array([[0.005], [0.1]])
    mpc = LinearMPC(A, B, Q=[10.0, 1.0], R=0.1, N=8, u_min=-10.0, u_max=10.0)
    u_opt, X_pred = mpc.solve(x0=[1.0, 0.0])
    assert len(u_opt) == 1
    assert u_opt[0] < 0  # Should apply negative control to reduce positive position error

    # 2. Cartesian Impedance Controller
    imp = CartesianImpedanceController(stiffness=500.0, damping=50.0)
    J = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
    tau = imp.compute_joint_torques(
        J=J,
        target_pos=np.array([1.0, 0.5, 0.0]),
        current_pos=np.array([0.9, 0.5, 0.0])
    )
    assert len(tau) == 2
    assert tau[0] > 0  # Positive restoring torque along X

    # 3. Control Barrier Function (CBF)
    cbf = ControlBarrierFilter(gamma=2.0, u_min=-5.0, u_max=5.0)
    # Barrier: h(x) = x_max - x >= 0 (wall at x_max), grad_h = -1.0
    # Robot is at x = 0.98 (h = 0.02), moving forward with u_nom = +2.0
    u_safe = cbf.filter_control(
        u_nominal=np.array([2.0]),
        h_val=0.02,
        grad_h=np.array([-1.0]),
        f_x=np.array([0.0]),
        g_x=np.array([[1.0]])
    )
    assert u_safe[0] < 2.0  # CBF enforces braking / reduces control action to 0.04
    assert math.isclose(u_safe[0], 0.04, rel_tol=1e-3)


def test_robot_topologies():
    """Verify Quadruped, Humanoid, Drone, Mecanum, Ackermann, and Delta kinematics."""
    # Quadruped
    quad = QuadrupedRobot(name="test_quad")
    p_foot = quad.foot_forward_kinematics("FL", [0.0, 0.3, -0.6])
    assert p_foot[2] < 0  # Foot is below the hip

    # Humanoid
    humanoid = HumanoidRobot(name="test_humanoid")
    assert len(humanoid.joints) >= 20

    # Drone
    drone = MultirotorDrone(name="test_drone", mass=1.2)
    thrusts = drone.allocate_thrusts(total_thrust=1.2 * 9.81, torques=[0, 0, 0])
    assert len(thrusts) == 4
    assert math.isclose(np.sum(thrusts), 1.2 * 9.81, rel_tol=1e-4)

    # Mecanum Drive
    mecanum = MecanumDrive(name="test_mecanum", wheel_radius=0.075)
    wheel_speeds = mecanum.inverse_kinematics(vx=1.0, vy=0.0, omega_z=0.0)
    assert len(wheel_speeds) == 4

    # Ackermann Car
    car = AckermannVehicle(name="test_car")
    delta_in, delta_out = car.steering_angles(delta_center=0.2)
    assert delta_in > delta_out  # Inner wheel turns sharper than outer wheel

    # Delta Parallel Robot
    delta = DeltaRobot(name="test_delta")
    t1, t2, t3 = delta.inverse_kinematics(0.0, 0.0, -0.45)
    assert math.isclose(t1, t2, rel_tol=1e-3) and math.isclose(t2, t3, rel_tol=1e-3)


def test_hal_fieldbuses_and_shm():
    """Verify CAN, 1000Hz SHM RingBuffer, and EtherCAT."""
    # Virtual CAN Bus
    bus = CANBus(channel="vcan0", is_virtual=True)
    odrive = ODriveCAN(node_id=3, bus=bus)
    odrive.set_position(pos_turns=10.5)
    frame = bus.recv()
    assert frame is not None
    assert frame.arbitration_id == (3 << 5) | ODriveCAN.CMD_SET_INPUT_POS

    # 1000Hz POSIX Shared Memory RingBuffer
    shm = SharedMemoryRingBuffer(name="test_stacc_shm", capacity=10, num_floats_per_slot=8)
    shm.write([1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8])
    data, ts, idx = shm.read_latest()
    assert np.allclose(data, [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8])
    shm.close()

    # EtherCAT CiA 402 Drive
    drive = EtherCATSlaveCiA402(slave_id=1)
    state = drive.parse_status_word(0x0027)
    assert state == CiA402State.OPERATION_ENABLED


def test_locomotion_and_manipulation():
    """Verify Quadruped Convex MPC, LIPM, and Grasp Synthesis."""
    # Quadruped Convex MPC
    mpc = QuadrupedConvexMPC(mass=12.0)
    forces = mpc.solve(
        body_pos=np.array([0, 0, 0.28]),
        body_vel=np.array([0, 0, 0]),
        body_rpy=np.array([0, 0, 0]),
        body_omega=np.array([0, 0, 0]),
        foot_positions=[np.array([0.2, 0.1, 0]), np.array([0.2, -0.1, 0]), np.array([-0.2, 0.1, 0]), np.array([-0.2, -0.1, 0])],
        contact_states=[True, True, True, True],
        target_vel=np.array([0.5, 0, 0])
    )
    assert forces.shape == (4, 3)
    assert np.sum(forces[:, 2]) > 0  # Total upward reaction force

    # LIPM & Capture Point
    lipm = LinearInvertedPendulum(com_height=0.80)
    px, py = lipm.compute_zmp(com_pos=(0.05, 0.02), com_acc=(0.1, 0.0))
    assert px < 0.05

    # Grasp Candidate Force Closure
    grasp = GraspCandidate(
        p1=np.array([-0.03, 0, 0]),
        p2=np.array([0.03, 0, 0]),
        n1=np.array([1, 0, 0]),
        n2=np.array([-1, 0, 0]),
        mu=0.6
    )
    assert grasp.is_antipodal()
    eps = grasp.ferrari_canny_metric()
    assert eps > 0.0
