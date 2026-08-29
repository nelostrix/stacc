"""stacc — The Universal End-to-End Robotics Operating System & Development Platform.

Build, control, simulate, and deploy any robot from micro-rovers to bipedal humanoids and 7-DOF arms.

Core Systems:
- stacc.kinematics: Lie Groups (SE3/SO3), Product of Exponentials (PoE), Forward/Inverse Kinematics, Jacobians
- stacc.dynamics: Recursive Newton-Euler (RNEA), Composite Rigid Body (CRBA), Articulated Body (ABA), Manipulability
- stacc.control: Linear MPC, Cartesian Impedance, Operational Space (OSC), LQR, Control Barrier (CBF), Hierarchical WBC
- stacc.estimation: 15-State Error-State EKF (IMU/GPS/LiDAR fusion), Generalized Momentum Disturbance Observers
- stacc.robot: SerialManipulators, QuadrupedWalkers, HumanoidBipeds, MultirotorDrones, MecanumDrives, AckermannCars, DeltaRobots
- stacc.locomotion: Convex MPC Ground Reaction Force QP, Linear Inverted Pendulum (LIPM), ZMP, Capture Point Recovery
- stacc.manipulation: Antipodal Grasp Synthesis, Ferrari-Canny Force Closure, Friction Cones
- stacc.planning: CHOMP Trajectory Optimization, 3D Voxel OctoMaps, Dual-Arm Coordination
- stacc.converters: Multi-Format Model Transpilers (URDF ↔ MJCF ↔ SDF ↔ OpenUSD for Omniverse)
- stacc.bridge: Native ROS 2 Topic Bridge (CDR), Zenoh Low-Latency Fleet Telemetry Streaming
- stacc.hal: CAN Bus / SocketCAN, 1000Hz POSIX Shared Memory RingBuffer, EtherCAT CiA 402, GPIO, I2C, SPI
- stacc.ai: Vision-Language-Action (VLA) Grounding, Visuomotor Policy Runtime, SLAM (EKF/FastSLAM/ICP), Vision, Behavior Trees
- stacc.executive: Autonomous Mission Executives & Hierarchical State Machines
- stacc.sim: Multi-Sensor Physics Engine, MuJoCo Real-Time Backend, Gym RL Environments
"""

__version__ = "0.3.0"

import time
from stacc.sim.engine import simulate

wait = time.sleep

__all__ = [
    "simulate",
    "wait",
    "__version__",
]
