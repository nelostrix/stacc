# stacc Architecture

## Overview

stacc is The Universal Robotics Development Platform — a unified platform to init, simulate, build, and deploy robots from line followers to Mars rovers.

## System Architecture

```
stacc/
├── stacc/              # Core Python package
│   ├── cli.py          # CLI entry point
│   ├── core/           # Robot model — joints, links, URDF
│   ├── kinematics/     # FK, IK, dynamics, trajectories
│   ├── hal/            # Hardware abstraction — GPIO, motors, buses
│   ├── sensors/        # Sensor drivers — IMU, LiDAR, camera
│   ├── sim/            # Simulation — MuJoCo, Bullet, rendering
│   ├── ai/             # SLAM, path planning, RL, perception
│   ├── robot/          # Robot type implementations
│   ├── control/        # PID, impedance, balance controllers
│   ├── gait/           # Walking gait generators
│   └── gripper/        # End-effector controllers
├── gui/                # GUI platform (React + FastAPI)
│   ├── src/            # React frontend
│   └── server/         # FastAPI backend + WebSocket
├── tests/              # Unit & integration tests
├── docs/               # Documentation
├── examples/           # Example projects
└── templates/          # Project templates
```

## Core Components

### CLI Layer
- `stacc init` - Scaffold a project with templates
- `stacc install` - Add packages with dependency resolution
- `stacc simulate` - Run in simulator (MuJoCo built-in)
- `stacc build` - Cross-compile for target hardware
- `stacc deploy` - Deploy to hardware

### Robot Model (core/)
- Joint types: revolute, prismatic, continuous, fixed
- Link composition with mass, inertia, collision
- URDF parsing and generation
- SDF support

### Kinematics Engine (kinematics/)
- Forward kinematics (FK)
- Inverse kinematics (IK) - analytical and numerical
- Dynamics - forward and inverse
- Trajectory planning

### Hardware Abstraction (hal/)
- GPIO interfaces
- Motor controllers (PWM, stepper, BLDC)
- Communication buses (CAN, I2C, SPI, EtherCAT)
- Board targets: RPi, Jetson, Arduino, ESP32, STM32

### Simulation (sim/)
- MuJoCo backend (primary)
- Bullet physics
- DART support
- Rendering with OpenGL

### Control (control/)
- `PIDController` + `CascadedPID` — with anti-windup, deadband, output limits
- `Feedforward` — velocity/acceleration/static feedforward
- `GravityCompensation` — torque computation for serial manipulators
- `FrictionModel` — Coulomb + viscous friction
- `LowPassFilter`, `MovingAverageFilter`, `ComplementaryFilter`, `KalmanFilter1D`, `NotchFilter`
- `RateLimiter`, `AccelerationLimiter`, `SCurveProfile` — smooth setpoint tracking with jerk limits
- `ControllerPipeline` — composable named stages with enable/disable and conditional skip

### AI/Perception (ai/)
- SLAM implementations (EKF-SLAM, FastSLAM, ICP, Occupancy Grid)
- Path planning (RRT*, A*, Dijkstra, potential fields)
- Behavior trees — Sequence, Selector, Parallel, Repeat, Timeout, Retry
- Natural language command parsing — 30+ regex patterns for robot control
- Speech interface — speech-to-text pipeline
- Object detection (OpenCV/ONNX/Torch), feature extraction, optical flow
- Visual odometry, stereo depth estimation, ArUco marker detection

## GUI Platform

Built with React + FastAPI:
- 3D viewport with real-time robot visualization
- Code editor with stacc autocomplete
- Robot builder - drag-and-drop component assembly
- Simulation controls - play, pause, step, speed, timeline
- Package manager - browse and install from registry
- Live telemetry - real-time sensor/joint plots
- Node editor - visual programming for control logic

## Package Registry

12 built-in packages:
| Package | Category |
|---------|----------|
| slam-toolkit | navigation |
| path-planning | navigation |
| perception-core | perception |
| rl-toolkit | ai |
| motor-drivers | hardware |
| sensor-suite | hardware |
| ros2-bridge | interop |
| gait-library | locomotion |
| flight-control | aerial |
| gripper-toolkit | manipulation |
| sim-bridge | simulation |
| eigen-bindings | core |

## Target Hardware Support

| Platform | Target Flag | Status |
|----------|-------------|--------|
| Raspberry Pi | `--target=rpi` | ✅ Supported |
| NVIDIA Jetson | `--target=jetson` | ✅ Supported |
| Arduino | `--target=arduino` | ✅ Supported |
| ESP32 | `--target=esp32` | ✅ Supported |
| STM32 | `--target=stm32` | 🚧 In progress |
| x86_64 Desktop | `--target=x86_64` | ✅ Supported |

## Robot Templates

| Template | Command |
|----------|---------|
| Line follower | `stacc init bot --template=line_follower` |
| 6-DOF arm | `stacc init bot --template=arm_6dof` |
| Quadruped | `stacc init bot --template=quadruped` |
| Mars rover | `stacc init bot --template=rover` |
| Drone | `stacc init bot --template=drone` |
| Blank | `stacc init bot --template=blank` |

## Data Flow

```
User Code → CLI/API → Robot Model → Kinematics → Control → HAL/Sim → Hardware/Physics
```

## Technology Stack

- **Language**: Python (primary), C++ (performance)
- **Simulation**: MuJoCo, Bullet, DART
- **GUI**: React, FastAPI, WebSocket
- **Build**: Cross-compilation for embedded targets
- **Package Management**: Custom registry with dependency resolution