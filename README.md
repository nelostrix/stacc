<p align="center">
  <img src="docs/assets/stacc-logo.png" alt="stacc" width="120"/>
</p>

<h1 align="center">stacc</h1>

<p align="center">
  <strong>The Universal Robotics Development Platform</strong><br>
  Build robots from line followers to Mars rovers. One CLI to init, simulate, build, and deploy.
</p>

<p align="center">
  <a href="#installation">Install</a> •
  <a href="#quick-start">Quick Start</a> •
  <a href="#templates">Templates</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#gui">GUI</a> •
  <a href="#contributing">Contributing</a>
</p>

---

## Why stacc?

Robotics development is fragmented. You need ROS for communication, Gazebo or Isaac Sim for simulation, MoveIt for motion planning, custom scripts for hardware — all stitched together with duct tape. **stacc** unifies all of this into one platform.

```
stacc init my_robot --template=arm_6dof    # Scaffold a project
stacc install slam-toolkit                  # Add packages
stacc simulate --duration=30                # Run in simulator
stacc build --target=raspberry_pi           # Cross-compile
stacc deploy 192.168.1.100                  # Deploy to hardware
```

**stacc is to robotics what CUDA is to GPU computing** — a platform that makes the entire pipeline 10x easier.

> **Implementation scope:** STACC is an alpha SDK. Simulation and the Python
> control runtime are usable on supported configurations, but the product is
> not yet a universal hardware deployment platform or hard real-time system.
> Board catalog entries and CLI targets do not guarantee verified upload or
> physical-device support. See [`stacc-sdk/docs/status.md`](stacc-sdk/docs/status.md)
> and [`stacc-sdk/docs/runtime-safety.md`](stacc-sdk/docs/runtime-safety.md).

### How stacc compares

| Feature | ROS 2 | Isaac Sim | stacc |
|---------|-------|-----------|-------|
| Learning curve | Weeks | Days | **Minutes** |
| Hardware requirement | Linux | NVIDIA GPU | **Any OS, any hardware** |
| Install time | Hours | Hours | **`pip install stacc`** |
| Simulation included | No (needs Gazebo) | Yes | **Yes (MuJoCo built-in)** |
| Hardware deployment | Manual | No | **`stacc deploy`** |
| Line follower to Mars rover | Painful | Overkill | **Built-in templates** |

## Installation

```bash
pip install stacc
```

Or install from source:

```bash
git clone https://github.com/YOUR_ORG/stacc.git
cd stacc
pip install -e .
```

Verify installation:

```bash
stacc --version
stacc doctor
```

## Quick Start

### 1. Create a project

```bash
stacc init my_robot --template=quadruped
cd my_robot
```

### 2. Write your robot code

```python
"""Quadruped walker — built with stacc"""
import stacc
from stacc.robot import LeggedRobot
from stacc.gait import TrotGait
from stacc.sensors import IMU
from stacc.control import BalanceController

robot = LeggedRobot(
    name="quadruped",
    num_legs=4,
    joints_per_leg=3,
    body_dims=[0.3, 0.15, 0.05],
    leg_lengths=[0.08, 0.12, 0.12]
)

imu = IMU()
robot.attach_sensor(imu, position="body_center")

balance = BalanceController(robot, imu)
trot = TrotGait(step_height=0.04, frequency=2.0)

@robot.on_update
def walk(dt):
    foot_targets = trot.compute(robot.time)
    corrected = balance.stabilize(foot_targets, dt)
    for leg_id in range(4):
        angles = robot.leg_ik(leg_id, corrected[leg_id])
        robot.set_leg_joints(leg_id, angles)

if __name__ == "__main__":
    stacc.simulate(robot, duration=30, render=True)
```

### 3. Simulate

```bash
stacc simulate --duration=30 --physics=mujoco
```

### 4. Build & deploy

```bash
stacc build --target=raspberry_pi
stacc deploy 192.168.1.100
```

## Templates

| Template | Command | Description |
|----------|---------|-------------|
| Line follower | `stacc init bot --template=line_follower` | 2-wheel differential drive with IR sensors |
| 6-DOF arm | `stacc init bot --template=arm_6dof` | Robotic arm with gripper, full IK |
| Quadruped | `stacc init bot --template=quadruped` | 4-legged walking robot, 12 servos |
| Mars rover | `stacc init bot --template=rover` | 6-wheel rocker-bogie with SLAM |
| Drone | `stacc init bot --template=drone` | Quadcopter with GPS and camera |
| Blank | `stacc init bot --template=blank` | Empty project |

## Architecture

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

## GUI

stacc includes a full GUI platform for visual robot development:

```bash
stacc gui
```

Features:
- **3D viewport** — real-time robot visualization with MuJoCo physics
- **Code editor** — write Python with stacc autocomplete
- **Robot builder** — drag-and-drop component assembly
- **Simulation controls** — play, pause, step, speed, timeline
- **Package manager** — browse and install from the registry
- **Live telemetry** — real-time plots of sensors and joints
- **Node editor** — visual programming for control logic

## Package Registry

```bash
stacc search navigation        # Search packages
stacc install slam-toolkit      # Add a package to your project manifest
stacc registry list             # List all available packages
```

> **Status**: `stacc install`/`registry` currently resolve against a built-in local
> package list (below) and record the choice in your project manifest — there's no
> hosted registry, dependency resolution, or real download yet. Publishing your own
> package (`stacc registry publish`) isn't available in this version.

### Available packages

| Package | Category | Description |
|---------|----------|-------------|
| `slam-toolkit` | navigation | EKF-SLAM, FastSLAM, ORB-SLAM |
| `path-planning` | navigation | RRT*, A*, D*, potential fields |
| `perception-core` | perception | Object detection, depth, point clouds |
| `rl-toolkit` | ai | PPO, SAC, TD3 with sim-to-real |
| `motor-drivers` | hardware | PWM, stepper, BLDC, CAN, EtherCAT |
| `sensor-suite` | hardware | IMU, LiDAR, cameras, encoders |
| `ros2-bridge` | interop | ROS 2 pub/sub, tf2, nav2 |
| `gait-library` | locomotion | Trot, crawl, gallop, CPG |
| `flight-control` | aerial | Attitude, position, PX4 bridge |
| `gripper-toolkit` | manipulation | Parallel, vacuum, soft, dexterous |
| `sim-bridge` | simulation | MuJoCo, Bullet, DART adapters |
| `eigen-bindings` | core | Linear algebra (Eigen C++) |

## Hardware Support

stacc deploys to any platform:

| Platform | Target flag | Status |
|----------|-------------|--------|
| Raspberry Pi | `--target=rpi` | ✅ Supported |
| NVIDIA Jetson | `--target=jetson` | ✅ Supported |
| Arduino | `--target=arduino` | ✅ Supported |
| ESP32 | `--target=esp32` | ✅ Supported |
| STM32 | `--target=stm32` | 🚧 In progress |
| x86_64 Desktop | `--target=x86_64` | ✅ Supported |

## Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

```bash
git clone https://github.com/YOUR_ORG/stacc.git
cd stacc
pip install -e ".[dev]"
pytest
```

## Roadmap

- [x] CLI framework with all commands
- [x] GUI platform prototype
- [x] Core robot model (joints, links, URDF)
- [x] Kinematics engine (FK, IK, dynamics)
- [x] MuJoCo simulation backend
- [x] Hardware abstraction layer
- [x] Sensor simulation
- [x] AI/perception toolkit
- [x] RSS — declarative robot description language (`stacc rss check|build|run`)
- [ ] Package registry server (`stacc install`/`registry` currently resolve against a
      built-in local package list — there's no hosted registry or real download yet)
- [ ] Cloud simulation
- [ ] Community marketplace

## License

MIT License — see [LICENSE](LICENSE) for details.

---

<p align="center">
  Built with 🤖 by the stacc team
</p>
