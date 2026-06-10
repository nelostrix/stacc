# stacc Status Report

## Current Version

**stacc v0.1.0** - Released 2026-03-29

## Project Overview

stacc is The Universal Robotics Development Platform — a unified platform to init, simulate, build, and deploy robots from line followers to Mars rovers.

## What's Completed

### CLI Framework
All core commands implemented:
- `stacc init` - Scaffold a project
- `stacc install` - Add packages with dependency resolution
- `stacc remove` - Remove packages
- `stacc search` - Search registry
- `stacc simulate` - Run in simulator
- `stacc build` - Cross-compile for target hardware
- `stacc deploy` - Deploy to hardware
- `stacc doctor` - System health check
- `stacc run` - Execute robot code
- `stacc registry` - Manage package registry

### Robot Templates (6)
- blank - Empty project
- line_follower - 2-wheel differential drive with IR sensors
- arm_6dof - Robotic arm with gripper, full IK
- quadruped - 4-legged walking robot, 12 servos
- rover - 6-wheel rocker-bogie with SLAM
- drone - Quadcopter with GPS and camera

### Package Registry
12 built-in packages available:
- slam-toolkit, path-planning (navigation)
- perception-core, rl-toolkit (AI/perception)
- motor-drivers, sensor-suite (hardware)
- ros2-bridge (interop)
- gait-library (locomotion)
- flight-control (aerial)
- gripper-toolkit (manipulation)
- sim-bridge (simulation)
- eigen-bindings (core)

### Build Targets
- Raspberry Pi (rpi) ✅
- NVIDIA Jetson (jetson) ✅
- Arduino (arduino) ✅
- ESP32 (esp32) ✅
- STM32 (stm32) 🚧 In Progress
- x86_64 Desktop ✅

### GUI Platform (Prototype)
- 3D viewport with MuJoCo physics
- Code editor with stacc autocomplete
- Robot builder - drag-and-drop
- Simulation controls
- Package manager
- Live telemetry
- Node editor

### Control Module
- `PIDController` + `CascadedPID` with anti-windup, deadband, output limits
- `Feedforward` with velocity/acceleration/static terms
- `GravityCompensation` for serial manipulators
- `FrictionModel` — Coulomb + viscous friction
- `LowPassFilter`, `MovingAverageFilter`, `ComplementaryFilter`
- `KalmanFilter1D` for optimal recursive estimation
- `NotchFilter` for frequency-specific noise removal
- `RateLimiter`, `AccelerationLimiter` for smooth setpoint tracking
- `SCurveProfile` — jerk-limited trajectory generation
- `ControllerPipeline` with named `ControllerStage`s, enable/disable, conditional skip

### HAL Module
- `Board` — auto-detect platform (RPi, Jetson, desktop) with `Board.detect()`
- `BoardDefinition` — 18 board specs across 9 platforms with full pin mappings
- `FirmwareUploader` — firmware upload via avrdude, esptool, stlink, bossac, UF2, SSH
- `GPIO` — universal GPIO with RPi.GPIO / Jetson.GPIO / gpiod / simulation backends
- `I2CBus` — universal I2C via smbus2 with device scanning
- `SPIBus` — universal SPI via spidev with full-duplex transfer
- `SerialPort` — universal UART/serial via pyserial
- `MotorDriver` — DC motor control via H-bridge (L298N) with PWM
- `ServoDriver` — standard hobby servo control with angle-to-pulse mapping
- `Deployer` — SSH-based project deployment to remote robots

### Gripper Module
- `Gripper` base class with open/close/stop/reset state machine and grasp detection
- `ParallelGripper` — parallel jaw with configurable width, force, speed
- `VacuumGripper` — suction cups with pressure, seal detection, holding force calc
- `SoftGripper` — compliant fingers with configurable curl and compliance
- `MagneticGripper` — electromagnetic with instant engage/release
- `DexterousHand` — multi-finger hand with power/pinch grasp and finger-level control

### Gait Module
- `GaitGenerator` base class with configurable swing_ratio, direction turning, reset
- `TrotGait`, `WalkGait`, `CrawlGait`, `GallopGait`, `BoundGait` — quadruped gaits
- `BipedWalkGait` — bipedal walking with hip sway
- `TripodGait` — hexapod alternating tripods
- `CPGGenerator` — oscillator-based adaptive gait with 5 coupling presets
- `FlightController` — multirotor attitude control and motor mixing

### Robot Module
- `DifferentialDrive` — 2-wheel mobile robot with speed, velocity, and heading control
- `SerialManipulator` — N-DOF arm with FK, IK, Jacobian via DH parameters

### AI Module
- `EKFSLAM`, `ParticleFilterSLAM` — full implementations with motion/update/resample
- `ICPMatcher` — scan-to-scan matching with SVD
- `OccupancyGridMapper` — log-odds grid with Bresenham ray tracing
- `SLAMNavigator` — high-level interface connecting sensors to SLAM
- `AStarPlanner`, `DijkstraPlanner` — grid-based optimal search
- `RRTStarPlanner` — sampling-based planning with rewiring
- `PotentialFieldPlanner` — reactive obstacle avoidance
- `Sequence`, `Selector`, `Parallel` composites + full decorator set
- `CommandParser` — 30+ regex patterns for natural language robot control
- `ActionMapper` + `SpeechInterface` — text/speech to robot actions
- `ObjectDetector` (OpenCV/ONNX/Torch), `VisualOdometry`, `StereoDepthEstimator`

## Coming Soon

- URDF parsing / serialization
- MuJoCo simulation backend
- Hardware abstraction layer
- Sensor simulation
- AI/perception toolkit

## Roadmap

- [x] CLI framework with all commands
- [x] GUI platform prototype
- [x] Control module — PID, filters, feedforward, limiters, pipelines
- [x] AI module — SLAM, path planning, behavior trees, NLP, vision
- [x] Gait module — generators for all legged gaits + CPG + flight controller
- [x] Gripper module — parallel, vacuum, soft, magnetic, dexterous end effectors
- [x] HAL module — board detection, GPIO, I2C, SPI, serial, motor/servo drivers, deploy
- [x] Core robot model — DifferentialDrive, SerialManipulator, Robot base, Joint, Link
- [x] Kinematics engine (FK, IK, trajectory generation)
- [ ] MuJoCo simulation backend
- [ ] Hardware abstraction layer
- [ ] Sensor simulation
- [ ] AI/perception toolkit
- [ ] Package registry server
- [ ] Cloud simulation
- [ ] Community marketplace

## License

MIT License