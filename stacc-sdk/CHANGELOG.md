# Changelog

All notable changes to stacc will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [0.1.0] - 2026-03-29

### Added
- CLI framework with all commands (`init`, `install`, `remove`, `search`, `simulate`, `build`, `deploy`, `doctor`, `run`, `registry`)
- 6 robot templates: blank, line_follower, arm_6dof, quadruped, rover, drone
- 12 built-in packages in the registry
- Project scaffolding with stacc.json manifest
- Dependency resolution for package installation
- Cross-compilation targets: RPi, Jetson, Arduino, ESP32, STM32, x86_64
- GUI platform prototype with 3D viewport, code editor, terminal, robot builder, package manager
- System health check (`stacc doctor`)

### Coming Soon
- Core robot model (joints, links, URDF parsing)
- Kinematics engine (FK, IK, dynamics)
- MuJoCo simulation backend
- Hardware abstraction layer
- Sensor simulation
- AI/perception toolkit
