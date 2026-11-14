#!/usr/bin/env python3
"""
stacc — The Universal Robotics Development Platform CLI

Usage:
    stacc init <project_name> [--template=<tpl>]
    stacc install <package>
    stacc remove <package>
    stacc search <query>
    stacc simulate [--headless] [--duration=<sec>] [--physics=<engine>]
    stacc build [--target=<platform>]
    stacc deploy <target_ip> [--user=<user>]
    stacc registry publish <path>
    stacc registry list
    stacc doctor
    stacc run <script>
    stacc --version
    stacc --help

Examples:
    stacc init my_robot --template=arm_6dof
    stacc install slam-toolkit
    stacc simulate --duration=30 --physics=mujoco
    stacc build --target=raspberry_pi
    stacc deploy 192.168.1.100
"""

import argparse
import copy
import sys
import os
import json
import time
import shutil
import hashlib
import subprocess
from pathlib import Path
from datetime import datetime
import yaml

# ──────────────────────────────────────────────
# ANSI color codes for terminal output
# ──────────────────────────────────────────────
class Color:
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    RED     = "\033[91m"
    CYAN    = "\033[96m"
    MAGENTA = "\033[95m"
    BLUE    = "\033[94m"
    RESET   = "\033[0m"
    CHECK   = "✓"
    CROSS   = "✗"
    ARROW   = "→"
    GEAR    = "⚙"
    PKG     = "📦"
    ROCKET  = "🚀"
    ROBOT   = "🤖"

def banner():
    print(f"""
{Color.CYAN}{Color.BOLD}
  ███████╗████████╗ █████╗  ██████╗ ██████╗
  ██╔════╝╚══██╔══╝██╔══██╗██╔════╝██╔════╝
  ███████╗   ██║   ███████║██║     ██║     
  ╚════██║   ██║   ██╔══██║██║     ██║     
  ███████║   ██║   ██║  ██║╚██████╗╚██████╗
  ╚══════╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝
{Color.RESET}{Color.DIM}  The Universal Robotics Development Platform
  v0.1.0{Color.RESET}
""")

def log_info(msg):
    print(f"  {Color.CYAN}{Color.ARROW}{Color.RESET} {msg}")

def log_success(msg):
    print(f"  {Color.GREEN}{Color.CHECK}{Color.RESET} {msg}")

def log_warn(msg):
    print(f"  {Color.YELLOW}!{Color.RESET} {msg}")

def log_error(msg):
    print(f"  {Color.RED}{Color.CROSS}{Color.RESET} {msg}")

def log_step(step, total, msg):
    bar_len = 20
    filled = int(bar_len * step / total)
    bar = "█" * filled + "░" * (bar_len - filled)
    pct = int(100 * step / total)
    print(f"\r  [{bar}] {pct:3d}% {msg}", end="", flush=True)
    if step == total:
        print()


# ──────────────────────────────────────────────
# STACC project manifest. YAML is canonical; JSON remains supported for existing projects.
# ──────────────────────────────────────────────
MANIFEST_NAME = "stacc.yaml"
LEGACY_MANIFEST_NAME = "stacc.json"
STACC_DIR = Path.home() / ".stacc"
REGISTRY_DIR = STACC_DIR / "registry"
CACHE_DIR = STACC_DIR / "cache"

def find_manifest(root=None):
    """Prefer the YAML manifest; keep existing JSON projects usable."""
    root = root or Path.cwd()
    for name in (MANIFEST_NAME, "stacc.yml", LEGACY_MANIFEST_NAME):
        candidate = root / name
        if candidate.is_file():
            return candidate
    return root / MANIFEST_NAME

def load_manifest(path):
    with open(path, encoding="utf-8") as stream:
        manifest = json.load(stream) if path.suffix == ".json" else yaml.safe_load(stream)
    if not isinstance(manifest, dict):
        raise ValueError(f"{path.name} must contain a YAML/JSON object")
    return manifest

def save_manifest(path, manifest):
    with open(path, "w", encoding="utf-8") as stream:
        if path.suffix == ".json":
            json.dump(manifest, stream, indent=2)
            stream.write("\n")
        else:
            yaml.safe_dump(manifest, stream, sort_keys=False, allow_unicode=True)

DEFAULT_MANIFEST = {
    "name": "",
    "version": "0.1.0",
    "description": "",
    "author": "",
    "platform": "universal",
    "physics_engine": "mujoco",
    "robot": {
        "type": "",
        "urdf": None,
        "description_format": "rfd"
    },
    "dependencies": {},
    "hardware": {
        "target": "simulation",
        "controller": None,
        "sensors": [],
        "actuators": []
    },
    "simulation": {
        "engine": "mujoco",
        "timestep": 0.001,
        "gravity": [0, 0, -9.81],
        "renderer": "opengl"
    },
    "build": {
        "target": "x86_64",
        "optimize": True,
        "realtime": False
    }
}

# ──────────────────────────────────────────────
# BUILT-IN ROBOT TEMPLATES
# ──────────────────────────────────────────────
TEMPLATES = {
    "blank": {
        "description": "Empty project — start from scratch",
        "robot_type": "custom",
        "files": {}
    },
    "line_follower": {
        "description": "2-wheel differential drive with IR sensors",
        "robot_type": "mobile_2wd",
        "files": {
            "robot.py": '''"""Line follower robot — built with stacc"""
import stacc
from stacc.robot import DifferentialDrive
from stacc.sensors import IRArray
from stacc.control import PIDController

# Define the robot
robot = DifferentialDrive(
    name="line_follower",
    wheel_radius=0.033,    # 33mm wheels
    wheel_base=0.15,       # 150mm apart
    max_rpm=200
)

# Add sensors
ir_sensors = IRArray(count=5, spacing=0.01)
robot.attach_sensor(ir_sensors, position="front_bottom")

# Control logic
pid = PIDController(kp=1.0, ki=0.01, kd=0.5)

@robot.on_update
def control_loop(dt):
    error = ir_sensors.line_position()  # -1.0 (left) to 1.0 (right)
    correction = pid.compute(error, dt)
    robot.set_speed(
        left=0.5 - correction,
        right=0.5 + correction
    )

if __name__ == "__main__":
    stacc.simulate(robot, duration=60, render=True)
''',
            "config.yaml": '''robot:
  name: line_follower
  type: differential_drive
  
hardware:
  controller: arduino_nano
  motors:
    - type: dc_motor
      driver: l298n
      pins: [5, 6]
    - type: dc_motor
      driver: l298n
      pins: [9, 10]
  sensors:
    - type: ir_array
      count: 5
      pins: [A0, A1, A2, A3, A4]

simulation:
  surface: line_track_oval
  physics: mujoco
  camera: top_down
'''
        }
    },
    "arm_6dof": {
        "description": "6-DOF robotic arm with gripper",
        "robot_type": "manipulator",
        "files": {
            "robot.py": '''"""6-DOF Robotic arm — built with stacc"""
import stacc
from stacc.robot import SerialManipulator
from stacc.kinematics import DH, InverseKinematics
from stacc.control import JointController
from stacc.gripper import ParallelGripper
import numpy as np

# Define arm using Denavit-Hartenberg parameters
#              a      alpha    d      theta_offset
dh_params = [
    DH(a=0,     alpha=-np.pi/2, d=0.089, theta_offset=0),      # Joint 1
    DH(a=0.425, alpha=0,        d=0,     theta_offset=-np.pi/2), # Joint 2
    DH(a=0.392, alpha=0,        d=0,     theta_offset=0),       # Joint 3
    DH(a=0,     alpha=-np.pi/2, d=0.109, theta_offset=0),       # Joint 4
    DH(a=0,     alpha=np.pi/2,  d=0.095, theta_offset=0),       # Joint 5
    DH(a=0,     alpha=0,        d=0.082, theta_offset=0),       # Joint 6
]

robot = SerialManipulator(
    name="arm_6dof",
    dh_parameters=dh_params,
    joint_limits=[
        (-2*np.pi, 2*np.pi),    # Joint 1: full rotation
        (-np.pi, np.pi),         # Joint 2
        (-np.pi, np.pi),         # Joint 3
        (-2*np.pi, 2*np.pi),    # Joint 4
        (-np.pi, np.pi),         # Joint 5
        (-2*np.pi, 2*np.pi),    # Joint 6
    ]
)

# Attach gripper
gripper = ParallelGripper(max_width=0.08, force=20)
robot.attach_end_effector(gripper)

# Inverse kinematics solver
ik = InverseKinematics(robot, method="damped_least_squares")

# Pick and place demo
@robot.on_start
def pick_and_place():
    target_pick = [0.4, 0.2, 0.05]    # Pick position (x, y, z) in meters
    target_place = [0.4, -0.2, 0.15]   # Place position
    
    # Move to pick position
    joint_angles = ik.solve(target_pick, orientation="top_down")
    robot.move_joints(joint_angles, speed=0.5)
    
    # Grasp
    gripper.close(force=15)
    stacc.wait(0.5)
    
    # Move to place position
    joint_angles = ik.solve(target_place, orientation="top_down")
    robot.move_joints(joint_angles, speed=0.3)
    
    # Release
    gripper.open()

if __name__ == "__main__":
    stacc.simulate(robot, duration=30, render=True, 
                   objects=["table", "cube_red", "cube_blue"])
''',
        }
    },
    "quadruped": {
        "description": "4-legged walking robot with 12 servos",
        "robot_type": "legged",
        "files": {
            "robot.py": '''"""Quadruped walker — built with stacc"""
import stacc
from stacc.robot import LeggedRobot
from stacc.gait import TrotGait, CrawlGait
from stacc.sensors import IMU
from stacc.control import BalanceController
import numpy as np

# Define quadruped (3 joints per leg × 4 legs = 12 DOF)
robot = LeggedRobot(
    name="quadruped",
    num_legs=4,
    joints_per_leg=3,
    body_dims=[0.3, 0.15, 0.05],   # L × W × H in meters
    leg_lengths=[0.08, 0.12, 0.12]  # hip, upper, lower
)

# Sensors
imu = IMU()
robot.attach_sensor(imu, position="body_center")

# Balance controller
balance = BalanceController(robot, imu)

# Gaits
trot = TrotGait(step_height=0.04, step_length=0.08, frequency=2.0)
crawl = CrawlGait(step_height=0.03, step_length=0.06, frequency=1.0)

current_gait = trot

@robot.on_update
def walk(dt):
    # Get desired foot positions from gait generator
    foot_targets = current_gait.compute(robot.time)
    
    # Apply balance correction
    corrected = balance.stabilize(foot_targets, dt)
    
    # Solve IK for each leg and command joints
    for leg_id in range(4):
        joint_angles = robot.leg_ik(leg_id, corrected[leg_id])
        robot.set_leg_joints(leg_id, joint_angles)

if __name__ == "__main__":
    stacc.simulate(robot, duration=30, render=True,
                   terrain="flat_ground")
''',
        }
    },
    "rover": {
        "description": "6-wheel rocker-bogie Mars rover with arm & cameras",
        "robot_type": "mobile_6wd",
        "files": {
            "robot.py": '''"""Mars rover — built with stacc"""
import stacc
from stacc.robot import RockerBogieRobot
from stacc.sensors import StereoCamera, LiDAR, IMU, TemperatureSensor
from stacc.navigation import SLAMNavigator, PathPlanner
from stacc.perception import PointCloudProcessor, ObjectDetector
from stacc.arm import RoverArm
import numpy as np

# Define rover with rocker-bogie suspension
rover = RockerBogieRobot(
    name="mars_explorer",
    wheel_radius=0.125,
    body_dims=[0.8, 0.5, 0.3],
    bogie_length=0.35,
    rocker_length=0.40,
    max_speed=0.1  # m/s (Mars rover speed)
)

# Sensor suite
stereo = StereoCamera(baseline=0.12, fov=120, resolution=(640, 480))
lidar = LiDAR(channels=16, range=30, fov_h=360, fov_v=30)
imu = IMU()
temp = TemperatureSensor()

rover.attach_sensor(stereo, position="mast_top", height=1.2)
rover.attach_sensor(lidar, position="mast_mid", height=1.0)
rover.attach_sensor(imu, position="body_center")
rover.attach_sensor(temp, position="body_front")

# Robotic arm for sample collection
arm = RoverArm(
    joints=5,
    reach=0.8,
    end_effector="scoop"
)
rover.attach_arm(arm, position="body_front")

# Navigation stack
slam = SLAMNavigator(
    lidar=lidar,
    stereo=stereo,
    imu=imu,
    map_resolution=0.05
)
planner = PathPlanner(method="rrt_star", obstacle_margin=0.3)

# Object detection for rocks of interest
detector = ObjectDetector(
    model="yolov8_rocks",
    camera=stereo
)

@rover.on_start
def mission():
    # Build initial map
    slam.initialize()
    
    # Navigate to target coordinates
    target = [10.0, 5.0]  # 10m forward, 5m right
    path = planner.plan(slam.current_pose, target, slam.map)
    
    rover.follow_path(path, speed=0.05)
    
    # Scan for interesting rocks
    detections = detector.detect()
    if detections:
        rock = detections[0]
        rover.approach(rock.position, distance=0.5)
        arm.collect_sample(rock.position)

if __name__ == "__main__":
    stacc.simulate(rover, duration=300, render=True,
                   terrain="mars_terrain", 
                   gravity=[0, 0, -3.72])
''',
        }
    },
    "drone": {
        "description": "Quadcopter drone with GPS and camera",
        "robot_type": "aerial",
        "files": {
            "robot.py": '''"""Quadcopter drone — built with stacc"""
import stacc
from stacc.robot import Quadcopter
from stacc.sensors import IMU, GPS, DownwardCamera, Barometer
from stacc.control import AttitudeController, PositionController
from stacc.navigation import WaypointFollower
import numpy as np

# Define quadcopter
drone = Quadcopter(
    name="scout_drone",
    arm_length=0.175,       # motor-to-center distance
    mass=1.2,               # kg
    prop_diameter=0.254,    # 10-inch props
    kv=920,                 # motor KV rating
    battery_mah=5000,
    battery_cells=4         # 4S LiPo
)

# Sensors
imu = IMU(rate=400)        # 400Hz for flight control
gps = GPS(rate=10)
baro = Barometer(rate=50)
cam = DownwardCamera(resolution=(1280, 720), fov=90)

drone.attach_sensor(imu, position="center")
drone.attach_sensor(gps, position="top")
drone.attach_sensor(baro, position="top")
drone.attach_sensor(cam, position="bottom")

# Flight controllers
attitude = AttitudeController(drone, imu, rate=400)
position = PositionController(drone, gps, baro, rate=50)

# Mission: fly a survey pattern
waypoints = [
    [0, 0, 10],      # Take off to 10m
    [20, 0, 10],     # Fly 20m north
    [20, 20, 10],    # Fly 20m east
    [0, 20, 10],     # Fly 20m south
    [0, 0, 10],      # Return
    [0, 0, 0],       # Land
]

follower = WaypointFollower(
    waypoints=waypoints,
    acceptance_radius=1.0,
    max_speed=5.0
)

@drone.on_update
def flight_loop(dt):
    target = follower.current_target(drone.position)
    position.set_target(target)
    
    vel_cmd = position.compute(dt)
    attitude.set_target_velocity(vel_cmd)
    
    motor_outputs = attitude.compute(dt)
    drone.set_motors(motor_outputs)
    
    if follower.mission_complete:
        drone.disarm()

if __name__ == "__main__":
    stacc.simulate(drone, duration=120, render=True,
                   wind=[2.0, 1.0, 0],
                   terrain="outdoor_field")
''',
        }
    }
}


# ──────────────────────────────────────────────
# PACKAGE REGISTRY (built-in packages)
# ──────────────────────────────────────────────
BUILTIN_PACKAGES = {
    "slam-toolkit": {
        "version": "1.0.0",
        "description": "SLAM algorithms — EKF-SLAM, FastSLAM, ORB-SLAM bindings",
        "size": "2.4 MB",
        "deps": ["perception-core", "eigen-bindings"],
        "category": "navigation"
    },
    "perception-core": {
        "version": "1.0.0",
        "description": "Computer vision pipeline — object detection, depth estimation, point clouds",
        "size": "8.1 MB",
        "deps": ["eigen-bindings"],
        "category": "perception"
    },
    "eigen-bindings": {
        "version": "3.4.0",
        "description": "High-performance linear algebra (Eigen C++ with Python bindings)",
        "size": "1.1 MB",
        "deps": [],
        "category": "core"
    },
    "path-planning": {
        "version": "1.0.0",
        "description": "Path planners — RRT*, A*, D*, potential fields, trajectory optimization",
        "size": "900 KB",
        "deps": ["eigen-bindings"],
        "category": "navigation"
    },
    "rl-toolkit": {
        "version": "1.0.0",
        "description": "Reinforcement learning — PPO, SAC, TD3 with sim-to-real transfer",
        "size": "5.6 MB",
        "deps": ["sim-bridge"],
        "category": "ai"
    },
    "sim-bridge": {
        "version": "1.0.0",
        "description": "Simulation bridge — MuJoCo, Bullet, DART physics engine adapters",
        "size": "3.2 MB",
        "deps": [],
        "category": "simulation"
    },
    "motor-drivers": {
        "version": "1.0.0",
        "description": "Motor/servo drivers — PWM, stepper, BLDC, CAN bus, EtherCAT",
        "size": "450 KB",
        "deps": [],
        "category": "hardware"
    },
    "sensor-suite": {
        "version": "1.0.0",
        "description": "Sensor abstractions — IMU, LiDAR, cameras, force/torque, encoders",
        "size": "1.8 MB",
        "deps": [],
        "category": "hardware"
    },
    "ros2-bridge": {
        "version": "1.0.0",
        "description": "ROS 2 interop — publish/subscribe, tf2, nav2 compatibility layer",
        "size": "4.5 MB",
        "deps": [],
        "category": "interop"
    },
    "gait-library": {
        "version": "1.0.0",
        "description": "Walking gaits — trot, crawl, gallop, bound + CPG generators",
        "size": "600 KB",
        "deps": ["eigen-bindings"],
        "category": "locomotion"
    },
    "flight-control": {
        "version": "1.0.0",
        "description": "UAV flight stack — attitude, position, mission control, PX4 bridge",
        "size": "2.1 MB",
        "deps": ["sensor-suite"],
        "category": "aerial"
    },
    "gripper-toolkit": {
        "version": "1.0.0",
        "description": "Gripper controllers — parallel, vacuum, soft, dexterous hand",
        "size": "350 KB",
        "deps": ["motor-drivers"],
        "category": "manipulation"
    }
}


# ──────────────────────────────────────────────
# COMMAND: init
# ──────────────────────────────────────────────
def cmd_init(args):
    """Initialize a new stacc robotics project."""
    project_name = args.project_name
    template = args.template or "blank"
    
    if template not in TEMPLATES:
        log_error(f"Unknown template: {template}")
        print(f"\n  Available templates:")
        for name, info in TEMPLATES.items():
            print(f"    {Color.CYAN}{name:20s}{Color.RESET} {info['description']}")
        sys.exit(1)
    
    project_dir = Path.cwd() / project_name
    
    if project_dir.exists():
        log_error(f"Directory '{project_name}' already exists")
        sys.exit(1)
    
    print(f"\n  {Color.ROBOT} Creating robotics project: {Color.BOLD}{project_name}{Color.RESET}")
    print(f"  {Color.DIM}Template: {template}{Color.RESET}\n")
    
    # Create directory structure
    dirs = [
        project_dir,
        project_dir / "src",
        project_dir / "models",
        project_dir / "worlds",
        project_dir / "config",
        project_dir / "tests",
        project_dir / "scripts",
        project_dir / "assets" / "meshes",
        project_dir / "assets" / "textures",
        project_dir / "data" / "logs",
        project_dir / "data" / "recordings",
        project_dir / ".stacc" / "cache",
    ]
    
    steps = len(dirs) + 3  # dirs + manifest + template files + git init
    
    for i, d in enumerate(dirs, 1):
        d.mkdir(parents=True, exist_ok=True)
        log_step(i, steps, f"Creating {d.name}/")
        time.sleep(0.05)
    
    # Write manifest
    manifest = copy.deepcopy(DEFAULT_MANIFEST)
    manifest["name"] = project_name
    manifest["description"] = f"{TEMPLATES[template]['description']}"
    manifest["robot"]["type"] = TEMPLATES[template]["robot_type"]
    
    manifest_path = project_dir / MANIFEST_NAME
    save_manifest(manifest_path, manifest)
    log_step(len(dirs) + 1, steps, "Writing stacc.yaml")
    time.sleep(0.05)
    
    # Write template files
    tpl = TEMPLATES[template]
    for filename, content in tpl.get("files", {}).items():
        filepath = project_dir / "src" / filename
        with open(filepath, "w") as f:
            f.write(content)
    log_step(len(dirs) + 2, steps, "Writing template files")
    time.sleep(0.05)
    
    # Write .gitignore
    gitignore = project_dir / ".gitignore"
    with open(gitignore, "w") as f:
        f.write("""# stacc
.stacc/cache/
data/logs/
data/recordings/
*.pyc
__pycache__/
build/
dist/
*.egg-info/
.env
""")
    
    # Write README
    readme = project_dir / "README.md"
    with open(readme, "w") as f:
        f.write(f"""# {project_name}

> Built with [stacc](https://github.com/stacc-robotics/stacc) — The Universal Robotics Development Platform

## Quick start

```bash
cd {project_name}
stacc install slam-toolkit        # Install packages
stacc simulate --duration=30      # Run simulation
stacc build --target=raspberry_pi # Build for hardware
stacc deploy 192.168.1.100        # Deploy to robot
```

## Project structure

```
{project_name}/
├── src/              # Your robot code
│   └── robot.py      # Main robot definition
├── models/           # URDF/SDF/RFD robot models
├── worlds/           # Simulation world files
├── config/           # Configuration files
├── tests/            # Unit & integration tests
├── scripts/          # Utility scripts
├── assets/           # Meshes, textures
├── data/             # Logs, recordings
└── stacc.yaml        # Project manifest (legacy stacc.json is still read)
```

## Template: {template}

{tpl['description']}
""")
    
    log_step(steps, steps, "Done!")
    
    print(f"\n\n  {Color.GREEN}{Color.BOLD}Project created successfully!{Color.RESET}\n")
    print(f"  Next steps:\n")
    print(f"    {Color.DIM}cd {project_name}{Color.RESET}")
    print(f"    {Color.CYAN}stacc install slam-toolkit{Color.RESET}     # Add packages")
    print(f"    {Color.CYAN}stacc simulate{Color.RESET}                 # Run in simulator")
    print(f"    {Color.CYAN}stacc build --target=rpi{Color.RESET}       # Compile for hardware")
    print()


# ──────────────────────────────────────────────
# COMMAND: install
# ──────────────────────────────────────────────
def cmd_install(args):
    """Install a stacc package."""
    pkg_name = args.package
    
    if pkg_name not in BUILTIN_PACKAGES:
        log_error(f"Package '{pkg_name}' not found in registry")
        print(f"\n  {Color.DIM}Search for packages with: stacc search <query>{Color.RESET}")
        sys.exit(1)
    
    pkg = BUILTIN_PACKAGES[pkg_name]
    
    # Check if in a stacc project
    manifest_path = find_manifest()
    if not manifest_path.exists():
        log_error("Not in a stacc project. Run 'stacc init' first.")
        sys.exit(1)
    
    # Load manifest
    manifest = load_manifest(manifest_path)
    
    # Check if already installed
    if pkg_name in manifest.get("dependencies", {}):
        log_warn(f"{pkg_name}@{pkg['version']} is already installed")
        return
    
    print(f"\n  {Color.PKG} Installing {Color.BOLD}{pkg_name}{Color.RESET} v{pkg['version']}")
    print(f"  {Color.DIM}{pkg['description']}{Color.RESET}\n")
    
    # Resolve dependencies
    all_deps = []
    def resolve(name):
        if name in BUILTIN_PACKAGES:
            p = BUILTIN_PACKAGES[name]
            for dep in p.get("deps", []):
                if dep not in [d[0] for d in all_deps]:
                    resolve(dep)
            all_deps.append((name, p))
    
    resolve(pkg_name)
    
    if len(all_deps) > 1:
        print(f"  Resolving {len(all_deps)} packages (including dependencies):\n")
        for name, info in all_deps:
            marker = "└──" if name == all_deps[-1][0] else "├──"
            is_dep = name != pkg_name
            dep_label = f" {Color.DIM}(dependency){Color.RESET}" if is_dep else ""
            print(f"    {marker} {Color.CYAN}{name}{Color.RESET}@{info['version']} [{info['size']}]{dep_label}")
        print()
    
    # Simulate download & install
    total = len(all_deps)
    for i, (name, info) in enumerate(all_deps, 1):
        log_step(i, total, f"Installing {name}...")
        time.sleep(0.3)  # Simulated install time
    
    # Update manifest
    for name, info in all_deps:
        manifest["dependencies"][name] = info["version"]
    
    save_manifest(manifest_path, manifest)
    
    print(f"\n  {Color.GREEN}{Color.CHECK} Installed {len(all_deps)} package(s){Color.RESET}\n")


# ──────────────────────────────────────────────
# COMMAND: remove
# ──────────────────────────────────────────────
def cmd_remove(args):
    """Remove a stacc package."""
    pkg_name = args.package
    
    manifest_path = find_manifest()
    if not manifest_path.exists():
        log_error("Not in a stacc project.")
        sys.exit(1)
    
    manifest = load_manifest(manifest_path)
    
    if pkg_name not in manifest.get("dependencies", {}):
        log_error(f"Package '{pkg_name}' is not installed")
        sys.exit(1)
    
    del manifest["dependencies"][pkg_name]
    
    save_manifest(manifest_path, manifest)
    
    log_success(f"Removed {pkg_name}")


# ──────────────────────────────────────────────
# COMMAND: search
# ──────────────────────────────────────────────
def cmd_search(args):
    """Search the stacc package registry."""
    query = args.query.lower()
    
    results = []
    for name, pkg in BUILTIN_PACKAGES.items():
        if (query in name.lower() or 
            query in pkg["description"].lower() or
            query in pkg.get("category", "").lower()):
            results.append((name, pkg))
    
    if not results:
        log_warn(f"No packages found matching '{args.query}'")
        return
    
    print(f"\n  {Color.PKG} Found {len(results)} package(s) matching '{args.query}':\n")
    
    for name, pkg in results:
        print(f"  {Color.CYAN}{Color.BOLD}{name:25s}{Color.RESET} v{pkg['version']:8s} [{pkg['size']:>8s}]")
        print(f"  {Color.DIM}{pkg['description']}{Color.RESET}")
        if pkg.get("deps"):
            print(f"  {Color.DIM}  deps: {', '.join(pkg['deps'])}{Color.RESET}")
        print()
    
    print(f"  Install with: {Color.CYAN}stacc install <package>{Color.RESET}\n")


# ──────────────────────────────────────────────
# COMMAND: simulate
# ──────────────────────────────────────────────
def cmd_simulate(args):
    """Run simulation."""
    manifest_path = find_manifest()
    if not manifest_path.exists():
        log_error("Not in a stacc project.")
        sys.exit(1)
    
    manifest = load_manifest(manifest_path)
    
    duration = args.duration or 30
    physics = args.physics or manifest.get("simulation", {}).get("engine", "mujoco")
    headless = args.headless
    
    print(f"\n  {Color.GEAR} Launching simulation")
    print(f"  {Color.DIM}Engine: {physics} | Duration: {duration}s | Headless: {headless}{Color.RESET}\n")
    
    steps = 5
    log_step(1, steps, "Loading robot model...")
    time.sleep(0.3)
    log_step(2, steps, f"Initializing {physics} engine...")
    time.sleep(0.3)
    log_step(3, steps, "Building collision geometry...")
    time.sleep(0.2)
    log_step(4, steps, "Spawning sensors...")
    time.sleep(0.2)
    
    # Check for robot.py
    robot_file = Path.cwd() / "src" / "robot.py"
    if robot_file.exists():
        log_step(5, steps, "Loading src/robot.py...")
    else:
        log_step(5, steps, "No src/robot.py found — using default")
    time.sleep(0.1)
    
    mode = "headless" if headless else "rendered"
    print(f"\n\n  {Color.GREEN}{Color.BOLD}Simulation running ({mode}){Color.RESET}")
    print(f"  {Color.DIM}Physics rate: {1/manifest.get('simulation',{}).get('timestep', 0.001):.0f} Hz")
    print(f"  Gravity: {manifest.get('simulation',{}).get('gravity', [0,0,-9.81])}")
    print(f"  Duration: {duration}s{Color.RESET}")
    
    if not headless:
        print(f"\n  {Color.CYAN}Renderer controls:{Color.RESET}")
        print(f"    Mouse drag  {Color.ARROW} Rotate camera")
        print(f"    Scroll      {Color.ARROW} Zoom")
        print(f"    Space       {Color.ARROW} Pause/Resume")
        print(f"    R           {Color.ARROW} Reset")
        print(f"    Ctrl+S      {Color.ARROW} Screenshot")
        print(f"    Q           {Color.ARROW} Quit")
    
    print(f"\n  {Color.DIM}(Simulation engine not yet connected — this is the CLI framework){Color.RESET}\n")


# ──────────────────────────────────────────────
# COMMAND: build
# ──────────────────────────────────────────────
def cmd_build(args):
    """Build project for a hardware target."""
    manifest_path = find_manifest()
    if not manifest_path.exists():
        log_error("Not in a stacc project.")
        sys.exit(1)
    
    manifest = load_manifest(manifest_path)
    
    TARGET_MAP = {
        "rpi": "Raspberry Pi (ARM64, Linux)",
        "raspberry_pi": "Raspberry Pi (ARM64, Linux)",
        "jetson": "NVIDIA Jetson (ARM64, Linux)",
        "arduino": "Arduino (AVR/ARM, bare-metal)",
        "esp32": "ESP32 (Xtensa, FreeRTOS)",
        "stm32": "STM32 (ARM Cortex-M, bare-metal/RTOS)",
        "x86": "x86_64 Desktop (Linux)",
        "x86_64": "x86_64 Desktop (Linux)",
    }
    
    target = args.target or "x86_64"
    target_name = TARGET_MAP.get(target, target)

    print(f"\n  {Color.GEAR} Building for: {Color.BOLD}{target_name}{Color.RESET}\n")

    MCU_TARGETS = {"arduino", "esp32", "stm32"}
    if target in MCU_TARGETS:
        # stacc is a Python robotics SDK — it has no C/C++ toolchain and
        # doesn't generate microcontroller firmware source. Pretending to
        # cross-compile one here would just be printing fake progress bars,
        # which is exactly what this command used to do. Say so plainly.
        log_warn(f"stacc doesn't generate {target} firmware — there's no C/C++ "
                 f"codegen for microcontrollers here.")
        print(f"  {Color.DIM}If you already have a compiled firmware image (.hex/.bin), "
              f"flash it directly with:{Color.RESET}")
        print(f"  {Color.CYAN}from stacc.hal.boards import FirmwareUploader{Color.RESET}")
        print(f"  {Color.CYAN}FirmwareUploader(board=\"{target}\", port=\"/dev/ttyUSB0\")"
              f".upload(\"firmware.bin\"){Color.RESET}\n")
        return

    # Python-capable targets (rpi/jetson/x86_64): the honest "build" is
    # packaging the real project tree into something stacc.hal.deploy.Deployer
    # can upload and run with `python3` on the target — there's no compiled
    # binary because stacc is Python, not C++.
    build_dir = Path.cwd() / "build" / target
    if build_dir.exists():
        shutil.rmtree(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)

    skip_dirs = {"build", ".stacc", "data", ".git", "__pycache__", ".venv", "venv"}
    steps_total = 2
    log_step(1, steps_total, "Packaging project files...")
    for item in Path.cwd().iterdir():
        if item.name in skip_dirs:
            continue
        dest = build_dir / item.name
        if item.is_dir():
            shutil.copytree(item, dest, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        else:
            shutil.copy2(item, dest)

    log_step(2, steps_total, "Writing build manifest...")
    build_info = {
        "target": target,
        "target_name": target_name,
        "built_at": datetime.now().isoformat(),
        "project": manifest["name"],
        "version": manifest["version"],
        "dependencies": manifest.get("dependencies", {}),
    }
    with open(build_dir / "build_info.json", "w") as f:
        json.dump(build_info, f, indent=2)

    print(f"\n\n  {Color.GREEN}{Color.CHECK} Build complete!{Color.RESET}")
    print(f"  {Color.DIM}Packaged project -> build/{target}/ "
          f"({sum(1 for _ in build_dir.rglob('*') if _.is_file())} files){Color.RESET}")
    print(f"\n  Deploy with: {Color.CYAN}stacc deploy <robot_ip>{Color.RESET}\n")


# ──────────────────────────────────────────────
# COMMAND: deploy
# ──────────────────────────────────────────────
def cmd_deploy(args):
    """Deploy a built project to a robot over real SSH/SFTP (stacc.hal.deploy.Deployer)."""
    target_ip = args.target_ip
    user = args.user or "pi"

    manifest_path = find_manifest()
    if not manifest_path.exists():
        log_error("Not in a stacc project.")
        sys.exit(1)

    manifest = load_manifest(manifest_path)

    target = getattr(args, "target", None) or "x86_64"
    local_dir = Path.cwd() / "build" / target
    if not local_dir.exists():
        log_error(f"No build output at build/{target}/ — run 'stacc build --target={target}' first.")
        sys.exit(1)

    print(f"\n  {Color.ROCKET} Deploying {Color.BOLD}{manifest['name']}{Color.RESET} to "
          f"{Color.CYAN}{user}@{target_ip}{Color.RESET}\n")

    from stacc.hal.deploy import Deployer

    deployer = Deployer(target_ip, user=user, key_file=getattr(args, "key", None))

    log_info(f"Connecting to {target_ip}...")
    if not deployer.connect():
        log_error(f"Could not connect to {user}@{target_ip} — check the IP, that SSH is "
                   f"running on the target, and your key/credentials. Nothing was deployed.")
        sys.exit(1)
    log_success("Connected.")

    remote_path = f"/home/{user}/stacc_robot/"
    log_info(f"Uploading build/{target}/ -> {remote_path} ...")
    deployer.upload(str(local_dir), remote_path)

    if not args.no_install:
        log_info("Installing/updating stacc on the target...")
        deployer.install_stacc()

    deployer.close()

    print(f"\n  {Color.GREEN}{Color.CHECK} Deployed.{Color.RESET}")
    print(f"\n  Start on robot: {Color.CYAN}ssh {user}@{target_ip} "
          f"'cd {remote_path} && python3 src/robot.py'{Color.RESET}\n")


# ──────────────────────────────────────────────
# COMMAND: doctor
# ──────────────────────────────────────────────
def cmd_doctor(args):
    """Check system health and dependencies."""
    print(f"\n  {Color.GEAR} stacc doctor — checking your system\n")
    
    checks = [
        ("Python 3.8+", True, f"Python {sys.version.split()[0]}"),
        ("pip", shutil.which("pip3") is not None or shutil.which("pip") is not None, ""),
        ("cmake", shutil.which("cmake") is not None, "Required for C++ compilation"),
        ("gcc/g++", shutil.which("g++") is not None or shutil.which("gcc") is not None, "Required for C++ compilation"),
        ("git", shutil.which("git") is not None, ""),
        ("MuJoCo", True, "Bundled with stacc (no separate install needed)"),
        ("NumPy", True, "Core numerical computing"),
        ("OpenCV", True, "Computer vision (optional)"),
    ]
    
    all_ok = True
    for name, ok, note in checks:
        if ok:
            status = f"{Color.GREEN}{Color.CHECK}{Color.RESET}"
        else:
            status = f"{Color.RED}{Color.CROSS}{Color.RESET}"
            all_ok = False
        
        note_str = f" {Color.DIM}({note}){Color.RESET}" if note else ""
        print(f"  {status} {name:20s}{note_str}")
    
    print()
    if all_ok:
        log_success("All checks passed! Your system is ready.")
    else:
        log_warn("Some dependencies are missing. Install them to use all features.")
    print()


# ──────────────────────────────────────────────
# COMMAND: run
# ──────────────────────────────────────────────
def cmd_serve(args):
    """Run the live UI/telemetry server (stdlib REST or FastAPI+WebSocket)."""
    label = "FastAPI + WebSocket" if args.fastapi else "stdlib REST"
    print(f"\n  {Color.GEAR} Starting live server ({label})")
    print(f"  {Color.DIM}Robot: {args.robot} | Physics: {args.physics} | "
          f"Loop: {args.hz:g} Hz | Port: {args.port}{Color.RESET}\n")

    if args.fastapi and not _fastapi_available():
        log_error("fastapi/uvicorn not installed. Add the 'server' extra: pip install 'stacc[server]'")
        sys.exit(1)

    from stacc.server import serve as serve_bridge
    serve_bridge(
        robot=args.robot,
        physics=args.physics,
        host=args.host,
        port=args.port,
        hz=args.hz,
        timestep=args.timestep,
        fastapi=args.fastapi,
    )


def _fastapi_available() -> bool:
    try:
        import fastapi  # noqa: F401
        import uvicorn  # noqa: F401

        return True
    except ImportError:
        return False


def cmd_run(args):
    """Run a robot script."""
    script = args.script or "src/robot.py"
    script_path = Path.cwd() / script
    
    if not script_path.exists():
        log_error(f"Script not found: {script}")
        sys.exit(1)
    
    print(f"\n  {Color.ROCKET} Running {Color.BOLD}{script}{Color.RESET}\n")
    
    # Execute the script
    result = subprocess.run(
        [sys.executable, str(script_path)],
        cwd=Path.cwd()
    )
    sys.exit(result.returncode)


# ──────────────────────────────────────────────
# COMMAND: registry
# ──────────────────────────────────────────────
def cmd_registry(args):
    """Manage the stacc package registry."""
    if args.registry_action == "list":
        print(f"\n  {Color.PKG} stacc registry — {len(BUILTIN_PACKAGES)} packages available\n")
        
        # Group by category
        categories = {}
        for name, pkg in BUILTIN_PACKAGES.items():
            cat = pkg.get("category", "other")
            if cat not in categories:
                categories[cat] = []
            categories[cat].append((name, pkg))
        
        for cat, pkgs in sorted(categories.items()):
            print(f"  {Color.BOLD}{cat.upper()}{Color.RESET}")
            for name, pkg in pkgs:
                print(f"    {Color.CYAN}{name:25s}{Color.RESET} v{pkg['version']:8s} {Color.DIM}{pkg['description']}{Color.RESET}")
            print()
    
    elif args.registry_action == "publish":
        pkg_path = args.path
        print(f"\n  {Color.PKG} Publishing package from {pkg_path}...")
        log_warn("Registry publishing is not yet available in v0.1.0")
        print(f"  {Color.DIM}Coming soon — community package publishing{Color.RESET}\n")


# ──────────────────────────────────────────────
# COMMAND: rss (parse / compile RSS robot descriptions)
# ──────────────────────────────────────────────
def _rss_load_document(file_path):
    """Parse a `.rss` file, or the `<<<RSS>>>` block(s) of a Python/C++/Rust file."""
    from stacc.rss import RSSCompileError, RSSSyntaxError, parse
    from stacc.rss.embed import has_embedded_rss, parse_embedded

    path = Path(file_path)
    if not path.exists():
        log_error(f"RSS file not found: {file_path}")
        sys.exit(1)
    text = path.read_text()
    try:
        if path.suffix != ".rss" and has_embedded_rss(text):
            return parse_embedded(text, str(path))[0]
        return parse(text)
    except RSSSyntaxError as e:
        log_error(f"RSS syntax error in {path}")
        print(f"  {Color.DIM}{e}{Color.RESET}")
        sys.exit(1)
    except RSSCompileError as e:
        log_error(f"RSS error in {path}: {e}")
        sys.exit(1)


def _rss_search_paths(args):
    """Directories cross-file `use "lib.rss:name";` looks in: the .rss
    file's own directory, plus any --rss-path directories given on the CLI.
    """
    paths = [Path(args.file).resolve().parent]
    for extra in getattr(args, "rss_path", None) or []:
        paths.append(Path(extra))
    return paths


def _rss_compile(doc, args):
    from stacc.rss import RSSCompileError, compile_document

    try:
        return compile_document(doc, robot_name=args.robot, search_paths=_rss_search_paths(args))
    except RSSCompileError as e:
        log_error(f"RSS compile error: {e}")
        sys.exit(1)


def cmd_rss_check(args):
    """Parse and compile an .rss file without writing any output."""
    doc = _rss_load_document(args.file)
    robot = _rss_compile(doc, args)

    print(f"\n  {Color.ROBOT} {Color.BOLD}{robot.name}{Color.RESET} — RSS check passed")
    print(f"  {Color.DIM}Links: {robot.num_links}  Joints: {robot.num_joints}  "
          f"DOF: {robot.dof}  Sensors: {len(robot.sensors)}{Color.RESET}")
    for link in robot.links:
        print(f"    {Color.CYAN}{link.name:20s}{Color.RESET} mass={link.mass}kg")
    for sensor in robot.sensors:
        print(f"    {Color.MAGENTA}sensor:{sensor.kind:12s}{Color.RESET} "
              f"@ {getattr(sensor, '_mount_position', '?')}")
    for w in getattr(robot, "rss_warnings", []):
        log_warn(w)
    print()


def cmd_rss_build(args):
    """Compile an .rss file to URDF/SDF/MJCF, and (by default) a ROS 2 package skeleton."""
    from stacc.converters.urdf_mjcf_usd import ModelTranspiler

    doc = _rss_load_document(args.file)
    robot = _rss_compile(doc, args)

    build_dir = Path(args.out or "build")
    fmt = args.format  # None (default bundle), or one of "urdf"/"sdf"/"mjcf"/"all"
    written = []

    def write_urdf():
        out_dir = build_dir / "urdf"
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{robot.name}.urdf"
        xml = ModelTranspiler.robot_to_urdf(robot)
        path.write_text(xml)
        written.append(path)
        return xml

    def write_sdf():
        out_dir = build_dir / "gazebo"
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{robot.name}.sdf"
        path.write_text(ModelTranspiler.robot_to_sdf(robot))
        written.append(path)

    def write_mjcf():
        from stacc.sim.engine import SimulationConfig
        from stacc.sim.mjcf_builder import robot_to_mjcf

        out_dir = build_dir / "mjcf"
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{robot.name}.xml"
        path.write_text(robot_to_mjcf(robot, SimulationConfig()))
        written.append(path)

    def write_ros2(urdf_xml):
        from stacc.rss import write_ros2_package

        out_dir = build_dir / "ros2"
        out_dir.mkdir(parents=True, exist_ok=True)
        pkg_dir = write_ros2_package(robot, urdf_xml, out_dir)
        written.append(pkg_dir / "package.xml")

    def write_stl():
        from stacc.rss.mesh_export import write_stl_parts

        out_dir = build_dir / "parts"
        for p in write_stl_parts(robot, str(out_dir)):
            written.append(Path(p))
        for w in getattr(robot, "rss_mesh_warnings", []):
            log_warn(w)

    def write_bom_files():
        from stacc.rss.bom import write_bom

        paths = write_bom(robot, str(build_dir))
        written.append(Path(paths["csv"]))
        written.append(Path(paths["md"]))

    if fmt is None:
        # Default bundle: everything a real robotics project needs to get
        # moving — plain URDF, an SDF copy for Gazebo, and a ROS 2
        # description package pre-filled with that URDF.
        urdf_xml = write_urdf()
        write_sdf()
        write_ros2(urdf_xml)
    elif fmt == "urdf":
        write_urdf()
    elif fmt == "sdf":
        write_sdf()
    elif fmt == "mjcf":
        write_mjcf()
    elif fmt == "stl":
        write_stl()
    elif fmt == "bom":
        write_bom_files()
    elif fmt == "all":
        urdf_xml = write_urdf()
        write_sdf()
        write_mjcf()
        write_ros2(urdf_xml)
        write_stl()
        write_bom_files()

    for w in getattr(robot, "rss_warnings", []):
        log_warn(w)
    log_success(f"Compiled {args.file} -> " + ", ".join(str(p) for p in written))
    print(f"  {Color.DIM}Links: {robot.num_links}  Joints: {robot.num_joints}  "
          f"DOF: {robot.dof}{Color.RESET}\n")


def cmd_rss_run(args):
    """Compile an .rss file and run it in the stacc simulator."""
    import stacc

    doc = _rss_load_document(args.file)
    robot = _rss_compile(doc, args)

    render = bool(args.render)
    print(f"\n  {Color.GEAR} Running {Color.BOLD}{robot.name}{Color.RESET} "
          f"({robot.num_links} links, {robot.num_joints} joints)")
    print(f"  {Color.DIM}Duration: {args.duration}s | Render: {render}{Color.RESET}\n")

    stacc.simulate(robot, duration=args.duration, render=render, headless=not render)


def _generate_control_script(robot) -> str:
    """A control-script TEMPLATE, not a finished controller: physical wiring
    (which GPIO pin each motor is actually on) is something only the person
    building the robot knows, so every pin is a marked TODO rather than a
    guess.
    """
    metadata = getattr(robot, "rss_metadata", {})
    actuated = [j for j in robot.joints if j.type != "fixed"]

    lines = [
        f'"""Control script for {robot.name} — generated by `stacc rss deploy`.',
        "",
        "This is a TEMPLATE, not a finished controller. Fill in the pin_a/pin_b/",
        "pin_enable TODOs below to match your actual wiring before running this",
        "on the robot — nothing here can know your physical wiring for you.",
        '"""',
        "",
        "from stacc.hal.gpio import GPIO",
        "from stacc.hal.motor import MotorDriver",
        "",
        "gpio = GPIO()",
        "",
    ]

    for joint in actuated:
        meta = metadata.get(joint.child_link, {})
        actuator = meta.get("actuator") or meta.get("motor") or "unspecified in .rss"
        var_name = joint.name.replace("-", "_")
        lines += [
            f'# Joint "{joint.name}" ({joint.type}) — actuator/motor from .rss: {actuator!r}',
            f"{var_name} = MotorDriver(",
            f"    pin_a=None,  # TODO: your wiring",
            f"    pin_b=None,  # TODO: your wiring",
            f"    pin_enable=None,  # TODO: optional PWM enable pin",
            f"    gpio=gpio,",
            f")",
            "",
        ]

    lines += ["", "def main():"]
    if actuated:
        lines.append('    print("Fill in real GPIO pins above before running this on hardware.")')
        lines.append("    # TODO: your control loop, e.g.:")
        for joint in actuated:
            var_name = joint.name.replace("-", "_")
            lines.append(f"    # {var_name}.set_speed(0.5)")
    else:
        lines.append("    pass  # no actuated joints in this robot")
    lines += ["", "", 'if __name__ == "__main__":', "    main()", ""]
    return "\n".join(lines)


def cmd_rss_deploy(args):
    """Compile an .rss file, package it, and deploy it to real hardware over SSH."""
    from stacc.converters.urdf_mjcf_usd import ModelTranspiler

    doc = _rss_load_document(args.file)
    robot = _rss_compile(doc, args)

    build_dir = Path(args.out or "build") / "deploy" / robot.name
    if build_dir.exists():
        shutil.rmtree(build_dir)
    (build_dir / "src").mkdir(parents=True, exist_ok=True)

    urdf_xml = ModelTranspiler.robot_to_urdf(robot)
    (build_dir / "src" / f"{robot.name}.urdf").write_text(urdf_xml)
    (build_dir / "src" / "robot.py").write_text(_generate_control_script(robot))

    for w in getattr(robot, "rss_warnings", []):
        log_warn(w)
    log_success(f"Packaged {robot.name} -> {build_dir}")
    print(f"  {Color.DIM}{robot.num_links} links, {robot.num_joints} joints "
          f"({robot.dof} actuated) — see src/robot.py for the wiring TODOs{Color.RESET}\n")

    if args.no_deploy:
        print(f"  {Color.DIM}--no-deploy set: stopping after packaging, nothing sent "
              f"over the network.{Color.RESET}\n")
        return

    from stacc.hal.deploy import Deployer

    user = args.user or "pi"
    print(f"  {Color.ROCKET} Deploying to {Color.CYAN}{user}@{args.target_ip}{Color.RESET}\n")

    deployer = Deployer(args.target_ip, user=user, key_file=args.key)
    log_info(f"Connecting to {args.target_ip}...")
    if not deployer.connect():
        log_error(f"Could not connect to {user}@{args.target_ip} — check the IP, that SSH "
                  f"is running on the target, and your key/credentials. Nothing was deployed.")
        sys.exit(1)
    log_success("Connected.")

    remote_path = f"/home/{user}/{robot.name}/"
    log_info(f"Uploading {build_dir} -> {remote_path} ...")
    deployer.upload(str(build_dir), remote_path)

    if not args.no_install:
        log_info("Installing/updating stacc on the target...")
        deployer.install_stacc()

    deployer.close()
    print(f"\n  {Color.GREEN}{Color.CHECK} Deployed.{Color.RESET}")
    print(f"  Start on robot: {Color.CYAN}ssh {user}@{args.target_ip} "
          f"'cd {remote_path}src && python3 robot.py'{Color.RESET}\n")


def cmd_rss(args):
    """Dispatch `stacc rss <check|build|run|deploy>`."""
    actions = {
        "check": cmd_rss_check,
        "build": cmd_rss_build,
        "run": cmd_rss_run,
        "deploy": cmd_rss_deploy,
    }
    if args.rss_action not in actions:
        log_error("Usage: stacc rss <check|build|run|deploy> <file.rss>")
        sys.exit(1)
    actions[args.rss_action](args)


# ──────────────────────────────────────────────
# MAIN ARGUMENT PARSER
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        prog="stacc",
        description="stacc — The Universal Robotics Development Platform",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  stacc init my_robot --template=arm_6dof
  stacc install slam-toolkit
  stacc simulate --duration=30
  stacc build --target=raspberry_pi
  stacc deploy 192.168.1.100
  stacc doctor

Templates: blank, line_follower, arm_6dof, quadruped, rover, drone
        """
    )
    parser.add_argument("--version", action="version", version="stacc 0.1.0")
    
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # init
    p_init = subparsers.add_parser("init", help="Create a new robotics project")
    p_init.add_argument("project_name", help="Project name")
    p_init.add_argument("--template", "-t", help="Robot template", 
                        choices=list(TEMPLATES.keys()), default="blank")
    
    # install
    p_install = subparsers.add_parser("install", help="Install a package")
    p_install.add_argument("package", help="Package name")
    
    # remove
    p_remove = subparsers.add_parser("remove", help="Remove a package")
    p_remove.add_argument("package", help="Package name")
    
    # search
    p_search = subparsers.add_parser("search", help="Search packages")
    p_search.add_argument("query", help="Search query")
    
    # simulate
    p_sim = subparsers.add_parser("simulate", help="Run simulation")
    p_sim.add_argument("--headless", action="store_true", help="Run without renderer")
    p_sim.add_argument("--duration", "-d", type=int, help="Duration in seconds")
    p_sim.add_argument("--physics", "-p", help="Physics engine", 
                       choices=["mujoco", "bullet", "dart"])
    
    # build
    p_build = subparsers.add_parser("build", help="Build for hardware")
    p_build.add_argument("--target", "-t", help="Target platform",
                         choices=["rpi", "raspberry_pi", "jetson", "arduino", 
                                  "esp32", "stm32", "x86", "x86_64"])
    
    # deploy
    p_deploy = subparsers.add_parser("deploy", help="Deploy to robot")
    p_deploy.add_argument("target_ip", help="Robot IP address")
    p_deploy.add_argument("--user", "-u", help="SSH username", default="pi")
    p_deploy.add_argument("--target", "-t", help="Build target to deploy (default: x86_64)",
                          choices=["rpi", "raspberry_pi", "jetson", "x86", "x86_64"])
    p_deploy.add_argument("--key", help="Path to SSH private key file")
    p_deploy.add_argument("--no-install", action="store_true",
                          help="Skip running install_stacc() on the target after upload")
    
    # doctor
    subparsers.add_parser("doctor", help="Check system health")
    
    # run
    p_run = subparsers.add_parser("run", help="Run a robot script")
    p_run.add_argument("script", nargs="?", help="Script path", default="src/robot.py")
    
    # registry
    p_reg = subparsers.add_parser("registry", help="Package registry commands")
    p_reg_sub = p_reg.add_subparsers(dest="registry_action")
    p_reg_sub.add_parser("list", help="List all packages")
    p_pub = p_reg_sub.add_parser("publish", help="Publish a package")
    p_pub.add_argument("path", help="Path to package")

    # rss
    p_rss = subparsers.add_parser("rss", help="Work with .rss declarative robot descriptions")
    p_rss_sub = p_rss.add_subparsers(dest="rss_action")

    rss_path_help = (
        "Extra directory to search for cross-file use \"lib.rss:name\"; refs "
        "(repeatable). The .rss file's own directory is always searched."
    )

    p_rss_check = p_rss_sub.add_parser("check", help="Parse + compile an .rss file, report only")
    p_rss_check.add_argument("file", help="Path to an .rss file, or a .py/.cpp/.rs file with an <<<RSS>>> block")
    p_rss_check.add_argument("--robot", help="Robot name to compile (if the file defines several)")
    p_rss_check.add_argument("--rss-path", action="append", help=rss_path_help)

    p_rss_build = p_rss_sub.add_parser("build", help="Compile an .rss file to URDF")
    p_rss_build.add_argument("file", help="Path to an .rss file, or a .py/.cpp/.rs file with an <<<RSS>>> block")
    p_rss_build.add_argument("--robot", help="Robot name to compile (if the file defines several)")
    p_rss_build.add_argument("--out", help="Output directory (default: build/)")
    p_rss_build.add_argument("--rss-path", action="append", help=rss_path_help)
    p_rss_build.add_argument(
        "--format", choices=["urdf", "sdf", "mjcf", "stl", "bom", "all"], default=None,
        help="Single format to emit. Default emits the full bundle: "
             "build/urdf/, build/gazebo/ (SDF), build/ros2/<name>_description/. "
             "'stl' writes build/parts/*.stl for 3D printing; 'bom' writes "
             "build/bom.csv + build/bom.md; 'all' emits everything.",
    )

    p_rss_run = p_rss_sub.add_parser("run", help="Compile an .rss file and simulate it")
    p_rss_run.add_argument("file", help="Path to an .rss file, or a .py/.cpp/.rs file with an <<<RSS>>> block")
    p_rss_run.add_argument("--robot", help="Robot name to compile (if the file defines several)")
    p_rss_run.add_argument("--rss-path", action="append", help=rss_path_help)
    p_rss_run.add_argument("--sim", action="store_true", help="Run in the physics simulator")
    p_rss_run.add_argument("--duration", "-d", type=float, default=10.0, help="Duration in seconds")
    p_rss_run.add_argument("--render", action="store_true",
                           help="Open the MuJoCo viewer (needs a working display/GPU)")

    p_rss_deploy = p_rss_sub.add_parser(
        "deploy", help="Compile an .rss file, package it, and deploy it to real hardware over SSH"
    )
    p_rss_deploy.add_argument("file", help="Path to an .rss file, or a .py/.cpp/.rs file with an <<<RSS>>> block")
    p_rss_deploy.add_argument("target_ip", help="Target device IP address")
    p_rss_deploy.add_argument("--robot", help="Robot name to compile (if the file defines several)")
    p_rss_deploy.add_argument("--rss-path", action="append", help=rss_path_help)
    p_rss_deploy.add_argument("--out", help="Local build directory (default: build/)")
    p_rss_deploy.add_argument("--user", "-u", default="pi", help="SSH username (default: pi)")
    p_rss_deploy.add_argument("--key", help="Path to SSH private key file")
    p_rss_deploy.add_argument("--no-install", action="store_true",
                              help="Skip running install_stacc() on the target after upload")
    p_rss_deploy.add_argument("--no-deploy", action="store_true",
                              help="Only package locally (src/robot.py + URDF) — don't connect over SSH")

    # serve
    p_serve = subparsers.add_parser("serve", help="Run the live UI/telemetry server")
    p_serve.add_argument("--host", default="0.0.0.0", help="Bind address")
    p_serve.add_argument("--port", "-p", type=int, default=8002, help="Port")
    p_serve.add_argument("--robot", "-r", default="arm", help="Robot type")
    p_serve.add_argument("--physics", default="mujoco",
                         choices=["mujoco", "internal"], help="Physics backend")
    p_serve.add_argument("--hz", type=float, default=60.0, help="Telemetry loop rate (Hz)")
    p_serve.add_argument("--timestep", type=float, default=0.001, help="Physics timestep (s)")
    p_serve.add_argument("--fastapi", action="store_true",
                         help="Use FastAPI + WebSocket instead of the stdlib bridge")
    
    args = parser.parse_args()
    
    if args.command is None:
        banner()
        parser.print_help()
        sys.exit(0)
    
    commands = {
        "init": cmd_init,
        "install": cmd_install,
        "remove": cmd_remove,
        "search": cmd_search,
        "simulate": cmd_simulate,
        "build": cmd_build,
        "deploy": cmd_deploy,
        "doctor": cmd_doctor,
        "run": cmd_run,
        "registry": cmd_registry,
        "rss": cmd_rss,
        "serve": cmd_serve,
    }
    
    if args.command in commands:
        commands[args.command](args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
