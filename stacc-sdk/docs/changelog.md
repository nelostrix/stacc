# Changelog

## 2026-11-14 — Live runtime reliability and safety (unreleased)

- Schedule control cycles against monotonic deadlines and distribute fractional
  fixed physics steps across cycles to prevent systematic timing drift.
- Validate runtime frequencies, physics timesteps, actuator vector sizes, finite
  values, joint targets, torque limits, rotor ranges, and controller gains.
- Add an opt-in stale-command watchdog, latched software emergency stop,
  explicit recovery, stale shared-memory command discard, and zero-command
  behavior on stop and runtime faults.
- Expose safety state and sensor read errors in telemetry, with emergency-stop
  routes on the stdlib, FastAPI, and NELO Studio STACC daemon APIs.
- Add `stacc serve --command-timeout` and `NELO_STACC_COMMAND_TIMEOUT` for the
  Studio daemon.
- Create MuJoCo camera contexts lazily on the sensor sampling thread, close
  them on their owning thread, and serve cached images to HTTP clients. This
  avoids EGL/GLX context errors when camera sensors run in the background loop.
- Declare the CLI's PyYAML dependency and update project-init tests to check the
  canonical `stacc.yaml` manifest. Convert legacy check scripts into
  pytest-discoverable functions while retaining their standalone runner.
- Add simulator-only runtime fault, watchdog, scheduling, and emergency-stop
  tests; no hardware motion or firmware upload was performed.

## 2026-06-10 — hal + gait + gripper + robot + control/ai improvements

### `stacc.sensors` — Full module overhaul

**`base.py`** — `BaseSensor` class:
- Added input validation on `rate > 0` and `noise_stddev >= 0`
- Added `reset()` method to clear last reading, timer, and re-enable sensor

**`imu.py`** — `IMU` class:
- **Bug fix**: `read()` was using `self.acceleration[i]` instead of `self.angular_velocity[i]` for gyroscope output
- Added `reset()` method to restore default readings (gravity at rest, zero angular velocity)
- Added input validation on `accel_noise`, `gyro_noise`, `mag_noise` (must be `>= 0`)
- Added `__repr__()` with noise parameters

**`camera.py`** — `Camera` class:
- Added input validation (`width > 0`, `height > 0`, `fov` in (0, 180))
- Added `reset()` method to clear image and backend references
- Added `__repr__()` with resolution and FOV

**`lidar.py`** — `LiDAR` class:
- Added input validation on all constructor args
- Added `reset()` method to clear scan data
- Added `__repr__()` with channel and range info

**`encoder.py`** — `Encoder` class:
- Added input validation (`ticks_per_rev > 0`, `gear_ratio > 0`)
- Added `reset()` method to zero tick counters
- Added `__repr__()` with resolution and gear ratio

**`ir.py`** — `IRArray` class:
- Added input validation (`count >= 1`, `spacing > 0`)
- Added `reset()` method to clear readings
- Added `__repr__()` with element count

**`ultrasonic.py`** — `Ultrasonic` class:
- Added input validation (`range_max > 0`, `range_min >= 0`, `range_min < range_max`, `beam_angle > 0`)
- Added `reset()` method
- Added `__repr__()` with range and beam angle

**`gps.py`** — `GPS` class:
- **Removed duplicate `Ultrasonic` class** that was accidentally copied here
- Added `reset()` method
- Added `__repr__()` with origin coordinates

**`barometer.py`** — `Barometer` class:
- Added `reset()` method
- Added `__repr__()`

**`temperature.py`** — `TemperatureSensor` class:
- Added input validation (`range_max > range_min`)
- Added `reset()` method
- Added `__repr__()` with range

**`contact.py`** — `ContactSensor` class:
- Added input validation (`threshold > 0`)
- Added `reset()` method to clear force state
- Added `__repr__()`

**`force_torque.py`** — `ForceTorqueSensor` class:
- Added input validation (`range_force > 0`, `range_torque > 0`)
- Added `reset()` method to zero wrench
- Added `__repr__()`

**`depth_camera.py`** — `DepthCamera` class:
- Added input validation on all constructor args
- Added `reset()` method
- Added `__repr__()`

**`stereo_camera.py`** — `StereoCamera` class:
- Added input validation (`baseline > 0`)
- Added `reset()` method (delegates to both cameras)
- Added `__repr__()` with baseline and resolution

### `stacc.robot` — Full module overhaul

**`differential_drive.py`** — `DifferentialDrive` class:
- Added input validation on all constructor args (`wheel_radius > 0`, `wheel_base > 0`, `max_rpm > 0`, `mass > 0`)
- Added `reset()` method to zero both wheel speeds
- Added `__repr__()` with key dimensions

### `stacc.sim` — Full module overhaul

**`mujoco_backend.py`** — `MuJoCoBackend` class:
- **Critical bug fix**: `close()` method was indented inside `step()`, making it unreachable
- Added `reset()` method to reset MuJoCo model state
- Added `__repr__()` with robot name, joint count, and viewer status

**`engine.py`** — `SimulationConfig`, `SimulationState`, `SimulationEngine`, `_InternalPhysics`:
- Added input validation (`duration > 0`, `timestep > 0`, `physics_engine` one of mujoco/bullet/internal)
- Added `reset()` on `SimulationState` and `SimulationEngine`
- Added `__repr__()` on all four classes

**`world.py`** — `WorldObject`, `Terrain`, `World` classes:
- Added input validation (`mass >= 0`, `friction >= 0`, box/sphere/cylinder dimension checks)
- Added `Terrain` type validation (flat/heightmap/slopes/stairs only)
- Added `__repr__()` on all three classes

**`recorder.py`** — `SimFrame`, `SimRecording`, `SimRecorder`:
- Added input validation (`record_every >= 1`)
- Added `__repr__()` on all three classes

**`gym_env.py`** — `StaccEnv` class:
- Added input validation (`max_steps >= 1`, `dt > 0`, `action_mode`, `obs_includes`, `obs_noise >= 0`, robot must have joints)
- Added `__repr__()` with robot name, action mode, and step count

**`__init__.py`** — Expanded exports:
- Added `SimulationState`, `World`, `WorldObject`, `Terrain`, `SimRecorder`, `SimRecording`, `SimFrame`, `load_recording`, `replay_recording`, `StaccEnv`, `reach_target_reward`, `balance_reward`, `locomotion_reward`

**`serial_manipulator.py`** — `SerialManipulator` class:
- Added input validation: `dh_parameters` type-checked as list of `DH` objects, `joint_limits` length/ordering validated
- Added `reset()` method to zero all joint positions
- Added `__repr__()` with joint count and parameter info
- Empty DH list is valid (zero-joint arm)

### `stacc.control` — Major expansions

**`filters.py`** — 4 new filter types added:
- `ComplementaryFilter` — gyroscope + accelerometer fusion for IMU attitude estimation. Blinds high-frequency gyro with low-frequency accel. Configurable alpha.
- `MovingAverageFilter` — sliding window average with configurable window size. Smoother than exponential for periodic noise.
- `KalmanFilter1D` — optimal recursive 1D state estimator. Configurable process/measurement noise. Supports velocity control input.
- `NotchFilter` — biquad band-stop filter to remove specific frequencies (e.g., 50/60 Hz electrical noise, mechanical resonance). Configurable Q factor.

**`feedforward.py`** — 2 new models added:
- `GravityCompensation` — computes gravity torques for serial manipulators using a simplified pendulum model. Takes link masses and COM distances.
- `FrictionModel` — Coulomb + viscous friction with stiction threshold. Includes `compute_array()` for multi-joint robots.

**`limiter.py`** — 2 new limiters added:
- `AccelerationLimiter` — two-stage velocity-then-acceleration clamping for smooth step response.
- `SCurveProfile` — full S-curve trajectory generator with bounded jerk. Produces continuous acceleration profiles (trapezoidal acceleration, not just trapezoidal velocity). Tracks position, velocity, acceleration state.

**`pipeline.py`** — Refactored with `ControllerStage`:
- Each stage is now a named `ControllerStage` with enable/disable and conditional skip support.
- `ControllerPipeline.add()`, `.remove()`, `.get()`, `.enable()`, `.disable()` for runtime stage management.
- Backwards compatible: existing `ControllerPipeline(*controllers)` still works, but outputs are now wrapped in `ControllerStage`.

### `stacc.ai` — Minor polish

**`behavior_tree.py`** — Added `-> None` return type hints to all `reset()` methods across all node types.

**`vision.py`** — `VisualOdometry.update()` now prints a clear warning when no `camera_matrix` is provided, explaining the fallback to 2D homography (no scale, x/y only).

### `stacc.gait` — Full module overhaul

**`base.py`** — `GaitGenerator` refactored:
- Added `swing_ratio` parameter to control swing/stance phase split (0.0–1.0, default 0.5)
- Added `set_direction(heading)` with proper yaw rotation of foot positions
- Added `reset()` method to reset phase offsets and heading after a fall or gait switch
- Added `num_legs` validation (must be >= 1)
- `compute()` now uses `time / period` instead of `time * frequency` for numerical consistency at very low/high frequencies
- Improved docstrings with full Args/Raises annotations

**`cpg.py`** — Central Pattern Generator expanded:
- Added named coupling pattern presets: `"trot"`, `"walk"`, `"bound"`, `"gallop"`, `"pace"`
- `CPGGenerator.__init__()` accepts a `pattern` kwarg for preset selection
- Added `set_pattern()`, `set_coupling_strength()`, `reset()` methods
- Added `set_speed()` and `set_direction()` for API consistency with `GaitGenerator`
- Input validation on `num_legs` and pattern leg count
- Full docstrings on all classes and public methods

**`biped.py`** — `BipedWalkGait` rewritten:
- Added `hip_width` parameter for configurable hip spacing
- Now uses explicit `step_height`, `step_length`, `frequency`, `body_height` kwargs (no more `**kwargs`)
- Proper elliptical foot trajectory with swing/stance phases

**`crawl.py`** — `CrawlGait` cleanup:
- Uses `swing_ratio=0.25` via base class rather than custom `_SWING_FRACTION`
- Explicit constructor kwargs instead of `**kwargs`

**`bound.py`, `gallop.py`, `trot.py`, `walk.py`, `tripod.py`** — Consistency pass:
- All gaits now use explicit constructor kwargs instead of `**kwargs`
- Consistent docstrings with Args annotations
- Consistent `_default_foot_pos` return `(0.0, 0.0)` instead of `(0, 0)`

**`flight.py`** — `FlightController` refined:
- Input validation on `num_rotors`
- Mixing matrix construction extracted to `_build_mixing_matrix()` static method
- Cleaned up type hints and docstrings

### `stacc.hal` — Full module overhaul

**`__init__.py`** — Properly exports all classes with documented usage:
- `Board`, `BoardDefinition`, `FirmwareUploader`, `get_board`, `list_boards`, `list_platforms`
- `Deployer`, `GPIO`, `I2CBus`, `MotorDriver`, `ServoDriver`, `SerialPort`, `SPIBus`

**`board.py`** — `Board` class:
- Fixed bug in `info()` where `result["platform"]` was incorrectly set to `self.definition` instead of `self.platform`
- Added `reset()` method to clear detection state for re-detection
- Added `__repr__()` with readable state

**`boards.py`** — Board database + firmware uploader:
- Added `__repr__()` on `FirmwareUploader` class
- `get_board()`, `list_boards()`, `list_platforms()` now have proper return-type annotations and docstrings
- Board database already covers 18 boards across 9 platforms — unchanged

**`deploy.py`** — `Deployer` class:
- Added context manager support (`__enter__` / `__exit__`) for `with` statement usage
- Added validation: `host` must not be empty (raises `ValueError`)
- Added `__repr__()` with host/user/port info
- `upload()` now returns `None` and prints a message when not connected

**`gpio.py`** — `GPIO` class:
- Backend detection extracted to `@staticmethod` (was inline in `__init__`)
- Added `__repr__()` showing backend and active pin count
- Added return-type annotations
- `cleanup()` now clears internal pin state dictionaries

**`i2c.py`** — `I2CBus` class:
- Added context manager support (`__enter__` / `__exit__`)
- Added input validation: `bus_number >= 0`
- Added `__repr__()` with bus number and open status
- All methods now have proper docstrings and type annotations
- `close()` sets `_bus = None` for clean re-open

**`motor.py`** — `MotorDriver` and `ServoDriver`:
- Added `__repr__()` on both classes
- Added `ServoDriver.reset()` to return servo to center (90°)
- Added type annotations on all methods
- `set_speed()` and `set_angle()` docstrings now document clamping behavior

**`serial_comm.py`** — `SerialPort` class:
- Added context manager support (`__enter__` / `__exit__`)
- Added input validation: `baudrate > 0`, `timeout >= 0`
- Added `__repr__()` with port, baudrate, and open status
- `close()` sets `_serial = None` for clean re-open

**`spi.py`** — `SPIBus` class:
- Added context manager support (`__enter__` / `__exit__`)
- Added input validation: `bus >= 0`, `device >= 0`, `speed > 0`, `mode` in 0–3
- Added `__repr__()` with bus/device/speed/mode and open status
- All methods now have docstrings and type annotations

### `stacc.gripper` — Full module overhaul

**`base.py`** — `Gripper` base class refactored:
- Fixed bare-string docstring to a proper triple-quoted docstring
- Added `reset()` method to reset gripper to fully open / zero-force state
- Replaced fragile `str.replace("ing", "ed")` in `stop()` with explicit state checks
- Added `__repr__()` for readable state inspection
- Added return type annotations (`-> None`) to all public methods
- Improved docstrings on all methods

**`parallel.py`** — `ParallelGripper` polished:
- Fixed mangled docstring formatting (capitalization, spacing)
- Added input validation (`min_width >= 0`, `max_width > min_width`)
- `close()` now accepts optional `width` parameter to stop before full closure (enables object grasp detection)
- Added `__repr__` via base class inheritance

**`dexterous.py`** — `DexterousHand` expanded:
- Added missing `from stacc.gripper.base import Gripper` import
- Added configurable `curl_power_grasp` and `curl_pinch_grasp` parameters (were hardcoded)
- Added `reset()` method that resets finger angles and calls `super().reset()`
- `set_finger()` now raises `IndexError` instead of silently ignoring out-of-range indices
- Added input validation (`num_fingers >= 1`, `joints_per_finger >= 1`)

**`magnetic.py`** — `MagneticGripper` cleaned:
- Added missing `Gripper` import
- Added `release()` convenience method (delegates to `open()`)
- Added input validation (`max_force > 0`, `diameter > 0`)

**`soft.py`** — `SoftGripper` polished:
- Added missing `Gripper` import
- Added `reset()` method that clears curl angles
- Added input validation (`num_fingers >= 1`, `compliance` in `[0, 1]`)

**`vacuum.py`** — `VacuumGripper` refined:
- Added missing `Gripper` import
- Extracted vacuum-to-force calculation into `holding_force` property with formula documentation
- Added input validation (`num_cups >= 1`, `cup_diameter > 0`, `seal_threshold` in valid range)

## 2026-11-14 — Hardware/control boundary corrections (unreleased)

- Real GPIO requires an implemented RPi/Jetson provider; missing/gpiod-only environments fail explicitly. Simulation requires explicit selection.
- SocketCAN errors no longer silently select virtual transport. Reject oversized frames and unsupported physical CAN-FD. The invalid generic Unitree hybrid encoder is disabled pending a verified model-specific protocol.
- Replace S-curve stepping with analytic rest-to-rest seven-segment motion. Validate limits; reject active-motion retargeting (finish/reset first).
- Firmware upload uses literal argv, rejects unsupported/manual providers, requires explicit ESP flash address and uses exact normalized board lookup.
- SSH uses trusted host keys and rejects unknown identities; optional `known_hosts` supports enrolled host files.
- Declare NumPy core dependency plus HAL/test extras. Clean wheel imports and control-boundary regression tests verified; no physical support certification implied.
