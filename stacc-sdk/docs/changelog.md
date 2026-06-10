# Changelog

## 2026-06-10 — gait module overhaul + control/ai improvements

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
