# Changelog

## 2026-06-10 — control + ai module improvements

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
