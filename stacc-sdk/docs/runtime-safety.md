# Live runtime safety and timing

`StaccRuntime` is a software simulation/control runtime. Its emergency stop,
command watchdog, and actuator limits improve the behavior of this process;
they are not safety-rated controls and do not replace a physical E-stop,
drive disable, independent watchdog, or a risk assessment for a real robot.

## Control timing

The runtime uses a monotonic absolute schedule at the configured `hz` and
records deadline misses from that schedule. Physics time is advanced in fixed
`timestep` increments. When one control period is not an integer number of
physics steps, the runtime distributes the remainder across later periods. It
skips missed wall-clock slots instead of trying to catch up with an unbounded
burst. `snapshot()["loop"]` reports measured rate, jitter, step duration, and
deadline misses. These are measurements; they do not guarantee a hard real-time
deadline on a general-purpose OS.

## Commands and watchdog

Position and torque commands must have the exact joint-vector length; rotor
commands must have the exact rotor count. All values must be finite numeric
values. Positions are clamped to joint limits, torques to each
joint's effort limit, and rotor thrusts to each rotor's range. PID gains must
be finite and non-negative.

An optional timeout enables a command watchdog:

```python
runtime = StaccRuntime(
    physics="mujoco", hz=500, timestep=0.001, command_timeout=0.25
)
```

The server CLI accepts the same setting with
`stacc serve --command-timeout 0.25`. The STACC daemon embedded in NELO Studio
reads `NELO_STACC_COMMAND_TIMEOUT` when configured for a command lease.

When no fresh actuator command arrives within the timeout, STACC clears the
position servo and commands zero torque and thrust. The timeout is disabled by
default because some simulation and teleoperation workflows intentionally hold
the last target. Set a timeout explicitly for a controller that requires a
heartbeat, and validate its zero-output behavior against the actual drive.

## Emergency stop and recovery

```python
runtime.emergency_stop("operator request")
# inspect runtime.snapshot()["safety"] and address the cause
runtime.clear_emergency_stop()
```

The stop latch prevents new actuator commands and prevents `start()` or
`resume()` until explicitly cleared. Clearing the latch does not restore old
commands. `stop()` also clears servo, torque, and thrust commands. Runtime
exceptions stop the loop, clear commands, and are exposed as
`snapshot()["safety"]["runtime_fault"]`; call `reset()` or load a new robot
before restarting after a fault. Sensor read errors are retained in
`snapshot()["safety"]["sensor_errors"]` while the physics loop continues.
Shared-memory commands buffered while stopped or E-stopped are discarded;
controllers should read the new state and send a fresh command after start or
explicit recovery.

HTTP controls are available at `POST /api/stacc/emergency-stop` and
`POST /api/stacc/emergency-stop/clear` on the stdlib bridge, and
`POST /api/safety/emergency-stop` and `/api/safety/emergency-stop/clear` on the
FastAPI bridge.

## Deployment boundary

The software stop only changes the state and outputs controlled by the STACC
runtime. It cannot stop power-stage motion if the process, OS, network, driver,
or motor controller has failed. Real deployments still need a hardware E-stop,
drive-level limits, an independent communication-loss failsafe, and hardware
validation before motion.
