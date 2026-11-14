# Real-time control over shared memory

A controller talks to the STACC runtime on two planes:

- **Control plane — HTTP** (`/v1/runtime/*`): load a robot, start/stop, set
  gains, enable the channel. Occasional, human-speed calls.
- **Data plane — shared memory** (`/dev/shm/stacc_rt_<port>`): the state of
  every control cycle out, a command back in. No sockets, no JSON, no
  allocation per step.

```python
from stacc.server.runtime_client import RuntimeClient

rt = RuntimeClient()
rt.load_file(__file__)                 # a .rss file, or this file's <<<RSS>>> block
ch = rt.enable_shm("lockstep")         # -> stacc.server.rt_channel.ControlChannel
s = ch.wait_state(after_step=-1)
while True:
    ch.command_positions(my_controller(s), answers_step=s.step)
    s = ch.wait_state(after_step=s.step)
```

C++: `stacc::ControlChannel ch(rt.enable_shm("lockstep"));` in
`bindings/cpp/stacc_runtime.hpp`. Rust: `rt.enable_shm("lockstep", 1000.0)?` in
`bindings/rust/stacc_runtime`. Full examples:
`examples/embedded/arm_shm.cpp`, `bindings/rust/stacc_runtime/examples/arm_shm.rs`.

## Modes

| Mode | The runtime… | Use it for |
|---|---|---|
| `realtime` | steps on its own clock and applies your newest command every cycle; never waits. Commands computed from an older state are counted as `late_commands`. | running against the real-time loop, like hardware |
| `lockstep` | publishes step *k*, then waits until a command answering step *k* arrives (up to `timeout_ms`, else counts a `missed_deadline` and continues). | deterministic runs: the same controller produces the same trajectory, bit for bit |

Lockstep with no controller attached stalls the simulation (each cycle waits
for the timeout) — disable the channel or switch to `realtime` when you're
done. `GET /v1/runtime/shm` reports the mode, layout, joint and rotor order
and the counters; `/v1/system/status` shows `shm_status` (e.g.
`lockstep /dev/shm/stacc_rt_8766`).

The runtime uses monotonic absolute deadlines and reports measured deadline
misses. Fixed physics steps are distributed across control periods, so 60 Hz
does not accumulate a 17 ms physics period with a 1 ms timestep. General-
purpose Linux and Python remain soft real time; these measurements do not
guarantee hard deadlines. See [`runtime-safety.md`](runtime-safety.md) for
actuator limits, watchdog setup, software E-stop behavior, and deployment
safety boundaries.

```
POST /v1/runtime/shm  {"enable": true, "mode": "lockstep", "timeout_ms": 1000}
POST /v1/runtime/shm  {"enable": false}
GET  /v1/runtime/shm
```

## Layout (version 1, little-endian, 8-byte aligned)

```
header   (64 B)  magic u32 'STRT' (0x54525453), version u32, n_joints u32,
                 n_thrusters u32, mode u32 (0 realtime, 1 lockstep), reserved u32,
                 state_offset u64, command_offset u64, reserved 16 B

state    seq u64, step u64, sim_time f64,
         pose f64[7]                 x, y, z, qw, qx, qy, qz (base, world)
         position f64[n_joints]      rad (m for prismatic joints)
         velocity f64[n_joints]      rad/s
         torque   f64[n_joints]      N·m actually applied
         thrust   f64[n_thrusters]   N

command  seq u64, cmd_id u64, answers_step u64, kind u32, flags u32,
         values f64[n_joints]        kind 1: target positions (rad); kind 2: torques (N·m)
         thrust f64[n_thrusters]     N, applied when flags & 1
```

Joint order is `GET /v1/runtime/shm`'s `joint_names` (it includes fixed
joints such as rotor mounts, whose values are ignored); rotor order is its
`thrusters`. The channel is rebuilt when a different robot is loaded.

Each block is a **seqlock** with exactly one writer (the runtime writes
`state`, the controller writes `command`): the writer makes `seq` odd, writes
the payload, then makes it even; a reader copies the payload between two
reads of `seq` and retries if either was odd or they differ. Neither side
ever blocks the other, and a reader never sees a half-written frame. The C++
binding uses acquire/release atomics on `seq` (correct on weakly-ordered
CPUs); Python's mmap stores rely on x86-64 total store order; the std-only
Rust binding uses positioned reads/writes on the tmpfs file, which are
system calls and therefore ordered.

## Measured (this machine, Python runtime at a 1000 Hz target)

| Controller | Closed-loop steps/s | Round trip median / p99 |
|---|---|---|
| C++ (`arm_shm.cpp`), lockstep | 938 | 1.06 ms / 1.23 ms |
| Rust (`arm_shm.rs`), lockstep | 938 | 1.06 ms / 1.26 ms |
| Python, separate process, lockstep | 937 | 1.06 ms / 1.27 ms |

All three languages hit the same rate as the runtime's own loop measured
with no controller attached (~940 Hz), so the limit is the Python physics
loop's pacing, not the channel or the controller's language.
A controller thread inside the runtime's own Python process is much slower
(both sides spin on the GIL) — run controllers as separate processes.
