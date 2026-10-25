# RSS — a declarative robot description language

RSS ("Robot Style Sheets") is a small, CSS-flavored language for describing
robots — geometry, limbs, sensors, and appearance — that compiles down to
real `stacc` robots and standard robotics formats (URDF, SDF, MJCF, a ROS 2
package skeleton).

```
robot warehouse_bot {
    chassis {
        shape: differential-drive;
        dimensions: [600mm, 450mm, 250mm];
    }

    sensors {
        lidar front { model: rplidar-a2; range: 12m; fov: 360deg; }
        camera front { model: realsense-d455; }
    }

    actuators {
        wheel left { motor: "BLDC-01"; radius: 120mm; }
        wheel right { motor: "BLDC-01"; radius: 120mm; }
    }
}
```

```bash
stacc rss check my_robot.rss              # parse + compile, report only
stacc rss build my_robot.rss              # -> build/urdf, build/gazebo, build/ros2
stacc rss build my_robot.rss --format sdf # a single format
stacc rss build my_robot.rss --format stl # -> build/parts/*.stl, for 3D printing
stacc rss build my_robot.rss --format bom # -> build/bom.csv + build/bom.md
stacc rss run my_robot.rss --duration 10  # compile + simulate (headless by default)
stacc rss deploy my_robot.rss 192.168.1.50 --user pi  # package + real SSH deploy
```

The implementation lives in `stacc/rss/`: `lexer.py` → `parser.py` →
`compiler.py` produce a real `stacc.core.robot_base.Robot`, which existing
stacc code (URDF/SDF exporters, the MuJoCo simulator, the real sensor
drivers in `stacc.sensors`) then does the actual work with. RSS itself adds
no physics or rendering of its own.

## Grammar

```
document      := statement*
statement     := component_def | robot_def
component_def := 'component' name '{' block_body '}'
robot_def     := 'robot' name '{' block_body '}'
name          := STRING | IDENT

block_body    := block_item*
block_item    := use_directive | property | nested_block
use_directive := 'use' name ';'
property      := IDENT ':' value ';'
nested_block  := IDENT [IDENT] '{' block_body '}'   # 2nd IDENT is an optional label
value         := STRING | NUMBER | IDENT | array
array         := '[' (value (',' value)*)? ']'
```

- `// comment` to end of line.
- Strings: `"..."` (supports `\"`, `\\`, `\n`, `\t`).
- Numbers: an optional unit suffix with no space — `650mm`, `12m`, `360deg`,
  `1.5m/s`, `400hz`, or a bare number (`6`, `10`). Length (`mm`/`cm`/`m`),
  mass (`g`/`kg`), and angle (`deg`/`rad`) units are normalized to SI
  (meters/kg/radians) at compile time; `m/s`, `deg/s`, `rad/s`, `hz`, `khz`
  pass through unchanged.
- Idents (bare words like `humanoid`, `matte`, `differential-drive`) may
  contain letters, digits, `_` and `-`.
- Arrays: `[600mm, 450mm, 250mm]`.
- A block can carry an optional label — `arm left { ... }`, `lidar front
  { ... }` — used to disambiguate multiple instances of the same block type.

## Semantic mapping (RSS → `Robot`)

| RSS block | Becomes |
|---|---|
| `body` / `chassis` | The robot's root `Link`. A robot with legs, wheels or rotors, or a `differential-drive`/`ackermann`/`mecanum`/`omni` chassis, is free to move (`robot.fixed_base = False`); anything else is fixed to the floor. `base: fixed;` / `base: free;` overrides that. |
| `arm` / `leg` / `wheel` / `gripper` / `head` / `rotor` (directly under the robot, or nested one level inside `actuators { }`) | A child `Link` + `Joint`, mounted on the body — see "Where limbs go" below. `wheel` gets a `continuous` joint, `rotor` a fixed joint plus a thrust actuator, everything else `revolute`. |
| ...with `joints: N` (N > 1) | An **N-segment serial chain** instead of a single joint — `arm left { joints: 6; reach: 900mm; }` produces `arm_left_1 … arm_left_6`, each its own revolute joint, `reach` split evenly across segments. |
| `sensors { ... }` | Each child becomes a sensor via `Robot.attach_sensor`. `lidar`/`camera`/`imu` kinds are wired to the **real** `stacc.sensors` drivers (so they produce actual MuJoCo-backed data in `stacc rss run --sim`); any other kind falls back to a generic, inert `SensorSpec` so nothing is silently dropped. |
| `appearance` / `style` | Real CSS-like selectors + cascade — see "Styling: selectors, variables, cascade" below. |
| `component "name" { ... }` + `use name;` | The component's own properties become **defaults** (the using block's own properties win on conflict); its child blocks are prepended. Applied recursively at every nesting depth, with cycle detection. |
| `use "path/to/file.rss:name";` | Cross-file component reuse — resolves `path/to/file.rss` against the compiling file's own directory, then any `--rss-path` directories, and pulls `name` from that file's `component` defs. This is the actual "shared stylesheet" behavior, not just per-file defaults. |
| Anything else at the top level (e.g. a `navigation { ... }` block) | Not a recognized structural block, so nothing is built from it — but it's **never silently dropped**: its properties land in `robot.rss_metadata[block_name]`, and a warning is emitted (`stacc rss check`/`build` print it; also available as `robot.rss_warnings`). |
| Any property that doesn't map onto a `Link`/`Joint`/`Geometry` field (`payload`, `actuator`, `motor`, sensor `model:` strings, ...) | Preserved verbatim in `robot.rss_metadata[link_name]`, unit-normalized, so it's inspectable even though nothing consumes it structurally yet. |

## Where limbs go

Each limb's first joint sits on the body's surface, chosen from its label,
and each segment hangs **off** its joint (so joints pivot at the end of a
segment, the way real hardware does). +X is forward, +Y is left (ROS REP 103).

| Limb | Mounted | Grows |
|---|---|---|
| `leg` | under the body, at the corner its label names (`front_left`, `rear_right`, `fl`, `rr`, …; unlabeled legs fill corners in order) | down |
| `wheel` | on the side its label names, axle along Y, at the front/rear its label names | — |
| `rotor` | on top, at a corner | — |
| `arm` / `head` | on top, offset by `left`/`right`/`front`/`rear` in the label | up |
| `gripper` | at the tip of the arm with the same label (or the first arm) | up |

- `at: [x, y, z];` places the first joint explicitly (metres from the body's centre).
- `axes: [z, y, y];` (or `axis: z;` for a one-joint limb) sets each joint's rotation axis.
  Defaults: legs `x, y, y` (hip abduction, hip flexion, knee), arms
  `z, y, y, x, y, x` (base yaw, shoulder, elbow, wrist roll/pitch/roll), heads `z, y`.
- Fixed robots stand on the floor; mobile robots spawn just above it.
- A body resting only on two centre-line wheels tips over; the compiler warns.

## Joints and motors

| Property | Meaning |
|---|---|
| `effort: 2.5Nm;` | Motor stall torque — the most the joint can apply (MuJoCo `forcerange`). |
| `speed: 300rpm;` (also `deg/s`, `rad/s`; alias `velocity:`) | Motor no-load speed. |
| `damping: 0.1;` | Viscous joint damping, N·m·s/rad. |
| `friction: 0.02;` | Dry (Coulomb) joint friction, N·m (MuJoCo `frictionloss`). |
| `limits: [-90deg, 45deg];` | Range of a revolute joint. |

With both `effort` and `speed`, the joint behaves like a DC motor: available
torque falls linearly with speed, τ = τ_cmd − (effort / speed)·ω, so a wheel
settles at a top speed instead of accelerating forever. (`speed` without
`effort` is recorded and warned about.) Joints are servoed by MuJoCo itself —
a position actuator integrated implicitly, stable at any gains — and the
torque reported is what was actually applied.

## Rotors

`rotor front_left { thrust: 6N; radius: 64mm; }` is a thrust actuator on top
of the body: force along the rotor's axis, up to `thrust:` newtons, plus a yaw
reaction torque of `torque_ratio:` (default 0.016 m) per newton, opposite to
its `spin:` (`cw`/`ccw`; default is the X-quad pattern — front-left and
rear-right counter-clockwise). Command it through the runtime API with
`{"thrusts": [N, ...]}` (`command_thrusts` in the Python/C++/Rust clients), in
the order listed by telemetry's `thrusters`.

## RSS inside Python, C++ and Rust

A program can carry its robot's description between `<<<RSS>>>` and
`<<</RSS>>>`, somewhere its own language ignores:

```python
from stacc.rss import rss
ROBOT = rss("""<<<RSS>>>
robot scout { chassis { shape: differential-drive; } wheel front_left { radius: 60mm; } }
<<</RSS>>>""")
```

```cpp
/* <<<RSS>>>
robot arm { body { shape: box; } arm main { joints: 3; reach: 600mm; } }
<<</RSS>>> */
```

Lines may also share a comment leader (`# `, `// `, ` * `), which is
stripped. A file may hold several blocks (e.g. a component library and a
robot). Errors are reported at the line and column of the host file.

- Python: `rss(text)` compiles an inline description; `this_file()` compiles
  the calling file's blocks; `load_file(path)` compiles any `.rss` or host file.
- `stacc rss check|build|run|deploy` accept `.py`/`.cpp`/`.rs` files directly.
- The daemon's `/v1/rss/compile`, `/v1/rss/load` and `/v1/runtime/load_project`
  accept a whole host file; `RuntimeClient.load_file(__file__)` (Python),
  `stacc::Runtime::load_file(__FILE__)` (C++, `bindings/cpp/stacc_runtime.hpp`)
  and `Runtime::load_file(file!())` (Rust, `bindings/rust/stacc_runtime`) send it.
- Examples: `examples/embedded/scout_rover.py`, `examples/embedded/arm_reach.cpp`,
  `bindings/rust/stacc_runtime/examples/quadruped_crouch.rs`.

## Styling: selectors, variables, cascade

`appearance` / `style` blocks (the two names are aliases) support real
CSS-like selectors, `--custom-properties`, and a cascade — not just
per-link flat colors keyed by exact name.

```
robot demo {
    body { shape: box; }
    arm left_arm { joints: 3; class: primary; }
    arm right_arm { joints: 3; }

    sensors {
        camera front { model: realsense-d455; }
    }

    style {
        --primary: "#151515";
        --accent: "#00ffaa";

        body { color: var(--primary); }        // element type -> root link
        arm  { color: "#333333"; }              // element type -> every arm segment
        #left_arm { color: "red"; }              // id (a block's label) -> that whole limb
        .primary { emission: "yellow"; }         // class (declared via `class: primary;`)
        sensor[type="camera"] { color: "black"; } // attribute selector on sensor kind
        joint { material: "steel"; }             // element type -> every joint
    }
}
```

- **Element type selectors** — a bare `chassis`/`body`/`arm`/`leg`/`wheel`/
  `gripper`/`head`/`joint`/`sensor` matches every compiled element of that
  type (every segment of every multi-joint chain, every joint, every
  sensor).
- **`#id` selectors** — the id is a limb's own **label** (`arm left_arm { }`
  → `#left_arm`). Matches every link generated from that labeled block
  (all segments of a multi-joint chain share one id), so you can restyle
  one specific limb without touching the others.
- **`.class` selectors** — declare `class: name;` (or `classes: [a, b];`
  for more than one) on any structural block or sensor; `.name { ... }`
  then matches everything tagged with it.
- **Attribute selectors** — `sensor[type="camera"]` / `sensor[type="lidar"]`
  / `sensor[type="imu"]` match sensors by kind; any other
  `sensor[key="value"]` matches against that sensor's own metadata
  properties.
- **`--custom-properties` and `var()`** — declare `--name: value;` directly
  inside a `style`/`appearance` block (order matters: a `--x` can reference
  an earlier `--y`, not a later one), then use `var(--name)` anywhere a
  value is expected. Referencing an undeclared property is a real compile
  error (`RSSCompileError`), not a silent no-op.
- **Named colors** — `color: "red";`, `"black"`, `"white"`, and a small set
  of other CSS-style names work alongside hex strings (`"#RRGGBB"` /
  `"#RRGGBBAA"`); both must still be quoted like every other RSS string.
- **Cascade** — when more than one rule matches the same target, the
  highest-specificity rule wins (`#id` > `.class`/`attr[...]` > bare
  element type), and among equal-specificity rules the one written later
  wins — real CSS semantics, not "last rule in the file always wins."
- **Backward compatible** — the original addressing mode still works
  exactly as before: a bare child block whose name is either the root
  block's own name (`chassis`/`body`) or an exact compiled link name
  (e.g. spelling out `arm_left` directly) targets that one link, at the
  same specificity as an `#id` selector.
- **Real material properties** — `material: <name>;` applies a named preset
  (`aluminum`, `steel`, `brushed_steel`, `brass`, `copper`, `carbon_fiber`,
  `plastic`, `rubber`, `wood`, `glass`, `brushless`, `servo`) that sets real
  color + `metallic`/`roughness`; an explicit `color:`/`metallic:`/
  `roughness:` on the same block still wins for that specific field.
  `emission: "#RRGGBB";` and `transparency: 0..1;` (or `opacity: 0..1;`,
  its inverse) are real too. All of it is applied to `Link.visual`/
  `Link.collision` and flows into real output: MJCF gets a real
  `<asset><material>` (referenced by the geom instead of a placeholder
  rainbow color), SDF/Gazebo gets a real `<material><pbr><metal>` block.
  URDF has no material model beyond flat color at the format level, so it
  honestly only carries `color:` there — not a stacc limitation, a URDF one.
  Anything targeting a `sensor`/`joint` (which have no `Geometry` to color)
  is still captured faithfully in `.properties`/`robot.rss_metadata`.

## From idea to physical robot

`stacc rss build`/`deploy` cover the rest of the pipeline beyond simulation —
each one does real, honest work, with an explicit boundary where "honest"
means "tells you when it can't do something" rather than faking it:

- **`--format stl`** (`stacc/rss/mesh_export.py`) — one real, watertight STL
  per link (via `trimesh`, sized to the link's actual dimensions) plus a
  best-effort merged `<robot>_assembly.stl` for a print preview, written to
  `build/parts/`. **Limitation**: RSS's `Geometry` only models box/cylinder/
  sphere/mesh primitives — no fillets, chamfers, or CSG (holes, unions,
  subtractions). Real parametric CAD features like that need an actual
  B-Rep solid-modeling kernel (OpenCASCADE), which isn't wired in here; a
  link with an unrecognized/missing mesh is skipped with a warning, never
  given fake geometry.
- **`--format bom`** (`stacc/rss/bom.py`) — a real procurement checklist
  (`build/bom.csv`/`build/bom.md`) listing every link (as a part to print),
  every actuated joint (with its `actuator`/`motor` value from the `.rss`
  file), and every sensor (with its `model:` value). **Limitation**: names
  are exactly what you wrote in the `.rss` file — not resolved to a real
  distributor SKU, price, or datasheet (no parts database or internet
  lookup is available here). It's a shopping list, not a quote.
- **`stacc rss deploy <file.rss> <ip>`** — compiles the robot, writes a URDF
  + a **template** `src/robot.py` control script (one `MotorDriver`/`GPIO`
  stub per actuated joint, named from the joint), then uses the real SSH/
  SFTP client in `stacc.hal.deploy.Deployer` to connect to `<ip>` and upload
  it. A failed connection is reported as a failure and stops there — it
  never prints a fake "deployed successfully". **Limitation**: the control
  script's GPIO pins are always `None  # TODO` — physical wiring is
  something only you know, so it's generated as a template to fill in, not
  an autonomous guess. `stacc build --target=arduino/esp32/stm32` is
  refused outright with the same honesty: stacc has no C/C++ toolchain and
  generates no microcontroller firmware; use `stacc.hal.boards
  .FirmwareUploader` to flash a firmware image you already have.

The HAL layer these commands build on (`stacc/hal/gpio.py`, `motor.py`,
`serial_comm.py`, `can.py`, `i2c.py`, `spi.py`) does genuine hardware I/O on
Linux/Raspberry Pi/Jetson (falling back to an in-memory simulation only when
the actual library/hardware isn't present) — it isn't RSS-specific, RSS just
generates a starting point that targets it.

## Known simplifications (by design, for this version)

- A multi-joint limb (`joints: N`) is a straight serial chain. Axes, limits
  and motor properties apply to every joint of the limb alike; non-serial
  (closed-loop, differential, parallel) topologies aren't supported.
- Sensor `model:` strings (`rplidar-a2`, `realsense-d455`, ...) aren't
  matched to specific hardware profiles — only the generic sensor *kind*
  (`lidar`/`camera`/`imu`) selects a real driver; the model name is kept as
  metadata for now.
- `color:`/`material:`/`metallic:`/`roughness:`/`emission:`/`transparency:`
  all have real effects now (see "Styling" above), but still per whole
  link, not per-geometry — no texture maps/UVs, no decals, and URDF export
  only ever carries flat color (a real limit of the URDF format, not of
  stacc).
- Cross-file `use` resolves one file deep per reference (a library file can
  itself `use` other libraries — that's supported — but there's no package
  manager or versioning around it, just relative paths / `--rss-path`).
- No parametric expressions (`radius: radius * 2`), CSG, fillets/chamfers,
  constraints, or STEP export — see "From idea to physical robot" above.
  These need a real CAD kernel and are a separate, much larger effort.

## Reference examples

See `examples/*.rss`:

- `warehouse_bot.rss` — wheeled mobile robot, sensors, `actuators { }`.
- `arm_6dof.rss` — a 6-joint serial arm via `joints: 6`.
- `quadruped.rss` — four 3-joint legs (12 DOF total).
- `drone.rss` — a quadcopter with IMU/GPS/camera.
- `factorybot.rss` / `lib_industrial_arm.rss` / `factorybot_shared_lib.rss` —
  same-file vs. cross-file `component`/`use` reuse, side by side.
- `styled_quadruped.rss` — the same shape as `quadruped.rss`, demonstrating
  `--variables`, element/`#id`/`.class`/`attr[...]` selectors, and cascade.
