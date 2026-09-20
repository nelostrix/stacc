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
| `body` / `chassis` | The robot's root `Link`. `shape: differential-drive` (or `ackermann`/`mecanum`/`omni`) sets `robot.fixed_base = False`; anything else is treated as a fixed-base robot. |
| `arm` / `leg` / `wheel` / `gripper` / `head` (directly under the robot, or nested one level inside `actuators { }`) | A child `Link` + `Joint`, attached to the root. `wheel` gets a `continuous` joint; everything else `revolute`. |
| ...with `joints: N` (N > 1) | An **N-segment serial chain** instead of a single joint — `arm left { joints: 6; reach: 900mm; }` produces `arm_left_1 … arm_left_6`, each its own revolute joint, `reach` split evenly across segments. Alternates Y/Z rotation axes down the chain. |
| `sensors { ... }` | Each child becomes a sensor via `Robot.attach_sensor`. `lidar`/`camera`/`imu` kinds are wired to the **real** `stacc.sensors` drivers (so they produce actual MuJoCo-backed data in `stacc rss run --sim`); any other kind falls back to a generic, inert `SensorSpec` so nothing is silently dropped. |
| `appearance` / `style` | Real CSS-like selectors + cascade — see "Styling: selectors, variables, cascade" below. |
| `component "name" { ... }` + `use name;` | The component's own properties become **defaults** (the using block's own properties win on conflict); its child blocks are prepended. Applied recursively at every nesting depth, with cycle detection. |
| `use "path/to/file.rss:name";` | Cross-file component reuse — resolves `path/to/file.rss` against the compiling file's own directory, then any `--rss-path` directories, and pulls `name` from that file's `component` defs. This is the actual "shared stylesheet" behavior, not just per-file defaults. |
| Anything else at the top level (e.g. a `navigation { ... }` block) | Not a recognized structural block, so nothing is built from it — but it's **never silently dropped**: its properties land in `robot.rss_metadata[block_name]`, and a warning is emitted (`stacc rss check`/`build` print it; also available as `robot.rss_warnings`). |
| Any property that doesn't map onto a `Link`/`Joint`/`Geometry` field (`payload`, `actuator`, `motor`, sensor `model:` strings, ...) | Preserved verbatim in `robot.rss_metadata[link_name]`, unit-normalized, so it's inspectable even though nothing consumes it structurally yet. |

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

- A multi-joint limb (`joints: N`) is a straight serial chain with a fixed
  Y/Z axis alternation — it doesn't yet support explicit per-joint axis,
  limits, or non-serial (e.g. differential/parallel) topologies.
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
