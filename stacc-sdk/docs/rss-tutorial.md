# RSS Tutorial — A Complete Guide to Robot Style Sheets

RSS ("Robot Style Sheets") is a small, CSS-flavored language for describing
robots — geometry, limbs, sensors, appearance — that compiles down to a real
`stacc` `Robot` object and from there to standard robotics formats (URDF,
SDF, MJCF, STL, a bill of materials, a ROS 2 package). This guide walks
through everything it currently does, from your first one-block robot to
real MuJoCo simulation and physical deployment.

> Looking for the terse language reference instead of a tutorial? See
> [`rss.md`](./rss.md) in this same folder. This document teaches; that one
> is the spec.

---

## 1. What RSS actually is

Think of it like CSS for robots: nested `block { property: value; }` syntax,
`//` comments, reusable `component`s you `use`, and a compiler that turns
your document into something real. RSS itself has **no physics or rendering
of its own** — it produces a `stacc.core.robot_base.Robot`, and existing
stacc code (URDF/SDF exporters, the real MuJoCo simulator, real sensor
drivers) does the actual work with it.

The pipeline, end to end:

```
your_robot.rss
   │  lexer.py  → tokens
   │  parser.py → an AST (Document)
   │  compiler.py → a real Robot (links, joints, sensors)
   ▼
Robot ──────────────┬─────────────┬──────────────┬─────────────┐
                     ▼             ▼              ▼             ▼
                  URDF/SDF/MJCF  STL parts     BOM (CSV/MD)  ROS 2 package
                     │
                     ▼
              real MuJoCo simulation (`stacc rss run --sim`)
```

---

## 2. Quick start — your first robot in 60 seconds

Create `my_robot.rss`:

```
robot my_robot {
    chassis {
        shape: differential-drive;
        dimensions: [600mm, 450mm, 250mm];
    }
}
```

Check it compiles:

```bash
stacc rss check my_robot.rss
```

You'll see something like:

```
🤖 my_robot — RSS check passed
```

That's it — you've described a robot. Everything from here is about adding
more to that one `robot { }` block.

---

## 3. The CLI at a glance

Every RSS workflow goes through `stacc rss <subcommand>`:

| Command | What it does |
|---|---|
| `stacc rss check <file>` | Parse + compile, report only. Fastest way to validate a file. |
| `stacc rss build <file>` | Compile to URDF/SDF/MJCF/STL/BOM/ROS 2 — see §6. |
| `stacc rss run <file>` | Compile and simulate (headless by default). |
| `stacc rss deploy <file> <ip>` | Compile, package, and SSH it to real hardware. |

All four accept `--robot <name>` (when a file defines more than one `robot`)
and `--rss-path <dir>` (repeatable — extra directories to search for
cross-file `use "lib.rss:name";` references; the file's own directory is
always searched).

---

## 4. Language tour

### 4.1 The shape of a document

```
document      := statement*
statement     := component_def | robot_def
component_def := 'component' name '{' block_body '}'
robot_def     := 'robot' name '{' block_body '}'
name          := STRING | IDENT
```

A file can contain multiple `robot { }` blocks and/or reusable
`component { }` blocks. If a file defines more than one robot, `--robot
<name>` picks which one to compile.

### 4.2 Comments, strings, numbers, arrays

```
// a comment, to end of line

robot demo {
    chassis {
        shape: box;                    // bare identifier
        dimensions: [600mm, 450mm, 250mm]; // array of numbers with units
        color: "#111111";              // string (also used for names/paths)
        max_speed: 1.5m/s;             // unit passes through unchanged
        angle_limit: 180deg;           // normalized to radians at compile time
    }
}
```

- **Idents** (`box`, `differential-drive`, `matte`) may contain letters,
  digits, `_`, and `-`.
- **Numbers** take an optional unit suffix with *no space*: `650mm`, `12m`,
  `360deg`, `1.5m/s`, `400hz`, or a bare number (`6`, `10`).
  - **Length** (`mm`, `cm`, `m`), **mass** (`g`, `kg`), and **angle** (`deg`,
    `rad`) units are normalized to SI (meters / kg / radians) at compile
    time.
  - `m/s`, `deg/s`, `rad/s`, `hz`, `khz` pass through unchanged — they're
    kept as metadata (see §4.7), not consumed structurally yet.
- **Strings** support `\"`, `\\`, `\n`, `\t`.
- A block can carry an **optional label** to disambiguate multiple
  instances of the same block type: `arm left { ... }`, `lidar front
  { ... }`, `wheel left { ... }`.

### 4.3 The body — `body` or `chassis`

Every robot needs exactly one root block, named either `body` or `chassis`
— they mean the same thing; use whichever reads better for your robot
(`chassis` for wheeled bases, `body` for arms/legs/drones is just
convention, not enforced).

```
robot arm_6dof {
    body {
        shape: box;
        material: aluminum;
        color: "#1e293b";
    }
}
```

Setting `shape:` to a mobile-base shape — `differential-drive`,
`ackermann`, `mecanum`, `omni`/`omnidirectional` — marks the robot as
**mobile** (`robot.fixed_base = False`); anything else (`box`, `cylinder`,
`sphere`) is a **fixed-base** robot (an arm bolted to a table, for
example).

### 4.4 Limbs — `arm`, `leg`, `wheel`, `gripper`, `head`

Any of these, placed directly under the robot (or nested one level inside
`actuators { }`), becomes a child `Link` + `Joint` attached to the root:

```
robot simple_arm {
    body { shape: box; }

    arm main {
        actuator: servo;
        reach: 500mm;
    }
}
```

- `wheel` gets a **continuous** joint (spins freely — for driven wheels).
- Everything else (`arm`, `leg`, `gripper`, `head`) gets a **revolute**
  joint (a hinge, with limits).

#### Multi-segment chains — `joints: N`

Add `joints: N` (N > 1) to turn a single hinge into an **N-segment serial
chain** — this is how you build a real multi-DOF arm or leg:

```
robot arm_6dof {
    body { shape: box; material: aluminum; }

    arm main {
        joints: 6;
        actuator: servo;
        reach: 900mm;
    }

    gripper {
        type: parallel;
        fingers: 2;
    }
}
```

`arm main { joints: 6; reach: 900mm; }` produces six real, independently
articulated links — `arm_main_1` through `arm_main_6` — with the 900mm
reach split evenly across them, alternating Y/Z rotation axes down the
chain (see §9 for the exact limitation this implies).

A full quadruped is just four `leg`s, each its own 3-joint chain:

```
robot quadruped {
    body {
        shape: box;
        material: carbon_fiber;
        dimensions: [500mm, 250mm, 120mm];
        color: "#22252b";
    }

    leg front_left  { joints: 3; reach: 350mm; actuator: brushless; }
    leg front_right { joints: 3; reach: 350mm; actuator: brushless; }
    leg rear_left   { joints: 3; reach: 350mm; actuator: brushless; }
    leg rear_right  { joints: 3; reach: 350mm; actuator: brushless; }

    sensors {
        imu body { model: bmi088; }
        camera front { model: realsense-d455; }
    }
}
```

That compiles to a real 12-DOF robot (`stacc rss check` reports `dof: 12`,
13 links: `body` + 4×3 leg segments).

### 4.5 Sensors

A `sensors { }` block holds one or more typed, optionally-labeled children:

```
sensors {
    lidar front {
        model: rplidar-a2;
        range: 12m;
        fov: 360deg;
    }

    camera front {
        model: realsense-d455;
    }

    imu body {
        model: bmi088;
        rate: 400hz;
    }

    gps top {
        model: ublox-m9n;
    }
}
```

Each child becomes a sensor via `Robot.attach_sensor()`. `lidar`, `camera`,
and `imu` kinds are wired to the **real** `stacc.sensors` drivers, so they
produce genuine MuJoCo-backed data when you `stacc rss run --sim` (§6.3).
Any other kind (like `gps` above) falls back to a generic, inert
`SensorSpec` so nothing is silently dropped — it just won't produce live
data yet. The `model:` string (`rplidar-a2`, `realsense-d455`, ...) is kept
as metadata; it currently only selects the generic sensor *kind*, not a
specific hardware profile (see §9).

### 4.6 Actuators

Wrap limbs in `actuators { }` if you'd rather group them explicitly — it's
equivalent to declaring them at the top level, just more organized for
robots with a lot of moving parts:

```
robot warehouse_bot {
    chassis { shape: differential-drive; dimensions: [600mm, 450mm, 250mm]; }

    actuators {
        wheel left  { motor: "BLDC-01"; radius: 120mm; }
        wheel right { motor: "BLDC-01"; radius: 120mm; }
    }
}
```

### 4.7 Anything else — metadata, never silently dropped

Put something RSS doesn't structurally know about — like a `navigation { }`
block — at the top level, and it isn't discarded:

```
navigation {
    algorithm: nav2;
    max_speed: 1.5m/s;
}
```

Its properties land in `robot.rss_metadata['navigation']`, unit-normalized,
and a warning is emitted (`stacc rss check`/`build` print it; also
available programmatically as `robot.rss_warnings`) — so a typo'd or
not-yet-supported block name is *visible*, not silently ignored. The same
applies to any property on a real block that doesn't map onto a `Link`/
`Joint`/`Geometry` field (`payload`, `actuator`, `motor`, sensor `model:`
strings, ...) — it's preserved verbatim in `robot.rss_metadata[link_name]`,
inspectable even though nothing consumes it structurally yet.

### 4.8 Appearance & styling

The simplest form — address one part by spelling out its exact name:

```
appearance {
    chassis {
        color: "#111111";
        finish: matte;
    }

    accent {
        color: "#00FFAA";
    }
}
```

(`style` works identically — it's just an alias.) A child block whose name
matches the root block's name (`chassis`/`body`) or an exact compiled link
name has its `color:` applied to that link's geometry. Other named entries
(like `accent` above, with no matching link) are kept as metadata only —
handy for documenting a brand accent color even though nothing renders it
yet.

#### Real CSS-like selectors, variables, and cascade

`style`/`appearance` also supports genuine CSS-style selectors on top of
that simple form — declare a color once as a custom property, then target
elements by type, by id, by class, or by attribute instead of spelling out
internal link names:

```
robot demo {
    body { shape: box; }

    arm left_arm  { joints: 3; class: primary; }
    arm right_arm { joints: 3; }

    sensors {
        camera front { model: realsense-d455; }
    }

    style {
        --primary: "#151515";
        --accent: "#00ffaa";

        body { color: var(--primary); }          // element type -> root link
        arm  { color: "#333333"; }                // element type -> every arm segment
        #left_arm { color: "red"; }                // id (a limb's own label)
        .primary { emission: "yellow"; }           // class, declared via `class: primary;`
        sensor[type="camera"] { color: "black"; }  // attribute selector, by sensor kind
        joint { material: "steel"; }               // element type -> every joint
    }
}
```

```bash
stacc rss check demo.rss
```

- **Element type selectors** (`body`, `arm`, `leg`, `wheel`, `gripper`,
  `head`, `joint`, `sensor`) match *every* compiled element of that kind —
  every segment of every multi-joint chain, every joint, every sensor.
- **`#id` selectors** use a limb's own **label** as the id — `arm left_arm
  { }` is addressable as `#left_arm`, and the id applies to every segment
  of that limb's chain at once (so `#left_arm { color: "red"; }` recolors
  all three segments above, not just the first).
- **`.class` selectors** need an explicit `class: name;` (or `classes: [a,
  b];` for more than one) on the block you want tagged — RSS doesn't infer
  classes from anything else.
- **Attribute selectors** — `sensor[type="camera"]` matches by sensor kind;
  any other `sensor[key="value"]` matches against that sensor's own
  declared properties.
- **`--custom-properties`** are declared inside the `style`/`appearance`
  block and read anywhere via `var(--name)`. Referencing an undeclared one
  is a real compile error, not a silent no-op — you'll get `undefined
  custom property '--typo' (referenced via var())` immediately.
- **Cascade** works like real CSS: `#id` beats `.class`/`attr[...]` beats a
  bare element type, and among equal-specificity rules the one written
  *later* wins. In the example above, `#left_arm`'s red beats `arm`'s grey
  for `left_arm`'s three segments specifically, regardless of which rule
  happens to appear first in the file.
- **Named colors** (`"red"`, `"black"`, `"white"`, ...) work anywhere a hex
  string would, still quoted like every other RSS string.
- **Real materials, not just color**: `material: aluminum;` (also `steel`,
  `brushed_steel`, `brass`, `copper`, `carbon_fiber`, `plastic`, `rubber`,
  `wood`, `glass`, `brushless`, `servo`) sets a real color + metallic +
  roughness preset; `metallic:`/`roughness:`/`emission:`/`transparency:`/
  `opacity:` all have real effects too, and an explicit value on the same
  block always wins over whatever a preset would have set. All of it flows
  into real output: MJCF gets a real `<asset><material>` (no more
  placeholder rainbow colors), SDF/Gazebo gets a real PBR `<material>`
  block, and NELO-Studio's 3D view renders it with a real PBR material too.
  URDF only ever carries flat `color:` — a real limit of the URDF format
  itself, not something stacc is choosing not to support. See
  `examples/styled_quadruped.rss` for a complete robot exercising all of
  this at once.

---

## 5. Reuse — `component` and `use`

### 5.1 Same-file components

Define shared defaults once, apply them to one or more robots:

```
component "industrial-arm" {
    joints: 6;
    payload: 10kg;

    appearance {
        body   { color: "#181818"; }
        accent { color: "#FF5500"; }
    }
}

robot "FactoryBot" {
    use industrial-arm;

    body {
        shape: box;
        material: aluminum;
        color: "#111111";
    }

    arm left {
        actuator: servo;
        reach: 650mm;
    }

    gripper { type: parallel; fingers: 2; }
}
```

`use industrial-arm;` makes the component's own properties become
**defaults** — the using block's own properties win on conflict — and
prepends its child blocks. This is applied recursively at every nesting
depth, with cycle detection, so components can build on other components
safely.

### 5.2 Cross-file components — a real shared stylesheet

Put reusable components in their own library file with no `robot { }` at
all:

```
// lib_industrial_arm.rss
component "industrial-arm" {
    joints: 6;
    payload: 10kg;

    appearance {
        body   { color: "#181818"; }
        accent { color: "#FF5500"; }
    }
}
```

Then pull it into another file:

```
// factorybot_shared_lib.rss
robot "FactoryBotShared" {
    use "lib_industrial_arm.rss:industrial-arm";

    body { shape: box; material: aluminum; color: "#111111"; }
    arm left { actuator: servo; reach: 650mm; }
    gripper { type: parallel; fingers: 2; }
}
```

`use "path/to/file.rss:name";` resolves `path/to/file.rss` against the
compiling file's own directory first, then any `--rss-path` directories
you passed on the CLI, and pulls `name` from that file's `component`
defs. This is the actual "import a stylesheet from elsewhere" behavior —
not just per-file defaults. It resolves one file deep per reference (a
library file can itself `use` other libraries — that's supported — but
there's no package manager or versioning layer, just relative paths /
`--rss-path`).

```bash
stacc rss check factorybot_shared_lib.rss
```

---

## 6. From idea to physical robot

### 6.1 `stacc rss build` — pick your output

```bash
stacc rss build my_robot.rss                # full bundle (see below)
stacc rss build my_robot.rss --format sdf    # just Gazebo SDF
stacc rss build my_robot.rss --format mjcf   # just MuJoCo XML
stacc rss build my_robot.rss --format stl    # 3D-printable parts
stacc rss build my_robot.rss --format bom    # shopping list
stacc rss build my_robot.rss --format all    # everything at once
stacc rss build my_robot.rss --out ./dist    # custom output directory
```

With no `--format`, you get the full default bundle: `build/urdf/`,
`build/gazebo/` (SDF), and `build/ros2/<name>_description/` (a minimal
ROS 2 `ament_cmake` package skeleton wrapping the URDF).

**`--format stl`** (`stacc/rss/mesh_export.py`) writes one real, watertight
STL per link via `trimesh`, sized to that link's actual dimensions, plus a
best-effort merged `<robot>_assembly.stl` for a print preview, into
`build/parts/`. Honest limitation: RSS's `Geometry` only models
box/cylinder/sphere/mesh primitives — no fillets, chamfers, or CSG (holes,
unions, subtractions). A link with an unrecognized/missing mesh is skipped
with a warning, never given fake geometry.

**`--format bom`** (`stacc/rss/bom.py`) writes `build/bom.csv` and
`build/bom.md` — a real procurement checklist listing every link (as a
part to print), every actuated joint (with its `actuator`/`motor` value
from the file), and every sensor (with its `model:` value). Honest
limitation: names are exactly what you wrote in the `.rss` file — not
resolved to a real distributor SKU, price, or datasheet. It's a shopping
list, not a quote.

### 6.2 `stacc rss run` — simulate it

```bash
stacc rss run my_robot.rss --duration 10       # headless, 10 simulated seconds
stacc rss run my_robot.rss --sim               # explicitly route through the physics simulator
stacc rss run my_robot.rss --sim --render      # open the MuJoCo viewer (needs a display/GPU)
```

### 6.3 Real MuJoCo, not a mock

`stacc rss run --sim` compiles your document into a real
`stacc.core.robot_base.Robot`, converts it to MJCF, and loads it into an
actual `mujoco.MjModel` — real physics stepping, real sensor data for any
`lidar`/`camera`/`imu` you declared. Nothing about this path is
hand-waved: if the `stacc` SDK or `mujoco` can't be imported, commands that
need them say so and stop, rather than faking a result.

### 6.4 `stacc rss deploy` — real hardware

```bash
stacc rss deploy my_robot.rss 192.168.1.50 --user pi
stacc rss deploy my_robot.rss 192.168.1.50 --user pi --key ~/.ssh/id_ed25519
```

This compiles the robot, writes a URDF plus a **template** `src/robot.py`
control script (one `MotorDriver`/`GPIO` stub per actuated joint, named
from the joint), then uses the real SSH/SFTP client in
`stacc.hal.deploy.Deployer` to connect to `<ip>` and upload it. A failed
connection is reported as a failure and stops there — it never prints a
fake "deployed successfully." Honest limitation: the control script's GPIO
pins are always `None  # TODO` — physical wiring is something only you
know, so it's generated as a template to fill in, never an autonomous
guess. (`stacc build --target=arduino/esp32/stm32` is refused outright for
the same reason — stacc has no C/C++ toolchain and generates no
microcontroller firmware.)

---

## 7. Using RSS inside NELO-Studio

If you're working in the NELO-Studio IDE, RSS is wired in directly — no
need to leave the editor:

1. **Create or open a `.rss` file** anywhere in the workspace (New File →
   name it `my_robot.rss`). It gets real syntax highlighting automatically.
2. **Write your robot** using anything from this guide.
3. **Click Run.** This compiles your file through the real RSS compiler
   (the same one the CLI uses), hot-swaps it into the live MuJoCo daemon,
   and opens the **Robot Designer** panel with your robot already loaded —
   real geometry, real DOF count, real joint sliders.
4. Alternatively, open the Robot Designer directly from **1. Model → Design
   a New Robot (RSS)** in the top nav — it ships with the same real
   examples from `stacc-sdk/examples/` in its "Examples" picker, and a
   "Compile" button if you'd rather not use a file at all.
5. Drag a joint slider — the value is sent to the real daemon, which steps
   real MuJoCo physics and reports the real resulting torques/energy back;
   the 3D view mirrors that real state, not an optimistic guess.

This is the same real pipeline as the CLI's `stacc rss run --sim` — just
integrated into the editor's normal Run button and a live 3D view.

---

## 8. Grammar reference

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

## 9. Semantic mapping — RSS block → what it becomes

| RSS block | Becomes |
|---|---|
| `body` / `chassis` | The robot's root `Link`. A mobile `shape` (`differential-drive`/`ackermann`/`mecanum`/`omni`) sets `robot.fixed_base = False`; anything else is fixed-base. |
| `arm` / `leg` / `wheel` / `gripper` / `head` (top-level or inside `actuators { }`) | A child `Link` + `Joint`. `wheel` → continuous joint; everything else → revolute. |
| ...with `joints: N` (N > 1) | An **N-segment serial chain** instead of one joint (`arm left { joints: 6; }` → `arm_left_1 … arm_left_6`), reach split evenly, alternating Y/Z axes. |
| `sensors { ... }` | Each child → a sensor via `Robot.attach_sensor`. `lidar`/`camera`/`imu` wire to real `stacc.sensors` drivers; anything else falls back to a generic, inert `SensorSpec`. |
| `appearance` / `style` | Real selectors + cascade: element type (`arm`, `sensor`, `joint`, ...) matches every element of that kind; `#id` matches a limb's label (whole chain); `.class` matches a declared `class:`; `elem[attr="val"]` matches sensors by kind/property. Plus `--var` declarations and `var()`. Legacy exact-name matching (root name or a literal compiled link name) still works, at `#id`-level specificity. See §4.8. |
| `component "name" { ... }` + `use name;` | The component's properties become defaults (using block wins on conflict); its children are prepended. Recursive, with cycle detection. |
| `use "path/to/file.rss:name";` | Cross-file reuse — real shared-stylesheet behavior, not just per-file defaults. |
| Anything else at the top level (e.g. `navigation { ... }`) | Not built structurally, but never silently dropped — lands in `robot.rss_metadata[block_name]` plus a warning. |
| A property with no structural home (`payload`, `actuator`, sensor `model:`, ...) | Preserved verbatim in `robot.rss_metadata[link_name]`, unit-normalized. |

Recognized top-level/limb block names exactly: `body`/`chassis` (root),
`arm`, `leg`, `wheel`, `gripper`, `head` (limbs), `actuators` (container),
`sensors` (container), `appearance`/`style` (appearance). Everything else
is metadata + a warning, per the table above.

## 10. Known limitations (by design, for this version)

- A multi-joint limb (`joints: N`) is a straight serial chain with fixed
  Y/Z axis alternation — no explicit per-joint axis/limits, and no
  non-serial (differential/parallel) topologies yet.
- Sensor `model:` strings aren't matched to specific hardware profiles —
  only the generic *kind* (`lidar`/`camera`/`imu`) selects a real driver.
- `color:`/`material:`/`metallic:`/`roughness:`/`emission:`/`transparency:`
  all have real effects now — but still per whole link, not per-geometry:
  no texture maps/UVs, no decals. URDF export only ever carries flat
  `color:` (a real limit of the URDF format, not of RSS).
- Cross-file `use` has no package manager or versioning — relative paths
  and `--rss-path` only.
- No parametric expressions (`radius: radius * 2`), CSG, fillets/chamfers,
  constraints, or STEP export. These need a real CAD kernel (e.g.
  OpenCASCADE) and are a separate, much larger effort.
- The auto-computed spatial layout for a limb's joints (used when
  rendering/simulating) stacks each limb straight along its joint axis
  from wherever it attaches — it has no inherent "front-left vs. rear-right"
  concept, so several limbs on the same parent (like a quadruped's four
  legs) currently lay out along the same line rather than spreading to
  visually distinct corners, unless you give them an explicit offset via a
  `Connection`.

None of these are silent — every one of them either refuses clearly, logs
a warning, or is documented here rather than faked.

## 11. Troubleshooting

**"expected SEMICOLON, got IDENT '...'"** — every property line needs a
trailing `;`, including the last one in a block:

```
chassis {
    shape: box
    dimensions: [1m, 1m, 1m];   // ← missing ';' on the line above
}
```

**A block I wrote seems to have no effect** — check `stacc rss check`'s
warnings. If your block name isn't one of `body`/`chassis`/`arm`/`leg`/
`wheel`/`gripper`/`head`/`actuators`/`sensors`/`appearance`/`style`, it
becomes metadata rather than structure (§4.7) — this is intentional, not a
bug, but easy to miss if you expected it to shape the robot.

**"document contains no 'robot' definitions"** — a library file (only
`component { }` blocks, no `robot { }`) is meant to be `use`d from another
file, not compiled directly.

**"document defines multiple robots (...); pass robot_name="** — add
`--robot <name>` to `check`/`build`/`run`/`deploy`.

## 12. Cheat sheet

```
robot my_robot {                        // or: component "name" { ... }
    chassis { shape: differential-drive; dimensions: [Xmm, Ymm, Zmm]; }
    body    { shape: box; material: aluminum; color: "#RRGGBB"; }

    arm/leg/wheel/gripper/head <label> {
        joints: N;           // > 1 → N-segment serial chain
        reach: Xmm;
        actuator: <ident>;   // metadata
        class: <name>;       // tag for a .class selector, below
    }

    sensors {
        lidar <label>  { model: <ident>; range: Xm; fov: Xdeg; }
        camera <label> { model: <ident>; }
        imu <label>    { model: <ident>; rate: Xhz; }
    }

    actuators { wheel left { motor: "..."; radius: Xmm; } }

    style {                                       // "appearance" is an alias
        --name: "value";                          // custom property
        <root-or-exact-link-name> { color: "#RRGGBB"; }  // legacy exact match
        arm/leg/wheel/gripper/head/joint/sensor { color: "..."; }  // element type
        #<limb-label> { color: "..."; }            // id (whole limb chain)
        .<class> { emission: "..."; }              // class
        sensor[type="camera"] { color: "..."; }    // attribute
        prop: var(--name);                         // anywhere a value goes
    }

    use <component-name>;                       // same file
    use "path/to/lib.rss:component-name";        // cross file
}
```

```bash
stacc rss check  my_robot.rss
stacc rss build  my_robot.rss [--format urdf|sdf|mjcf|stl|bom|all] [--out DIR]
stacc rss run    my_robot.rss [--sim] [--render] [--duration N]
stacc rss deploy my_robot.rss <ip> [--user pi] [--key PATH]
```
