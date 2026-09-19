"""Test the RSS declarative robot language — lexer, parser, compiler, URDF export."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

results = []
EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def test(name, func):
    try:
        func()
        results.append((name, "PASS"))
        print(f"  \033[92m✓\033[0m {name}")
    except Exception as e:
        results.append((name, "FAIL"))
        print(f"  \033[91m✗\033[0m {name}: {e}")


print("\n" + "=" * 60)
print("  RSS language tests")
print("=" * 60 + "\n")

# ═══════ LEXER ═══════


def test_lexer_tokens():
    from stacc.rss.lexer import tokenize

    toks = tokenize('robot "X" { reach: 650mm; ratio: 1.5m/s; tags: [a, "b"]; }')
    kinds = [t.kind for t in toks]
    assert kinds[:4] == ["IDENT", "STRING", "LBRACE", "IDENT"], kinds[:4]

    num_toks = [t for t in toks if t.kind == "NUMBER"]
    assert num_toks[0].value == 650.0 and num_toks[0].unit == "mm"
    assert num_toks[1].value == 1.5 and num_toks[1].unit == "m/s"

    assert toks[-1].kind == "EOF"


test("Lexer — units, strings, arrays", test_lexer_tokens)


def test_lexer_comments_and_errors():
    from stacc.rss.errors import RSSSyntaxError
    from stacc.rss.lexer import tokenize

    toks = tokenize("// a comment\nrobot X {}\n")
    assert [t.kind for t in toks[:4]] == ["IDENT", "IDENT", "LBRACE", "RBRACE"]

    try:
        tokenize('robot "unterminated')
        raise AssertionError("expected RSSSyntaxError")
    except RSSSyntaxError:
        pass


test("Lexer — comments + syntax errors", test_lexer_comments_and_errors)

# ═══════ PARSER ═══════


def test_parser_nested_blocks():
    from stacc.rss.parser import parse

    doc = parse(
        """
        robot warehouse_bot {
            chassis { shape: differential-drive; dimensions: [600mm, 450mm, 250mm]; }
            sensors {
                lidar front { model: rplidar-a2; range: 12m; fov: 360deg; }
            }
        }
        """
    )
    assert len(doc.robots) == 1
    root = doc.robots[0].block
    chassis = root.child("chassis")
    assert chassis is not None
    assert chassis.properties["shape"] == "differential-drive"
    dims = chassis.properties["dimensions"]
    assert [d.value for d in dims] == [600.0, 450.0, 250.0]
    assert [d.unit for d in dims] == ["mm", "mm", "mm"]

    sensors = root.child("sensors")
    lidar = sensors.children[0]
    assert lidar.name == "lidar" and lidar.label == "front"
    assert lidar.properties["model"] == "rplidar-a2"


test("Parser — nested blocks, labels, arrays", test_parser_nested_blocks)


def test_parser_component_and_use():
    from stacc.rss.parser import parse

    doc = parse(
        """
        component "industrial-arm" { joints: 6; payload: 10kg; }
        robot "FactoryBot" {
            use industrial-arm;
            body { shape: box; }
        }
        """
    )
    assert "industrial-arm" in doc.components
    robot_block = doc.robots[0].block
    assert robot_block.uses == ["industrial-arm"]


test("Parser — component defs + use directive", test_parser_component_and_use)

# ═══════ COMPILER ═══════


def test_compile_warehouse_bot():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    assert robot.name == "warehouse_bot"
    link_names = {l.name for l in robot.links}
    assert "base" in link_names
    assert any(n.startswith("wheel_left") for n in link_names)
    assert any(n.startswith("wheel_right") for n in link_names)
    assert robot.num_joints == 2
    assert robot.dof == 2  # both wheels are continuous (actuated)
    assert not robot.fixed_base  # differential-drive is a mobile shape

    assert len(robot.sensors) == 2
    sensor_kinds = {s.kind for s in robot.sensors}
    assert sensor_kinds == {"lidar", "camera"}

    # Recognized sensor kinds (lidar/camera/imu) get wired to real
    # stacc.sensors drivers, not the inert fallback, so they produce real
    # MuJoCo-backed data in `stacc rss run --sim` instead of just being a
    # declaration nobody can act on.
    by_kind = {s.kind: s for s in robot.sensors}
    assert type(by_kind["lidar"]).__name__ == "LiDAR"
    assert type(by_kind["camera"]).__name__ == "Camera"
    assert by_kind["lidar"].enabled and by_kind["camera"].enabled
    assert by_kind["lidar"].fov_h == 360.0  # "360deg" read back as degrees, not radians
    assert by_kind["lidar"].range_max == 12.0  # "12m" normalized to meters


test("Compiler — warehouse_bot.rss -> Robot", test_compile_warehouse_bot)


def test_compile_multi_dof_limb():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "arm_6dof.rss").read_text())
    robot = compile_document(doc)

    arm_joints = [j for j in robot.joints if j.name.startswith("arm_main_")]
    assert len(arm_joints) == 6, [j.name for j in robot.joints]
    assert robot.num_joints == 7  # 6 arm joints + 1 gripper joint
    assert robot.dof == 7

    # Verify it's a real serial chain: base -> arm_main_1 -> ... -> arm_main_6,
    # not 6 segments all independently attached to base.
    by_child = {j.child_link: j.parent_link for j in robot.joints}
    assert by_child["arm_main_1"] == "base"
    for i in range(2, 7):
        assert by_child[f"arm_main_{i}"] == f"arm_main_{i - 1}"

    # reach: 900mm split evenly across 6 joints -> 0.15m per segment
    for link in robot.links:
        if link.name.startswith("arm_main_"):
            assert abs(link.visual.size[2] - 0.15) < 1e-9


test("Compiler — multi-DOF limb synthesis (arm_6dof.rss)", test_compile_multi_dof_limb)


def test_compile_use_merge():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "factorybot.rss").read_text())
    robot = compile_document(doc)

    # Component defaults (joints/payload) land on the robot-level metadata.
    robot_meta = robot.rss_metadata["__robot__"]
    assert robot_meta["payload"] == 10.0  # 10kg normalized to kg
    assert robot_meta["joints"] == 6

    # Component's appearance.accent color is preserved even though no link
    # named "accent" exists.
    assert robot.rss_metadata["__appearance__"]["accent"]["color"] == "#FF5500"

    link_names = {l.name for l in robot.links}
    assert "arm_left" in link_names
    assert "gripper" in link_names


test("Compiler — component/use merge (factorybot.rss)", test_compile_use_merge)


def test_compile_cross_file_use():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "factorybot_shared_lib.rss").read_text())

    # Without search_paths, the cross-file reference can't resolve.
    from stacc.rss.errors import RSSCompileError
    try:
        compile_document(doc)
        raise AssertionError("expected RSSCompileError without search_paths")
    except RSSCompileError:
        pass

    # With the examples/ directory as a search path, it resolves and behaves
    # exactly like the same-file factorybot.rss component/use test.
    robot = compile_document(doc, search_paths=[EXAMPLES])
    robot_meta = robot.rss_metadata["__robot__"]
    assert robot_meta["payload"] == 10.0
    assert robot_meta["joints"] == 6
    assert robot.rss_metadata["__appearance__"]["accent"]["color"] == "#FF5500"


test("Compiler — cross-file `use \"file.rss:name\";`", test_compile_cross_file_use)


def test_unrecognized_block_warns_but_keeps_metadata():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    # warehouse_bot.rss has a `navigation { ... }` block, which isn't a
    # recognized structural block type (no Link/Joint/sensor comes from it).
    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    assert any("navigation" in w for w in robot.rss_warnings)
    assert robot.rss_metadata["navigation"]["algorithm"] == "nav2"
    assert robot.rss_metadata["navigation"]["max_speed"] == 1.5  # "1.5m/s" passthrough


test(
    "Compiler — unrecognized block warns, keeps metadata (not silently dropped)",
    test_unrecognized_block_warns_but_keeps_metadata,
)


def test_compile_unknown_use_raises():
    from stacc.rss.compiler import compile_document
    from stacc.rss.errors import RSSCompileError
    from stacc.rss.parser import parse

    doc = parse('robot X { use nonexistent; body {} }')
    try:
        compile_document(doc)
        raise AssertionError("expected RSSCompileError")
    except RSSCompileError:
        pass


test("Compiler — unknown `use` target raises", test_compile_unknown_use_raises)

# ═══════ URDF EXPORT ═══════


def test_urdf_roundtrip():
    from stacc.converters.urdf_mjcf_usd import ModelTranspiler
    from stacc.core.urdf_parser import load_urdf_string
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    urdf_xml = ModelTranspiler.robot_to_urdf(robot)
    assert "<robot name=\"warehouse_bot\">" in urdf_xml

    reloaded = load_urdf_string(urdf_xml)
    assert {l.name for l in reloaded.links} == {l.name for l in robot.links}
    assert {j.name for j in reloaded.joints} == {j.name for j in robot.joints}
    assert reloaded.num_joints == robot.num_joints


test("URDF export — round-trips through load_urdf_string", test_urdf_roundtrip)


def test_sdf_roundtrip():
    from stacc.converters.urdf_mjcf_usd import ModelTranspiler
    from stacc.core.urdf_parser import load_sdf
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    sdf_xml = ModelTranspiler.robot_to_sdf(robot)
    sdf_path = Path("/tmp/stacc_rss_test.sdf")
    sdf_path.write_text(sdf_xml)
    try:
        reloaded = load_sdf(str(sdf_path))
        assert {l.name for l in reloaded.links} == {l.name for l in robot.links}
        assert {j.name for j in reloaded.joints} == {j.name for j in robot.joints}
        assert {j.type for j in reloaded.joints} == {j.type for j in robot.joints}
    finally:
        sdf_path.unlink(missing_ok=True)


test("SDF export — round-trips through load_sdf", test_sdf_roundtrip)


def test_mjcf_export_loads_in_mujoco():
    import mujoco

    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse
    from stacc.sim.engine import SimulationConfig
    from stacc.sim.mjcf_builder import robot_to_mjcf

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    mjcf_xml = robot_to_mjcf(robot, SimulationConfig())
    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    assert model.njnt >= robot.num_joints


test("MJCF export — loads as a real MuJoCo model", test_mjcf_export_loads_in_mujoco)


def test_ros2_package_skeleton():
    import shutil
    import tempfile

    from stacc.converters.urdf_mjcf_usd import ModelTranspiler
    from stacc.rss import write_ros2_package
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)
    urdf_xml = ModelTranspiler.robot_to_urdf(robot)

    tmp_dir = Path(tempfile.mkdtemp())
    try:
        pkg_dir = write_ros2_package(robot, urdf_xml, tmp_dir)
        assert (pkg_dir / "package.xml").exists()
        assert (pkg_dir / "urdf" / "warehouse_bot.urdf").exists()
        assert (pkg_dir / "launch" / "display.launch.py").exists()

        controllers = (pkg_dir / "config" / "controllers.yaml").read_text()
        for joint in robot.joints:
            if joint.type != "fixed":
                assert joint.name in controllers

        ET.parse(pkg_dir / "package.xml")  # must be valid XML
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


test("ROS2 skeleton — package.xml/urdf/launch/controllers.yaml written", test_ros2_package_skeleton)


def test_stl_export_watertight():
    import shutil
    import tempfile

    import trimesh

    from stacc.rss.compiler import compile_document
    from stacc.rss.mesh_export import write_stl_parts
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "quadruped.rss").read_text())
    robot = compile_document(doc)

    tmp_dir = Path(tempfile.mkdtemp())
    try:
        written = write_stl_parts(robot, str(tmp_dir))
        assert len(written) == robot.num_links + 1  # one per link + assembly
        assert getattr(robot, "rss_mesh_warnings", []) == []

        base_mesh = trimesh.load(str(tmp_dir / "base.stl"))
        assert base_mesh.is_watertight
        leg_mesh = trimesh.load(str(tmp_dir / "leg_front_left_1.stl"))
        assert leg_mesh.is_watertight
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


test("STL export — real, watertight meshes per link (quadruped.rss)", test_stl_export_watertight)


def test_bom_generation():
    import shutil
    import tempfile

    from stacc.rss.bom import generate_bom, write_bom
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    rows = generate_bom(robot)
    by_name = {r.name: r for r in rows}
    assert "base" in by_name and by_name["base"].category == "printed_part"
    assert "wheel_left_joint" in by_name
    assert "BLDC-01" in by_name["wheel_left_joint"].notes
    sensor_rows = [r for r in rows if r.category == "sensor"]
    assert any("rplidar-a2" in r.spec for r in sensor_rows)

    tmp_dir = Path(tempfile.mkdtemp())
    try:
        paths = write_bom(robot, str(tmp_dir))
        with open(paths["csv"], newline="") as f:
            import csv as csv_mod
            csv_rows = list(csv_mod.DictReader(f))
        assert len(csv_rows) == len(rows)
        assert set(csv_rows[0].keys()) == {"category", "name", "spec", "source", "notes"}
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


test("BOM generation — real rows from links/joints/sensors, CSV round-trips", test_bom_generation)


def test_control_script_template_is_valid_python():
    import ast

    from stacc.cli import _generate_control_script
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)

    script = _generate_control_script(robot)
    ast.parse(script)  # raises SyntaxError if invalid
    assert "wheel_left_joint = MotorDriver(" in script
    assert "pin_a=None,  # TODO" in script


test(
    "rss deploy — generated control script is valid Python with wiring TODOs",
    test_control_script_template_is_valid_python,
)

# ═══════ SUMMARY ═══════

print("\n" + "=" * 60)
passed = sum(1 for _, s in results if s == "PASS")
failed = sum(1 for _, s in results if s == "FAIL")
total = len(results)
if failed == 0:
    print(f"  \033[92mALL {total} TESTS PASSED\033[0m")
else:
    print(
        f"  \033[92m{passed} passed\033[0m, \033[91m{failed} failed\033[0m out of {total}"
    )
print("=" * 60 + "\n")
sys.exit(1 if failed > 0 else 0)
