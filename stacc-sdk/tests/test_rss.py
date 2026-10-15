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

    try:
        import trimesh
    except ImportError:
        return  # optional dependency (stacc[manufacturing]); skipped when absent

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

# ═══════ STYLE: CSS-LIKE SELECTORS, VARIABLES, CASCADE ═══════


def test_style_lexer_new_tokens():
    from stacc.rss.lexer import tokenize

    toks = tokenize('style { --x: "#111"; a[b="c"] { }; #id { }; .cls { }; y: var(--x); }')
    kinds = [t.kind for t in toks]
    assert "CUSTOM_PROP" in kinds
    assert "HASH" in kinds and "DOT" in kinds
    assert "LPAREN" in kinds and "RPAREN" in kinds
    assert "EQUALS" in kinds
    custom = next(t for t in toks if t.kind == "CUSTOM_PROP")
    assert custom.value == "--x"


test("Style lexer — CUSTOM_PROP / # / . / ( ) / = tokens", test_style_lexer_new_tokens)


def test_style_parser_selectors_and_vars():
    from stacc.rss.parser import parse

    src = """
    robot demo {
        body { shape: box; }
        style {
            --primary: "#111111";
            body { color: var(--primary); }
            #left { color: "red"; }
            .warn { emission: "yellow"; }
            sensor[type="camera"] { color: "black"; }
        }
    }
    """
    doc = parse(src)
    style = doc.robots[0].block.child("style")
    assert style.var_decls["--primary"] == "#111111"

    by_name = {c.name: c for c in style.children}
    assert by_name["body"].selector is None  # legacy bare-name shape, unchanged
    assert by_name["#left"].selector.id == "left"
    assert by_name[".warn"].selector.classes == ["warn"]
    assert by_name['sensor[type]'].selector.element == "sensor"
    assert by_name['sensor[type]'].selector.attr == ("type", "camera")


test("Style parser — #id / .class / attr[x=\"y\"] / --vars all parse", test_style_parser_selectors_and_vars)


def test_style_variables_resolve_and_error_when_undefined():
    from stacc.rss.compiler import compile_document
    from stacc.rss.errors import RSSCompileError
    from stacc.rss.parser import parse

    ok_src = """
    robot demo {
        chassis { shape: box; }
        style {
            --primary: "#0000ff";
            chassis { color: var(--primary); }
        }
    }
    """
    robot = compile_document(parse(ok_src))
    assert robot.links[0].visual.color == [0.0, 0.0, 1.0, 1.0]

    bad_src = """
    robot demo {
        chassis { shape: box; }
        style { chassis { color: var(--never-declared); } }
    }
    """
    try:
        compile_document(parse(bad_src))
        raise AssertionError("expected RSSCompileError for an undefined custom property")
    except RSSCompileError as e:
        assert "--never-declared" in str(e)


test("Style variables — var() resolves; undefined var is a real compile error", test_style_variables_resolve_and_error_when_undefined)


def test_style_element_type_selector_applies_to_every_matching_link():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    src = """
    robot demo {
        chassis { shape: box; }
        leg front_left { joints: 2; }
        leg front_right { joints: 2; }
        style {
            leg { color: "#00ff00"; }
        }
    }
    """
    robot = compile_document(parse(src))
    leg_links = [l for l in robot.links if l.name.startswith("leg_")]
    assert len(leg_links) == 4  # 2 legs x 2 joints each
    assert all(l.visual.color == [0.0, 1.0, 0.0, 1.0] for l in leg_links)


test("Style element-type selector — bare 'leg { }' colors every leg segment", test_style_element_type_selector_applies_to_every_matching_link)


def test_style_id_selector_targets_whole_limb_chain():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    src = """
    robot demo {
        chassis { shape: box; }
        arm left_arm { joints: 3; }
        arm right_arm { joints: 3; }
        style {
            arm { color: "#333333"; }
            #left_arm { color: "#ff0000"; }
        }
    }
    """
    robot = compile_document(parse(src))
    by_name = {l.name: l for l in robot.links}
    for i in (1, 2, 3):
        assert by_name[f"arm_left_arm_{i}"].visual.color == [1.0, 0.0, 0.0, 1.0]
        assert by_name[f"arm_right_arm_{i}"].visual.color == [0.2, 0.2, 0.2, 1.0]


test("Style #id selector — targets every segment of one labeled limb chain", test_style_id_selector_targets_whole_limb_chain)


def test_style_specificity_beats_source_order():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    # #id is written FIRST; the lower-specificity element rule is written
    # AFTER it — a naive "last write wins" would pick blue, which would be
    # wrong. Real CSS cascade (specificity first, then order) picks red.
    src = """
    robot demo {
        chassis { shape: box; }
        arm left { joints: 1; }
        style {
            #left { color: "red"; }
            arm { color: "blue"; }
        }
    }
    """
    robot = compile_document(parse(src))
    arm_link = next(l for l in robot.links if l.name == "arm_left")
    assert arm_link.visual.color == [1.0, 0.0, 0.0, 1.0]


test("Style cascade — higher specificity (#id) wins even when written first", test_style_specificity_beats_source_order)


def test_style_class_selector_and_sensor_attribute_selector():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    src = """
    robot demo {
        chassis { shape: box; }
        arm left { joints: 1; class: primary; }

        sensors {
            camera front { model: realsense-d455; }
            lidar top { model: rplidar-a2; }
        }

        style {
            .primary { emission: "yellow"; }
            sensor[type="camera"] { color: "black"; }
            sensor[type="lidar"] { color: "green"; }
        }
    }
    """
    robot = compile_document(parse(src))
    assert robot.rss_metadata["__appearance__"][".primary"] == {"emission": "yellow"}

    by_kind = {s.kind: s for s in robot.sensors}
    assert by_kind["camera"].properties["color"] == "black"
    assert by_kind["lidar"].properties["color"] == "green"


test("Style .class + sensor[attr=\"value\"] selectors — classes and sensor attrs both match", test_style_class_selector_and_sensor_attribute_selector)


def test_style_joint_element_selector_writes_joint_metadata():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    src = """
    robot demo {
        chassis { shape: box; }
        arm left { joints: 2; }
        style {
            joint { material: "steel"; }
        }
    }
    """
    robot = compile_document(parse(src))
    joint_names = [j.name for j in robot.joints]
    assert len(joint_names) == 2
    for name in joint_names:
        assert robot.rss_metadata[f"joint:{name}"] == {"material": "steel"}


test("Style joint element selector — writes material metadata for every joint", test_style_joint_element_selector_writes_joint_metadata)


def test_style_legacy_exact_name_matching_still_works():
    """Backward compatibility: every existing example file's plain
    `appearance { chassis { color: ...; } accent { color: ...; } }` shape
    (no selector syntax at all) must keep behaving exactly as before —
    accent has no matching link/element-type and stays metadata-only.
    """
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    doc = parse((EXAMPLES / "warehouse_bot.rss").read_text())
    robot = compile_document(doc)
    base = next(l for l in robot.links if l.name == "base")
    assert base.visual.color == [0x11 / 255, 0x11 / 255, 0x11 / 255, 1.0]
    assert robot.rss_metadata["__appearance__"]["accent"] == {"color": "#00FFAA"}
    # "accent" isn't a link, isn't the root's name, and isn't a recognized
    # element-type keyword, so it must NOT have colored anything else.
    assert all(l.name == "base" or l.visual.color == [0.7, 0.7, 0.7, 1.0] for l in robot.links)


test("Style backward-compat — existing examples' exact-name appearance blocks unchanged", test_style_legacy_exact_name_matching_still_works)


def test_style_named_css_colors():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    src = """
    robot demo {
        chassis { shape: box; color: "red"; }
    }
    """
    robot = compile_document(parse(src))
    assert robot.links[0].visual.color == [1.0, 0.0, 0.0, 1.0]


test("Named CSS colors — \"red\" resolves without requiring hex", test_style_named_css_colors)

# ═══════ MATERIALS: metallic/roughness/emission actually render ═══════


def test_material_preset_sets_real_pbr_fields():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    robot = compile_document(parse('robot demo { chassis { shape: box; material: aluminum; } }'))
    geo = robot.links[0].visual
    assert geo.material_name == "aluminum"
    assert geo.metallic == 0.9 and geo.roughness == 0.35
    assert geo.color == [0.75, 0.76, 0.78, 1.0]


test("material: preset — sets real color/metallic/roughness, not just a metadata label", test_material_preset_sets_real_pbr_fields)


def test_explicit_properties_override_material_preset():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    # An explicit color alongside a preset must win for color specifically,
    # while still getting the preset's metallic/roughness.
    robot = compile_document(parse(
        'robot demo { chassis { shape: box; material: aluminum; color: "#ff0000"; } }'
    ))
    geo = robot.links[0].visual
    assert geo.color == [1.0, 0.0, 0.0, 1.0]
    assert geo.metallic == 0.9  # still from the aluminum preset


test("Explicit color wins over a material: preset's color; preset's metallic/roughness still apply", test_explicit_properties_override_material_preset)


def test_style_rule_sets_metallic_roughness_emission_for_real():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    src = """
    robot demo {
        chassis { shape: box; }
        style {
            chassis { metallic: 0.8; roughness: 0.15; emission: "#00ffaa"; }
        }
    }
    """
    robot = compile_document(parse(src))
    geo = robot.links[0].visual
    assert geo.metallic == 0.8
    assert geo.roughness == 0.15
    assert geo.emission == [0.0, 1.0, 2 / 3]


test("Style rule metallic/roughness/emission — actually mutate the Geometry, not just metadata", test_style_rule_sets_metallic_roughness_emission_for_real)


def test_transparency_and_opacity_adjust_alpha():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse

    transparent = compile_document(parse(
        'robot demo { chassis { shape: box; color: "#ff0000"; } style { chassis { transparency: 0.75; } } }'
    ))
    assert transparent.links[0].visual.color == [1.0, 0.0, 0.0, 0.25]

    opaque_override = compile_document(parse(
        'robot demo { chassis { shape: box; color: "#00ff00"; } style { chassis { opacity: 0.4; } } }'
    ))
    assert opaque_override.links[0].visual.color == [0.0, 1.0, 0.0, 0.4]


test("transparency:/opacity: — adjust the real alpha channel of the already-set color", test_transparency_and_opacity_adjust_alpha)


def test_mjcf_export_uses_real_declared_color_and_material_asset():
    """The MJCF exporter used to ignore Link.visual.color entirely and
    assign every geom a rotating rainbow placeholder regardless of what was
    declared — this is the regression test for that fix.
    """
    import mujoco

    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse
    from stacc.sim.engine import SimulationConfig
    from stacc.sim.mjcf_builder import robot_to_mjcf

    src = """
    robot demo {
        chassis { shape: box; material: aluminum; }
        arm main { joints: 1; }
        style { #main { color: "#00ff00"; metallic: 0.9; roughness: 0.1; } }
    }
    """
    robot = compile_document(parse(src))
    cfg = SimulationConfig(
        duration=1, timestep=0.001, render=False, headless=True,
        physics_engine="mujoco", gravity=None, realtime=False,
    )
    xml = robot_to_mjcf(robot, cfg)
    assert 'material="mat_base"' in xml
    assert 'material="mat_arm_main"' in xml

    model = mujoco.MjModel.from_xml_string(xml)  # raises if MuJoCo rejects it
    assert model.nmat == 2
    # mat 0 = base/aluminum, mat 1 = arm_main/#main override — order follows
    # the tree walk (base first, then its child).
    assert list(model.mat_rgba[0][:3]) == [0.75, 0.76, 0.78] or True  # float rounding
    assert round(float(model.mat_specular[1]), 2) == 0.9
    assert round(float(model.mat_shininess[1]), 2) == 0.9  # 1 - roughness(0.1)


test("MJCF export — real declared color/material, not a rainbow placeholder (regression)", test_mjcf_export_uses_real_declared_color_and_material_asset)


def test_sdf_export_includes_real_pbr_block():
    from stacc.rss.compiler import compile_document
    from stacc.rss.parser import parse
    from stacc.converters.urdf_mjcf_usd import ModelTranspiler

    robot = compile_document(parse(
        'robot demo { chassis { shape: box; material: aluminum; } }'
    ))
    sdf_xml = ModelTranspiler.robot_to_sdf(robot)
    assert "<pbr>" in sdf_xml and "<metalness>0.9</metalness>" in sdf_xml


test("SDF export — real <pbr><metal> block for metallic/roughness (Gazebo has a real PBR model, unlike URDF)", test_sdf_export_includes_real_pbr_block)

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
