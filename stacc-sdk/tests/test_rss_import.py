"""Importing .rss files as Python modules (stacc/rss/importer.py)."""

import importlib
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import stacc.rss  # noqa: E402  (installs the importer)

SDK = Path(__file__).resolve().parents[1]

DESIGN = textwrap.dedent(
    """
    component servo_leg { joints: 3; reach: 350mm; effort: 2.5Nm; }

    robot dog {
        body { shape: box; dimensions: [500mm, 250mm, 120mm]; }
        leg front_left  { use servo_leg; class: lead_leg; }
        leg front_right { use servo_leg; }
        sensors { imu body { model: bmi088; } }
    }
    """
)


@pytest.fixture
def project(tmp_path, monkeypatch):
    (tmp_path / "design.rss").write_text(DESIGN)
    monkeypatch.syspath_prepend(str(tmp_path))
    yield tmp_path
    for name in [m for m in sys.modules if m == "design" or m.startswith("robots")]:
        del sys.modules[name]


def test_import_exposes_robot_parts_classes_and_components(project):
    from design import classes, components, joints, links, robot, sensors

    assert robot.name == "dog"
    assert len(joints) == robot.num_joints and joints.names() == [j.name for j in robot.joints]
    first = robot.joints[0].name
    assert getattr(joints, first) is joints[first] is robot.joints[0]
    assert "base" in links
    assert len(sensors) == 1
    lead = classes.lead_leg
    assert len(lead.joints) == 3 and all(j.child_link.startswith("leg_front_left") for j in lead.joints)
    assert components["servo_leg"] == {"joints": 3, "reach": pytest.approx(0.35), "effort": pytest.approx(2.5)}


def test_robot_is_also_available_by_name_and_reload_recompiles(project):
    import design

    assert design.dog is design.robot
    (project / "design.rss").write_text(DESIGN.replace("leg front_right { use servo_leg; }", ""))
    importlib.reload(design)
    assert len(design.classes.lead_leg.joints) == 3
    assert design.robot.num_joints < 7


def test_unknown_part_names_the_alternatives(project):
    from design import joints

    with pytest.raises(AttributeError, match="no joint named 'elbow'"):
        joints.elbow


def test_packages_and_explicit_paths(project):
    pkg = project / "robots"
    pkg.mkdir()
    (pkg / "__init__.py").write_text("")
    (pkg / "arm-v2.rss").write_text("robot arm { body { shape: box; } arm main { joints: 2; reach: 400mm; } }")
    (pkg / "scout.rss").write_text("robot scout { chassis { shape: differential-drive; } wheel left { radius: 60mm; } }")

    from robots.scout import robot as scout

    assert scout.name == "scout"
    arm = stacc.rss.module(pkg / "arm-v2.rss")
    assert arm.robot.dof == 2


def test_py_module_of_the_same_name_wins(project):
    (project / "design.py").write_text("WHO = 'python'\n")
    import design

    assert design.WHO == "python"


def test_files_with_several_robots_need_a_name(project):
    (project / "fleet.rss").write_text(
        "robot a { body { shape: box; } }\nrobot b { body { shape: box; } arm main { joints: 1; } }"
    )
    import fleet

    assert fleet.robots["b"].dof == 1 and fleet.a.name == "a"
    with pytest.raises(AttributeError, match="defines 2 robots"):
        fleet.joints
    del sys.modules["fleet"]


def test_syntax_errors_surface_on_import(project):
    (project / "broken.rss").write_text("robot x { body { shape: box }")
    with pytest.raises(stacc.rss.RSSSyntaxError):
        import broken  # noqa: F401


def test_script_imports_design_next_to_it(project):
    script = project / "main.py"
    script.write_text("import stacc.rss\nfrom design import robot, joints\nprint(robot.name, len(joints))\n")
    out = subprocess.run(
        [sys.executable, str(script)], capture_output=True, text=True, env={"PYTHONPATH": str(SDK)}, cwd="/"
    )
    assert out.returncode == 0, out.stderr
    assert out.stdout.split()[0] == "dog"
