"""Embedded RSS: `<<<RSS>>> ... <<</RSS>>>` blocks inside Python/C++/Rust files."""

import runpy
import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stacc.rss import RSSCompileError, RSSSyntaxError, extract_rss_blocks, load_file  # noqa: E402
from stacc.rss.embed import compile_embedded  # noqa: E402

SDK = Path(__file__).resolve().parents[1]

ARM = "robot arm { body { shape: box; } arm main { joints: 2; reach: 400mm; } }"


def test_python_string_block():
    text = f'ROBOT = rss("""<<<RSS>>>\n{ARM}\n<<</RSS>>>""")\n'
    (block,) = extract_rss_blocks(text)
    assert block.source.strip() == ARM
    assert (block.start_line, block.end_line) == (1, 3)
    assert compile_embedded(text).dof == 2


def test_cpp_block_comment():
    text = f"#include <x>\n/* <<<RSS>>>\n{ARM}\n<<</RSS>>> */\nint main() {{}}\n"
    (block,) = extract_rss_blocks(text)
    assert block.source.strip() == ARM
    assert compile_embedded(text).name == "arm"


@pytest.mark.parametrize("leader", ["# ", "// ", " * "])
def test_line_comment_leader_is_stripped(leader):
    body = "\n".join(leader + line for line in ["robot arm {", "  body { shape: box; }", "}"])
    text = f"{leader}<<<RSS>>>\n{body}\n{leader}<<</RSS>>>\n"
    (block,) = extract_rss_blocks(text)
    assert block.source.splitlines()[0] == "robot arm {"
    assert compile_embedded(text).name == "arm"


def test_syntax_error_points_at_host_file_line_and_column():
    text = textwrap.dedent(
        """\
        import stacc
        # <<<RSS>>>
        # robot arm {
        #   body { shape: box }
        # }
        # <<</RSS>>>
        """
    )
    with pytest.raises(RSSSyntaxError) as info:
        compile_embedded(text, file="robot.py")
    err = info.value
    # The missing ';' is reported on line 4 of the host file, after `box`.
    assert err.line == 4
    assert text.splitlines()[3][err.col - 1] == "}"
    assert err.file == "robot.py"


def test_unclosed_block_is_an_error_at_the_opening_marker():
    with pytest.raises(RSSSyntaxError) as info:
        extract_rss_blocks(f"x = 1\n/* <<<RSS>>>\n{ARM}\n")
    assert info.value.line == 2


def test_no_block_is_a_clear_error():
    with pytest.raises(RSSCompileError, match="no RSS block"):
        compile_embedded("print('hi')\n", file="robot.py")


def test_blocks_merge_components_and_robots():
    text = textwrap.dedent(
        """\
        /* <<<RSS>>>
        component "light-arm" { joints: 2; reach: 300mm; }
        <<</RSS>>> */
        /* <<<RSS>>>
        robot bot { body { shape: box; } arm left { use "light-arm"; } }
        <<</RSS>>> */
        """
    )
    assert len(extract_rss_blocks(text)) == 2
    assert compile_embedded(text).dof == 2


def test_rss_and_this_file_from_a_real_module(tmp_path):
    module = tmp_path / "bot.py"
    module.write_text(
        textwrap.dedent(
            f'''\
            from stacc.rss import rss, this_file
            INLINE = rss("""{ARM}""")
            # <<<RSS>>>
            # robot commented {{ body {{ shape: box; }} }}
            # <<</RSS>>>
            '''
        )
    )
    ns = runpy.run_path(str(module))
    assert ns["INLINE"].dof == 2

    bad = tmp_path / "bad.py"
    bad.write_text('from stacc.rss import rss\n\nBROKEN = rss("""\nrobot x {\n  body { shape: box }\n}\n""")\n')
    with pytest.raises(RSSSyntaxError) as info:
        runpy.run_path(str(bad))
    assert info.value.line == 5  # `  body { shape: box }` in bad.py


@pytest.mark.parametrize(
    "path, name, dof",
    [
        ("examples/embedded/scout_rover.py", "scout", 4),
        ("examples/embedded/arm_reach.cpp", "reach_arm", 3),
        ("bindings/rust/stacc_runtime/examples/quadruped_crouch.rs", "pup", 12),
    ],
)
def test_example_host_files_compile(path, name, dof):
    robot = load_file(SDK / path)
    assert (robot.name, robot.dof) == (name, dof)


def test_scout_rover_is_mobile_with_four_side_wheels():
    robot = load_file(SDK / "examples/embedded/scout_rover.py")
    assert robot.fixed_base is False
    ys = sorted(robot.get_connection_offset("base", j.child_link)[1] for j in robot.joints)
    assert ys[0] < 0 < ys[-1]
    assert not robot.rss_warnings
