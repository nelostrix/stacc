"""C++ / Rust declarations generated from .rss files (stacc/rss/codegen.py)."""

import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stacc.rss import parse  # noqa: E402
from stacc.rss.codegen import generate, main, output_path  # noqa: E402
from stacc.rss.compiler import compile_document  # noqa: E402

SDK = Path(__file__).resolve().parents[1]

DESIGN = textwrap.dedent(
    """
    robot dog {
        body { shape: box; dimensions: [500mm, 250mm, 120mm]; }
        leg front_left  { joints: 3; reach: 350mm; class: lead_leg; }
        leg front_right { joints: 3; reach: 350mm; }
    }
    """
)


def robot():
    return compile_document(parse(DESIGN))


def test_cpp_header_compiles_and_matches_runtime_order(tmp_path):
    if not shutil.which("g++"):
        pytest.skip("g++ not installed")
    r = robot()
    (tmp_path / "design.rss.hpp").write_text(generate(r, "cpp", "design.rss"))
    first = r.joints[0].name
    lead = [i for i, j in enumerate(r.joints) if j.child_link.startswith("leg_front_left")]
    (tmp_path / "main.cpp").write_text(
        textwrap.dedent(
            f"""
            #include "design.rss.hpp"
            #include <cstring>
            static_assert(design::joints::count == {len(r.joints)});
            static_assert(design::joints::{first} == 0);
            static_assert(design::classes::lead_leg.size() == {len(lead)});
            static_assert(design::classes::lead_leg[0] == {lead[0]});
            int main() {{
                return std::strcmp(design::joints::names[design::joints::{first}], "{first}") == 0
                    && std::strcmp(design::robot_name, "dog") == 0 ? 0 : 1;
            }}
            """
        )
    )
    exe = tmp_path / "main"
    build = subprocess.run(["g++", "-std=c++17", "-Wall", "-Werror", str(tmp_path / "main.cpp"), "-o", str(exe)], capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    assert subprocess.run([str(exe)]).returncode == 0


def test_names_become_valid_unique_identifiers():
    from stacc.rss.codegen import _identifier, _unique

    assert _identifier("int") == "int_" and _identifier("count") == "count_"
    assert _identifier("2nd-arm") == "_2nd_arm"
    assert _identifier("count", upper=True) == "COUNT_"
    assert [i for i, _ in _unique(["a-b", "a_b"], upper=False)] == ["a_b", "a_b_2"]


def test_rust_module_compiles(tmp_path):
    if not shutil.which("rustc"):
        pytest.skip("rustc not installed")
    r = robot()
    (tmp_path / "design.rss.rs").write_text(generate(r, "rust", "design.rss"))
    first = r.joints[0].name.upper()
    (tmp_path / "main.rs").write_text(
        textwrap.dedent(
            f"""
            #[path = "design.rss.rs"]
            mod design;
            fn main() {{
                assert_eq!(design::joints::{first}, 0);
                assert_eq!(design::joints::NAMES.len(), design::joints::COUNT);
                assert_eq!(design::ROBOT_NAME, "dog");
                assert!(!design::classes::LEAD_LEG.is_empty());
            }}
            """
        )
    )
    exe = tmp_path / "main"
    build = subprocess.run(["rustc", "--edition", "2021", "-D", "warnings", str(tmp_path / "main.rs"), "-o", str(exe)], capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    assert subprocess.run([str(exe)]).returncode == 0


def test_cli_writes_next_to_the_rss_and_only_when_changed(tmp_path, capsys):
    src = tmp_path / "design.rss"
    src.write_text(DESIGN)
    assert main([str(src), "--lang", "cpp"]) == 0
    out = output_path(src, "cpp")
    assert out.name == "design.rss.hpp" and "namespace design" in out.read_text()
    mtime = out.stat().st_mtime_ns
    assert main([str(src), "--lang", "cpp"]) == 0
    assert out.stat().st_mtime_ns == mtime
