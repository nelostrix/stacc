"""Generate C++ and Rust declarations from an `.rss` file.

C++ and Rust can't import a `.rss` file the way Python can (`from design import
joints`), so the same names are generated as source code:

    python -m stacc.rss.codegen design.rss --lang cpp     # -> design.rss.hpp
    python -m stacc.rss.codegen design.rss --lang rust    # -> design.rss.rs

    #include "design.rss.hpp"                    // C++17
    cmd[design::joints::leg_front_left_1] = 0.4;
    for (auto j : design::classes::lead_leg) ...

    #[path = "design.rss.rs"] mod design;        // Rust
    cmd[design::joints::LEG_FRONT_LEFT_1] = 0.4;

Joint indices follow the runtime's joint order (`/v1/runtime/shm` `joint_names`,
fixed joints included), so they index command and state arrays directly. A
misspelled joint is a compile error instead of a wrong motor.

NELO Studio regenerates the file whenever the `.rss` changes, for any C++ file
that includes `<name>.rss.hpp` or Rust file that names `<name>.rss.rs`.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

CPP_KEYWORDS = {
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor", "bool", "break", "case", "catch",
    "char", "char8_t", "char16_t", "char32_t", "class", "compl", "concept", "const", "consteval", "constexpr",
    "constinit", "const_cast", "continue", "co_await", "co_return", "co_yield", "decltype", "default", "delete",
    "do", "double", "dynamic_cast", "else", "enum", "explicit", "export", "extern", "false", "float", "for",
    "friend", "goto", "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept", "not", "not_eq",
    "nullptr", "operator", "or", "or_eq", "private", "protected", "public", "register", "reinterpret_cast",
    "requires", "return", "short", "signed", "sizeof", "static", "static_assert", "static_cast", "struct",
    "switch", "template", "this", "thread_local", "throw", "true", "try", "typedef", "typeid", "typename",
    "union", "unsigned", "using", "virtual", "void", "volatile", "wchar_t", "while", "xor", "xor_eq",
    # names the generated namespaces use themselves
    "count", "names",
}


def _identifier(name: str, upper: bool = False) -> str:
    ident = re.sub(r"[^0-9A-Za-z_]", "_", name)
    if not ident or ident[0].isdigit():
        ident = f"_{ident}"
    if upper:
        ident = ident.upper()
        return f"{ident}_" if ident in ("COUNT", "NAMES") else ident
    return f"{ident}_" if ident in CPP_KEYWORDS else ident


def _unique(names: Iterable[str], upper: bool) -> List[Tuple[str, str]]:
    """(identifier, original name) pairs; identifiers that collide after cleaning get a suffix."""
    out: List[Tuple[str, str]] = []
    taken: Dict[str, int] = {}
    for name in names:
        ident = _identifier(name, upper)
        if ident in taken:
            taken[ident] += 1
            ident = f"{ident}_{taken[ident]}"
        else:
            taken[ident] = 1
        out.append((ident, name))
    return out


def _classes(robot: Any) -> Dict[str, List[int]]:
    """RSS class name -> runtime indices of the joints whose child link carries that class."""
    link_classes: Dict[str, List[str]] = getattr(robot, "rss_classes", {}) or {}
    out: Dict[str, List[int]] = {}
    for index, joint in enumerate(robot.joints):
        for c in link_classes.get(joint.child_link, []):
            out.setdefault(c, []).append(index)
    return out


def _quote(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def generate_cpp(robot: Any, module: str, source_name: str) -> str:
    joints = [j.name for j in robot.joints]
    links = [l.name for l in robot.links]
    classes = _classes(robot)
    ns = _identifier(module)
    lines = [
        f"// Generated from {source_name} by stacc.rss.codegen. Do not edit: regenerate when {source_name} changes.",
        "#pragma once",
        "#include <array>",
        "#include <cstddef>",
        "",
        f"namespace {ns} {{",
        f"inline constexpr const char* robot_name = {_quote(robot.name)};",
        f"inline constexpr std::size_t dof = {robot.dof};",
        "",
        "// Runtime joint order: indices into command / state arrays (fixed joints included).",
        "namespace joints {",
    ]
    for i, (ident, _name) in enumerate(_unique(joints, upper=False)):
        lines.append(f"inline constexpr std::size_t {ident} = {i};")
    lines += [
        f"inline constexpr std::size_t count = {len(joints)};",
        f"inline constexpr std::array<const char*, {len(joints)}> names = {{{', '.join(_quote(n) for n in joints)}}};",
        "}  // namespace joints",
        "",
        "namespace links {",
    ]
    for i, (ident, _name) in enumerate(_unique(links, upper=False)):
        lines.append(f"inline constexpr std::size_t {ident} = {i};")
    lines += [
        f"inline constexpr std::size_t count = {len(links)};",
        f"inline constexpr std::array<const char*, {len(links)}> names = {{{', '.join(_quote(n) for n in links)}}};",
        "}  // namespace links",
        "",
        "// RSS `class:` names -> indices of the joints on parts with that class.",
        "namespace classes {",
    ]
    for ident, name in _unique(classes, upper=False):
        idx = classes[name]
        lines.append(f"inline constexpr std::array<std::size_t, {len(idx)}> {ident} = {{{', '.join(map(str, idx))}}};")
    lines += ["}  // namespace classes", f"}}  // namespace {ns}", ""]
    return "\n".join(lines)


def generate_rust(robot: Any, module: str, source_name: str) -> str:
    joints = [j.name for j in robot.joints]
    links = [l.name for l in robot.links]
    classes = _classes(robot)
    lines = [
        f"// Generated from {source_name} by stacc.rss.codegen. Do not edit: regenerate when {source_name} changes.",
        f"// Use with: #[path = \"{source_name}.rs\"] mod {_identifier(module)};",
        "#![allow(dead_code)]",
        "",
        f"pub const ROBOT_NAME: &str = {_quote(robot.name)};",
        f"pub const DOF: usize = {robot.dof};",
        "",
        "/// Runtime joint order: indices into command / state arrays (fixed joints included).",
        "pub mod joints {",
    ]
    for i, (ident, _name) in enumerate(_unique(joints, upper=True)):
        lines.append(f"    pub const {ident}: usize = {i};")
    lines += [
        f"    pub const COUNT: usize = {len(joints)};",
        f"    pub const NAMES: [&str; {len(joints)}] = [{', '.join(_quote(n) for n in joints)}];",
        "}",
        "",
        "pub mod links {",
    ]
    for i, (ident, _name) in enumerate(_unique(links, upper=True)):
        lines.append(f"    pub const {ident}: usize = {i};")
    lines += [
        f"    pub const COUNT: usize = {len(links)};",
        f"    pub const NAMES: [&str; {len(links)}] = [{', '.join(_quote(n) for n in links)}];",
        "}",
        "",
        "/// RSS `class:` names -> indices of the joints on parts with that class.",
        "pub mod classes {",
    ]
    for ident, name in _unique(classes, upper=True):
        idx = classes[name]
        lines.append(f"    pub const {ident}: [usize; {len(idx)}] = [{', '.join(map(str, idx))}];")
    lines += ["}", ""]
    return "\n".join(lines)


GENERATORS = {"cpp": (generate_cpp, ".hpp"), "rust": (generate_rust, ".rs")}


def generate(robot: Any, lang: str, source_name: str) -> str:
    """Source text declaring `robot`'s joints, links and classes in `lang` ("cpp" or "rust")."""
    if lang not in GENERATORS:
        raise ValueError(f"unknown language '{lang}' (expected cpp or rust)")
    module = Path(source_name).name.split(".")[0]
    return GENERATORS[lang][0](robot, module, Path(source_name).name)


def output_path(rss_path: Path, lang: str) -> Path:
    """`design.rss` -> `design.rss.hpp` / `design.rss.rs`, next to it."""
    return rss_path.with_name(rss_path.name + GENERATORS[lang][1])


def main(argv: Optional[List[str]] = None) -> int:
    from stacc.rss.embed import load_file

    parser = argparse.ArgumentParser(prog="python -m stacc.rss.codegen", description="Generate C++ / Rust declarations from an .rss file.")
    parser.add_argument("rss", type=Path, help="the .rss file")
    parser.add_argument("--lang", choices=sorted(GENERATORS), required=True)
    parser.add_argument("--robot", help="which robot, when the file defines several")
    parser.add_argument("-o", "--output", type=Path, help="output file (default: <file>.rss.hpp / .rss.rs next to it); - for stdout")
    args = parser.parse_args(argv)

    robot = load_file(args.rss, robot_name=args.robot)
    text = generate(robot, args.lang, args.rss.name)
    if args.output is not None and str(args.output) == "-":
        sys.stdout.write(text)
        return 0
    out = args.output or output_path(args.rss, args.lang)
    if not out.exists() or out.read_text() != text:
        out.write_text(text)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
