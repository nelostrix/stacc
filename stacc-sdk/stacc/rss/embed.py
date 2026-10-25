"""Embedded RSS: robot descriptions written inside Python, C++ or Rust files.

A host file keeps its control code in its own language and describes the
robot's body in an RSS block between `<<<RSS>>>` and `<<</RSS>>>`. The block
sits somewhere the host language already ignores, so the file still runs,
lints and formats normally:

    # Python — a string passed to rss(), which returns the compiled Robot
    from stacc.rss import rss
    ROBOT = rss('''<<<RSS>>>
    robot rover {
        chassis { shape: differential-drive; dimensions: [400mm, 300mm, 120mm]; }
        wheel front_left { radius: 60mm; }
    }
    <<</RSS>>>''')

    // C++ / Rust — a block comment
    /* <<<RSS>>>
    robot rover { ... }
    <<</RSS>>> */

Lines inside a block may also all carry the same line-comment leader
(`# `, `// `, ` * `), which is stripped. Extraction is plain-text and
language-agnostic, so one implementation serves every host language. Every
RSS line remembers where it came from, so syntax errors point at the line and
column in the host file, not in the extracted text.
"""

from __future__ import annotations

import inspect
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple, Union

from stacc.rss.ast_nodes import Document
from stacc.rss.errors import RSSCompileError, RSSSyntaxError
from stacc.rss.parser import parse

OPEN_MARKER = "<<<RSS>>>"
CLOSE_MARKER = "<<</RSS>>>"

# A line-comment leader shared by every non-empty line of a block.
_LEADER_RE = re.compile(r"^(\s*(?:#|//|\*|--|;)\s?)")


@dataclass
class EmbeddedRSS:
    """One `<<<RSS>>> ... <<</RSS>>>` block found in a host file."""

    source: str
    """The RSS text, with any shared comment leader removed."""

    start_line: int
    """1-based host-file line holding the opening marker."""

    end_line: int
    """1-based host-file line holding the closing marker."""

    index: int
    """0-based position of this block among the file's blocks."""

    line_origins: List[Tuple[int, int]] = field(default_factory=list)
    """For RSS line i (0-based): (host-file line, columns removed before it)."""

    def to_file_position(self, line: int, col: int) -> Tuple[int, int]:
        """Map a 1-based (line, col) inside `source` to the host file."""
        if 1 <= line <= len(self.line_origins):
            file_line, shift = self.line_origins[line - 1]
            return file_line, col + shift
        return self.start_line + max(line - 1, 0), col


def has_embedded_rss(text: str) -> bool:
    return OPEN_MARKER in text


def extract_rss_blocks(text: str) -> List[EmbeddedRSS]:
    """Every RSS block in `text`, in order.

    An opening marker without a closing one is a syntax error pointing at the
    opening marker, rather than silently taking the rest of the file.
    """
    lines = text.split("\n")
    blocks: List[EmbeddedRSS] = []
    i = 0
    while i < len(lines):
        open_col = lines[i].find(OPEN_MARKER)
        if open_col < 0:
            i += 1
            continue
        start = i
        after_open = open_col + len(OPEN_MARKER)

        # Body lines: (host line index, text, column the text starts at).
        body: List[Tuple[int, str, int]] = []
        close_line = -1
        first_rest = lines[i][after_open:]
        same_line_close = first_rest.find(CLOSE_MARKER)
        if same_line_close >= 0:
            body.append((i, first_rest[:same_line_close], after_open))
            close_line = i
        else:
            if first_rest.strip():
                body.append((i, first_rest, after_open))
            j = i + 1
            while j < len(lines):
                close_col = lines[j].find(CLOSE_MARKER)
                if close_col >= 0:
                    before = lines[j][:close_col]
                    if before.strip() and not _LEADER_RE.fullmatch(before):
                        body.append((j, before, 0))
                    close_line = j
                    break
                body.append((j, lines[j], 0))
                j += 1
        if close_line < 0:
            raise RSSSyntaxError(
                f"'{OPEN_MARKER}' is never closed with '{CLOSE_MARKER}'",
                start + 1,
                open_col + 1,
                lines[start],
            )

        # Strip a comment leader only when every non-empty line (other than the
        # marker line's own remainder) has the same one.
        candidates = [(ln, t) for ln, t, c in body if c == 0 and t.strip()]
        leader = None
        if candidates:
            first = _LEADER_RE.match(candidates[0][1])
            if first and all(t.startswith(first.group(1)) or t.strip() == first.group(1).strip() for _, t in candidates):
                leader = first.group(1)

        source_lines: List[str] = []
        origins: List[Tuple[int, int]] = []
        for ln, t, col in body:
            shift = col
            if leader is not None and col == 0:
                if t.startswith(leader):
                    t, shift = t[len(leader):], len(leader)
                elif t.strip() == leader.strip():
                    t, shift = "", len(t)
            source_lines.append(t)
            origins.append((ln + 1, shift))

        blocks.append(
            EmbeddedRSS(
                source="\n".join(source_lines),
                start_line=start + 1,
                end_line=close_line + 1,
                index=len(blocks),
                line_origins=origins,
            )
        )
        i = close_line + 1
    return blocks


def _strip_markers(text: str) -> str:
    """For rss(): accept the RSS text with or without the markers."""
    return text if OPEN_MARKER in text else f"{OPEN_MARKER}\n{text}\n{CLOSE_MARKER}"


def parse_embedded(text: str, file: Optional[str] = None) -> Tuple[Document, List[EmbeddedRSS]]:
    """Parse every RSS block in a host file into one Document.

    Blocks are parsed separately (so each error maps to its own lines) and then
    merged: a file can hold a component library block and a robot block.
    """
    blocks = extract_rss_blocks(text)
    if not blocks:
        raise RSSCompileError(
            f"no RSS block found{f' in {file}' if file else ''}: "
            f"describe the robot between {OPEN_MARKER} and {CLOSE_MARKER}"
        )
    merged = Document()
    for block in blocks:
        try:
            doc = parse(block.source)
        except RSSSyntaxError as e:
            line, col = block.to_file_position(e.line, e.col)
            host_lines = text.split("\n")
            source_line = host_lines[line - 1] if 0 < line <= len(host_lines) else e.source_line
            err = RSSSyntaxError(e.message, line, col, source_line)
            err.file = file  # type: ignore[attr-defined]
            raise err from None
        for name, comp in doc.components.items():
            if name in merged.components:
                raise RSSCompileError(f"component '{name}' is defined in more than one RSS block")
            merged.components[name] = comp
        merged.robots.extend(doc.robots)
    return merged, blocks


def compile_embedded(
    text: str,
    file: Optional[Union[str, Path]] = None,
    robot_name: Optional[str] = None,
    search_paths: Optional[Sequence[Path]] = None,
):
    """Compile the robot described by a host file's RSS block(s)."""
    from stacc.rss.compiler import compile_document

    file_str = str(file) if file is not None else None
    doc, _blocks = parse_embedded(text, file_str)
    paths = list(search_paths or [])
    if file is not None:
        paths.insert(0, Path(file).resolve().parent)
    return compile_document(doc, robot_name=robot_name, search_paths=paths)


def load_file(path: Union[str, Path], robot_name: Optional[str] = None, search_paths: Optional[Sequence[Path]] = None):
    """Compile a `.rss` file, or a Python/C++/Rust file with an RSS block."""
    p = Path(path)
    text = p.read_text()
    if p.suffix == ".rss":
        from stacc.rss.compiler import compile_document

        return compile_document(parse(text), robot_name=robot_name, search_paths=[p.resolve().parent, *(search_paths or [])])
    return compile_embedded(text, file=p, robot_name=robot_name, search_paths=search_paths)


def rss(text: str, robot_name: Optional[str] = None):
    """Compile an inline RSS description into a Robot.

        ROBOT = rss('''<<<RSS>>> robot arm { ... } <<</RSS>>>''')

    The markers are optional here. Cross-file `use "lib.rss:name";` resolves
    against the calling file's directory. Syntax errors point at the line in
    the calling file when the string's text can be found there.
    """
    caller = inspect.stack()[1].filename
    real_file = caller if caller and not caller.startswith("<") and Path(caller).exists() else None
    base = Path(real_file).resolve().parent if real_file else None
    wrapped = _strip_markers(text)
    try:
        return compile_embedded(wrapped, robot_name=robot_name, search_paths=[base] if base else None)
    except RSSSyntaxError as e:
        if real_file is None:
            raise
        file_text = Path(real_file).read_text()
        at = file_text.find(text)
        if at < 0:
            raise
        # Lines of `wrapped` before `text` begins (1 when markers were added).
        lead = wrapped[: wrapped.find(text)].count("\n")
        line = file_text[:at].count("\n") + (e.line - lead)
        col = e.col + (at - file_text.rfind("\n", 0, at) - 1 if e.line - lead == 1 else 0)
        host_lines = file_text.split("\n")
        err = RSSSyntaxError(e.message, line, col, host_lines[line - 1] if 0 < line <= len(host_lines) else "")
        err.file = real_file  # type: ignore[attr-defined]
        raise err from None


def this_file(robot_name: Optional[str] = None):
    """Compile the RSS block(s) of the file that calls this function.

    For files that keep the robot in a comment block rather than a string:

        # <<<RSS>>>
        # robot arm { ... }
        # <<</RSS>>>
        ROBOT = this_file()
    """
    caller = inspect.stack()[1].filename
    return load_file(caller, robot_name=robot_name)
