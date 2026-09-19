"""Errors for the RSS (declarative robot description) language."""

from __future__ import annotations


class RSSError(Exception):
    """Base class for all RSS-related errors."""


class RSSSyntaxError(RSSError):
    """A lexing or parsing error, with a line/column pointer into the source."""

    def __init__(self, message: str, line: int, col: int, source_line: str = ""):
        self.message = message
        self.line = line
        self.col = col
        self.source_line = source_line
        super().__init__(str(self))

    def __str__(self) -> str:
        header = f"RSS syntax error at line {self.line}, col {self.col}: {self.message}"
        if not self.source_line:
            return header
        pointer = " " * max(self.col - 1, 0) + "^"
        return f"{header}\n  {self.source_line}\n  {pointer}"


class RSSCompileError(RSSError):
    """An error resolving/compiling a parsed RSS document into a Robot."""
