"""Hand-written tokenizer for RSS.

Zero dependencies (stdlib only), matching the rest of stacc's core.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from stacc.rss.errors import RSSSyntaxError

# Units recognized directly after a number with no space, e.g. "650mm", "1.5m/s".
# Longest-first so "m/s" is tried before "m".
_UNITS = ["m/s", "deg/s", "rad/s", "mm", "cm", "m", "kg", "g", "deg", "rad", "hz", "khz"]

_PUNCTUATION = {
    "{": "LBRACE",
    "}": "RBRACE",
    "[": "LBRACKET",
    "]": "RBRACKET",
    ":": "COLON",
    ";": "SEMICOLON",
    ",": "COMMA",
}


@dataclass
class Token:
    kind: str  # IDENT, STRING, NUMBER, LBRACE, RBRACE, LBRACKET, RBRACKET, COLON, SEMICOLON, COMMA, EOF
    value: object
    unit: Optional[str]
    line: int
    col: int


def _is_ident_start(ch: str) -> bool:
    return ch.isalpha() or ch == "_"


def _is_ident_char(ch: str) -> bool:
    return ch.isalnum() or ch in "_-"


class Lexer:
    def __init__(self, source: str):
        self.source = source
        self.lines = source.splitlines() or [""]
        self.pos = 0
        self.line = 1
        self.col = 1

    def _peek(self, offset: int = 0) -> str:
        idx = self.pos + offset
        return self.source[idx] if idx < len(self.source) else ""

    def _advance(self) -> str:
        ch = self.source[self.pos]
        self.pos += 1
        if ch == "\n":
            self.line += 1
            self.col = 1
        else:
            self.col += 1
        return ch

    def _error(self, message: str) -> RSSSyntaxError:
        source_line = self.lines[self.line - 1] if self.line - 1 < len(self.lines) else ""
        return RSSSyntaxError(message, self.line, self.col, source_line)

    def _skip_whitespace_and_comments(self) -> None:
        while self.pos < len(self.source):
            ch = self._peek()
            if ch in " \t\r\n":
                self._advance()
            elif ch == "/" and self._peek(1) == "/":
                while self.pos < len(self.source) and self._peek() != "\n":
                    self._advance()
            else:
                break

    def _read_string(self) -> Token:
        start_line, start_col = self.line, self.col
        self._advance()  # opening quote
        chars: List[str] = []
        while True:
            if self.pos >= len(self.source):
                raise self._error("unterminated string literal")
            ch = self._advance()
            if ch == '"':
                break
            if ch == "\\" and self.pos < len(self.source):
                esc = self._advance()
                chars.append({"n": "\n", "t": "\t", '"': '"', "\\": "\\"}.get(esc, esc))
            else:
                chars.append(ch)
        return Token("STRING", "".join(chars), None, start_line, start_col)

    def _read_number(self) -> Token:
        start_line, start_col = self.line, self.col
        chars: List[str] = []
        while self._peek().isdigit():
            chars.append(self._advance())
        if self._peek() == "." and self._peek(1).isdigit():
            chars.append(self._advance())
            while self._peek().isdigit():
                chars.append(self._advance())
        value = float("".join(chars))

        unit = None
        for candidate in _UNITS:
            if self.source[self.pos:self.pos + len(candidate)] == candidate:
                after = self.pos + len(candidate)
                after_ch = self.source[after] if after < len(self.source) else ""
                if not _is_ident_char(after_ch):
                    for _ in candidate:
                        self._advance()
                    unit = candidate
                    break

        return Token("NUMBER", value, unit, start_line, start_col)

    def _read_ident(self) -> Token:
        start_line, start_col = self.line, self.col
        chars = [self._advance()]
        while _is_ident_char(self._peek()):
            chars.append(self._advance())
        return Token("IDENT", "".join(chars), None, start_line, start_col)

    def tokenize(self) -> List[Token]:
        tokens: List[Token] = []
        while True:
            self._skip_whitespace_and_comments()
            if self.pos >= len(self.source):
                tokens.append(Token("EOF", None, None, self.line, self.col))
                break

            ch = self._peek()
            line, col = self.line, self.col

            if ch == '"':
                tokens.append(self._read_string())
            elif ch.isdigit():
                tokens.append(self._read_number())
            elif _is_ident_start(ch):
                tokens.append(self._read_ident())
            elif ch in _PUNCTUATION:
                self._advance()
                tokens.append(Token(_PUNCTUATION[ch], ch, None, line, col))
            else:
                raise self._error(f"unexpected character {ch!r}")

        return tokens


def tokenize(source: str) -> List[Token]:
    return Lexer(source).tokenize()
