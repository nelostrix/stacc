"""Recursive-descent parser for RSS.

Grammar (informal):

    document      := statement*
    statement     := component_def | robot_def
    component_def := 'component' name '{' block_body '}'
    robot_def     := 'robot' name '{' block_body '}'
    name          := STRING | IDENT

    block_body    := block_item*
    block_item    := use_directive | property | nested_block
    use_directive := 'use' name ';'
    property      := IDENT ':' value ';'
    nested_block  := IDENT [IDENT] '{' block_body '}'   # second IDENT is an optional label
    value         := STRING | NUMBER | IDENT | array
    array         := '[' (value (',' value)*)? ']'

A block_item starting with IDENT is disambiguated by lookahead: `:` means a
property, `{` or `IDENT {` means a nested block.
"""

from __future__ import annotations

from typing import List

from stacc.rss.ast_nodes import Block, ComponentDef, Document, NumberValue, RobotDef, Selector, Value, VarRef
from stacc.rss.errors import RSSSyntaxError
from stacc.rss.lexer import Token, tokenize


class Parser:
    def __init__(self, source: str):
        self.source = source
        self.tokens: List[Token] = tokenize(source)
        self.pos = 0

    # ── token stream helpers ──

    def _peek(self, offset: int = 0) -> Token:
        idx = min(self.pos + offset, len(self.tokens) - 1)
        return self.tokens[idx]

    def _advance(self) -> Token:
        tok = self.tokens[self.pos]
        if self.pos < len(self.tokens) - 1:
            self.pos += 1
        return tok

    def _error(self, message: str, tok: "Token | None" = None) -> RSSSyntaxError:
        if tok is None:
            tok = self._peek()
        lines = self.source.splitlines() or [""]
        source_line = lines[tok.line - 1] if tok.line - 1 < len(lines) else ""
        return RSSSyntaxError(message, tok.line, tok.col, source_line)

    def _expect(self, kind: str) -> Token:
        tok = self._peek()
        if tok.kind != kind:
            raise self._error(f"expected {kind}, got {tok.kind} {tok.value!r}")
        return self._advance()

    def _at(self, kind: str, offset: int = 0) -> bool:
        return self._peek(offset).kind == kind

    def _at_keyword(self, keyword: str, offset: int = 0) -> bool:
        tok = self._peek(offset)
        return tok.kind == "IDENT" and tok.value == keyword

    # ── entry point ──

    def parse(self) -> Document:
        doc = Document()
        while not self._at("EOF"):
            if self._at_keyword("component"):
                comp = self._parse_component_def()
                doc.components[comp.name] = comp
            elif self._at_keyword("robot"):
                doc.robots.append(self._parse_robot_def())
            else:
                raise self._error(
                    f"expected 'component' or 'robot' at top level, got "
                    f"{self._peek().kind} {self._peek().value!r}"
                )
        return doc

    # ── top-level statements ──

    def _parse_name(self) -> str:
        if self._at("STRING"):
            return str(self._advance().value)
        if self._at("IDENT"):
            return str(self._advance().value)
        raise self._error(f"expected a name (string or identifier), got {self._peek().kind}")

    def _parse_component_def(self) -> ComponentDef:
        self._advance()  # 'component'
        name = self._parse_name()
        self._expect("LBRACE")
        block = self._parse_block_body("component", None)
        self._expect("RBRACE")
        return ComponentDef(name=name, block=block)

    def _parse_robot_def(self) -> RobotDef:
        self._advance()  # 'robot'
        name = self._parse_name()
        self._expect("LBRACE")
        block = self._parse_block_body("robot", None)
        self._expect("RBRACE")
        return RobotDef(name=name, block=block)

    # ── block bodies ──

    def _parse_block_body(self, name: str, label: "str | None") -> Block:
        block = Block(name=name, label=label)
        while not self._at("RBRACE"):
            if self._at("EOF"):
                raise self._error("unexpected end of file inside block")

            if self._at_keyword("use"):
                self._advance()
                block.uses.append(self._parse_name())
                self._expect("SEMICOLON")
                continue

            # --name: value;  — a CSS-style custom property declaration.
            if self._at("CUSTOM_PROP"):
                name_tok = self._advance()
                self._expect("COLON")
                value = self._parse_value()
                self._expect("SEMICOLON")
                block.var_decls[str(name_tok.value)] = value
                continue

            # #id { ... }  — id-selector style rule.
            if self._at("HASH"):
                self._advance()
                id_name = str(self._expect("IDENT").value)
                self._expect("LBRACE")
                child = self._parse_block_body(f"#{id_name}", None)
                child.selector = Selector(id=id_name)
                self._expect("RBRACE")
                block.children.append(child)
                continue

            # .class { ... }  — class-selector style rule.
            if self._at("DOT"):
                self._advance()
                class_name = str(self._expect("IDENT").value)
                self._expect("LBRACE")
                child = self._parse_block_body(f".{class_name}", None)
                child.selector = Selector(classes=[class_name])
                self._expect("RBRACE")
                block.children.append(child)
                continue

            if not self._at("IDENT"):
                raise self._error(
                    f"expected a property or nested block, got {self._peek().kind} "
                    f"{self._peek().value!r}"
                )

            key_tok = self._advance()
            key = str(key_tok.value)

            if self._at("COLON"):
                self._advance()
                value = self._parse_value()
                self._expect("SEMICOLON")
                block.properties[key] = value
                continue

            # elem[attr="value"] { ... }  — attribute-selector style rule.
            # `IDENT [` is otherwise never valid at block-item position (an
            # array literal only ever appears after a ':'), so this is
            # unambiguous with no lookahead beyond what we already have.
            if self._at("LBRACKET"):
                self._advance()
                attr_name = str(self._expect("IDENT").value)
                self._expect("EQUALS")
                attr_value = str(self._expect("STRING").value)
                self._expect("RBRACKET")
                self._expect("LBRACE")
                child = self._parse_block_body(f"{key}[{attr_name}]", None)
                child.selector = Selector(element=key, attr=(attr_name, attr_value))
                self._expect("RBRACE")
                block.children.append(child)
                continue

            if self._at("LBRACE"):
                self._advance()
                child = self._parse_block_body(key, None)
                self._expect("RBRACE")
                block.children.append(child)
                continue

            if self._at("IDENT") and self._at("LBRACE", 1):
                label_tok = self._advance()
                self._advance()  # LBRACE
                child = self._parse_block_body(key, str(label_tok.value))
                self._expect("RBRACE")
                block.children.append(child)
                continue

            raise self._error(
                f"expected ':' (property) or '{{' (nested block) after "
                f"'{key}', got {self._peek().kind} {self._peek().value!r}"
            )

        return block

    def _parse_value(self) -> Value:
        tok = self._peek()
        if tok.kind == "STRING":
            self._advance()
            return str(tok.value)
        if tok.kind == "IDENT" and tok.value == "var" and self._at("LPAREN", 1):
            self._advance()  # 'var'
            self._expect("LPAREN")
            name_tok = self._expect("CUSTOM_PROP")
            self._expect("RPAREN")
            return VarRef(name=str(name_tok.value))
        if tok.kind == "IDENT":
            self._advance()
            return str(tok.value)
        if tok.kind == "NUMBER":
            self._advance()
            return NumberValue(value=float(tok.value), unit=tok.unit)
        if tok.kind == "LBRACKET":
            return self._parse_array()
        raise self._error(f"expected a value, got {tok.kind} {tok.value!r}")

    def _parse_array(self) -> List[Value]:
        self._expect("LBRACKET")
        items: List[Value] = []
        if not self._at("RBRACKET"):
            items.append(self._parse_value())
            while self._at("COMMA"):
                self._advance()
                items.append(self._parse_value())
        self._expect("RBRACKET")
        return items


def parse(source: str) -> Document:
    return Parser(source).parse()
