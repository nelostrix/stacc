"""Universal NLP interface — control robots with natural language.

Parse human commands into robot actions. Works with text input,
and optionally with speech recognition.

Modules:
- CommandParser: parse text commands into structured actions
- IntentClassifier: classify intent from natural language
- SpeechInterface: speech-to-text → command → action pipeline
- ActionMapper: map parsed commands to robot control functions

Usage:
    from stacc.ai.nlp import CommandParser, ActionMapper

    parser = CommandParser()
    action = parser.parse("move forward 2 meters")
    # → {"action": "move", "direction": "forward", "distance": 2.0, "unit": "meters"}

    mapper = ActionMapper(robot)
    mapper.register("move", move_handler)
    mapper.execute(action)
"""

from __future__ import annotations

import math
import re
from typing import Any, Callable, Dict, List, Optional, Tuple

# ══════════════════════════════════════════
# COMMAND PARSER
# ══════════════════════════════════════════


class ParsedCommand:
    """Structured result from parsing a natural language command."""

    def __init__(
        self,
        action: str = "",
        params: Optional[Dict[str, Any]] = None,
        raw_text: str = "",
        confidence: float = 1.0,
    ):
        self.action = action
        self.params = params or {}
        self.raw_text = raw_text
        self.confidence = confidence

    def __repr__(self):
        return f"Command(action='{self.action}', params={self.params})"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "params": self.params,
            "raw_text": self.raw_text,
            "confidence": self.confidence,
        }


class CommandParser:
    """Parse natural language commands into structured actions.

    Handles common robot commands without requiring an LLM:
    - Movement: "move forward 2 meters", "go left", "turn right 90 degrees"
    - Manipulation: "pick up the red cube", "place on table", "open gripper"
    - Navigation: "go to position 3, 5", "navigate to the kitchen"
    - Queries: "what do you see?", "where am I?", "how far is the wall?"
    - Control: "stop", "pause", "faster", "slower", "reset"

    For complex commands, integrate with an LLM API.
    """

    # Pattern definitions: (regex, action, param_extractor)
    PATTERNS = [
        # Movement
        (
            r"(?:move|go|drive|walk)\s+(?:forward|ahead|front)\s*([\d.]+)?\s*(m|meter|meters|cm|ft)?",
            "move",
            lambda m: {
                "direction": "forward",
                "distance": float(m.group(1) or 1),
                "unit": m.group(2) or "meters",
            },
        ),
        (
            r"(?:move|go|drive|walk)\s+(?:backward|back|reverse)\s*([\d.]+)?\s*(m|meter|meters|cm|ft)?",
            "move",
            lambda m: {
                "direction": "backward",
                "distance": float(m.group(1) or 1),
                "unit": m.group(2) or "meters",
            },
        ),
        (
            r"(?:move|go|drive|walk)\s+(?:left)\s*([\d.]+)?\s*(m|meter|meters|cm|ft)?",
            "move",
            lambda m: {
                "direction": "left",
                "distance": float(m.group(1) or 1),
                "unit": m.group(2) or "meters",
            },
        ),
        (
            r"(?:move|go|drive|walk)\s+(?:right)\s*([\d.]+)?\s*(m|meter|meters|cm|ft)?",
            "move",
            lambda m: {
                "direction": "right",
                "distance": float(m.group(1) or 1),
                "unit": m.group(2) or "meters",
            },
        ),
        (
            r"(?:move|go)\s+(?:up)\s*([\d.]+)?\s*(m|meter|meters|cm|ft)?",
            "move",
            lambda m: {
                "direction": "up",
                "distance": float(m.group(1) or 0.5),
                "unit": m.group(2) or "meters",
            },
        ),
        (
            r"(?:move|go)\s+(?:down)\s*([\d.]+)?\s*(m|meter|meters|cm|ft)?",
            "move",
            lambda m: {
                "direction": "down",
                "distance": float(m.group(1) or 0.5),
                "unit": m.group(2) or "meters",
            },
        ),
        # Rotation
        (
            r"(?:turn|rotate|spin)\s+(?:left|counter[\s-]?clockwise)\s*([\d.]+)?\s*(?:deg|degrees?)?",
            "turn",
            lambda m: {"direction": "left", "angle": float(m.group(1) or 90)},
        ),
        (
            r"(?:turn|rotate|spin)\s+(?:right|clockwise)\s*([\d.]+)?\s*(?:deg|degrees?)?",
            "turn",
            lambda m: {"direction": "right", "angle": float(m.group(1) or 90)},
        ),
        (
            r"(?:turn|face)\s+(?:around|back)",
            "turn",
            lambda m: {"direction": "around", "angle": 180},
        ),
        # Navigation
        (
            r"(?:go|navigate|move)\s+to\s+(?:position\s+)?([-\d.]+)[,\s]+([-\d.]+)(?:[,\s]+([-\d.]+))?",
            "goto",
            lambda m: {
                "x": float(m.group(1)),
                "y": float(m.group(2)),
                "z": float(m.group(3)) if m.group(3) else 0.0,
            },
        ),
        (
            r"(?:go|navigate|move)\s+to\s+(?:the\s+)?(.+)",
            "goto_named",
            lambda m: {"location": m.group(1).strip()},
        ),
        # Manipulation
        (
            r"(?:pick\s*up|grab|grasp|take)\s+(?:the\s+)?(.+)",
            "pick",
            lambda m: {"object": m.group(1).strip()},
        ),
        (
            r"(?:place|put|drop|release)\s+(?:on|at|onto)\s+(?:the\s+)?(.+)",
            "place",
            lambda m: {"target": m.group(1).strip()},
        ),
        (
            r"(?:open|release)\s+(?:the\s+)?(?:gripper|hand|claw)",
            "gripper",
            lambda m: {"state": "open"},
        ),
        (
            r"(?:close|grip|clamp)\s+(?:the\s+)?(?:gripper|hand|claw)",
            "gripper",
            lambda m: {"state": "close"},
        ),
        # Arm joint control
        (
            r"(?:move|set)\s+joint\s+(\d+)\s+to\s+([-\d.]+)\s*(?:deg|degrees?|rad|radians?)?",
            "set_joint",
            lambda m: {"joint": int(m.group(1)), "angle": float(m.group(2))},
        ),
        # Speed
        (
            r"(?:set\s+)?speed\s+(?:to\s+)?([\d.]+)\s*(%|percent)?",
            "set_speed",
            lambda m: {"speed": float(m.group(1)) / (100 if m.group(2) else 1)},
        ),
        (
            r"(?:go\s+)?(?:faster|speed\s*up)",
            "set_speed",
            lambda m: {"speed": "increase"},
        ),
        (
            r"(?:go\s+)?(?:slower|slow\s*down)",
            "set_speed",
            lambda m: {"speed": "decrease"},
        ),
        # Control
        (r"\b(?:stop|halt|freeze|emergency)\b", "stop", lambda m: {}),
        (r"\b(?:pause)\b", "pause", lambda m: {}),
        (r"\b(?:resume|continue|go)\b", "resume", lambda m: {}),
        (r"\b(?:reset|home|zero)\b", "reset", lambda m: {}),
        # Queries
        (
            r"(?:what\s+(?:do\s+you|can\s+you)\s+see|look\s+around|scan|detect)",
            "look",
            lambda m: {},
        ),
        (
            r"(?:where\s+am\s+[iI]|what.s\s+my\s+position|current\s+(?:location|position))",
            "get_pose",
            lambda m: {},
        ),
        (
            r"(?:how\s+far|distance\s+to|range\s+to)\s+(?:the\s+)?(.+)",
            "measure_distance",
            lambda m: {"target": m.group(1).strip()},
        ),
        # Mode
        (
            r"(?:switch\s+to|enter|start)\s+(?:manual|teleop)\s*(?:mode)?",
            "set_mode",
            lambda m: {"mode": "manual"},
        ),
        (
            r"(?:switch\s+to|enter|start)\s+(?:auto|autonomous)\s*(?:mode)?",
            "set_mode",
            lambda m: {"mode": "autonomous"},
        ),
    ]

    def __init__(self):
        self._compiled = [
            (re.compile(pattern, re.IGNORECASE), action, extractor)
            for pattern, action, extractor in self.PATTERNS
        ]
        self._custom_patterns: List[Tuple[Any, str, Callable]] = []

    def add_pattern(self, pattern: str, action: str, extractor: Callable):
        """Add a custom command pattern."""
        self._custom_patterns.append(
            (re.compile(pattern, re.IGNORECASE), action, extractor)
        )

    def parse(self, text: str) -> ParsedCommand:
        """Parse a natural language command.

        Returns ParsedCommand with action and params.
        """
        text = text.strip()

        # Try custom patterns first
        for regex, action, extractor in self._custom_patterns:
            match = regex.search(text)
            if match:
                params = extractor(match)
                return ParsedCommand(
                    action=action, params=params, raw_text=text, confidence=0.9
                )

        # Try built-in patterns
        for regex, action, extractor in self._compiled:
            match = regex.search(text)
            if match:
                params = extractor(match)
                return ParsedCommand(
                    action=action, params=params, raw_text=text, confidence=0.8
                )

        return ParsedCommand(
            action="unknown", params={"text": text}, raw_text=text, confidence=0.0
        )


# ══════════════════════════════════════════
# ACTION MAPPER
# ══════════════════════════════════════════


class ActionMapper:
    """Map parsed commands to robot control functions.

    Usage:
        mapper = ActionMapper(robot)

        def handle_move(robot, params):
            direction = params["direction"]
            distance = params["distance"]
            # ... control robot

        mapper.register("move", handle_move)
        mapper.register("stop", lambda r, p: r.set_joint_torques([0]*r.dof))

        command = parser.parse("move forward 2 meters")
        mapper.execute(command)
    """

    def __init__(self, robot=None):
        self.robot = robot
        self._handlers: Dict[str, Callable] = {}

        # Register default handlers
        self._register_defaults()

    def _register_defaults(self):
        self.register("stop", self._handle_stop)
        self.register("reset", self._handle_reset)
        self.register("gripper", self._handle_gripper)

    def register(self, action: str, handler: Callable):
        """Register a handler for an action type.

        Handler signature: handler(robot, params) -> Any
        """
        self._handlers[action] = handler

    def execute(self, command: ParsedCommand) -> Any:
        """Execute a parsed command."""
        handler = self._handlers.get(command.action)
        if handler:
            return handler(self.robot, command.params)
        return None

    def can_handle(self, action: str) -> bool:
        return action in self._handlers

    def _handle_stop(self, robot, params):
        if robot:
            robot.set_joint_torques([0.0] * len(robot._state["joint_torques"]))

    def _handle_reset(self, robot, params):
        if robot:
            n = len(robot._state["joint_positions"])
            robot._state["joint_positions"] = [0.0] * n
            robot._state["joint_velocities"] = [0.0] * n
            robot._state["joint_torques"] = [0.0] * n

    def _handle_gripper(self, robot, params):
        if robot and robot.end_effectors:
            gripper = robot.end_effectors[0]
            if params.get("state") == "open":
                gripper.open()
            elif params.get("state") == "close":
                gripper.close()


# ══════════════════════════════════════════
# SPEECH INTERFACE
# ══════════════════════════════════════════


class SpeechInterface:
    """Speech-to-text → command → action pipeline.

    Uses system microphone for speech recognition.
    Requires: pip install SpeechRecognition pyaudio

    Usage:
        speech = SpeechInterface(parser, mapper)
        speech.listen_once()  # single command
        speech.listen_continuous()  # continuous listening
    """

    def __init__(
        self,
        parser: Optional[CommandParser] = None,
        mapper: Optional[ActionMapper] = None,
        language: str = "en-US",
    ):
        self.parser = parser or CommandParser()
        self.mapper = mapper
        self.language = language
        self._recognizer = None
        self._microphone = None

        try:
            import speech_recognition as sr

            self._recognizer = sr.Recognizer()
            self._microphone = sr.Microphone()
        except ImportError:
            pass

    @property
    def available(self) -> bool:
        return self._recognizer is not None

    def listen_once(self, timeout: float = 5.0) -> Optional[ParsedCommand]:
        """Listen for a single voice command."""
        if not self.available:
            print("  [speech] speech_recognition not installed")
            return None

        import speech_recognition as sr

        with self._microphone as source:
            self._recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                audio = self._recognizer.listen(source, timeout=timeout)
                text = self._recognizer.recognize_google(audio, language=self.language)
                print(f"  [speech] Heard: '{text}'")
                command = self.parser.parse(text)
                if self.mapper:
                    self.mapper.execute(command)
                return command
            except sr.WaitTimeoutError:
                return None
            except sr.UnknownValueError:
                print("  [speech] Could not understand")
                return None
            except sr.RequestError as e:
                print(f"  [speech] Recognition error: {e}")
                return None

    def text_command(self, text: str) -> ParsedCommand:
        """Process a text command (no speech recognition needed)."""
        command = self.parser.parse(text)
        if self.mapper:
            self.mapper.execute(command)
        return command
