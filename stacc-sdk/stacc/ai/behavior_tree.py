"""Universal behavior tree — task planning and decision making for any robot.

Behavior trees are the standard for game AI and increasingly for robot autonomy.
They compose simple behaviors into complex task sequences using a tree of nodes.

Node types:
- Sequence: run children in order, fail if any fails
- Selector: try children in order, succeed if any succeeds
- Parallel: run all children simultaneously
- Action: execute a function (your robot code)
- Condition: check a boolean (sensor reading, state check)
- Decorator: modify a child (repeat, invert, timeout, retry)

Usage:
    from stacc.ai.behavior_tree import *

    tree = Sequence("pick_and_place", [
        Action("move_to_pick", move_to_pick_fn),
        Action("close_gripper", close_gripper_fn),
        Action("move_to_place", move_to_place_fn),
        Action("open_gripper", open_gripper_fn),
    ])

    @robot.on_update
    def control(dt):
        tree.tick(dt)
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Any, Callable, List, Optional


class Status(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class Node:
    """Base behavior tree node."""

    def __init__(self, name: str = "node"):
        self.name = name
        self.status = Status.RUNNING
        self.parent: Optional[Node] = None

    def tick(self, dt: float = 0.0) -> Status:
        raise NotImplementedError

    def reset(self) -> None:
        self.status = Status.RUNNING

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}('{self.name}', {self.status.value})"


class Action(Node):
    """Leaf node that executes a function.

    The function should return Status.SUCCESS, Status.FAILURE, or Status.RUNNING.
    If it returns None or True, it's treated as SUCCESS.
    If it returns False, it's treated as FAILURE.

    Usage:
        def move_forward(dt):
            robot.set_speed(0.5, 0.5)
            if robot.position[0] > target_x:
                return Status.SUCCESS
            return Status.RUNNING

        node = Action("move_forward", move_forward)
    """

    def __init__(self, name: str, action_fn: Callable):
        super().__init__(name)
        self.action_fn = action_fn

    def tick(self, dt: float = 0.0) -> Status:
        result = self.action_fn(dt)
        if result is None or result is True:
            self.status = Status.SUCCESS
        elif result is False:
            self.status = Status.FAILURE
        elif isinstance(result, Status):
            self.status = result
        else:
            self.status = Status.SUCCESS
        return self.status


class Condition(Node):
    """Leaf node that checks a boolean condition.

    Usage:
        node = Condition("battery_ok", lambda dt: robot.battery > 20)
    """

    def __init__(self, name: str, check_fn: Callable):
        super().__init__(name)
        self.check_fn = check_fn

    def tick(self, dt: float = 0.0) -> Status:
        result = self.check_fn(dt)
        self.status = Status.SUCCESS if result else Status.FAILURE
        return self.status


class Wait(Node):
    """Wait for a specified duration."""

    def __init__(self, name: str, duration: float):
        super().__init__(name)
        self.duration = duration
        self._elapsed = 0.0

    def tick(self, dt: float = 0.0) -> Status:
        self._elapsed += dt
        if self._elapsed >= self.duration:
            self.status = Status.SUCCESS
        else:
            self.status = Status.RUNNING
        return self.status

    def reset(self) -> None:
        super().reset()
        self._elapsed = 0.0


class Sequence(Node):
    """Run children in order. Fails immediately if any child fails.

    Like an AND gate: all must succeed for the sequence to succeed.
    If a child returns RUNNING, the sequence pauses there and resumes next tick.
    """

    def __init__(self, name: str, children: List[Node]):
        super().__init__(name)
        self.children = children
        self._current = 0
        for child in children:
            child.parent = self

    def tick(self, dt: float = 0.0) -> Status:
        while self._current < len(self.children):
            child = self.children[self._current]
            result = child.tick(dt)

            if result == Status.FAILURE:
                self.status = Status.FAILURE
                return self.status
            elif result == Status.RUNNING:
                self.status = Status.RUNNING
                return self.status

            self._current += 1

        self.status = Status.SUCCESS
        return self.status

    def reset(self) -> None:
        super().reset()
        self._current = 0
        for child in self.children:
            child.reset()


class Selector(Node):
    """Try children in order. Succeeds immediately if any child succeeds.

    Like an OR gate: at least one must succeed.
    Tries each child until one succeeds or returns RUNNING.
    """

    def __init__(self, name: str, children: List[Node]):
        super().__init__(name)
        self.children = children
        self._current = 0
        for child in children:
            child.parent = self

    def tick(self, dt: float = 0.0) -> Status:
        while self._current < len(self.children):
            child = self.children[self._current]
            result = child.tick(dt)

            if result == Status.SUCCESS:
                self.status = Status.SUCCESS
                return self.status
            elif result == Status.RUNNING:
                self.status = Status.RUNNING
                return self.status

            self._current += 1

        self.status = Status.FAILURE
        return self.status

    def reset(self) -> None:
        super().reset()
        self._current = 0
        for child in self.children:
            child.reset()


class Parallel(Node):
    """Run all children simultaneously.

    Args:
        success_threshold: Number of children that must succeed (default: all)
        failure_threshold: Number of children that must fail to fail (default: 1)
    """

    def __init__(
        self,
        name: str,
        children: List[Node],
        success_threshold: Optional[int] = None,
        failure_threshold: int = 1,
    ):
        super().__init__(name)
        self.children = children
        self.success_threshold = success_threshold or len(children)
        self.failure_threshold = failure_threshold
        for child in children:
            child.parent = self

    def tick(self, dt: float = 0.0) -> Status:
        successes = 0
        failures = 0

        for child in self.children:
            if child.status == Status.SUCCESS:
                successes += 1
                continue
            if child.status == Status.FAILURE:
                failures += 1
                continue

            result = child.tick(dt)
            if result == Status.SUCCESS:
                successes += 1
            elif result == Status.FAILURE:
                failures += 1

        if successes >= self.success_threshold:
            self.status = Status.SUCCESS
        elif failures >= self.failure_threshold:
            self.status = Status.FAILURE
        else:
            self.status = Status.RUNNING

        return self.status

    def reset(self) -> None:
        super().reset()
        for child in self.children:
            child.reset()


# ── Decorators ──


class Repeat(Node):
    """Repeat a child node N times (or forever if count=-1)."""

    def __init__(self, name: str, child: Node, count: int = -1):
        super().__init__(name)
        self.child = child
        self.child.parent = self
        self.count = count
        self._completed = 0

    def tick(self, dt: float = 0.0) -> Status:
        result = self.child.tick(dt)

        if result == Status.RUNNING:
            self.status = Status.RUNNING
            return self.status

        if result == Status.FAILURE:
            self.status = Status.FAILURE
            return self.status

        self._completed += 1
        if self.count > 0 and self._completed >= self.count:
            self.status = Status.SUCCESS
            return self.status

        self.child.reset()
        self.status = Status.RUNNING
        return self.status

    def reset(self) -> None:
        super().reset()
        self._completed = 0
        self.child.reset()


class Invert(Node):
    """Invert child result: SUCCESS ↔ FAILURE. RUNNING stays RUNNING."""

    def __init__(self, name: str, child: Node):
        super().__init__(name)
        self.child = child
        self.child.parent = self

    def tick(self, dt: float = 0.0) -> Status:
        result = self.child.tick(dt)
        if result == Status.SUCCESS:
            self.status = Status.FAILURE
        elif result == Status.FAILURE:
            self.status = Status.SUCCESS
        else:
            self.status = Status.RUNNING
        return self.status

    def reset(self) -> None:
        super().reset()
        self.child.reset()


class Timeout(Node):
    """Fail if child doesn't complete within time limit."""

    def __init__(self, name: str, child: Node, time_limit: float):
        super().__init__(name)
        self.child = child
        self.child.parent = self
        self.time_limit = time_limit
        self._elapsed = 0.0

    def tick(self, dt: float = 0.0) -> Status:
        self._elapsed += dt
        if self._elapsed >= self.time_limit:
            self.status = Status.FAILURE
            return self.status

        self.status = self.child.tick(dt)
        return self.status

    def reset(self) -> None:
        super().reset()
        self._elapsed = 0.0
        self.child.reset()


class Retry(Node):
    """Retry child up to N times on failure."""

    def __init__(self, name: str, child: Node, max_retries: int = 3):
        super().__init__(name)
        self.child = child
        self.child.parent = self
        self.max_retries = max_retries
        self._retries = 0

    def tick(self, dt: float = 0.0) -> Status:
        result = self.child.tick(dt)

        if result == Status.FAILURE:
            self._retries += 1
            if self._retries > self.max_retries:
                self.status = Status.FAILURE
                return self.status
            self.child.reset()
            self.status = Status.RUNNING
            return self.status

        self.status = result
        return self.status

    def reset(self) -> None:
        super().reset()
        self._retries = 0
        self.child.reset()


class AlwaysSucceed(Node):
    """Convert any result to SUCCESS (except RUNNING)."""

    def __init__(self, name: str, child: Node):
        super().__init__(name)
        self.child = child
        self.child.parent = self

    def tick(self, dt: float = 0.0) -> Status:
        result = self.child.tick(dt)
        if result == Status.RUNNING:
            self.status = Status.RUNNING
        else:
            self.status = Status.SUCCESS
        return self.status

    def reset(self) -> None:
        super().reset()
        self.child.reset()
