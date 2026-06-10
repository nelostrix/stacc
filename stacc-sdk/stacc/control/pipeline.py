from __future__ import annotations

from typing import Any, Callable, Optional


class ControllerStage:
    """A single named stage in a controller pipeline.

    Args:
        name: Stage identifier (for debugging and targeted overrides).
        controller: The controller object. Must have a ``compute(**kwargs)`` method.
        enabled: Whether this stage is active. Can be toggled at runtime.
        skip_if: Optional function. If provided and returns True, this stage
            is skipped. Signature: skip_if(stage_name, value, **kwargs) -> bool.
    """

    def __init__(
        self,
        name: str,
        controller: Any,
        enabled: bool = True,
        skip_if: Optional[Callable[..., bool]] = None,
    ):
        self.name = name
        self.controller = controller
        self.enabled = enabled
        self.skip_if = skip_if

    def compute(self, value: Any, **kwargs) -> Any:
        if not self.enabled:
            return value
        if self.skip_if is not None and self.skip_if(self.name, value, **kwargs):
            return value
        return self.controller.compute(value=value, **kwargs)

    def reset(self):
        if hasattr(self.controller, "reset"):
            self.controller.reset()

    def __repr__(self) -> str:
        status = "enabled" if self.enabled else "disabled"
        return f"Stage('{self.name}', {type(self.controller).__name__}, {status})"


class ControllerPipeline:
    """A composable pipeline of named control stages.

    Passes the output of each stage as the input to the next. Supports
    per-stage enable/disable and conditional skipping.

    Usage:
        pipeline = ControllerPipeline(
            ControllerStage("smooth", MovingAverageFilter(5)),
            ControllerStage("pid", PIDController(kp=1.0, ki=0.1)),
            ControllerStage(
                "limit", RateLimiter(max_rate=10.0),
                skip_if=lambda name, value, **kw: abs(value) < 0.01,
            ),
        )
        output = pipeline.compute(measurement, dt=0.01)
    """

    def __init__(self, *stages: ControllerStage):
        if not all(isinstance(s, ControllerStage) for s in stages):
            raise TypeError("All pipeline elements must be ControllerStage instances")
        self.stages: list[ControllerStage] = list(stages)
        self._stage_map: dict[str, int] = {
            s.name: i for i, s in enumerate(stages)
        }

    def add(self, stage: ControllerStage, index: Optional[int] = None):
        """Add a stage at a specific index (or at the end)."""
        if index is None:
            self.stages.append(stage)
            self._stage_map[stage.name] = len(self.stages) - 1
        else:
            self.stages.insert(index, stage)
            self._stage_map = {s.name: i for i, s in enumerate(self.stages)}

    def remove(self, name: str):
        """Remove a stage by name."""
        idx = self._stage_map.get(name)
        if idx is not None:
            self.stages.pop(idx)
            self._stage_map = {s.name: i for i, s in enumerate(self.stages)}

    def get(self, name: str) -> Optional[ControllerStage]:
        """Get a stage by name."""
        idx = self._stage_map.get(name)
        return self.stages[idx] if idx is not None else None

    def enable(self, name: str):
        stage = self.get(name)
        if stage:
            stage.enabled = True

    def disable(self, name: str):
        stage = self.get(name)
        if stage:
            stage.enabled = False

    def compute(self, *args, **kwargs) -> Any:
        value = None
        for stage in self.stages:
            if value is None:
                value = stage.controller.compute(*args, **kwargs)
            else:
                value = stage.compute(value=value, **kwargs)
        return value

    def reset(self):
        for stage in self.stages:
            stage.reset()

    def __len__(self) -> int:
        return len(self.stages)

    def __repr__(self) -> str:
        stages_repr = ", ".join(s.name for s in self.stages)
        return f"ControllerPipeline({stages_repr})"
