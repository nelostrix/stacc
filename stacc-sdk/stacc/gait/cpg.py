# ══════════════════════════════════════════
# CPG — Central Pattern Generator
# ══════════════════════════════════════════
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple


# ── Coupling pattern presets ─────────────
_COUPLING_PRESETS: Dict[str, List[float]] = {
    "walk":   [0.0, 0.25, 0.5, 0.75],
    "trot":   [0.0, 0.5, 0.5, 0.0],
    "bound":  [0.0, 0.0, 0.5, 0.5],
    "gallop": [0.0, 0.1, 0.5, 0.6],
    "pace":   [0.0, 0.5, 0.0, 0.5],
}


class CPGOscillator:
    """Single CPG oscillator — a phase-coupled neural oscillator.

    Args:
        frequency: Natural oscillation frequency in Hz
        amplitude: Output amplitude
        phase_offset: Initial phase offset in radians
    """

    def __init__(
        self,
        frequency: float = 2.0,
        amplitude: float = 1.0,
        phase_offset: float = 0.0,
    ):
        self.frequency = frequency
        self.amplitude = amplitude
        self.phase = phase_offset
        self.output: float = 0.0

    def step(self, dt: float, coupling_input: float = 0.0) -> float:
        """Advance oscillator by dt seconds with optional coupling.

        Args:
            dt: Time step in seconds
            coupling_input: Phase coupling from other oscillators

        Returns:
            Current oscillator output value
        """
        self.phase += 2.0 * math.pi * self.frequency * dt + coupling_input * dt
        self.output = self.amplitude * math.sin(self.phase)
        return self.output

    def reset(self) -> None:
        """Reset oscillator to initial state."""
        self.phase = 0.0
        self.output = 0.0


class CPGGenerator:
    """Central Pattern Generator — adaptive oscillator-based gait.

    A network of coupled oscillators that generates rhythmic locomotion
    patterns. Adapts to terrain and speed changes naturally through
    phase coupling dynamics.

    Provides a public API consistent with GaitGenerator (``compute()``,
    ``set_speed()``, ``reset()``) but uses oscillator coupling instead
    of phase-based foot placement.

    Args:
        num_legs: Number of legs (default 4)
        frequency: Base oscillation frequency in Hz
        coupling_strength: How strongly oscillators influence each other
        step_height: Maximum foot lift in meters
        step_length: Forward step length in meters
        body_height: Body height above ground in meters
        pattern: Coupling pattern name ("trot", "walk", "bound", "gallop",
                 "pace") or ``None`` for automatic distribution
    """

    def __init__(
        self,
        num_legs: int = 4,
        frequency: float = 2.0,
        coupling_strength: float = 2.0,
        step_height: float = 0.04,
        step_length: float = 0.08,
        body_height: float = 0.15,
        pattern: Optional[str] = None,
    ):
        if num_legs < 1:
            raise ValueError(f"num_legs must be >= 1, got {num_legs}")

        self.num_legs = num_legs
        self.frequency = frequency
        self.step_height = step_height
        self.step_length = step_length
        self.body_height = body_height
        self.coupling_strength = coupling_strength

        # Resolve phase offsets from pattern or defaults
        phases = self._resolve_phases(num_legs, pattern)

        self.oscillators = [
            CPGOscillator(frequency=frequency, phase_offset=phases[i])
            for i in range(num_legs)
        ]

        # Coupling matrix (who influences whom)
        self._coupling = self._build_coupling_matrix(num_legs, coupling_strength)

    # ── Public API ──────────────────────

    def compute(self, time: float, dt: float = 0.002) -> List[List[float]]:
        """Compute foot targets from CPG outputs.

        Args:
            time: Current simulation time in seconds
            dt: Time step for oscillator integration (default 0.002)

        Returns:
            List of [x, y, z] foot positions, one per leg
        """
        outputs = self.step(dt)
        targets: List[List[float]] = []

        for i in range(self.num_legs):
            angle = 2.0 * math.pi * i / self.num_legs + math.pi / self.num_legs
            radius = 0.12
            default_x = radius * math.cos(angle)
            default_y = radius * math.sin(angle)

            x = default_x + outputs[i] * self.step_length * 0.5
            z_lift = max(0.0, outputs[i]) * self.step_height
            z = -self.body_height + z_lift

            targets.append([x, default_y, z])

        return targets

    def step(self, dt: float) -> List[float]:
        """Advance all oscillators by dt. Returns oscillator outputs."""
        outputs: List[float] = []
        for i, osc in enumerate(self.oscillators):
            coupling = sum(
                self._coupling[i][j] * math.sin(self.oscillators[j].phase - osc.phase)
                for j in range(self.num_legs)
            )
            osc.step(dt, coupling)
            outputs.append(osc.output)
        return outputs

    def set_speed(self, speed: float) -> None:
        """Scale gait speed by adjusting oscillator frequency.

        Args:
            speed: Speed scalar (1.0 = base frequency)
        """
        freq = max(0.5, speed * 3.0)
        self.frequency = freq
        for osc in self.oscillators:
            osc.frequency = freq

    def set_frequency(self, freq: float) -> None:
        """Set oscillator base frequency directly."""
        self.frequency = freq
        for osc in self.oscillators:
            osc.frequency = freq

    def set_amplitude(self, amp: float) -> None:
        """Set oscillator amplitude for all legs."""
        for osc in self.oscillators:
            osc.amplitude = amp

    def set_direction(self, heading: float) -> None:
        """Set preferred direction (not implemented for CPG)."""
        pass

    def reset(self) -> None:
        """Reset all oscillators to initial state."""
        for osc in self.oscillators:
            osc.reset()

    def set_coupling_strength(self, strength: float) -> None:
        """Adjust coupling strength between oscillators."""
        self.coupling_strength = strength
        self._coupling = self._build_coupling_matrix(self.num_legs, strength)

    def set_pattern(self, pattern: str) -> None:
        """Switch to a named coupling pattern.

        Args:
            pattern: One of "walk", "trot", "bound", "gallop", "pace"
        """
        phases = _COUPLING_PRESETS.get(pattern)
        if phases is None:
            raise ValueError(
                f"Unknown pattern '{pattern}'. "
                f"Available: {list(_COUPLING_PRESETS)}"
            )
        if len(phases) != self.num_legs:
            raise ValueError(
                f"Pattern '{pattern}' expects {len(phases)} legs, "
                f"but CPG has {self.num_legs}"
            )
        for i, osc in enumerate(self.oscillators):
            osc.phase = phases[i]

    # ── Internal helpers ────────────────

    @staticmethod
    def _resolve_phases(num_legs: int, pattern: Optional[str]) -> List[float]:
        if pattern is not None:
            phases = _COUPLING_PRESETS.get(pattern)
            if phases is None:
                raise ValueError(
                    f"Unknown pattern '{pattern}'. "
                    f"Available: {list(_COUPLING_PRESETS)}"
                )
            if len(phases) != num_legs:
                raise ValueError(
                    f"Pattern '{pattern}' expects {len(phases)} legs, "
                    f"got {num_legs}"
                )
            return phases

        # Automatic distribution
        if num_legs == 2:
            return [0.0, math.pi]
        if num_legs == 4:
            return [0.0, math.pi, math.pi, 0.0]  # trot-like
        if num_legs == 6:
            return [0.0, math.pi, 0.0, math.pi, 0.0, math.pi]
        return [2.0 * math.pi * i / num_legs for i in range(num_legs)]

    @staticmethod
    def _build_coupling_matrix(
        num_legs: int, strength: float
    ) -> List[List[float]]:
        matrix: List[List[float]] = []
        for i in range(num_legs):
            row: List[float] = []
            for j in range(num_legs):
                row.append(strength / num_legs if i != j else 0.0)
            matrix.append(row)
        return matrix
