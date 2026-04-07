# ══════════════════════════════════════════
# CPG — Central Pattern Generator
# ══════════════════════════════════════════
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple


class CPGOscillator:
    """Single CPG oscillator — a phase-coupled neural oscillator."""

    def __init__(
        self, frequency: float = 2.0, amplitude: float = 1.0, phase_offset: float = 0.0
    ):
        self.frequency = frequency
        self.amplitude = amplitude
        self.phase = phase_offset
        self.output = 0.0

    def step(self, dt: float, coupling_input: float = 0.0):
        self.phase += 2 * math.pi * self.frequency * dt + coupling_input * dt
        self.output = self.amplitude * math.sin(self.phase)
        return self.output


class CPGGenerator:
    """Central Pattern Generator — adaptive oscillator-based gait.

    A network of coupled oscillators that generates rhythmic locomotion
    patterns. Adapts to terrain and speed changes naturally.

    Args:
        num_legs: Number of legs
        frequency: Base oscillation frequency
        coupling_strength: How strongly oscillators influence each other
    """

    def __init__(
        self,
        num_legs: int = 4,
        frequency: float = 2.0,
        coupling_strength: float = 2.0,
        step_height: float = 0.04,
        step_length: float = 0.08,
        body_height: float = 0.15,
    ):
        self.num_legs = num_legs
        self.step_height = step_height
        self.step_length = step_length
        self.body_height = body_height
        self.coupling_strength = coupling_strength

        # Default phase coupling for trot
        if num_legs == 4:
            phases = [0.0, math.pi, math.pi, 0.0]
        elif num_legs == 6:
            phases = [0, math.pi, 0, math.pi, 0, math.pi]
        elif num_legs == 2:
            phases = [0, math.pi]
        else:
            phases = [2 * math.pi * i / num_legs for i in range(num_legs)]

        self.oscillators = [
            CPGOscillator(frequency=frequency, phase_offset=phases[i])
            for i in range(num_legs)
        ]

        # Coupling matrix (who influences whom)
        self._coupling = [[0.0] * num_legs for _ in range(num_legs)]
        for i in range(num_legs):
            for j in range(num_legs):
                if i != j:
                    self._coupling[i][j] = coupling_strength / num_legs

    def step(self, dt: float) -> List[float]:
        """Advance all oscillators by dt. Returns oscillator outputs."""
        outputs = []
        for i, osc in enumerate(self.oscillators):
            coupling = sum(
                self._coupling[i][j] * math.sin(self.oscillators[j].phase - osc.phase)
                for j in range(self.num_legs)
            )
            osc.step(dt, coupling)
            outputs.append(osc.output)
        return outputs

    def compute(self, time: float, dt: float = 0.002) -> List[List[float]]:
        """Compute foot targets from CPG outputs."""
        outputs = self.step(dt)
        targets = []

        for i in range(self.num_legs):
            angle = 2 * math.pi * i / self.num_legs + math.pi / self.num_legs
            radius = 0.12
            default_x = radius * math.cos(angle)
            default_y = radius * math.sin(angle)

            # Use oscillator output for foot trajectory
            x = default_x + outputs[i] * self.step_length * 0.5
            z_lift = max(0, outputs[i]) * self.step_height
            z = -self.body_height + z_lift

            targets.append([x, default_y, z])

        return targets

    def set_frequency(self, freq: float):
        for osc in self.oscillators:
            osc.frequency = freq

    def set_amplitude(self, amp: float):
        for osc in self.oscillators:
            osc.amplitude = amp
