"""STACC Sim: Sim-to-Real Domain Randomization & Physics Noise Injector."""

import numpy as np
from typing import Dict, Any


class DomainRandomizer:
    """
    Randomizes physics simulation parameters for robust Sim-to-Real transfer.
    Perturbs mass, inertia, friction coefficients, actuator latency, and sensor noise.
    """

    def __init__(
        self,
        mass_perturb_range: float = 0.15,       # +/- 15%
        friction_perturb_range: float = 0.25,   # +/- 25%
        latency_jitter_ms_range: float = 3.0,   # +/- 3ms
        encoder_noise_std_deg: float = 0.05,
    ):
        self.mass_range = mass_perturb_range
        self.fric_range = friction_perturb_range
        self.latency_range = latency_jitter_ms_range
        self.encoder_noise = encoder_noise_std_deg

    def randomize_parameters(self, nominal_masses: np.ndarray, nominal_friction: float) -> Dict[str, Any]:
        """Generate a randomized environment rollout sample."""
        mass_scales = 1.0 + np.random.uniform(-self.mass_range, self.mass_range, size=nominal_masses.shape)
        perturbed_masses = nominal_masses * mass_scales

        fric_scale = 1.0 + np.random.uniform(-self.fric_range, self.fric_range)
        perturbed_friction = max(0.05, nominal_friction * fric_scale)

        added_latency_ms = max(0.0, np.random.normal(1.5, self.latency_range))

        return {
            "perturbed_masses_kg": np.round(perturbed_masses, 3).tolist(),
            "friction_coefficient": round(float(perturbed_friction), 3),
            "transmission_latency_ms": round(float(added_latency_ms), 2),
            "encoder_noise_deg": self.encoder_noise,
        }

    def inject_sensor_noise(self, true_angles_deg: np.ndarray) -> np.ndarray:
        """Inject Gaussian noise into joint position readings."""
        noise = np.random.normal(0.0, self.encoder_noise, size=true_angles_deg.shape)
        return true_angles_deg + noise
