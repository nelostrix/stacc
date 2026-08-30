"""STACC Terramechanics: Bekker-Wong Soil-Wheel Interaction & Slip Estimation."""

import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass


@dataclass
class SoilParameters:
    """Terramechanics properties of planetary terrain (e.g. Martian regolith or dry sand)."""
    name: str = "Martian Regolith"
    kc: float = 1.4e3        # Cohesive modulus of deformation (N/m^(n+1))
    kphi: float = 8.2e5      # Frictional modulus of deformation (N/m^(n+2))
    n: float = 0.9           # Soil exponent of sinkage
    c: float = 0.17e3        # Soil cohesion (Pa)
    phi_deg: float = 35.0    # Internal angle of friction (deg)
    K: float = 0.025         # Shear deformation modulus (m)
    density: float = 1520.0  # Bulk density (kg/m^3)


class TerramechanicsModel:
    """
    Bekker-Wong Wheel-Soil Interaction and Terramechanics Model.
    Computes sinkage, shear stress, drawbar pull, and wheel slip estimation.
    """

    def __init__(self, soil: SoilParameters = None):
        self.soil = soil or SoilParameters()

    def compute_sinkage(self, normal_load_N: float, wheel_radius_m: float, wheel_width_m: float) -> float:
        """
        Compute wheel sinkage z (m) into soft regolith using Bekker's equation:
        W = (k_c/b + k_phi) * (z^(n+1) / (n+1)) * b * sqrt(D*z)
        """
        b = wheel_width_m
        D = 2.0 * wheel_radius_m
        kc = self.soil.kc
        kphi = self.soil.kphi
        n = self.soil.n

        k_eq = (kc / b) + kphi
        # Numerical inversion for sinkage z
        z_guess = 0.02  # 2 cm initial guess
        for _ in range(15):
            f = k_eq * b * np.sqrt(D) * (z_guess ** (n + 0.5)) / (n + 1.0) - normal_load_N
            f_prime = k_eq * b * np.sqrt(D) * (n + 0.5) * (z_guess ** (n - 0.5)) / (n + 1.0)
            if abs(f_prime) < 1e-6:
                break
            dz = f / f_prime
            z_guess = max(1e-4, z_guess - dz)
            if abs(dz) < 1e-5:
                break

        return float(z_guess)

    def compute_slip(self, wheel_speed_rad_s: float, wheel_radius_m: float, chassis_velocity_m_s: float) -> float:
        """
        Calculate longitudinal wheel slip ratio s in [-1.0, 1.0]:
        s = (r*omega - v) / (r*omega) for positive drive (r*omega > v)
        """
        v_wheel = wheel_speed_rad_s * wheel_radius_m
        v_actual = chassis_velocity_m_s

        if abs(v_wheel) < 1e-4 and abs(v_actual) < 1e-4:
            return 0.0

        if v_wheel >= v_actual:
            if abs(v_wheel) < 1e-4:
                return 0.0
            slip = (v_wheel - v_actual) / v_wheel
        else:
            if abs(v_actual) < 1e-4:
                return 0.0
            slip = -(v_actual - v_wheel) / v_actual

        return float(np.clip(slip, -1.0, 1.0))

    def compute_tractive_forces(
        self,
        normal_load_N: float,
        wheel_radius_m: float,
        wheel_width_m: float,
        slip_ratio: float,
    ) -> Dict[str, float]:
        """
        Compute rolling compaction resistance, thrust (traction), and net drawbar pull.
        """
        z = self.compute_sinkage(normal_load_N, wheel_radius_m, wheel_width_m)
        b = wheel_width_m
        n = self.soil.n
        kc = self.soil.kc
        kphi = self.soil.kphi
        c = self.soil.c
        phi_rad = np.radians(self.soil.phi_deg)
        K = self.soil.K

        # Compaction resistance Rc
        k_eq = (kc / b) + kphi
        Rc = (k_eq * b * (z ** (n + 1))) / (n + 1.0)

        # Contact patch length
        L_contact = np.sqrt(2 * wheel_radius_m * z - z**2) if (2 * wheel_radius_m * z >= z**2) else np.sqrt(2 * wheel_radius_m * z)
        A_contact = b * max(0.01, L_contact)

        # Maximum shear strength tau_max = c + sigma_n * tan(phi)
        sigma_n = normal_load_N / A_contact
        tau_max = c + sigma_n * np.tan(phi_rad)

        # Janosi-Hanamoto shear equation: tau = tau_max * (1 - exp(-j / K))
        j_slip = abs(slip_ratio) * L_contact
        shear_stress = tau_max * (1.0 - np.exp(-j_slip / K))

        # Gross tractive thrust H
        thrust_H = A_contact * shear_stress

        # Net Drawbar Pull DP = H - Rc
        drawbar_pull = max(-normal_load_N, thrust_H - Rc)

        return {
            "sinkage_m": z,
            "compaction_resistance_N": float(Rc),
            "thrust_N": float(thrust_H),
            "drawbar_pull_N": float(drawbar_pull),
            "slip_ratio": slip_ratio,
        }
