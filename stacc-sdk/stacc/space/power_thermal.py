"""STACC Space: Planetary Power & Thermal Management (MMRTG, Solar, Sub-Zero Heaters)."""

import numpy as np
from typing import Dict, Any


class PlanetaryPowerThermalSystem:
    """
    Simulates planetary power generation (MMRTG / Solar Arrays) and thermal management
    in extreme Martian environments (-120°C to +20°C).
    """

    def __init__(
        self,
        power_source: str = "MMRTG",  # 'MMRTG' or 'SOLAR'
        battery_capacity_Wh: float = 1200.0,
        initial_soc: float = 0.90,
    ):
        self.power_source = power_source.upper()
        self.battery_capacity_Wh = battery_capacity_Wh
        self.current_energy_Wh = battery_capacity_Wh * initial_soc
        self.ambient_temp_C = -65.0  # Typical Martian night/day average
        self.internal_avionics_temp_C = 15.0
        self.heater_active = False

        # MMRTG constants (e.g. Curiosity/Perseverance 110W electrical, 2000W thermal)
        self.mmrtg_electrical_watts = 110.0
        self.mmrtg_thermal_watts = 2000.0

        # Solar constants
        self.solar_area_m2 = 1.8
        self.solar_efficiency = 0.28
        self.mars_dust_optical_depth_tau = 0.60  # Atmospheric dust factor

    def step(
        self,
        base_power_draw_W: float,
        actuator_power_draw_W: float,
        sun_elevation_deg: float = 45.0,
        dt_hours: float = 0.01,
    ) -> Dict[str, Any]:
        """
        Step power & thermal budget simulation.
        """
        # 1. Power Generation
        if self.power_source == "MMRTG":
            generated_power_W = self.mmrtg_electrical_watts
        else:
            # Solar generation: Solar constant at Mars ~ 590 W/m^2
            irradiance = 590.0 * max(0.0, np.sin(np.radians(sun_elevation_deg))) * np.exp(-self.mars_dust_optical_depth_tau)
            generated_power_W = self.solar_area_m2 * irradiance * self.solar_efficiency

        # 2. Thermal Management & Survival Heaters
        # Heat loss to ambient: Q_loss = U * A * (T_in - T_amb)
        heat_loss_W = 0.85 * (self.internal_avionics_temp_C - self.ambient_temp_C)
        heater_power_W = 0.0

        if self.internal_avionics_temp_C < -10.0:
            # Turn on survival heaters
            self.heater_active = True
            heater_power_W = 60.0
        elif self.internal_avionics_temp_C > 18.0:
            self.heater_active = False

        # Update internal temperature
        thermal_input_W = (self.mmrtg_thermal_watts * 0.1 if self.power_source == "MMRTG" else 0.0) + heater_power_W
        dT = ((thermal_input_W - heat_loss_W) / 850.0) * (dt_hours * 3600.0)
        self.internal_avionics_temp_C = float(np.clip(self.internal_avionics_temp_C + dT, -40.0, 45.0))

        # 3. Energy Balance
        total_load_W = base_power_draw_W + actuator_power_draw_W + heater_power_W
        net_power_W = generated_power_W - total_load_W
        
        self.current_energy_Wh += net_power_W * dt_hours
        self.current_energy_Wh = float(np.clip(self.current_energy_Wh, 0.0, self.battery_capacity_Wh))
        soc = self.current_energy_Wh / self.battery_capacity_Wh

        return {
            "power_source": self.power_source,
            "generated_power_W": round(float(generated_power_W), 1),
            "total_load_W": round(float(total_load_W), 1),
            "net_power_W": round(float(net_power_W), 1),
            "battery_soc_percent": round(soc * 100.0, 1),
            "battery_energy_Wh": round(self.current_energy_Wh, 1),
            "internal_temp_C": round(self.internal_avionics_temp_C, 1),
            "heater_active": self.heater_active,
        }
