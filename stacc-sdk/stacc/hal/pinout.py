"""STACC HAL: Declarative Microcontroller Pinout Mapper & Wiring Generator."""

from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field


@dataclass
class BoardPinout:
    board_name: str
    digital_pins: List[int]
    pwm_pins: List[int]
    analog_pins: List[int]
    i2c_bus: Dict[str, int]
    spi_bus: Dict[str, int]
    uart_pins: Dict[str, int]


BOARD_DEFINITIONS = {
    "arduino_uno": BoardPinout(
        board_name="Arduino Uno R3 (ATmega328P)",
        digital_pins=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
        pwm_pins=[3, 5, 6, 9, 10, 11],
        analog_pins=[14, 15, 16, 17, 18, 19],  # A0..A5
        i2c_bus={"sda": 18, "scl": 19},
        spi_bus={"mosi": 11, "miso": 12, "sck": 13, "ss": 10},
        uart_pins={"rx": 0, "tx": 1},
    ),
    "esp32_wroom": BoardPinout(
        board_name="ESP32-WROOM-32 (Dual Core 240MHz)",
        digital_pins=[2, 4, 5, 12, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33],
        pwm_pins=[2, 4, 5, 12, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33],
        analog_pins=[34, 35, 36, 39, 32, 33, 25, 26, 27, 14, 12, 13, 15, 4, 2],
        i2c_bus={"sda": 21, "scl": 22},
        spi_bus={"mosi": 23, "miso": 19, "sck": 18, "ss": 5},
        uart_pins={"rx": 3, "tx": 1},
    ),
    "stm32_blackpill": BoardPinout(
        board_name="STM32F401 BlackPill (ARM Cortex-M4)",
        digital_pins=list(range(0, 32)),
        pwm_pins=[1, 2, 3, 6, 7, 8, 9, 10],
        analog_pins=[0, 1, 2, 3, 4, 5, 6, 7],
        i2c_bus={"sda": 7, "scl": 6},
        spi_bus={"mosi": 15, "miso": 14, "sck": 13, "ss": 12},
        uart_pins={"rx": 10, "tx": 9},
    ),
}


class PinoutManager:
    """
    Manages and validates hardware pin assignments for line followers, rovers, and arms.
    Checks for conflict overlaps and generates readable wiring schematics.
    """

    def __init__(self, target_board: str = "esp32_wroom"):
        self.target_board = target_board
        self.board_spec = BOARD_DEFINITIONS.get(target_board, BOARD_DEFINITIONS["esp32_wroom"])
        self.assigned_pins: Dict[str, int] = {}

    def assign_pin(self, component_name: str, pin: int, function_type: str = "digital") -> bool:
        """Assign pin to a component and verify capability."""
        # Check collision
        for comp, p in self.assigned_pins.items():
            if p == pin and comp != component_name:
                raise ValueError(f"Pin collision! Pin {pin} is already assigned to '{comp}'.")

        # Verify capability
        if function_type == "pwm" and pin not in self.board_spec.pwm_pins:
            raise ValueError(f"Pin {pin} on {self.board_spec.board_name} does not support Hardware PWM.")

        self.assigned_pins[component_name] = pin
        return True

    def generate_wiring_table(self) -> List[Dict[str, Any]]:
        """Generate formatted wiring list for hardware assembly."""
        table = []
        for comp, pin in sorted(self.assigned_pins.items(), key=lambda x: x[1]):
            table.append({
                "component": comp,
                "pin_number": pin,
                "board": self.board_spec.board_name,
                "status": "VALIDATED",
            })
        return table
