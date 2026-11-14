"""Universal microcontroller support — every major board and protocol.

Provides board definitions, pin mappings, firmware upload, and
communication for all common robotics microcontrollers.

Supported boards:
- Raspberry Pi (all models: Zero, 3, 4, 5)
- Arduino (Uno, Mega, Nano, Due, ESP-based)
- ESP32 / ESP8266
- STM32 (F1, F4, H7 series)
- NVIDIA Jetson (Nano, Xavier, Orin)
- BeagleBone Black
- Teensy (3.x, 4.x)
- Microbit
- RP2040 (Raspberry Pi Pico)

Communication protocols:
- GPIO (digital read/write, interrupts)
- PWM (hardware and software)
- ADC (analog to digital conversion)
- I2C / TWI
- SPI
- UART / Serial
- CAN bus
- USB HID
- Bluetooth (via serial)
- WiFi (via socket/MQTT)

Usage:
    from stacc.hal.boards import get_board, list_boards
    from stacc.hal import I2CBus, SPIBus

    board = get_board("raspberry_pi_4")
    print(board.gpio_pins)
    print(board.i2c_buses)
    print(board.uart_ports)
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Set

# ══════════════════════════════════════════
# BOARD DEFINITIONS
# ══════════════════════════════════════════


class BoardDefinition:
    """Hardware board specification.

    Args:
        name: Human-readable board name
        platform: Platform identifier (e.g. "raspberry_pi", "arduino")
        architecture: CPU architecture (e.g. "ARM64", "AVR")
        cpu: CPU model name
        clock_mhz: CPU clock speed in MHz
        ram_kb: RAM size in KB
        flash_kb: Flash storage size in KB
        gpio_pins: Available GPIO pin numbers
        pwm_pins: PWM-capable pin numbers
        adc_pins: ADC-capable pin numbers
        i2c_buses: I2C bus descriptions
        spi_buses: SPI bus descriptions
        uart_ports: UART port descriptions
        voltage: Logic voltage level (default 3.3)
        has_wifi: Whether board has WiFi (default False)
        has_bluetooth: Whether board has Bluetooth (default False)
        has_can: Whether board has CAN bus (default False)
        has_usb: Whether board has USB (default True)
        upload_method: Firmware upload method identifier
        notes: Additional notes about the board
    """

    def __init__(
        self,
        name: str,
        platform: str,
        architecture: str,
        cpu: str,
        clock_mhz: float,
        ram_kb: int,
        flash_kb: int,
        gpio_pins: List[int],
        pwm_pins: List[int],
        adc_pins: List[int],
        i2c_buses: List[Dict[str, Any]],
        spi_buses: List[Dict[str, Any]],
        uart_ports: List[Dict[str, Any]],
        voltage: float = 3.3,
        has_wifi: bool = False,
        has_bluetooth: bool = False,
        has_can: bool = False,
        has_usb: bool = True,
        upload_method: str = "serial",
        notes: str = "",
    ):
        self.name = name
        self.platform = platform
        self.architecture = architecture
        self.cpu = cpu
        self.clock_mhz = clock_mhz
        self.ram_kb = ram_kb
        self.flash_kb = flash_kb
        self.gpio_pins = gpio_pins
        self.pwm_pins = pwm_pins
        self.adc_pins = adc_pins
        self.i2c_buses = i2c_buses
        self.spi_buses = spi_buses
        self.uart_ports = uart_ports
        self.voltage = voltage
        self.has_wifi = has_wifi
        self.has_bluetooth = has_bluetooth
        self.has_can = has_can
        self.has_usb = has_usb
        self.upload_method = upload_method
        self.notes = notes

    def supports_protocol(self, protocol: str) -> bool:
        """Check whether this board supports a given protocol.

        Args:
            protocol: One of ``gpio``, ``pwm``, ``adc``, ``i2c``, ``spi``,
                      ``uart`` / ``serial``, ``can``, ``wifi``, ``bluetooth``
        """
        p = protocol.lower()
        if p == "gpio":
            return len(self.gpio_pins) > 0
        if p == "pwm":
            return len(self.pwm_pins) > 0
        if p == "adc":
            return len(self.adc_pins) > 0
        if p == "i2c":
            return len(self.i2c_buses) > 0
        if p == "spi":
            return len(self.spi_buses) > 0
        if p in ("uart", "serial"):
            return len(self.uart_ports) > 0
        if p == "can":
            return self.has_can
        if p == "wifi":
            return self.has_wifi
        if p == "bluetooth":
            return self.has_bluetooth
        return False

    def to_dict(self) -> Dict[str, Any]:
        """Serialize board definition to a dictionary."""
        return {
            "name": self.name,
            "platform": self.platform,
            "architecture": self.architecture,
            "cpu": self.cpu,
            "clock_mhz": self.clock_mhz,
            "ram_kb": self.ram_kb,
            "flash_kb": self.flash_kb,
            "gpio_count": len(self.gpio_pins),
            "pwm_count": len(self.pwm_pins),
            "adc_count": len(self.adc_pins),
            "i2c_buses": len(self.i2c_buses),
            "spi_buses": len(self.spi_buses),
            "uart_ports": len(self.uart_ports),
            "voltage": self.voltage,
            "wifi": self.has_wifi,
            "bluetooth": self.has_bluetooth,
            "can": self.has_can,
        }

    def __repr__(self) -> str:
        return (
            f"Board('{self.name}', {self.cpu}, "
            f"{self.ram_kb}KB RAM, {self.flash_kb}KB flash)"
        )


# ══════════════════════════════════════════
# BOARD DATABASE
# ══════════════════════════════════════════

BOARDS: Dict[str, BoardDefinition] = {}


def _register(board: BoardDefinition) -> None:
    """Register a board definition with automatic alias creation."""
    BOARDS[board.name] = board
    for alias in [
        board.name.lower(),
        board.name.replace(" ", "_").lower(),
        board.platform.lower(),
    ]:
        if alias not in BOARDS:
            BOARDS[alias] = board


# ── Raspberry Pi ──

_register(
    BoardDefinition(
        name="Raspberry Pi 4",
        platform="raspberry_pi",
        architecture="ARM64",
        cpu="BCM2711 Cortex-A72",
        clock_mhz=1500,
        ram_kb=4 * 1024 * 1024,
        flash_kb=0,
        gpio_pins=list(range(2, 28)),
        pwm_pins=[12, 13, 18, 19],
        adc_pins=[],
        i2c_buses=[{"bus": 1, "sda": 2, "scl": 3}],
        spi_buses=[{"bus": 0, "mosi": 10, "miso": 9, "sclk": 11, "cs": [8, 7]}],
        uart_ports=[{"port": "/dev/ttyS0", "tx": 14, "rx": 15}],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        upload_method="ssh",
        notes="Full Linux OS. Use SSH deploy. pip install RPi.GPIO or gpiod.",
    )
)

_register(
    BoardDefinition(
        name="Raspberry Pi 5",
        platform="raspberry_pi",
        architecture="ARM64",
        cpu="BCM2712 Cortex-A76",
        clock_mhz=2400,
        ram_kb=8 * 1024 * 1024,
        flash_kb=0,
        gpio_pins=list(range(2, 28)),
        pwm_pins=[12, 13, 18, 19],
        adc_pins=[],
        i2c_buses=[{"bus": 1, "sda": 2, "scl": 3}],
        spi_buses=[{"bus": 0, "mosi": 10, "miso": 9, "sclk": 11, "cs": [8, 7]}],
        uart_ports=[{"port": "/dev/ttyAMA0", "tx": 14, "rx": 15}],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        upload_method="ssh",
    )
)

_register(
    BoardDefinition(
        name="Raspberry Pi Zero 2W",
        platform="raspberry_pi",
        architecture="ARM64",
        cpu="BCM2710A1 Cortex-A53",
        clock_mhz=1000,
        ram_kb=512 * 1024,
        flash_kb=0,
        gpio_pins=list(range(2, 28)),
        pwm_pins=[12, 13, 18, 19],
        adc_pins=[],
        i2c_buses=[{"bus": 1, "sda": 2, "scl": 3}],
        spi_buses=[{"bus": 0, "mosi": 10, "miso": 9, "sclk": 11, "cs": [8, 7]}],
        uart_ports=[{"port": "/dev/ttyS0", "tx": 14, "rx": 15}],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        upload_method="ssh",
    )
)

_register(
    BoardDefinition(
        name="Raspberry Pi Pico",
        platform="rp2040",
        architecture="ARM Cortex-M0+",
        cpu="RP2040",
        clock_mhz=133,
        ram_kb=264,
        flash_kb=2048,
        gpio_pins=list(range(0, 29)),
        pwm_pins=list(range(0, 29)),
        adc_pins=[26, 27, 28],
        i2c_buses=[{"bus": 0, "sda": 0, "scl": 1}, {"bus": 1, "sda": 2, "scl": 3}],
        spi_buses=[{"bus": 0, "mosi": 19, "miso": 16, "sclk": 18, "cs": [17]}],
        uart_ports=[{"port": "USB", "tx": 0, "rx": 1}],
        voltage=3.3,
        has_wifi=False,
        has_bluetooth=False,
        upload_method="uf2",
        notes="MicroPython or C/C++ SDK. Upload via UF2 drag-and-drop.",
    )
)

_register(
    BoardDefinition(
        name="Raspberry Pi Pico W",
        platform="rp2040",
        architecture="ARM Cortex-M0+",
        cpu="RP2040 + CYW43439",
        clock_mhz=133,
        ram_kb=264,
        flash_kb=2048,
        gpio_pins=list(range(0, 29)),
        pwm_pins=list(range(0, 29)),
        adc_pins=[26, 27, 28],
        i2c_buses=[{"bus": 0, "sda": 0, "scl": 1}],
        spi_buses=[{"bus": 0, "mosi": 19, "miso": 16, "sclk": 18, "cs": [17]}],
        uart_ports=[{"port": "USB", "tx": 0, "rx": 1}],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        upload_method="uf2",
    )
)

# ── Arduino ──

_register(
    BoardDefinition(
        name="Arduino Uno",
        platform="arduino",
        architecture="AVR",
        cpu="ATmega328P",
        clock_mhz=16,
        ram_kb=2,
        flash_kb=32,
        gpio_pins=list(range(0, 14)) + list(range(14, 20)),
        pwm_pins=[3, 5, 6, 9, 10, 11],
        adc_pins=[14, 15, 16, 17, 18, 19],
        i2c_buses=[{"bus": 0, "sda": 18, "scl": 19}],
        spi_buses=[{"bus": 0, "mosi": 11, "miso": 12, "sclk": 13, "cs": [10]}],
        uart_ports=[{"port": "/dev/ttyACM0", "tx": 1, "rx": 0}],
        voltage=5.0,
        upload_method="avrdude",
        notes="Classic Arduino. 5V logic. Upload via avrdude/Arduino IDE.",
    )
)

_register(
    BoardDefinition(
        name="Arduino Mega",
        platform="arduino",
        architecture="AVR",
        cpu="ATmega2560",
        clock_mhz=16,
        ram_kb=8,
        flash_kb=256,
        gpio_pins=list(range(0, 54)),
        pwm_pins=[2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46],
        adc_pins=list(range(54, 70)),
        i2c_buses=[{"bus": 0, "sda": 20, "scl": 21}],
        spi_buses=[{"bus": 0, "mosi": 51, "miso": 50, "sclk": 52, "cs": [53]}],
        uart_ports=[
            {"port": "/dev/ttyACM0", "tx": 1, "rx": 0},
            {"port": "Serial1", "tx": 18, "rx": 19},
            {"port": "Serial2", "tx": 16, "rx": 17},
            {"port": "Serial3", "tx": 14, "rx": 15},
        ],
        voltage=5.0,
        upload_method="avrdude",
        notes="54 digital pins, 16 analog, 4 UARTs. Best for complex robots.",
    )
)

_register(
    BoardDefinition(
        name="Arduino Nano",
        platform="arduino",
        architecture="AVR",
        cpu="ATmega328P",
        clock_mhz=16,
        ram_kb=2,
        flash_kb=32,
        gpio_pins=list(range(0, 14)) + list(range(14, 22)),
        pwm_pins=[3, 5, 6, 9, 10, 11],
        adc_pins=[14, 15, 16, 17, 18, 19, 20, 21],
        i2c_buses=[{"bus": 0, "sda": 18, "scl": 19}],
        spi_buses=[{"bus": 0, "mosi": 11, "miso": 12, "sclk": 13, "cs": [10]}],
        uart_ports=[{"port": "/dev/ttyUSB0", "tx": 1, "rx": 0}],
        voltage=5.0,
        upload_method="avrdude",
    )
)

_register(
    BoardDefinition(
        name="Arduino Due",
        platform="arduino",
        architecture="ARM Cortex-M3",
        cpu="ATSAM3X8E",
        clock_mhz=84,
        ram_kb=96,
        flash_kb=512,
        gpio_pins=list(range(0, 54)),
        pwm_pins=list(range(2, 14)),
        adc_pins=list(range(54, 66)),
        i2c_buses=[{"bus": 0, "sda": 20, "scl": 21}, {"bus": 1, "sda": 70, "scl": 71}],
        spi_buses=[{"bus": 0, "mosi": 75, "miso": 74, "sclk": 76, "cs": [10, 4, 52]}],
        uart_ports=[
            {"port": "/dev/ttyACM0", "tx": 1, "rx": 0},
            {"port": "Serial1", "tx": 18, "rx": 19},
        ],
        voltage=3.3,
        has_can=True,
        upload_method="bossac",
        notes="3.3V ARM. CAN bus support. Native USB.",
    )
)

# ── ESP32 ──

_register(
    BoardDefinition(
        name="ESP32",
        platform="esp32",
        architecture="Xtensa LX6",
        cpu="ESP32-WROOM-32",
        clock_mhz=240,
        ram_kb=520,
        flash_kb=4096,
        gpio_pins=[
            0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19,
            21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 39,
        ],
        pwm_pins=[0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27],
        adc_pins=[32, 33, 34, 35, 36, 39, 25, 26, 27, 14, 12, 13, 15, 2, 0, 4],
        i2c_buses=[{"bus": 0, "sda": 21, "scl": 22}],
        spi_buses=[{"bus": 2, "mosi": 23, "miso": 19, "sclk": 18, "cs": [5]}],
        uart_ports=[
            {"port": "/dev/ttyUSB0", "tx": 1, "rx": 3},
            {"port": "Serial1", "tx": 17, "rx": 16},
        ],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        has_can=True,
        upload_method="esptool",
        notes="Dual core, WiFi+BT. Upload via esptool.py. Supports CAN (TWAI).",
    )
)

_register(
    BoardDefinition(
        name="ESP32-S3",
        platform="esp32",
        architecture="Xtensa LX7",
        cpu="ESP32-S3",
        clock_mhz=240,
        ram_kb=512,
        flash_kb=8192,
        gpio_pins=list(range(0, 49)),
        pwm_pins=list(range(0, 49)),
        adc_pins=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        i2c_buses=[{"bus": 0, "sda": 8, "scl": 9}],
        spi_buses=[{"bus": 2, "mosi": 35, "miso": 37, "sclk": 36, "cs": [34]}],
        uart_ports=[{"port": "/dev/ttyACM0", "tx": 43, "rx": 44}],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        upload_method="esptool",
        notes="USB OTG, AI acceleration. Best for camera + ML projects.",
    )
)

_register(
    BoardDefinition(
        name="ESP8266",
        platform="esp8266",
        architecture="Xtensa L106",
        cpu="ESP8266EX",
        clock_mhz=80,
        ram_kb=80,
        flash_kb=4096,
        gpio_pins=[0, 2, 4, 5, 12, 13, 14, 15, 16],
        pwm_pins=[0, 2, 4, 5, 12, 13, 14, 15],
        adc_pins=[17],
        i2c_buses=[{"bus": 0, "sda": 4, "scl": 5}],
        spi_buses=[{"bus": 0, "mosi": 13, "miso": 12, "sclk": 14, "cs": [15]}],
        uart_ports=[{"port": "/dev/ttyUSB0", "tx": 1, "rx": 3}],
        voltage=3.3,
        has_wifi=True,
        upload_method="esptool",
    )
)

# ── STM32 ──

_register(
    BoardDefinition(
        name="STM32F103 Blue Pill",
        platform="stm32",
        architecture="ARM Cortex-M3",
        cpu="STM32F103C8T6",
        clock_mhz=72,
        ram_kb=20,
        flash_kb=64,
        gpio_pins=list(range(0, 37)),
        pwm_pins=[0, 1, 2, 3, 6, 7, 8, 9],
        adc_pins=[0, 1, 2, 3, 4, 5, 6, 7, 16, 17],
        i2c_buses=[
            {"bus": 1, "sda": "PB7", "scl": "PB6"},
            {"bus": 2, "sda": "PB11", "scl": "PB10"},
        ],
        spi_buses=[
            {"bus": 1, "mosi": "PA7", "miso": "PA6", "sclk": "PA5", "cs": ["PA4"]},
        ],
        uart_ports=[
            {"port": "USART1", "tx": "PA9", "rx": "PA10"},
            {"port": "USART2", "tx": "PA2", "rx": "PA3"},
        ],
        voltage=3.3,
        has_can=True,
        upload_method="stlink",
        notes="Cheap, powerful. Upload via ST-Link or serial bootloader.",
    )
)

_register(
    BoardDefinition(
        name="STM32F407 Discovery",
        platform="stm32",
        architecture="ARM Cortex-M4F",
        cpu="STM32F407VGT6",
        clock_mhz=168,
        ram_kb=192,
        flash_kb=1024,
        gpio_pins=list(range(0, 80)),
        pwm_pins=list(range(0, 32)),
        adc_pins=list(range(0, 16)),
        i2c_buses=[{"bus": 1}, {"bus": 2}, {"bus": 3}],
        spi_buses=[{"bus": 1}, {"bus": 2}, {"bus": 3}],
        uart_ports=[{"port": f"USART{i}"} for i in range(1, 7)],
        voltage=3.3,
        has_can=True,
        has_usb=True,
        upload_method="stlink",
        notes="FPU, DSP. Good for real-time control. 6 UARTs, 3 I2C, 3 SPI, 2 CAN.",
    )
)

_register(
    BoardDefinition(
        name="STM32H743",
        platform="stm32",
        architecture="ARM Cortex-M7",
        cpu="STM32H743ZIT6",
        clock_mhz=480,
        ram_kb=1024,
        flash_kb=2048,
        gpio_pins=list(range(0, 144)),
        pwm_pins=list(range(0, 48)),
        adc_pins=list(range(0, 20)),
        i2c_buses=[{"bus": i} for i in range(1, 5)],
        spi_buses=[{"bus": i} for i in range(1, 7)],
        uart_ports=[{"port": f"USART{i}"} for i in range(1, 9)],
        voltage=3.3,
        has_can=True,
        has_usb=True,
        upload_method="stlink",
        notes="Highest performance STM32. Ethernet, SDMMC, JPEG codec.",
    )
)

# ── NVIDIA Jetson ──

_register(
    BoardDefinition(
        name="Jetson Nano",
        platform="jetson",
        architecture="ARM64",
        cpu="Tegra X1 Cortex-A57",
        clock_mhz=1430,
        ram_kb=4 * 1024 * 1024,
        flash_kb=0,
        gpio_pins=[
            7, 11, 12, 13, 15, 16, 18, 19, 21, 22,
            23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40,
        ],
        pwm_pins=[32, 33],
        adc_pins=[],
        i2c_buses=[{"bus": 0}, {"bus": 1}],
        spi_buses=[{"bus": 0, "cs": [0, 1]}, {"bus": 1, "cs": [0]}],
        uart_ports=[{"port": "/dev/ttyTHS1"}],
        voltage=3.3,
        has_wifi=False,
        has_usb=True,
        upload_method="ssh",
        notes="128 CUDA cores for ML inference. Full Linux. SSH deploy.",
    )
)

_register(
    BoardDefinition(
        name="Jetson Orin Nano",
        platform="jetson",
        architecture="ARM64",
        cpu="Cortex-A78AE + Ampere GPU",
        clock_mhz=1500,
        ram_kb=8 * 1024 * 1024,
        flash_kb=0,
        gpio_pins=list(range(1, 41)),
        pwm_pins=[32, 33],
        adc_pins=[],
        i2c_buses=[{"bus": 0}, {"bus": 1}, {"bus": 7}],
        spi_buses=[{"bus": 0}, {"bus": 1}],
        uart_ports=[{"port": "/dev/ttyTHS0"}, {"port": "/dev/ttyTHS1"}],
        voltage=3.3,
        has_wifi=True,
        has_bluetooth=True,
        upload_method="ssh",
        notes="1024 CUDA cores. 40 TOPS AI. Best for advanced robotics.",
    )
)

# ── BeagleBone ──

_register(
    BoardDefinition(
        name="BeagleBone Black",
        platform="beaglebone",
        architecture="ARM Cortex-A8",
        cpu="AM3358",
        clock_mhz=1000,
        ram_kb=512 * 1024,
        flash_kb=4 * 1024 * 1024,
        gpio_pins=list(range(0, 65)),
        pwm_pins=[0, 1, 2, 3, 4, 5, 6, 7],
        adc_pins=[0, 1, 2, 3, 4, 5, 6],
        i2c_buses=[{"bus": 1}, {"bus": 2}],
        spi_buses=[{"bus": 0}, {"bus": 1}],
        uart_ports=[{"port": f"/dev/ttyO{i}"} for i in range(1, 5)],
        voltage=3.3,
        has_can=True,
        upload_method="ssh",
        notes="PRU real-time coprocessors. 65 GPIO. Good for industrial.",
    )
)

# ── Teensy ──

_register(
    BoardDefinition(
        name="Teensy 4.1",
        platform="teensy",
        architecture="ARM Cortex-M7",
        cpu="IMXRT1062",
        clock_mhz=600,
        ram_kb=1024,
        flash_kb=8192,
        gpio_pins=list(range(0, 42)),
        pwm_pins=list(range(0, 42)),
        adc_pins=list(range(14, 28)),
        i2c_buses=[{"bus": 0}, {"bus": 1}, {"bus": 2}],
        spi_buses=[{"bus": 0}, {"bus": 1}, {"bus": 2}],
        uart_ports=[{"port": f"Serial{i}"} for i in range(1, 9)],
        voltage=3.3,
        has_can=True,
        has_usb=True,
        upload_method="teensy_loader",
        notes="600MHz M7. Ethernet, SD card. Best latency of any Arduino-compatible.",
    )
)

# ── micro:bit ──

_register(
    BoardDefinition(
        name="micro:bit V2",
        platform="microbit",
        architecture="ARM Cortex-M4",
        cpu="nRF52833",
        clock_mhz=64,
        ram_kb=128,
        flash_kb=512,
        gpio_pins=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 19, 20],
        pwm_pins=[0, 1, 2],
        adc_pins=[0, 1, 2, 3, 4, 10],
        i2c_buses=[{"bus": 0, "sda": 20, "scl": 19}],
        spi_buses=[{"bus": 0, "mosi": 15, "miso": 14, "sclk": 13}],
        uart_ports=[{"port": "USB"}],
        voltage=3.3,
        has_wifi=False,
        has_bluetooth=True,
        upload_method="webusb",
        notes="Educational. Built-in accelerometer, compass, LEDs, buttons, mic, speaker.",
    )
)


# ══════════════════════════════════════════
# BOARD LOOKUP
# ══════════════════════════════════════════


def get_board(name: str) -> Optional[BoardDefinition]:
    """Look up a board by name (case-insensitive, fuzzy match).

    Args:
        name: Board name or alias

    Returns:
        BoardDefinition if found, else None
    """
    def normalize(value: str) -> str:
        return " ".join(value.lower().replace("-", " ").replace("_", " ").split())

    key = normalize(name)
    if not key:
        return None
    # Compare every exact normalized name before considering fuzzy matches:
    # ESP32-S3 must never resolve to the earlier ESP32 entry.
    for alias, board in BOARDS.items():
        if normalize(alias) == key:
            return board
    matches = {id(board): board for alias, board in BOARDS.items()
               if key in normalize(alias)}
    return next(iter(matches.values())) if len(matches) == 1 else None


def list_boards() -> List[BoardDefinition]:
    """List all supported boards (deduplicated).

    Returns:
        Sorted list of unique BoardDefinition objects
    """
    seen: Set[str] = set()
    boards: List[BoardDefinition] = []
    for board in BOARDS.values():
        if board.name not in seen:
            seen.add(board.name)
            boards.append(board)
    return sorted(boards, key=lambda b: b.platform)


def list_platforms() -> List[str]:
    """List all supported platform identifiers.

    Returns:
        Sorted list of platform names
    """
    return sorted(set(b.platform for b in BOARDS.values()))


# ══════════════════════════════════════════
# FIRMWARE UPLOADER
# ══════════════════════════════════════════


class FirmwareUploader:
    """Upload firmware to any supported board.

    Auto-detects the upload method from the board definition.

    Usage:
        uploader = FirmwareUploader(board="arduino_uno", port="/dev/ttyACM0")
        uploader.upload("firmware.hex")
    """

    def __init__(
        self,
        board: str = "arduino_uno",
        port: str = "/dev/ttyACM0",
        baud: int = 115200,
        flash_address: Optional[int] = None,
    ):
        self.board_def = get_board(board)
        self.port = port
        self.baud = baud
        if not isinstance(baud, int) or baud <= 0:
            raise ValueError("baud must be a positive integer")
        if flash_address is not None and (not isinstance(flash_address, int) or flash_address < 0):
            raise ValueError("flash_address must be a nonnegative integer")
        self.flash_address = flash_address

        if self.board_def is None:
            raise ValueError(
                f"Unknown board: {board!r}. Use list_boards() to see available."
            )

    def get_upload_args(self, firmware_path: str) -> List[str]:
        """Build a literal argument vector, never a shell program.

        Only concrete tool-backed providers are supported. Browser/manual,
        mount-copy and remote uploads must not masquerade as successful echo
        commands. ESP images require an explicit address for their layout.
        """
        path = str(firmware_path)
        if not path or "\0" in path or "\0" in self.port:
            raise ValueError("Firmware path and port must be valid nonempty arguments")
        method = self.board_def.upload_method
        cpu = self.board_def.cpu
        if method == "avrdude":
            targets = {"ATmega328P": ("m328p", "arduino"),
                       "ATmega2560": ("m2560", "wiring")}
            if cpu not in targets:
                raise NotImplementedError(f"No verified avrdude target for {cpu}")
            mcu, programmer = targets[cpu]
            return ["avrdude", "-p", mcu, "-c", programmer, "-P", self.port,
                    "-b", str(self.baud), "-U", f"flash:w:{path}:i"]
        if method == "esptool":
            chips = {"ESP32": "esp32", "ESP32-S3": "esp32s3", "ESP8266EX": "esp8266"}
            if cpu not in chips or self.flash_address is None:
                raise ValueError("ESP upload requires a supported chip and explicit flash_address for the image layout")
            return ["esptool.py", "--chip", chips[cpu], "--port", self.port,
                    "--baud", str(self.baud), "write_flash", hex(self.flash_address), path]
        if method == "stlink":
            address = self.flash_address if self.flash_address is not None else 0x08000000
            return ["st-flash", "write", path, hex(address)]
        if method == "bossac":
            return ["bossac", f"--port={self.port}", "-U", "-e", "-w", "-b", path, "-R"]
        raise NotImplementedError(
            f"No automatic upload provider for {self.board_def.name} ({method}); "
            "use the board-specific tool or an enrolled remote deployment provider"
        )

    def get_upload_command(self, firmware_path: str) -> str:
        """Display-only shell-quoted command. Execution always uses get_upload_args."""
        import shlex
        return shlex.join(self.get_upload_args(firmware_path))

    def upload(self, firmware_path: str, dry_run: bool = False) -> bool:
        """Upload firmware to the board.

        Args:
            firmware_path: Path to compiled firmware file
            dry_run: If True, print command without executing

        Returns:
            True if upload succeeded
        """
        import subprocess

        try:
            args = self.get_upload_args(firmware_path)
        except (ValueError, NotImplementedError) as exc:
            print(f"  [firmware] Upload unavailable: {exc}")
            return False
        import shlex
        cmd = shlex.join(args)
        print(f"  [firmware] Uploading to {self.board_def.name}...")
        print(f"  [firmware] Command: {cmd}")

        if dry_run:
            return True

        try:
            result = subprocess.run(
                args, shell=False, capture_output=True, text=True, timeout=60
            )
            if result.returncode == 0:
                print("  [firmware] Upload successful")
                return True
            else:
                print(f"  [firmware] Upload failed: {result.stderr}")
                return False
        except Exception as e:
            print(f"  [firmware] Error: {e}")
            return False

    def __repr__(self) -> str:
        return (
            f"FirmwareUploader(board={self.board_def.name!r}, "
            f"method={self.board_def.upload_method!r})"
        )
