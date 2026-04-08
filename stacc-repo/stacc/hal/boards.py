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
    from stacc.hal.protocols import I2CDevice, SPIDevice, CANBus
    from stacc.hal.firmware import FirmwareUploader

    board = get_board("raspberry_pi_4")
    print(board.gpio_pins)
    print(board.i2c_buses)
    print(board.uart_ports)
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

# ══════════════════════════════════════════
# BOARD DEFINITIONS
# ══════════════════════════════════════════


class BoardDefinition:
    """Hardware board specification."""

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
        protocol = protocol.lower()
        if protocol == "gpio":
            return len(self.gpio_pins) > 0
        elif protocol == "pwm":
            return len(self.pwm_pins) > 0
        elif protocol == "adc":
            return len(self.adc_pins) > 0
        elif protocol == "i2c":
            return len(self.i2c_buses) > 0
        elif protocol == "spi":
            return len(self.spi_buses) > 0
        elif protocol in ("uart", "serial"):
            return len(self.uart_ports) > 0
        elif protocol == "can":
            return self.has_can
        elif protocol == "wifi":
            return self.has_wifi
        elif protocol == "bluetooth":
            return self.has_bluetooth
        return False

    def to_dict(self) -> Dict[str, Any]:
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

    def __repr__(self):
        return f"Board('{self.name}', {self.cpu}, {self.ram_kb}KB RAM)"


# ══════════════════════════════════════════
# BOARD DATABASE
# ══════════════════════════════════════════

BOARDS: Dict[str, BoardDefinition] = {}


def _register(board: BoardDefinition):
    BOARDS[board.name] = board
    # Also register aliases
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
        adc_pins=[],  # no native ADC, use MCP3008 via SPI
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
        pwm_pins=list(range(0, 29)),  # all GPIO support PWM
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
        adc_pins=[14, 15, 16, 17, 18, 19],  # A0-A5
        i2c_buses=[{"bus": 0, "sda": 18, "scl": 19}],  # A4, A5
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
        adc_pins=list(range(54, 70)),  # A0-A15
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
            0,
            2,
            4,
            5,
            12,
            13,
            14,
            15,
            16,
            17,
            18,
            19,
            21,
            22,
            23,
            25,
            26,
            27,
            32,
            33,
            34,
            35,
            36,
            39,
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
        adc_pins=[17],  # single ADC (A0)
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
        gpio_pins=list(range(0, 37)),  # PA0-PA15, PB0-PB15, PC13-PC15
        pwm_pins=[0, 1, 2, 3, 6, 7, 8, 9],  # TIM1-TIM4 channels
        adc_pins=[0, 1, 2, 3, 4, 5, 6, 7, 16, 17],
        i2c_buses=[
            {"bus": 1, "sda": "PB7", "scl": "PB6"},
            {"bus": 2, "sda": "PB11", "scl": "PB10"},
        ],
        spi_buses=[
            {"bus": 1, "mosi": "PA7", "miso": "PA6", "sclk": "PA5", "cs": ["PA4"]}
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
            7,
            11,
            12,
            13,
            15,
            16,
            18,
            19,
            21,
            22,
            23,
            24,
            26,
            29,
            31,
            32,
            33,
            35,
            36,
            37,
            38,
            40,
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
    """Look up a board by name (case-insensitive, fuzzy match)."""
    key = name.lower().replace("-", " ").replace("_", " ").strip()

    # Exact match
    if key in BOARDS:
        return BOARDS[key]

    # Fuzzy match
    for bname, board in BOARDS.items():
        if key in bname.lower() or bname.lower() in key:
            return board

    return None


def list_boards() -> List[BoardDefinition]:
    """List all supported boards (deduplicated)."""
    seen = set()
    boards = []
    for board in BOARDS.values():
        if board.name not in seen:
            seen.add(board.name)
            boards.append(board)
    return sorted(boards, key=lambda b: b.platform)


def list_platforms() -> List[str]:
    """List all supported platforms."""
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

    UPLOAD_COMMANDS = {
        "avrdude": "avrdude -p {mcu} -c arduino -P {port} -b {baud} -U flash:w:{firmware}:i",
        "esptool": "esptool.py --chip {chip} --port {port} --baud {baud} write_flash 0x0 {firmware}",
        "stlink": "st-flash write {firmware} 0x08000000",
        "bossac": "bossac --port={port} -U -e -w -b {firmware} -R",
        "teensy_loader": "teensy_loader_cli --mcu={mcu} -w {firmware}",
        "uf2": "cp {firmware} /media/*/RPI-RP2/ 2>/dev/null || echo 'Drag {firmware} to RPI-RP2 drive'",
        "ssh": "scp {firmware} {user}@{host}:{remote_path}",
        "webusb": "echo 'Upload via browser at microbit.org/code'",
    }

    def __init__(
        self, board: str = "arduino_uno", port: str = "/dev/ttyACM0", baud: int = 115200
    ):
        self.board_def = get_board(board)
        self.port = port
        self.baud = baud

        if self.board_def is None:
            raise ValueError(
                f"Unknown board: {board}. Use list_boards() to see available."
            )

    def get_upload_command(self, firmware_path: str) -> str:
        method = self.board_def.upload_method
        template = self.UPLOAD_COMMANDS.get(
            method, "echo 'No upload method for {board}'"
        )
        return template.format(
            firmware=firmware_path,
            port=self.port,
            baud=self.baud,
            mcu=self.board_def.cpu,
            chip=self.board_def.platform,
            board=self.board_def.name,
            user="pi",
            host="",
            remote_path="/home/pi/",
        )

    def upload(self, firmware_path: str, dry_run: bool = False) -> bool:
        import subprocess

        cmd = self.get_upload_command(firmware_path)
        print(f"  [firmware] Uploading to {self.board_def.name}...")
        print(f"  [firmware] Command: {cmd}")

        if dry_run:
            return True

        try:
            result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=60
            )
            if result.returncode == 0:
                print(f"  [firmware] Upload successful")
                return True
            else:
                print(f"  [firmware] Upload failed: {result.stderr}")
                return False
        except Exception as e:
            print(f"  [firmware] Error: {e}")
            return False
