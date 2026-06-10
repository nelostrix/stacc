"""stacc.hal — Hardware Abstraction Layer.

Provides a universal interface for hardware control across all supported
platforms (Raspberry Pi, Jetson, Arduino, ESP32, STM32, and more).

Includes:
- Board detection and configuration (``Board.detect()``)
- Board definitions database with pin mappings (``get_board()``, ``list_boards()``)
- GPIO, I2C, SPI, UART/Serial communication
- Motor drivers for DC motors, servos, steppers
- Project deployment via SSH
- Firmware upload for microcontrollers

Usage:
    from stacc.hal import Board, GPIO, I2CBus, SPIBus, SerialPort
    from stacc.hal import MotorDriver, ServoDriver, Deployer

    board = Board.detect()
    gpio = GPIO()
    gpio.setup(18, GPIO.OUTPUT)
    gpio.write(18, GPIO.HIGH)
"""

from stacc.hal.board import Board
from stacc.hal.boards import (
    BoardDefinition,
    FirmwareUploader,
    get_board,
    list_boards,
    list_platforms,
)
from stacc.hal.deploy import Deployer
from stacc.hal.gpio import GPIO
from stacc.hal.i2c import I2CBus
from stacc.hal.motor import MotorDriver, ServoDriver
from stacc.hal.serial_comm import SerialPort
from stacc.hal.spi import SPIBus

__all__ = [
    "Board",
    "BoardDefinition",
    "FirmwareUploader",
    "get_board",
    "list_boards",
    "list_platforms",
    "Deployer",
    "GPIO",
    "I2CBus",
    "MotorDriver",
    "ServoDriver",
    "SerialPort",
    "SPIBus",
]
