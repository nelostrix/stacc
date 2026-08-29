"""stacc.hal — Hardware Abstraction Layer (HAL) & Industrial Fieldbus Interfaces."""

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
from stacc.hal.can import CANBus, CANFrame, ODriveCAN, UnitreeMotorCAN
from stacc.hal.shm import SharedMemoryRingBuffer
from stacc.hal.ethercat import EtherCATSlaveCiA402, CiA402State

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
    "CANBus",
    "CANFrame",
    "ODriveCAN",
    "UnitreeMotorCAN",
    "SharedMemoryRingBuffer",
    "EtherCATSlaveCiA402",
    "CiA402State",
]
