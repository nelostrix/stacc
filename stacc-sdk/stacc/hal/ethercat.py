"""Industrial EtherCAT Master Interface & CiA 402 Drive State Machine.

High-speed 1000Hz Process Data Object (PDO) exchange for industrial multi-axis robot controllers
(Beckhoff, Kollmorgen, Elmo Motion Gold/Platinum, Copley Controls).
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import enum
import time


class CiA402State(enum.Enum):
    """CiA 402 Drive State Machine."""
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7


class EtherCATSlaveCiA402:
    """CiA 402 Servo Drive over EtherCAT (CoE - CANopen over EtherCAT)."""

    def __init__(self, slave_id: int, name: str = "drive"):
        self.slave_id = int(slave_id)
        self.name = name
        self.state = CiA402State.SWITCH_ON_DISABLED
        self.actual_position = 0.0
        self.actual_velocity = 0.0
        self.actual_torque = 0.0

        # Control / Status words
        self.control_word = 0x0000
        self.status_word = 0x0000

    def parse_status_word(self, status: int) -> CiA402State:
        """Decode CiA 402 16-bit status word into state machine enum."""
        self.status_word = status
        masked = status & 0x006F

        if masked == 0x0000:
            self.state = CiA402State.NOT_READY_TO_SWITCH_ON
        elif masked == 0x0040:
            self.state = CiA402State.SWITCH_ON_DISABLED
        elif masked == 0x0021:
            self.state = CiA402State.READY_TO_SWITCH_ON
        elif masked == 0x0023:
            self.state = CiA402State.SWITCHED_ON
        elif masked == 0x0027:
            self.state = CiA402State.OPERATION_ENABLED
        elif masked == 0x0007:
            self.state = CiA402State.QUICK_STOP_ACTIVE
        elif (status & 0x0008) != 0:
            self.state = CiA402State.FAULT

        return self.state

    def get_enable_command(self) -> int:
        """Generate transition command word to bring drive into OPERATION_ENABLED state."""
        if self.state == CiA402State.FAULT:
            return 0x0080  # Fault reset
        elif self.state == CiA402State.SWITCH_ON_DISABLED:
            return 0x0006  # Shutdown -> Ready to Switch On
        elif self.state == CiA402State.READY_TO_SWITCH_ON:
            return 0x0007  # Switch On
        elif self.state == CiA402State.SWITCHED_ON:
            return 0x000F  # Enable Operation
        return 0x000F  # Maintain Operation
