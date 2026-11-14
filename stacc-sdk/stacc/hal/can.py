"""Industrial CAN Bus, SocketCAN, and Motor Drive Interfaces.

Supports:
- Linux SocketCAN interface (`can0`, `can1`, `vcan0`)
- Standard CAN 2.0A/B and CAN-FD frame pack/unpack
- Brushless Robot Servo Drivers:
  * Unitree Robotics (GO-M8010, A1, B1, Aliengo)
  * ODrive Robotics (CAN Simple Protocol v3.x / v5.x / S1 / Pro)
  * MyActuator RMD (X-series, L-series)
  * Maxon Motor EPOS4 (CANopen CiA 402 profile)
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple, Union
import struct
import time


class CANFrame:
    """Standard CAN Bus Frame."""

    def __init__(self, arbitration_id: int, data: bytes, is_extended: bool = False, is_fd: bool = False):
        self.arbitration_id = int(arbitration_id)
        self.data = bytes(data)
        self.is_extended = bool(is_extended)
        self.is_fd = bool(is_fd)
        self.dlc = len(self.data)
        if not 0 <= self.arbitration_id <= (0x1FFFFFFF if self.is_extended else 0x7FF):
            raise ValueError("CAN arbitration ID is outside the selected frame format")
        if self.dlc > (64 if self.is_fd else 8):
            raise ValueError("CAN payload exceeds the selected frame format")
        self.timestamp = time.time()

    def __repr__(self) -> str:
        data_hex = " ".join(f"{b:02X}" for b in self.data)
        return f"<CANFrame ID=0x{self.arbitration_id:03X} DLC={self.dlc} Data=[{data_hex}]>"


class CANBus:
    """Universal CAN Bus Hardware Adapter (SocketCAN / Virtual / USB).

    Args:
        channel: Interface name (e.g. 'can0', 'vcan0', 'slcan0')
        bitrate: Baudrate in bps (default 1,000,000 = 1 Mbps for robotics)
        is_virtual: If True, operates in in-memory simulation mode without hardware socket
    """

    def __init__(self, channel: str = "can0", bitrate: int = 1000000, is_virtual: bool = False):
        self.channel = str(channel)
        self.bitrate = int(bitrate)
        self.is_virtual = bool(is_virtual)
        self._socket = None
        self._rx_buffer: List[CANFrame] = []

        if not self.is_virtual:
            try:
                import socket
                self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self._socket.bind((self.channel,))
                self._socket.setblocking(False)
            except Exception as exc:
                if self._socket is not None:
                    self._socket.close()
                    self._socket = None
                raise ConnectionError(
                    f"Cannot open real CAN interface {self.channel!r}; "
                    "use is_virtual=True explicitly for simulation"
                ) from exc

    def send(self, frame: CANFrame) -> bool:
        """Transmit a CAN frame onto the bus."""
        if frame.is_fd and not self.is_virtual:
            raise NotImplementedError("Physical CAN-FD transmission is not implemented")
        if self.is_virtual or self._socket is None:
            self._rx_buffer.append(frame)
            return True

        # SocketCAN struct: <I B 3x 8s (can_id, can_dlc, pad, data)
        can_id = frame.arbitration_id
        if frame.is_extended:
            can_id |= 0x80000000  # CAN_EFF_FLAG

        data_padded = frame.data.ljust(8, b"\x00")[:8]
        raw_frame = struct.pack("=IB3x8s", can_id, frame.dlc, data_padded)
        try:
            self._socket.send(raw_frame)
            return True
        except Exception:
            return False

    def recv(self, timeout: float = 0.001) -> Optional[CANFrame]:
        """Receive a CAN frame from the bus."""
        if self.is_virtual or self._socket is None:
            return self._rx_buffer.pop(0) if self._rx_buffer else None

        try:
            raw_frame = self._socket.recv(16)
            can_id, dlc, data = struct.unpack("=IB3x8s", raw_frame)
            is_extended = bool(can_id & 0x80000000)
            clean_id = can_id & 0x1FFFFFFF if is_extended else can_id & 0x7FF
            return CANFrame(arbitration_id=clean_id, data=data[:dlc], is_extended=is_extended)
        except Exception:
            return None


class ODriveCAN:
    """ODrive Brushless Motor Controller (CAN Simple Protocol)."""

    CMD_SET_INPUT_POS = 0x00C
    CMD_SET_INPUT_VEL = 0x00D
    CMD_SET_INPUT_TORQUE = 0x00E
    CMD_GET_ENCODER_ESTIMATES = 0x009

    def __init__(self, node_id: int, bus: CANBus):
        self.node_id = int(node_id)
        self.bus = bus

    def set_position(self, pos_turns: float, vel_ff_turns_per_sec: float = 0.0, torque_ff_nm: float = 0.0) -> bool:
        """Command position setpoint via CAN (turns, vel_feedforward, torque_feedforward)."""
        arb_id = (self.node_id << 5) | self.CMD_SET_INPUT_POS
        vel_ff_int = int(vel_ff_turns_per_sec * 1000)  # 0.001 turns/s
        torque_ff_int = int(torque_ff_nm * 1000)        # 0.001 Nm
        data = struct.pack("<fhh", float(pos_turns), vel_ff_int, torque_ff_int)
        return self.bus.send(CANFrame(arbitration_id=arb_id, data=data))

    def set_torque(self, torque_nm: float) -> bool:
        """Command motor torque directly in Nm."""
        arb_id = (self.node_id << 5) | self.CMD_SET_INPUT_TORQUE
        data = struct.pack("<f", float(torque_nm))
        return self.bus.send(CANFrame(arbitration_id=arb_id, data=data))


class UnitreeMotorCAN:
    """Unitree High-Performance Robot Actuator (GO-M8010 / A1 / B1 / H1)."""

    def __init__(self, motor_id: int, bus: CANBus):
        self.motor_id = int(motor_id)
        self.bus = bus

    def send_hybrid_control(
        self,
        q_des: float,
        dq_des: float,
        kp: float,
        kd: float,
        tau_ff: float
    ) -> bool:
        """Send 5-parameter hybrid impedance control command: tau = tau_ff + kp*(q_des - q) + kd*(dq_des - dq)."""
        # The former generic encoder packed ten bytes and discarded torque by
        # truncating to eight. Unitree families require model-specific protocols;
        # an invented packet must never be sent to a physical motor.
        raise NotImplementedError(
            "Unitree hybrid control requires a verified model-specific driver; "
            "the previous lossy generic CAN encoding is disabled"
        )
