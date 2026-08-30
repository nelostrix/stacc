"""STACC Space: Delay-Tolerant Networking (DTN) & Long-Latency Telemetry Link."""

import time
import json
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field


@dataclass
class TelemetryPacket:
    packet_id: int
    timestamp_utc: float
    source: str
    destination: str
    payload: Dict[str, Any]
    checksum: str
    delivered: bool = False
    in_flight_time_s: float = 0.0


@dataclass
class UplinkCommand:
    cmd_id: str
    action: str
    parameters: Dict[str, Any]
    queued_time: float
    execution_time_est: float
    status: str = "QUEUED"  # QUEUED, IN_FLIGHT, EXECUTED, FAILED


class DelayTolerantNetwork:
    """
    Delay-Tolerant Networking (DTN) and Space Link Buffer.
    Simulates Deep Space Network (DSN) transmission delays (e.g. Earth to Mars 4 to 24 min).
    """

    def __init__(self, one_way_light_time_s: float = 720.0): # Default 12 minutes (720s)
        self.owlt = one_way_light_time_s
        self.uplink_queue: List[UplinkCommand] = []
        self.downlink_buffer: List[TelemetryPacket] = []
        self.executed_commands: List[UplinkCommand] = []
        self._packet_counter = 0

    def set_one_way_latency(self, seconds: float) -> None:
        """Configure simulated one-way transmission latency (seconds)."""
        self.owlt = max(0.0, seconds)

    def queue_uplink_command(self, action: str, parameters: Dict[str, Any]) -> str:
        """Package and schedule a high-level command bundle for deep-space uplink."""
        cmd_id = f"CMD_DSN_{int(time.time()*1000)}_{len(self.uplink_queue)+1}"
        cmd = UplinkCommand(
            cmd_id=cmd_id,
            action=action,
            parameters=parameters,
            queued_time=time.time(),
            execution_time_est=time.time() + self.owlt,
            status="IN_FLIGHT",
        )
        self.uplink_queue.append(cmd)
        return cmd_id

    def buffer_downlink_telemetry(self, source: str, payload: Dict[str, Any]) -> int:
        """Transmit telemetry packet bundle down to mission control."""
        self._packet_counter += 1
        checksum = hex(hash(json.dumps(payload, sort_keys=True)) & 0xFFFFFFFF)
        packet = TelemetryPacket(
            packet_id=self._packet_counter,
            timestamp_utc=time.time(),
            source=source,
            destination="DSN_GOLDSTONE",
            payload=payload,
            checksum=checksum,
            delivered=False,
            in_flight_time_s=0.0,
        )
        self.downlink_buffer.append(packet)
        return self._packet_counter

    def step(self, dt: float = 1.0) -> Dict[str, Any]:
        """Step the network simulation clock and process arrived command bundles."""
        now = time.time()
        executable_cmds = []

        for cmd in self.uplink_queue:
            if cmd.status == "IN_FLIGHT" and (now >= cmd.execution_time_est or self.owlt < 1.0):
                cmd.status = "EXECUTED"
                executable_cmds.append(cmd)
                self.executed_commands.append(cmd)

        self.uplink_queue = [c for c in self.uplink_queue if c.status == "IN_FLIGHT"]

        return {
            "latency_owlt_s": self.owlt,
            "latency_rtt_min": round((self.owlt * 2.0) / 60.0, 1),
            "in_flight_uplinks": len(self.uplink_queue),
            "executed_commands": [c.action for c in executable_cmds],
            "downlink_packets_ready": len(self.downlink_buffer),
        }
