"""RTDE implementation of TorqueSource using a minimal, dependency-free client.

receive-only: it opens an independent RTDE connection to the robot/URSim and
never sends any motion command, so it coexists with the ROS 2 driver's own RTDE
connection (verified: a second receive client connects fine while the driver is
attached).

Why a raw RTDE client instead of the ur_rtde library: ur_rtde's high-level API
only exposes a fixed set of getters and has no getActualCurrentAsTorque(), so it
cannot read the measured-torque field actual_current_as_torque even when the
firmware supports it (PolyScope >= 5.23/10.11). The RTDE wire protocol lets us
request *any* output field by name, which is exactly what the rtde_field
parameter needs. This client uses only the standard library (socket, struct),
so the package has zero external dependency.

RTDE protocol (port 30004): every packet is a 3-byte header
(uint16 big-endian size, uint8 type) followed by size-3 payload bytes.
"""

import socket
import struct
from typing import List, Optional, Tuple

from .torque_source import UR7E_JOINT_ORDER, TorqueSource

# RTDE packet types we use.
_RTDE_REQUEST_PROTOCOL_VERSION = 86      # 'V'
_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
_RTDE_CONTROL_PACKAGE_START = 83         # 'S'
_RTDE_DATA_PACKAGE = 85                  # 'U'
_RTDE_PROTOCOL_VERSION = 2

# Known VECTOR6D joint fields and their unit, for validation and labelling.
#   actual_current_as_torque -> measured torque (N.m), PolyScope >= 5.23/10.11
#   target_moment            -> commanded/target torque (N.m), all versions
#   actual_current           -> motor current (A); diagnostic only
_KNOWN_FIELDS = {
    "actual_current_as_torque": "N.m",
    "target_moment": "N.m",
    "actual_current": "A",
}


class RtdeTorqueSource(TorqueSource):
    def __init__(
        self,
        robot_ip: str,
        rtde_field: str,
        rtde_port: int = 30004,
        frequency: float = 125.0,
        logger=None,
    ):
        if rtde_field not in _KNOWN_FIELDS:
            raise ValueError(
                f"Unknown rtde_field '{rtde_field}'. "
                f"Expected one of {sorted(_KNOWN_FIELDS)}."
            )
        self._robot_ip = robot_ip
        self._rtde_port = rtde_port
        self._rtde_field = rtde_field
        self._frequency = frequency if frequency > 0.0 else 125.0
        self._logger = logger
        self._sock = None
        self._buf = b""
        self._connected = False

    def connect(self) -> bool:
        if not self._port_reachable():
            self._log(
                f"RTDE port {self._robot_ip}:{self._rtde_port} not reachable; "
                "is URSim/the robot up and RTDE enabled?",
                level="warn",
            )
            return False

        try:
            self._sock = socket.create_connection(
                (self._robot_ip, self._rtde_port), timeout=5.0
            )
            self._sock.settimeout(5.0)

            self._negotiate_version()
            if not self._setup_outputs():
                self.disconnect()
                return False
            if not self._start():
                self.disconnect()
                return False

            self._sock.setblocking(False)
            self._buf = b""
            self._connected = True
            self._log(
                f"RTDE connected to {self._robot_ip}, streaming "
                f"'{self._rtde_field}' ({_KNOWN_FIELDS[self._rtde_field]}) "
                f"at {self._frequency:g} Hz.",
                level="info",
            )
            return True
        except OSError as exc:
            self._log(
                f"RTDE connection to {self._robot_ip} failed: {exc}",
                level="warn",
            )
            self.disconnect()
            return False

    def disconnect(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._buf = b""
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._sock is not None

    def get_torques(self) -> Tuple[List[str], Optional[List[float]]]:
        if not self.is_connected():
            return UR7E_JOINT_ORDER, None

        # Drain everything currently buffered (non-blocking) and keep the most
        # recent data package so we always publish the freshest sample.
        try:
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise ConnectionError("RTDE connection closed by peer")
                self._buf += chunk
        except BlockingIOError:
            pass
        except OSError as exc:
            self._log(f"RTDE read failed: {exc}", level="warn")
            self.disconnect()
            return UR7E_JOINT_ORDER, None

        latest = None
        while len(self._buf) >= 3:
            size = struct.unpack(">H", self._buf[:2])[0]
            if size < 3 or len(self._buf) < size:
                break
            ptype = self._buf[2]
            payload = self._buf[3:size]
            self._buf = self._buf[size:]
            # Data package payload: 1-byte recipe id + the requested VECTOR6D.
            if ptype == _RTDE_DATA_PACKAGE and len(payload) >= 1 + 48:
                values = struct.unpack(">6d", payload[1:49])
                latest = [float(v) for v in values]

        return UR7E_JOINT_ORDER, latest

    # --- RTDE handshake helpers (blocking, used only during connect) ---------

    def _negotiate_version(self) -> None:
        self._send(
            _RTDE_REQUEST_PROTOCOL_VERSION,
            struct.pack(">H", _RTDE_PROTOCOL_VERSION),
        )
        self._recv_packet(_RTDE_REQUEST_PROTOCOL_VERSION)

    def _setup_outputs(self) -> bool:
        payload = struct.pack(">d", self._frequency) + self._rtde_field.encode()
        self._send(_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, payload)
        resp = self._recv_packet(_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS)
        # resp = 1-byte recipe id + variable types CSV.
        types = resp[1:].decode(errors="replace")
        if "NOT_FOUND" in types:
            self._log(
                f"Firmware does not expose RTDE field '{self._rtde_field}' "
                "(returned NOT_FOUND). Use rtde_field 'target_moment'.",
                level="error",
            )
            return False
        return True

    def _start(self) -> bool:
        self._send(_RTDE_CONTROL_PACKAGE_START)
        resp = self._recv_packet(_RTDE_CONTROL_PACKAGE_START)
        accepted = bool(resp[0]) if resp else False
        if not accepted:
            self._log("RTDE start was not accepted by the controller.",
                      level="error")
        return accepted

    def _send(self, ptype: int, payload: bytes = b"") -> None:
        self._sock.sendall(struct.pack(">HB", 3 + len(payload), ptype) + payload)

    def _recv_packet(self, expected_type: int) -> bytes:
        """Read packets until one of expected_type arrives; return its payload."""
        while True:
            header = self._recv_exact(3)
            size = struct.unpack(">H", header[:2])[0]
            ptype = header[2]
            payload = self._recv_exact(size - 3) if size > 3 else b""
            if ptype == expected_type:
                return payload

    def _recv_exact(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                raise ConnectionError("RTDE connection closed by peer")
            data += chunk
        return data

    def _port_reachable(self, timeout: float = 1.0) -> bool:
        try:
            with socket.create_connection(
                (self._robot_ip, self._rtde_port), timeout=timeout
            ):
                return True
        except OSError:
            return False

    def _log(self, message: str, level: str = "info") -> None:
        if self._logger is None:
            return
        getattr(self._logger, level, self._logger.info)(message)
