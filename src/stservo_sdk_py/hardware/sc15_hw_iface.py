# sc15_hw_iface.py  —  ros2_control hardware interface for SC15 / SC-series servos
#
#  • Maps joint position commands (rad) ↔ servo ticks (0–1023, mid=512)
#  • Uses STservo_sdk.PortHandler + scscl helper you already shipped
#  • Supports any baud rate (default 1 000 000 bps)
#  • Handles N joints on one half-duplex TTL bus
#
#  ROS 2 Jazzy  ·  Python ≥ 3.10  ·  ros2_control 2.x
#
import math, time
from typing import Dict, List

from rclpy.clock import Clock
from rclpy.time  import Time

from hardware_interface import (
    BaseInterface,
    CommandInterface,
    StateInterface,
)
from hardware_interface.hw_info import IndividualHardwareInfo

from STservo_sdk import PortHandler, scscl


TICKS_PER_REV = 1024.0          # SC-series = 10-bit
MID_TICK      = 512.0           # mechanical centre
RAD2TICK = TICKS_PER_REV / (2 * math.pi)
TICK2RAD = (2 * math.pi) / TICKS_PER_REV


class SC15Bus(BaseInterface):
    """
    Minimal ros2_control plugin:
    – position-only joints
    – blocking read() / write() with per-cycle timeout ~3 ms for 6 servos
    """
    ###########  life-cycle ###################################################

    def configure(self, hw_info: IndividualHardwareInfo):
        # --- parameters from YAML ------------------------------------------
        self._port = hw_info.hardware_parameters.get("device", "/dev/scbus")
        self._baud = int(hw_info.hardware_parameters.get("baud", 1_000_000))
        # joint.name → servo ID (int) map
        self._id_map: Dict[str, int] = {
            j.name: int(j.parameters["id"]) for j in hw_info.joints
        }

        # open serial
        self._ph  = PortHandler(self._port)
        if not self._ph.openPort():
            self._logger.error(f"Failed to open {self._port}")
            return False
        if not self._ph.setBaudRate(self._baud):
            self._logger.error(f"Failed to set baud to {self._baud}")
            return False
        self._bus = scscl(self._ph)

        # command / state buffers (rad)
        self._cmd  = {j: 0.0 for j in self._id_map}
        self._pos  = {j: 0.0 for j in self._id_map}

        self._logger.info(
            f"SC15Bus ready on {self._port} @ {self._baud} baud, "
            f"{len(self._id_map)} joints."
        )
        return True

    def start(self) -> bool:
        # read once to seed positions
        return self.read(Clock().now())    # returns bool

    def stop(self) -> bool:
        self._ph.closePort()
        return True


    ###########  interface exports ###########################################

    def export_state_interfaces(self) -> List[StateInterface]:
        return [
            StateInterface(j, "position", lambda i=j: self._pos[i])
            for j in self._id_map
        ]

    def export_command_interfaces(self) -> List[CommandInterface]:
        return [
            CommandInterface(
                j,
                "position",
                lambda v, i=j: self._cmd.__setitem__(i, v),
                lambda i=j: self._cmd[i],
            )
            for j in self._id_map
        ]


    ###########  real-time hooks #############################################

    def read(self, now: Time) -> bool:
        """Poll each servo for its present position."""
        try:
            for j, sid in self._id_map.items():
                ticks, _, _ = self._bus.ReadPos(sid)
                self._pos[j] = (ticks - MID_TICK) * TICK2RAD
            return True
        except Exception as e:
            self._logger.error(f"read() failed: {e}")
            return False

    def write(self, now: Time) -> bool:
        """Push most-recent command to each servo (position-mode)."""
        try:
            for j, sid in self._id_map.items():
                tgt_tick = int(round(self._cmd[j] * RAD2TICK + MID_TICK))
                tgt_tick = max(0, min(1023, tgt_tick))
                # speed=0, accel=0 → use servo defaults
                self._bus.WritePos(sid, tgt_tick, 0, 0)
            return True
        except Exception as e:
            self._logger.error(f"write() failed: {e}")
            return False
