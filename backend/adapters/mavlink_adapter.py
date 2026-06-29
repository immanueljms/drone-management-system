"""
Adapter for MAVLink-speaking drones (e.g. our simulator, or real
PX4/ArduPilot SITL/hardware later — same adapter, no code changes needed
on the backend side, only the connection string).

Unit conversions handled here, invisible to everything downstream:
  - lat/lon: MAVLink sends as int32 * 1e7  -> degrees (float)
  - alt:     MAVLink sends as int32 mm     -> meters (float)
  - velocity: MAVLink sends as int16 cm/s  -> m/s (float)
  - battery: MAVLink sends 0-100 already   -> passed through
"""
from __future__ import annotations

import asyncio
import math

from pymavlink import mavutil

from .base import BatteryStatus, CommandResult, DroneAdapter, DroneState


class MavlinkAdapter(DroneAdapter):
    def __init__(self, drone_id: str, on_state, udp_listen_port: int, udp_command_port: int | None = None, drone_type: str = "Networked UAS"):
        super().__init__(drone_id, on_state, drone_type=drone_type, encryption_status="AES-256 Secured")
        self.udp_listen_port = udp_listen_port
        # Commands go out on a SEPARATE connection from the one we
        # receive telemetry on. udpin (telemetry) is a receive-only
        # bind; sending on it relies on pymavlink having already seen
        # a packet from a specific peer, which is fragile. A dedicated
        # udpout connection to the sim's command port is explicit and
        # reliable — this mirrors how a real ground station typically
        # has separate send/receive paths even over the same physical
        # radio link.
        self.udp_command_port = udp_command_port or (udp_listen_port + 5)
        self._mav = None          # telemetry (receive) connection
        self._cmd_mav = None      # command (send) connection
        self._latest_battery_pct: float | None = None

    async def connect(self) -> None:
        conn_str = f"udpin:127.0.0.1:{self.udp_listen_port}"
        self._mav = mavutil.mavlink_connection(conn_str)
        print(f"[{self.drone_id}] MAVLink adapter listening on {conn_str}")

        cmd_conn_str = f"udpout:127.0.0.1:{self.udp_command_port}"
        self._cmd_mav = mavutil.mavlink_connection(cmd_conn_str)
        print(f"[{self.drone_id}] MAVLink adapter will send commands -> {cmd_conn_str}")

        self._home_lat: float | None = None
        self._home_lon: float | None = None
        self._flight_mode: str = "UNKNOWN"

    async def listen(self) -> None:
        self._running = True
        loop = asyncio.get_event_loop()
        while self._running:
            # pymavlink's recv_match is blocking, so run it in a thread
            # pool executor to avoid stalling the asyncio event loop —
            # this is the standard pattern for wrapping blocking I/O.
            msg = await loop.run_in_executor(
                None, lambda: self._mav.recv_match(blocking=True, timeout=2)
            )
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "GLOBAL_POSITION_INT":
                state = DroneState(
                    drone_id=self.drone_id,
                    protocol="mavlink",
                    drone_type=self.drone_type,
                    encryption_status=self.encryption_status,   
                    lat=msg.lat / 1e7,
                    lon=msg.lon / 1e7,
                    alt_m=msg.alt / 1000.0,
                    speed_mps=math.sqrt(msg.vx**2 + msg.vy**2) / 100.0,
                    heading_deg=None,  # GLOBAL_POSITION_INT.hdg unused in our sim
                    battery_pct=self._latest_battery_pct,
                    battery_status=self._battery_status(self._latest_battery_pct),
                    flight_mode=self._flight_mode,
                    home_lat=self._home_lat,
                    home_lon=self._home_lon,
                    raw=msg.to_dict(),
                )
                await self.on_state(state)

            elif msg_type == "SYS_STATUS":
                self._latest_battery_pct = float(msg.battery_remaining)
                # SYS_STATUS doesn't carry position, so we don't emit a
                # full DroneState here — the next GLOBAL_POSITION_INT
                # will pick up the updated battery value.

            elif msg_type == "STATUSTEXT":
                # Our simulator overloads STATUSTEXT to announce HOME
                # position and current flight mode (see the "Why
                # STATUSTEXT" note in mavlink_drone_sim.py) — a real
                # platform would expose this via dedicated messages
                # (HOME_POSITION, HEARTBEAT.custom_mode), but parsing
                # free text here keeps the simulator simple without
                # adding new message types. The adapter's job is the
                # same either way: extract meaning, update local state,
                # don't forward raw protocol text downstream.
                # pymavlink decodes STATUSTEXT.text into a plain Python
                # str already (it strips the null-padding internally),
                # so no bytes/decode step is needed here.
                text = msg.text.rstrip("\x00")
                if text.startswith("HOME:"):
                    try:
                        lat_str, lon_str = text[5:].split(",")
                        self._home_lat = float(lat_str)
                        self._home_lon = float(lon_str)
                    except ValueError:
                        pass
                elif text.startswith("MODE:"):
                    self._flight_mode = text[5:]

    @staticmethod
    def _battery_status(pct: float | None) -> BatteryStatus:
        if pct is None:
            return BatteryStatus.UNKNOWN
        if pct > 30:
            return BatteryStatus.OK
        if pct > 15:
            return BatteryStatus.WARN
        return BatteryStatus.CRITICAL

    async def send_command(self, command: str, params: dict) -> CommandResult:
        if self._cmd_mav is None:
            return CommandResult(drone_id=self.drone_id, command=command, accepted=False, detail="not connected")

        target_system = 1  # matches the sim's --system-id default

        if command == "RTL":
            self._cmd_mav.mav.command_long_send(
                target_system, 0,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            return CommandResult(drone_id=self.drone_id, command=command, accepted=True, detail="RTL command sent")

        if command == "GOTO":
            lat, lon, alt = params["lat"], params["lon"], params.get("alt_m", 50)
            # param5/6/7 (the last three positional args here) carry
            # lat/lon/alt — the sim's command_listener reads them back
            # out as msg.param5/param6/param7, matching this layout.
            self._cmd_mav.mav.command_long_send(
                target_system, 0,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0,
                lat, lon, alt,
            )
            return CommandResult(drone_id=self.drone_id, command=command, accepted=True, detail=f"GOTO {lat:.5f},{lon:.5f} sent")

        return CommandResult(drone_id=self.drone_id, command=command, accepted=False, detail=f"unsupported command: {command}")

    async def disconnect(self) -> None:
        self._running = False
        if self._mav:
            self._mav.close()
        if self._cmd_mav:
            self._cmd_mav.close()
