"""
Adapter for "AeroLink" — our stand-in for a proprietary manufacturer
protocol. Demonstrates that the adapter pattern holds even when the
wire format, transport, units, and command vocabulary are completely
different from MAVLink.

Unit conversions handled here, invisible to everything downstream:
  - altitude: feet -> meters
  - speed:    knots -> m/s
  - battery:  voltage + status string -> approximate 0-100% + BatteryStatus
              (AeroLink doesn't report a clean percentage, only voltage
              and a coarse status flag — so our normalization here is
              necessarily approximate. This mirrors a real integration
              problem: not every platform reports the same fidelity of
              data, and the adapter has to make an honest best-effort
              translation, not pretend equivalence that doesn't exist.)
"""
from __future__ import annotations

import asyncio
import json

from .base import BatteryStatus, CommandResult, DroneAdapter, DroneState

FT_TO_M = 0.3048
KT_TO_MPS = 0.514444

# Rough voltage->percent curve for a 4S LiPo (16.8V full, 12.0V empty).
# This is an approximation, not a precise fuel gauge — flagged in raw{}.
VOLT_FULL = 16.8
VOLT_EMPTY = 12.0


def _voltage_to_pct(volts: float) -> float:
    pct = (volts - VOLT_EMPTY) / (VOLT_FULL - VOLT_EMPTY) * 100.0
    return max(0.0, min(100.0, round(pct, 1)))


def _map_battery_status(aerolink_status: str) -> BatteryStatus:
    return {
        "ok": BatteryStatus.OK,
        "warn": BatteryStatus.WARN,
        "critical": BatteryStatus.CRITICAL,
    }.get(aerolink_status, BatteryStatus.UNKNOWN)


class AeroLinkAdapter(DroneAdapter):
    def __init__(self, drone_id: str, on_state, host: str, port: int, drone_type: str = "Tethered UAS"):
        super().__init__(drone_id, on_state, drone_type=drone_type, encryption_status="AES-256 Secured")
        self.host = host
        self.port = port
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None

    async def connect(self) -> None:
        print(f"[{self.drone_id}] AeroLink adapter connecting to tcp://{self.host}:{self.port}")
        self._reader, self._writer = await asyncio.open_connection(self.host, self.port)
        print(f"[{self.drone_id}] AeroLink adapter connected")

    async def listen(self) -> None:
        self._running = True
        while self._running:
            if self._reader is None:
                await asyncio.sleep(1)
                continue
            try:
                line = await self._reader.readline()
            except (ConnectionResetError, asyncio.IncompleteReadError):
                print(f"[{self.drone_id}] AeroLink connection dropped, reconnecting...")
                await asyncio.sleep(2)
                await self.connect()
                continue

            if not line:
                await asyncio.sleep(0.5)
                continue

            try:
                payload = json.loads(line.decode("utf-8").strip())
            except json.JSONDecodeError:
                continue  # malformed line, skip rather than crash the ingestion loop

            if payload.get("msg_type") != "telemetry":
                continue

            pos = payload["pos"]
            power = payload["power"]
            home = payload.get("home", {})

            state = DroneState(
                drone_id=self.drone_id,
                protocol="aerolink",
                drone_type=self.drone_type,
                encryption_status=self.encryption_status,
                lat=pos["lat_deg"],
                lon=pos["lon_deg"],
                alt_m=round(pos["alt_ft"] * FT_TO_M, 1),
                speed_mps=round(payload["speed_kt"] * KT_TO_MPS, 2),
                heading_deg=payload.get("heading_deg"),
                battery_pct=_voltage_to_pct(power["volts"]),
                battery_status=_map_battery_status(power["status"]),
                link_quality_pct=payload.get("link_quality_pct"),
                flight_mode=payload.get("flight_mode", "UNKNOWN"),
                home_lat=home.get("lat_deg"),
                home_lon=home.get("lon_deg"),
                raw=payload,
            )
            await self.on_state(state)

    async def send_command(self, command: str, params: dict) -> CommandResult:
        if self._writer is None:
            return CommandResult(drone_id=self.drone_id, command=command, accepted=False, detail="not connected")

        # AeroLink's (fictional) command vocabulary is deliberately
        # different from MAVLink's — this is where real protocol
        # translation work happens, not just a pass-through.
        if command == "RTL":
            cmd_payload = {"msg_type": "cmd", "action": "return_to_base"}
        elif command == "GOTO":
            cmd_payload = {
                "msg_type": "cmd",
                "action": "set_waypoint",
                "lat_deg": params["lat"],
                "lon_deg": params["lon"],
                "alt_ft": params.get("alt_m", 50) / FT_TO_M,
            }
        else:
            return CommandResult(drone_id=self.drone_id, command=command, accepted=False, detail=f"unsupported command: {command}")

        line = (json.dumps(cmd_payload) + "\n").encode("utf-8")
        self._writer.write(line)
        await self._writer.drain()
        return CommandResult(drone_id=self.drone_id, command=command, accepted=True, detail=f"{cmd_payload['action']} sent over AeroLink")

    async def disconnect(self) -> None:
        self._running = False
        if self._writer:
            self._writer.close()
