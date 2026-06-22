"""
The core abstraction: a single, protocol-agnostic representation of a
drone's state, plus the interface every protocol-specific adapter must
implement.

This is the piece that proves the architecture. The dashboard, the
decision-support layer, and the command API all talk to DroneState and
DroneAdapter only — they never know MAVLink or AeroLink exist. Add a
third protocol later by writing one new adapter; nothing else changes.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Awaitable, Callable, Optional

from pydantic import BaseModel, Field


class BatteryStatus(str, Enum):
    OK = "ok"
    WARN = "warn"
    CRITICAL = "critical"
    UNKNOWN = "unknown"


class DroneState(BaseModel):
    """
    The unified, protocol-agnostic snapshot of one drone at one moment.
    Every adapter's job is to produce this from whatever wire format
    its drone actually speaks.
    """
    drone_id: str
    protocol: str  # e.g. "mavlink", "aerolink" — for display/debugging only
    lat: float
    lon: float
    alt_m: float  # always normalized to meters, regardless of source units
    speed_mps: Optional[float] = None  # normalized to m/s
    heading_deg: Optional[float] = None
    battery_pct: Optional[float] = None  # normalized to 0-100 where possible
    battery_status: BatteryStatus = BatteryStatus.UNKNOWN
    link_quality_pct: Optional[float] = None
    flight_mode: str = "UNKNOWN"  # CIRCLING | GOTO_WAYPOINT | RETURNING_TO_LAUNCH
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None
    last_update_unix: float = Field(default_factory=lambda: time.time())
    raw: dict = Field(default_factory=dict)  # original payload, kept for debugging/audit


class CommandResult(BaseModel):
    drone_id: str
    command: str
    accepted: bool
    detail: str = ""


# Callback type: adapters push new state up to the registry via this
StateCallback = Callable[[DroneState], Awaitable[None]]


class DroneAdapter(ABC):
    """
    One adapter instance manages the connection to ONE drone over its
    native protocol. The registry/backend only ever calls these four
    methods — it never touches MAVLink, AeroLink, or any future protocol
    directly.
    """

    def __init__(self, drone_id: str, on_state: StateCallback):
        self.drone_id = drone_id
        self.on_state = on_state
        self._running = False

    @abstractmethod
    async def connect(self) -> None:
        """Establish the connection to the drone (open socket, etc.)."""
        ...

    @abstractmethod
    async def listen(self) -> None:
        """
        Run the receive loop. Must call `await self.on_state(state)`
        each time a new DroneState is parsed. Should run until
        `disconnect()` is called.
        """
        ...

    @abstractmethod
    async def send_command(self, command: str, params: dict) -> CommandResult:
        """
        Translate a generic command (e.g. command="RTL") into this
        protocol's native wire format and send it.
        """
        ...

    @abstractmethod
    async def disconnect(self) -> None:
        """Tear down the connection cleanly."""
        ...
