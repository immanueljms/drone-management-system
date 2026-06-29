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
    """
    drone_id: str
    protocol: str  
    drone_type: str = "Untethered" # "Tethered" or "Untethered (Networked)"
    encryption_status: str = "AES-256 Secured" # Simulating the secure mesh requirement
    lat: float
    lon: float
    alt_m: float  
    speed_mps: Optional[float] = None  
    heading_deg: Optional[float] = None
    battery_pct: Optional[float] = None  
    battery_status: BatteryStatus = BatteryStatus.UNKNOWN
    link_quality_pct: Optional[float] = None
    flight_mode: str = "UNKNOWN"  
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None
    last_update_unix: float = Field(default_factory=lambda: time.time())
    raw: dict = Field(default_factory=dict)


class CommandResult(BaseModel):
    drone_id: str
    command: str
    accepted: bool
    detail: str = ""


# Callback type: adapters push new state up to the registry via this
StateCallback = Callable[[DroneState], Awaitable[None]]


class DroneAdapter(ABC):
    def __init__(self, drone_id: str, on_state: StateCallback, drone_type: str = "Networked UAS", encryption_status: str = "AES-256"):
        self.drone_id = drone_id
        self.on_state = on_state
        self.drone_type = drone_type
        self.encryption_status = encryption_status
        self._running = False
