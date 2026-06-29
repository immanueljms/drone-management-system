"""
FastAPI backend — Layer 3 (ingestion) + Layer 4 API (dashboard support)
+ Layer 6 (command channel) from the architecture slide.

Key point: nothing below explicitly mentions MAVLink or AeroLink except
the one-time setup in `lifespan()` where we decide which adapter to
instantiate for which drone. Every route and the WebSocket broadcaster
work purely in terms of DroneState / DroneAdapter.

Run: uvicorn main:app --reload --port 8000
     (or, if that fails with a ModuleNotFoundError: python3 -m uvicorn main:app --reload --port 8000)
"""
from __future__ import annotations

import asyncio
import json
import sys
from contextlib import asynccontextmanager
from pathlib import Path

# Make sure this file's own directory (backend/) is on sys.path, so the
# `adapters` and `registry` imports below resolve regardless of how this
# script is launched (bare `uvicorn`, `python3 -m uvicorn`, from a
# different cwd, etc.) — some uvicorn/reloader combinations don't
# reliably add the current directory to sys.path on their own.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from adapters.base import DroneState
from adapters.customlink_adapter import AeroLinkAdapter
from adapters.mavlink_adapter import MavlinkAdapter
from registry import DroneRegistry

registry = DroneRegistry()
active_websockets: set[WebSocket] = set()


async def broadcast_to_websockets(state: DroneState) -> None:
    if not active_websockets:
        return
    message = json.dumps({"type": "state_update", "data": state.model_dump()})
    dead = set()
    for ws in active_websockets:
        try:
            await ws.send_text(message)
        except Exception:
            dead.add(ws)
    active_websockets.difference_update(dead)


@asynccontextmanager
async def lifespan(app: FastAPI):
    registry.on_broadcast(broadcast_to_websockets)
    
    mavlink_drone = MavlinkAdapter(
        drone_id="UAS-ALPHA-MOBILE", on_state=None, udp_listen_port=14550, udp_command_port=14555, drone_type="Networked UAS"
    )
    
    aerolink_drone = AeroLinkAdapter(
        drone_id="UAS-BRAVO-TETHERED", on_state=None, host="127.0.0.1", port=9100, drone_type="Tethered UAS"
    )
    
    await registry.add_drone(mavlink_drone)
    await registry.add_drone(aerolink_drone)
    yield
    await registry.shutdown()

app = FastAPI(title="Integrated Drone Management System — PoC", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # PoC only — tighten before anything resembling production
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/drones")
async def list_drones():
    """Current snapshot of every drone's latest known state."""
    return [s.model_dump() for s in registry.all_states()]


class CommandRequest(BaseModel):
    command: str  # "RTL" | "GOTO"
    params: dict = {}


@app.post("/api/drones/{drone_id}/command")
async def send_command(drone_id: str, req: CommandRequest):
    """
    Layer 6 — the bidirectional command channel. Same endpoint shape
    regardless of whether drone_id resolves to a MAVLink or AeroLink
    adapter; the registry and adapter handle translation.
    """
    result = await registry.send_command(drone_id, req.command, req.params)
    return result.model_dump()


@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    """Live push feed for the dashboard — Layer 4."""
    await websocket.accept()
    active_websockets.add(websocket)

    # send current snapshot immediately so the dashboard doesn't wait
    # for the next tick to populate
    for state in registry.all_states():
        await websocket.send_text(json.dumps({"type": "state_update", "data": state.model_dump()}))

    try:
        while True:
            await websocket.receive_text()  # we don't expect client messages; just keep alive
    except WebSocketDisconnect:
        active_websockets.discard(websocket)
