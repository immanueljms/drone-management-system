"""
Registry: owns all active DroneAdapter instances, keeps the latest
DroneState per drone, and fans out updates to anything subscribed
(currently: the WebSocket broadcaster in main.py).

This is the layer that makes "multiple platforms, one dashboard" real —
it doesn't care whether a given drone is MAVLinkAdapter or
AeroLinkAdapter, only that it implements DroneAdapter.
"""
from __future__ import annotations

import asyncio
from typing import Awaitable, Callable

from adapters.base import CommandResult, DroneAdapter, DroneState

BroadcastCallback = Callable[[DroneState], Awaitable[None]]


class DroneRegistry:
    def __init__(self):
        self._adapters: dict[str, DroneAdapter] = {}
        self._latest_state: dict[str, DroneState] = {}
        self._broadcast_callbacks: list[BroadcastCallback] = []
        self._tasks: list[asyncio.Task] = []

    def on_broadcast(self, cb: BroadcastCallback) -> None:
        self._broadcast_callbacks.append(cb)

    async def _handle_state(self, state: DroneState) -> None:
        self._latest_state[state.drone_id] = state
        for cb in self._broadcast_callbacks:
            await cb(state)

    async def add_drone(self, adapter: DroneAdapter) -> None:
        adapter.on_state = self._handle_state
        self._adapters[adapter.drone_id] = adapter
        await adapter.connect()
        task = asyncio.create_task(adapter.listen())
        self._tasks.append(task)
        print(f"[registry] {adapter.drone_id} ({type(adapter).__name__}) online")

    def all_states(self) -> list[DroneState]:
        return list(self._latest_state.values())

    async def send_command(self, drone_id: str, command: str, params: dict) -> CommandResult:
        adapter = self._adapters.get(drone_id)
        if adapter is None:
            return CommandResult(drone_id=drone_id, command=command, accepted=False, detail="unknown drone_id")
        return await adapter.send_command(command, params)

    async def shutdown(self) -> None:
        for adapter in self._adapters.values():
            await adapter.disconnect()
        for task in self._tasks:
            task.cancel()
