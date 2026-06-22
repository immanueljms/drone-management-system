"""
Self-contained correctness test for both adapters, run in a single
process (no cross-process backgrounding) to sidestep sandbox
process/networking flakiness. This proves the parsing and unit-
conversion logic is correct; the multi-process deployment (separate
sim scripts + separate backend) is the real intended architecture and
will work the same way on a normal machine.
"""
import asyncio
import math
import socket
import sys
import threading
import time
from pathlib import Path

# Resolve the backend/ directory relative to this file's own location,
# so this works regardless of where the project is cloned/copied to.
BACKEND_DIR = Path(__file__).resolve().parent / "backend"
sys.path.insert(0, str(BACKEND_DIR))

from adapters.mavlink_adapter import MavlinkAdapter
from adapters.customlink_adapter import AeroLinkAdapter
from pymavlink import mavutil


received_states = []


async def collector(state):
    received_states.append(state)
    print(f"  -> {state.drone_id:10s} [{state.protocol:8s}] "
          f"lat={state.lat:.5f} lon={state.lon:.5f} alt_m={state.alt_m:.1f} "
          f"speed_mps={state.speed_mps} battery={state.battery_pct}% ({state.battery_status})")


def run_mavlink_sender(port):
    """Send a handful of real MAVLink messages, mimicking the sim script."""
    mav = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1)
    time.sleep(0.3)
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_PX4, 0, 0, 0
    )
    mav.mav.sys_status_send(0, 0, 0, 500, 12600, -1, 77, 0, 0, 0, 0, 0, 0)
    mav.mav.global_position_int_send(
        1000, int(12.9716 * 1e7), int(77.5946 * 1e7), int(925000), int(45000),
        300, 150, 0, 0,
    )


def run_aerolink_server(port):
    """Minimal TCP server sending one AeroLink JSON line, mimicking the sim script."""
    import json
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(1)
    conn, _ = server.accept()
    payload = {
        "msg_type": "telemetry",
        "unit_callsign": "DRONE-B",
        "pos": {"lat_deg": 12.9352, "lon_deg": 77.6245, "alt_ft": 3200.0},
        "speed_kt": 18.5,
        "heading_deg": 270.0,
        "power": {"volts": 15.2, "status": "ok"},
        "link_quality_pct": 92,
        "ts_unix": time.time(),
    }
    conn.sendall((json.dumps(payload) + "\n").encode())
    time.sleep(1)
    conn.close()
    server.close()


async def main():
    mav_port = 14551
    aero_port = 9101

    print("=== Connecting adapters (listeners bind first) ===")
    mav_adapter = MavlinkAdapter("DRONE-A", collector, mav_port)
    await mav_adapter.connect()

    mav_task = asyncio.create_task(mav_adapter.listen())
    await asyncio.sleep(0.3)  # ensure the UDP socket is actually bound and recv-ready

    threading.Thread(target=run_aerolink_server, args=(aero_port,), daemon=True).start()
    time.sleep(0.3)

    aero_adapter = AeroLinkAdapter("DRONE-B", collector, "127.0.0.1", aero_port)
    await aero_adapter.connect()
    aero_task = asyncio.create_task(aero_adapter.listen())

    print("\n=== Sending MAVLink messages now that listener is ready ===")
    # Send repeatedly for a couple seconds to rule out a one-shot UDP race
    for _ in range(5):
        run_mavlink_sender(mav_port)
        await asyncio.sleep(0.4)

    await asyncio.sleep(1)

    mav_task.cancel()
    aero_task.cancel()
    await mav_adapter.disconnect()
    await aero_adapter.disconnect()

    print(f"\n=== Received {len(received_states)} normalized DroneState objects ===")

    # --- Correctness assertions ---
    by_drone = {s.drone_id: s for s in received_states}
    assert "DRONE-A" in by_drone, "Did not receive any state from MAVLink drone"
    assert "DRONE-B" in by_drone, "Did not receive any state from AeroLink drone"

    a = by_drone["DRONE-A"]
    b = by_drone["DRONE-B"]

    # Drone A (MAVLink): sent lat=12.9716e7 -> should decode to 12.9716
    assert abs(a.lat - 12.9716) < 1e-4, f"MAVLink lat conversion wrong: {a.lat}"
    assert abs(a.alt_m - 925.0) < 0.1, f"MAVLink alt conversion wrong: {a.alt_m}"
    assert a.protocol == "mavlink"

    # Drone B (AeroLink): sent alt_ft=3200.0 -> should convert to meters (~975.4m)
    expected_alt_m = 3200.0 * 0.3048
    assert abs(b.alt_m - expected_alt_m) < 0.1, f"AeroLink ft->m conversion wrong: {b.alt_m} vs {expected_alt_m}"
    # speed_kt=18.5 -> m/s (~9.52)
    expected_speed = 18.5 * 0.514444
    assert abs(b.speed_mps - expected_speed) < 0.01, f"AeroLink kt->mps conversion wrong: {b.speed_mps}"
    assert b.protocol == "aerolink"
    assert b.battery_status.value == "ok"

    print("\n✅ ALL ASSERTIONS PASSED")
    print("   - MAVLink adapter correctly decoded int32*1e7 lat/lon and mm altitude")
    print("   - AeroLink adapter correctly converted feet->meters and knots->m/s")
    print("   - Both produced the SAME DroneState schema despite totally different wire formats")


asyncio.run(main())
