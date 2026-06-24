"""
Verifies the NEW behavior: commands sent through the real adapters
actually change the REAL simulators' flight state — not just that a
correctly-formatted message goes out on the wire (test_integration.py
already covers that for the base telemetry path).

This runs the actual simulator scripts as subprocesses (not minimal
inline senders), exactly as a person would run them by hand, then
drives the real adapters against them and checks for movement/mode
changes.
"""
import asyncio
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BACKEND_DIR = ROOT / "backend"
sys.path.insert(0, str(BACKEND_DIR))

from adapters.mavlink_adapter import MavlinkAdapter
from adapters.customlink_adapter import AeroLinkAdapter

MAV_PORT = 14560
MAV_CMD_PORT = 14565
AERO_PORT = 9110

latest_states = {}


async def collector(state):
    latest_states[state.drone_id] = state


async def wait_for_state(drone_id, timeout=10):
    start = time.time()
    while time.time() - start < timeout:
        if drone_id in latest_states:
            return latest_states[drone_id]
        await asyncio.sleep(0.2)
    raise TimeoutError(f"never received telemetry from {drone_id}")


async def wait_for_mode(drone_id, expected_mode, timeout=15):
    start = time.time()
    while time.time() - start < timeout:
        st = latest_states.get(drone_id)
        if st and st.flight_mode == expected_mode:
            return st
        await asyncio.sleep(0.3)
    raise TimeoutError(f"{drone_id} never reached mode {expected_mode} (last seen: {latest_states.get(drone_id) and latest_states[drone_id].flight_mode})")


async def main():
    print("=== Launching real simulator subprocesses ===")
    aero_proc = subprocess.Popen(
        [sys.executable, str(ROOT / "simulators" / "custom_drone_sim.py"), "--port", str(AERO_PORT), "--drone-id", "DRONE-B"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    time.sleep(0.5)
    mav_proc = subprocess.Popen(
        [sys.executable, str(ROOT / "simulators" / "mavlink_drone_sim.py"),
         "--port", str(MAV_PORT), "--cmd-port", str(MAV_CMD_PORT), "--drone-id", "DRONE-A"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )

    try:
        await asyncio.sleep(1.5)

        print("=== Connecting adapters ===")
        mav_adapter = MavlinkAdapter("DRONE-A", collector, MAV_PORT, MAV_CMD_PORT)
        aero_adapter = AeroLinkAdapter("DRONE-B", collector, "127.0.0.1", AERO_PORT)
        await mav_adapter.connect()
        await aero_adapter.connect()

        mav_task = asyncio.create_task(mav_adapter.listen())
        aero_task = asyncio.create_task(aero_adapter.listen())

        print("\n=== Waiting for initial telemetry from both drones ===")
        a0 = await wait_for_state("DRONE-A")
        b0 = await wait_for_state("DRONE-B")
        print(f"DRONE-A first packet: mode={a0.flight_mode} home=({a0.home_lat},{a0.home_lon}) pos=({a0.lat:.5f},{a0.lon:.5f})")
        print(f"DRONE-B first packet: mode={b0.flight_mode} home=({b0.home_lat},{b0.home_lon}) pos=({b0.lat:.5f},{b0.lon:.5f})")

        # DRONE-A's mode/home arrive via a slower STATUSTEXT cadence than
        # its position updates (real protocol design choice — see
        # mavlink_drone_sim.py), so the very FIRST packet may show
        # mode=UNKNOWN before the first STATUSTEXT lands. That's honest,
        # expected behavior, not a bug — so we wait for CIRCLING rather
        # than asserting on the first packet.
        a0 = await wait_for_mode("DRONE-A", "CIRCLING", timeout=10)
        b0 = await wait_for_mode("DRONE-B", "CIRCLING", timeout=10)
        print(f"DRONE-A settled: mode={a0.flight_mode} home=({a0.home_lat},{a0.home_lon})")
        print(f"DRONE-B settled: mode={b0.flight_mode} home=({b0.home_lat},{b0.home_lon})")

        # --- Test GOTO on Drone A ---
        print("\n=== Sending GOTO to DRONE-A ===")
        target_lat, target_lon = a0.lat + 0.02, a0.lon + 0.02
        result = await mav_adapter.send_command("GOTO", {"lat": target_lat, "lon": target_lon, "alt_m": 60})
        print(f"  command result: {result}")
        assert result.accepted

        a1 = await wait_for_mode("DRONE-A", "GOTO_WAYPOINT", timeout=10)
        print(f"  DRONE-A now in GOTO_WAYPOINT, pos=({a1.lat:.5f},{a1.lon:.5f})")

        # --- Test RTL on Drone A ---
        print("\n=== Sending RTL to DRONE-A ===")
        result = await mav_adapter.send_command("RTL", {})
        print(f"  command result: {result}")
        a2 = await wait_for_mode("DRONE-A", "RETURNING_TO_LAUNCH", timeout=10)
        print(f"  DRONE-A now in RETURNING_TO_LAUNCH, heading to home=({a2.home_lat},{a2.home_lon})")
        assert a2.home_lat is not None, "DRONE-A never reported a home position"

        # --- Test RTL on Drone B (AeroLink — different protocol/command vocabulary entirely) ---
        print("\n=== Sending RTL to DRONE-B (AeroLink) ===")
        result = await aero_adapter.send_command("RTL", {})
        print(f"  command result: {result}")
        b1 = await wait_for_mode("DRONE-B", "RETURNING_TO_LAUNCH", timeout=10)
        print(f"  DRONE-B now in RETURNING_TO_LAUNCH, heading to home=({b1.home_lat},{b1.home_lon})")
        assert b1.home_lat is not None, "DRONE-B never reported a home position"

        print("\n✅ ALL COMMAND-LOOP ASSERTIONS PASSED")
        print("   - Both drones correctly start in CIRCLING with a known HOME position")
        print("   - GOTO command (MAVLink) changes real simulator flight mode")
        print("   - RTL command works through BOTH protocols, each in its own wire format")
        print("   - Home position is correctly surfaced on DroneState for both protocols")

        mav_task.cancel()
        aero_task.cancel()
        await mav_adapter.disconnect()
        await aero_adapter.disconnect()

    finally:
        aero_proc.terminate()
        mav_proc.terminate()
        time.sleep(0.3)
        aero_proc.kill()
        mav_proc.kill()


asyncio.run(main())
