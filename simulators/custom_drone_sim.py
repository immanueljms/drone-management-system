"""
Drone B — speaks a fictional proprietary protocol we're calling "AeroLink".

This stands in for a real-world proprietary manufacturer protocol: no
MAVLink, no standard schema. It's deliberately different from MAVLink in
every way that matters for the adapter layer:

  - Transport: TCP (line-delimited JSON), not MAVLink's binary UDP framing
  - Coordinates: degrees as floats (not 1e7 ints), altitude in FEET (not mm)
  - Velocity: knots, not cm/s
  - Battery: reported as a voltage curve + "ok/warn/critical" flag, not a %
  - Message shape: one flat JSON object per line, no heartbeat/position split

Now bidirectional over the SAME TCP connection: a background thread reads
incoming command lines (set_waypoint / return_to_base) sent by the
backend's AeroLinkAdapter.send_command(), while the main loop keeps
sending telemetry. TCP sockets support concurrent send/recv from
different threads safely, so this doesn't need a second port the way
the MAVLink (UDP) simulator does.

Run: python custom_drone_sim.py [--port 9100] [--drone-id DRONE-B]
"""
import argparse
import json
import math
import socket
import threading
import time

START_LAT = 12.9352
START_LON = 77.6245
START_ALT_FT = 3200.0  # ~975m, different cruise alt from Drone A on purpose

FT_PER_M = 3.28084
CRUISE_ALT_FT = 25.0  # height above HOME during normal flight
WAYPOINT_SPEED_DEG_S = 0.00006
ARRIVAL_THRESHOLD_DEG = 0.0003


def battery_status(voltage: float) -> str:
    if voltage > 14.5:
        return "ok"
    if voltage > 13.5:
        return "warn"
    return "critical"


class FlightState:
    """Same state-machine concept as the MAVLink sim's FlightState —
    see mavlink_drone_sim.py for the fuller explanation. Kept as a
    separate class here (rather than shared code) so each simulator
    stays a single, independently-readable file, matching how two
    real manufacturers' SDKs would never actually share an internal
    implementation."""

    def __init__(self, home_lat: float, home_lon: float, home_alt_ft: float):
        self.lock = threading.Lock()
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt_ft = home_alt_ft

        self.lat = home_lat
        self.lon = home_lon
        self.alt_ft = home_alt_ft + CRUISE_ALT_FT

        self.mode = "CIRCLING"
        self.target_lat = None
        self.target_lon = None
        self.target_alt_ft = None

        # Same fix as mavlink_drone_sim.py — angle tracks incrementally
        # rather than being derived from elapsed time, so resume_circling()
        # can initialize from the drone's actual current position.
        self.circle_angle = 1.0       # offset from zero so it doesn't overlap with Drone A
        self.CIRCLE_SPEED = -0.005    # negative = clockwise, different direction from Drone A
        self.RADIUS_DEG = 0.008

        self.voltage = 16.8

    def set_goto(self, lat: float, lon: float, alt_ft: float):
        with self.lock:
            self.mode = "GOTO_WAYPOINT"
            self.target_lat = lat
            self.target_lon = lon
            self.target_alt_ft = alt_ft
        print(f"[state] -> GOTO_WAYPOINT ({lat:.5f}, {lon:.5f}, {alt_ft:.0f}ft)")

    def set_rtl(self):
        with self.lock:
            self.mode = "RETURNING_TO_LAUNCH"
            self.target_lat = self.home_lat
            self.target_lon = self.home_lon
            self.target_alt_ft = self.home_alt_ft + CRUISE_ALT_FT
        print(f"[state] -> RETURNING_TO_LAUNCH (home: {self.home_lat:.5f}, {self.home_lon:.5f})")

    def resume_circling(self):
        with self.lock:
            dlat = self.lat - self.home_lat
            dlon = self.lon - self.home_lon
            self.circle_angle = math.atan2(dlat, dlon)
            self.mode = "CIRCLING"
            self.target_lat = None
            self.target_lon = None
        print("[state] -> CIRCLING (smooth resume)")


def command_reader(conn: socket.socket, state: FlightState, drone_id: str):
    """
    Background thread: reads incoming JSON lines from the same TCP
    connection we're sending telemetry on, and applies them to `state`.
    This is the half of the loop that was previously missing.
    """
    buffer = b""
    while True:
        try:
            chunk = conn.recv(4096)
        except OSError:
            return  # connection closed/replaced; thread exits, a new one is started on reconnect
        if not chunk:
            return
        buffer += chunk
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                cmd = json.loads(line.decode("utf-8"))
            except json.JSONDecodeError:
                continue
            if cmd.get("msg_type") != "cmd":
                continue

            action = cmd.get("action")
            if action == "return_to_base":
                state.set_rtl()
            elif action == "set_waypoint":
                state.set_goto(cmd["lat_deg"], cmd["lon_deg"], cmd.get("alt_ft", CRUISE_ALT_FT))


def step_position(state: FlightState):
    """Mirrors mavlink_drone_sim.py's step_position — see that file for
    the fuller explanation of the CIRCLING / GOTO / RTL state machine.
    Returns (lat, lon, alt_ft, speed_kt, heading_deg)."""
    with state.lock:
        mode = state.mode
        lat, lon, alt_ft = state.lat, state.lon, state.alt_ft
        target_lat, target_lon, target_alt_ft = state.target_lat, state.target_lon, state.target_alt_ft
        home_lat, home_lon, home_alt_ft = state.home_lat, state.home_lon, state.home_alt_ft

    if mode == "CIRCLING":
        with state.lock:
            state.circle_angle += state.CIRCLE_SPEED
            angle = state.circle_angle
            radius_deg = state.RADIUS_DEG

        new_lat = home_lat + radius_deg * math.sin(angle)
        new_lon = home_lon + radius_deg * math.cos(angle)
        new_alt_ft = home_alt_ft + CRUISE_ALT_FT
        heading = (math.degrees(angle) + 360) % 360
        speed_kt = 18.5
        with state.lock:
            state.lat, state.lon, state.alt_ft = new_lat, new_lon, new_alt_ft
        return new_lat, new_lon, new_alt_ft, speed_kt, heading

    dlat = target_lat - lat
    dlon = target_lon - lon
    dist = math.sqrt(dlat**2 + dlon**2)

    if dist <= ARRIVAL_THRESHOLD_DEG:
        with state.lock:
            state.lat, state.lon, state.alt_ft = target_lat, target_lon, target_alt_ft
        state.resume_circling()
        return target_lat, target_lon, target_alt_ft, 0.0, 0.0

    step = min(WAYPOINT_SPEED_DEG_S, dist)
    new_lat = lat + (dlat / dist) * step
    new_lon = lon + (dlon / dist) * step
    heading = (math.degrees(math.atan2(dlon, dlat)) + 360) % 360
    speed_kt = 22.0  # plausible transit speed, knots

    with state.lock:
        state.lat, state.lon = new_lat, new_lon

    return new_lat, new_lon, alt_ft, speed_kt, heading


def serve(port: int, drone_id: str):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(1)
    print(f"[{drone_id}] AeroLink sim listening on tcp://127.0.0.1:{port}")
    print(f"[{drone_id}] waiting for backend to connect...")

    state = FlightState(START_LAT, START_LON, START_ALT_FT)
    print(f"[{drone_id}] HOME set at ({START_LAT:.5f}, {START_LON:.5f}, {START_ALT_FT:.0f}ft)")

    conn, addr = server.accept()
    print(f"[{drone_id}] backend connected from {addr}")
    conn.settimeout(None)

    reader_thread = threading.Thread(target=command_reader, args=(conn, state, drone_id), daemon=True)
    reader_thread.start()

    t0 = time.time()

    try:
        while True:
            elapsed = time.time() - t0
            lat, lon, alt_ft, speed_kt, heading = step_position(state)

            if int(elapsed) % 15 == 0:
                with state.lock:
                    state.voltage = max(state.voltage - 0.02, 12.0)

            with state.lock:
                voltage = state.voltage
                home_lat_for_payload = state.home_lat
                home_lon_for_payload = state.home_lon

            payload = {
                "msg_type": "telemetry",
                "unit_callsign": drone_id,
                "pos": {"lat_deg": round(lat, 6), "lon_deg": round(lon, 6), "alt_ft": round(alt_ft, 1)},
                "home": {"lat_deg": home_lat_for_payload, "lon_deg": home_lon_for_payload},
                "speed_kt": round(speed_kt, 1),
                "heading_deg": round(heading, 1),
                "power": {"volts": round(voltage, 2), "status": battery_status(voltage)},
                "link_quality_pct": 92,
                "flight_mode": state.mode,
                "ts_unix": time.time(),
            }

            line = (json.dumps(payload) + "\n").encode("utf-8")
            try:
                conn.sendall(line)
            except (BrokenPipeError, ConnectionResetError, OSError):
                print(f"[{drone_id}] backend disconnected, waiting for reconnect...")
                conn.close()
                conn, addr = server.accept()
                print(f"[{drone_id}] backend reconnected from {addr}")
                conn.settimeout(None)
                reader_thread = threading.Thread(target=command_reader, args=(conn, state, drone_id), daemon=True)
                reader_thread.start()

            time.sleep(0.5)
    finally:
        conn.close()
        server.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulated AeroLink (custom protocol) drone")
    parser.add_argument("--port", type=int, default=9100, help="TCP port to serve on")
    parser.add_argument("--drone-id", type=str, default="DRONE-B", help="human-readable id")
    args = parser.parse_args()
    serve(args.port, args.drone_id)
