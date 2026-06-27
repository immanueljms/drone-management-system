"""
Drone A — speaks real MAVLink over UDP. Now bidirectional: emits
telemetry AND listens for incoming commands (GOTO, RTL), reacting to
them with real (if simplified) flight-state changes.

Flight state machine:
  CIRCLING            -> default behavior, loops around the home position
  GOTO_WAYPOINT        -> flies in a straight line toward a commanded point
  RETURNING_TO_LAUNCH  -> flies in a straight line back to HOME, then
                          switches back to CIRCLING once arrived

HOME is recorded once, at startup, before any movement begins — exactly
like a real flight controller records its launch point at arm time.
This is what "Return to Launch" actually returns to.

Run: python mavlink_drone_sim.py [--port 14550] [--drone-id DRONE-A]
"""
import argparse
import math
import threading
import time

from pymavlink import mavutil

# Bengaluru-ish starting point, arbitrary
START_LAT = 12.9716
START_LON = 77.5946
START_ALT_M = 920.0

CRUISE_ALT_M = 40.0       # height above HOME during normal flight
WAYPOINT_SPEED_DEG_S = 0.00015  # fast enough to complete a ~200m GOTO in ~7s
ARRIVAL_THRESHOLD_DEG = 0.0003  # ~30m — "close enough" to call a waypoint reached


class FlightState:
    """Mutable flight state shared between the telemetry-sender loop
    and the command-listener thread. Lock included for explicitness
    even though Python's GIL makes the simple read/write pattern here
    low-risk — being explicit beats relying on interpreter internals."""

    def __init__(self, home_lat: float, home_lon: float, home_alt: float):
        self.lock = threading.Lock()
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt = home_alt

        self.lat = home_lat
        self.lon = home_lon
        self.alt_m = home_alt + CRUISE_ALT_M

        self.mode = "CIRCLING"  # CIRCLING | GOTO_WAYPOINT | RETURNING_TO_LAUNCH
        self.target_lat = None
        self.target_lon = None
        self.target_alt = None

        # Circle angle tracked as incrementally-advancing state rather than
        # computed from wall-clock elapsed time. This is the fix for the
        # "teleport on GOTO completion" bug:
        #   Old: angle = elapsed * 0.15  → when CIRCLING resumes, elapsed
        #        has advanced far, placing the drone at a random point on the
        #        home circle regardless of where it physically just arrived.
        #   New: angle advances by a fixed step each tick, and when CIRCLING
        #        resumes after a transit, it's initialized to atan2 of the
        #        drone's current offset from home — so the circle continues
        #        smoothly from where the drone actually is, no jump.
        self.circle_angle = 0.0
        self.CIRCLE_SPEED = 0.006   # radians per tick at 0.5s interval ≈ 0.15 rad/s
        self.RADIUS_DEG = 0.003     # ~330m radius — proportionate to GOTO distances

        self.battery_pct = 95

    def set_goto(self, lat: float, lon: float, alt_m: float):
        with self.lock:
            self.mode = "GOTO_WAYPOINT"
            self.target_lat = lat
            self.target_lon = lon
            self.target_alt = alt_m
        print(f"[state] -> GOTO_WAYPOINT ({lat:.5f}, {lon:.5f}, {alt_m:.0f}m)")

    def set_rtl(self):
        with self.lock:
            self.mode = "RETURNING_TO_LAUNCH"
            self.target_lat = self.home_lat
            self.target_lon = self.home_lon
            self.target_alt = self.home_alt + CRUISE_ALT_M
        print(f"[state] -> RETURNING_TO_LAUNCH (home: {self.home_lat:.5f}, {self.home_lon:.5f})")

    def resume_circling(self, from_lat: float = None, from_lon: float = None):
        """
        Resume CIRCLING smoothly from a known position.

        `from_lat`/`from_lon` should be the drone's actual position at
        the moment of arrival — passed explicitly because self.lat/lon
        may already have been updated by a previous call, making the
        angle calculation unreliable.

        The angle is derived from atan2 of (from_pos - home), then
        self.lat/lon is immediately snapped to the exact circle point
        at that angle so the next step_position() tick is continuous.
        """
        with self.lock:
            ref_lat = from_lat if from_lat is not None else self.lat
            ref_lon = from_lon if from_lon is not None else self.lon
            dlat = ref_lat - self.home_lat
            dlon = ref_lon - self.home_lon
            # If we're arriving right at home (RTL case), the offset is
            # near-zero — use the last known angle to avoid atan2(0,0)=0
            # snapping the drone to the east side of the circle.
            if abs(dlat) < 1e-6 and abs(dlon) < 1e-6:
                # keep self.circle_angle as-is (last approach angle)
                pass
            else:
                self.circle_angle = math.atan2(dlat, dlon)
            self.lat = self.home_lat + self.RADIUS_DEG * math.sin(self.circle_angle)
            self.lon = self.home_lon + self.RADIUS_DEG * math.cos(self.circle_angle)
            self.mode = "CIRCLING"
            self.target_lat = None
            self.target_lon = None
        print("[state] -> CIRCLING (smooth resume)")


def command_listener(state: FlightState, listen_port: int, drone_id: str):
    """
    Runs in its own thread. Binds a second MAVLink connection (a UDP
    *input* socket) purely to receive COMMAND_LONG messages sent by the
    backend's MavlinkAdapter.send_command(), and updates `state`
    accordingly. This is the half of the loop that was previously
    missing — commands were sent correctly but nothing was listening.
    """
    conn_str = f"udpin:127.0.0.1:{listen_port}"
    print(f"[{drone_id}] command listener binding -> {conn_str}")
    cmd_mav = mavutil.mavlink_connection(conn_str)

    while True:
        msg = cmd_mav.recv_match(type="COMMAND_LONG", blocking=True)
        if msg is None:
            continue

        if msg.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            state.set_rtl()
        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            # param5/6/7 carry lat/lon/alt, matching command_long_send's
            # call signature in mavlink_adapter.py's send_command()
            state.set_goto(msg.param5, msg.param6, msg.param7)


def step_position(state: FlightState):
    """
    Compute (lat, lon, alt_m, vx_cms, vy_cms) for the current tick,
    based on state.mode. Velocity returned in cm/s to match MAVLink's
    GLOBAL_POSITION_INT field units directly.
    """
    with state.lock:
        mode = state.mode
        lat, lon, alt = state.lat, state.lon, state.alt_m
        target_lat, target_lon, target_alt = state.target_lat, state.target_lon, state.target_alt
        home_lat, home_lon, home_alt = state.home_lat, state.home_lon, state.home_alt

    if mode == "CIRCLING":
        with state.lock:
            # Advance the angle by one fixed step per tick — smooth,
            # continuous, and independent of wall-clock elapsed time.
            state.circle_angle += state.CIRCLE_SPEED
            angle = state.circle_angle
            radius_deg = state.RADIUS_DEG

        new_lat = home_lat + radius_deg * math.sin(angle)
        new_lon = home_lon + radius_deg * math.cos(angle)
        new_alt = home_alt + CRUISE_ALT_M
        vx = math.cos(angle) * 300
        vy = -math.sin(angle) * 300
        with state.lock:
            state.lat, state.lon, state.alt_m = new_lat, new_lon, new_alt
        return new_lat, new_lon, new_alt, vx, vy

    # GOTO_WAYPOINT or RETURNING_TO_LAUNCH: both are "fly straight toward target_*"
    dlat = target_lat - lat
    dlon = target_lon - lon
    dist = math.sqrt(dlat**2 + dlon**2)

    if dist <= ARRIVAL_THRESHOLD_DEG:
        # Pass the final approach position explicitly so resume_circling()
        # can compute a meaningful atan2 angle regardless of whether the
        # target is home (RTL) or a waypoint far from home (GOTO).
        state.resume_circling(from_lat=lat, from_lon=lon)
        return state.lat, state.lon, state.alt_m, 0.0, 0.0

    step = min(WAYPOINT_SPEED_DEG_S, dist)
    new_lat = lat + (dlat / dist) * step
    new_lon = lon + (dlon / dist) * step
    vx = (dlat / dist) * 600
    vy = (dlon / dist) * 600

    with state.lock:
        state.lat, state.lon = new_lat, new_lon

    return new_lat, new_lon, alt, vx, vy


def run(udp_port: int, drone_id: str, system_id: int, cmd_port: int):
    conn_str = f"udpout:127.0.0.1:{udp_port}"
    print(f"[{drone_id}] MAVLink sim starting -> {conn_str} (system_id={system_id})")
    mav = mavutil.mavlink_connection(conn_str, source_system=system_id)

    state = FlightState(START_LAT, START_LON, START_ALT_M)
    print(f"[{drone_id}] HOME set at ({START_LAT:.5f}, {START_LON:.5f}, {START_ALT_M:.0f}m)")

    # Announce HOME once via STATUSTEXT — a real MAVLink message type
    # meant for free-text status — so the backend adapter can pick it
    # up without us inventing a new message type. Real GCS software
    # typically learns home position from the first GPS-valid position
    # after arming; STATUSTEXT is a simpler stand-in for that here.
    home_announcement = f"HOME:{START_LAT:.7f},{START_LON:.7f}"
    mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, home_announcement.encode("utf-8")[:50])

    listener_thread = threading.Thread(
        target=command_listener, args=(state, cmd_port, drone_id), daemon=True
    )
    listener_thread.start()

    t0 = time.time()
    heartbeat_interval = 1.0
    position_interval = 0.5
    status_interval = 0.5  # same cadence as position, so mode/home arrive promptly rather than flickering UNKNOWN for ~2s after connect
    last_heartbeat = 0.0
    last_position = 0.0
    last_status = 0.0
    last_battery_tick = -1
    last_announced_mode = None

    while True:
        now = time.time()
        elapsed = now - t0

        if now - last_heartbeat >= heartbeat_interval:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_PX4,
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            last_heartbeat = now

        if now - last_status >= status_interval:
            # Re-send HOME periodically (not just once at startup) so an
            # adapter that connects a few seconds after this sim started
            # still learns the home position. Also announce flight mode
            # on every status tick so the dashboard can show it live.
            with state.lock:
                mode = state.mode
            mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"HOME:{START_LAT:.7f},{START_LON:.7f}".encode("utf-8")[:50],
            )
            mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"MODE:{mode}".encode("utf-8")[:50],
            )
            last_status = now

        if now - last_position >= position_interval:
            lat, lon, alt_m, vx, vy = step_position(state)

            mav.mav.global_position_int_send(
                int(elapsed * 1000),
                int(lat * 1e7),
                int(lon * 1e7),
                int(alt_m * 1000),
                int((alt_m - START_ALT_M) * 1000),
                int(vx), int(vy), 0,
                0,
            )

            tick = int(elapsed) // 20
            if tick != last_battery_tick:
                with state.lock:
                    state.battery_pct = max(state.battery_pct - 1, 0)
                last_battery_tick = tick

            mav.mav.sys_status_send(
                0, 0, 0, 500, 12600, -1, state.battery_pct, 0, 0, 0, 0, 0, 0,
            )
            last_position = now

        time.sleep(0.05)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulated MAVLink drone")
    parser.add_argument("--port", type=int, default=14550, help="UDP port the backend listens on for telemetry")
    parser.add_argument("--cmd-port", type=int, default=14555, help="UDP port this sim listens on for incoming commands")
    parser.add_argument("--drone-id", type=str, default="DRONE-A", help="human-readable id for logs")
    parser.add_argument("--system-id", type=int, default=1, help="MAVLink system_id")
    args = parser.parse_args()
    run(args.port, args.drone_id, args.system_id, args.cmd_port)