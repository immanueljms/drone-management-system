# Integrated Drone Management System — Proof of Concept

Challenge 16 (DISC 14): "Lack of an integrated system to monitor, manage
and provide decision support for drone operations across multiple
platforms and locations."

This PoC proves the core architectural claim: **one backend, one
dashboard, two drones speaking two completely different protocols,
normalized into a single unified view — with commands flowing back
through each drone's native protocol and actually changing each
simulated drone's real flight behavior (not just transmitting into the
void).**

- **Drone A** speaks real **MAVLink** (the open protocol PX4/ArduPilot use)
- **Drone B** speaks a fictional proprietary protocol we call **AeroLink**
  (JSON over TCP, different units, different command vocabulary —
  standing in for a real manufacturer's closed protocol)

Both simulators record a real **home/launch position** at startup and
run a small flight-mode state machine (`CIRCLING` → `GOTO_WAYPOINT` /
`RETURNING_TO_LAUNCH` → back to `CIRCLING` on arrival), so RTL and GOTO
commands sent from the dashboard cause genuine, observable changes —
not just a correctly-formatted message that nothing reacts to.

Both are simulated (no physical hardware or PX4/Gazebo required for this
PoC), but the parsing/conversion code is genuine — swapping the MAVLink
simulator for real PX4 SITL later requires zero changes to the adapter
or anything downstream of it.

## What this proves (& what it doesnt)

Proves: protocol-agnostic ingestion, unit normalization across
   wildly different wire formats, a unified dashboard, and a
   bidirectional command channel — the actual hard engineering problem
   named in Challenge 16's "manage" and "monitor" requirements.

Does not attempt: real proprietary military protocol access (the
   access/partnership problem we can't solve by building ), autonomous multi-drone coordination, or production
   failure-mode handling.

## Project structure

```
folder/
├── simulators/
│   ├── mavlink_drone_sim.py      # Drone A — real MAVLink over UDP
│   └── custom_drone_sim.py       # Drone B — fictional AeroLink JSON/TCP
├── backend/
│   ├── main.py                   # FastAPI app — REST + WebSocket
│   ├── registry.py               # manages adapters, fans out updates
│   ├── adapters/
│   │   ├── base.py               # DroneState schema + DroneAdapter interface
│   │   ├── mavlink_adapter.py    # MAVLink parsing/commands
│   │   └── customlink_adapter.py # AeroLink parsing/commands
│   └── requirements.txt
├── frontend/
│   └── dashboard.html            # single-file React+Leaflet dashboard (no build step)
└── test_integration.py           # correctness test for adapter parsing/unit conversion
└── test_command_loop.py          # proves RTL/GOTO actually change real simulator behavior
```

## Setup

```bash
# from folder/
pip install -r backend/requirements.txt
```

(`pymavlink`, `fastapi`, `uvicorn`, `pydantic`, `websockets` — that's the
whole dependency list.)

## Running it (requires 3 terminals)

**Terminal 1 — Drone A (MAVLink):**
```bash
cd simulators
python3 mavlink_drone_sim.py --port 14550 --cmd-port 14555 --drone-id DRONE-A
```
`--port` is where it sends telemetry; `--cmd-port` (default 14555) is
where it listens for incoming RTL/GOTO commands — a separate UDP
socket from telemetry, since UDP is connectionless and a dedicated
send/receive pair is simpler and more reliable than trying to reuse one
socket bidirectionally. The backend's default `udp_command_port` in
`main.py` already matches this default, so you don't need to pass
`--cmd-port` unless you're changing the port.

**Terminal 2 — Drone B (AeroLink), start this BEFORE the backend:**
```bash
cd simulators
python3 custom_drone_sim.py --port 9100 --drone-id DRONE-B
```
It will print "waiting for backend to connect..." — that's expected,
AeroLink is a TCP server here and the backend connects to it as a client
(this mirrors how you'd actually integrate with a manufacturer's onboard
TCP/JSON telemetry service).

**Terminal 3 — Backend:**
```bash
cd backend
uvicorn main:app --reload --port 8000
```
You should see both adapters come online:
```
[DRONE-A] MAVLink adapter listening on udpin:127.0.0.1:14550
[registry] DRONE-A (MavlinkAdapter) online
[DRONE-B] AeroLink adapter connecting to tcp://127.0.0.1:9100
[DRONE-B] AeroLink adapter connected
[registry] DRONE-B (AeroLinkAdapter) online
```

**Dashboard:** just open `frontend/dashboard.html` directly in a browser
(double-click it, or `open frontend/dashboard.html`). No build step, no
npm install — it loads React and Leaflet from CDN. You should see:
- A rotating drone-shaped icon per drone (quadcopter silhouette, points
  in its current heading direction), color-coded by protocol
- A distinct house-shaped marker showing each drone's **launch/home
  position**, placed once it's reported and never moving afterward
- A faint dashed line connecting each drone to its home point
- Live-updating sidebar cards, including a flight-mode badge
  (CIRCLING / GOTO WAYPOINT / RETURNING TO LAUNCH) and the launch
  point's coordinates

Note: `file://` (double-clicking the HTML file directly) can hit
browser security restrictions on some setups. If the page loads blank
or you see script errors in the console, serve it instead:
```bash
cd frontend && python3 -m http.server 5500
```
then open `http://localhost:5500/dashboard.html`.

## Verifying correctness without the UI

**Telemetry parsing + unit conversion:**
```bash
python3 test_integration.py
```
Runs both adapters against minimal synthetic senders in a single
process and asserts the unit conversions are exactly correct (e.g.
MAVLink's `alt=925000` mm decodes to `925.0` m; AeroLink's `alt_ft=3200`
converts to `975.4` m).

**The command loop (RTL / GOTO actually working):**
```bash
python3 test_command_loop.py
```
Launches the real simulator scripts as subprocesses, sends real RTL and
GOTO commands through the real adapters, and asserts that each
simulator's `flight_mode` actually changes in response — proving the
command channel isn't just transmitting into the void.

## What "Return to Launch" and "Nudge waypoint" actually do

**Launch / home position:** each simulator picks a starting coordinate
at startup and treats it as HOME for the rest of that run — this is
the "launch point" referenced everywhere in the UI. It's recorded once
and reported on every telemetry update via `home_lat`/`home_lon`.

**Return to Launch (RTL):** sends a real `MAV_CMD_NAV_RETURN_TO_LAUNCH`
(MAVLink) or `{"action": "return_to_base"}` (AeroLink). On receipt, the
simulator switches its internal flight-mode state to
`RETURNING_TO_LAUNCH` and starts computing a straight-line path back to
its stored HOME coordinate. Once it arrives (within ~30m), it
automatically resumes its default `CIRCLING` patrol pattern around
HOME. The dashboard's RTL button disables itself while a drone is
mid-return, and the home marker + dashed line let you visually confirm
it's actually heading there.

**Nudge waypoint:** sends a `GOTO` command to a point ~220m north of
the drone's position at the moment you click. This switches the
simulator into `GOTO_WAYPOINT` mode, which behaves identically to RTL's
navigation logic but targets the commanded point instead of HOME —
once it arrives, it likewise resumes `CIRCLING`, but now around HOME
(not the waypoint), matching how a real return-to-pattern behavior
would work after an ad hoc tasking.

## Order of operations matters

Start things in this order: **Drone B sim → Drone A sim → backend →
dashboard**. AeroLink's simulator is a single-client TCP server in this
PoC version, so it needs to be listening before the backend tries to
connect to it. (A production version would make this more robust with
retry/reconnect logic on both sides — noted as a known simplification.)

## What to extend next (Phase 6–7)

- **Phase 5: done and verified.** Command channel exists for both
  protocols, and both simulators genuinely react — flight mode changes,
  navigation happens, RTL returns to a real stored home position. See
  `test_command_loop.py`.
- **Phase 6:** decision-support is still just the battery warning
  banner; extend with geofencing and proximity-conflict checks (you
  now have `flight_mode` and `home_lat`/`home_lon` on every
  `DroneState`, which a geofence check could use directly).
- **Phase 7:** the adapter pattern is already in place — adding a third
  protocol means writing one new `adapters/your_protocol_adapter.py`
  implementing `DroneAdapter`, with zero changes to `main.py`'s routes,
  the registry, or the dashboard.
- **Swap in real PX4 SITL:** once Gazebo/PX4 is running locally, point
  `MavlinkAdapter` at SITL's UDP port instead of `mavlink_drone_sim.py`
  — no adapter code changes needed, since SITL speaks the same MAVLink
  messages our simulator does. Note real PX4 already implements its own
  RTL/waypoint logic natively, so our simulator's hand-rolled flight
  state machine becomes unnecessary at that point — it only exists here
  to stand in for what a real autopilot does internally.

## Known simplifications 


- AeroLink's battery is voltage-derived, not a true percentage — flagged
  explicitly in `customlink_adapter.py`'s comments, since real platforms
  vary in what fidelity of data they actually expose.
- Commands have no acknowledgment-tracking or retry logic if a command
  packet is lost in transit (the simulator reacting and the next
  telemetry tick reflecting it is the only confirmation mechanism right
  now — there's no explicit "command received" ack message).
- Single AeroLink client connection only (fine for a PoC with one
  backend instance; would need a proper connection pool for production).
- The flight-mode state machine (CIRCLING / GOTO_WAYPOINT /
  RETURNING_TO_LAUNCH) is a simplified stand-in for what a real
  autopilot (PX4/ArduPilot) already does internally — it exists purely
  so this PoC can demonstrate closed-loop command behavior without
  needing a real flight controller.
