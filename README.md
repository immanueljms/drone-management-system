# Integrated Drone Management System – DRDO Proof of Concept

**Context:** Problem Statement 16 (DISC 14) – Indian Army  
*"Lack of an integrated system to monitor, manage and provide decision support for drone operations across multiple platforms and locations."*

This Proof of Concept (PoC) demonstrates a **centralised command-and-control (C2) centre** designed for the Tactical Battle Area (TBA). It proves the core architectural claim: **one unified backend and tactical dashboard concurrently managing multiple heterogeneous UAS (Tethered and Untethered) speaking completely different OEM protocols.**

---

## Setup & Execution

### 1. Environment Setup
Install the required dependencies (requires Python 3.8+):
`pip install -r backend/requirements.txt`

### 2. Running the System (Requires 4 Terminals)

**Terminal 1 – Tethered UAS (TCP Server):**
*Must be started BEFORE the backend so the TCP socket is ready to accept connections.*
`cd simulators`
`python3 custom_drone_sim.py --port 9100 --drone-id UAS-BRAVO-TETHERED`

**Terminal 2 – Networked Mobile UAS (UDP Sender):**
`cd simulators`
`python3 mavlink_drone_sim.py --port 14550 --cmd-port 14555 --drone-id UAS-ALPHA-MOBILE`

**Terminal 3 – C2 Backend:**
`cd backend`
`uvicorn main:app --reload --port 8000`
*Note: Watch this terminal to verify both drone adapters successfully connect to the simulators.*

**Terminal 4 – Tactical Dashboard (Frontend):**
To avoid browser strict CORS/local-file restrictions with the CDN scripts, serve the frontend folder:
`cd frontend`
`python3 -m http.server 5500`

Open your web browser and navigate to: **http://localhost:5500/dashboard.html**

---

## System Architecture

*   **UAS-ALPHA-MOBILE (Networked UAS):** Simulates an untethered, mobile asset. Communicates via standard **MAVLink** over UDP (used by PX4/ArduPilot).
*   **UAS-BRAVO-TETHERED (Tethered UAS):** Simulates a persistent observation asset. Communicates via a proprietary OEM protocol (**AeroLink**) using JSON over a raw TCP socket.

Both simulators record a genuine home/launch position at startup and run an internal flight-mode state machine (`CIRCLING` -> `GOTO_WAYPOINT` / `RETURNING_TO_LAUNCH` -> `CIRCLING`). Commands issued from the C2 dashboard dictate real, observable navigation changes.

### Key Demonstrated Features for Pitch:
*   **Protocol-Agnostic Ingestion:** A modular adapter pattern that ingests both UDP binary frames and TCP JSON payloads, converting altitude, velocity, and coordinates into a normalized schema.
*   **Tactical Dashboard (UI):** A dark-themed, military-grade interface displaying real-time positions, custom base markers, dynamic return-path lines, and AES-256 encryption status.
*   **Simulated Decision Support:** Synthetic EO/IR video feeds featuring mock AI/ML object detection overlays ("PERSONNEL DETECTED") to simulate the edge-analytics requirement.
*   **Closed-Loop Command:** Bi-directional architecture allowing operators to task waypoints or trigger RTL. The backend translates these into the specific wire format required by each respective drone.

---

## Project Structure

```text
.
├── simulators/
│   ├── mavlink_drone_sim.py      # Mobile UAS – MAVLink over UDP
│   └── custom_drone_sim.py       # Tethered UAS – Fictional OEM JSON/TCP
├── backend/
│   ├── main.py                   # FastAPI app – REST + WebSocket
│   ├── registry.py               # Manages active adapters and fans out updates
│   ├── adapters/
│   │   ├── base.py               # Unified DroneState schema
│   │   ├── mavlink_adapter.py    # MAVLink parsing/commands
│   │   └── customlink_adapter.py # Proprietary OEM parsing/commands
│   └── requirements.txt
├── frontend/
│   └── dashboard.html            # Tactical C2 React+Leaflet Dashboard
├── test_integration.py           # Unit normalization correctness tests
└── test_command_loop.py          # Automated CLI testing for RTL/GOTO
```
## Testing & Verification
If you wish to run the automated Python tests without the UI, ensure you update the target callsigns in `test_command_loop.py` from the legacy `DRONE-A`/`DRONE-B` to the new `UAS-ALPHA-MOBILE`/`UAS-BRAVO-TETHERED` strings.

Run the unit parsing test (verifies altitude/coordinate math conversions):
`python3 test_integration.py`

## What this PoC Simulates vs. Solves
**Solved (Real Code):**
*   Heterogeneous datalink ingestion and normalization.
*   Bi-directional UDP/TCP command channeling.
*   Live geospatial rendering and WebSocket telemetry distribution.

**Simulated (Mocked for Pitch Context):**
*   **Video / Analytics:** The live video feed and AI bounding boxes are synthetic visual representations of the end-goal software capabilities.
*   **Encryption:** The "AES-256" badges denote the architecture's capacity to monitor mesh network security states, but the local simulation runs over unencrypted localhost sockets for demonstration ease.