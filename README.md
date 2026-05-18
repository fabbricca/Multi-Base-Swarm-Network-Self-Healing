# Autonomous Network Self-Healing with Drones

A self-healing protocol for drone swarms that autonomously detects loss of connectivity with a base station and reorganizes to form multi-hop relay chains, restoring network integrity without centralized control.

## Overview

This project implements a decentralized self-healing protocol where drones in a swarm can detect communication failures and cooperatively reposition themselves to maintain connectivity with a base station. The system uses:

- **UWB (Ultra-Wideband) PHY-layer communication** for all node-to-node data exchange
- **UWB ToF (Time-of-Flight) trilateration** for GPS-free drone localization
- **ACK-based reachability detection** to identify connectivity loss
- **Flooding-based hop discovery** to establish distance metrics to the base station
- **Periodic neighbor broadcasts** for local situational awareness
- **Virtual spring-damper formation control** for coordinated repositioning

When a drone loses direct communication with the base station (detected via missing acknowledgments), it broadcasts a `HELP_PROXY` request. Neighboring drones then activate their formation controllers, repositioning to form a multi-hop relay chain that restores end-to-end connectivity.

## Architecture

### System Components

The architecture is designed around three node types:

| Component | Description |
|-----------|-------------|
| **Base Station** | Stationary node that tracks drone positions, sends acknowledgments, triggers periodic floods, and acts as a UWB anchor |
| **UWB Anchors** | Static infrastructure nodes that broadcast timestamped beacons for drone trilateration |
| **Drones** | Mobile agents running the same control stack with unicast (to base) and broadcast (swarm) communication, localized via UWB trilateration |

### Module Structure

```
├── apps/                          # Executable applications
│   └── help_proxy_sim.cpp         # Main simulation scenario
├── common/                        # Shared data structures
│   ├── messages.h                 # Protocol message definitions (incl. UwbBeaconMsg)
│   ├── packet.h                   # Packet envelope format
│   └── vector3D.h                 # 3D vector utilities
├── interfaces/                    # Abstract interfaces for modularity
│   ├── position.h                 # Position interface
│   ├── uwb_ranging_manager.h      # UWB ranging interface
│   └── ...
├── modules/
│   ├── communication/             # Packet packing/unpacking & transport delegation
│   ├── controller/                # Virtual spring-damper formation control
│   ├── dispatch/                  # Message routing to protocol handlers
│   ├── flood/                     # Hop discovery via flooding protocol
│   ├── neighbor/                  # Local neighbor state management
│   └── uwb_ranging/              # UWB trilateration engine & position adapter
│       ├── uwb_ranging_manager.*  # ToF-based ranging + least-squares trilateration
│       └── uwb_position.h        # PositionInterface adapter for trilaterated coords
└── platform/ns3/                  # NS-3 specific implementations
    ├── base_station/              # NS-3 base station (extends UWB anchor)
    ├── custom_mobility/           # Kinematic mobility model
    ├── drone/                     # NS-3 drone node logic
    ├── position/                  # NS-3 position interface (used by anchors)
    ├── uwb_anchor/                # UWB anchor beacon broadcaster
    ├── uwb_channel/               # PHY-level UWB channel (propagation delay model)
    ├── uwb_transport/             # Transport interface over UWB channel
    └── velocity_actuator/         # Velocity command application
```

### Communication Stack

All communication uses a single **UWB PHY-layer channel** — no WiFi, no MAC layer, no IP stack, no UDP sockets.

```
Application (drone/base logic)
        ↓
CommunicationManager (serialization)
        ↓
UwbTransport (unicast/broadcast)
        ↓
UwbChannel (PHY propagation: distance/c delay, range cutoff)
```

**DispatchManager** routes received packets to appropriate handlers:
- `FloodManager` → Hop discovery floods
- `NeighborManager` → Neighbor state updates
- `UwbRangingManager` → UWB beacon processing and trilateration
- Node Logic → Position updates, ACKs, and help requests

### Localization

Drones have **no GPS**. Position is determined entirely through UWB trilateration:

1. **UWB anchors** (static, known positions) broadcast beacons at 100 Hz containing their ID, position, and a PHY-level timestamp
2. **Drones** receive beacons and compute range from Time-of-Flight: `range = (rx_time - tx_timestamp) × speed_of_light`
3. **UwbRangingManager** trilaterates from 3+ anchor ranges using linearized least-squares (subtracts first anchor's sphere equation, solves via normal equations + Cramer's rule)
4. **UwbPosition** wraps the ranging manager as a `PositionInterface`, providing trilaterated coordinates to all consumers (controller, neighbor broadcasts, position updates)

Stale anchor measurements (>0.5s without a beacon) are automatically expired, so trilateration always reflects the current set of reachable anchors. The solver detects coplanar anchors and uses 2D mode to avoid degenerate 3D systems.

## Self-Healing Protocol

### Formation Control

The controller uses a virtual spring-damper model combining linear attraction to neighbors at adjacent hop levels with 1/d² short-range repulsion:

**Net Force:**
```
F_tot = Σ K_att(p_j - p_i)  [j ∈ neighbors at hop h±1]
      - Σ (K_rep/d_ij²) × (p_j - p_i)/d_ij  [j ∈ neighbors, d_ij < D_safe]
```

The attractive term drives drones toward the geometric midpoint between their preceding and following neighbors in the relay chain, maximizing SNR on both links. Repulsion applies to all neighbors, including peers at the same hop count.

**Acceleration** is computed via Newton's second law:
```
a = F_tot / m
```

This is then integrated into velocity and position updates using the kinematic mobility model.

### Key Parameters

| Parameter | Symbol | Description | Baseline |
|-----------|--------|-------------|----------|
| Coverage radius | R_max | UWB communication range | 50 m |
| Coverage area |  | Topology | Circular, centered at base station |
| Attractive gain | K_att | Spring constant for attraction | 0.3 |
| Repulsive gain | K_rep | Repulsion strength | 8.0 |
| Safety distance | D_safe | Minimum separation threshold | 2.0 m |
| Max velocity | V_max | Drone speed limit | 1.0 m/s |
| Drone mass | m | Based on Crazyflie 2.1 | 29 g (0.029 kg) |
| Control tick interval | T_ctrl | Position update period | 0.05 s |
| Number of drones | N_d | Baseline swarm size | 20 |
| Number of anchors | N_a | Standalone UWB anchors | 18 |

## Simulation

The simulation runs in **NS-3** (Network Simulator 3) with:
- UWB PHY-layer channel with propagation delay modeled as `distance / speed_of_light`
- Range-based cutoff (default 50 m)
- No MAC layer — direct PHY delivery with nanosecond timing resolution
- UWB anchor infrastructure for drone trilateration

### Kinematic Mobility Model

Drones use a lightweight kinematic model that integrates commanded accelerations under standard Newtonian mechanics:

```
p_new = p_old + v_old × Δt + ½a × Δt²
v_new = v_old + a × Δt
```

where:
- `p` and `v` are position and velocity vectors
- `a` is the acceleration computed from the spring-damper controller
- `Δt = 0.05 s` is the control tick interval
- Velocity magnitude is capped at `V_max` (default 1.0 m/s)

This model focuses on network-driven coordination by abstracting away aerodynamic complexity; it does not account for multipath or drag effects.

### Baseline Evaluation Scenario

- **1 base station** at origin (0, 0, 0) — also acts as UWB anchor
- **20 drones** positioned in three zones:
  - 6 helpers on a ~40 m ring just inside the 50 m coverage radius
  - 11 drones in the 68–82 m belt just outside coverage
  - 3 far scouts at 112–118 m to force multi-hop relay chain
- **18 standalone UWB anchors** guaranteeing at least 3 line-of-sight beacons everywhere
- **Evaluation**: 15 scenarios covering different topologies, swarm densities, anchor configurations, ranging noise, and scheduled drone failures
- **Metrics**: Trilateration error, healing latency, return latency, recovery rate, final coverage, and station-keeping drift

## Building and Running

### Using Docker (Recommended)

Build the Docker image:

```bash
docker build -t swarm-sim .
```

Run the simulation:

```bash
docker run --rm swarm-sim sim --animOut=/project/output/drone-simulation.xml
```

### Simulation Parameters

Pass custom parameters via command line:

```bash
docker run --rm swarm-sim sim \
  --kAtt=1.0 --kRep=8.0 --dSafe=2.0 --vMax=1.0 \
  --maxRangeMeters=50.0 --simSeconds=60.0 \
  --animOut=/project/output/drone-simulation.xml
```

### Scenario overrides

Base station, drone, and anchor positions can be overridden per-run via a
plain-text file:

```bash
docker run --rm \
  -v /path/to/scenario.txt:/tmp/scenario.txt:ro \
  -v $(pwd)/output:/output \
  swarm-sim sim --scenarioFile=/tmp/scenario.txt
```

The file format is line-oriented (`#` comments allowed); any subset of keys
may be overridden, others keep their compiled-in defaults:

```
base=0,0,0
drone1=35.0,20.0,0.0
drone4=70.0,25.0,0.0
anchor5=65.0,0.0,0.0
```

Unknown keys error out so typos can't silently no-op.

### Batch runs

`scripts/run_declaration.py` runs a batch of simulations declared in a JSON
file and reports per-metric averages across all runs:

```bash
python3 scripts/run_declaration.py scripts/simulations/declaration.json \
  --algorithm=centroid --timeout=600
```

Each declared scenario layers its own overrides on top of shared defaults; the
runner writes per-run artefacts under `output/decl_run_<timestamp>/runs/*/`
and aggregate summaries (`aggregate.json`, `aggregate.md`) at the top level.
Pass `--algorithm=weighted` to rerun the same declaration against the other
formation controller.

### Formation Control Algorithm

Select the formation-control strategy with `--algorithm` (default: `centroid`):

```bash
docker run --rm swarm-sim sim --algorithm=centroid   # hop-group centroid
docker run --rm swarm-sim sim --algorithm=weighted   # count-weighted per-neighbor
```

- `centroid` — neighbors at the same hop count contribute a single attractive
  force toward their centroid. N drones at one hop pull as a single virtual
  drone, so the equilibrium sits on the midpoint between sides.
- `weighted` — each attraction to a lower-hop neighbor is scaled by the number
  of higher-hop neighbors and vice versa. The Σw·(p−s)=0 solution places the
  equilibrium exactly at (centroid_low + centroid_high)/2.

## Visualization

To visualize the simulation, use the **NetAnim** tool included in NS-3:

1. The simulation generates `output/drone-simulation.xml`
2. Open with NetAnim following the [NS-3 animation documentation](https://www.nsnam.org/docs/release/3.46/models/html/animation.html)

## Results

Across 15 single-base evaluation scenarios (30 total runs—15 per controller variant), the system achieves:

| Metric | Centroid | Weighted |
|--------|----------|----------|
| Trilateration error (avg) | 0.47 ± 0.25 m | 0.43 ± 0.23 m |
| Healing latency | 0.92 ± 0.11 s | 0.92 ± 0.10 s |
| Return latency | 23.24 ± 12.04 s | 21.36 ± 6.19 s |
| Recovery rate | 85.24 ± 28.11 % | 83.84 ± 33.24 % |
| Final coverage | 98.63 ± 3.61 % | 98.63 ± 3.61 % |

The weighted controller variant reduces mean return latency by **8%** and the p₉₅ tail by **21%** compared to centroid, while healing latency remains identical across both variants. Both variants restore connectivity within ~1 second in every scenario.

## Authors

- Riccardo Fabbian (University of Padua)

### Supervisor

- Federico Corò (University of Padua)

## References

This work is inspired by research on virtual spring-damper formation control and the Crazyflie 2.1 drone platform with the DWM1000 UWB Loco Positioning System. See the technical report in `report/` for detailed methodology and analysis.

## License

University of Padua - Wireless Networks for Mobile Applications
