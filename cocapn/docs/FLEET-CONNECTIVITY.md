# Fleet Connectivity — Multi-Arm Coordination via PLATO

## What Is Fleet Connectivity?

Every OpenArm running Cocapn publishes telemetry to a PLATO knowledge server. Fleet agents can:
- **Monitor** all arms in real-time
- **Send commands** to any arm (constraint-checked)
- **Coordinate** multiple arms (collision avoidance, shared workspace)
- **Upgrade** arm intelligence over the network (embodiment protocol)

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  OpenArm 1  │     │  OpenArm 2  │     │  OpenArm N  │
│  (ESP32)    │     │  (Jetson)   │     │  (Linux)    │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
                    PLATO Server
                  (8847/tcp, HTTP)
                           │
              ┌────────────┼────────────┐
              │            │            │
         Fleet Agent   Fleet Agent   Dashboard
         (Oracle1)     (Forgemaster)  (web UI)
```

## Quick Setup

```python
from cocapn_openarm import ConstraintArm, JointLimit

arm = ConstraintArm(
    raw_arm,
    device_id="openarm-lab-01",
    plato_server="192.168.1.100:8847",  # your PLATO server
)

# Arm automatically registers as a fleet device
# Telemetry publishes at 10 Hz
# Commands polled at 5 Hz
```

## Multi-Arm Collision Avoidance

```python
# On a fleet agent, coordinate two arms:
arm1_state = plato.fetch("openarm-lab-01")
arm2_state = plato.fetch("openarm-lab-02")

# Check if proposed position for arm1 conflicts with arm2
# (constraint-checked on BOTH arms)
arm1.set_position(3, target)
```

## Embodiment Protocol

Agents can upgrade an arm's intelligence level:

| Level | Name | Behavior |
|-------|------|----------|
| 0 | Raw | Direct motor commands only |
| 1 | Conditioned | Threshold filtering, delta publishing |
| 2 | Smart | Context-aware, combines sensors |
| 3 | Autonomous | Self-directed goals, proactive alerts |
| 4 | Ensign | Fleet coordination, scouts for other devices |

## Security

- PLATO is HTTP on your local network
- No data leaves your LAN
- Commands are constraint-checked on the arm, not the agent
- Emergency stop is always available (hardware button + fleet command + automatic)
