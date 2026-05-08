# ESP32 CAN + PLATO Integration

## Hardware Setup

- **ESP32-S3** (or any ESP32 with TWAI/CAN support)
- **CAN transceiver**: SN65HVD230 or TJA1050
- **Wiring**:
  - GPIO21 → CAN TX
  - GPIO22 → CAN RX
  - CAN H/H → OpenArm CAN bus
  - CAN L/L → OpenArm CAN bus

## Build

```bash
cd cocapn/integration/esp32
idf.py set-target esp32s3
idf.py menuconfig
# Set under "OpenArm PLATO Node":
#   - WiFi SSID/password
#   - PLATO server IP:port
#   - Device ID
idf.py build flash monitor
```

## What It Does

The ESP32 sits between your control software and the OpenArm CAN bus:

1. **10 Hz**: Publishes joint telemetry to PLATO (positions, velocities, torques)
2. **5 Hz**: Polls PLATO for fleet commands
3. **100 Hz**: Runs constraint checks on all motor commands
4. **On violation**: Rejects command, logs to PLATO, triggers e-stop on critical violations

## CAN Protocol

Uses Damiao Motor protocol (same as openarm_can):
- MIT control mode: `kp, kd, q, dq, tau`
- Standard CAN IDs: 0x01-0x07 (send), 0x11-0x17 (receive)
- CAN-FD supported

## MCP Tools

The ESP32 registers these tools via PLATO MCP:

| Tool | Description |
|------|-------------|
| `get_joint_states` | Current positions/velocities/torques |
| `set_joint_position` | Set joint target (constraint-checked) |
| `emergency_stop` | Trigger emergency stop |
| `get_safety_state` | Safety envelope state + violation history |
| `set_constraint` | Dynamically update a safety constraint |
