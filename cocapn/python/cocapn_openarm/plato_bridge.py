"""
PLATO Bridge — connects OpenArm to the fleet knowledge system.

Uses Oracle1's bare-metal PLATO protocol (HTTP-based)
to publish telemetry, poll commands, and register the arm
as a fleet device.
"""

import json
import urllib.request
import urllib.error
from typing import Optional, Dict, Any


class PlatoBridge:
    """
    Bridge between OpenArm and PLATO knowledge server.

    Provides:
    - Device registration (ensign tile)
    - Telemetry publishing (sensor readings, safety state)
    - Command polling (fleet agents can control the arm)
    - Constraint state sharing (fleet-wide safety visibility)
    """

    def __init__(self, server: str, device_id: str, port: int = 8847):
        self.server = server.rstrip("/")
        self.port = port
        self.device_id = device_id
        self.base_url = f"http://{self.server}:{self.port}"
        self._registered = False

    def publish(self, domain: str, question: str, answer: Any) -> bool:
        """POST a tile to the PLATO server."""
        payload = json.dumps({
            "room": self.device_id,
            "domain": domain,
            "question": question,
            "answer": answer if isinstance(answer, str) else json.dumps(answer),
        }).encode()

        req = urllib.request.Request(
            f"{self.base_url}/submit",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                return resp.status == 200
        except (urllib.error.URLError, OSError) as e:
            print(f"[PLATO] publish failed: {e}")
            return False

    def fetch(self, room_name: str) -> Optional[dict]:
        """GET tiles from a PLATO room."""
        try:
            url = f"{self.base_url}/room/{room_name}"
            with urllib.request.urlopen(url, timeout=5.0) as resp:
                return json.loads(resp.read().decode())
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as e:
            print(f"[PLATO] fetch failed: {e}")
            return None

    def poll_commands(self) -> list:
        """Poll for pending commands from fleet agents."""
        result = self.fetch(f"{self.device_id}/commands")
        if result and isinstance(result, dict):
            commands = result.get("commands", [])
            return commands if isinstance(commands, list) else []
        return []

    def register_device(self, hardware: str = "openarm", tools: list = None) -> bool:
        """Register this arm as a PLATO fleet device (ensign tile)."""
        ensign = {
            "type": "robotic_arm",
            "hardware": hardware,
            "device_id": self.device_id,
            "capability_level": 1,  # Starts at "conditioned" with constraint safety
            "capability_name": "constraint-aware",
            "protocol": "cocapn-openarm-v1",
            "tools": tools or [
                {"name": "get_state", "description": "Get current joint states"},
                {"name": "set_position", "description": "Set joint positions with constraint checking"},
                {"name": "get_safety", "description": "Get safety envelope state"},
                {"name": "emergency_stop", "description": "Trigger emergency stop"},
            ],
            "fleet": "cocapn",
            "constraints_enabled": True,
        }

        success = self.publish("ensign", "presence", ensign)
        if success:
            self._registered = True
        return success

    def publish_telemetry(self, joint_states: dict, safety_telemetry: dict) -> bool:
        """Publish current arm state + safety telemetry."""
        ok1 = self.publish("sensors", "joints", joint_states)
        ok2 = self.publish("safety", "envelope", safety_telemetry)
        return ok1 and ok2

    def publish_constraint_violation(self, violation: dict) -> bool:
        """Publish a constraint violation event."""
        return self.publish("safety", "violation", violation)

    def publish_insight(self, insight: dict) -> bool:
        """Publish a discovered insight from the insight engine."""
        return self.publish("insights", "discovery", insight)

    @property
    def is_connected(self) -> bool:
        return self._registered
