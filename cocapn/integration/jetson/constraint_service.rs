// constraint_service.rs — Rust constraint service for Jetson + OpenArm
//
// Runs on Jetson (aarch64 Linux) alongside openarm_can.
// Reads motor state via SocketCAN, checks constraints, publishes to PLATO.
// Can be compiled standalone or used as a library.

use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Joint constraint limits
#[derive(Debug, Clone)]
pub struct JointLimit {
    pub joint_index: usize,
    pub min_pos: f32,
    pub max_pos: f32,
    pub max_torque: f32,
    pub max_velocity: f32,
}

/// Safety severity levels
#[derive(Debug, Clone, PartialEq)]
pub enum Severity {
    Advisory,
    Soft,
    Hard,
    Critical,
}

/// Result of a constraint check
#[derive(Debug, Clone)]
pub struct ConstraintResult {
    pub satisfied: bool,
    pub constraint_name: String,
    pub severity: Severity,
    pub message: String,
    pub actual_value: f32,
    pub margin: f32,
}

/// Eisenstein workspace constraint
#[derive(Debug, Clone)]
pub struct EisensteinWorkspace {
    pub radius: i32,
    pub scale: f32,
    pub center_a: i32,
    pub center_b: i32,
}

impl EisensteinWorkspace {
    /// Eisenstein integer norm: a² - ab + b²
    fn eisenstein_norm(a: i32, b: i32) -> i32 {
        a * a - a * b + b * b
    }

    /// Check if (x, y) position is within the Eisenstein workspace disk
    pub fn check(&self, x: f32, y: f32) -> ConstraintResult {
        let inv_scale = 1.0 / self.scale;
        let inv_sqrt3 = 1.0 / 3.0_f32.sqrt();

        let a = (x * inv_scale - y * inv_scale * inv_sqrt3).round() as i32;
        let b = (2.0 * y * inv_scale * inv_sqrt3).round() as i32;

        let da = a - self.center_a;
        let db = b - self.center_b;
        let norm = Self::eisenstein_norm(da, db);
        let norm_sqrt = (norm as f32).sqrt();
        let margin = self.radius as f32 - norm_sqrt;

        ConstraintResult {
            satisfied: margin >= 0.0,
            constraint_name: "eisenstein_workspace".into(),
            severity: Severity::Critical,
            message: if margin >= 0.0 {
                format!("Eisenstein norm {:.2} within radius {}", norm_sqrt, self.radius)
            } else {
                format!("Eisenstein workspace violation: norm {:.2} > radius {}", norm_sqrt, self.radius)
            },
            actual_value: norm_sqrt,
            margin,
        }
    }
}

/// Full arm state
#[derive(Debug, Clone, Default)]
pub struct ArmState {
    pub positions: [f32; 7],
    pub velocities: [f32; 7],
    pub torques: [f32; 7],
    pub end_effector: (f32, f32, f32),
    pub last_update: Option<Instant>,
}

/// Safety envelope manager
pub struct SafetyEnvelope {
    pub constraints: Vec<JointLimit>,
    pub eisenstein: Option<EisensteinWorkspace>,
    pub emergency_stop: bool,
    pub total_commands: u64,
    pub blocked_commands: u64,
    pub clamped_commands: u64,
    violation_history: Vec<ConstraintResult>,
}

impl SafetyEnvelope {
    pub fn new() -> Self {
        let mut constraints = Vec::new();
        for i in 0..7 {
            constraints.push(JointLimit {
                joint_index: i,
                min_pos: -std::f32::consts::PI,
                max_pos: std::f32::consts::PI,
                max_torque: 5.0,
                max_velocity: 2.0,
            });
        }

        Self {
            constraints,
            eisenstein: Some(EisensteinWorkspace {
                radius: 10,
                scale: 0.1,
                center_a: 0,
                center_b: 0,
            }),
            emergency_stop: false,
            total_commands: 0,
            blocked_commands: 0,
            clamped_commands: 0,
            violation_history: Vec::new(),
        }
    }

    /// Check if a joint position command is safe
    pub fn check_command(&mut self, joint: usize, target: f32) -> ConstraintResult {
        if self.emergency_stop {
            return ConstraintResult {
                satisfied: false,
                constraint_name: "emergency_stop".into(),
                severity: Severity::Critical,
                message: "EMERGENCY STOP active".into(),
                actual_value: target,
                margin: 0.0,
            };
        }

        self.total_commands += 1;

        if joint >= self.constraints.len() {
            return ConstraintResult {
                satisfied: false,
                constraint_name: "invalid_joint".into(),
                severity: Severity::Hard,
                message: format!("Invalid joint index {}", joint),
                actual_value: target,
                margin: 0.0,
            };
        }

        let limit = &self.constraints[joint];
        let margin_min = target - limit.min_pos;
        let margin_max = limit.max_pos - target;

        if margin_min < 0.0 || margin_max < 0.0 {
            self.blocked_commands += 1;
            let result = ConstraintResult {
                satisfied: false,
                constraint_name: format!("joint_{}_limit", joint),
                severity: Severity::Hard,
                message: format!(
                    "Joint {}: {:.3} outside [{:.3}, {:.3}]",
                    joint, target, limit.min_pos, limit.max_pos
                ),
                actual_value: target,
                margin: margin_min.min(margin_max),
            };
            self.violation_history.push(result.clone());
            return result;
        }

        ConstraintResult {
            satisfied: true,
            constraint_name: format!("joint_{}_limit", joint),
            severity: Severity::Advisory,
            message: "Command accepted".into(),
            actual_value: target,
            margin: margin_min.min(margin_max),
        }
    }

    /// Safety score (0.0 = all violated, 1.0 = all satisfied)
    pub fn safety_score(&self) -> f32 {
        if self.total_commands == 0 {
            return 1.0;
        }
        1.0 - (self.blocked_commands as f32 / self.total_commands as f32)
    }
}

/// PLATO client for Jetson (HTTP-based, same protocol as ESP32)
pub struct PlatoClient {
    server: String,
    device_id: String,
    client: reqwest::blocking::Client,
}

impl PlatoClient {
    pub fn new(server: &str, device_id: &str) -> Self {
        Self {
            server: server.to_string(),
            device_id: device_id.to_string(),
            client: reqwest::blocking::Client::builder()
                .timeout(Duration::from_secs(5))
                .build()
                .unwrap_or_default(),
        }
    }

    pub fn publish(&self, domain: &str, question: &str, answer: &str) -> bool {
        let url = format!("http://{}/submit", self.server);
        let payload = serde_json::json!({
            "room": self.device_id,
            "domain": domain,
            "question": question,
            "answer": answer,
        });

        match self.client.post(&url).json(&payload).send() {
            Ok(resp) => resp.status().is_success(),
            Err(e) => {
                eprintln!("[PLATO] publish failed: {}", e);
                false
            }
        }
    }

    pub fn publish_telemetry(&self, state: &ArmState, envelope: &SafetyEnvelope) {
        let telemetry = serde_json::json!({
            "positions": state.positions,
            "velocities": state.velocities,
            "torques": state.torques,
            "safety": {
                "score": envelope.safety_score(),
                "estop": envelope.emergency_stop,
                "commands": envelope.total_commands,
                "blocked": envelope.blocked_commands,
            }
        });
        self.publish("sensors", "joints", &telemetry.to_string());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joint_limit_safe() {
        let mut env = SafetyEnvelope::new();
        let result = env.check_command(0, 1.0);
        assert!(result.satisfied);
    }

    #[test]
    fn test_joint_limit_violation() {
        let mut env = SafetyEnvelope::new();
        let result = env.check_command(0, 5.0);
        assert!(!result.satisfied);
        assert_eq!(env.blocked_commands, 1);
    }

    #[test]
    fn test_eisenstein_workspace() {
        let ws = EisensteinWorkspace { radius: 10, scale: 0.1, center_a: 0, center_b: 0 };
        let result = ws.check(0.5, 0.3);
        assert!(result.satisfied);
    }

    #[test]
    fn test_eisenstein_violation() {
        let ws = EisensteinWorkspace { radius: 2, scale: 0.1, center_a: 0, center_b: 0 };
        let result = ws.check(1.0, 1.0);
        assert!(!result.satisfied);
    }

    #[test]
    fn test_emergency_stop() {
        let mut env = SafetyEnvelope::new();
        env.emergency_stop = true;
        let result = env.check_command(0, 0.0);
        assert!(!result.satisfied);
    }
}
