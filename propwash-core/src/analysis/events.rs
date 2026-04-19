use serde::Serialize;

use crate::types::Axis;

#[derive(Debug, Clone, Serialize)]
pub struct FlightEvent {
    pub frame_index: usize,
    pub time_us: i64,
    pub time_seconds: f64,
    pub kind: EventKind,
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type")]
pub enum EventKind {
    ThrottleChop {
        from_percent: f64,
        to_percent: f64,
        duration_ms: f64,
    },
    ThrottlePunch {
        from_percent: f64,
        to_percent: f64,
        duration_ms: f64,
    },
    MotorSaturation {
        motor_index: usize,
        duration_frames: usize,
    },
    GyroSpike {
        axis: Axis,
        magnitude: f64,
    },
    Overshoot {
        axis: Axis,
        setpoint: f64,
        actual: f64,
        overshoot_percent: f64,
    },
    Desync {
        motor_index: usize,
        motor_value: i64,
        average_others: f64,
    },
    FirmwareMessage {
        level: String,
        message: String,
    },
}
