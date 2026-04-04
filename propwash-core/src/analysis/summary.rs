use serde::Serialize;

use super::events::{EventKind, FlightEvent};
use crate::types::Session;

#[derive(Debug, Serialize)]
pub struct FlightSummary {
    pub session_index: usize,
    pub firmware: String,
    pub craft: String,
    pub duration_seconds: f64,
    pub sample_rate_hz: f64,
    pub frame_count: usize,
    pub motor_count: usize,
    pub throttle_chops: usize,
    pub throttle_punches: usize,
    pub motor_saturations: usize,
    pub gyro_spikes: usize,
    pub overshoots: usize,
    pub desyncs: usize,
    pub total_events: usize,
}

/// Builds a summary from a session and its detected events.
pub fn summarize(session: &Session, events: &[FlightEvent]) -> FlightSummary {
    let unified = session.unified();

    let mut throttle_chops = 0;
    let mut throttle_punches = 0;
    let mut motor_saturations = 0;
    let mut gyro_spikes = 0;
    let mut overshoots = 0;
    let mut desyncs = 0;

    for event in events {
        match &event.kind {
            EventKind::ThrottleChop { .. } => throttle_chops += 1,
            EventKind::ThrottlePunch { .. } => throttle_punches += 1,
            EventKind::MotorSaturation { .. } => motor_saturations += 1,
            EventKind::GyroSpike { .. } => gyro_spikes += 1,
            EventKind::Overshoot { .. } => overshoots += 1,
            EventKind::Desync { .. } => desyncs += 1,
            EventKind::FirmwareMessage { .. } => {}
        }
    }

    FlightSummary {
        session_index: session.index,
        firmware: unified.firmware_version().to_string(),
        craft: unified.craft_name().to_string(),
        duration_seconds: unified.duration_seconds(),
        sample_rate_hz: unified.sample_rate_hz(),
        frame_count: unified.frame_count(),
        motor_count: unified.motor_count(),
        throttle_chops,
        throttle_punches,
        motor_saturations,
        gyro_spikes,
        overshoots,
        desyncs,
        total_events: events.len(),
    }
}
