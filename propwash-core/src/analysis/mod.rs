pub mod diagnostics;
pub mod events;
pub mod fft;
pub mod summary;
pub mod unified_events;

use diagnostics::Diagnostic;
use events::{EventKind, FlightEvent};
use fft::VibrationAnalysis;
use summary::FlightSummary;

use crate::format::ap::types::{ApRawSession, ApValue};
use crate::format::px4::types::Px4RawSession;
use crate::types::{RawSession, Session};

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct FlightAnalysis {
    pub summary: FlightSummary,
    pub events: Vec<FlightEvent>,
    pub vibration: Option<VibrationAnalysis>,
    pub diagnostics: Vec<Diagnostic>,
}

/// Analyzes a parsed session, detecting events and producing a summary.
pub fn analyze(session: &Session) -> FlightAnalysis {
    let (detected, vibration) = {
        let mut events = unified_events::detect_all(session);
        let vib = fft::analyze_vibration_unified(session);

        // Format-specific event sources
        match &session.raw {
            RawSession::Betaflight(_) => {}
            RawSession::ArduPilot(ap) => {
                events.extend(detect_ardupilot_events(ap));
                events.extend(detect_ardupilot_firmware_messages(ap));
            }
            RawSession::Px4(px4) => {
                events.extend(detect_px4_log_events(px4));
            }
        }

        (events, vib)
    };
    let summary = summary::summarize(session, &detected);
    let diags = diagnostics::diagnose(
        &detected,
        vibration.as_ref(),
        summary.motor_count,
        summary.duration_seconds,
    );
    FlightAnalysis {
        summary,
        events: detected,
        vibration,
        diagnostics: diags,
    }
}

/// Detect events from PX4 firmware log messages (warnings/errors).
#[allow(clippy::cast_precision_loss)]
fn detect_px4_log_events(px4: &Px4RawSession) -> Vec<FlightEvent> {
    let mut events = Vec::new();

    for msg in &px4.log_messages {
        // Skip debug messages (level 7), show everything else
        if msg.level >= 7 {
            continue;
        }
        let level_name = match msg.level {
            0 => "emergency",
            1 => "alert",
            2 => "critical",
            3 => "error",
            4 => "warning",
            5 => "notice",
            6 => "info",
            _ => "debug",
        };
        events.push(FlightEvent {
            frame_index: 0,
            #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
            time_us: msg.timestamp_us as i64,
            time_seconds: msg.timestamp_us as f64 / 1_000_000.0,
            kind: EventKind::FirmwareMessage {
                level: level_name.to_string(),
                message: msg.message.clone(),
            },
        });
    }

    events
}

/// Detect events from `ArduPilot` EV/ERR messages.
#[allow(clippy::cast_precision_loss)]
fn detect_ardupilot_events(ap: &ApRawSession) -> Vec<FlightEvent> {
    let mut events = Vec::new();

    for msg in ap.messages_by_name("EV") {
        let id = msg.values.get(1).map_or(0, ApValue::as_i64);
        let kind = match id {
            10 => EventKind::ThrottlePunch {
                from_percent: 0.0,
                to_percent: 100.0,
                duration_ms: 0.0,
            },
            11 => EventKind::ThrottleChop {
                from_percent: 100.0,
                to_percent: 0.0,
                duration_ms: 0.0,
            },
            _ => continue,
        };
        events.push(FlightEvent {
            frame_index: 0,
            time_us: msg.time_us.cast_signed(),
            time_seconds: msg.time_us as f64 / 1_000_000.0,
            kind,
        });
    }

    events
}

/// Detect firmware messages from `ArduPilot` ERR log messages.
#[allow(clippy::cast_precision_loss)]
fn detect_ardupilot_firmware_messages(ap: &ApRawSession) -> Vec<FlightEvent> {
    let mut events = Vec::new();

    for msg in ap.messages_by_name("ERR") {
        let subsys = msg.values.get(1).map_or(0, ApValue::as_i64);
        let code = msg.values.get(2).map_or(0, ApValue::as_i64);
        events.push(FlightEvent {
            frame_index: 0,
            time_us: msg.time_us.cast_signed(),
            time_seconds: msg.time_us as f64 / 1_000_000.0,
            kind: EventKind::FirmwareMessage {
                level: "error".to_string(),
                message: format!("Subsystem {subsys} error code {code}"),
            },
        });
    }

    events
}
