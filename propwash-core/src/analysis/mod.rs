pub mod diagnostics;
pub mod episodes;
pub mod events;
pub mod fft;
pub mod pid;
pub mod step_response;
pub mod summary;
pub mod trend;
pub mod unified_events;

use diagnostics::Diagnostic;
use events::{EventKind, FlightEvent};
use fft::VibrationAnalysis;
use pid::PidAnalysis;
use step_response::StepResponseAnalysis;
use summary::FlightSummary;

use crate::format::ap::types::ApSession;
use crate::format::mavlink::types::MavlinkSession;
use crate::format::px4::types::Px4Session;
use crate::types::Session;

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct FlightAnalysis {
    pub summary: FlightSummary,
    pub events: Vec<FlightEvent>,
    pub vibration: Option<VibrationAnalysis>,
    pub step_response: Option<StepResponseAnalysis>,
    pub pid: Option<PidAnalysis>,
    pub diagnostics: Vec<Diagnostic>,
}

/// Analyzes a parsed session, detecting events and producing a summary.
pub fn analyze(session: &Session) -> FlightAnalysis {
    // Detect events first — propwash analysis needs throttle chop timestamps
    let mut detected = unified_events::detect_all(session);
    match session {
        Session::Betaflight(_) => {}
        Session::ArduPilot(ap) => {
            detected.extend(detect_ardupilot_events(ap));
            detected.extend(detect_ardupilot_firmware_messages(ap));
        }
        Session::Px4(px4) => {
            detected.extend(detect_px4_log_events(px4));
        }
        Session::Mavlink(mav) => {
            detected.extend(detect_mavlink_status_messages(mav));
        }
    }

    let vibration = fft::analyze_vibration_unified(session, &detected);
    let summary = summary::summarize(session, &detected);
    let filter_config = session.filter_config();
    let step_resp = step_response::analyze_step_response(session);
    let pid_gains = session.pid_gains();
    let pid_analysis = pid::analyze_pid(session, step_resp.as_ref(), &pid_gains);
    let diags = diagnostics::diagnose(
        &detected,
        vibration.as_ref(),
        step_resp.as_ref(),
        pid_analysis.as_ref(),
        &filter_config,
        summary.motor_count,
        summary.duration_seconds,
    );

    FlightAnalysis {
        summary,
        events: detected,
        vibration,
        step_response: step_resp,
        pid: pid_analysis,
        diagnostics: diags,
    }
}

/// Detect events from PX4 firmware log messages (warnings/errors).
#[allow(clippy::cast_precision_loss)]
fn detect_px4_log_events(px4: &Px4Session) -> Vec<FlightEvent> {
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
fn detect_ardupilot_events(ap: &ApSession) -> Vec<FlightEvent> {
    let mut events = Vec::new();

    let ev_ts = ap.msg_timestamps("EV");
    if let Some(id_col) = ap.msg_column("EV", "Id") {
        for (i, &id) in id_col.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let kind = match id as i64 {
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
            let t = ev_ts.get(i).copied().unwrap_or(0);
            events.push(FlightEvent {
                frame_index: 0,
                time_us: t.cast_signed(),
                time_seconds: t as f64 / 1_000_000.0,
                kind,
            });
        }
    }

    events
}

/// Detect events from `MAVLink` `STATUSTEXT` messages (warnings/errors).
#[allow(clippy::cast_precision_loss)]
fn detect_mavlink_status_messages(mav: &MavlinkSession) -> Vec<FlightEvent> {
    use crate::format::mavlink::types::Severity;

    mav.status_messages
        .iter()
        .filter(|msg| msg.severity <= Severity::Warning)
        .map(|msg| FlightEvent {
            frame_index: 0,
            #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
            time_us: msg.timestamp_us as i64,
            time_seconds: msg.timestamp_us as f64 / 1_000_000.0,
            kind: EventKind::FirmwareMessage {
                level: msg.severity.as_str().to_string(),
                message: msg.text.clone(),
            },
        })
        .collect()
}

/// Detect firmware messages from `ArduPilot` ERR log messages.
#[allow(clippy::cast_precision_loss)]
fn detect_ardupilot_firmware_messages(ap: &ApSession) -> Vec<FlightEvent> {
    let mut events = Vec::new();

    let err_ts = ap.msg_timestamps("ERR");
    let subsys_col = ap.msg_column("ERR", "Subsys");
    let ecode_col = ap.msg_column("ERR", "ECode");
    if let (Some(subsys), Some(codes)) = (subsys_col, ecode_col) {
        #[allow(clippy::cast_possible_truncation)]
        for (i, (&s, &c)) in subsys.iter().zip(codes.iter()).enumerate() {
            let t = err_ts.get(i).copied().unwrap_or(0);
            events.push(FlightEvent {
                frame_index: 0,
                time_us: t.cast_signed(),
                time_seconds: t as f64 / 1_000_000.0,
                kind: EventKind::FirmwareMessage {
                    level: "error".to_string(),
                    message: format!("Subsystem {} error code {}", s as i64, c as i64),
                },
            });
        }
    }

    events
}
