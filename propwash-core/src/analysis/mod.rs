pub mod diagnostics;
pub mod episodes;
pub mod events;
pub mod fft;
pub mod pid;
pub mod step_response;
pub mod summary;
pub mod trend;
pub mod unified_events;
pub(crate) mod util;

use diagnostics::Diagnostic;
use events::{EventKind as FlightEventKind, FlightEvent};
use fft::VibrationAnalysis;
use pid::PidAnalysis;
use step_response::StepResponseAnalysis;
use summary::FlightSummary;

use crate::session as sess;
use crate::types::Session;

use az::Az;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct FlightAnalysis {
    pub summary: FlightSummary,
    pub events: Vec<FlightEvent>,
    pub vibration: Option<VibrationAnalysis>,
    pub step_response: Option<StepResponseAnalysis>,
    pub pid: Option<PidAnalysis>,
    pub diagnostics: Vec<Diagnostic>,
}

/// Analyzes a parsed session, detecting events and producing a summary.
///
/// Returned `events` combines two sources:
/// 1. Behavior-detected events from [`unified_events::detect_all`]
///    (gyro spikes, throttle chops, motor saturation, overshoot, desync).
/// 2. Format-native events that landed on `session.events` during parse
///    (firmware messages, mode changes, armed transitions). These are
///    surfaced as [`FlightEventKind::FirmwareMessage`] with a synthetic
///    severity so a single timeline view can render both.
pub fn analyze(session: &Session) -> FlightAnalysis {
    let mut detected = unified_events::detect_all(session);
    detected.extend(fold_session_events(session));
    detected.sort_by(|a, b| a.time_seconds.total_cmp(&b.time_seconds));

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

/// Translate `Session.events` (format-native events emitted by parsers
/// during the build step) into [`FlightEvent`]s for the timeline.
///
/// All variants currently fold into [`FlightEventKind::FirmwareMessage`]
/// with a synthetic level string — this preserves the timeline without
/// expanding the analysis-layer `EventKind` enum. Frontends rendering
/// the timeline see one shape regardless of source.
fn fold_session_events(session: &Session) -> Vec<FlightEvent> {
    let first_t = session
        .gyro
        .time_us
        .first()
        .copied()
        .unwrap_or(0)
        .az::<f64>();
    session
        .events
        .iter()
        .map(|ev| {
            let (level, message) = describe_session_event(ev);
            let t = ev.time_us.az::<f64>();
            FlightEvent {
                frame_index: 0,
                time_us: t.az::<i64>(),
                time_seconds: (t - first_t) / 1_000_000.0,
                kind: FlightEventKind::FirmwareMessage { level, message },
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::{Event, EventKind, FlightMode, LogSeverity};

    #[test]
    fn session_events_fold_into_flight_analysis() {
        let mut s = Session::default();
        // Need at least 2 gyro samples for first_t to be meaningful.
        s.gyro.time_us = vec![1_000_000, 2_000_000];
        s.gyro.values.roll = vec![Default::default(); 2];
        s.gyro.values.pitch = vec![Default::default(); 2];
        s.gyro.values.yaw = vec![Default::default(); 2];
        s.events = vec![
            Event {
                time_us: 1_500_000,
                kind: EventKind::Armed,
                message: None,
            },
            Event {
                time_us: 1_750_000,
                kind: EventKind::ModeChange {
                    to: FlightMode::Stabilize,
                },
                message: None,
            },
            Event {
                time_us: 1_900_000,
                kind: EventKind::LogMessage {
                    severity: LogSeverity::Warning,
                },
                message: Some("low battery".into()),
            },
        ];

        let folded = fold_session_events(&s);
        assert_eq!(folded.len(), 3, "all 3 session events should fold");

        // Each should be a FirmwareMessage with a sensible level.
        let messages: Vec<&str> = folded
            .iter()
            .filter_map(|e| match &e.kind {
                FlightEventKind::FirmwareMessage { level, message } => Some(level.as_str())
                    .zip(Some(message.as_str()))
                    .map(|_| level.as_str()),
                _ => None,
            })
            .collect();
        assert_eq!(messages, vec!["info", "info", "warning"]);

        // Time projection: events should land at t-first_t in seconds.
        // first_t = 1_000_000 µs, so Armed at 1.5s = 0.5s.
        assert!((folded[0].time_seconds - 0.5).abs() < 1e-9);
    }
}

fn describe_session_event(ev: &sess::Event) -> (String, String) {
    use sess::{EventKind, FlightMode, LogSeverity};
    let mode_label = |m: &FlightMode| match m {
        FlightMode::Other(s) => s.clone(),
        other => format!("{other:?}"),
    };
    let level = match &ev.kind {
        EventKind::LogMessage { severity } => match severity {
            LogSeverity::Emergency => "emergency",
            LogSeverity::Alert => "alert",
            LogSeverity::Critical => "critical",
            LogSeverity::Error => "error",
            LogSeverity::Warning => "warning",
            LogSeverity::Notice => "notice",
            LogSeverity::Info => "info",
            LogSeverity::Debug => "debug",
        }
        .to_string(),
        EventKind::Crash | EventKind::Failsafe { .. } => "critical".into(),
        EventKind::GpsRescue { .. } => "warning".into(),
        // Armed/Disarmed/ModeChange/Custom are state changes, not severity-
        // bearing — surface as info-level so they show up but don't trip
        // any "looks bad" UI heuristic.
        _ => "info".into(),
    };
    let message = match &ev.kind {
        EventKind::Armed => "Armed".into(),
        EventKind::Disarmed => ev
            .message
            .clone()
            .map_or_else(|| "Disarmed".into(), |m| format!("Disarmed: {m}")),
        EventKind::ModeChange { to } => format!("Mode change → {}", mode_label(to)),
        EventKind::Crash => "Crash detected".into(),
        EventKind::Failsafe { reason } => format!("Failsafe: {reason}"),
        EventKind::GpsRescue { phase } => format!("GPS rescue ({phase})"),
        EventKind::LogMessage { .. } => ev.message.clone().unwrap_or_default(),
        EventKind::Custom(name) => ev
            .message
            .clone()
            .map_or_else(|| name.clone(), |m| format!("{name}: {m}")),
    };
    (level, message)
}
