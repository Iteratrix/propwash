pub mod diagnostics;
pub mod events;
pub mod fft;
pub mod summary;

use diagnostics::Diagnostic;
use events::{EventKind, FlightEvent};
use fft::VibrationAnalysis;
use summary::FlightSummary;

use crate::format::ap::types::{ApRawSession, ApValue};
use crate::types::{RawSession, Session, Unified};

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
    let (detected, vibration) = match &session.raw {
        RawSession::Betaflight(bf) => {
            let events = events::detect_events(bf);
            let sample_rate = session.unified().sample_rate_hz();
            let vib = if sample_rate > 0.0 {
                Some(fft::analyze_vibration(bf, sample_rate))
            } else {
                None
            };
            (events, vib)
        }
        RawSession::ArduPilot(ap) => {
            let events = detect_ardupilot_events(ap);
            let vib = analyze_vibration_via_unified(session.unified());
            (events, vib)
        }
        RawSession::Px4(_) => {
            let vib = analyze_vibration_via_unified(session.unified());
            (Vec::new(), vib)
        }
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

/// Run FFT vibration analysis using the `Unified` trait (format-agnostic).
fn analyze_vibration_via_unified(unified: &dyn Unified) -> Option<VibrationAnalysis> {
    let sample_rate = unified.sample_rate_hz();
    if sample_rate <= 0.0 {
        return None;
    }

    let axis_names = ["roll", "pitch", "yaw"];
    let mut spectra = Vec::new();

    for (i, axis) in crate::types::Axis::ALL.iter().enumerate() {
        let gyro = unified.field(&crate::types::SensorField::Gyro(*axis));
        if gyro.len() >= 1024 {
            #[allow(clippy::cast_precision_loss)]
            let samples: Vec<f64> = gyro.iter().map(|&v| v as f64).collect();
            spectra.push(fft::compute_spectrum_from_samples(
                &samples,
                sample_rate,
                axis_names[i],
            ));
        }
    }

    if spectra.is_empty() {
        return None;
    }

    let noise_floor_db = std::array::from_fn(|i| {
        spectra.get(i).map_or(0.0, |s| {
            let sum: f64 = s.magnitudes_db.iter().sum();
            #[allow(clippy::cast_precision_loss)]
            {
                sum / s.magnitudes_db.len() as f64
            }
        })
    });

    Some(VibrationAnalysis {
        spectra,
        noise_floor_db,
        throttle_bands: Vec::new(),
        accel: None,
    })
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
