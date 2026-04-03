pub mod diagnostics;
pub mod events;
pub mod fft;
pub mod summary;

use diagnostics::Diagnostic;
use events::{EventKind, FlightEvent};
use fft::VibrationAnalysis;
use summary::FlightSummary;

use crate::format::ap::types::ApRawSession;
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
            let sample_rate = session.unified().sample_rate_hz();
            let vib = if sample_rate > 0.0 {
                // Use Unified trait to extract gyro data for FFT
                let unified = session.unified();
                let mut spectra = Vec::new();
                let axis_names = ["roll", "pitch", "yaw"];
                for (i, axis) in crate::types::Axis::ALL.iter().enumerate() {
                    let gyro = unified.field(&crate::types::SensorField::Gyro(*axis));
                    if gyro.len() >= 1024 {
                        #[allow(clippy::cast_precision_loss)]
                        let samples: Vec<f64> = gyro.iter().map(|&v| v as f64).collect();
                        spectra.push(fft::compute_spectrum_from_samples(&samples, sample_rate, axis_names[i]));
                    }
                }
                if spectra.is_empty() {
                    None
                } else {
                    let noise_floor_db = std::array::from_fn(|i| {
                        spectra.get(i).map_or(0.0, |s| {
                            let sum: f64 = s.magnitudes_db.iter().sum();
                            #[allow(clippy::cast_precision_loss)]
                            { sum / s.magnitudes_db.len() as f64 }
                        })
                    });
                    Some(fft::VibrationAnalysis {
                        spectra,
                        noise_floor_db,
                        throttle_bands: Vec::new(),
                        accel: None,
                    })
                }
            } else {
                None
            };
            (events, vib)
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

/// Detect events from ArduPilot EV/ERR messages.
#[allow(clippy::cast_precision_loss)]
fn detect_ardupilot_events(ap: &ApRawSession) -> Vec<FlightEvent> {
    let mut events = Vec::new();

    // EV messages: arm/disarm, land, etc.
    for msg in ap.messages_by_name("EV") {
        let id = msg.values.get(1).map_or(0, |v| v.as_i64());
        let kind = match id {
            10 => EventKind::ThrottlePunch { from_percent: 0.0, to_percent: 100.0, duration_ms: 0.0 },
            11 => EventKind::ThrottleChop { from_percent: 100.0, to_percent: 0.0, duration_ms: 0.0 },
            _ => continue,
        };
        events.push(FlightEvent {
            frame_index: 0,
            time_us: msg.time_us as i64,
            time_seconds: msg.time_us as f64 / 1_000_000.0,
            kind,
        });
    }

    events
}
