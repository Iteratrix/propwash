pub mod events;
pub mod fft;
pub mod summary;

use events::FlightEvent;
use fft::VibrationAnalysis;
use summary::FlightSummary;

use crate::types::{RawSession, Session};

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct FlightAnalysis {
    pub summary: FlightSummary,
    pub events: Vec<FlightEvent>,
    pub vibration: Option<VibrationAnalysis>,
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
    };
    let summary = summary::summarize(session, &detected);
    FlightAnalysis {
        summary,
        events: detected,
        vibration,
    }
}
