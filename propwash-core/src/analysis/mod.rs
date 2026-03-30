pub mod events;
pub mod summary;

use events::FlightEvent;
use summary::FlightSummary;

use crate::types::{RawSession, Session};

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct FlightAnalysis {
    pub summary: FlightSummary,
    pub events: Vec<FlightEvent>,
}

/// Analyzes a parsed session, detecting events and producing a summary.
pub fn analyze(session: &Session) -> FlightAnalysis {
    let detected = match &session.raw {
        RawSession::Betaflight(bf) => events::detect_events(bf),
    };
    let summary = summary::summarize(session, &detected);
    FlightAnalysis {
        summary,
        events: detected,
    }
}
