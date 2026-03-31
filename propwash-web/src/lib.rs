use serde::Serialize;
use wasm_bindgen::prelude::*;

use propwash_core::analysis::{self, FlightAnalysis};

#[derive(Serialize)]
struct AnalysisResult {
    sessions: Vec<SessionResult>,
    warnings: Vec<String>,
}

#[derive(Serialize)]
struct SessionResult {
    index: usize,
    firmware: String,
    craft: String,
    duration_seconds: f64,
    sample_rate_hz: f64,
    frame_count: usize,
    analysis: FlightAnalysis,
}

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn analyze(data: &[u8]) -> String {
    let log = propwash_core::decode(data);

    let mut sessions = Vec::new();
    for session in &log.sessions {
        let unified = session.unified();
        let flight_analysis = analysis::analyze(session);

        sessions.push(SessionResult {
            index: session.index,
            firmware: unified.firmware_version().to_string(),
            craft: unified.craft_name().to_string(),
            duration_seconds: unified.duration_seconds(),
            sample_rate_hz: unified.sample_rate_hz(),
            frame_count: unified.frame_count(),
            analysis: flight_analysis,
        });
    }

    let mut warnings = Vec::new();
    for w in &log.warnings {
        warnings.push(w.to_string());
    }

    let result = AnalysisResult {
        sessions,
        warnings,
    };

    serde_json::to_string(&result).unwrap_or_else(|e| {
        format!(r#"{{"error":"serialization failed: {e}"}}"#)
    })
}
