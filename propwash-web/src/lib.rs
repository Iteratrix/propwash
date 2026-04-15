use std::cell::RefCell;
use std::collections::HashMap;

use serde::Serialize;
use wasm_bindgen::prelude::*;

use propwash_core::analysis::{self, fft, FlightAnalysis};
use propwash_core::types::{Log, SensorField};

thread_local! {
    static CURRENT_LOG: RefCell<Option<Log>> = const { RefCell::new(None) };
}

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
    is_truncated: bool,
    corrupt_bytes: usize,
    analysis: FlightAnalysis,
}

#[derive(Serialize)]
struct TimeseriesResult {
    time_s: Vec<f64>,
    fields: HashMap<String, Vec<f64>>,
    sample_rate_hz: f64,
    total_frames: usize,
    decimation: usize,
}

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn analyze(data: &[u8]) -> String {
    let log = match propwash_core::decode(data) {
        Ok(log) => log,
        Err(e) => {
            return serde_json::to_string(&AnalysisResult {
                sessions: Vec::new(),
                warnings: vec![e.to_string()],
            })
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#));
        }
    };

    let mut sessions = Vec::new();
    for session in &log.sessions {
        let flight_analysis = analysis::analyze(session);

        sessions.push(SessionResult {
            index: session.index(),
            firmware: session.firmware_version().to_string(),
            craft: session.craft_name().to_string(),
            duration_seconds: session.duration_seconds(),
            sample_rate_hz: session.sample_rate_hz(),
            frame_count: session.frame_count(),
            is_truncated: session.is_truncated(),
            corrupt_bytes: session.corrupt_bytes(),
            analysis: flight_analysis,
        });
    }

    let mut warnings = Vec::new();
    for w in &log.warnings {
        warnings.push(w.to_string());
    }

    let result = AnalysisResult { sessions, warnings };

    CURRENT_LOG.with(|cell| {
        *cell.borrow_mut() = Some(log);
    });

    serde_json::to_string(&result)
        .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
}

#[wasm_bindgen]
#[allow(clippy::cast_precision_loss)]
pub fn get_timeseries(session_idx: usize, max_points: usize, field_list: &str) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let total_frames = session.frame_count();
        let sample_rate = session.sample_rate_hz();

        let step = if total_frames > max_points {
            total_frames / max_points
        } else {
            1
        };

        let requested: Vec<&str> = field_list.split(',').collect();

        let mut fields: HashMap<String, Vec<f64>> = HashMap::new();

        for &name in &requested {
            let Ok(field) = SensorField::parse(name) else {
                continue;
            };
            let raw = session.field(&field);
            let decimated: Vec<f64> = raw.iter().step_by(step).copied().collect();
            fields.insert(name.to_string(), decimated);
        }

        let time_raw = session.field(&SensorField::Time);
        let t0 = time_raw.first().copied().unwrap_or(0.0);
        let time_s: Vec<f64> = time_raw
            .iter()
            .step_by(step)
            .map(|&v| (v - t0) / 1_000_000.0)
            .collect();

        let result = TimeseriesResult {
            time_s,
            fields,
            sample_rate_hz: sample_rate,
            total_frames,
            decimation: step,
        };

        serde_json::to_string(&result)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

#[wasm_bindgen]
pub fn get_spectrogram(session_idx: usize, axis_list: &str) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let axes: Vec<(&str, SensorField)> = axis_list
            .split(',')
            .filter_map(|a| {
                let field_name = match a {
                    "roll" => "gyro[roll]",
                    "pitch" => "gyro[pitch]",
                    "yaw" => "gyro[yaw]",
                    other => other,
                };
                SensorField::parse(field_name).ok().map(|f| (a, f))
            })
            .collect();

        let axis_refs: Vec<(&str, &SensorField)> =
            axes.iter().map(|(name, field)| (*name, field)).collect();

        let Some(spectrogram) = fft::compute_spectrogram(session, &axis_refs) else {
            return r#"{"error":"no spectrogram data"}"#.to_string();
        };

        serde_json::to_string(&spectrogram)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

#[wasm_bindgen]
pub fn get_filter_config(session_idx: usize) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        serde_json::to_string(&session.filter_config())
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

#[derive(Serialize)]
struct RawFramesResult {
    field_names: Vec<String>,
    frames: Vec<Vec<f64>>,
    start: usize,
    total: usize,
}

#[wasm_bindgen]
pub fn get_raw_frames(session_idx: usize, start: usize, count: usize, field_list: &str) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let total = session.frame_count();
        let requested: Vec<&str> = field_list.split(',').collect();

        let end = (start + count).min(total);
        let mut frames = Vec::with_capacity(end - start);

        // Pre-resolve field names to SensorFields and fetch all columns
        let resolved: Vec<SensorField> = requested
            .iter()
            .filter_map(|&name| SensorField::parse(name).ok())
            .collect();
        let columns: Vec<Vec<f64>> = resolved.iter().map(|f| session.field(f)).collect();

        for frame_idx in start..end {
            let mut row = Vec::with_capacity(columns.len());
            for col in &columns {
                row.push(col.get(frame_idx).copied().unwrap_or(0.0));
            }
            frames.push(row);
        }

        let result = RawFramesResult {
            field_names: requested.iter().map(|s| (*s).to_string()).collect(),
            frames,
            start,
            total,
        };

        serde_json::to_string(&result)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}
