use std::cell::RefCell;
use std::collections::HashMap;

use serde::Serialize;
use wasm_bindgen::prelude::*;

use propwash_core::analysis::{self, fft, trend, FlightAnalysis};
use propwash_core::types::{Log, SensorField};

// ---------------------------------------------------------------------------
// Workspace state — supports multiple loaded files
// ---------------------------------------------------------------------------

struct WorkspaceEntry {
    id: u32,
    filename: String,
    log: Log,
    analyses: Vec<FlightAnalysis>,
}

#[derive(Default)]
struct Workspace {
    entries: Vec<WorkspaceEntry>,
    next_id: u32,
}

impl Workspace {
    fn find(&self, file_id: u32) -> Option<&WorkspaceEntry> {
        self.entries.iter().find(|e| e.id == file_id)
    }
}

thread_local! {
    static WORKSPACE: RefCell<Workspace> = RefCell::new(Workspace::default());
}

/// Look up a session from the workspace, call `f` with it, return its result.
/// Returns a JSON error string if not found.
fn with_session<F>(file_id: u32, session_idx: usize, f: F) -> String
where
    F: FnOnce(&propwash_core::types::Session) -> String,
{
    WORKSPACE.with(|cell| {
        let borrow = cell.borrow();
        let Some(entry) = borrow.find(file_id) else {
            return format!(r#"{{"error":"unknown file_id {file_id}"}}"#);
        };
        let Some(session) = entry.log.sessions.get(session_idx) else {
            return format!(r#"{{"error":"invalid session_idx {session_idx}"}}"#);
        };
        f(session)
    })
}

// ---------------------------------------------------------------------------
// JSON response types
// ---------------------------------------------------------------------------

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
struct AddFileResult {
    file_id: u32,
    filename: String,
    sessions: Vec<SessionResult>,
    warnings: Vec<String>,
}

#[derive(Serialize)]
struct TimeseriesResult {
    time_s: Vec<f64>,
    fields: HashMap<String, Vec<f64>>,
    sample_rate_hz: f64,
    total_frames: usize,
    decimation: usize,
}

#[derive(Serialize)]
struct RawFramesResult {
    field_names: Vec<String>,
    frames: Vec<Vec<f64>>,
    start: usize,
    total: usize,
}

// ---------------------------------------------------------------------------
// WASM exports
// ---------------------------------------------------------------------------

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// Add a file to the workspace. Returns JSON with `file_id`, sessions, and warnings.
#[wasm_bindgen]
pub fn add_file(data: &[u8], filename: &str) -> String {
    let log = match propwash_core::decode(data) {
        Ok(log) => log,
        Err(e) => {
            return serde_json::to_string(&AddFileResult {
                file_id: u32::MAX,
                filename: filename.to_string(),
                sessions: Vec::new(),
                warnings: vec![e.to_string()],
            })
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#));
        }
    };

    let mut cached_analyses = Vec::new();
    let session_results: Vec<SessionResult> = log
        .sessions
        .iter()
        .map(|session| {
            let flight_analysis = analysis::analyze(session);
            cached_analyses.push(flight_analysis.clone());
            SessionResult {
                index: session.index(),
                firmware: session.firmware_version().to_string(),
                craft: session.craft_name().to_string(),
                duration_seconds: session.duration_seconds(),
                sample_rate_hz: session.sample_rate_hz(),
                frame_count: session.frame_count(),
                is_truncated: session.is_truncated(),
                corrupt_bytes: session.corrupt_bytes(),
                analysis: flight_analysis,
            }
        })
        .collect();

    let warnings: Vec<String> = log.warnings.iter().map(ToString::to_string).collect();

    WORKSPACE.with(|cell| {
        let mut ws = cell.borrow_mut();
        let file_id = ws.next_id;
        ws.next_id += 1;

        let result = AddFileResult {
            file_id,
            filename: filename.to_string(),
            sessions: session_results,
            warnings,
        };

        ws.entries.push(WorkspaceEntry {
            id: file_id,
            filename: filename.to_string(),
            log,
            analyses: cached_analyses,
        });

        serde_json::to_string(&result)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

/// Remove a file from the workspace by `file_id`.
#[wasm_bindgen]
pub fn remove_file(file_id: u32) {
    WORKSPACE.with(|cell| {
        cell.borrow_mut().entries.retain(|e| e.id != file_id);
    });
}

/// Clear all files from the workspace.
#[wasm_bindgen]
pub fn clear_workspace() {
    WORKSPACE.with(|cell| {
        let mut ws = cell.borrow_mut();
        ws.entries.clear();
        ws.next_id = 0;
    });
}

/// Backward-compatible analyze: clears workspace, adds the file, returns old-style result.
#[wasm_bindgen]
pub fn analyze(data: &[u8]) -> String {
    clear_workspace();
    add_file(data, "")
}

/// Get timeseries data for a session.
#[wasm_bindgen]
#[allow(clippy::cast_precision_loss)]
pub fn get_timeseries(
    file_id: u32,
    session_idx: usize,
    max_points: usize,
    field_list: &str,
) -> String {
    with_session(file_id, session_idx, |session| {
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

/// Get spectrogram data for a session.
#[wasm_bindgen]
pub fn get_spectrogram(file_id: u32, session_idx: usize, axis_list: &str) -> String {
    with_session(file_id, session_idx, |session| {
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

/// Get filter configuration for a session.
#[wasm_bindgen]
pub fn get_filter_config(file_id: u32, session_idx: usize) -> String {
    with_session(file_id, session_idx, |session| {
        serde_json::to_string(&session.filter_config())
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

/// Get raw frame data for a session.
#[wasm_bindgen]
pub fn get_raw_frames(
    file_id: u32,
    session_idx: usize,
    start: usize,
    count: usize,
    field_list: &str,
) -> String {
    with_session(file_id, session_idx, |session| {
        let total = session.frame_count();
        let requested: Vec<&str> = field_list.split(',').collect();

        let end = (start + count).min(total);

        let resolved: Vec<SensorField> = requested
            .iter()
            .filter_map(|&name| SensorField::parse(name).ok())
            .collect();
        let columns: Vec<Vec<f64>> = resolved.iter().map(|f| session.field(f)).collect();

        let frames: Vec<Vec<f64>> = (start..end)
            .map(|frame_idx| {
                columns
                    .iter()
                    .map(|col| col.get(frame_idx).copied().unwrap_or(0.0))
                    .collect()
            })
            .collect();

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

/// Get step response overlay data for a session.
#[wasm_bindgen]
pub fn get_step_overlay(file_id: u32, session_idx: usize) -> String {
    use propwash_core::analysis::step_response;

    with_session(
        file_id,
        session_idx,
        |session| match step_response::extract_step_overlay(session) {
            Some(overlay) => serde_json::to_string(&overlay)
                .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#)),
            None => r#"{"error":"no step data"}"#.to_string(),
        },
    )
}

/// Compute trend across all sessions in the workspace.
#[wasm_bindgen]
pub fn get_trend() -> String {
    WORKSPACE.with(|cell| {
        let ws = cell.borrow();

        let mut refs: Vec<(String, &FlightAnalysis)> = Vec::new();
        for entry in &ws.entries {
            for (i, analysis) in entry.analyses.iter().enumerate() {
                let label = if entry.analyses.len() == 1 {
                    entry.filename.clone()
                } else {
                    format!(
                        "{} / s{}",
                        entry.filename,
                        entry
                            .log
                            .sessions
                            .get(i)
                            .map_or(i, propwash_core::Session::index)
                    )
                };
                refs.push((label, analysis));
            }
        }
        let points = trend::compute_trend(&refs);

        serde_json::to_string(&points)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}
