use std::path::Path;
use std::str::FromStr;

use propwash_core::types::SensorField;
use propwash_web::SensorFields;

fn fixtures_dir() -> &'static Path {
    Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../propwash-core/tests/fixtures"
    ))
}

fn read_fixture(rel_path: &str) -> Vec<u8> {
    let path = fixtures_dir().join(rel_path);
    std::fs::read(&path).unwrap_or_else(|e| panic!("Failed to read {rel_path}: {e}"))
}

/// Parse a comma-separated list of canonical field names (e.g.
/// `"gyro[roll],motor[0]"`) into the typed `SensorFields` payload the
/// bridge functions now expect.
fn fields(s: &str) -> SensorFields {
    SensorFields(
        s.split(',')
            .map(|n| SensorField::from_str(n).unwrap())
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Workspace lifecycle
// ---------------------------------------------------------------------------

#[test]
fn add_file_returns_sessions() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let json = propwash_web::add_file(&data, "btfl_001.bbl");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    assert_eq!(result["filename"], "btfl_001.bbl");
    assert!(result["file_id"].as_u64().is_some());

    let sessions = result["sessions"].as_array().unwrap();
    assert!(!sessions.is_empty(), "expected at least one session");

    let s0 = &sessions[0];
    assert!(s0["frame_count"].as_u64().unwrap() > 0);
    assert!(s0["duration_seconds"].as_f64().unwrap() > 0.0);
    assert!(s0["sample_rate_hz"].as_f64().unwrap() > 0.0);
}

#[test]
fn add_file_invalid_data_returns_warning() {
    propwash_web::clear_workspace();
    let data = b"this is not a flight log";
    let json = propwash_web::add_file(data, "garbage.bbl");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    let sessions = result["sessions"].as_array().unwrap();
    assert!(
        sessions.is_empty(),
        "invalid data should produce no sessions"
    );

    let warnings = result["warnings"].as_array().unwrap();
    assert!(!warnings.is_empty(), "expected a warning for invalid data");
}

#[test]
fn clear_workspace_resets_state() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    propwash_web::add_file(&data, "btfl_001.bbl");

    propwash_web::clear_workspace();

    // Trend on empty workspace should return empty array
    let trend_json = propwash_web::get_trend();
    let trend: serde_json::Value = serde_json::from_str(&trend_json).unwrap();
    assert_eq!(trend.as_array().unwrap().len(), 0);
}

#[test]
fn remove_file_drops_entry() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let json = propwash_web::add_file(&data, "btfl_001.bbl");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();
    let file_id = result["file_id"].as_u64().unwrap() as u32;

    propwash_web::remove_file(file_id);

    // Accessing removed file should return error
    let ts = propwash_web::get_timeseries(file_id, 0, 100, fields("gyro[roll]"));
    let ts_result: serde_json::Value = serde_json::from_str(&ts).unwrap();
    assert!(
        ts_result["error"].is_string(),
        "expected error for removed file"
    );
}

#[test]
fn multiple_files_in_workspace() {
    propwash_web::clear_workspace();
    let data1 = read_fixture("fc-blackbox/btfl_001.bbl");
    let data2 = read_fixture("fc-blackbox/btfl_002.bbl");

    let r1: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data1, "btfl_001.bbl")).unwrap();
    let r2: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data2, "btfl_002.bbl")).unwrap();

    let id1 = r1["file_id"].as_u64().unwrap();
    let id2 = r2["file_id"].as_u64().unwrap();
    assert_ne!(id1, id2, "file IDs should be unique");

    // Both should be accessible
    let ts1 = propwash_web::get_timeseries(id1 as u32, 0, 100, fields("gyro[roll]"));
    let ts2 = propwash_web::get_timeseries(id2 as u32, 0, 100, fields("gyro[roll]"));
    assert!(!ts1.contains("error"));
    assert!(!ts2.contains("error"));
}

// ---------------------------------------------------------------------------
// Backward-compatible analyze()
// ---------------------------------------------------------------------------

#[test]
fn analyze_backward_compat() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let json = propwash_web::analyze(&data);
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    // analyze() wraps add_file(), so same shape
    assert!(result["sessions"].as_array().unwrap().len() > 0);
    assert!(result["file_id"].as_u64().is_some());
}

// ---------------------------------------------------------------------------
// Timeseries
// ---------------------------------------------------------------------------

#[test]
fn get_timeseries_returns_data() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json =
        propwash_web::get_timeseries(file_id, 0, 1000, fields("gyro[roll],gyro[pitch],motor[0]"));
    let ts: serde_json::Value = serde_json::from_str(&json).unwrap();

    assert!(ts["time_s"].as_array().unwrap().len() > 0);
    assert!(ts["fields"]["gyro[roll]"].as_array().unwrap().len() > 0);
    assert!(ts["fields"]["motor[0]"].as_array().unwrap().len() > 0);
    assert!(ts["sample_rate_hz"].as_f64().unwrap() > 0.0);
    assert!(ts["total_frames"].as_u64().unwrap() > 0);
}

#[test]
fn get_timeseries_decimation() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_002.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_timeseries(file_id, 0, 100, fields("gyro[roll]"));
    let ts: serde_json::Value = serde_json::from_str(&json).unwrap();

    let points = ts["time_s"].as_array().unwrap().len();
    let total = ts["total_frames"].as_u64().unwrap() as usize;
    // step_by rounds up: ceil(total / max_points) steps → points can be max_points + 1
    assert!(
        points <= 101,
        "decimation should cap near max_points, got {points}"
    );
    assert!(points < total, "expected fewer points than total frames");
    assert!(
        ts["decimation"].as_u64().unwrap() > 1,
        "expected decimation > 1"
    );
}

#[test]
fn get_timeseries_unknown_field_skipped() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_timeseries(file_id, 0, 100, fields("nonexistent_field"));
    let ts: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Unknown fields resolve to empty data (SensorField::Unknown)
    let fields = ts["fields"].as_object().unwrap();
    for (_key, val) in fields {
        let arr = val.as_array().unwrap();
        assert!(
            arr.is_empty() || arr.iter().all(|v| v.as_f64() == Some(0.0)),
            "unknown field should have empty or zero data"
        );
    }
}

#[test]
fn get_timeseries_invalid_session() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_timeseries(file_id, 999, 100, fields("gyro[roll]"));
    let ts: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(ts["error"].is_string());
}

// ---------------------------------------------------------------------------
// Spectrogram
// ---------------------------------------------------------------------------

#[test]
fn get_spectrogram_returns_axes() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_002.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json =
        propwash_web::get_spectrogram(file_id, 0, fields("gyro[roll],gyro[pitch],gyro[yaw]"));
    let sg: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Either axes data or an error for too-short logs
    if sg.get("error").is_none() {
        let axes = sg["axes"].as_array().unwrap();
        assert!(!axes.is_empty());
    }
}

// ---------------------------------------------------------------------------
// Filter config
// ---------------------------------------------------------------------------

#[test]
fn get_filter_config_returns_json() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_filter_config(file_id, 0);
    let fc: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Should be valid JSON with filter fields (may be null if not configured)
    assert!(fc.is_object());
}

// ---------------------------------------------------------------------------
// Raw frames
// ---------------------------------------------------------------------------

#[test]
fn get_raw_frames_returns_data() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_raw_frames(file_id, 0, 0, 10, fields("gyro[roll],motor[0]"));
    let raw: serde_json::Value = serde_json::from_str(&json).unwrap();

    assert_eq!(raw["start"], 0);
    assert!(raw["total"].as_u64().unwrap() > 0);
    let frames = raw["frames"].as_array().unwrap();
    assert_eq!(frames.len(), 10);
    // Each frame should have 2 columns
    assert_eq!(frames[0].as_array().unwrap().len(), 2);
}

#[test]
fn get_raw_frames_clamped_to_total() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    // Request beyond total frames
    let json = propwash_web::get_raw_frames(file_id, 0, 999_999, 100, fields("gyro[roll]"));
    let raw: serde_json::Value = serde_json::from_str(&json).unwrap();

    let frames = raw["frames"].as_array().unwrap();
    assert!(frames.is_empty() || frames.len() <= 100);
}

// ---------------------------------------------------------------------------
// Trend
// ---------------------------------------------------------------------------

#[test]
fn get_trend_with_multiple_files() {
    propwash_web::clear_workspace();
    let data1 = read_fixture("fc-blackbox/btfl_001.bbl");
    let data2 = read_fixture("fc-blackbox/btfl_002.bbl");
    propwash_web::add_file(&data1, "btfl_001.bbl");
    propwash_web::add_file(&data2, "btfl_002.bbl");

    let json = propwash_web::get_trend();
    let trend: Vec<serde_json::Value> = serde_json::from_str(&json).unwrap();

    assert!(
        trend.len() >= 2,
        "expected at least 2 trend points, got {}",
        trend.len()
    );
    for pt in &trend {
        assert!(pt["label"].is_string());
        assert!(pt["duration_seconds"].as_f64().unwrap() > 0.0);
    }
}

// ---------------------------------------------------------------------------
// Multi-format support
// ---------------------------------------------------------------------------

#[test]
fn add_ardupilot_file() {
    propwash_web::clear_workspace();
    let data = read_fixture("ardupilot/dronekit-copter-log171.bin");
    let json = propwash_web::add_file(&data, "copter.bin");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    let sessions = result["sessions"].as_array().unwrap();
    assert!(!sessions.is_empty());
    assert!(sessions[0]["frame_count"].as_u64().unwrap() > 0);
}

#[test]
fn add_px4_file() {
    propwash_web::clear_workspace();
    let data = read_fixture("px4/sample_log_small.ulg");
    let json = propwash_web::add_file(&data, "sample.ulg");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    let sessions = result["sessions"].as_array().unwrap();
    assert!(!sessions.is_empty());
}

#[test]
fn add_mavlink_file() {
    propwash_web::clear_workspace();
    let data = read_fixture("mavlink/dronekit-flight.tlog");
    let json = propwash_web::add_file(&data, "flight.tlog");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    let sessions = result["sessions"].as_array().unwrap();
    assert!(!sessions.is_empty());
    assert!(sessions[0]["frame_count"].as_u64().unwrap() > 0);
}

// ---------------------------------------------------------------------------
// Analysis fields in session result
// ---------------------------------------------------------------------------

#[test]
fn session_result_contains_analysis() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_002.bbl");
    let json = propwash_web::add_file(&data, "test.bbl");
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    let s = &result["sessions"][0];
    let analysis = &s["analysis"];

    // Summary fields
    assert!(analysis["summary"]["total_events"].is_number());
    assert!(analysis["summary"]["motor_count"].is_number());
    assert!(analysis["summary"]["motor_balance"].is_array());

    // Diagnostics
    assert!(analysis["diagnostics"].is_array());

    // Events
    assert!(analysis["events"].is_array());
}

// ---------------------------------------------------------------------------
// Step overlay
// ---------------------------------------------------------------------------

#[test]
fn get_step_overlay_gentle_flight() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_002.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_step_overlay(file_id, 0);
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Gentle flight may not have enough steps — either error or valid response
    assert!(
        result.get("error").is_some() || result.get("axes").is_some(),
        "should return error or axes"
    );
}

#[test]
fn get_step_overlay_aggressive_flight() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_035.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_step_overlay(file_id, 0);
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();

    // btfl_035 has aggressive flying — should have step data
    if let Some(axes) = result.get("axes") {
        let axes = axes.as_array().unwrap();
        if !axes.is_empty() {
            let first = &axes[0];
            assert!(first["time_ms"].is_array());
            assert!(first["gyro_steps"].is_array());
            assert!(first["gyro_average"].is_array());
            assert!(first["setpoint_steps"].is_array());
        }
    }
}

#[test]
fn get_step_overlay_invalid_session() {
    propwash_web::clear_workspace();
    let data = read_fixture("fc-blackbox/btfl_001.bbl");
    let r: serde_json::Value =
        serde_json::from_str(&propwash_web::add_file(&data, "test.bbl")).unwrap();
    let file_id = r["file_id"].as_u64().unwrap() as u32;

    let json = propwash_web::get_step_overlay(file_id, 999);
    let result: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(result["error"].is_string());
}
