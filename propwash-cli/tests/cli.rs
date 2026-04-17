use std::path::Path;
use std::process::Command;

fn fixtures_dir() -> &'static Path {
    Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../propwash-core/tests/fixtures"
    ))
}

fn fixture(rel_path: &str) -> String {
    fixtures_dir().join(rel_path).to_string_lossy().to_string()
}

fn propwash() -> Command {
    Command::new(env!("CARGO_BIN_EXE_propwash-cli"))
}

// ---------------------------------------------------------------------------
// info
// ---------------------------------------------------------------------------

#[test]
fn info_betaflight() {
    let output = propwash()
        .args(["info", &fixture("fc-blackbox/btfl_001.bbl")])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Session"), "expected session info");
    assert!(stdout.contains("Firmware"), "expected firmware info");
    assert!(stdout.contains("Duration"), "expected duration");
    assert!(stdout.contains("Fields"), "expected field list");
}

#[test]
fn info_json() {
    let output = propwash()
        .args(["info", "--json", &fixture("fc-blackbox/btfl_001.bbl")])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let sessions = parsed["sessions"].as_array().unwrap();
    assert!(!sessions.is_empty());
    assert!(sessions[0]["frame_count"].as_u64().unwrap() > 0);
}

#[test]
fn info_ardupilot() {
    let output = propwash()
        .args(["info", &fixture("ardupilot/dronekit-copter-log171.bin")])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Session"));
}

#[test]
fn info_px4() {
    let output = propwash()
        .args(["info", &fixture("px4/sample_log_small.ulg")])
        .output()
        .unwrap();
    assert!(output.status.success());
}

#[test]
fn info_mavlink() {
    let output = propwash()
        .args(["info", &fixture("mavlink/dronekit-flight.tlog")])
        .output()
        .unwrap();
    assert!(output.status.success());
}

#[test]
fn info_nonexistent_file() {
    let output = propwash()
        .args(["info", "/tmp/does_not_exist.bbl"])
        .output()
        .unwrap();
    assert!(!output.status.success());
}

// ---------------------------------------------------------------------------
// analyze
// ---------------------------------------------------------------------------

#[test]
fn analyze_summary() {
    let output = propwash()
        .args(["analyze", &fixture("fc-blackbox/btfl_002.bbl")])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Session"), "expected session header");
    assert!(
        stdout.contains("Vibration Analysis"),
        "expected vibration section"
    );
    assert!(stdout.contains("Summary"), "expected summary section");
}

#[test]
fn analyze_json() {
    let output = propwash()
        .args([
            "analyze",
            "--output",
            "json",
            &fixture("fc-blackbox/btfl_002.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert!(parsed["summary"].is_object());
    assert!(parsed["diagnostics"].is_array());
    assert!(parsed["episodes"].is_array());
}

#[test]
fn analyze_invalid_output_format() {
    let output = propwash()
        .args([
            "analyze",
            "--output",
            "xml",
            &fixture("fc-blackbox/btfl_001.bbl"),
        ])
        .output()
        .unwrap();
    assert!(!output.status.success());
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("Unknown output format"));
}

// ---------------------------------------------------------------------------
// compare
// ---------------------------------------------------------------------------

#[test]
fn compare_two_files() {
    let output = propwash()
        .args([
            "compare",
            &fixture("fc-blackbox/btfl_001.bbl"),
            &fixture("fc-blackbox/btfl_002.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Duration"), "expected duration comparison");
    assert!(
        stdout.contains("Diagnostics"),
        "expected diagnostics section"
    );
}

#[test]
fn compare_same_file() {
    let f = fixture("fc-blackbox/btfl_001.bbl");
    let output = propwash().args(["compare", &f, &f]).output().unwrap();
    assert!(output.status.success());
}

// ---------------------------------------------------------------------------
// scan
// ---------------------------------------------------------------------------

#[test]
fn scan_multiple_files() {
    let output = propwash()
        .args([
            "scan",
            &fixture("fc-blackbox/btfl_001.bbl"),
            &fixture("fc-blackbox/btfl_002.bbl"),
            &fixture("fc-blackbox/LOG00037.BFL"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let lines: Vec<&str> = stdout.lines().collect();
    assert!(
        lines.len() >= 3,
        "expected at least 3 scan results, got {}",
        lines.len()
    );
}

// ---------------------------------------------------------------------------
// trend
// ---------------------------------------------------------------------------

#[test]
fn trend_multiple_files() {
    let output = propwash()
        .args([
            "trend",
            &fixture("fc-blackbox/btfl_001.bbl"),
            &fixture("fc-blackbox/btfl_002.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Session"), "expected table header");
    assert!(stdout.contains("NF(dB)"), "expected noise floor column");
}

#[test]
fn trend_json() {
    let output = propwash()
        .args([
            "trend",
            "--json",
            &fixture("fc-blackbox/btfl_001.bbl"),
            &fixture("fc-blackbox/btfl_002.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert!(parsed.len() >= 2);
}

#[test]
fn trend_directory() {
    let dir = fixtures_dir()
        .join("fc-blackbox")
        .to_string_lossy()
        .to_string();
    let output = propwash().args(["trend", &dir]).output().unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let data_lines = stdout
        .lines()
        .filter(|l| !l.starts_with("Session") && !l.starts_with('-'))
        .count();
    assert!(
        data_lines >= 3,
        "expected multiple trend rows from directory"
    );
}

// ---------------------------------------------------------------------------
// dump
// ---------------------------------------------------------------------------

#[test]
fn dump_default() {
    let output = propwash()
        .args(["dump", &fixture("fc-blackbox/btfl_001.bbl")])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let sessions = parsed["sessions"].as_array().unwrap();
    assert!(!sessions.is_empty());
    assert!(sessions[0]["frames"].as_array().unwrap().len() > 0);
}

#[test]
fn dump_with_session_filter() {
    let output = propwash()
        .args([
            "dump",
            "--session",
            "1",
            &fixture("fc-blackbox/btfl_all.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let sessions = parsed["sessions"].as_array().unwrap();
    // Should have exactly the filtered session (or 0 if index doesn't match)
    assert!(sessions.len() <= 1);
}

#[test]
fn dump_with_frame_range() {
    let output = propwash()
        .args([
            "dump",
            "--frames",
            "0-10",
            &fixture("fc-blackbox/btfl_001.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let frames = parsed["sessions"][0]["frames"].as_array().unwrap();
    assert!(
        frames.len() <= 11,
        "frame range 0-10 should return at most 11 frames"
    );
}

#[test]
fn dump_with_field_filter() {
    let output = propwash()
        .args([
            "dump",
            "--fields",
            "gyro",
            "--frames",
            "0-5",
            &fixture("fc-blackbox/btfl_001.bbl"),
        ])
        .output()
        .unwrap();
    assert!(output.status.success());

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    let fields = parsed["sessions"][0]["fields"].as_array().unwrap();
    for field in fields {
        assert!(
            field.as_str().unwrap().starts_with("gyro"),
            "field {:?} should start with 'gyro'",
            field
        );
    }
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

#[test]
fn no_subcommand_shows_help() {
    let output = propwash().output().unwrap();
    // clap exits with error when no subcommand given
    assert!(!output.status.success());
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("Usage") || stderr.contains("usage"),
        "expected usage info"
    );
}
