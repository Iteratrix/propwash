//! Integration tests against the typed [`Session`] API.
//!
//! Per-format smoke tests + targeted assertions on units (verifies the
//! conversions in each `build.rs` actually produced sensible values).
//! Fixtures live in `propwash-core/tests/fixtures/`.
//!
//! The pre-refactor golden suite (frame-count assertions, etc.) is
//! preserved at `tests/integration.rs.bak` and can be selectively
//! ported as needed.

use std::path::PathBuf;

use propwash_core::session::{Format, Session};

fn fixture(rel: &str) -> Vec<u8> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(rel);
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {}", path.display(), e))
}

fn decode(rel: &str) -> Vec<Session> {
    let log =
        propwash_core::decode(&fixture(rel)).unwrap_or_else(|e| panic!("decode {rel}: {e:?}"));
    log.sessions
}

// ── Betaflight ─────────────────────────────────────────────────────────────

#[test]
fn bf_decodes_typical_log() {
    let sessions = decode("fc-blackbox/btfl_002.bbl");
    assert!(!sessions.is_empty(), "expected ≥1 session");
    let s = &sessions[0];
    assert_eq!(s.meta.format, Format::Betaflight);
    assert!(!s.meta.firmware.is_empty(), "firmware string populated");
    assert!(s.gyro.len() > 100, "expected many gyro samples");
    assert_eq!(
        s.gyro.values.roll.len(),
        s.gyro.time_us.len(),
        "gyro roll vec length matches time axis"
    );
}

#[test]
fn bf_gyro_in_realistic_deg_per_sec_range() {
    let sessions = decode("fc-blackbox/btfl_002.bbl");
    let gyro = &sessions[0].gyro.values.roll;
    let max = gyro.iter().map(|v| v.0.abs()).fold(0.0_f64, f64::max);
    assert!(max > 0.1, "gyro roll should have non-trivial magnitude");
    assert!(
        max < 5000.0,
        "gyro roll magnitude {max} exceeds 5000 deg/s — likely unit miscalc"
    );
}

#[test]
fn bf_motors_normalized_to_unit_range() {
    let sessions = decode("fc-blackbox/btfl_002.bbl");
    let motors = &sessions[0].motors;
    assert!(!motors.commands.is_empty(), "expected ≥1 motor");
    for (i, col) in motors.commands.iter().enumerate() {
        let max = col.iter().map(|n| n.0).fold(0.0_f32, f32::max);
        let min = col.iter().map(|n| n.0).fold(1.0_f32, f32::min);
        assert!(
            (0.0..=1.0).contains(&min),
            "motor {i} min {min} outside [0,1]"
        );
        assert!(
            (0.0..=1.0).contains(&max),
            "motor {i} max {max} outside [0,1]"
        );
    }
}

#[test]
fn bf_gps_coords_in_decimal_degrees() {
    let sessions = decode("fc-blackbox/btfl_gps_rescue.bbl");
    let gps = sessions[0]
        .gps
        .as_ref()
        .expect("gps_rescue fixture should have GPS");
    assert!(!gps.lat.is_empty());
    let lat0 = gps.lat[0].0;
    let lng0 = gps.lng[0].0;
    assert!((-90.0..=90.0).contains(&lat0), "lat {lat0} not in [-90,90]");
    assert!(
        (-180.0..=180.0).contains(&lng0),
        "lng {lng0} not in [-180,180]"
    );
    // gps_rescue fixture is in Europe (Sweden ~50.20° N).
    assert!((49.0..=51.0).contains(&lat0), "lat {lat0} ≠ ~50°");
}

#[test]
fn bf_multi_session_log_yields_multiple_sessions() {
    let sessions = decode("fc-blackbox/btfl_all.bbl");
    assert!(
        sessions.len() > 5,
        "btfl_all has many concatenated sessions; got {}",
        sessions.len()
    );
}

// ── ArduPilot ──────────────────────────────────────────────────────────────

#[test]
fn ap_decodes_typical_log() {
    let sessions = decode("ardupilot/dronekit-copter-log171.bin");
    assert!(!sessions.is_empty());
    let s = &sessions[0];
    assert_eq!(s.meta.format, Format::ArduPilot);
    assert!(!s.gyro.is_empty(), "expected gyro data from IMU/GYR");
}

#[test]
fn ap_gyro_in_realistic_deg_per_sec_range() {
    let sessions = decode("ardupilot/dronekit-copter-log171.bin");
    let gyro = &sessions[0].gyro.values.roll;
    if gyro.is_empty() {
        return; // skip if this fixture lacks gyro
    }
    let max = gyro.iter().map(|v| v.0.abs()).fold(0.0_f64, f64::max);
    assert!(
        max < 5000.0,
        "gyro roll magnitude {max} exceeds 5000 deg/s — rad→deg conversion likely missing"
    );
}

#[test]
fn ap_esc_telemetry_rounds_trip() {
    let sessions = decode("ardupilot/esc-telem-quadplane-v4.4.4.bin");
    let s = &sessions[0];
    let esc = s.motors.esc.as_ref().expect("esc telemetry expected");
    assert!(!esc.erpm.is_empty(), "expected ≥1 ESC instance");
}

// ── PX4 ────────────────────────────────────────────────────────────────────

#[test]
fn px4_decodes_small_log() {
    let sessions = decode("px4/sample_log_small.ulg");
    assert!(!sessions.is_empty());
    let s = &sessions[0];
    assert_eq!(s.meta.format, Format::Px4);
    assert!(
        !s.gyro.is_empty(),
        "expected gyro from vehicle_angular_velocity / sensor_combined / sensor_gyro"
    );
}

#[test]
fn px4_gyro_in_realistic_deg_per_sec_range() {
    let sessions = decode("px4/sample_log_small.ulg");
    let gyro = &sessions[0].gyro.values.roll;
    if gyro.is_empty() {
        return;
    }
    let max = gyro.iter().map(|v| v.0.abs()).fold(0.0_f64, f64::max);
    assert!(max < 5000.0, "gyro roll magnitude {max} too large");
}

// ── MAVLink ────────────────────────────────────────────────────────────────

#[test]
fn mavlink_decodes_tlog() {
    let sessions = decode("mavlink/dronekit-flight.tlog");
    assert!(!sessions.is_empty());
    let s = &sessions[0];
    assert_eq!(s.meta.format, Format::Mavlink);
}

#[test]
fn mavlink_gyro_in_realistic_deg_per_sec_range() {
    let sessions = decode("mavlink/dronekit-flight.tlog");
    let gyro = &sessions[0].gyro.values.roll;
    if gyro.is_empty() {
        return;
    }
    let max = gyro.iter().map(|v| v.0.abs()).fold(0.0_f64, f64::max);
    assert!(max < 5000.0, "gyro roll magnitude {max} too large");
}

// ── Heading regression (bug_006) ──────────────────────────────────────────

#[test]
fn ap_heading_uses_airframe_attitude_not_gps_cog() {
    let sessions = decode("ardupilot/dronekit-copter-log171.bin");
    let s = &sessions[0];
    // ATT.Yaw should populate session.attitude.yaw for AP logs.
    assert!(
        !s.attitude.values.yaw.is_empty(),
        "expected airframe yaw from AP ATT message"
    );
    // Heading values should be in degrees, well-bounded.
    let max = s
        .attitude
        .values
        .yaw
        .iter()
        .map(|v| v.abs())
        .fold(0.0_f32, f32::max);
    assert!(max <= 360.0, "yaw {max} not in plausible degree range");
}

#[test]
fn field_heading_prefers_attitude_over_gps_cog() {
    use propwash_core::types::SensorField;
    let sessions = decode("ardupilot/dronekit-copter-log171.bin");
    let s = &sessions[0];
    let heading = s.field(&SensorField::Heading);
    assert_eq!(
        heading.len(),
        s.attitude.values.yaw.len(),
        "field(Heading) should source from attitude.yaw, not gps.heading"
    );
}

// ── Cross-format unit-sanity ─────────────────────────────────────────────
//
// One assert per (format, quantity) that the value range is physically
// plausible. Catches whole classes of regressions: a build-step that
// forgets to rad→deg, mV→V, ×0.01, etc. would push values outside
// these ranges. Skips per-fixture when the underlying field is empty.

fn sanity_check_session(s: &Session, label: &str) {
    if let Some(v) = s.vbat.values.first().map(|v| v.0) {
        assert!(
            (0.0..=80.0).contains(&v),
            "{label} vbat[0] = {v} V outside plausible 0-80 V range"
        );
    }
    if let Some(c) = s.current.values.first().map(|c| c.0) {
        // Allow small negative for sensor noise; cap at 500 A (extreme racing).
        assert!(
            (-5.0..=500.0).contains(&c),
            "{label} current[0] = {c} A outside plausible range"
        );
    }
    for (i, motor) in s.motors.commands.iter().enumerate() {
        if let Some(n) = motor.first() {
            assert!(
                (0.0..=1.0).contains(&n.0),
                "{label} motor[{i}][0] = {} outside [0, 1]",
                n.0
            );
        }
    }
    if let Some(gps) = &s.gps {
        if let Some(lat) = gps.lat.first().map(|d| d.0) {
            assert!(
                (-90.0..=90.0).contains(&lat),
                "{label} gps.lat[0] = {lat} outside [-90, 90]"
            );
        }
        if let Some(lng) = gps.lng.first().map(|d| d.0) {
            assert!(
                (-180.0..=180.0).contains(&lng),
                "{label} gps.lng[0] = {lng} outside [-180, 180]"
            );
        }
    }
    // Attitude (added by bug_006) — degrees, finite or NaN (NaN-padded
    // axes from PX4/MAVLink fallback paths are intentional).
    for (axis_name, axis_vals) in [
        ("roll", &s.attitude.values.roll),
        ("pitch", &s.attitude.values.pitch),
        ("yaw", &s.attitude.values.yaw),
    ] {
        for &v in axis_vals.iter().take(50) {
            if v.is_nan() {
                continue;
            }
            assert!(
                (-360.0..=360.0).contains(&v),
                "{label} attitude.{axis_name} = {v} outside [-360, 360]"
            );
        }
    }
}

#[test]
fn bf_unit_sanity() {
    for s in decode("fc-blackbox/btfl_002.bbl") {
        sanity_check_session(&s, "btfl_002");
    }
}

#[test]
fn ap_unit_sanity() {
    for s in decode("ardupilot/dronekit-copter-log171.bin") {
        sanity_check_session(&s, "dronekit-copter");
    }
    // bug_006 + F7 cross-check: AP attitude is centidegrees on the wire;
    // post-F7 the parser scales them, so values should be plausible
    // degrees, not raw centidegrees (which would be 100× larger).
    let sessions = decode("ardupilot/dronekit-copter-log171.bin");
    let yaw = &sessions[0].attitude.values.yaw;
    if let Some(&v) = yaw.iter().find(|v| !v.is_nan()) {
        assert!(
            v.abs() <= 360.0,
            "AP attitude yaw = {v} not in [-360, 360] — \
             centidegree scaling likely double-applied or missing"
        );
    }
}

#[test]
fn px4_unit_sanity() {
    for s in decode("px4/sample_log_small.ulg") {
        sanity_check_session(&s, "px4-sample");
    }
}

#[test]
fn mavlink_unit_sanity() {
    for s in decode("mavlink/dronekit-flight.tlog") {
        sanity_check_session(&s, "mavlink-dronekit");
    }
}

// ── Spectrogram smoke per format ──────────────────────────────────────────

#[test]
fn spectrogram_smoke_each_format() {
    use propwash_core::analysis::fft::compute_spectrogram;
    use propwash_core::units::DegPerSec;

    for fixture in [
        "fc-blackbox/btfl_002.bbl",
        "ardupilot/dronekit-copter-log171.bin",
        "px4/sample_log_small.ulg",
        "mavlink/dronekit-flight.tlog",
    ] {
        let sessions = decode(fixture);
        let s = &sessions[0];
        let roll: &[f64] = bytemuck::cast_slice::<DegPerSec, f64>(&s.gyro.values.roll);
        let pitch: &[f64] = bytemuck::cast_slice::<DegPerSec, f64>(&s.gyro.values.pitch);
        let yaw: &[f64] = bytemuck::cast_slice::<DegPerSec, f64>(&s.gyro.values.yaw);
        let axes = [("roll", roll), ("pitch", pitch), ("yaw", yaw)];
        let spec = compute_spectrogram(s.sample_rate_hz(), &s.gyro.time_us, &axes);
        // Some fixtures may not have enough samples for a full
        // spectrogram window; require ≥1 axis produced output for
        // those that do, but skip cleanly for those that don't.
        if let Some(spec) = spec {
            assert!(
                !spec.axes.is_empty(),
                "{fixture}: spectrogram returned Some but no axes"
            );
            assert!(spec.sample_rate_hz > 0.0);
        }
    }
}

// ── Format dispatch ────────────────────────────────────────────────────────

#[test]
fn empty_input_errors() {
    assert!(propwash_core::decode(b"").is_err());
}

#[test]
fn garbage_input_errors() {
    assert!(propwash_core::decode(b"this is not a log").is_err());
}
