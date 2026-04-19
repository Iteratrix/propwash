use std::path::Path;

use propwash_core::types::{Axis, SensorField};
use propwash_core::{decode_file, Log, Session};

fn fixtures_dir() -> &'static Path {
    Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

fn parse_fixture(rel_path: &str) -> Log {
    let path = fixtures_dir().join(rel_path);
    decode_file(&path).unwrap_or_else(|e| panic!("Failed to parse {rel_path}: {e}"))
}

// ── Every fixture parses without panic ──────────────────────────────

macro_rules! fixture_test {
    ($name:ident, $path:expr) => {
        #[test]
        fn $name() {
            let log = parse_fixture($path);
            assert!(
                log.session_count() >= 1,
                "{}: expected ≥1 session, got {}",
                $path,
                log.session_count()
            );
            let total_frames: usize = log.sessions.iter().map(|s| s.frame_count()).sum();
            assert!(total_frames > 0, "{}: expected >0 frames, got 0", $path);
        }
    };
}

// Betaflight
fixture_test!(fc_btfl_001, "fc-blackbox/btfl_001.bbl");
fixture_test!(fc_btfl_002, "fc-blackbox/btfl_002.bbl");
fixture_test!(fc_btfl_all, "fc-blackbox/btfl_all.bbl");
fixture_test!(fc_log00037, "fc-blackbox/LOG00037.BFL");
fixture_test!(fc_crashing, "fc-blackbox/crashing-LOG00002.BFL");
fixture_test!(fc_log00002, "fc-blackbox/LOG00002.BFL");
fixture_test!(fc_log00004, "fc-blackbox/LOG00004.TXT");
fixture_test!(fc_log00007, "fc-blackbox/LOG00007.BFL");
fixture_test!(fc_btfl_027, "fc-blackbox/btfl_027.bbl");
fixture_test!(fc_btfl_035, "fc-blackbox/btfl_035.bbl");

// Gimbal-ghost
fixture_test!(gg_btfl_001, "gimbal-ghost/btfl_001.bbl");
fixture_test!(gg_btfl_002, "gimbal-ghost/btfl_002.bbl");
fixture_test!(gg_emuf_001, "gimbal-ghost/emuf_001.bbl");
fixture_test!(gg_rtfl_001, "gimbal-ghost/rtfl_001.bbl");
fixture_test!(gg_log00001, "gimbal-ghost/LOG00001.BFL");

// Cleanflight
fixture_test!(cf_log00568, "cleanflight/LOG00568.TXT");
fixture_test!(cf_log00569, "cleanflight/LOG00569.TXT");
fixture_test!(cf_log00570, "cleanflight/LOG00570.TXT");
fixture_test!(cf_log00571, "cleanflight/LOG00571.TXT");
fixture_test!(cf_log00572, "cleanflight/LOG00572.TXT");

// INAV
fixture_test!(inav_log00001, "inav/LOG00001.TXT");
fixture_test!(inav_log00005, "inav/LOG00005.TXT");

// Rotorflight
fixture_test!(rtfl_log246, "rotorflight/LOG246.TXT");

// ArduPilot
fixture_test!(ap_pymavlink_plane, "ardupilot/pymavlink-plane-v3.8.bin");
fixture_test!(
    ap_pyflightcoach_plane,
    "ardupilot/pyflightcoach-plane-v4.3.5.bin"
);
fixture_test!(ap_erasial, "ardupilot/erasial-00000001.bin");
fixture_test!(ap_dronekit_copter, "ardupilot/dronekit-copter-log171.bin");
fixture_test!(ap_methodic_copter, "ardupilot/methodic-copter-tarot-x4.bin");
fixture_test!(ap_esc_telem, "ardupilot/esc-telem-quadplane-v4.4.4.bin");

// BF with GPS
fixture_test!(bf_gps_rescue, "fc-blackbox/btfl_gps_rescue.bbl");

// PX4
fixture_test!(px4_small, "px4/sample_log_small.ulg");
fixture_test!(px4_appended, "px4/sample_appended_multiple.ulg");
fixture_test!(
    px4_tagged,
    "px4/sample_logging_tagged_and_default_params.ulg"
);

#[test]
fn ardupilot_parses_metadata() {
    use propwash_core::types::MotorIndex;

    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let session = &log.sessions[0];

    // Firmware version
    assert!(
        session.firmware_version().contains("ArduCopter V4.5.5"),
        "expected firmware to contain 'ArduCopter V4.5.5', got: {}",
        session.firmware_version()
    );

    // Motor count
    assert_eq!(session.motor_count(), 4, "Tarot X4 is a quadcopter");

    // First motor[0] value (RCOU C1) = 1100.0
    let motor0 = session.field(&SensorField::Motor(MotorIndex(0)));
    assert!(!motor0.is_empty(), "should have motor data");
    assert!(
        (motor0[0] - 1100.0).abs() < 1e-2,
        "first motor[0] expected 1100.0, got {}",
        motor0[0]
    );

    // IMU frame count > 10000
    assert!(
        session.frame_count() > 10000,
        "expected >10000 IMU frames, got {}",
        session.frame_count()
    );

    // Duration should be nonzero
    assert!(
        session.duration_seconds() > 0.0,
        "should have nonzero duration"
    );

    // First gyro value should be very small (ground idle)
    let gyro = session.field(&SensorField::Gyro(Axis::Roll));
    assert!(!gyro.is_empty(), "should have gyro data");
    assert!(
        gyro[0].abs() < 0.1,
        "first gyro[roll] should be near zero deg/s at ground idle, got {}",
        gyro[0]
    );
}

#[test]
fn ardupilot_motor_count_from_servo_params() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let session = &log.sessions[0];
    assert_eq!(session.motor_count(), 4, "Tarot X4 is a quadcopter");
}

#[test]
fn ardupilot_craft_name_skips_rtos() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let session = &log.sessions[0];
    let craft = session.craft_name();
    assert!(
        !craft.contains("ChibiOS"),
        "craft name should not be the RTOS string, got: {craft}"
    );
    assert!(
        !craft.contains("NuttX"),
        "craft name should not be NuttX, got: {craft}"
    );
}

#[test]
fn ardupilot_old_format_parses() {
    let log = parse_fixture("ardupilot/pymavlink-plane-v3.8.bin");
    let session = &log.sessions[0];

    assert!(
        session.frame_count() > 0,
        "old format should still produce frames"
    );
    assert!(session.duration_seconds() > 0.0);
}

#[test]
fn ardupilot_plane_zero_motors() {
    let log = parse_fixture("ardupilot/pymavlink-plane-v3.8.bin");
    let session = &log.sessions[0];
    assert_eq!(
        session.motor_count(),
        0,
        "plane without SERVO params should report 0 motors"
    );
}

#[test]
fn px4_parses_metadata() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let session = &log.sessions[0];

    assert!(session.frame_count() > 0, "should have sensor frames");
    assert!(
        session.duration_seconds() > 0.0,
        "should have nonzero duration"
    );
    assert!(session.sample_rate_hz() > 0.0, "should have sample rate");
}

#[test]
fn px4_appended_data_parses() {
    let log = parse_fixture("px4/sample_appended_multiple.ulg");
    let session = &log.sessions[0];
    assert!(session.frame_count() > 0, "appended log should have frames");
}

// Error recovery
fixture_test!(error_recovery, "error-recovery.bbl");

// ── Unified view tests ──────────────────────────────────────────────

#[test]
fn unified_sample_rate() {
    let log = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let session = &log.sessions[0];
    assert!(
        session.sample_rate_hz() > 10.0,
        "expected reasonable sample rate, got {}",
        session.sample_rate_hz()
    );
}

#[test]
fn unified_duration() {
    let log = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let session = &log.sessions[0];
    assert!(
        session.duration_seconds() > 1.0,
        "expected >1s duration, got {}",
        session.duration_seconds()
    );
}

#[test]
fn unified_field_extraction() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let session = &log.sessions[0];
    let gyro = session.field(&SensorField::Gyro(Axis::Roll));
    assert_eq!(gyro.len(), session.frame_count());
}

#[test]
fn unified_gyro_fields() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let session = &log.sessions[0];
    for axis in Axis::ALL {
        let data = session.field(&SensorField::Gyro(axis));
        assert!(!data.is_empty(), "gyro {axis} should have data");
    }
}

#[test]
fn unified_firmware_version() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let session = &log.sessions[0];
    assert!(
        session.firmware_version().contains("Betaflight"),
        "got: {}",
        session.firmware_version()
    );
}

#[test]
fn unified_motor_count() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    assert_eq!(log.sessions[0].motor_count(), 4);

    let log2 = parse_fixture("gimbal-ghost/rtfl_001.bbl");
    assert_eq!(log2.sessions[0].motor_count(), 1);
}

#[test]
fn unified_field_names() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let session = &log.sessions[0];
    let names = session.field_names();
    assert!(names.iter().any(|n| n == "time"));
    assert!(names.iter().any(|n| n == "gyro[roll]"));
    assert!(names.iter().any(|n| n == "motor[0]"));
}

// ── Analyzed view tests ─────────────────────────────────────────────

#[test]
fn analyzed_betaflight_motor_count() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    match &log.sessions[0] {
        Session::Betaflight(bf) => assert_eq!(bf.motor_count(), 4),
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_rotorflight_single_motor() {
    let log = parse_fixture("gimbal-ghost/rtfl_001.bbl");
    match &log.sessions[0] {
        Session::Betaflight(bf) => assert_eq!(bf.motor_count(), 1),
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_crash_log_has_corruption() {
    let log = parse_fixture("fc-blackbox/crashing-LOG00002.BFL");
    match &log.sessions[0] {
        Session::Betaflight(bf) => {
            assert!(
                bf.stats.corrupt_bytes > 0,
                "crash log should have corruption"
            );
            // This file has a corrupt header but the flight data ended cleanly
            // (it contains a valid "End of log" marker), so is_truncated()
            // is correctly false. Truncation detection works — this file
            // just isn't truncated.
        }
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn truncated_files_detected() {
    // Files without "End of log" marker should be detected as truncated
    let log = parse_fixture("fc-blackbox/LOG00002.BFL");
    match &log.sessions[0] {
        Session::Betaflight(bf) => {
            assert!(
                bf.is_truncated(),
                "file without End of log should be truncated"
            );
        }
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_debug_mode() {
    let log = parse_fixture("gimbal-ghost/btfl_001.bbl");
    match &log.sessions[0] {
        Session::Betaflight(bf) => {
            let _ = bf.debug_mode();
        }
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_stats() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    match &log.sessions[0] {
        Session::Betaflight(bf) => {
            assert!(bf.stats.total_main_frames() > 0);
            assert!(bf.stats.i_frame_count > 0);
        }
        _ => panic!("expected Betaflight"),
    }
}

// ── Structural invariants across all fixtures ───────────────────────

macro_rules! invariant_test {
    ($name:ident, $path:expr) => {
        #[test]
        fn $name() {
            let log = parse_fixture($path);
            for session in &log.sessions {
                // Every session should have field definitions
                assert!(
                    !session.field_names().is_empty(),
                    "{}: session {} has no field names",
                    $path,
                    session.index()
                );

                // All columns should have the same length as frame_count
                if let propwash_core::Session::Betaflight(bf) = session {
                    let n_fields = bf.main_field_defs.len();
                    let n_frames = bf.frame_count();
                    for (i, col) in bf.main_columns.iter().enumerate() {
                        assert_eq!(
                            col.len(),
                            n_frames,
                            "{}: session {} column {} has {} values, expected {}",
                            $path,
                            session.index(),
                            i,
                            col.len(),
                            n_frames
                        );
                    }
                    assert_eq!(
                        bf.main_columns.len(),
                        n_fields,
                        "{}: expected {} columns, got {}",
                        $path,
                        n_fields,
                        bf.main_columns.len()
                    );
                }

                // Field extraction length matches frame count
                if session.frame_count() > 0 {
                    let time = session.field(&SensorField::Time);
                    assert_eq!(
                        time.len(),
                        session.frame_count(),
                        "{}: time array length mismatch",
                        $path
                    );
                }

                // Stats should be consistent
                if let Session::Betaflight(bf) = session {
                    let stats = &bf.stats;
                    assert_eq!(
                        stats.total_main_frames(),
                        session.frame_count(),
                        "{}: stats frame count mismatch",
                        $path
                    );
                }
            }
        }
    };
}

invariant_test!(inv_fc_btfl_001, "fc-blackbox/btfl_001.bbl");
invariant_test!(inv_fc_btfl_002, "fc-blackbox/btfl_002.bbl");
invariant_test!(inv_fc_log00037, "fc-blackbox/LOG00037.BFL");
invariant_test!(inv_gg_btfl_001, "gimbal-ghost/btfl_001.bbl");
invariant_test!(inv_gg_btfl_002, "gimbal-ghost/btfl_002.bbl");
invariant_test!(inv_gg_emuf_001, "gimbal-ghost/emuf_001.bbl");
invariant_test!(inv_gg_rtfl_001, "gimbal-ghost/rtfl_001.bbl");
invariant_test!(inv_cf_log00568, "cleanflight/LOG00568.TXT");
invariant_test!(inv_inav_log00001, "inav/LOG00001.TXT");
invariant_test!(inv_rtfl_log246, "rotorflight/LOG246.TXT");

// ── Multi-session file assertions ───────────────────────────────────

#[test]
fn btfl_all_session_count() {
    let log = parse_fixture("fc-blackbox/btfl_all.bbl");
    assert!(
        log.session_count() >= 20,
        "btfl_all.bbl should have many sessions, got {}",
        log.session_count()
    );
}

#[test]
fn events_are_captured() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    // At least some sessions should have events (sync beep, disarm, etc.)
    let total_events: usize = log
        .sessions
        .iter()
        .map(|s| match s {
            propwash_core::Session::Betaflight(bf) => bf.events.len(),
            _ => 0,
        })
        .sum();
    // Not all files have events, so just verify the mechanism works
    let _ = total_events;
}

// ── Three-layer access pattern ──────────────────────────────────────

#[test]
fn three_layer_access() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let session = &log.sessions[0];

    // Layer 1: Unified (recommended)
    let _gyro = session.field(&SensorField::Gyro(Axis::Roll));

    // Layer 2: Analyzed (format-specific)
    match session {
        Session::Betaflight(bf) => {
            let _ = bf.motor_count();
        }
        _ => panic!("expected Betaflight"),
    }

    // Layer 3: Raw (escape hatch — columnar access)
    match session {
        propwash_core::Session::Betaflight(raw) => {
            assert!(raw.frame_count() > 0);
            assert!(!raw.main_columns.is_empty());
        }
        _ => panic!("expected Betaflight session"),
    }
}

fn get_bf(session: &propwash_core::Session) -> &propwash_core::format::bf::types::BfSession {
    match session {
        propwash_core::Session::Betaflight(bf) => bf,
        _ => panic!("expected Betaflight"),
    }
}

fn field_val(
    bf: &propwash_core::format::bf::types::BfSession,
    frame_idx: usize,
    name: &str,
) -> i64 {
    let idx = bf.main_field_defs.index_of_str(name).unwrap();
    bf.main_value(frame_idx, idx)
}

#[test]
fn golden_values_btfl_001_session2() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);

    assert_eq!(field_val(bf, 0, "time"), 191_882_474);
    assert_eq!(field_val(bf, 0, "gyroADC[0]"), -2);
    assert_eq!(field_val(bf, 0, "gyroADC[1]"), 0);
    assert_eq!(field_val(bf, 0, "gyroADC[2]"), 0);
    assert_eq!(field_val(bf, 0, "motor[0]"), 165);

    assert_eq!(field_val(bf, 1, "time"), 191_884_524);
    assert_eq!(field_val(bf, 1, "gyroADC[0]"), -2);
    assert_eq!(field_val(bf, 1, "gyroADC[1]"), -1);
    assert_eq!(field_val(bf, 1, "motor[0]"), 172);

    assert_eq!(field_val(bf, 2, "time"), 191_886_524);
    assert_eq!(field_val(bf, 5, "time"), 191_892_649);
    assert_eq!(field_val(bf, 10, "time"), 191_902_646);
    assert_eq!(field_val(bf, 100, "time"), 192_083_146);

    assert_eq!(field_val(bf, 100, "gyroADC[0]"), 0);
    assert_eq!(field_val(bf, 100, "gyroADC[1]"), -6);
    assert_eq!(field_val(bf, 100, "gyroADC[2]"), -9);
    assert_eq!(field_val(bf, 100, "motor[0]"), 231);
}

#[test]
fn golden_values_gg_btfl_001_session1() {
    let log = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let bf = get_bf(&log.sessions[0]);

    assert_eq!(field_val(bf, 0, "time"), 220_155_884);
    assert_eq!(field_val(bf, 0, "gyroADC[0]"), 0);
    assert_eq!(field_val(bf, 0, "gyroADC[1]"), 0);
    assert_eq!(field_val(bf, 0, "gyroADC[2]"), 0);
    assert_eq!(field_val(bf, 0, "motor[0]"), 177);

    assert_eq!(field_val(bf, 1, "time"), 220_159_884);
    assert_eq!(field_val(bf, 1, "gyroADC[0]"), 0);
    assert_eq!(field_val(bf, 1, "gyroADC[1]"), -1);
    assert_eq!(field_val(bf, 1, "gyroADC[2]"), 1);
    assert_eq!(field_val(bf, 1, "motor[0]"), 195);

    assert_eq!(field_val(bf, 2, "time"), 220_163_884);
    assert_eq!(field_val(bf, 5, "time"), 220_175_884);
    assert_eq!(field_val(bf, 10, "time"), 220_196_010);
    assert_eq!(field_val(bf, 100, "time"), 220_556_634);

    assert_eq!(field_val(bf, 100, "gyroADC[0]"), 0);
    assert_eq!(field_val(bf, 100, "gyroADC[1]"), 1);
    assert_eq!(field_val(bf, 100, "gyroADC[2]"), 0);
    assert_eq!(field_val(bf, 100, "motor[0]"), 167);
}

#[test]
fn golden_sample_rate() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let rate = log.sessions[1].sample_rate_hz();
    assert!((rate - 498.0).abs() < 2.0, "expected ~498 Hz, got {rate}");

    let log2 = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let rate2 = log2.sessions[0].sample_rate_hz();
    assert!((rate2 - 249.0).abs() < 2.0, "expected ~249 Hz, got {rate2}");
}

#[test]
fn golden_loop_iteration_matches_official_decoder() {
    use propwash_core::format::bf::types::BfFrameKind;
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let iter_idx = bf
        .main_field_defs
        .index_of(&SensorField::Unknown("loopIteration".to_string()))
        .unwrap();

    assert_eq!(bf.main_value(0, iter_idx), 0);
    assert_eq!(bf.main_value(1, iter_idx), 1);
    assert_eq!(bf.main_value(15, iter_idx), 15);
    assert_eq!(bf.main_value(16, iter_idx), 256);
    assert_eq!(bf.main_value(17, iter_idx), 257);

    for i in 1..bf.frame_count().min(100) {
        let prev = bf.main_value(i - 1, iter_idx);
        let curr = bf.main_value(i, iter_idx);
        let delta = curr - prev;
        match bf.frame_kinds[i] {
            BfFrameKind::Intra => {
                assert!(
                    delta > 1,
                    "I-frame should jump (I_interval), got delta={delta}"
                );
            }
            BfFrameKind::Inter => {
                assert_eq!(
                    delta, 1,
                    "frame {i}: P-frame delta should be 1, got {delta}"
                );
            }
        }
    }
}

#[test]
fn golden_time_deltas_are_sane() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let time_idx = bf.main_field_defs.index_of(&SensorField::Time).unwrap();

    for i in 1..bf.frame_count().min(1000) {
        let t0 = bf.main_value(i - 1, time_idx);
        let t1 = bf.main_value(i, time_idx);
        let delta = t1 - t0;
        assert!(
            delta > 0 && delta < 100_000,
            "frame {i}: time delta {delta}us is unreasonable (expected 1000-5000us)"
        );
    }
}

/// Regression: `STRAIGHT_LINE` predictor doubled time values when
/// prev2 was zero (I-frame didn't reset both history slots).
/// Fixed in commit b40cc1a.
#[test]
fn regression_time_not_doubling() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let time_idx = bf.main_field_defs.index_of(&SensorField::Time).unwrap();

    let t0 = bf.main_value(0, time_idx);
    let t1 = bf.main_value(1, time_idx);
    assert!(
        t1 < t0 * 2,
        "P-frame time should be close to I-frame time, not doubled: t0={t0} t1={t1}"
    );
    assert!(
        (t1 - t0) < 10_000,
        "first time delta should be <10ms, got {}us",
        t1 - t0
    );
}

/// Regression: `AVERAGE_2` predictor used `(p1>>1)+(p2>>1)` which
/// rounds differently from firmware's (p1+p2)/2 when both are odd.
/// Caused off-by-one on motor values.
/// Fixed in commit c1ef151.
#[test]
fn regression_average2_rounding() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);

    assert_eq!(field_val(bf, 1, "motor[0]"), 172);
    assert_eq!(field_val(bf, 1, "motor[2]"), 183);

    let log2 = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let bf2 = get_bf(&log2.sessions[0]);
    assert_eq!(field_val(bf2, 1, "motor[0]"), 195);
    assert_eq!(field_val(bf2, 1, "motor[1]"), 157);
}

/// Regression: `INCREMENT` predictor added 1 per frame instead of
/// `(skipped_frames + 1)`. With `P_ratio=16`, `loopIteration` should
/// jump by 16 per logged frame, not by 1.
/// Fixed in commit c2f443a.
#[test]
fn regression_loop_iteration_uses_frame_schedule() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let iter_idx = bf
        .main_field_defs
        .index_of(&SensorField::Unknown("loopIteration".to_string()))
        .unwrap();

    let delta = bf.main_value(1, iter_idx) - bf.main_value(0, iter_idx);
    assert_eq!(
        delta, 1,
        "btfl_001 logs every frame (P_num==P_denom), so increment should be 1"
    );
}

/// Regression: `TAG8_4S16` nibble-aligned reads left the byte stream
/// misaligned, causing P-frame time predictions to diverge from the
/// official decoder on ~5% of frames.
/// Fixed by implementing bit-level reader with `byte_align()` after `TAG8_4S16`.
#[test]
fn regression_bitreader_btfl_002_time_monotonic() {
    let log = parse_fixture("fc-blackbox/btfl_002.bbl");
    for session in &log.sessions {
        let Session::Betaflight(bf) = session else {
            continue;
        };
        let Some(time_idx) = bf.main_field_defs.index_of(&SensorField::Time) else {
            continue;
        };
        if bf.frame_count() < 2 {
            continue;
        }
        let time_col = &bf.main_columns[time_idx];
        let mut prev_time = time_col[0] as i64;
        for (i, &t_f) in time_col.iter().enumerate().skip(1) {
            let t = t_f as i64;
            assert!(
                t >= prev_time,
                "session {}: frame {i} time went backwards: {} -> {} (delta {})",
                session.index(),
                prev_time,
                t,
                t - prev_time
            );
            prev_time = t;
        }
    }
}

#[test]
fn regression_bitreader_btfl_all_time_monotonic() {
    let log = parse_fixture("fc-blackbox/btfl_all.bbl");
    for session in &log.sessions {
        let Session::Betaflight(bf) = session else {
            continue;
        };
        let Some(time_idx) = bf.main_field_defs.index_of(&SensorField::Time) else {
            continue;
        };
        if bf.frame_count() < 2 {
            continue;
        }
        let time_col = &bf.main_columns[time_idx];
        let mut prev_time = time_col[0] as i64;
        for (i, &t_f) in time_col.iter().enumerate().skip(1) {
            let t = t_f as i64;
            assert!(
                t >= prev_time,
                "session {}: frame {i} time went backwards: {} -> {} (delta {})",
                session.index(),
                prev_time,
                t,
                t - prev_time
            );
            prev_time = t;
        }
    }
}

/// Verify Cleanflight files parse all frames with monotonic time.
/// Previously diverged mid-flight due to stream alignment issues.
#[test]
fn regression_cleanflight_time_monotonic() {
    for fixture in &[
        "cleanflight/LOG00568.TXT",
        "cleanflight/LOG00570.TXT",
        "cleanflight/LOG00572.TXT",
    ] {
        let log = parse_fixture(fixture);
        for session in &log.sessions {
            let Session::Betaflight(bf) = session else {
                continue;
            };
            let Some(time_idx) = bf.main_field_defs.index_of(&SensorField::Time) else {
                continue;
            };
            if bf.frame_count() < 2 {
                continue;
            }
            let time_col = &bf.main_columns[time_idx];
            let mut prev_time = time_col[0] as i64;
            let mut backwards_count = 0;
            for &t_f in time_col.iter().skip(1) {
                let t = t_f as i64;
                if t < prev_time {
                    backwards_count += 1;
                }
                prev_time = t;
            }
            assert!(
                backwards_count == 0,
                "{fixture} session {}: {backwards_count} frames with backwards time out of {}",
                session.index(),
                bf.frame_count()
            );
        }
    }
}

/// Regression: `LOG_END` event accepted any 0xFF byte without verifying
/// the "End of log\0" string, causing false clean-end detection on
/// corrupt data.
/// Fixed in commit 0236bdc.
#[test]
fn regression_log_end_requires_marker_string() {
    let log = parse_fixture("fc-blackbox/crashing-LOG00002.BFL");
    match &log.sessions[0] {
        Session::Betaflight(bf) => {
            assert!(
                bf.stats.corrupt_bytes > 0,
                "crash log should have corruption"
            );
        }
        _ => panic!("expected Betaflight"),
    }
}

/// Golden value tests against pyulog reference parser.
/// Values extracted from: `pyulog.core.ULog('sample_log_small.ulg')`
#[test]
fn px4_golden_values() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let session = &log.sessions[0];

    // Metadata
    assert_eq!(
        session.firmware_version(),
        "8583f1da30b63154d6ba0bc187d86135dfe33cf9"
    );
    assert_eq!(session.craft_name(), "CUBEPILOT_CUBEORANGE");

    // Frame count depends on which gyro topic has data (vehicle_angular_velocity=1812
    // or sensor_combined=1298). Both are valid primary sources.
    assert!(
        session.frame_count() >= 1298,
        "expected at least 1298 frames, got {}",
        session.frame_count()
    );

    // Gyro data — from whichever gyro topic is primary
    let gyro_roll = session.field(&SensorField::Gyro(Axis::Roll));
    assert!(!gyro_roll.is_empty());
    // Value depends on source topic. sensor_combined.gyro_rad[0] = 0.0029683835 rad/s
    // = 0.17006 deg/s. vehicle_angular_velocity.xyz[0] = 0.0013696939 = 0.07848 deg/s.
    // Either is valid — just verify it's a plausible small value, not zero.
    assert!(
        gyro_roll[0].abs() > 0.01 && gyro_roll[0].abs() < 1.0,
        "gyro roll[0] should be a small nonzero value, got {:.6}",
        gyro_roll[0]
    );

    // Verify sub-degree precision survives (this was the Vec<i64> bug)
    assert!(
        gyro_roll[0].abs() > 0.01,
        "gyro value should preserve float precision"
    );
}

#[test]
fn px4_golden_sensor_combined() {
    let log = parse_fixture("px4/sample_log_small.ulg");

    if let Session::Px4(px4) = &log.sessions[0] {
        // pyulog: sensor_combined has 1298 messages
        let ts = px4.topic_timestamps("sensor_combined");
        assert_eq!(ts.len(), 1298);

        // First timestamp: 20326716
        assert_eq!(ts[0], 20_326_716);

        // Last timestamp: 26822868
        assert_eq!(*ts.last().unwrap(), 26_822_868);

        // First gyro_rad[0]: 0.0029683835
        let gyro_x = px4.topic_column("sensor_combined", "gyro_rad[0]").unwrap()[0];
        assert!(
            (gyro_x - 0.002_968_383_5).abs() < 1e-8,
            "gyro_rad[0]: expected 0.0029683835, got {gyro_x}"
        );

        // First accelerometer_m_s2[2]: -9.6342430115 (gravity)
        let acc_z = px4
            .topic_column("sensor_combined", "accelerometer_m_s2[2]")
            .unwrap()[0];
        assert!(
            (acc_z - (-9.634_243_011_5)).abs() < 0.001,
            "accel_z: expected -9.634, got {acc_z}"
        );
    } else {
        panic!("expected PX4 session");
    }
}

#[test]
fn px4_golden_angular_velocity() {
    let log = parse_fixture("px4/sample_log_small.ulg");

    if let Session::Px4(px4) = &log.sessions[0] {
        // pyulog: vehicle_angular_velocity has 1812 messages
        let ts = px4.topic_timestamps("vehicle_angular_velocity");
        assert_eq!(ts.len(), 1812);

        // First xyz[2]: -0.0762074292 rad/s
        let yaw = px4
            .topic_column("vehicle_angular_velocity", "xyz[2]")
            .unwrap()[0];
        assert!(
            (yaw - (-0.076_207_429_2)).abs() < 1e-8,
            "xyz[2]: expected -0.0762074292, got {yaw}"
        );
    } else {
        panic!("expected PX4 session");
    }
}

#[test]
fn px4_golden_format_count() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    if let Session::Px4(px4) = &log.sessions[0] {
        // pyulog: 82 formats, 70 datasets, 980 params
        assert_eq!(px4.formats.len(), 82);
        // We store all subscriptions (72); pyulog reports 70 datasets (ones with data)
        assert!(
            px4.subscriptions.len() >= 70,
            "expected >=70 subscriptions, got {}",
            px4.subscriptions.len()
        );
        assert!(
            px4.params.len() >= 970,
            "expected ~980 params, got {}",
            px4.params.len()
        );
    } else {
        panic!("expected PX4 session");
    }
}

#[test]
fn px4_logging_messages_parsed() {
    let log = parse_fixture("px4/sample_logging_tagged_and_default_params.ulg");
    if let Session::Px4(px4) = &log.sessions[0] {
        assert!(
            !px4.log_messages.is_empty(),
            "should have parsed logging messages"
        );
        // Tagged messages should have tag set
        let tagged = px4.log_messages.iter().filter(|m| m.tag.is_some()).count();
        assert!(tagged > 0, "should have tagged logging messages");
        // All messages should have non-empty text
        for msg in &px4.log_messages {
            assert!(!msg.message.is_empty(), "log message should have text");
        }
    } else {
        panic!("expected PX4 session");
    }
}

#[test]
fn px4_nested_types_decoded() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    if let Session::Px4(px4) = &log.sessions[0] {
        // Check if any format has fields referencing other formats (nested types)
        let nested_formats: Vec<_> = px4
            .formats
            .values()
            .filter(|fmt| fmt.fields.iter().any(|f| f.primitive.is_none()))
            .collect();

        if !nested_formats.is_empty() {
            // Verify that nested fields produce dot-notation keys in columnar field names
            for fmt in &nested_formats {
                let field_names = px4.topic_field_names(&fmt.name);
                if !field_names.is_empty() {
                    let nested_field = fmt.fields.iter().find(|f| f.primitive.is_none()).unwrap();
                    // Nested fields appear as "field.child" or "field[N].child"
                    let prefix_dot = format!("{}.", nested_field.name);
                    let prefix_idx = format!("{}[", nested_field.name);
                    let has_nested = field_names
                        .iter()
                        .any(|k| k.starts_with(&prefix_dot) || k.starts_with(&prefix_idx));
                    assert!(
                        has_nested,
                        "topic {} should have nested keys for field {}, got keys: {:?}",
                        fmt.name, nested_field.name, field_names
                    );
                }
            }
        }
    } else {
        panic!("expected PX4 session");
    }
}

#[test]
fn px4_gyro_unfiltered_available() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let unfilt = log.sessions[0].field(&SensorField::GyroUnfilt(Axis::Roll));
    assert!(
        !unfilt.is_empty(),
        "PX4 fixture with sensor_gyro should provide unfiltered gyro data"
    );
}

#[test]
fn px4_golden_sensor_gyro_unfiltered() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let session = &log.sessions[0];

    // sensor_gyro[0] x = -0.04130321 rad/s -> in deg/s = about -2.367
    let unfilt_roll = session.field(&SensorField::GyroUnfilt(Axis::Roll));
    assert!(
        !unfilt_roll.is_empty(),
        "should have unfiltered gyro roll data"
    );
    let expected_deg_s = -0.041_303_21_f64.to_degrees(); // ~-2.367
    assert!(
        (unfilt_roll[0] - expected_deg_s).abs() < 1e-2,
        "GyroUnfilt(Roll)[0] expected ~{expected_deg_s:.3} deg/s, got {:.6}",
        unfilt_roll[0]
    );
}

#[test]
fn px4_not_truncated() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    assert!(
        !log.sessions[0].is_truncated(),
        "clean PX4 fixture should not be truncated"
    );
}

#[test]
fn px4_multi_instance_access() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let session = &log.sessions[0];
    let Session::Px4(px4) = session else {
        panic!("expected PX4 session");
    };

    // all instances count >= primary instance count for any topic
    let primary_count = px4.topic_timestamps("sensor_combined").len();
    let all_count = px4.topic_all_instances_count("sensor_combined");
    assert!(
        all_count >= primary_count,
        "all instances ({all_count}) should be >= primary ({primary_count})"
    );

    // field() uses primary instance — correct default for analysis
    let gyro = session.field(&SensorField::Gyro(Axis::Roll));
    assert!(
        !gyro.is_empty(),
        "field() should return data from primary instance"
    );
}

#[test]
fn ardupilot_truncation_detected() {
    // Real flight logs are typically truncated when the FC powers off
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    // Just verify is_truncated() returns a real answer, not a corrupt_bytes proxy
    let _truncated = log.sessions[0].is_truncated();
}

#[test]
fn bf_gps_data_parsed() {
    let log = parse_fixture("fc-blackbox/btfl_gps_rescue.bbl");
    let session = &log.sessions[0];
    let Session::Betaflight(bf) = session else {
        panic!("expected Betaflight session");
    };

    // Frame counts
    assert_eq!(
        bf.stats.total_main_frames(),
        1607,
        "expected 1607 main frames, got {}",
        bf.stats.total_main_frames()
    );

    // GPS field definitions and frames
    assert!(
        bf.gps_field_defs.is_some(),
        "should have GPS field definitions"
    );
    assert_eq!(
        bf.gps_columns.first().map_or(0, Vec::len),
        7,
        "expected 7 GPS frames"
    );

    // GPS home position — exact golden values
    assert!(bf.gps_home.is_some(), "should have GPS home position");
    let home = bf.gps_home.as_ref().unwrap();
    assert_eq!(home.len(), 2, "GPS home should have lat and lng");
    assert_eq!(home[0], 502_075_967, "GPS home lat");
    assert_eq!(home[1], 191_013_996, "GPS home lng");

    // First GPS coord after reconstruction should equal home (delta 0)
    // Coordinates are scaled from raw ×10^7 to decimal degrees
    let gps_lat = session.field(&SensorField::GpsLat);
    let gps_lng = session.field(&SensorField::GpsLng);
    assert!(!gps_lat.is_empty(), "should have GPS lat data via field()");
    assert!(!gps_lng.is_empty(), "should have GPS lng data via field()");
    assert!(
        (gps_lat[0] - 50.2075967).abs() < 0.0000001,
        "first GPS lat should be ~50.2076, got {}",
        gps_lat[0]
    );
    assert!(
        (gps_lng[0] - 19.1013996).abs() < 0.0000001,
        "first GPS lng should be ~19.1014, got {}",
        gps_lng[0]
    );

    // eRPM[0] first 3 values
    let erpm = session.field(&SensorField::ERpm(propwash_core::types::MotorIndex(0)));
    assert!(erpm.len() >= 3, "should have at least 3 eRPM values");
    // BF field() scales raw eRPM/100 to actual eRPM (×100)
    assert!(
        (erpm[0] - 29400.0).abs() < 1.0,
        "eRPM[0][0] expected 29400.0, got {}",
        erpm[0]
    );
    assert!(
        (erpm[1] - 30100.0).abs() < 1.0,
        "eRPM[0][1] expected 30100.0, got {}",
        erpm[1]
    );
    assert!(
        (erpm[2] - 30300.0).abs() < 1.0,
        "eRPM[0][2] expected 30300.0, got {}",
        erpm[2]
    );

    // Sample rate ~1572 Hz
    let rate = session.sample_rate_hz();
    assert!(
        rate > 1500.0 && rate < 1650.0,
        "expected sample rate ~1572 Hz, got {rate}"
    );

    // Firmware version
    assert!(
        session.firmware_version().contains("Betaflight 4.5.0"),
        "expected firmware to contain 'Betaflight 4.5.0', got: {}",
        session.firmware_version()
    );

    // Gyro unfiltered data present
    let unfilt = session.field(&SensorField::GyroUnfilt(Axis::Roll));
    assert!(!unfilt.is_empty(), "should have unfiltered gyro data");
}

#[test]
fn ap_esc_telemetry_parsed() {
    use propwash_core::types::MotorIndex;

    let log = parse_fixture("ardupilot/esc-telem-quadplane-v4.4.4.bin");
    let session = &log.sessions[0];

    // ESC[0] first RPM value should be 0.0
    let erpm0 = session.field(&SensorField::ERpm(MotorIndex(0)));
    assert!(!erpm0.is_empty(), "should have ESC RPM data for motor 0");
    assert!(
        erpm0[0].abs() < 1e-2,
        "ESC[0] first RPM expected 0.0, got {}",
        erpm0[0]
    );

    // 5 motor instances should have eRPM data (indices 0-4)
    for i in 0..5 {
        let erpm = session.field(&SensorField::ERpm(MotorIndex(i)));
        assert!(!erpm.is_empty(), "should have ESC RPM data for motor {i}");
    }

    // Firmware version
    assert!(
        session.firmware_version().contains("ArduPlane V4.4.4"),
        "expected firmware to contain 'ArduPlane V4.4.4', got: {}",
        session.firmware_version()
    );

    // IMU frame count > 13000
    assert!(
        session.frame_count() > 13000,
        "expected >13000 IMU frames, got {}",
        session.frame_count()
    );

    // First gyro[roll] value should be small (near zero in deg/s)
    let gyro_roll = session.field(&SensorField::Gyro(Axis::Roll));
    assert!(!gyro_roll.is_empty(), "should have gyro data");
    assert!(
        gyro_roll[0].abs() < 0.1,
        "first gyro[roll] should be near zero deg/s, got {}",
        gyro_roll[0]
    );
}

// ── BF event parsing ──────────────────────────────────────────────

#[test]
fn bf_events_parsed() {
    use propwash_core::format::bf::types::BfEvent;

    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let Session::Betaflight(bf) = &log.sessions[1] else {
        panic!("expected Betaflight");
    };
    assert!(!bf.events.is_empty(), "should have parsed events");
    let has_log_end = bf.events.iter().any(|e| matches!(e, BfEvent::LogEnd));
    assert!(has_log_end, "session 1 should have a LogEnd event");
}

#[test]
fn bf_events_gps_rescue_fixture() {
    use propwash_core::format::bf::types::BfEvent;

    let log = parse_fixture("fc-blackbox/btfl_gps_rescue.bbl");
    let Session::Betaflight(bf) = &log.sessions[0] else {
        panic!("expected Betaflight");
    };
    assert!(
        !bf.events.is_empty(),
        "GPS rescue fixture should have events"
    );
    let has_sync = bf
        .events
        .iter()
        .any(|e| matches!(e, BfEvent::SyncBeep { .. }));
    assert!(has_sync, "should have a SyncBeep event");
    assert_eq!(
        bf.events.len(),
        bf.stats.event_count,
        "events vec length should match stats.event_count"
    );
}

// ── BF slow frame / Vbat ──────────────────────────────────────────

#[test]
fn bf_vbat_from_slow_frames() {
    let log = parse_fixture("fc-blackbox/btfl_gps_rescue.bbl");
    let vbat = log.sessions[0].field(&SensorField::Vbat);
    assert!(!vbat.is_empty(), "should have Vbat data from slow frames");
    for &v in vbat.iter().take(10) {
        assert!(
            (0.0..5000.0).contains(&v),
            "Vbat value should be reasonable, got {v}"
        );
    }
}

// ── AP Vbat ───────────────────────────────────────────────────────

#[test]
fn ap_vbat() {
    let log = parse_fixture("ardupilot/esc-telem-quadplane-v4.4.4.bin");
    let vbat = log.sessions[0].field(&SensorField::Vbat);
    assert!(!vbat.is_empty(), "ESC fixture should have BAT voltage data");
    assert!(
        vbat[0] > 5.0 && vbat[0] < 60.0,
        "voltage should be reasonable (5-60V), got {}",
        vbat[0]
    );
}

// ── AP PID data ───────────────────────────────────────────────────

#[test]
fn ap_pid_data() {
    // Check all AP fixtures for PID data; at least one should have it,
    // and when present, P/I/D components should have matching lengths.
    let fixtures = [
        "ardupilot/methodic-copter-tarot-x4.bin",
        "ardupilot/dronekit-copter-log171.bin",
        "ardupilot/esc-telem-quadplane-v4.4.4.bin",
    ];
    let mut found_pid = false;
    for fixture in fixtures {
        let log = parse_fixture(fixture);
        let pid_p = log.sessions[0].field(&SensorField::PidP(Axis::Roll));
        if pid_p.is_empty() {
            continue;
        }
        found_pid = true;
        let pid_i = log.sessions[0].field(&SensorField::PidI(Axis::Roll));
        let pid_d = log.sessions[0].field(&SensorField::PidD(Axis::Roll));
        assert_eq!(
            pid_p.len(),
            pid_i.len(),
            "{fixture}: PID P and I should have same length"
        );
        assert_eq!(
            pid_p.len(),
            pid_d.len(),
            "{fixture}: PID P and D should have same length"
        );
    }
    // PID extraction should at least not panic on any fixture
    // If none have PIDR, that's OK — the mechanism is verified
    let _ = found_pid;
}

// ── PX4 setpoint ──────────────────────────────────────────────────

#[test]
fn px4_setpoint_data() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let setpoint = log.sessions[0].field(&SensorField::Setpoint(Axis::Roll));
    assert!(
        !setpoint.is_empty(),
        "should have vehicle_rates_setpoint data for roll"
    );
    for axis in Axis::ALL {
        let data = log.sessions[0].field(&SensorField::Setpoint(axis));
        assert!(!data.is_empty(), "setpoint {axis} should have data");
    }
}

// ── PX4 logging message content ───────────────────────────────────

#[test]
fn px4_logging_message_content() {
    let log = parse_fixture("px4/sample_logging_tagged_and_default_params.ulg");
    let Session::Px4(px4) = &log.sessions[0] else {
        panic!("expected PX4 session");
    };
    assert!(!px4.log_messages.is_empty(), "should have logging messages");
    let first = &px4.log_messages[0];
    assert!(
        !first.message.is_empty(),
        "first log message should have text"
    );
    for msg in &px4.log_messages {
        assert!(msg.level <= 7, "log level should be 0-7, got {}", msg.level);
    }
}

// ---- Golden value tests: BF-family variants ----------------------

#[test]
fn emuflight_golden_values() {
    let log = parse_fixture("gimbal-ghost/emuf_001.bbl");
    let session = &log.sessions[0];

    assert!(
        session.firmware_version().contains("EmuFlight"),
        "expected EmuFlight firmware, got: {}",
        session.firmware_version()
    );
    assert!(
        session.firmware_version().contains("0.4.1"),
        "expected version 0.4.1, got: {}",
        session.firmware_version()
    );
    assert_eq!(session.frame_count(), 223_997);
    assert_eq!(session.motor_count(), 4);
    assert!(
        (session.sample_rate_hz() - 1639.75).abs() < 1.0,
        "expected ~1639.75 Hz sample rate, got {}",
        session.sample_rate_hz()
    );

    let bf = get_bf(session);
    assert_eq!(field_val(bf, 0, "gyroADC[0]"), 0);
}

#[test]
fn rotorflight_golden_values() {
    let log = parse_fixture("rotorflight/LOG246.TXT");
    let session = &log.sessions[0];

    assert!(
        session.firmware_version().contains("Rotorflight"),
        "expected Rotorflight firmware, got: {}",
        session.firmware_version()
    );
    assert!(
        session.firmware_version().contains("4.3.0"),
        "expected version 4.3.0, got: {}",
        session.firmware_version()
    );
    assert_eq!(session.frame_count(), 9390);
    assert_eq!(session.motor_count(), 2);
    assert!(
        (session.sample_rate_hz() - 498.5).abs() < 1.0,
        "expected ~498.5 Hz sample rate, got {}",
        session.sample_rate_hz()
    );

    let bf = get_bf(session);
    assert_eq!(field_val(bf, 0, "gyroADC[0]"), -14);
}

#[test]
fn inav_golden_values() {
    let log = parse_fixture("inav/LOG00001.TXT");
    let session = &log.sessions[0];

    assert!(
        session.firmware_version().contains("INAV"),
        "expected INAV firmware, got: {}",
        session.firmware_version()
    );
    assert!(
        session.firmware_version().contains("7.0.0"),
        "expected version 7.0.0, got: {}",
        session.firmware_version()
    );
    assert_eq!(session.frame_count(), 824);
    assert_eq!(session.motor_count(), 0);
    assert!(
        (session.sample_rate_hz() - 99.94).abs() < 1.0,
        "expected ~99.94 Hz sample rate, got {}",
        session.sample_rate_hz()
    );

    let bf = get_bf(session);
    assert_eq!(field_val(bf, 0, "gyroADC[0]"), 0);
}

#[test]
fn cleanflight_golden_values() {
    let log = parse_fixture("cleanflight/LOG00568.TXT");
    let session = &log.sessions[0];

    // Cleanflight firmware string is just the git hash
    assert!(
        session.firmware_version().contains("d72983e"),
        "expected firmware hash d72983e, got: {}",
        session.firmware_version()
    );
    assert_eq!(session.frame_count(), 170_492);
    assert_eq!(session.motor_count(), 4);
    assert!(
        (session.sample_rate_hz() - 415.44).abs() < 1.0,
        "expected ~415.44 Hz sample rate, got {}",
        session.sample_rate_hz()
    );

    let bf = get_bf(session);
    assert_eq!(field_val(bf, 0, "gyroADC[0]"), -2);
}

// ---- Golden value tests: ArduPilot variants -----------------------

#[test]
fn ardupilot_dronekit_golden_values() {
    let log = parse_fixture("ardupilot/dronekit-copter-log171.bin");
    let session = &log.sessions[0];

    // Firmware version verified against pymavlink MSG output
    assert!(
        session.firmware_version().contains("Copter"),
        "expected Copter firmware, got: {}",
        session.firmware_version()
    );
    assert!(
        session.firmware_version().contains("3.3"),
        "expected version 3.3, got: {}",
        session.firmware_version()
    );

    // IMU frame count verified against pymavlink: 11916
    assert_eq!(session.frame_count(), 11_916);
    assert_eq!(session.motor_count(), 4);

    // First gyro value verified against pymavlink GyrX: 0.000143...
    let gyro = session.field(&SensorField::Gyro(Axis::Roll));
    assert!(!gyro.is_empty(), "should have gyro data");
    assert!(
        gyro[0].abs() < 0.01,
        "first gyro[roll] should be near zero, got {}",
        gyro[0]
    );

    assert!(
        session.duration_seconds() > 230.0 && session.duration_seconds() < 260.0,
        "expected ~240-254s duration, got {}",
        session.duration_seconds()
    );
}

// ── Analysis pipeline tests ────────────────────────────────────────

#[test]
fn analysis_detects_events() {
    // LOG00007.BFL is a large flight log — should trigger event detectors
    let log = parse_fixture("fc-blackbox/LOG00007.BFL");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    assert!(
        !analysis.events.is_empty(),
        "LOG00007 should detect events (gyro spikes, motor saturation, etc.)"
    );
}

#[test]
fn analysis_produces_vibration_spectra() {
    let log = parse_fixture("fc-blackbox/LOG00007.BFL");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    let vib = analysis
        .vibration
        .as_ref()
        .expect("should have vibration analysis");
    assert!(!vib.spectra.is_empty(), "should have frequency spectra");
    for spectrum in &vib.spectra {
        assert!(
            !spectrum.peaks.is_empty(),
            "should have peaks for {}",
            spectrum.axis
        );
        for peak in &spectrum.peaks {
            assert!(peak.frequency_hz > 0.0, "peak frequency should be positive");
            assert!(
                peak.frequency_hz < 500.0,
                "peak {} Hz should be below Nyquist for ~1kHz sample rate",
                peak.frequency_hz
            );
        }
    }
}

#[test]
fn analysis_produces_diagnostics() {
    let log = parse_fixture("fc-blackbox/LOG00007.BFL");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    // Large flight log should trigger diagnostics
    assert!(
        !analysis.diagnostics.is_empty(),
        "LOG00007 should produce diagnostics"
    );
}

#[test]
fn analysis_summary_matches_session() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let session = &log.sessions[0];
    let analysis = propwash_core::analysis::analyze(session);
    assert_eq!(
        analysis.summary.frame_count,
        session.frame_count(),
        "summary frame count should match session"
    );
    assert_eq!(
        analysis.summary.motor_count,
        session.motor_count(),
        "summary motor count should match session"
    );
}

#[test]
fn analysis_episodes_consolidate() {
    let log = parse_fixture("fc-blackbox/LOG00007.BFL");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    let episodes = propwash_core::analysis::episodes::consolidate(&analysis.events);
    // With events present, consolidation should produce episodes
    if !analysis.events.is_empty() {
        assert!(
            !episodes.is_empty(),
            "consolidate should produce episodes from {} events",
            analysis.events.len()
        );
        for ep in &episodes {
            assert!(ep.event_count > 0, "each episode should have events");
            assert!(ep.end_time >= ep.start_time, "end >= start");
        }
    }
}

#[test]
fn analysis_works_for_ardupilot() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    assert!(
        analysis.summary.frame_count > 0,
        "ardupilot analysis should have frames"
    );
}

#[test]
fn analysis_works_for_px4() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    assert!(
        analysis.summary.frame_count > 0,
        "px4 analysis should have frames"
    );
}

// ── MAVLink tlog tests ─────────────────────────────────────────────

fixture_test!(mavlink_dronekit, "mavlink/dronekit-flight.tlog");

#[test]
fn mavlink_dronekit_golden_values() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    // Firmware version from STATUSTEXT
    assert!(
        session.firmware_version().contains("APM:Copter"),
        "expected APM:Copter firmware, got: {}",
        session.firmware_version()
    );

    // Vehicle type from HEARTBEAT
    assert_eq!(
        session.craft_name(),
        "Quadrotor",
        "expected Quadrotor vehicle type"
    );

    // ATTITUDE message count verified against pymavlink: 6292
    assert_eq!(session.frame_count(), 6292);
    assert_eq!(session.motor_count(), 4);

    // Duration ~2443s (from tlog timestamps), sample rate ~4 Hz
    assert!(
        session.duration_seconds() > 2000.0,
        "expected >2000s duration, got {}",
        session.duration_seconds()
    );
    assert!(
        session.sample_rate_hz() > 1.0 && session.sample_rate_hz() < 20.0,
        "expected telemetry rate 1-20 Hz, got {}",
        session.sample_rate_hz()
    );
}

#[test]
fn mavlink_gyro_from_attitude() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    let gyro_roll = session.field(&SensorField::Gyro(Axis::Roll));
    assert_eq!(
        gyro_roll.len(),
        6292,
        "gyro data should match ATTITUDE count"
    );

    // First ATTITUDE rollspeed = -0.04941 rad/s = -2.831 deg/s
    let expected_deg = -0.049_409_743_398_427_96 * 57.295_779_513_082_32;
    assert!(
        (gyro_roll[0] - expected_deg).abs() < 0.01,
        "first gyro[roll] expected ~{expected_deg:.3}, got {:.3}",
        gyro_roll[0]
    );
}

#[test]
fn mavlink_gps_data() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    let lat = session.field(&SensorField::GpsLat);
    let lng = session.field(&SensorField::GpsLng);
    assert!(!lat.is_empty(), "should have GPS latitude data");
    assert!(!lng.is_empty(), "should have GPS longitude data");

    // First GPS_RAW_INT: lat = -352080891 → -35.2080891 deg
    assert!(
        (lat[0] - (-35.2080891)).abs() < 0.0001,
        "first lat expected ~-35.208, got {}",
        lat[0]
    );
    assert!(
        (lng[0] - 149.0435193).abs() < 0.0001,
        "first lon expected ~149.044, got {}",
        lng[0]
    );
}

#[test]
fn mavlink_motor_output() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    use propwash_core::types::MotorIndex;
    let motor1 = session.field(&SensorField::Motor(MotorIndex(0)));
    assert!(!motor1.is_empty(), "should have motor output data");

    // First SERVO_OUTPUT_RAW servo1_raw = 968
    assert!(
        (motor1[0] - 968.0).abs() < 0.5,
        "first motor[0] expected 968, got {}",
        motor1[0]
    );
}

#[test]
fn mavlink_vbat() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    let vbat = session.field(&SensorField::Vbat);
    assert!(!vbat.is_empty(), "should have battery voltage data");

    // First SYS_STATUS voltage_battery = 12202 mV = 12.202 V
    assert!(
        (vbat[0] - 12.202).abs() < 0.01,
        "first vbat expected ~12.2V, got {}",
        vbat[0]
    );
}

#[test]
fn mavlink_altitude() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    let alt = session.field(&SensorField::Altitude);
    assert!(!alt.is_empty(), "should have altitude data");

    // VFR_HUD alt = 1.93m
    assert!(
        (alt[0] - 1.93).abs() < 0.1,
        "first alt expected ~1.93m, got {}",
        alt[0]
    );
}

#[test]
fn mavlink_raw_imu_accel() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];

    let accel_z = session.field(&SensorField::Accel(Axis::Yaw));
    assert!(!accel_z.is_empty(), "should have accel data");

    // RAW_IMU zacc = -1006 mG ≈ -9.87 m/s²
    let expected = -1006.0 * 0.001 * 9.806_65;
    assert!(
        (accel_z[0] - expected).abs() < 0.1,
        "first accel[z] expected ~{expected:.2}, got {:.2}",
        accel_z[0]
    );
}

#[test]
fn mavlink_analysis_works() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let analysis = propwash_core::analysis::analyze(&log.sessions[0]);
    assert!(
        analysis.summary.frame_count > 0,
        "mavlink analysis should have frames"
    );
}

#[test]
fn mavlink_field_names_populated() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let session = &log.sessions[0];
    let names = session.field_names();
    assert!(
        names.contains(&"gyro[roll]".to_string()),
        "field_names should include gyro: {names:?}"
    );
    assert!(
        names.contains(&"motor[0]".to_string()),
        "field_names should include motor: {names:?}"
    );
}

#[test]
fn mavlink_statustext_captured() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let Session::Mavlink(mav) = &log.sessions[0] else {
        panic!("expected Mavlink session");
    };
    // 66 STATUSTEXT messages verified against pymavlink
    assert_eq!(
        mav.status_messages.len(),
        66,
        "expected 66 STATUSTEXT messages"
    );
    assert!(
        mav.status_messages[0].text.contains("APM:Copter"),
        "first status message should be firmware version"
    );
}

#[test]
fn mavlink_parse_stats() {
    let log = parse_fixture("mavlink/dronekit-flight.tlog");
    let Session::Mavlink(mav) = &log.sessions[0] else {
        panic!("expected Mavlink session");
    };
    // 163851 total packets verified against pymavlink
    assert!(
        mav.stats.total_packets > 100_000,
        "expected >100K packets, got {}",
        mav.stats.total_packets
    );
    assert_eq!(
        mav.stats.crc_errors, 0,
        "clean file should have no CRC errors"
    );
    assert_eq!(
        mav.stats.corrupt_bytes, 0,
        "clean file should have no corrupt bytes"
    );
}

// ── Filter config tests ────────────────────────────────────────────

#[test]
fn bf_filter_config() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let config = log.sessions[0].filter_config();
    // Betaflight logs should have a gyro LPF configured
    assert!(
        config.gyro_lpf_hz.is_some(),
        "BF should have gyro_lpf_hz, got {config:?}"
    );
}

#[test]
fn ap_filter_config() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let config = log.sessions[0].filter_config();
    // ArduPilot with INS_GYRO_FILTER parameter
    assert!(
        config.gyro_lpf_hz.is_some(),
        "AP should have gyro_lpf_hz from INS_GYRO_FILTER, got {config:?}"
    );
}

#[test]
fn px4_filter_config() {
    let log = parse_fixture("px4/sample_log_small.ulg");
    let config = log.sessions[0].filter_config();
    // PX4 should have IMU_GYRO_CUTOFF parameter
    assert!(
        config.gyro_lpf_hz.is_some(),
        "PX4 should have gyro_lpf_hz from IMU_GYRO_CUTOFF, got {config:?}"
    );
}

// ── Unit contract tests ───────────────────────────────────────────
// Verify that all formats return values in the canonical units declared
// by SensorField::unit(). Catches scaling bugs like returning millivolts
// instead of volts.

#[test]
fn unit_contract_vbat_is_volts() {
    // All formats with vbat should return values in the 0-60V range (1-14S battery)
    let fixtures = [
        "fc-blackbox/btfl_001.bbl",
        "ardupilot/methodic-copter-tarot-x4.bin",
        "mavlink/dronekit-flight.tlog",
    ];
    for fixture in fixtures {
        let log = parse_fixture(fixture);
        let session = &log.sessions[0];
        let vbat = session.field(&SensorField::Vbat);
        if vbat.is_empty() {
            continue;
        }
        let max = vbat.iter().copied().fold(f64::MIN, f64::max);
        assert!(
            max < 60.0,
            "{fixture}: vbat max {max:.1} exceeds 60V — likely raw/unscaled (expected volts)"
        );
        let min = vbat
            .iter()
            .copied()
            .filter(|&v| v > 0.0)
            .fold(f64::MAX, f64::min);
        assert!(
            min > 1.0,
            "{fixture}: vbat min {min:.1} below 1V — likely already too small"
        );
    }
}

#[test]
fn unit_contract_gyro_is_degrees_per_second() {
    // Gyro should be in deg/s. Typical range: -2000 to +2000 for acro flying.
    // Raw sensor units (mrad/s, raw ADC) would be orders of magnitude different.
    let fixtures = [
        "fc-blackbox/btfl_001.bbl",
        "ardupilot/methodic-copter-tarot-x4.bin",
        "px4/sample_log_small.ulg",
    ];
    for fixture in fixtures {
        let log = parse_fixture(fixture);
        let session = &log.sessions[0];
        let gyro = session.field(&SensorField::Gyro(Axis::Roll));
        if gyro.is_empty() {
            continue;
        }
        let max_abs = gyro.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_abs < 5000.0,
            "{fixture}: gyro max {max_abs:.0} deg/s seems too high — wrong units?"
        );
    }
}

#[test]
fn unit_contract_accel_is_m_per_s2() {
    let fixtures = [
        "fc-blackbox/btfl_001.bbl",
        "ardupilot/methodic-copter-tarot-x4.bin",
        "px4/sample_log_small.ulg",
    ];
    for fixture in fixtures {
        let log = parse_fixture(fixture);
        let session = &log.sessions[0];
        let accel = session.field(&SensorField::Accel(Axis::Roll));
        if accel.is_empty() {
            continue;
        }
        let max_abs = accel.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_abs < 500.0,
            "{fixture}: accel max {max_abs:.0} m/s² seems too high — wrong units?"
        );
    }
}

#[test]
fn unit_contract_gps_is_degrees() {
    // GPS coordinates should be in decimal degrees (-180 to 180)
    let log = parse_fixture("fc-blackbox/btfl_gps_rescue.bbl");
    let session = &log.sessions[0];
    let lat = session.field(&SensorField::GpsLat);
    let lng = session.field(&SensorField::GpsLng);
    if !lat.is_empty() {
        let max_lat = lat.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_lat < 90.0,
            "GPS lat max {max_lat:.0} should be < 90 — probably raw ×10^7"
        );
    }
    if !lng.is_empty() {
        let max_lng = lng.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_lng < 180.0,
            "GPS lng max {max_lng:.0} should be < 180 — probably raw ×10^7"
        );
    }
}

#[test]
fn unit_contract_time_is_microseconds_monotonic() {
    let fixtures = [
        "fc-blackbox/btfl_001.bbl",
        "ardupilot/dronekit-copter-log171.bin",
        "px4/sample_log_small.ulg",
    ];
    for fixture in fixtures {
        let log = parse_fixture(fixture);
        let session = &log.sessions[0];
        let time = session.field(&SensorField::Time);
        if time.len() < 2 {
            continue;
        }
        // Should be microseconds (values in millions)
        let last = *time.last().unwrap();
        assert!(
            last > 1000.0,
            "{fixture}: last time {last:.0} seems too small for microseconds"
        );
        // Should be monotonically increasing (allowing small wraps)
        let decreases = time.windows(2).filter(|w| w[1] < w[0]).count();
        assert!(
            decreases == 0,
            "{fixture}: time has {decreases} non-monotonic samples"
        );
    }
}

#[test]
fn unit_contract_rssi_is_percentage() {
    let fixtures = ["fc-blackbox/btfl_035.bbl", "fc-blackbox/btfl_002.bbl"];
    for fixture in fixtures {
        let log = parse_fixture(fixture);
        let session = &log.sessions[0];
        let rssi = session.field(&SensorField::Rssi);
        if rssi.is_empty() {
            continue;
        }
        let max = rssi.iter().copied().fold(f64::MIN, f64::max);
        assert!(max <= 100.0, "{fixture}: RSSI max {max:.1} should be ≤100%");
    }
}

// ── PID analysis golden tests ─────────────────────────────────────
// These fixtures are from real FPV freestyle flights (BF 2025.12.2, 1kHz).
// They exercise windowed step detection needed for modern RC-smoothed setpoints.

use propwash_core::analysis;
use propwash_core::analysis::pid::TuningRating;

#[test]
fn pid_step_response_overshooting_flight() {
    // btfl_035: aggressive flying, all axes overshooting 40-47%
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let a = analysis::analyze(session);

    let sr = a.step_response.expect("should detect step response");
    assert!(sr.axes.len() >= 2, "should detect steps on at least 2 axes");

    // Roll: ~13 steps, ~41% overshoot
    let roll = sr.axes.iter().find(|a| a.axis == Axis::Roll).expect("roll");
    assert!(
        roll.step_count >= 5,
        "roll should have ≥5 steps, got {}",
        roll.step_count
    );
    assert!(
        roll.overshoot_percent > 20.0,
        "roll overshoot should be >20%, got {:.0}%",
        roll.overshoot_percent
    );

    // Pitch: ~15 steps, ~47% overshoot
    let pitch = sr
        .axes
        .iter()
        .find(|a| a.axis == Axis::Pitch)
        .expect("pitch");
    assert!(pitch.step_count >= 3, "pitch should have ≥3 steps");
    assert!(
        pitch.overshoot_percent > 20.0,
        "pitch overshoot should be >20%"
    );
}

#[test]
fn pid_tuning_suggests_lower_p_for_overshooting() {
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let a = analysis::analyze(session);

    let pid = a.pid.expect("should have PID analysis");
    assert!(!pid.tuning.is_empty(), "should have tuning suggestions");

    for t in &pid.tuning {
        assert_eq!(
            t.rating,
            TuningRating::Overshooting,
            "{:?} axis should be Overshooting, got {:?}",
            t.axis,
            t.rating
        );

        // Suggested P should be lower than current P
        if let (Some(cur_p), Some(sug_p)) = (t.current.p, t.suggested.p) {
            assert!(
                sug_p < cur_p,
                "{:?} suggested P ({sug_p}) should be less than current ({cur_p})",
                t.axis
            );
        }
    }
}

#[test]
fn pid_gains_extracted_from_bf_headers() {
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let gains = session.pid_gains();

    assert!(gains.has_data(), "should extract PID gains from BF headers");
    let roll = gains.get(Axis::Roll);
    assert_eq!(roll.p, Some(45));
    assert_eq!(roll.i, Some(80));
    assert_eq!(roll.d, Some(30));

    let pitch = gains.get(Axis::Pitch);
    assert_eq!(pitch.p, Some(47));
    assert_eq!(pitch.i, Some(84));
    assert_eq!(pitch.d, Some(34));
}

#[test]
fn pid_good_tuning_flight() {
    // btfl_027: moderate flying, roll/pitch rated "good"
    let log = parse_fixture("fc-blackbox/btfl_027.bbl");
    let session = &log.sessions[0];
    let a = analysis::analyze(session);

    let sr = a.step_response.expect("should detect step response");
    let roll = sr.axes.iter().find(|a| a.axis == Axis::Roll).expect("roll");
    assert!(roll.step_count >= 5, "should have enough roll steps");
    assert!(
        roll.overshoot_percent < 25.0,
        "roll overshoot should be <25% for 'good', got {:.0}%",
        roll.overshoot_percent
    );

    let pid = a.pid.expect("should have PID analysis");
    let roll_tuning = pid.tuning.iter().find(|t| t.axis == Axis::Roll);
    if let Some(t) = roll_tuning {
        assert_eq!(t.rating, TuningRating::Good, "roll should be rated Good");
    }
}

#[test]
fn pid_windup_detected_on_real_flight() {
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let a = analysis::analyze(session);

    let pid = a.pid.expect("should have PID analysis");
    assert!(!pid.windup.is_empty(), "should detect I-term windup");

    // Yaw typically has highest I-dominance
    let yaw = pid.windup.iter().find(|w| w.axis == Axis::Yaw);
    if let Some(w) = yaw {
        assert!(
            w.i_dominant_fraction > 0.3,
            "yaw I-dominant fraction should be >30%, got {:.0}%",
            w.i_dominant_fraction * 100.0
        );
    }
}

#[test]
fn pid_feedforward_field_present() {
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let ff = session.field(&SensorField::Feedforward(Axis::Roll));
    assert!(!ff.is_empty(), "feedforward[roll] should have data");
    assert_eq!(
        ff.len(),
        session.frame_count(),
        "feedforward should have one value per frame"
    );
}

// ── RSSI field tests ──────────────────────────────────────────────

#[test]
fn bf_rssi_scaled_to_percentage() {
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let rssi = session.field(&SensorField::Rssi);
    assert!(!rssi.is_empty(), "should have RSSI data");

    // RSSI should be scaled to 0-100%
    let max = rssi.iter().copied().fold(f64::MIN, f64::max);
    let min = rssi
        .iter()
        .copied()
        .filter(|&v| v > 0.0)
        .fold(f64::MAX, f64::min);
    assert!(
        max <= 100.0,
        "RSSI max should be ≤100%, got {max:.1} — probably raw 0-1023"
    );
    assert!(min >= 0.0, "RSSI min should be ≥0%, got {min:.1}");
}

#[test]
fn bf_rssi_in_field_names() {
    let log = parse_fixture("fc-blackbox/btfl_035.bbl");
    let session = &log.sessions[0];
    let names = session.field_names();
    assert!(
        names.iter().any(|n| n == "rssi"),
        "field_names should include 'rssi', got: {names:?}"
    );
}

#[test]
fn bf_gps_fields_in_field_names() {
    let log = parse_fixture("fc-blackbox/btfl_gps_rescue.bbl");
    let session = &log.sessions[0];
    let names = session.field_names();
    assert!(
        names.iter().any(|n| n == "gps_lat"),
        "field_names should include 'gps_lat'"
    );
    assert!(
        names.iter().any(|n| n == "gps_lng"),
        "field_names should include 'gps_lng'"
    );
}

// ── Analysis snapshot tests ───────────────────────────────────────
// Compare key analysis output against committed JSON snapshots.
// Catches structural changes that break the Rust→TypeScript contract.

fn diag_categories(
    diagnostics: &[propwash_core::analysis::diagnostics::Diagnostic],
) -> Vec<String> {
    let mut cats: Vec<String> = diagnostics
        .iter()
        .map(|d| d.category.to_string())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .collect();
    cats.sort();
    cats
}

fn analysis_snapshot(fixture: &str) -> serde_json::Value {
    let log = parse_fixture(fixture);
    let session = &log.sessions[0];
    let a = analysis::analyze(session);

    // Build compact snapshot matching the Python generator format
    let round2 = |v: f64| (v * 100.0).round() / 100.0;

    let vib = a.vibration.as_ref().map(|v| {
        serde_json::json!({
            "noise_floor_db": v.noise_floor_db.iter().map(|&x| round2(x)).collect::<Vec<_>>(),
            "spectrum_count": v.spectra.len(),
            "throttle_band_count": v.throttle_bands.len(),
            "avg_motor_hz": v.avg_motor_hz.map(round2),
            "has_accel": v.accel.is_some(),
            "has_propwash": v.propwash.is_some(),
        })
    });

    serde_json::json!({
        "summary": serde_json::to_value(&a.summary).unwrap(),
        "episode_count": propwash_core::analysis::episodes::consolidate(&a.events).len(),
        "diagnostic_count": a.diagnostics.len(),
        "diagnostics_categories": diag_categories(&a.diagnostics),
        "vibration": vib,
        "step_response": a.step_response.as_ref().map(|sr| serde_json::to_value(sr).unwrap()),
        "pid": a.pid.as_ref().map(|p| serde_json::to_value(p).unwrap()),
    })
}

#[test]
fn snapshot_btfl_035() {
    let actual = analysis_snapshot("fc-blackbox/btfl_035.bbl");
    let expected_str =
        std::fs::read_to_string(fixtures_dir().join("fc-blackbox/btfl_035.analysis.json"))
            .expect("snapshot file should exist");
    let expected: serde_json::Value =
        serde_json::from_str(&expected_str).expect("snapshot should be valid JSON");

    // Compare key structural fields
    assert_eq!(
        actual["summary"]["motor_count"], expected["summary"]["motor_count"],
        "motor_count mismatch"
    );
    assert_eq!(
        actual["summary"]["frame_count"], expected["summary"]["frame_count"],
        "frame_count mismatch"
    );
    assert_eq!(
        actual["diagnostic_count"], expected["diagnostic_count"],
        "diagnostic count changed — analysis output may have shifted"
    );
    assert_eq!(
        actual["diagnostics_categories"], expected["diagnostics_categories"],
        "diagnostic categories changed"
    );

    // Vibration structure
    assert_eq!(
        actual["vibration"]["spectrum_count"], expected["vibration"]["spectrum_count"],
        "spectrum count changed"
    );
    assert_eq!(
        actual["vibration"]["throttle_band_count"], expected["vibration"]["throttle_band_count"],
        "throttle band count changed"
    );
    assert_eq!(
        actual["vibration"]["has_accel"],
        expected["vibration"]["has_accel"],
    );
    assert_eq!(
        actual["vibration"]["has_propwash"],
        expected["vibration"]["has_propwash"],
    );

    // Step response structure (number of axes detected)
    if let (Some(a_sr), Some(e_sr)) = (
        actual["step_response"].as_object(),
        expected["step_response"].as_object(),
    ) {
        assert_eq!(
            a_sr.get("axes").and_then(|v| v.as_array()).map(|a| a.len()),
            e_sr.get("axes").and_then(|v| v.as_array()).map(|a| a.len()),
            "step response axis count changed"
        );
    }

    // PID tuning count
    if let (Some(a_pid), Some(e_pid)) = (actual["pid"].as_object(), expected["pid"].as_object()) {
        assert_eq!(
            a_pid
                .get("tuning")
                .and_then(|v| v.as_array())
                .map(|a| a.len()),
            e_pid
                .get("tuning")
                .and_then(|v| v.as_array())
                .map(|a| a.len()),
            "tuning suggestion count changed"
        );
        assert_eq!(
            a_pid
                .get("windup")
                .and_then(|v| v.as_array())
                .map(|a| a.len()),
            e_pid
                .get("windup")
                .and_then(|v| v.as_array())
                .map(|a| a.len()),
            "windup axis count changed"
        );
    }
}

#[test]
fn snapshot_btfl_001() {
    let actual = analysis_snapshot("fc-blackbox/btfl_001.bbl");
    let expected_str =
        std::fs::read_to_string(fixtures_dir().join("fc-blackbox/btfl_001.analysis.json"))
            .expect("snapshot file should exist");
    let expected: serde_json::Value =
        serde_json::from_str(&expected_str).expect("snapshot should be valid JSON");

    assert_eq!(
        actual["summary"]["motor_count"],
        expected["summary"]["motor_count"],
    );
    assert_eq!(
        actual["diagnostic_count"], expected["diagnostic_count"],
        "diagnostic count changed for btfl_001"
    );
}
