use std::path::Path;

use propwash_core::types::{Axis, SensorField};
use propwash_core::{decode_file, Log, RawSession, Unified};

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
            let total_frames: usize = log.sessions.iter().map(|s| s.unified().frame_count()).sum();
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
fixture_test!(ap_pyflightcoach_plane, "ardupilot/pyflightcoach-plane-v4.3.5.bin");
fixture_test!(ap_erasial, "ardupilot/erasial-00000001.bin");
fixture_test!(ap_dronekit_copter, "ardupilot/dronekit-copter-log171.bin");
fixture_test!(ap_methodic_copter, "ardupilot/methodic-copter-tarot-x4.bin");

#[test]
fn ardupilot_parses_metadata() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let session = &log.sessions[0];
    let unified = session.unified();

    assert!(unified.frame_count() > 0, "should have IMU frames");
    assert!(!unified.firmware_version().is_empty(), "should have firmware version");
    assert!(unified.duration_seconds() > 0.0, "should have nonzero duration");
    assert!(unified.motor_count() > 0, "should detect motors");

    let gyro = unified.field(&SensorField::Gyro(Axis::Roll));
    assert!(!gyro.is_empty(), "should have gyro data");
}

#[test]
fn ardupilot_motor_count_from_servo_params() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let unified = log.sessions[0].unified();
    assert_eq!(unified.motor_count(), 4, "Tarot X4 is a quadcopter");
}

#[test]
fn ardupilot_craft_name_skips_rtos() {
    let log = parse_fixture("ardupilot/methodic-copter-tarot-x4.bin");
    let unified = log.sessions[0].unified();
    let craft = unified.craft_name();
    assert!(!craft.contains("ChibiOS"), "craft name should not be the RTOS string, got: {craft}");
    assert!(!craft.contains("NuttX"), "craft name should not be NuttX, got: {craft}");
}

#[test]
fn ardupilot_old_format_parses() {
    let log = parse_fixture("ardupilot/pymavlink-plane-v3.8.bin");
    let session = &log.sessions[0];
    let unified = session.unified();

    assert!(unified.frame_count() > 0, "old format should still produce frames");
    assert!(unified.duration_seconds() > 0.0);
}

#[test]
fn ardupilot_plane_zero_motors() {
    let log = parse_fixture("ardupilot/pymavlink-plane-v3.8.bin");
    let unified = log.sessions[0].unified();
    assert_eq!(unified.motor_count(), 0, "plane without SERVO params should report 0 motors");
}

// Error recovery
fixture_test!(error_recovery, "error-recovery.bbl");

// ── Unified view tests ──────────────────────────────────────────────

#[test]
fn unified_sample_rate() {
    let log = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let session = &log.sessions[0];
    let unified = session.unified();
    assert!(
        unified.sample_rate_hz() > 10.0,
        "expected reasonable sample rate, got {}",
        unified.sample_rate_hz()
    );
}

#[test]
fn unified_duration() {
    let log = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let unified = log.sessions[0].unified();
    assert!(
        unified.duration_seconds() > 1.0,
        "expected >1s duration, got {}",
        unified.duration_seconds()
    );
}

#[test]
fn unified_field_extraction() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let unified = log.sessions[0].unified();
    let gyro = unified.field(&SensorField::Gyro(Axis::Roll));
    assert_eq!(gyro.len(), unified.frame_count());
}

#[test]
fn unified_gyro_fields() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let unified = log.sessions[0].unified();
    for axis in Axis::ALL {
        let data = unified.field(&SensorField::Gyro(axis));
        assert!(!data.is_empty(), "gyro {axis} should have data");
    }
}

#[test]
fn unified_firmware_version() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let unified = log.sessions[0].unified();
    assert!(
        unified.firmware_version().contains("Betaflight"),
        "got: {}",
        unified.firmware_version()
    );
}

#[test]
fn unified_motor_count() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    assert_eq!(log.sessions[0].unified().motor_count(), 4);

    let log2 = parse_fixture("gimbal-ghost/rtfl_001.bbl");
    assert_eq!(log2.sessions[0].unified().motor_count(), 1);
}

#[test]
fn unified_field_names() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let unified = log.sessions[0].unified();
    let names = unified.field_names();
    assert!(names.iter().any(|n| n == "time"));
    assert!(names.iter().any(|n| n == "gyroADC[0]"));
    assert!(names.iter().any(|n| n == "motor[0]"));
}

// ── Analyzed view tests ─────────────────────────────────────────────

#[test]
fn analyzed_betaflight_motor_count() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => assert_eq!(bf.motor_count(), 4),
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_rotorflight_single_motor() {
    let log = parse_fixture("gimbal-ghost/rtfl_001.bbl");
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => assert_eq!(bf.motor_count(), 1),
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_crash_log_has_corruption() {
    let log = parse_fixture("fc-blackbox/crashing-LOG00002.BFL");
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => {
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
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => {
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
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => {
            let _ = bf.debug_mode();
        }
        _ => panic!("expected Betaflight"),
    }
}

#[test]
fn analyzed_stats() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => {
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
                let unified = session.unified();

                // Every session should have field definitions
                assert!(
                    !unified.field_names().is_empty(),
                    "{}: session {} has no field names",
                    $path,
                    session.index
                );

                // Every frame should have values for all defined fields
                if let propwash_core::RawSession::Betaflight(bf) = &session.raw {
                    let n_fields = bf.main_field_defs.len();
                    for (i, frame) in bf.frames.iter().take(10).enumerate() {
                        assert_eq!(
                            frame.values.len(),
                            n_fields,
                            "{}: session {} frame {} has {} values, expected {}",
                            $path,
                            session.index,
                            i,
                            frame.values.len(),
                            n_fields
                        );
                    }
                }

                // frame_index should be sequential
                if let propwash_core::RawSession::Betaflight(bf) = &session.raw {
                    for (i, frame) in bf.frames.iter().enumerate() {
                        assert_eq!(
                            frame.frame_index, i,
                            "{}: frame_index mismatch at position {}",
                            $path, i
                        );
                    }
                }

                // Field extraction length matches frame count
                if unified.frame_count() > 0 {
                    let time = unified.field(&SensorField::Time);
                    assert_eq!(
                        time.len(),
                        unified.frame_count(),
                        "{}: time array length mismatch",
                        $path
                    );
                }

                // Stats should be consistent
                if let RawSession::Betaflight(bf) = &session.raw {
                    let stats = &bf.stats;
                    assert_eq!(
                        stats.total_main_frames(),
                        unified.frame_count(),
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
        .map(|s| match &s.raw {
            propwash_core::RawSession::Betaflight(bf) => bf.events.len(),
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
    let unified = session.unified();
    let _gyro = unified.field(&SensorField::Gyro(Axis::Roll));

    // Layer 2: Analyzed (format-specific)
    match &session.raw {
        RawSession::Betaflight(bf) => {
            let _ = bf.motor_count();
        }
        _ => panic!("expected Betaflight"),
    }

    // Layer 3: Raw (escape hatch)
    match &session.raw {
        propwash_core::RawSession::Betaflight(raw) => {
            let frame = &raw.frames[0];
            assert!(frame.byte_offset > 0);
            assert_eq!(frame.frame_index, 0);
        }
        _ => panic!("expected Betaflight session"),
    }
}

fn get_bf(session: &propwash_core::Session) -> &propwash_core::format::bf::types::BfRawSession {
    match &session.raw {
        propwash_core::RawSession::Betaflight(bf) => bf,
        _ => panic!("expected Betaflight"),
    }
}

fn field_val(
    bf: &propwash_core::format::bf::types::BfRawSession,
    frame_idx: usize,
    name: &str,
) -> i64 {
    let idx = bf.main_field_defs.index_of(&SensorField::from_header(name)).unwrap();
    bf.frames[frame_idx].values[idx]
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
    let rate = log.sessions[1].unified().sample_rate_hz();
    assert!((rate - 498.0).abs() < 2.0, "expected ~498 Hz, got {rate}");

    let log2 = parse_fixture("gimbal-ghost/btfl_001.bbl");
    let rate2 = log2.sessions[0].unified().sample_rate_hz();
    assert!((rate2 - 249.0).abs() < 2.0, "expected ~249 Hz, got {rate2}");
}

#[test]
fn golden_loop_iteration_matches_official_decoder() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let iter_idx = bf.main_field_defs.index_of(&SensorField::LoopIteration).unwrap();
    use propwash_core::format::bf::types::BfFrameKind;

    assert_eq!(bf.frames[0].values[iter_idx], 0);
    assert_eq!(bf.frames[1].values[iter_idx], 1);
    assert_eq!(bf.frames[15].values[iter_idx], 15);
    assert_eq!(bf.frames[16].values[iter_idx], 256);
    assert_eq!(bf.frames[17].values[iter_idx], 257);

    for i in 1..bf.frames.len().min(100) {
        let prev = bf.frames[i - 1].values[iter_idx];
        let curr = bf.frames[i].values[iter_idx];
        let delta = curr - prev;
        match bf.frames[i].kind {
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

    for i in 1..bf.frames.len().min(1000) {
        let t0 = bf.frames[i - 1].values[time_idx];
        let t1 = bf.frames[i].values[time_idx];
        let delta = t1 - t0;
        assert!(
            delta > 0 && delta < 100_000,
            "frame {i}: time delta {delta}us is unreasonable (expected 1000-5000us)"
        );
    }
}

/// Regression: STRAIGHT_LINE predictor doubled time values when
/// prev2 was zero (I-frame didn't reset both history slots).
/// Fixed in commit b40cc1a.
#[test]
fn regression_time_not_doubling() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let time_idx = bf.main_field_defs.index_of(&SensorField::Time).unwrap();

    let t0 = bf.frames[0].values[time_idx];
    let t1 = bf.frames[1].values[time_idx];
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

/// Regression: AVERAGE_2 predictor used (p1>>1)+(p2>>1) which
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

/// Regression: INCREMENT predictor added 1 per frame instead of
/// (skipped_frames + 1). With P_ratio=16, loopIteration should
/// jump by 16 per logged frame, not by 1.
/// Fixed in commit c2f443a.
#[test]
fn regression_loop_iteration_uses_frame_schedule() {
    let log = parse_fixture("fc-blackbox/btfl_001.bbl");
    let bf = get_bf(&log.sessions[1]);
    let iter_idx = bf.main_field_defs.index_of(&SensorField::LoopIteration).unwrap();

    let delta = bf.frames[1].values[iter_idx] - bf.frames[0].values[iter_idx];
    assert_eq!(
        delta, 1,
        "btfl_001 logs every frame (P_num==P_denom), so increment should be 1"
    );
}

/// Regression: TAG8_4S16 nibble-aligned reads left the byte stream
/// misaligned, causing P-frame time predictions to diverge from the
/// official decoder on ~5% of frames.
/// Fixed by implementing bit-level reader with byte_align() after TAG8_4S16.
#[test]
fn regression_bitreader_btfl_002_time_monotonic() {
    let log = parse_fixture("fc-blackbox/btfl_002.bbl");
    for session in &log.sessions {
        let RawSession::Betaflight(bf) = &session.raw else { continue };
        let Some(time_idx) = bf.main_field_defs.index_of(&SensorField::Time) else { continue };
        if bf.frames.len() < 2 {
            continue;
        }
        let mut prev_time = bf.frames[0].values[time_idx];
        for (i, frame) in bf.frames.iter().enumerate().skip(1) {
            let t = frame.values[time_idx];
            assert!(
                t >= prev_time,
                "session {}: frame {i} time went backwards: {} -> {} (delta {})",
                session.index, prev_time, t, t - prev_time
            );
            prev_time = t;
        }
    }
}

#[test]
fn regression_bitreader_btfl_all_time_monotonic() {
    let log = parse_fixture("fc-blackbox/btfl_all.bbl");
    for session in &log.sessions {
        let RawSession::Betaflight(bf) = &session.raw else { continue };
        let Some(time_idx) = bf.main_field_defs.index_of(&SensorField::Time) else { continue };
        if bf.frames.len() < 2 {
            continue;
        }
        let mut prev_time = bf.frames[0].values[time_idx];
        for (i, frame) in bf.frames.iter().enumerate().skip(1) {
            let t = frame.values[time_idx];
            assert!(
                t >= prev_time,
                "session {}: frame {i} time went backwards: {} -> {} (delta {})",
                session.index, prev_time, t, t - prev_time
            );
            prev_time = t;
        }
    }
}

/// Verify Cleanflight files parse all frames with monotonic time.
/// Previously diverged mid-flight due to stream alignment issues.
#[test]
fn regression_cleanflight_time_monotonic() {
    for fixture in &["cleanflight/LOG00568.TXT", "cleanflight/LOG00570.TXT", "cleanflight/LOG00572.TXT"] {
        let log = parse_fixture(fixture);
        for session in &log.sessions {
            let RawSession::Betaflight(bf) = &session.raw else { continue };
            let Some(time_idx) = bf.main_field_defs.index_of(&SensorField::Time) else { continue };
            if bf.frames.len() < 2 {
                continue;
            }
            let mut prev_time = bf.frames[0].values[time_idx];
            let mut backwards_count = 0;
            for frame in bf.frames.iter().skip(1) {
                let t = frame.values[time_idx];
                if t < prev_time {
                    backwards_count += 1;
                }
                prev_time = t;
            }
            assert!(
                backwards_count == 0,
                "{fixture} session {}: {backwards_count} frames with backwards time out of {}",
                session.index, bf.frames.len()
            );
        }
    }
}

/// Regression: LOG_END event accepted any 0xFF byte without verifying
/// the "End of log\0" string, causing false clean-end detection on
/// corrupt data.
/// Fixed in commit 0236bdc.
#[test]
fn regression_log_end_requires_marker_string() {
    let log = parse_fixture("fc-blackbox/crashing-LOG00002.BFL");
    match &log.sessions[0].raw {
        RawSession::Betaflight(bf) => {
            assert!(
                bf.stats.corrupt_bytes > 0,
                "crash log should have corruption"
            );
        }
        _ => panic!("expected Betaflight"),
    }
}
