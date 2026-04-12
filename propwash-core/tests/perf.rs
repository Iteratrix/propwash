//! Performance regression tests.
//!
//! These tests are `#[ignore]` by default — run with:
//!   cargo test --release -p propwash-core --test perf -- --ignored
//!
//! They assert minimum parse throughput (MB/s) to catch regressions.
//! Thresholds are set generously below measured performance so they
//! pass on slower CI runners while still catching major regressions.

use std::path::PathBuf;
use std::time::Instant;

fn fixture_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(rel)
}

/// Parse a fixture file in release mode and return (MB/s, frame_count).
fn bench_parse(rel: &str) -> (f64, usize) {
    let path = fixture_path(rel);
    let data = std::fs::read(&path).unwrap_or_else(|e| {
        panic!("Failed to read fixture {}: {e}", path.display());
    });
    let size_mb = data.len() as f64 / 1_048_576.0;

    // Warm up
    let _ = propwash_core::decode(&data);

    // Best of 3 runs
    let mut best = std::time::Duration::MAX;
    let mut frames = 0;
    for _ in 0..3 {
        let start = Instant::now();
        let log = propwash_core::decode(&data).unwrap();
        let elapsed = start.elapsed();
        std::hint::black_box(&log);

        if elapsed < best {
            best = elapsed;
            frames = log.sessions.iter().map(|s| s.frame_count()).sum();
        }
    }

    let mb_s = size_mb / best.as_secs_f64();
    eprintln!(
        "  {rel}: {size_mb:.1}MB in {:.0}ms = {mb_s:.0} MB/s ({frames} frames)",
        best.as_millis()
    );
    (mb_s, frames)
}

// ── Betaflight ───────────────────────────────────────────────────────

#[test]
#[ignore]
fn perf_bf_btfl_001() {
    let (mb_s, frames) = bench_parse("fc-blackbox/btfl_001.bbl");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 20.0,
        "BF parse throughput {mb_s:.0} MB/s below floor of 20 MB/s"
    );
}

#[test]
#[ignore]
fn perf_bf_log00007() {
    let (mb_s, frames) = bench_parse("fc-blackbox/LOG00007.BFL");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 20.0,
        "BF parse throughput {mb_s:.0} MB/s below floor of 20 MB/s"
    );
}

// ── ArduPilot ────────────────────────────────────────────────────────

#[test]
#[ignore]
fn perf_ap_methodic() {
    let (mb_s, frames) = bench_parse("ardupilot/methodic-copter-tarot-x4.bin");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 50.0,
        "AP parse throughput {mb_s:.0} MB/s below floor of 50 MB/s"
    );
}

#[test]
#[ignore]
fn perf_ap_dronekit() {
    let (mb_s, frames) = bench_parse("ardupilot/dronekit-copter-log171.bin");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 50.0,
        "AP parse throughput {mb_s:.0} MB/s below floor of 50 MB/s"
    );
}

// ── PX4 ──────────────────────────────────────────────────────────────

#[test]
#[ignore]
fn perf_px4_sample() {
    let (mb_s, frames) = bench_parse("px4/sample_log_small.ulg");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 50.0,
        "PX4 parse throughput {mb_s:.0} MB/s below floor of 50 MB/s"
    );
}

#[test]
#[ignore]
fn perf_px4_tagged() {
    let (mb_s, frames) = bench_parse("px4/sample_logging_tagged_and_default_params.ulg");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 50.0,
        "PX4 parse throughput {mb_s:.0} MB/s below floor of 50 MB/s"
    );
}

// ── MAVLink ─────────────────────────────────────────────────────────

#[test]
#[ignore]
fn perf_mavlink_dronekit() {
    let (mb_s, frames) = bench_parse("mavlink/dronekit-flight.tlog");
    assert!(frames > 0, "should parse frames");
    assert!(
        mb_s > 20.0,
        "MAVLink parse throughput {mb_s:.0} MB/s below floor of 20 MB/s"
    );
}

// ── Field extraction ─────────────────────────────────────────────────

#[test]
#[ignore]
fn perf_field_extraction_all_formats() {
    use propwash_core::types::{Axis, SensorField};

    let fields = [
        SensorField::Time,
        SensorField::Gyro(Axis::Roll),
        SensorField::Gyro(Axis::Pitch),
        SensorField::Gyro(Axis::Yaw),
    ];

    for (name, fixture) in [
        ("BF", "fc-blackbox/btfl_001.bbl"),
        ("AP", "ardupilot/methodic-copter-tarot-x4.bin"),
        ("PX4", "px4/sample_log_small.ulg"),
        ("MAVLink", "mavlink/dronekit-flight.tlog"),
    ] {
        let path = fixture_path(fixture);
        let data = std::fs::read(&path).unwrap();
        let log = propwash_core::decode(&data).unwrap();
        let session = &log.sessions[0];

        let start = Instant::now();
        for _ in 0..100 {
            for f in &fields {
                std::hint::black_box(session.field(f));
            }
        }
        let elapsed = start.elapsed();
        let ms_per = elapsed.as_secs_f64() * 1000.0 / 100.0;

        eprintln!("  {name} field extract (4 fields): {ms_per:.2}ms per iteration");

        // Columnar storage should make field extraction fast
        assert!(
            ms_per < 50.0,
            "{name} field extraction {ms_per:.1}ms exceeds 50ms ceiling"
        );
    }
}
