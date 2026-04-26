//! Integration tests.
//!
//! TODO(refactor/session-typed): the legacy golden suite (preserved at
//! `tests/integration.rs.bak`) was tied to the old `Session` enum and
//! `field(SensorField)` API. Rebuild golden tests against the new typed
//! `Session` shape as each format parser is filled in.
//!
//! For the duration of the refactor, this stub asserts only that each
//! format's decoder returns at least one session with the expected
//! `Format` tag — enough to catch regression in the dispatch layer.

use propwash_core::session::Format;

fn fixture(name: &str) -> Vec<u8> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test-data")
        .join(name);
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {}", path.display(), e))
}

#[test]
fn betaflight_dispatch_stub() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test-data/betaflight");
    if !path.exists() {
        eprintln!("skip: no test-data/betaflight directory");
        return;
    }
    let entry = std::fs::read_dir(&path)
        .unwrap()
        .filter_map(Result::ok)
        .find(|e| e.path().extension().is_some_and(|x| x == "bbl"));
    let Some(entry) = entry else {
        eprintln!("skip: no .bbl fixtures present");
        return;
    };
    let data = std::fs::read(entry.path()).unwrap();
    let log = propwash_core::decode(&data).expect("decode bf fixture");
    assert!(!log.sessions.is_empty(), "expected ≥1 session");
    for s in &log.sessions {
        assert_eq!(s.meta.format, Format::Betaflight);
    }
}

#[test]
fn ardupilot_dispatch_stub() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test-data/ardupilot");
    if !path.exists() {
        eprintln!("skip: no test-data/ardupilot directory");
        return;
    }
    let entry = std::fs::read_dir(&path)
        .unwrap()
        .filter_map(Result::ok)
        .find(|e| e.path().extension().is_some_and(|x| x == "bin"));
    let Some(entry) = entry else {
        eprintln!("skip: no .bin fixtures present");
        return;
    };
    let data = std::fs::read(entry.path()).unwrap();
    let log = propwash_core::decode(&data).expect("decode ap fixture");
    assert!(!log.sessions.is_empty(), "expected ≥1 session");
    for s in &log.sessions {
        assert_eq!(s.meta.format, Format::ArduPilot);
    }
}

#[test]
fn px4_dispatch_stub() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test-data/px4");
    if !path.exists() {
        eprintln!("skip: no test-data/px4 directory");
        return;
    }
    let entry = std::fs::read_dir(&path)
        .unwrap()
        .filter_map(Result::ok)
        .find(|e| e.path().extension().is_some_and(|x| x == "ulg"));
    let Some(entry) = entry else {
        eprintln!("skip: no .ulg fixtures present");
        return;
    };
    let data = std::fs::read(entry.path()).unwrap();
    let log = propwash_core::decode(&data).expect("decode px4 fixture");
    assert!(!log.sessions.is_empty(), "expected ≥1 session");
    for s in &log.sessions {
        assert_eq!(s.meta.format, Format::Px4);
    }
}

#[test]
fn empty_input_errors() {
    assert!(propwash_core::decode(b"").is_err());
}

// Suppress unused import in case fixture loaders aren't called.
#[allow(dead_code)]
fn _suppress() {
    let _ = fixture;
}
