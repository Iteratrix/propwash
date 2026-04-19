pub mod ap;
pub mod bf;
pub mod common;
pub mod mavlink;
pub mod px4;

use crate::types::{Log, ParseError};

const BETAFLIGHT_MARKER: &[u8] = b"H Product:Blackbox flight data recorder";
const ARDUPILOT_MARKER: &[u8] = &[0xA3, 0x95, 0x80]; // HEAD1 HEAD2 FMT_TYPE
const ULOG_MAGIC: &[u8] = b"\x55\x4c\x6f\x67\x01\x12\x35";

/// Decodes a blackbox log from raw bytes.
/// Never panics on corrupt data. Collects warnings instead.
///
/// # Errors
///
/// Returns `ParseError::UnrecognizedFormat` if the data does not match
/// any known blackbox format.
pub fn decode(data: &[u8]) -> Result<Log, ParseError> {
    if memchr::memmem::find(data, BETAFLIGHT_MARKER).is_some() {
        return Ok(bf::decode(data));
    }

    if data.len() >= 3 && data[..3] == *ARDUPILOT_MARKER {
        return Ok(ap::decode(data));
    }

    if data.len() >= 7 && data[..7] == *ULOG_MAGIC {
        return Ok(px4::decode(data));
    }

    if is_mavlink_tlog(data) {
        return Ok(mavlink::decode(data));
    }

    Err(ParseError::UnrecognizedFormat)
}

/// Heuristic detection for `MAVLink` telemetry logs (.tlog).
///
/// A tlog is a sequence of `[8-byte timestamp][MAVLink frame]` records.
/// We check that byte 8 is a `MAVLink` marker (0xFE for v1, 0xFD for v2)
/// and that the first 8 bytes decode to a plausible Unix timestamp.
fn is_mavlink_tlog(data: &[u8]) -> bool {
    if data.len() < 17 {
        return false; // Need at least timestamp + minimal v1 frame
    }

    let marker = data[8];
    if marker != 0xFE && marker != 0xFD {
        // 0xFE = MAVLink v1, 0xFD = MAVLink v2
        return false;
    }

    // Validate timestamp: must be between year 2000 and 2100
    let ts = u64::from_be_bytes(data[..8].try_into().unwrap_or([0; 8]));
    let ts_seconds = ts / 1_000_000;
    // 2000-01-01 = 946_684_800, 2100-01-01 = 4_102_444_800
    (946_684_800..4_102_444_800).contains(&ts_seconds)
}
