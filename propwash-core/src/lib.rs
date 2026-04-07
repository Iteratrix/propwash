#![doc = "propwash-core: lenient parser for flight controller blackbox logs."]
#![doc = ""]
#![doc = "Two-layer API:"]
#![doc = "- **Unified** — `Session` implements the `Unified` trait for format-agnostic sensor data"]
#![doc = "- **Raw** (`session.raw`) — format-specific parsed data, faithful to the file"]

pub mod analysis;
#[cfg(feature = "raw")]
pub mod format;
#[cfg(not(feature = "raw"))]
pub(crate) mod format;
mod reader;
pub mod types;

pub use types::{Log, ParseError, RawSession, Session, Warning};

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
        return Ok(format::bf::decode(data));
    }

    if data.len() >= 3 && data[..3] == *ARDUPILOT_MARKER {
        return Ok(format::ap::decode(data));
    }

    if data.len() >= 7 && data[..7] == *ULOG_MAGIC {
        return Ok(format::px4::decode(data));
    }

    Err(ParseError::UnrecognizedFormat)
}

/// Decodes a blackbox log file.
///
/// # Errors
///
/// Returns `ParseError::Io` if the file cannot be read,
/// `ParseError::NoData` if the file is empty, or
/// `ParseError::UnrecognizedFormat` if the data does not match any known format.
pub fn decode_file(path: impl AsRef<std::path::Path>) -> Result<Log, ParseError> {
    let data = std::fs::read(path)?;
    if data.is_empty() {
        return Err(ParseError::NoData);
    }
    decode(&data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_empty_returns_unrecognized() {
        let err = decode(b"").unwrap_err();
        assert!(matches!(err, ParseError::UnrecognizedFormat));
    }

    #[test]
    fn decode_garbage_returns_unrecognized() {
        let err = decode(b"this is not a blackbox log").unwrap_err();
        assert!(matches!(err, ParseError::UnrecognizedFormat));
    }
}
