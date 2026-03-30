#![doc = "propwash-core: lenient parser for flight controller blackbox logs."]
#![doc = ""]
#![doc = "Three-layer API:"]
#![doc = "- **Unified** (`session.unified()`) — format-agnostic sensor data in canonical units"]
#![doc = "- **Analyzed** (`session.analyzed()`) — format-specific domain knowledge"]
#![doc = "- **Raw** (`session.raw`) — format-specific parsed data, faithful to the file"]

mod betaflight;
mod encoding;
mod frame;
mod header;
mod predictor;
mod reader;
pub mod types;

pub use types::{
    Analyzed, BfAnalyzedView, Log, ParseError, RawSession, Session, UnifiedView, Warning,
};

/// Magic bytes that identify a Betaflight-family blackbox log.
const BETAFLIGHT_MARKER: &[u8] = b"H Product:Blackbox flight data recorder";

/// Decode a blackbox log from raw bytes.
/// Never panics on corrupt data. Collects warnings instead.
pub fn decode(data: &[u8]) -> Log {
    // Auto-detect format by checking for known markers
    if memchr::memmem::find(data, BETAFLIGHT_MARKER).is_some() {
        return betaflight::decode(data);
    }

    // No recognized format
    Log {
        sessions: Vec::new(),
        warnings: vec![Warning {
            message: "No recognized blackbox format found in data".into(),
            byte_offset: None,
        }],
    }
}

/// Decode a blackbox log file. Only fails on I/O errors.
///
/// # Errors
///
/// Returns `ParseError::Io` if the file cannot be read,
/// or `ParseError::NoData` if the file is empty.
pub fn decode_file(path: impl AsRef<std::path::Path>) -> Result<Log, ParseError> {
    let data = std::fs::read(path)?;
    if data.is_empty() {
        return Err(ParseError::NoData);
    }
    Ok(decode(&data))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_empty_returns_no_sessions() {
        let log = decode(b"");
        assert_eq!(log.session_count(), 0);
        assert!(!log.warnings.is_empty());
    }

    #[test]
    fn decode_garbage_returns_no_sessions() {
        let log = decode(b"this is not a blackbox log");
        assert_eq!(log.session_count(), 0);
    }
}
