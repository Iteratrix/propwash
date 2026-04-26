#![doc = "propwash-core: lenient parser for flight controller blackbox logs."]
#![doc = ""]
#![doc = "Decodes Betaflight `.bbl`, `ArduPilot` `.bin`, PX4 `.ulg`, and"]
#![doc = "`MAVLink` `.tlog` formats into a unified, typed [`Session`]."]

pub mod analysis;
pub mod filter;
pub(crate) mod format;
mod reader;
pub mod session;
pub mod types;
pub mod units;

pub use session::Session;
pub use types::{FilterConfig, Log, ParseError, Warning};

/// Decodes a blackbox log from raw bytes.
/// Never panics on corrupt data. Collects warnings instead.
///
/// # Errors
///
/// Returns `ParseError::UnrecognizedFormat` if the data does not match
/// any known blackbox format.
pub fn decode(data: &[u8]) -> Result<Log, ParseError> {
    format::decode(data)
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
