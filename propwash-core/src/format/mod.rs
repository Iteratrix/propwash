pub mod ap;
pub mod bf;
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

    Err(ParseError::UnrecognizedFormat)
}
