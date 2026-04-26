//! MAVLink telemetry log decoder.
//!
//! Pipeline:
//! 1. [`parser::parse`] decodes the .tlog stream into per-message-name
//!    columnar intermediate ([`parser::MavlinkParsed`]).
//! 2. [`build::session`] folds the intermediate into a typed
//!    [`crate::session::Session`], applying all unit conversions.

mod build;
mod parser;
pub mod types;

use crate::types::{Log, Warning};

/// Decodes a `MAVLink` telemetry log (.tlog).
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let parsed = parser::parse(data, &mut warnings);
    let session = build::session(parsed, warnings, 1);
    Log {
        sessions: vec![session],
        warnings: Vec::new(),
    }
}
