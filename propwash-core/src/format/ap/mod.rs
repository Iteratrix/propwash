//! `ArduPilot` `DataFlash` decoder.
//!
//! Pipeline:
//! 1. [`parser::parse`] decodes the binary stream into per-message-type
//!    columnar intermediate ([`parser::ApParsed`]).
//! 2. [`build::session`] folds the intermediate into a typed
//!    [`crate::session::Session`], applying all unit conversions.

mod build;
mod parser;
pub mod types;

use crate::types::{Log, Warning};

/// Decodes an `ArduPilot` `DataFlash` binary log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let parsed = parser::parse(data, &mut warnings);
    let session = build::session(parsed, warnings, 1);
    Log {
        sessions: vec![session],
        warnings: Vec::new(),
    }
}
