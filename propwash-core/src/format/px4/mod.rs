//! PX4 ULog decoder.
//!
//! Pipeline:
//! 1. [`parser::parse`] decodes the binary ULog into per-msg-id columnar
//!    intermediate ([`parser::Px4Parsed`]).
//! 2. [`build::session`] folds the intermediate into a typed
//!    [`crate::session::Session`], applying all unit conversions.

mod build;
mod parser;
pub mod types;

use crate::types::{Log, Warning};

/// Decodes a PX4 `ULog` binary log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let parsed = parser::parse(data, &mut warnings);
    let session = build::session(parsed, warnings, 1);
    Log {
        sessions: vec![session],
        warnings: Vec::new(),
    }
}
