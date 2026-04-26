//! Betaflight blackbox decoder.
//!
//! Pipeline:
//! 1. [`header::find_sessions`] locates the start byte of each session.
//! 2. [`header::parse_headers`] consumes the text header block, returning
//!    field definitions + the raw header map.
//! 3. [`frames::BfFrames`] streams typed [`frames::BfFrame`]s from the
//!    binary body (stateful — holds the prev-frame buffer for delta
//!    predictors and the GPS home reference).
//! 4. [`build::session`] folds the frame stream into a typed
//!    [`crate::session::Session`], applying all unit conversions.

mod build;
mod encoding;
mod frames;
mod header;
mod predictor;
pub mod types;

use az::WrappingAs;

use crate::session::Session;
use crate::types::{Log, Warning};
use frames::BfFrames;
use header::{find_sessions, parse_headers};
use types::BfHeaderValue;

/// Decodes a Betaflight-family blackbox log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let offsets = find_sessions(data);
    let mut sessions: Vec<Session> = Vec::new();
    let mut global_warnings: Vec<Warning> = Vec::new();

    if offsets.is_empty() {
        global_warnings.push(Warning {
            message: "No log sessions found in data".into(),
            byte_offset: None,
        });
        return Log {
            sessions,
            warnings: global_warnings,
        };
    }

    for (i, &offset) in offsets.iter().enumerate() {
        let session_end = offsets.get(i + 1).copied().unwrap_or(data.len());
        let mut warnings: Vec<Warning> = Vec::new();

        let parsed = parse_headers(data, offset, session_end, &mut warnings);

        if parsed.main_field_defs.is_empty() {
            warnings.push(Warning {
                message: "No main frame field definitions found".into(),
                byte_offset: Some(offset),
            });
            continue;
        }

        let motor_output = match parsed.raw.get("motorOutput") {
            Some(BfHeaderValue::IntList(v)) => v.first().copied().unwrap_or(0).wrapping_as::<i32>(),
            _ => 0,
        };

        let binary = &data[parsed.binary_start..session_end];
        let frames = BfFrames::new(
            binary,
            parsed.binary_start,
            &parsed.raw,
            &parsed.main_field_defs,
            &parsed.p_encodings,
            &parsed.p_predictors,
            parsed.slow_field_defs.as_ref(),
            parsed.gps_field_defs.as_ref(),
            parsed.gps_home_field_defs.as_ref(),
            motor_output,
        );

        let session = build::session(
            frames,
            &parsed.raw,
            &parsed.main_field_defs,
            parsed.slow_field_defs.as_ref(),
            parsed.gps_field_defs.as_ref(),
            parsed.firmware_version,
            parsed.craft_name,
            i + 1,
            warnings,
        );

        sessions.push(session);
    }

    Log {
        sessions,
        warnings: global_warnings,
    }
}
