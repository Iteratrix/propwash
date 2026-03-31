mod encoding;
mod frame;
mod header;
mod predictor;
pub mod types;

use crate::types::{Log, RawSession, Session, Warning};
use header::{find_sessions, parse_headers};
use types::{BfHeaderValue, BfRawSession};

/// Decodes a Betaflight-family blackbox log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let offsets = find_sessions(data);
    let mut sessions = Vec::new();
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
        }

        #[allow(clippy::cast_possible_truncation)]
        let motor_output = match parsed.raw.get("motorOutput") {
            Some(BfHeaderValue::IntList(v)) => v.first().copied().unwrap_or(0) as i32,
            _ => 0,
        };

        let mut raw_session = BfRawSession::new(
            parsed.raw,
            parsed.firmware_type,
            parsed.firmware_version,
            parsed.craft_name,
            parsed.main_field_defs,
            parsed.p_encodings,
            parsed.p_predictors,
            parsed.slow_field_defs,
            parsed.gps_field_defs,
            parsed.gps_home_field_defs,
            motor_output,
        );

        if !raw_session.main_field_defs.is_empty() {
            let binary_data = &data[parsed.binary_start..session_end];
            let (main, slow, gps, events, stats) = frame::parse_session_frames(
                binary_data,
                parsed.binary_start,
                &raw_session,
                &mut warnings,
            );
            raw_session.frames = main;
            raw_session.slow_frames = slow;
            raw_session.gps_frames = gps;
            raw_session.events = events;
            raw_session.stats = stats;
        }

        sessions.push(Session {
            raw: RawSession::Betaflight(raw_session),
            warnings,
            index: i + 1,
        });
    }

    Log {
        sessions,
        warnings: global_warnings,
    }
}
