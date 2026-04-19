mod encoding;
mod frame;
mod header;
mod predictor;
pub mod types;

use az::{Az, WrappingAs};

use crate::types::{Log, Session, Warning};
use header::{find_sessions, parse_headers};
use types::{BfFrame, BfHeaderValue, BfSession};

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

        let motor_output = match parsed.raw.get("motorOutput") {
            Some(BfHeaderValue::IntList(v)) => v.first().copied().unwrap_or(0).wrapping_as::<i32>(),
            _ => 0,
        };

        let mut raw_session = BfSession::new(
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
            let parsed_frames = frame::parse_session_frames(
                binary_data,
                parsed.binary_start,
                &raw_session,
                &mut warnings,
            );

            // Transpose row-oriented frames into columnar storage
            raw_session.main_columns =
                transpose(&parsed_frames.main, raw_session.main_field_defs.len());
            raw_session.frame_kinds = parsed_frames.main.iter().map(|f| f.kind).collect();
            if let Some(ref defs) = raw_session.slow_field_defs {
                raw_session.slow_columns = transpose(&parsed_frames.slow, defs.len());
            }
            if let Some(ref defs) = raw_session.gps_field_defs {
                raw_session.gps_columns = transpose(&parsed_frames.gps, defs.len());
            }

            raw_session.gps_home = parsed_frames.gps_home;
            raw_session.events = parsed_frames.events;
            raw_session.stats = parsed_frames.stats;
        }

        raw_session.warnings = warnings;
        raw_session.session_index = i + 1;
        sessions.push(Session::Betaflight(raw_session));
    }

    Log {
        sessions,
        warnings: global_warnings,
    }
}

/// Transpose row-oriented `BfFrame` vectors into column-oriented `Vec<Vec<f64>>`.
fn transpose(frames: &[BfFrame], n_fields: usize) -> Vec<Vec<f64>> {
    let mut columns = Vec::with_capacity(n_fields);
    for _ in 0..n_fields {
        columns.push(Vec::with_capacity(frames.len()));
    }
    for frame in frames {
        for (i, col) in columns.iter_mut().enumerate() {
            col.push(frame.values.get(i).copied().unwrap_or(0).az::<f64>());
        }
    }
    columns
}
