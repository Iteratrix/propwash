// TODO(refactor/session-typed): rewrite this module as a pipeline:
//   bytes → header parse → frame iterator (frames.rs) → session builder (build.rs)
// For now this stubs decode() to produce empty Session objects of Format::Betaflight,
// keeping the workspace compiling while the new shape is wired in.
#![allow(dead_code)]

mod encoding;
mod frame;
mod header;
mod predictor;
pub mod types;

use crate::session::{Format, Session, SessionMeta};
use crate::types::{Log, Warning};
use header::find_sessions;

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

    for (i, _offset) in offsets.iter().enumerate() {
        sessions.push(Session {
            meta: SessionMeta {
                format: Format::Betaflight,
                session_index: i + 1,
                ..SessionMeta::default()
            },
            ..Session::default()
        });
    }

    Log {
        sessions,
        warnings: global_warnings,
    }
}
