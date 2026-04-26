// TODO(refactor/session-typed): rewrite this module as a pipeline:
//   bytes → message stream (frames.rs) → session builder (build.rs)
#![allow(dead_code)]

mod parser;
pub mod types;

use crate::session::{Format, Session, SessionMeta};
use crate::types::{Log, Warning};

/// Decodes an `ArduPilot` `DataFlash` binary log.
pub(crate) fn decode(_data: &[u8]) -> Log {
    let warnings: Vec<Warning> = Vec::new();
    Log {
        sessions: vec![Session {
            meta: SessionMeta {
                format: Format::ArduPilot,
                session_index: 1,
                ..SessionMeta::default()
            },
            ..Session::default()
        }],
        warnings,
    }
}
