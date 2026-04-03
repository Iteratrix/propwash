mod parser;
pub mod types;

use crate::types::{Log, RawSession, Session, Warning};

/// Decodes an `ArduPilot` `DataFlash` binary log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let raw_session = parser::parse(data, &mut warnings);

    let sessions = vec![Session {
        raw: RawSession::ArduPilot(raw_session),
        warnings,
        index: 1,
    }];

    Log {
        sessions,
        warnings: Vec::new(),
    }
}
