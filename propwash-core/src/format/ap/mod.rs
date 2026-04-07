mod parser;
pub mod types;

use crate::types::{Log, RawSession, Warning};

/// Decodes an `ArduPilot` `DataFlash` binary log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let mut raw_session = parser::parse(data, &mut warnings);
    raw_session.warnings = warnings;
    raw_session.session_index = 1;

    Log {
        sessions: vec![RawSession::ArduPilot(raw_session)],
        warnings: Vec::new(),
    }
}
