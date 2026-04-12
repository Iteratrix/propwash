mod parser;
pub mod types;

use crate::types::{Log, Session, Warning};

/// Decodes a `MAVLink` telemetry log (.tlog).
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let mut session = parser::parse(data, &mut warnings);
    session.warnings = warnings;
    session.session_index = 1;

    Log {
        sessions: vec![Session::Mavlink(session)],
        warnings: Vec::new(),
    }
}
