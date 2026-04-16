mod parser;
pub mod types;

use crate::types::{Log, Session, Warning};

/// Decodes a PX4 `ULog` binary log.
pub(crate) fn decode(data: &[u8]) -> Log {
    let mut warnings: Vec<Warning> = Vec::new();
    let mut raw_session = parser::parse(data, &mut warnings);
    raw_session.warnings = warnings;
    raw_session.session_index = 1;

    Log {
        sessions: vec![Session::Px4(raw_session)],
        warnings: Vec::new(),
    }
}
