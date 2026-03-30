mod analyzed;
pub mod raw;
mod unified;

use std::fmt;

pub use analyzed::{Analyzed, BfAnalyzedView};
pub use raw::{
    BfEvent, BfFieldDef, BfFrame, BfFrameDefs, BfFrameKind, BfHeaderValue, BfParseStats,
    BfRawSession, RawSession,
};
pub use unified::UnifiedView;

/// A non-fatal diagnostic collected during parsing.
/// The parser never panics on corrupt data — it collects these instead.
#[derive(Debug, Clone)]
pub struct Warning {
    pub message: String,
    pub byte_offset: Option<usize>,
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.byte_offset {
            Some(offset) => write!(f, "[0x{offset:x}] {}", self.message),
            None => write!(f, "{}", self.message),
        }
    }
}

/// The only error the public API can return — file I/O failures.
/// Parse corruption is never an error; it produces warnings.
#[derive(Debug)]
pub enum ParseError {
    Io(std::io::Error),
    NoData,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "I/O error: {e}"),
            Self::NoData => write!(f, "No data to parse"),
        }
    }
}

impl std::error::Error for ParseError {}

impl From<std::io::Error> for ParseError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// A value decoded from a log frame. Formats differ in what types they produce.
#[derive(Debug, Clone)]
pub enum Value {
    Int(i64),
    Float(f64),
    Str(String),
    Bool(bool),
}

impl Value {
    /// Converts to an integer, truncating floats.
    #[allow(clippy::cast_possible_truncation)]
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Self::Int(v) => Some(*v),
            Self::Float(v) => Some(*v as i64),
            Self::Str(_) | Self::Bool(_) => None,
        }
    }

    /// Converts to a float, promoting integers.
    #[allow(clippy::cast_precision_loss)]
    pub fn as_float(&self) -> Option<f64> {
        match self {
            Self::Float(v) => Some(*v),
            Self::Int(v) => Some(*v as f64),
            Self::Str(_) | Self::Bool(_) => None,
        }
    }
}

/// One parsed session from a log file. Entry point for all three layers.
#[derive(Debug)]
pub struct Session {
    /// Format-specific raw data — always available.
    pub raw: RawSession,
    /// Non-fatal diagnostics from parsing.
    pub warnings: Vec<Warning>,
    /// 1-based session index within the file.
    pub index: usize,
}

impl Session {
    /// Returns a unified view — format-agnostic sensor data in canonical units.
    /// This is the recommended API for most consumers.
    pub fn unified(&self) -> UnifiedView<'_> {
        UnifiedView::new(&self.raw)
    }

    /// Returns a format-specific analyzed view — for firmware-specific insights.
    pub fn analyzed(&self) -> Analyzed<'_> {
        analyzed::analyzed_from_raw(&self.raw)
    }

    /// Returns the number of main frames.
    pub fn frame_count(&self) -> usize {
        match &self.raw {
            RawSession::Betaflight(bf) => bf.frames.len(),
        }
    }

    /// Returns the field names from header definitions.
    pub fn field_names(&self) -> Vec<&str> {
        match &self.raw {
            RawSession::Betaflight(bf) => bf.main_field_defs.names(),
        }
    }
}

/// Complete parsed log file.
#[derive(Debug)]
pub struct Log {
    pub sessions: Vec<Session>,
    pub warnings: Vec<Warning>,
}

impl Log {
    /// Returns the number of sessions in the log.
    pub fn session_count(&self) -> usize {
        self.sessions.len()
    }
}
