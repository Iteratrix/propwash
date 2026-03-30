use std::collections::HashMap;
use std::fmt;

use crate::format::bf::analyzed::BfAnalyzedView;
use crate::format::bf::types::{BfFrame, BfRawSession};

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

/// Format-specific raw session data.
#[derive(Debug)]
#[non_exhaustive]
pub enum RawSession {
    Betaflight(BfRawSession),
}

/// Format-specific analyzed view.
#[non_exhaustive]
pub enum Analyzed<'a> {
    Betaflight(BfAnalyzedView<'a>),
}

/// Format-agnostic unified view over any session.
/// Provides sensor data in canonical units regardless of source format.
/// This is the recommended API for most consumers.
pub struct UnifiedView<'a> {
    raw: &'a RawSession,
}

impl fmt::Debug for UnifiedView<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UnifiedView").finish_non_exhaustive()
    }
}

impl UnifiedView<'_> {
    /// Returns the number of main frames.
    pub fn frame_count(&self) -> usize {
        self.bf_parts().1.len()
    }

    /// Returns field names from header definitions.
    pub fn field_names(&self) -> Vec<&str> {
        match self.raw {
            RawSession::Betaflight(bf) => bf.main_field_defs.names(),
        }
    }

    /// Returns the firmware version string.
    pub fn firmware_version(&self) -> &str {
        match self.raw {
            RawSession::Betaflight(bf) => &bf.firmware_version,
        }
    }

    /// Returns the craft name.
    pub fn craft_name(&self) -> &str {
        match self.raw {
            RawSession::Betaflight(bf) => &bf.craft_name,
        }
    }

    /// Computes sample rate from first/last frame timestamps.
    pub fn sample_rate_hz(&self) -> f64 {
        let (session, frames) = self.bf_parts();
        if frames.len() < 2 {
            return 0.0;
        }
        let Some(time_idx) = session.main_field_defs.index_of("time") else {
            return 0.0;
        };
        let t0 = frames
            .first()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let tn = frames
            .last()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let dt_us = tn - t0;
        if dt_us <= 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)]
        let rate = (frames.len() - 1) as f64 / (dt_us as f64 / 1_000_000.0);
        rate
    }

    /// Returns flight duration in seconds.
    pub fn duration_seconds(&self) -> f64 {
        let (session, frames) = self.bf_parts();
        if frames.len() < 2 {
            return 0.0;
        }
        let Some(time_idx) = session.main_field_defs.index_of("time") else {
            return 0.0;
        };
        let t0 = frames
            .first()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let tn = frames
            .last()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let dt_us = tn - t0;
        if dt_us <= 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)]
        let dur = dt_us as f64 / 1_000_000.0;
        dur
    }

    /// Extracts one field as a `Vec<i64>` across all main frames.
    pub fn field(&self, name: &str) -> Vec<i64> {
        let (session, frames) = self.bf_parts();
        let Some(idx) = session.main_field_defs.index_of(name) else {
            return vec![0; frames.len()];
        };
        frames
            .iter()
            .map(|f| f.values.get(idx).copied().unwrap_or(0))
            .collect()
    }

    /// Extracts all fields matching a name prefix.
    pub fn fields(&self, prefix: &str) -> HashMap<String, Vec<i64>> {
        let (session, frames) = self.bf_parts();
        session
            .main_field_defs
            .fields
            .iter()
            .enumerate()
            .filter(|(_, f)| f.name.starts_with(prefix))
            .map(|(idx, f)| {
                let values: Vec<i64> = frames
                    .iter()
                    .map(|frame| frame.values.get(idx).copied().unwrap_or(0))
                    .collect();
                (f.name.clone(), values)
            })
            .collect()
    }

    /// Returns the number of motors detected.
    pub fn motor_count(&self) -> usize {
        self.field_names()
            .iter()
            .filter(|n| n.starts_with("motor["))
            .count()
    }

    fn bf_parts(&self) -> (&BfRawSession, &[BfFrame]) {
        match self.raw {
            RawSession::Betaflight(bf) => (bf, &bf.frames),
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
    pub fn unified(&self) -> UnifiedView<'_> {
        UnifiedView { raw: &self.raw }
    }

    /// Returns a format-specific analyzed view.
    pub fn analyzed(&self) -> Analyzed<'_> {
        match &self.raw {
            RawSession::Betaflight(bf) => Analyzed::Betaflight(BfAnalyzedView { raw: bf }),
        }
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
