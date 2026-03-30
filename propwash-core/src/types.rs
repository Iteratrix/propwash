use std::collections::HashMap;
use std::fmt;

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
    #[allow(clippy::cast_possible_truncation)]
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Self::Int(v) => Some(*v),
            Self::Float(v) => Some(*v as i64),
            Self::Str(_) | Self::Bool(_) => None,
        }
    }

    #[allow(clippy::cast_precision_loss)]
    pub fn as_float(&self) -> Option<f64> {
        match self {
            Self::Float(v) => Some(*v),
            Self::Int(v) => Some(*v as f64),
            Self::Str(_) | Self::Bool(_) => None,
        }
    }
}

/// Definition of one field, parsed from header metadata.
#[derive(Debug, Clone)]
pub struct BfFieldDef {
    pub name: String,
    pub signed: bool,
    pub predictor: u8,
    pub encoding: u8,
}

/// All field definitions for one frame type.
/// Also provides name→index lookup for O(1) field access.
#[derive(Debug, Clone)]
pub struct BfFrameDefs {
    pub fields: Vec<BfFieldDef>,
    name_index: HashMap<String, usize>,
}

impl BfFrameDefs {
    pub fn new(fields: Vec<BfFieldDef>) -> Self {
        let name_index = fields
            .iter()
            .enumerate()
            .map(|(i, f)| (f.name.clone(), i))
            .collect();
        Self { fields, name_index }
    }

    pub fn names(&self) -> Vec<&str> {
        self.fields.iter().map(|f| f.name.as_str()).collect()
    }

    pub fn len(&self) -> usize {
        self.fields.len()
    }

    pub fn is_empty(&self) -> bool {
        self.fields.is_empty()
    }

    /// Look up the index of a field by name.
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.name_index.get(name).copied()
    }
}

/// Whether a frame was decoded from a keyframe or a delta frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BfFrameKind {
    /// I-frame: self-contained keyframe.
    Intra,
    /// P-frame: delta-decoded against previous frames.
    Inter,
}

/// A single decoded Betaflight frame with provenance.
/// Values are indexed by field position (matching `BfFrameDefs`).
/// Use `BfRawSession::get_field()` for name-based access.
#[derive(Debug, Clone)]
pub struct BfFrame {
    /// Field values indexed by position in the session's `main_field_defs`.
    pub values: Vec<i64>,
    /// Whether this was an I-frame or P-frame.
    pub kind: BfFrameKind,
    /// Byte offset in the original file.
    pub byte_offset: usize,
    /// Sequential index within the session (0-based).
    pub frame_index: usize,
}

/// Discrete events recorded during a Betaflight flight.
#[derive(Debug, Clone)]
pub enum BfEvent {
    SyncBeep {
        time_us: u64,
    },
    Disarm {
        reason: u32,
    },
    FlightMode {
        flags: u32,
        last_flags: u32,
    },
    InflightAdjustment {
        function: u8,
        value: i32,
    },
    LoggingResume {
        log_iteration: u32,
        current_time: u32,
    },
    LogEnd,
    Unknown {
        type_id: u8,
    },
}

/// Structured parse statistics for a Betaflight session.
#[derive(Debug, Clone, Default)]
pub struct BfParseStats {
    pub i_frame_count: usize,
    pub p_frame_count: usize,
    pub slow_frame_count: usize,
    pub gps_frame_count: usize,
    pub event_count: usize,
    pub corrupt_bytes: usize,
    pub clean_end: bool,
}

impl BfParseStats {
    pub fn total_main_frames(&self) -> usize {
        self.i_frame_count + self.p_frame_count
    }
}

/// A header value from a Betaflight log.
#[derive(Debug, Clone)]
pub enum BfHeaderValue {
    Str(String),
    Int(i64),
    IntList(Vec<i64>),
}

/// Complete raw data for one Betaflight session.
#[derive(Debug)]
pub struct BfRawSession {
    /// Raw header key-value pairs.
    pub headers: HashMap<String, BfHeaderValue>,
    /// Firmware type string (e.g., "Cleanflight").
    pub firmware_type: String,
    /// Firmware version string (e.g., "Betaflight 4.2.9 ...").
    pub firmware_version: String,
    /// Craft name from headers.
    pub craft_name: String,
    /// I-frame field definitions.
    pub main_field_defs: BfFrameDefs,
    /// P-frame encoding IDs (parallel to `main_field_defs`).
    pub p_encodings: Vec<u8>,
    /// P-frame predictor IDs (parallel to `main_field_defs`).
    pub p_predictors: Vec<u8>,
    /// Slow-frame field definitions.
    pub slow_field_defs: Option<BfFrameDefs>,
    /// GPS-frame field definitions.
    pub gps_field_defs: Option<BfFrameDefs>,
    /// GPS home field definitions.
    pub gps_home_field_defs: Option<BfFrameDefs>,
    /// Decoded main frames (I + P interleaved in order).
    pub frames: Vec<BfFrame>,
    /// Decoded slow frames.
    pub slow_frames: Vec<BfFrame>,
    /// Decoded GPS frames.
    pub gps_frames: Vec<BfFrame>,
    /// Discrete events.
    pub events: Vec<BfEvent>,
    /// Parse statistics.
    pub stats: BfParseStats,

    /// `get_int` helper for predictor functions.
    min_throttle: i32,
    min_motor: i32,
    vbat_ref: i32,
}

impl BfRawSession {
    /// Create a new `BfRawSession` with parsed headers but no frames yet.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        headers: HashMap<String, BfHeaderValue>,
        firmware_type: String,
        firmware_version: String,
        craft_name: String,
        main_field_defs: BfFrameDefs,
        p_encodings: Vec<u8>,
        p_predictors: Vec<u8>,
        slow_field_defs: Option<BfFrameDefs>,
        gps_field_defs: Option<BfFrameDefs>,
        gps_home_field_defs: Option<BfFrameDefs>,
        min_motor_override: i32,
    ) -> Self {
        #[allow(clippy::cast_possible_truncation)]
        let min_throttle = match headers.get("minthrottle") {
            Some(BfHeaderValue::Int(v)) => *v as i32,
            _ => 0,
        };
        #[allow(clippy::cast_possible_truncation)]
        let vbat_ref = match headers.get("vbatref") {
            Some(BfHeaderValue::Int(v)) => *v as i32,
            _ => 0,
        };
        Self {
            headers,
            firmware_type,
            firmware_version,
            craft_name,
            main_field_defs,
            p_encodings,
            p_predictors,
            slow_field_defs,
            gps_field_defs,
            gps_home_field_defs,
            frames: Vec::new(),
            slow_frames: Vec::new(),
            gps_frames: Vec::new(),
            events: Vec::new(),
            stats: BfParseStats::default(),
            min_throttle,
            min_motor: min_motor_override,
            vbat_ref,
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn get_header_int(&self, key: &str, default: i32) -> i32 {
        match self.headers.get(key) {
            Some(BfHeaderValue::Int(v)) => *v as i32,
            Some(BfHeaderValue::Str(s)) => s.parse().unwrap_or(default),
            Some(BfHeaderValue::IntList(_)) | None => default,
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn get_header_int_list(&self, key: &str) -> Vec<i32> {
        match self.headers.get(key) {
            Some(BfHeaderValue::IntList(v)) => v.iter().map(|&x| x as i32).collect(),
            Some(BfHeaderValue::Int(v)) => vec![*v as i32],
            Some(BfHeaderValue::Str(s)) => {
                s.split(',').filter_map(|p| p.trim().parse().ok()).collect()
            }
            None => Vec::new(),
        }
    }

    pub fn min_throttle(&self) -> i32 {
        self.min_throttle
    }

    pub fn min_motor(&self) -> i32 {
        self.min_motor
    }

    pub fn vbat_ref(&self) -> i32 {
        self.vbat_ref
    }

    /// Get a field value from a frame by name.
    pub fn get_field(&self, frame: &BfFrame, name: &str) -> Option<i64> {
        let idx = self.main_field_defs.index_of(name)?;
        frame.values.get(idx).copied()
    }

    /// Get a field value from a frame by name, with a default.
    pub fn get_field_or(&self, frame: &BfFrame, name: &str, default: i64) -> i64 {
        self.get_field(frame, name).unwrap_or(default)
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

/// Betaflight-specific analyzed view. Borrows raw data.
pub struct BfAnalyzedView<'a> {
    pub raw: &'a BfRawSession,
}

impl fmt::Debug for BfAnalyzedView<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BfAnalyzedView").finish_non_exhaustive()
    }
}

impl BfAnalyzedView<'_> {
    /// Number of motors detected from field definitions.
    pub fn motor_count(&self) -> usize {
        self.raw
            .main_field_defs
            .fields
            .iter()
            .filter(|f| f.name.starts_with("motor["))
            .count()
    }

    /// Whether bidirectional `DShot` RPM telemetry is present.
    pub fn has_rpm_telemetry(&self) -> bool {
        self.raw
            .main_field_defs
            .fields
            .iter()
            .any(|f| f.name.starts_with("eRPM["))
    }

    /// Whether unfiltered gyro data is logged.
    pub fn has_gyro_unfiltered(&self) -> bool {
        self.raw
            .main_field_defs
            .fields
            .iter()
            .any(|f| f.name.starts_with("gyroUnfilt["))
    }

    /// Debug mode from headers (determines what `debug[0-3]` fields mean).
    pub fn debug_mode(&self) -> i32 {
        self.raw.get_header_int("debug_mode", 0)
    }

    /// Whether the log ended cleanly (vs truncation/crash).
    pub fn is_truncated(&self) -> bool {
        !self.raw.stats.clean_end
    }

    /// Parse statistics.
    pub fn stats(&self) -> &BfParseStats {
        &self.raw.stats
    }
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
    /// Number of main frames.
    pub fn frame_count(&self) -> usize {
        self.bf_parts().1.len()
    }

    /// Field names from header definitions.
    pub fn field_names(&self) -> Vec<&str> {
        match self.raw {
            RawSession::Betaflight(bf) => bf.main_field_defs.names(),
        }
    }

    /// Firmware version string.
    pub fn firmware_version(&self) -> &str {
        match self.raw {
            RawSession::Betaflight(bf) => &bf.firmware_version,
        }
    }

    /// Craft name.
    pub fn craft_name(&self) -> &str {
        match self.raw {
            RawSession::Betaflight(bf) => &bf.craft_name,
        }
    }

    /// Compute sample rate from first/last frame timestamps.
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

    /// Flight duration in seconds.
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

    /// Extract one field as a `Vec<i64>` across all main frames.
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

    /// Extract all fields matching a name prefix.
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

    /// Number of motors detected.
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
    /// Unified view — format-agnostic sensor data in canonical units.
    /// This is the recommended API for most consumers.
    pub fn unified(&self) -> UnifiedView<'_> {
        UnifiedView { raw: &self.raw }
    }

    /// Format-specific analyzed view — for firmware-specific insights.
    pub fn analyzed(&self) -> Analyzed<'_> {
        match &self.raw {
            RawSession::Betaflight(bf) => Analyzed::Betaflight(BfAnalyzedView { raw: bf }),
        }
    }

    /// Convenience: number of main frames.
    pub fn frame_count(&self) -> usize {
        match &self.raw {
            RawSession::Betaflight(bf) => bf.frames.len(),
        }
    }

    /// Convenience: field names from header definitions.
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
    pub fn session_count(&self) -> usize {
        self.sessions.len()
    }
}
