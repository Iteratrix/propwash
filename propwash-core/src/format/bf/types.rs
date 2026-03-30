use std::collections::HashMap;

/// Whether a field's predicted value wraps as unsigned or signed 32-bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BfFieldSign {
    Signed,
    Unsigned,
}

/// Definition of one field, parsed from header metadata.
#[derive(Debug, Clone)]
pub struct BfFieldDef {
    pub name: String,
    pub signed: bool,
    pub predictor: u8,
    pub encoding: u8,
    pub value_sign: BfFieldSign,
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

    /// Looks up the index of a field by name.
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
    /// Returns the total number of main frames (I + P).
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
    min_throttle: i32,
    min_motor: i32,
    vbat_ref: i32,
}

impl BfRawSession {
    /// Creates a new `BfRawSession` with parsed headers but no frames yet.
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

    /// Gets a header value as an integer with a default fallback.
    #[allow(clippy::cast_possible_truncation)]
    pub fn get_header_int(&self, key: &str, default: i32) -> i32 {
        match self.headers.get(key) {
            Some(BfHeaderValue::Int(v)) => *v as i32,
            Some(BfHeaderValue::Str(s)) => s.parse().unwrap_or(default),
            Some(BfHeaderValue::IntList(_)) | None => default,
        }
    }

    /// Gets a header value as a list of integers.
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

    /// Returns the minimum throttle value from headers.
    pub fn min_throttle(&self) -> i32 {
        self.min_throttle
    }

    /// Returns the minimum motor output value from headers.
    pub fn min_motor(&self) -> i32 {
        self.min_motor
    }

    /// Returns the battery voltage reference from headers.
    pub fn vbat_ref(&self) -> i32 {
        self.vbat_ref
    }

    /// Gets a field value from a frame by name.
    pub fn get_field(&self, frame: &BfFrame, name: &str) -> Option<i64> {
        let idx = self.main_field_defs.index_of(name)?;
        frame.values.get(idx).copied()
    }

    /// Gets a field value from a frame by name, with a default.
    pub fn get_field_or(&self, frame: &BfFrame, name: &str, default: i64) -> i64 {
        self.get_field(frame, name).unwrap_or(default)
    }
}
