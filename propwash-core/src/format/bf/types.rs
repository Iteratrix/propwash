use std::collections::HashMap;

use super::header::parse_bf_field_name;
use crate::types::{FilterConfig, SensorField, Warning};

/// Whether a field's predicted value wraps as unsigned or signed 32-bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BfFieldSign {
    Signed,
    Unsigned,
}

/// Encoding scheme used to decode field values from the binary stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Encoding {
    SignedVb,
    UnsignedVb,
    Neg14Bit,
    Tag8_8Svb,
    Tag2_3S32,
    Tag8_4S16,
    Null,
    Tag2_3SVariable,
    Unknown(u8),
}

impl From<u8> for Encoding {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::SignedVb,
            1 => Self::UnsignedVb,
            3 => Self::Neg14Bit,
            6 => Self::Tag8_8Svb,
            7 => Self::Tag2_3S32,
            8 => Self::Tag8_4S16,
            9 => Self::Null,
            10 => Self::Tag2_3SVariable,
            other => Self::Unknown(other),
        }
    }
}

/// Predictor used to reconstruct field values from deltas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Predictor {
    /// Value is used as-is (no prediction).
    Zero,
    /// Delta from previous frame's value.
    Previous,
    /// Linear extrapolation from two previous frames.
    StraightLine,
    /// Average of two previous frames.
    Average2,
    /// Offset from `minthrottle` header value.
    MinThrottle,
    /// Offset from motor[0] value (same frame for I, previous frame for P).
    Motor0,
    /// Incrementing counter (loop iteration).
    Increment,
    /// Offset from 1500.
    FifteenHundred,
    /// Offset from `vbatref` header value.
    VbatRef,
    /// Offset from previous frame's time field.
    LastMainFrameTime,
    /// Offset from `minmotor` header value.
    MinMotor,
    Unknown(u8),
}

impl From<u8> for Predictor {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Zero,
            1 => Self::Previous,
            2 => Self::StraightLine,
            3 => Self::Average2,
            4 => Self::MinThrottle,
            5 => Self::Motor0,
            6 => Self::Increment,
            8 => Self::FifteenHundred,
            9 => Self::VbatRef,
            10 => Self::LastMainFrameTime,
            11 => Self::MinMotor,
            other => Self::Unknown(other),
        }
    }
}

/// Definition of one field, parsed from header metadata.
#[derive(Debug, Clone)]
pub struct BfFieldDef {
    pub name: SensorField,
    pub signed: bool,
    pub predictor: Predictor,
    pub encoding: Encoding,
    pub value_sign: BfFieldSign,
}

/// All field definitions for one frame type.
/// Also provides `SensorField` → index lookup for O(1) field access.
#[derive(Debug, Clone)]
pub struct BfFrameDefs {
    pub fields: Vec<BfFieldDef>,
    field_index: HashMap<SensorField, usize>,
}

impl BfFrameDefs {
    pub fn new(fields: Vec<BfFieldDef>) -> Self {
        let field_index = fields
            .iter()
            .enumerate()
            .map(|(i, f)| (f.name.clone(), i))
            .collect();
        Self {
            fields,
            field_index,
        }
    }

    pub fn names(&self) -> Vec<String> {
        self.fields.iter().map(|f| f.name.to_string()).collect()
    }

    pub fn len(&self) -> usize {
        self.fields.len()
    }

    pub fn is_empty(&self) -> bool {
        self.fields.is_empty()
    }

    /// Looks up the index of a field by `SensorField`.
    pub fn index_of(&self, field: &SensorField) -> Option<usize> {
        self.field_index.get(field).copied()
    }

    /// Looks up the index of a field by BF-native header string name.
    /// Converts to `SensorField` via the BF field name parser.
    pub fn index_of_str(&self, name: &str) -> Option<usize> {
        self.index_of(&parse_bf_field_name(name))
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
/// Use `BfSession::get_field()` for name-based access.
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
        value: f64,
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
pub struct BfSession {
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
    /// P-frame encodings (parallel to `main_field_defs`).
    pub p_encodings: Vec<Encoding>,
    /// P-frame predictors (parallel to `main_field_defs`).
    pub p_predictors: Vec<Predictor>,
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
    /// Non-fatal diagnostics from parsing.
    pub warnings: Vec<Warning>,
    /// 1-based session index within the file.
    pub session_index: usize,
    min_throttle: i32,
    min_motor: i32,
    vbat_ref: i32,
}

impl BfSession {
    /// Creates a new `BfSession` with parsed headers but no frames yet.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        headers: HashMap<String, BfHeaderValue>,
        firmware_type: String,
        firmware_version: String,
        craft_name: String,
        main_field_defs: BfFrameDefs,
        p_encodings: Vec<Encoding>,
        p_predictors: Vec<Predictor>,
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
            warnings: Vec::new(),
            session_index: 0,
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

    /// Gets a field value from a frame by name string.
    pub fn get_field(&self, frame: &BfFrame, name: &str) -> Option<i64> {
        let idx = self.main_field_defs.index_of_str(name)?;
        frame.values.get(idx).copied()
    }

    /// Gets a field value from a frame by name, with a default.
    pub fn get_field_or(&self, frame: &BfFrame, name: &str, default: i64) -> i64 {
        self.get_field(frame, name).unwrap_or(default)
    }

    // ── Methods absorbed from BfAnalyzedView ────────────────────────

    /// Returns the number of main frames.
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    /// Returns field names from header definitions.
    pub fn field_names(&self) -> Vec<String> {
        self.main_field_defs.names()
    }

    /// Returns the firmware version string.
    pub fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    /// Returns the craft name.
    pub fn craft_name(&self) -> &str {
        &self.craft_name
    }

    /// Computes sample rate from first/last frame timestamps.
    #[allow(clippy::cast_precision_loss)]
    pub fn sample_rate_hz(&self) -> f64 {
        if self.frames.len() < 2 {
            return 0.0;
        }
        let Some(time_idx) = self.main_field_defs.index_of(&SensorField::Time) else {
            return 0.0;
        };
        let t0 = self
            .frames
            .first()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let tn = self
            .frames
            .last()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let dt_us = tn - t0;
        if dt_us <= 0 {
            return 0.0;
        }
        (self.frames.len() - 1) as f64 / (dt_us as f64 / 1_000_000.0)
    }

    /// Returns flight duration in seconds.
    #[allow(clippy::cast_precision_loss)]
    pub fn duration_seconds(&self) -> f64 {
        if self.frames.len() < 2 {
            return 0.0;
        }
        let Some(time_idx) = self.main_field_defs.index_of(&SensorField::Time) else {
            return 0.0;
        };
        let t0 = self
            .frames
            .first()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let tn = self
            .frames
            .last()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let dt_us = tn - t0;
        if dt_us <= 0 {
            return 0.0;
        }
        dt_us as f64 / 1_000_000.0
    }

    /// Extracts one field as a `Vec<f64>` across all main frames.
    ///
    /// BF fields are indexed by position, so lookup is via the field-definition
    /// hashmap. `SensorField::Unknown` values resolve only if the parser
    /// previously mapped that exact string to a field index. Unresolvable
    /// fields (including truly unknown names) return an empty `Vec`.
    #[allow(clippy::cast_precision_loss)]
    pub fn field(&self, sensor_field: &SensorField) -> Vec<f64> {
        let Some(idx) = self.main_field_defs.index_of(sensor_field) else {
            return Vec::new();
        };
        self.frames
            .iter()
            .map(|f| f.values.get(idx).copied().unwrap_or(0) as f64)
            .collect()
    }

    /// Returns the number of motors detected from field definitions.
    pub fn motor_count(&self) -> usize {
        self.main_field_defs
            .fields
            .iter()
            .filter(|f| matches!(f.name, SensorField::Motor(_)))
            .count()
    }

    /// Returns the debug mode from headers.
    pub fn debug_mode(&self) -> i32 {
        self.get_header_int("debug_mode", 0)
    }

    /// Returns whether the log ended cleanly (vs truncation/crash).
    pub fn is_truncated(&self) -> bool {
        !self.stats.clean_end
    }

    /// Returns the number of corrupt bytes encountered during parsing.
    pub fn corrupt_bytes(&self) -> usize {
        self.stats.corrupt_bytes
    }

    /// Returns the filter configuration extracted from headers.
    pub fn filter_config(&self) -> FilterConfig {
        let non_zero = |v: i32| -> Option<f64> {
            if v > 0 {
                Some(f64::from(v))
            } else {
                None
            }
        };
        FilterConfig {
            gyro_lpf_hz: non_zero(self.get_header_int("gyro_lowpass_hz", 0)),
            gyro_lpf2_hz: non_zero(self.get_header_int("gyro_lowpass2_hz", 0)),
            dterm_lpf_hz: non_zero(self.get_header_int("dterm_lpf_hz", 0))
                .or_else(|| non_zero(self.get_header_int("dterm_lowpass_hz", 0))),
            dyn_notch_min_hz: non_zero(self.get_header_int("dyn_notch_min_hz", 0)),
            dyn_notch_max_hz: non_zero(self.get_header_int("dyn_notch_max_hz", 0)),
            gyro_notch1_hz: non_zero(self.get_header_int("gyro_notch_hz", 0)),
            gyro_notch2_hz: non_zero(self.get_header_int("gyro_notch2_hz", 0)),
        }
    }

    /// Returns the motor output range `(min, max)` from header metadata.
    pub fn motor_range(&self) -> (f64, f64) {
        let motor_output = self.get_header_int_list("motorOutput");
        let max = match motor_output.len() {
            0 => 2047,
            1 => motor_output[0],
            _ => motor_output[1],
        };
        (0.0, f64::from(max))
    }

    /// Returns parse warnings.
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    /// Returns the 1-based session index within the file.
    pub fn index(&self) -> usize {
        self.session_index
    }
}
