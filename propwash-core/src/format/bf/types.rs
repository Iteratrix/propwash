//! BF parser-internal data types: field definitions, encoding/predictor
//! enums, header values, frame markers, events, parse stats.
//!
//! No `BfSession` here — the parser writes directly into
//! [`crate::session::Session`] via [`super::build`].
//!
//! Some fields exist for round-trip fidelity (e.g. `BfEvent::FlightMode`'s
//! `last_flags`) even when build doesn't consume them today.
#![allow(dead_code)]

use std::collections::HashMap;

use super::header::parse_bf_field_name;
use crate::types::SensorField;

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
    Zero,
    Previous,
    StraightLine,
    Average2,
    MinThrottle,
    Motor0,
    Increment,
    HomeCoord,
    FifteenHundred,
    VbatRef,
    LastMainFrameTime,
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
            7 => Self::HomeCoord,
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
/// Provides `SensorField` → index lookup for O(1) field access.
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

    pub fn index_of(&self, field: &SensorField) -> Option<usize> {
        self.field_index.get(field).copied()
    }

    pub fn index_of_str(&self, name: &str) -> Option<usize> {
        self.index_of(&parse_bf_field_name(name))
    }
}

/// Whether a frame was decoded from a keyframe or a delta frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BfFrameKind {
    Intra,
    Inter,
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

/// Per-session parse statistics. Useful for diagnostics — the parser
/// exposes them on `Session.meta`.
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

impl BfHeaderValue {
    /// Helper: `int(headers, key, default)` — returns header as i32 with fallback.
    pub fn int(map: &HashMap<String, Self>, key: &str, default: i32) -> i32 {
        use az::WrappingAs;
        match map.get(key) {
            Some(Self::Int(v)) => (*v).wrapping_as::<i32>(),
            Some(Self::Str(s)) => s.parse().unwrap_or(default),
            Some(Self::IntList(_)) | None => default,
        }
    }

    /// Helper: returns header as a Vec<i32>, splitting comma strings if needed.
    pub fn int_list(map: &HashMap<String, Self>, key: &str) -> Vec<i32> {
        use az::WrappingAs;
        match map.get(key) {
            Some(Self::IntList(v)) => v.iter().map(|&x| x.wrapping_as::<i32>()).collect(),
            Some(Self::Int(v)) => vec![(*v).wrapping_as::<i32>()],
            Some(Self::Str(s)) => s.split(',').filter_map(|p| p.trim().parse().ok()).collect(),
            None => Vec::new(),
        }
    }
}
