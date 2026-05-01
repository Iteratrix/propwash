//! AP parser-internal data types: FMT field types, message definitions.
//!
//! No `ApSession` here — the parser writes directly into
//! [`crate::session::Session`] via [`super::build`].
#![allow(dead_code)]

/// Format character from FMT message — determines wire size, sign, and
/// (where applicable) the canonical scale factor that the AP `DataFlash`
/// spec attaches to each character.
///
/// AP scaling conventions:
/// - `c` / `C` — int16/uint16 × 100 (centidegrees / fixed-point ×0.01)
/// - `e` / `E` — int32/uint32 × 100 (same idea, 32-bit)
/// - `L` — int32 × 1e7 (lat/lng raw → decimal degrees)
///
/// All other types are unscaled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldType {
    I8,
    U8,
    I16,
    /// `c` — int16 raw / 100 → degrees-with-0.01°-precision (or other ×0.01 quantities).
    I16Centi,
    U16,
    /// `C` — uint16 raw / 100 → ×0.01-precision quantity.
    U16Centi,
    I32,
    /// `e` — int32 raw / 100 → ×0.01-precision quantity (e.g. some altitudes).
    I32Centi,
    /// `L` — int32 raw × 1e-7 → decimal degrees (lat/lng).
    I32DegreesE7,
    U32,
    /// `E` — uint32 raw / 100 → ×0.01-precision quantity.
    U32Centi,
    I64,
    U64,
    Float,
    Double,
    Float16,
    Char4,
    Char16,
    Char64,
    I16Array32,
    FlightMode,
    Unknown(u8),
}

impl FieldType {
    pub fn from_format_char(c: u8) -> Self {
        match c {
            b'b' => Self::I8,
            b'B' | b'M' => Self::U8,
            b'h' => Self::I16,
            b'c' => Self::I16Centi,
            b'H' => Self::U16,
            b'C' => Self::U16Centi,
            b'i' => Self::I32,
            b'e' => Self::I32Centi,
            b'L' => Self::I32DegreesE7,
            b'I' => Self::U32,
            b'E' => Self::U32Centi,
            b'q' => Self::I64,
            b'Q' => Self::U64,
            b'f' => Self::Float,
            b'd' => Self::Double,
            b'g' => Self::Float16,
            b'n' => Self::Char4,
            b'N' => Self::Char16,
            b'Z' => Self::Char64,
            b'a' => Self::I16Array32,
            other => Self::Unknown(other),
        }
    }

    pub fn wire_size(self) -> usize {
        match self {
            Self::I8 | Self::U8 | Self::FlightMode => 1,
            Self::I16 | Self::I16Centi | Self::U16 | Self::U16Centi | Self::Float16 => 2,
            Self::I32
            | Self::I32Centi
            | Self::I32DegreesE7
            | Self::U32
            | Self::U32Centi
            | Self::Float
            | Self::Char4 => 4,
            Self::I64 | Self::U64 | Self::Double => 8,
            Self::Char16 => 16,
            Self::Char64 | Self::I16Array32 => 64,
            Self::Unknown(_) => 0,
        }
    }
}

/// Definition of one message type, parsed from a FMT message.
#[derive(Debug, Clone)]
pub struct ApMsgDef {
    pub msg_type: u8,
    pub name: String,
    pub field_types: Vec<FieldType>,
    pub field_names: Vec<String>,
    pub msg_len: usize,
    pub format_str: String,
}

pub use crate::format::common::MsgColumns;

/// Per-session parse statistics.
#[derive(Debug, Default, Clone, Copy)]
pub struct ApParseStats {
    pub total_messages: usize,
    pub fmt_count: usize,
    pub corrupt_bytes: usize,
    pub unknown_types: usize,
    pub truncated: bool,
}
