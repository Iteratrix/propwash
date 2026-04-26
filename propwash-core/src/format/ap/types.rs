//! AP parser-internal data types: FMT field types, message definitions.
//!
//! No `ApSession` here — the parser writes directly into
//! [`crate::session::Session`] via [`super::build`].
#![allow(dead_code)]

/// Format character from FMT message — determines wire size and interpretation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldType {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
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
            b'h' | b'c' => Self::I16,
            b'H' | b'C' => Self::U16,
            b'i' | b'e' | b'L' => Self::I32,
            b'I' | b'E' => Self::U32,
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
            Self::I16 | Self::U16 | Self::Float16 => 2,
            Self::I32 | Self::U32 | Self::Float | Self::Char4 => 4,
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
