//! PX4 parser-internal data types: `ULog` field types, format definitions,
//! subscriptions, log messages, parse stats.
//!
//! No `Px4Session` here — the parser writes directly into
//! [`crate::session::Session`] via [`super::build`].
#![allow(dead_code)]

/// Primitive types in the `ULog` type system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ULogType {
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    Float,
    Double,
    Bool,
    Char,
}

impl ULogType {
    pub fn size(self) -> usize {
        match self {
            Self::Int8 | Self::UInt8 | Self::Bool | Self::Char => 1,
            Self::Int16 | Self::UInt16 => 2,
            Self::Int32 | Self::UInt32 | Self::Float => 4,
            Self::Int64 | Self::UInt64 | Self::Double => 8,
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "int8_t" => Some(Self::Int8),
            "uint8_t" => Some(Self::UInt8),
            "int16_t" => Some(Self::Int16),
            "uint16_t" => Some(Self::UInt16),
            "int32_t" => Some(Self::Int32),
            "uint32_t" => Some(Self::UInt32),
            "int64_t" => Some(Self::Int64),
            "uint64_t" => Some(Self::UInt64),
            "float" => Some(Self::Float),
            "double" => Some(Self::Double),
            "bool" => Some(Self::Bool),
            "char" => Some(Self::Char),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ULogField {
    pub name: String,
    pub type_name: String,
    pub primitive: Option<ULogType>,
    pub array_size: Option<usize>,
    pub byte_size: usize,
}

#[derive(Debug, Clone)]
pub struct ULogFormat {
    pub name: String,
    pub fields: Vec<ULogField>,
    pub total_size: usize,
}

#[derive(Debug, Clone)]
pub struct ULogSubscription {
    pub msg_id: u16,
    pub multi_id: u8,
    pub format_name: String,
}

pub type TopicData = crate::format::common::MsgColumns;

#[derive(Debug, Clone)]
pub struct ULogLogMessage {
    pub level: u8,
    pub timestamp_us: u64,
    pub message: String,
    pub tag: Option<u16>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Px4ParseStats {
    pub total_messages: usize,
    pub format_count: usize,
    pub data_count: usize,
    pub subscription_count: usize,
    pub dropout_count: usize,
    pub corrupt_bytes: usize,
    pub truncated: bool,
}
