use std::collections::HashMap;

use crate::types::{Axis, MotorIndex, RcChannel, SensorField, Unified};

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

    /// Wire size in bytes for this field type.
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
    /// Message type ID (0-255).
    pub msg_type: u8,
    /// Message name (e.g., "IMU", "ATT", "GPS").
    pub name: String,
    /// Field types in order.
    pub field_types: Vec<FieldType>,
    /// Field names in order.
    pub field_names: Vec<String>,
    /// Total message length including 3-byte header.
    pub msg_len: usize,
    /// Original format string from FMT.
    pub format_str: String,
}

/// A single decoded field value.
#[derive(Debug, Clone)]
pub enum ApValue {
    Int(i64),
    UInt(u64),
    Float(f64),
    Str(String),
}

impl ApValue {
    pub fn as_i64(&self) -> i64 {
        match self {
            Self::Int(v) => *v,
            #[allow(clippy::cast_possible_wrap)]
            Self::UInt(v) => *v as i64,
            #[allow(clippy::cast_possible_truncation)]
            Self::Float(v) => *v as i64,
            Self::Str(_) => 0,
        }
    }

    #[allow(clippy::cast_precision_loss)]
    pub fn as_f64(&self) -> f64 {
        match self {
            Self::Int(v) => *v as f64,
            Self::UInt(v) => *v as f64,
            Self::Float(v) => *v,
            Self::Str(_) => 0.0,
        }
    }
}

/// A single decoded message with its field values.
#[derive(Debug, Clone)]
pub struct ApMessage {
    /// Message type ID.
    pub msg_type: u8,
    /// Timestamp in microseconds (from TimeUS field, if present).
    pub time_us: u64,
    /// Field values in order matching the FMT definition.
    pub values: Vec<ApValue>,
}

/// Complete raw data for one ArduPilot session.
#[derive(Debug)]
pub struct ApRawSession {
    /// Message type definitions (FMT messages), keyed by type ID.
    pub msg_defs: HashMap<u8, ApMsgDef>,
    /// All decoded messages, in file order.
    pub messages: Vec<ApMessage>,
    /// Firmware version string.
    pub firmware_version: String,
    /// Vehicle name / board type.
    pub vehicle_name: String,
    /// Parameters from PARM messages.
    pub params: HashMap<String, f64>,
    /// Parse statistics.
    pub stats: ApParseStats,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ApParseStats {
    pub total_messages: usize,
    pub fmt_count: usize,
    pub corrupt_bytes: usize,
    pub unknown_types: usize,
}

impl ApRawSession {
    /// Returns messages of a given type name.
    pub fn messages_by_name(&self, name: &str) -> Vec<&ApMessage> {
        let Some(msg_type) = self.msg_type_for_name(name) else {
            return Vec::new();
        };
        self.messages.iter().filter(|m| m.msg_type == msg_type).collect()
    }

    /// Looks up the type ID for a message name.
    pub fn msg_type_for_name(&self, name: &str) -> Option<u8> {
        self.msg_defs.values().find(|d| d.name == name).map(|d| d.msg_type)
    }

    /// Looks up the field index within a message type by field name.
    pub fn field_index(&self, msg_type: u8, field_name: &str) -> Option<usize> {
        self.msg_defs.get(&msg_type)?
            .field_names.iter().position(|n| n == field_name)
    }

    /// Extracts a time-series of one field from one message type.
    fn extract_series(&self, msg_name: &str, field_name: &str) -> Vec<(u64, i64)> {
        let Some(msg_type) = self.msg_type_for_name(msg_name) else {
            return Vec::new();
        };
        let Some(field_idx) = self.field_index(msg_type, field_name) else {
            return Vec::new();
        };
        self.messages
            .iter()
            .filter(|m| m.msg_type == msg_type)
            .map(|m| {
                let val = m.values.get(field_idx).map_or(0, ApValue::as_i64);
                (m.time_us, val)
            })
            .collect()
    }
}

impl Unified for ApRawSession {
    fn frame_count(&self) -> usize {
        // Use IMU message count as the "frame count" — it's the primary sensor stream
        let imu_type = self.msg_type_for_name("IMU")
            .or_else(|| self.msg_type_for_name("ACC"));
        match imu_type {
            Some(t) => self.messages.iter().filter(|m| m.msg_type == t).count(),
            None => 0,
        }
    }

    fn field_names(&self) -> Vec<String> {
        // Return SensorField-style names for the fields we can map
        let mut names = Vec::new();
        if self.msg_type_for_name("IMU").is_some() || self.msg_type_for_name("GYR").is_some() {
            for axis in Axis::ALL {
                names.push(SensorField::Gyro(axis).to_string());
            }
        }
        if self.msg_type_for_name("IMU").is_some() || self.msg_type_for_name("ACC").is_some() {
            for axis in Axis::ALL {
                names.push(SensorField::Accel(axis).to_string());
            }
        }
        if self.msg_type_for_name("RCOU").is_some() {
            for i in 0..8 {
                let msg_type = self.msg_type_for_name("RCOU").unwrap();
                let field = format!("C{}", i + 1);
                if self.field_index(msg_type, &field).is_some() {
                    names.push(SensorField::Motor(MotorIndex(i)).to_string());
                }
            }
        }
        if self.msg_type_for_name("RCIN").is_some() {
            for (i, ch) in [RcChannel::Roll, RcChannel::Pitch, RcChannel::Yaw, RcChannel::Throttle].iter().enumerate() {
                let msg_type = self.msg_type_for_name("RCIN").unwrap();
                let field = format!("C{}", i + 1);
                if self.field_index(msg_type, &field).is_some() {
                    names.push(SensorField::Rc(*ch).to_string());
                }
            }
        }
        names
    }

    fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    fn craft_name(&self) -> &str {
        &self.vehicle_name
    }

    #[allow(clippy::cast_precision_loss)]
    fn sample_rate_hz(&self) -> f64 {
        // Estimate from IMU timestamps
        let series = self.extract_series("IMU", "TimeUS");
        if series.len() < 2 {
            return 0.0;
        }
        let t0 = series.first().unwrap().0;
        let tn = series.last().unwrap().0;
        let dt_us = tn.saturating_sub(t0);
        if dt_us == 0 {
            return 0.0;
        }
        (series.len() - 1) as f64 / (dt_us as f64 / 1_000_000.0)
    }

    #[allow(clippy::cast_precision_loss)]
    fn duration_seconds(&self) -> f64 {
        // Duration from first to last message timestamp
        let mut min_t = u64::MAX;
        let mut max_t = 0u64;
        for m in &self.messages {
            if m.time_us > 0 {
                min_t = min_t.min(m.time_us);
                max_t = max_t.max(m.time_us);
            }
        }
        if min_t >= max_t {
            return 0.0;
        }
        (max_t - min_t) as f64 / 1_000_000.0
    }

    #[allow(clippy::cast_precision_loss)]
    fn field(&self, field: &SensorField) -> Vec<i64> {
        match field {
            SensorField::Time => {
                let series = self.extract_series("IMU", "TimeUS");
                series.into_iter().map(|(t, _)| {
                    #[allow(clippy::cast_possible_wrap)]
                    { t as i64 }
                }).collect()
            }
            // ArduPilot IMU gyro is in rad/s — convert to deg/s to match Betaflight
            SensorField::Gyro(axis) => {
                let field_name = match axis {
                    Axis::Roll => "GyrX",
                    Axis::Pitch => "GyrY",
                    Axis::Yaw => "GyrZ",
                };
                // Try IMU first (lower rate), then GYR (raw, higher rate)
                let mut series = self.extract_series("IMU", field_name);
                if series.is_empty() {
                    series = self.extract_series("GYR", field_name);
                }
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Accel(axis) => {
                let field_name = match axis {
                    Axis::Roll => "AccX",
                    Axis::Pitch => "AccY",
                    Axis::Yaw => "AccZ",
                };
                let mut series = self.extract_series("IMU", field_name);
                if series.is_empty() {
                    series = self.extract_series("ACC", field_name);
                }
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Motor(idx) => {
                let field_name = format!("C{}", idx.0 + 1);
                let series = self.extract_series("RCOU", &field_name);
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Rc(ch) => {
                let field_name = format!("C{}", ch.index() + 1);
                let series = self.extract_series("RCIN", &field_name);
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Setpoint(axis) => {
                let field_name = match axis {
                    Axis::Roll => "RDes",
                    Axis::Pitch => "PDes",
                    Axis::Yaw => "YDes",
                };
                let series = self.extract_series("RATE", field_name);
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Unknown(name) => {
                // Try to parse "MSGNAME.FieldName" format
                if let Some((msg, fld)) = name.split_once('.') {
                    let series = self.extract_series(msg, fld);
                    series.into_iter().map(|(_, v)| v).collect()
                } else {
                    Vec::new()
                }
            }
            _ => Vec::new(),
        }
    }

    fn motor_count(&self) -> usize {
        let Some(msg_type) = self.msg_type_for_name("RCOU") else {
            return 0;
        };
        let Some(def) = self.msg_defs.get(&msg_type) else {
            return 0;
        };
        // Count C1..C14 fields that exist
        def.field_names.iter().filter(|n| n.starts_with('C') && n[1..].parse::<usize>().is_ok()).count()
    }
}
