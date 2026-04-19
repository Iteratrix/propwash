use std::collections::HashMap;

use az::{Az, SaturatingAs};

use crate::format::common::{ardupilot_filter_config, ardupilot_pid_gains};
use crate::types::{Axis, FilterConfig, MotorIndex, RcChannel, SensorField, Warning};

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

pub use crate::format::common::MsgColumns;

/// Complete raw data for one `ArduPilot` session.
#[derive(Debug)]
pub struct ApSession {
    /// Message type definitions (FMT messages), keyed by type ID.
    pub msg_defs: HashMap<u8, ApMsgDef>,
    /// Columnar data keyed by message type ID.
    pub topics: HashMap<u8, MsgColumns>,
    /// Firmware version string.
    pub firmware_version: String,
    /// Vehicle name / board type.
    pub vehicle_name: String,
    /// Parameters from PARM messages.
    pub params: HashMap<String, f64>,
    /// Parse statistics.
    pub stats: ApParseStats,
    /// Non-fatal diagnostics from parsing.
    pub warnings: Vec<Warning>,
    /// 1-based session index within the file.
    pub session_index: usize,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ApParseStats {
    pub total_messages: usize,
    pub fmt_count: usize,
    pub corrupt_bytes: usize,
    pub unknown_types: usize,
    pub truncated: bool,
}

impl ApSession {
    /// Looks up the type ID for a message name.
    pub fn msg_type_for_name(&self, name: &str) -> Option<u8> {
        self.msg_defs
            .values()
            .find(|d| d.name == name)
            .map(|d| d.msg_type)
    }

    /// Returns the timestamps for a message type by name.
    pub fn msg_timestamps(&self, name: &str) -> &[u64] {
        self.msg_type_for_name(name)
            .and_then(|t| self.topics.get(&t))
            .map_or(&[], |mc| &mc.timestamps)
    }

    /// Returns a column for a field within a message type.
    pub fn msg_column(&self, msg_name: &str, field_name: &str) -> Option<&[f64]> {
        let t = self.msg_type_for_name(msg_name)?;
        let mc = self.topics.get(&t)?;
        mc.column(field_name)
    }

    /// Returns field names for a message type.
    pub fn msg_field_names(&self, msg_name: &str) -> &[String] {
        self.msg_type_for_name(msg_name)
            .and_then(|t| self.topics.get(&t))
            .map_or(&[], |mc| &mc.field_names)
    }

    /// Returns the number of main frames (IMU or ACC messages).
    pub fn frame_count(&self) -> usize {
        let ts = self.msg_timestamps("IMU");
        if !ts.is_empty() {
            return ts.len();
        }
        self.msg_timestamps("ACC").len()
    }

    /// Returns field names available in this session.
    pub fn field_names(&self) -> Vec<String> {
        let has_gyro =
            !self.msg_timestamps("IMU").is_empty() || !self.msg_timestamps("GYR").is_empty();
        let has_accel =
            !self.msg_timestamps("IMU").is_empty() || !self.msg_timestamps("ACC").is_empty();
        let has_rc = !self.msg_timestamps("RCIN").is_empty();
        let n_motors = self.motor_count();

        let has_esc = self.msg_column("ESC", "RPM").is_some();

        Axis::ALL
            .iter()
            .filter(|_| has_gyro)
            .map(|&a| SensorField::Gyro(a).to_string())
            .chain(
                Axis::ALL
                    .iter()
                    .filter(|_| has_accel)
                    .map(|&a| SensorField::Accel(a).to_string()),
            )
            .chain((0..n_motors).map(|i| SensorField::Motor(MotorIndex(i)).to_string()))
            .chain(
                [
                    RcChannel::Roll,
                    RcChannel::Pitch,
                    RcChannel::Yaw,
                    RcChannel::Throttle,
                ]
                .iter()
                .filter(|_| has_rc)
                .map(|&ch| SensorField::Rc(ch).to_string()),
            )
            .chain(
                (0..n_motors)
                    .filter(|_| has_esc)
                    .map(|i| SensorField::ERpm(MotorIndex(i)).to_string()),
            )
            .collect()
    }

    pub fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    pub fn craft_name(&self) -> &str {
        &self.vehicle_name
    }

    pub fn sample_rate_hz(&self) -> f64 {
        let ts = self.msg_timestamps("IMU");
        if ts.len() >= 2 {
            let t0 = ts[0];
            let tn = ts[ts.len() - 1];
            let dt = tn.saturating_sub(t0);
            if dt > 0 {
                return (ts.len() - 1).az::<f64>() / (dt.az::<f64>() / 1_000_000.0);
            }
        }
        0.0
    }

    pub fn duration_seconds(&self) -> f64 {
        let mut min_t = u64::MAX;
        let mut max_t = 0u64;
        for mc in self.topics.values() {
            if let Some(&first) = mc.timestamps.first() {
                if first > 0 {
                    min_t = min_t.min(first);
                }
            }
            if let Some(&last) = mc.timestamps.last() {
                max_t = max_t.max(last);
            }
        }
        if min_t >= max_t {
            return 0.0;
        }
        (max_t - min_t).az::<f64>() / 1_000_000.0
    }

    #[allow(clippy::too_many_lines)]
    pub fn field(&self, field: &SensorField) -> Vec<f64> {
        match field {
            SensorField::Time => {
                let ts = self.msg_timestamps("IMU");
                ts.iter().map(|&t| t.az::<f64>()).collect()
            }
            SensorField::Gyro(axis) => {
                let field_name = match axis {
                    Axis::Roll => "GyrX",
                    Axis::Pitch => "GyrY",
                    Axis::Yaw => "GyrZ",
                };
                if let Some(col) = self.msg_column("IMU", field_name) {
                    return col.iter().map(|&v| v * 57.295_779_513_082_32).collect();
                }
                self.msg_column("GYR", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter().map(|&v| v * 57.295_779_513_082_32).collect()
                    })
            }
            SensorField::Accel(axis) => {
                let field_name = match axis {
                    Axis::Roll => "AccX",
                    Axis::Pitch => "AccY",
                    Axis::Yaw => "AccZ",
                };
                if let Some(col) = self.msg_column("IMU", field_name) {
                    return col.to_vec();
                }
                self.msg_column("ACC", field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Motor(idx) => {
                let field_name = format!("C{}", idx.0 + 1);
                self.msg_column("RCOU", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Rc(ch) => {
                let field_name = format!("C{}", ch.index() + 1);
                self.msg_column("RCIN", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Setpoint(axis) => {
                let field_name = match axis {
                    Axis::Roll => "RDes",
                    Axis::Pitch => "PDes",
                    Axis::Yaw => "YDes",
                };
                self.msg_column("RATE", field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Altitude => self
                .msg_column("BARO", "Alt")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::GpsSpeed => self
                .msg_column("GPS", "Spd")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::GpsLat => self
                .msg_column("GPS", "Lat")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::GpsLng => self
                .msg_column("GPS", "Lng")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::Heading => self
                .msg_column("ATT", "Yaw")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::ERpm(idx) => {
                // ESC messages have Instance field — filter to matching motor index
                let t = self.msg_type_for_name("ESC");
                let Some(mc) = t.and_then(|t| self.topics.get(&t)) else {
                    return Vec::new();
                };
                let Some(inst_col) = mc.column("Instance") else {
                    return Vec::new();
                };
                let Some(rpm_col) = mc.column("RPM") else {
                    return Vec::new();
                };
                let target = idx.0.az::<f64>();
                inst_col
                    .iter()
                    .zip(rpm_col.iter())
                    .filter(|(&inst, _)| (inst - target).abs() < 0.5)
                    .map(|(_, &rpm)| rpm)
                    .collect()
            }
            SensorField::GyroUnfilt(axis) => {
                let field_name = match axis {
                    Axis::Roll => "GyrX",
                    Axis::Pitch => "GyrY",
                    Axis::Yaw => "GyrZ",
                };
                self.msg_column("GYR", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter().map(|&v| v * 57.295_779_513_082_32).collect()
                    })
            }
            SensorField::Vbat => self
                .msg_column("BAT", "Volt")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::PidP(axis) => {
                let msg_name = match axis {
                    Axis::Roll => "PIDR",
                    Axis::Pitch => "PIDP",
                    Axis::Yaw => "PIDY",
                };
                self.msg_column(msg_name, "P")
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::PidI(axis) => {
                let msg_name = match axis {
                    Axis::Roll => "PIDR",
                    Axis::Pitch => "PIDP",
                    Axis::Yaw => "PIDY",
                };
                self.msg_column(msg_name, "I")
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::PidD(axis) => {
                let msg_name = match axis {
                    Axis::Roll => "PIDR",
                    Axis::Pitch => "PIDP",
                    Axis::Yaw => "PIDY",
                };
                self.msg_column(msg_name, "D")
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Feedforward(_) => Vec::new(),
            SensorField::Rssi => self
                .msg_column("RCIN", "RSSI")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::Unknown(name) => {
                if let Some((msg, fld)) = name.split_once('.') {
                    self.msg_column(msg, fld)
                        .map_or_else(Vec::new, <[f64]>::to_vec)
                } else {
                    Vec::new()
                }
            }
        }
    }

    pub fn motor_count(&self) -> usize {
        if let Some(&count) = self.params.get("MOT_PWM_COUNT") {
            return count.saturating_as::<usize>().min(8);
        }
        let mut count = 0;
        for i in 1..=14 {
            let key = format!("SERVO{i}_FUNCTION");
            if let Some(&func) = self.params.get(&key) {
                let f = func.az::<i32>();
                if (33..=40).contains(&f) {
                    count += 1;
                }
            }
        }
        if count > 0 {
            return count;
        }
        if self.firmware_version.contains("Copter") {
            4
        } else {
            0
        }
    }

    pub fn motor_range(&self) -> (f64, f64) {
        let min = self.params.get("MOT_PWM_MIN").copied().unwrap_or(1000.0);
        let max = self.params.get("MOT_PWM_MAX").copied().unwrap_or(2000.0);
        (min, max)
    }

    pub fn is_truncated(&self) -> bool {
        self.stats.truncated
    }

    pub fn corrupt_bytes(&self) -> usize {
        self.stats.corrupt_bytes
    }

    pub fn pid_gains(&self) -> crate::types::PidGains {
        ardupilot_pid_gains(&self.params)
    }

    pub fn filter_config(&self) -> FilterConfig {
        ardupilot_filter_config(&self.params)
    }

    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    pub fn index(&self) -> usize {
        self.session_index
    }
}
