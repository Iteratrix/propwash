use std::collections::HashMap;

use crate::types::{Axis, MotorIndex, RcChannel, SensorField, Unified};

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

/// A single field in a format definition.
#[derive(Debug, Clone)]
pub struct ULogField {
    pub name: String,
    pub type_name: String,
    pub primitive: Option<ULogType>,
    pub array_size: Option<usize>,
    pub byte_size: usize,
}

/// A format definition from an 'F' message.
#[derive(Debug, Clone)]
pub struct ULogFormat {
    pub name: String,
    pub fields: Vec<ULogField>,
    pub total_size: usize,
}

/// A subscription binding `msg_id` to a format.
#[derive(Debug, Clone)]
pub struct ULogSubscription {
    pub msg_id: u16,
    pub multi_id: u8,
    pub format_name: String,
}

/// A decoded data value.
#[derive(Debug, Clone)]
pub enum ULogValue {
    Int(i64),
    UInt(u64),
    Float(f64),
    Str(String),
}

impl ULogValue {
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

/// A decoded data message.
#[derive(Debug, Clone)]
pub struct ULogDataMsg {
    pub msg_id: u16,
    pub timestamp_us: u64,
    pub values: HashMap<String, ULogValue>,
}

/// Complete raw data for one `PX4` session.
#[derive(Debug)]
pub struct Px4RawSession {
    /// Format definitions keyed by name.
    pub formats: HashMap<String, ULogFormat>,
    /// Subscriptions keyed by `msg_id`.
    pub subscriptions: HashMap<u16, ULogSubscription>,
    /// All decoded data messages in file order.
    pub data_messages: Vec<ULogDataMsg>,
    /// Info key-value pairs.
    pub info: HashMap<String, String>,
    /// Parameters.
    pub params: HashMap<String, f64>,
    /// Firmware version string.
    pub firmware_version: String,
    /// Hardware name.
    pub hardware_name: String,
    /// Parse statistics.
    pub stats: Px4ParseStats,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Px4ParseStats {
    pub total_messages: usize,
    pub format_count: usize,
    pub data_count: usize,
    pub subscription_count: usize,
    pub dropout_count: usize,
    pub corrupt_bytes: usize,
}

impl Px4RawSession {
    /// Returns all data messages for a given topic name (e.g., `sensor_gyro`).
    pub fn topic_data(&self, topic_name: &str) -> Vec<&ULogDataMsg> {
        let msg_ids: Vec<u16> = self
            .subscriptions
            .values()
            .filter(|s| s.format_name == topic_name)
            .map(|s| s.msg_id)
            .collect();
        self.data_messages
            .iter()
            .filter(|d| msg_ids.contains(&d.msg_id))
            .collect()
    }

    /// Extracts a time-series of one field from one topic.
    fn extract_series(&self, topic: &str, field: &str) -> Vec<(u64, f64)> {
        self.topic_data(topic)
            .iter()
            .filter_map(|msg| {
                let val = msg.values.get(field)?;
                Some((msg.timestamp_us, val.as_f64()))
            })
            .collect()
    }
}

impl Unified for Px4RawSession {
    fn frame_count(&self) -> usize {
        // Use highest-rate gyro source as frame count
        let candidates = ["vehicle_angular_velocity", "sensor_combined", "sensor_gyro"];
        for topic in candidates {
            let count = self.topic_data(topic).len();
            if count > 0 {
                return count;
            }
        }
        0
    }

    fn field_names(&self) -> Vec<String> {
        let mut names = Vec::new();

        let has_gyro = !self.topic_data("sensor_combined").is_empty()
            || !self.topic_data("sensor_gyro").is_empty();
        let has_accel = !self.topic_data("sensor_combined").is_empty()
            || !self.topic_data("sensor_accel").is_empty();

        if has_gyro {
            for axis in Axis::ALL {
                names.push(SensorField::Gyro(axis).to_string());
            }
        }
        if has_accel {
            for axis in Axis::ALL {
                names.push(SensorField::Accel(axis).to_string());
            }
        }

        let n_motors = self.motor_count();
        for i in 0..n_motors {
            names.push(SensorField::Motor(MotorIndex(i)).to_string());
        }

        if !self.topic_data("input_rc").is_empty() {
            for ch in [
                RcChannel::Roll,
                RcChannel::Pitch,
                RcChannel::Yaw,
                RcChannel::Throttle,
            ] {
                names.push(SensorField::Rc(ch).to_string());
            }
        }

        names
    }

    fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    fn craft_name(&self) -> &str {
        &self.hardware_name
    }

    #[allow(clippy::cast_precision_loss)]
    fn sample_rate_hz(&self) -> f64 {
        let candidates = ["vehicle_angular_velocity", "sensor_combined", "sensor_gyro"];
        let mut series = Vec::new();
        for topic in candidates {
            series = self.extract_series(topic, "timestamp");
            if series.len() >= 2 {
                break;
            }
        }
        if series.len() < 2 {
            return 0.0;
        }
        let t0 = series.first().unwrap().0;
        let tn = series.last().unwrap().0;
        let dt = tn.saturating_sub(t0);
        if dt == 0 {
            return 0.0;
        }
        (series.len() - 1) as f64 / (dt as f64 / 1_000_000.0)
    }

    #[allow(clippy::cast_precision_loss)]
    fn duration_seconds(&self) -> f64 {
        let mut min_t = u64::MAX;
        let mut max_t = 0u64;
        for m in &self.data_messages {
            if m.timestamp_us > 0 {
                min_t = min_t.min(m.timestamp_us);
                max_t = max_t.max(m.timestamp_us);
            }
        }
        if min_t >= max_t {
            return 0.0;
        }
        (max_t - min_t) as f64 / 1_000_000.0
    }

    #[allow(
        clippy::cast_precision_loss,
        clippy::too_many_lines,
        clippy::cast_possible_truncation
    )]
    fn field(&self, field: &SensorField) -> Vec<f64> {
        match field {
            SensorField::Time => {
                let candidates = ["vehicle_angular_velocity", "sensor_combined", "sensor_gyro"];
                for topic in candidates {
                    let series = self.extract_series(topic, "timestamp");
                    if !series.is_empty() {
                        return series.into_iter().map(|(_, v)| v).collect();
                    }
                }
                Vec::new()
            }
            // PX4 gyro is in rad/s — convert to deg/s (rounded to nearest integer)
            SensorField::Gyro(axis) => {
                let idx = axis.index();
                // Try sensor_combined (gyro_rad[N]), then vehicle_angular_velocity (xyz[N]),
                // then sensor_gyro (x/y/z)
                let mut series =
                    self.extract_series("sensor_combined", &format!("gyro_rad[{idx}]"));
                if series.is_empty() {
                    series =
                        self.extract_series("vehicle_angular_velocity", &format!("xyz[{idx}]"));
                }
                if series.is_empty() {
                    let field_name = match axis {
                        Axis::Roll => "x",
                        Axis::Pitch => "y",
                        Axis::Yaw => "z",
                    };
                    series = self.extract_series("sensor_gyro", field_name);
                }
                series
                    .into_iter()
                    .map(|(_, v)| v * 57.295_779_513_082_32)
                    .collect()
            }
            SensorField::Accel(axis) => {
                let idx = axis.index();
                let mut series =
                    self.extract_series("sensor_combined", &format!("accelerometer_m_s2[{idx}]"));
                if series.is_empty() {
                    let field_name = match axis {
                        Axis::Roll => "x",
                        Axis::Pitch => "y",
                        Axis::Yaw => "z",
                    };
                    series = self.extract_series("sensor_accel", field_name);
                }
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Motor(idx) => {
                let field_name = format!("output[{}]", idx.0);
                let series = self.extract_series("actuator_outputs", &field_name);
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Rc(ch) => {
                let field_name = format!("values[{}]", ch.index());
                let series = self.extract_series("input_rc", &field_name);
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Altitude => {
                let series = self.extract_series("vehicle_global_position", "alt");
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::GpsLat => {
                let series = self.extract_series("vehicle_gps_position", "lat");
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::GpsLng => {
                let series = self.extract_series("vehicle_gps_position", "lon");
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Vbat => {
                let series = self.extract_series("battery_status", "voltage_v");
                series.into_iter().map(|(_, v)| v).collect()
            }
            SensorField::Unknown(name) => {
                if let Some((topic, fld)) = name.split_once('.') {
                    self.extract_series(topic, fld)
                        .into_iter()
                        .map(|(_, v)| v)
                        .collect()
                } else {
                    Vec::new()
                }
            }
            _ => Vec::new(),
        }
    }

    fn motor_count(&self) -> usize {
        // Use MOT_COUNT parameter if available
        if let Some(&count) = self.params.get("MOT_COUNT") {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            return (count as usize).min(8);
        }
        // Count actuator_outputs fields with non-zero values in first message
        let msgs = self.topic_data("actuator_outputs");
        if let Some(msg) = msgs.first() {
            let mut count = 0;
            for i in 0..16 {
                let key = format!("output[{i}]");
                match msg.values.get(&key) {
                    Some(v) if v.as_f64().abs() > 0.0 => count += 1,
                    _ => break,
                }
            }
            count
        } else {
            0
        }
    }
}
