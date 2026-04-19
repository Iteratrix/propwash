use std::collections::HashMap;

use az::{Az, SaturatingAs};

use crate::types::{Axis, FilterConfig, MotorIndex, RcChannel, SensorField, Warning};

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

/// Columnar storage for one subscription's data messages.
///
/// All column vectors have the same length as `timestamps`.
#[derive(Debug)]
pub struct TopicData {
    /// Timestamps in microseconds, one per message.
    pub timestamps: Vec<u64>,
    /// Parallel column vectors of decoded field values.
    pub columns: Vec<Vec<f64>>,
    /// Field names, parallel to `columns`.
    pub field_names: Vec<String>,
    /// name -> column index for O(1) field lookup.
    field_index: HashMap<String, usize>,
}

impl TopicData {
    pub(crate) fn new(field_names: Vec<String>) -> Self {
        let field_index = field_names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.clone(), i))
            .collect();
        let columns = vec![Vec::new(); field_names.len()];
        Self {
            timestamps: Vec::new(),
            columns,
            field_names,
            field_index,
        }
    }

    /// Returns the column for a given field name.
    pub fn column(&self, field: &str) -> Option<&[f64]> {
        let idx = self.field_index.get(field)?;
        Some(&self.columns[*idx])
    }

    /// Pushes one row of decoded values into the columns.
    #[inline]
    pub(crate) fn push_row(&mut self, timestamp: u64, values: &[f64]) {
        self.timestamps.push(timestamp);
        for (col, &val) in self.columns.iter_mut().zip(values.iter()) {
            col.push(val);
        }
    }
}

/// A log message emitted by PX4 firmware (debug/warning/error).
#[derive(Debug, Clone)]
pub struct ULogLogMessage {
    /// Log level (0=emerg, 1=alert, 2=crit, 3=err, 4=warn, 5=notice, 6=info, 7=debug).
    pub level: u8,
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// Message text.
    pub message: String,
    /// Tag value (only set for tagged logging messages).
    pub tag: Option<u16>,
}

/// Complete raw data for one `PX4` session.
#[derive(Debug)]
pub struct Px4Session {
    /// Format definitions keyed by name.
    pub formats: HashMap<String, ULogFormat>,
    /// Subscriptions keyed by `msg_id`.
    pub subscriptions: HashMap<u16, ULogSubscription>,
    /// Columnar data keyed by `msg_id`.
    pub topics: HashMap<u16, TopicData>,
    /// Info key-value pairs.
    pub info: HashMap<String, String>,
    /// Parameters.
    pub params: HashMap<String, f64>,
    /// Log messages (debug/warning/error strings from firmware).
    pub log_messages: Vec<ULogLogMessage>,
    /// Firmware version string.
    pub firmware_version: String,
    /// Hardware name.
    pub hardware_name: String,
    /// Parse statistics.
    pub stats: Px4ParseStats,
    /// Non-fatal diagnostics from parsing.
    pub warnings: Vec<Warning>,
    /// 1-based session index within the file.
    pub session_index: usize,
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

impl Px4Session {
    /// Returns the `msg_id` for the primary (lowest `multi_id`) instance of a topic.
    fn primary_msg_id(&self, topic_name: &str) -> Option<u16> {
        self.subscriptions
            .values()
            .filter(|s| s.format_name == topic_name)
            .min_by_key(|s| s.multi_id)
            .map(|s| s.msg_id)
    }

    /// Returns whether the primary instance of a topic has data.
    fn has_topic(&self, topic_name: &str) -> bool {
        self.primary_msg_id(topic_name)
            .and_then(|id| self.topics.get(&id))
            .is_some_and(|td| !td.timestamps.is_empty())
    }

    /// Returns the timestamps for the primary instance of a topic.
    pub fn topic_timestamps(&self, topic_name: &str) -> &[u64] {
        self.primary_msg_id(topic_name)
            .and_then(|id| self.topics.get(&id))
            .map_or(&[], |td| &td.timestamps)
    }

    /// Returns the column data for a field in the primary instance of a topic.
    pub fn topic_column(&self, topic_name: &str, field: &str) -> Option<&[f64]> {
        let id = self.primary_msg_id(topic_name)?;
        let td = self.topics.get(&id)?;
        td.column(field)
    }

    /// Returns the field names available for the primary instance of a topic.
    pub fn topic_field_names(&self, topic_name: &str) -> &[String] {
        self.primary_msg_id(topic_name)
            .and_then(|id| self.topics.get(&id))
            .map_or(&[], |td| &td.field_names)
    }

    /// Returns the total message count across all instances of a topic.
    pub fn topic_all_instances_count(&self, topic_name: &str) -> usize {
        self.subscriptions
            .values()
            .filter(|s| s.format_name == topic_name)
            .filter_map(|s| self.topics.get(&s.msg_id))
            .map(|td| td.timestamps.len())
            .sum()
    }

    /// Returns the number of main frames from the highest-rate gyro source.
    pub fn frame_count(&self) -> usize {
        let candidates = ["vehicle_angular_velocity", "sensor_combined", "sensor_gyro"];
        for topic in candidates {
            let len = self.topic_timestamps(topic).len();
            if len > 0 {
                return len;
            }
        }
        0
    }

    /// Returns field names available in this session.
    pub fn field_names(&self) -> Vec<String> {
        let has_gyro = self.has_topic("sensor_combined") || self.has_topic("sensor_gyro");
        let has_accel = self.has_topic("sensor_combined") || self.has_topic("sensor_accel");
        let has_rc = self.has_topic("input_rc");
        let n_motors = self.motor_count();

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
            .collect()
    }

    /// Returns the firmware version string.
    pub fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    /// Returns the hardware name.
    pub fn craft_name(&self) -> &str {
        &self.hardware_name
    }

    /// Computes sample rate from gyro timestamps.
    pub fn sample_rate_hz(&self) -> f64 {
        let candidates = ["vehicle_angular_velocity", "sensor_combined", "sensor_gyro"];
        for topic in candidates {
            let ts = self.topic_timestamps(topic);
            if ts.len() >= 2 {
                let dt = ts[ts.len() - 1].saturating_sub(ts[0]);
                if dt > 0 {
                    return (ts.len() - 1).az::<f64>() / (dt.az::<f64>() / 1_000_000.0);
                }
            }
        }
        0.0
    }

    /// Returns flight duration in seconds.
    pub fn duration_seconds(&self) -> f64 {
        let mut min_t = u64::MAX;
        let mut max_t = 0u64;
        for td in self.topics.values() {
            if let Some(&first) = td.timestamps.first() {
                if first > 0 {
                    min_t = min_t.min(first);
                }
            }
            if let Some(&last) = td.timestamps.last() {
                max_t = max_t.max(last);
            }
        }
        if min_t >= max_t {
            return 0.0;
        }
        (max_t - min_t).az::<f64>() / 1_000_000.0
    }

    /// Extracts one field as a `Vec<f64>` across all relevant messages.
    #[allow(clippy::too_many_lines)]
    pub fn field(&self, field: &SensorField) -> Vec<f64> {
        match field {
            SensorField::Time => {
                let candidates = ["vehicle_angular_velocity", "sensor_combined", "sensor_gyro"];
                for topic in candidates {
                    let ts = self.topic_timestamps(topic);
                    if !ts.is_empty() {
                        return ts.iter().map(|&t| t.az::<f64>()).collect();
                    }
                }
                Vec::new()
            }
            SensorField::Gyro(axis) => {
                let idx = axis.index();
                let try_topics = [
                    ("sensor_combined", format!("gyro_rad[{idx}]")),
                    ("vehicle_angular_velocity", format!("xyz[{idx}]")),
                    (
                        "sensor_gyro",
                        match axis {
                            Axis::Roll => "x",
                            Axis::Pitch => "y",
                            Axis::Yaw => "z",
                        }
                        .to_string(),
                    ),
                ];
                for (topic, field_name) in &try_topics {
                    if let Some(col) = self.topic_column(topic, field_name) {
                        return col.iter().map(|&v| v * 57.295_779_513_082_32).collect();
                    }
                }
                Vec::new()
            }
            SensorField::Accel(axis) => {
                let idx = axis.index();
                if let Some(col) =
                    self.topic_column("sensor_combined", &format!("accelerometer_m_s2[{idx}]"))
                {
                    return col.to_vec();
                }
                let field_name = match axis {
                    Axis::Roll => "x",
                    Axis::Pitch => "y",
                    Axis::Yaw => "z",
                };
                self.topic_column("sensor_accel", field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Motor(idx) => {
                let field_name = format!("output[{}]", idx.0);
                self.topic_column("actuator_outputs", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Rc(ch) => {
                let field_name = format!("values[{}]", ch.index());
                self.topic_column("input_rc", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Altitude => self
                .topic_column("vehicle_global_position", "alt")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::GpsLat => self
                .topic_column("vehicle_gps_position", "lat")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::GpsLng => self
                .topic_column("vehicle_gps_position", "lon")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::Vbat => self
                .topic_column("battery_status", "voltage_v")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::Heading => self
                .topic_column("vehicle_global_position", "yaw")
                .map_or_else(Vec::new, |col| {
                    col.iter().map(|&v| v * 57.295_779_513_082_32).collect()
                }),
            SensorField::GpsSpeed => {
                // Compute ground speed from vel_n and vel_e
                let vn = self.topic_column("vehicle_global_position", "vel_n");
                let ve = self.topic_column("vehicle_global_position", "vel_e");
                match (vn, ve) {
                    (Some(vn), Some(ve)) => vn
                        .iter()
                        .zip(ve.iter())
                        .map(|(&n, &e)| n.hypot(e))
                        .collect(),
                    _ => Vec::new(),
                }
            }
            SensorField::Unknown(name) => {
                if let Some((topic, fld)) = name.split_once('.') {
                    self.topic_column(topic, fld)
                        .map_or_else(Vec::new, <[f64]>::to_vec)
                } else {
                    Vec::new()
                }
            }
            SensorField::ERpm(idx) => {
                let field_name = format!("esc[{}].esc_rpm", idx.0);
                self.topic_column("esc_status", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::GyroUnfilt(axis) => {
                let field_name = match axis {
                    Axis::Roll => "x",
                    Axis::Pitch => "y",
                    Axis::Yaw => "z",
                };
                self.topic_column("sensor_gyro", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter().map(|&v| v * 57.295_779_513_082_32).collect()
                    })
            }
            SensorField::Setpoint(axis) => {
                let field_name = match axis {
                    Axis::Roll => "roll",
                    Axis::Pitch => "pitch",
                    Axis::Yaw => "yaw",
                };
                self.topic_column("vehicle_rates_setpoint", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter().map(|&v| v * 57.295_779_513_082_32).collect()
                    })
            }
            SensorField::PidP(_)
            | SensorField::PidI(_)
            | SensorField::PidD(_)
            | SensorField::Feedforward(_) => Vec::new(),
            SensorField::Rssi => {
                // PX4 input_rc has an rssi field (int32)
                self.topic_column("input_rc", "rssi")
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
        }
    }

    /// Returns the number of motors detected from parameters or data.
    pub fn motor_count(&self) -> usize {
        if let Some(&count) = self.params.get("MOT_COUNT") {
            return count.saturating_as::<usize>().min(8);
        }
        let id = self.primary_msg_id("actuator_outputs");
        if let Some(td) = id.and_then(|id| self.topics.get(&id)) {
            let mut count = 0;
            for i in 0..16 {
                let key = format!("output[{i}]");
                if let Some(col) = td.column(&key) {
                    if col.first().is_some_and(|&v| v.abs() > 0.0) {
                        count += 1;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
            count
        } else {
            0
        }
    }

    /// Returns the motor output range `(min, max)` detected from data.
    pub fn motor_range(&self) -> (f64, f64) {
        let id = self.primary_msg_id("actuator_outputs");
        if let Some(td) = id.and_then(|id| self.topics.get(&id)) {
            let max_val = (0..16)
                .filter_map(|i| td.column(&format!("output[{i}]")))
                .flat_map(|col| col.iter())
                .fold(0.0_f64, |a, &b| a.max(b));
            if max_val > 10.0 {
                (1000.0, 2000.0)
            } else {
                (0.0, 1.0)
            }
        } else {
            (0.0, 1.0)
        }
    }

    /// Returns whether the log appears truncated (ended mid-message).
    pub fn is_truncated(&self) -> bool {
        self.stats.truncated
    }

    /// Returns the number of corrupt bytes encountered during parsing.
    pub fn corrupt_bytes(&self) -> usize {
        self.stats.corrupt_bytes
    }

    pub fn pid_gains(&self) -> crate::types::PidGains {
        let parse = |p_key: &str, i_key: &str, d_key: &str| -> crate::types::AxisGains {
            let get = |k: &str| -> Option<u32> {
                self.params
                    .get(k)
                    .copied()
                    .filter(|&v| v > 0.0)
                    .map(|v| (v * 1000.0).saturating_as::<u32>())
            };
            crate::types::AxisGains {
                p: get(p_key),
                i: get(i_key),
                d: get(d_key),
            }
        };
        crate::types::PidGains::new(
            parse("MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D"),
            parse("MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D"),
            parse("MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D"),
        )
    }

    /// Returns the filter configuration extracted from parameters.
    pub fn filter_config(&self) -> FilterConfig {
        let p = |k: &str| self.params.get(k).copied().unwrap_or(0.0);
        let non_zero = |v: f64| -> Option<f64> {
            if v > 0.0 {
                Some(v)
            } else {
                None
            }
        };
        FilterConfig {
            gyro_lpf_hz: non_zero(p("IMU_GYRO_CUTOFF")),
            gyro_lpf2_hz: None,
            dterm_lpf_hz: non_zero(p("IMU_DGYRO_CUTOFF")),
            dyn_notch_min_hz: None,
            dyn_notch_max_hz: None,
            gyro_notch1_hz: non_zero(p("IMU_GYRO_NF0_FRQ")),
            gyro_notch2_hz: non_zero(p("IMU_GYRO_NF1_FRQ")),
        }
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
