use std::collections::HashMap;

use crate::types::{Axis, FilterConfig, MotorIndex, RcChannel, SensorField, Warning};

/// Columnar storage for one `MAVLink` message type.
///
/// All column vectors have the same length as `timestamps`.
#[derive(Debug)]
pub struct MsgColumns {
    /// Timestamps in microseconds (boot-relative or tlog-receive, depending on message type).
    pub timestamps: Vec<u64>,
    /// Parallel column vectors of decoded field values.
    pub columns: Vec<Vec<f64>>,
    /// Field names, parallel to `columns`.
    pub field_names: Vec<String>,
    /// name → column index for O(1) field lookup.
    field_index: HashMap<String, usize>,
}

impl MsgColumns {
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

/// `MAVLink` `MAV_TYPE` — vehicle type from HEARTBEAT.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MavType {
    #[default]
    Generic,
    FixedWing,
    Quadrotor,
    CoaxialHelicopter,
    Helicopter,
    AntennaTracker,
    Gcs,
    Airship,
    GroundRover,
    SurfaceBoat,
    Submarine,
    Hexarotor,
    Octorotor,
    Tricopter,
    VtolTiltrotor,
    Vtol,
    Unknown(u8),
}

impl MavType {
    pub fn from_id(id: u8) -> Self {
        match id {
            0 => Self::Generic,
            1 => Self::FixedWing,
            2 => Self::Quadrotor,
            3 => Self::CoaxialHelicopter,
            4 => Self::Helicopter,
            5 => Self::AntennaTracker,
            6 => Self::Gcs,
            7 => Self::Airship,
            10 => Self::GroundRover,
            11 => Self::SurfaceBoat,
            12 => Self::Submarine,
            13 => Self::Hexarotor,
            14 => Self::Octorotor,
            15 => Self::Tricopter,
            19 => Self::VtolTiltrotor,
            20 => Self::Vtol,
            other => Self::Unknown(other),
        }
    }

    pub fn motor_count(self) -> Option<usize> {
        match self {
            Self::Quadrotor => Some(4),
            Self::Hexarotor => Some(6),
            Self::Octorotor => Some(8),
            Self::Tricopter => Some(3),
            Self::Helicopter | Self::FixedWing => Some(1),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Generic => "Generic",
            Self::FixedWing => "Fixed-wing",
            Self::Quadrotor => "Quadrotor",
            Self::CoaxialHelicopter => "Coaxial helicopter",
            Self::Helicopter => "Helicopter",
            Self::AntennaTracker => "Antenna tracker",
            Self::Gcs => "GCS",
            Self::Airship => "Airship",
            Self::GroundRover => "Ground rover",
            Self::SurfaceBoat => "Surface boat",
            Self::Submarine => "Submarine",
            Self::Hexarotor => "Hexarotor",
            Self::Octorotor => "Octorotor",
            Self::Tricopter => "Tricopter",
            Self::VtolTiltrotor => "VTOL tiltrotor",
            Self::Vtol => "VTOL",
            Self::Unknown(_) => "Unknown",
        }
    }
}

/// `MAVLink` message severity level from STATUSTEXT.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Emergency = 0,
    Alert = 1,
    Critical = 2,
    Error = 3,
    Warning = 4,
    Notice = 5,
    Info = 6,
    Debug = 7,
}

impl Severity {
    pub fn from_id(id: u8) -> Self {
        match id {
            0 => Self::Emergency,
            1 => Self::Alert,
            2 => Self::Critical,
            3 => Self::Error,
            4 => Self::Warning,
            5 => Self::Notice,
            6 => Self::Info,
            _ => Self::Debug,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Emergency => "emergency",
            Self::Alert => "alert",
            Self::Critical => "critical",
            Self::Error => "error",
            Self::Warning => "warning",
            Self::Notice => "notice",
            Self::Info => "info",
            Self::Debug => "debug",
        }
    }
}

/// A STATUSTEXT message captured during parsing.
#[derive(Debug, Clone)]
pub struct StatusMessage {
    pub timestamp_us: u64,
    pub severity: Severity,
    pub text: String,
}

/// Parse statistics for a `MAVLink` tlog.
#[derive(Debug, Default, Clone, Copy)]
pub struct MavlinkParseStats {
    pub total_packets: usize,
    pub v1_packets: usize,
    pub v2_packets: usize,
    pub crc_errors: usize,
    pub corrupt_bytes: usize,
    pub truncated: bool,
}

/// Complete raw data for one `MAVLink` telemetry log session.
#[derive(Debug)]
pub struct MavlinkSession {
    /// Columnar data keyed by message name (e.g., `ATTITUDE`, `RAW_IMU`).
    pub topics: HashMap<String, MsgColumns>,
    /// Firmware version string extracted from STATUSTEXT.
    pub firmware_version: String,
    /// Vehicle type from HEARTBEAT.
    pub vehicle_type: MavType,
    /// Parameters from `PARAM_VALUE` messages.
    pub params: HashMap<String, f64>,
    /// Captured STATUSTEXT messages.
    pub status_messages: Vec<StatusMessage>,
    /// Parse statistics.
    pub stats: MavlinkParseStats,
    /// Non-fatal diagnostics from parsing.
    pub warnings: Vec<Warning>,
    /// 1-based session index within the file.
    pub session_index: usize,
}

impl MavlinkSession {
    /// Returns a column for a field within a message type.
    pub fn msg_column(&self, msg_name: &str, field_name: &str) -> Option<&[f64]> {
        self.topics.get(msg_name)?.column(field_name)
    }

    /// Returns timestamps for a message type.
    pub fn msg_timestamps(&self, msg_name: &str) -> &[u64] {
        self.topics.get(msg_name).map_or(&[], |mc| &mc.timestamps)
    }

    // -- Unified interface methods --

    pub fn frame_count(&self) -> usize {
        self.msg_timestamps("ATTITUDE").len()
    }

    pub fn field_names(&self) -> Vec<String> {
        let has_attitude = !self.msg_timestamps("ATTITUDE").is_empty();
        let has_imu = !self.msg_timestamps("RAW_IMU").is_empty()
            || !self.msg_timestamps("SCALED_IMU").is_empty();
        let has_rc = !self.msg_timestamps("RC_CHANNELS").is_empty()
            || !self.msg_timestamps("RC_CHANNELS_RAW").is_empty();
        let n_motors = self.motor_count();

        let mut names = Vec::new();

        if has_attitude {
            for &a in &Axis::ALL {
                names.push(SensorField::Gyro(a).to_string());
            }
        }
        if has_imu {
            for &a in &Axis::ALL {
                names.push(SensorField::Accel(a).to_string());
            }
        }
        for i in 0..n_motors {
            names.push(SensorField::Motor(MotorIndex(i)).to_string());
        }
        if has_rc {
            for &ch in &[
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

    pub fn firmware_version(&self) -> &str {
        &self.firmware_version
    }

    pub fn craft_name(&self) -> &str {
        self.vehicle_type.as_str()
    }

    #[allow(clippy::cast_precision_loss, clippy::missing_panics_doc)]
    pub fn sample_rate_hz(&self) -> f64 {
        let ts = self.msg_timestamps("ATTITUDE");
        if ts.len() >= 2 {
            let t0 = *ts.first().unwrap();
            let tn = *ts.last().unwrap();
            let dt = tn.saturating_sub(t0);
            if dt > 0 {
                return (ts.len() - 1) as f64 / (dt as f64 / 1_000_000.0);
            }
        }
        0.0
    }

    #[allow(clippy::cast_precision_loss)]
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
        (max_t - min_t) as f64 / 1_000_000.0
    }

    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
    pub fn field(&self, field: &SensorField) -> Vec<f64> {
        match field {
            SensorField::Time => {
                // ATTITUDE time_boot_ms (stored as microseconds)
                let ts = self.msg_timestamps("ATTITUDE");
                ts.iter().map(|&t| t as f64).collect()
            }
            SensorField::Gyro(axis) => {
                // ATTITUDE rollspeed/pitchspeed/yawspeed (rad/s → deg/s)
                let field_name = match axis {
                    Axis::Roll => "rollspeed",
                    Axis::Pitch => "pitchspeed",
                    Axis::Yaw => "yawspeed",
                };
                self.msg_column("ATTITUDE", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter().map(|&v| v * 57.295_779_513_082_32).collect()
                    })
            }
            SensorField::GyroUnfilt(axis) => {
                // RAW_IMU xgyro/ygyro/zgyro (mrad/s → deg/s)
                let field_name = match axis {
                    Axis::Roll => "xgyro",
                    Axis::Pitch => "ygyro",
                    Axis::Yaw => "zgyro",
                };
                if let Some(col) = self.msg_column("RAW_IMU", field_name) {
                    return col
                        .iter()
                        .map(|&v| v * 0.001 * 57.295_779_513_082_32)
                        .collect();
                }
                self.msg_column("SCALED_IMU", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter()
                            .map(|&v| v * 0.001 * 57.295_779_513_082_32)
                            .collect()
                    })
            }
            SensorField::Accel(axis) => {
                // RAW_IMU xacc/yacc/zacc (mG → m/s²)
                let field_name = match axis {
                    Axis::Roll => "xacc",
                    Axis::Pitch => "yacc",
                    Axis::Yaw => "zacc",
                };
                if let Some(col) = self.msg_column("RAW_IMU", field_name) {
                    return col.iter().map(|&v| v * 0.001 * 9.806_65).collect();
                }
                self.msg_column("SCALED_IMU", field_name)
                    .map_or_else(Vec::new, |col| {
                        col.iter().map(|&v| v * 0.001 * 9.806_65).collect()
                    })
            }
            SensorField::Motor(idx) => {
                let field_name = format!("servo{}_raw", idx.0 + 1);
                self.msg_column("SERVO_OUTPUT_RAW", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Rc(ch) => {
                let chan_idx = ch.index() + 1;
                let field_name = format!("chan{chan_idx}_raw");
                if let Some(col) = self.msg_column("RC_CHANNELS", &field_name) {
                    return col.to_vec();
                }
                self.msg_column("RC_CHANNELS_RAW", &field_name)
                    .map_or_else(Vec::new, <[f64]>::to_vec)
            }
            SensorField::Altitude => self
                .msg_column("VFR_HUD", "alt")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::GpsSpeed => {
                // VFR_HUD groundspeed (m/s) preferred, else GPS_RAW_INT vel (cm/s → m/s)
                if let Some(col) = self.msg_column("VFR_HUD", "groundspeed") {
                    return col.to_vec();
                }
                self.msg_column("GPS_RAW_INT", "vel")
                    .map_or_else(Vec::new, |col| col.iter().map(|&v| v * 0.01).collect())
            }
            SensorField::GpsLat => {
                // GPS_RAW_INT lat (degE7 → degrees)
                self.msg_column("GPS_RAW_INT", "lat")
                    .map_or_else(Vec::new, |col| col.iter().map(|&v| v * 1e-7).collect())
            }
            SensorField::GpsLng => self
                .msg_column("GPS_RAW_INT", "lon")
                .map_or_else(Vec::new, |col| col.iter().map(|&v| v * 1e-7).collect()),
            SensorField::Heading => self
                .msg_column("VFR_HUD", "heading")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::Vbat => {
                // SYS_STATUS voltage_battery (mV → V)
                self.msg_column("SYS_STATUS", "voltage_battery")
                    .map_or_else(Vec::new, |col| col.iter().map(|&v| v * 0.001).collect())
            }
            SensorField::ERpm(_)
            | SensorField::Setpoint(_)
            | SensorField::PidP(_)
            | SensorField::PidI(_)
            | SensorField::PidD(_)
            | SensorField::Feedforward(_) => Vec::new(),
            SensorField::Rssi => self
                .msg_column("RC_CHANNELS", "rssi")
                .map_or_else(Vec::new, <[f64]>::to_vec),
            SensorField::Unknown(name) => {
                // Try "MSG_NAME.field_name" format
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
        if let Some(count) = self.vehicle_type.motor_count() {
            return count;
        }
        // Fallback: count non-zero servo channels
        self.topics.get("SERVO_OUTPUT_RAW").map_or(0, |mc| {
            (0..8)
                .filter(|i| {
                    mc.column(&format!("servo{}_raw", i + 1))
                        .is_some_and(|col| col.iter().any(|&v| v > 0.0))
                })
                .count()
        })
    }

    pub fn motor_range(&self) -> (f64, f64) {
        // ArduPilot params
        let min = self
            .params
            .get("MOT_PWM_MIN")
            .or_else(|| self.params.get("PWM_MIN"))
            .copied()
            .unwrap_or(1000.0);
        let max = self
            .params
            .get("MOT_PWM_MAX")
            .or_else(|| self.params.get("PWM_MAX"))
            .copied()
            .unwrap_or(2000.0);
        (min, max)
    }

    pub fn is_truncated(&self) -> bool {
        self.stats.truncated
    }

    pub fn corrupt_bytes(&self) -> usize {
        self.stats.corrupt_bytes
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn pid_gains(&self) -> crate::types::PidGains {
        let parse = |p_key: &str, i_key: &str, d_key: &str| -> crate::types::AxisGains {
            let get = |k: &str| -> Option<u32> {
                self.params
                    .get(k)
                    .copied()
                    .filter(|&v| v > 0.0)
                    .map(|v| (v * 1000.0) as u32)
            };
            crate::types::AxisGains {
                p: get(p_key),
                i: get(i_key),
                d: get(d_key),
            }
        };
        crate::types::PidGains::new(
            parse("ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D"),
            parse("ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D"),
            parse("ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D"),
        )
    }

    pub fn filter_config(&self) -> FilterConfig {
        let non_zero = |v: f64| if v > 0.0 { Some(v) } else { None };
        let p = |k: &str| self.params.get(k).copied().unwrap_or(0.0);

        // ArduPilot params (same as AP DataFlash)
        FilterConfig {
            gyro_lpf_hz: non_zero(p("INS_GYRO_FILTER")),
            gyro_lpf2_hz: None,
            dterm_lpf_hz: non_zero(p("ATC_RAT_RLL_FLTE")),
            dyn_notch_min_hz: if p("INS_HNTCH_ENABLE") > 0.0 {
                non_zero(p("INS_HNTCH_FREQ"))
            } else {
                None
            },
            dyn_notch_max_hz: None,
            gyro_notch1_hz: if p("INS_NOTCH_ENABLE") > 0.0 {
                non_zero(p("INS_NOTCH_FREQ"))
            } else {
                None
            },
            gyro_notch2_hz: if p("INS_NOTC2_ENABLE") > 0.0 {
                non_zero(p("INS_NOTC2_FREQ"))
            } else {
                None
            },
        }
    }

    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    pub fn index(&self) -> usize {
        self.session_index
    }
}
