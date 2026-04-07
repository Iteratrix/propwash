use std::fmt;

use serde::Serialize;

use crate::format::ap::types::ApSession;
use crate::format::bf::types::BfSession;
use crate::format::px4::types::Px4Session;

/// Rotational axis: roll, pitch, or yaw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axis {
    Roll,
    Pitch,
    Yaw,
}

impl Axis {
    /// All three axes in standard order.
    pub const ALL: [Self; 3] = [Self::Roll, Self::Pitch, Self::Yaw];

    /// Index used in indexed field names (e.g., `gyroADC[0]` = Roll).
    pub fn index(self) -> usize {
        match self {
            Self::Roll => 0,
            Self::Pitch => 1,
            Self::Yaw => 2,
        }
    }
}

impl fmt::Display for Axis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Roll => write!(f, "roll"),
            Self::Pitch => write!(f, "pitch"),
            Self::Yaw => write!(f, "yaw"),
        }
    }
}

/// RC channel: roll, pitch, yaw, or throttle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RcChannel {
    Roll,
    Pitch,
    Yaw,
    Throttle,
}

impl RcChannel {
    pub fn index(self) -> usize {
        match self {
            Self::Roll => 0,
            Self::Pitch => 1,
            Self::Yaw => 2,
            Self::Throttle => 3,
        }
    }
}

/// Typed motor index. Prevents mixing with axis indices or other ordinals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MotorIndex(pub usize);

impl fmt::Display for MotorIndex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Format-agnostic sensor field identifier.
///
/// Known fields get proper variants with typed indices.
/// Unknown fields preserve the original header string.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum SensorField {
    Time,
    LoopIteration,
    Gyro(Axis),
    Motor(MotorIndex),
    Rc(RcChannel),
    Setpoint(Axis),
    Accel(Axis),
    PidP(Axis),
    PidI(Axis),
    PidD(Axis),
    ERpm(MotorIndex),
    GyroUnfilt(Axis),
    Vbat,
    Altitude,
    GpsSpeed,
    GpsLat,
    GpsLng,
    Heading,
    Unknown(String),
}

impl SensorField {
    /// Parses a header field name into a `SensorField`.
    /// Handles known aliases (e.g., `gyroData` = `gyroADC`).
    pub fn from_header(name: &str) -> Self {
        match name {
            "time" => Self::Time,
            "loopIteration" => Self::LoopIteration,
            "vbatLatest" | "vbat" => Self::Vbat,
            "altitude" | "BarAlt" | "Alt" => Self::Altitude,
            "gpsSpeed" | "Spd" => Self::GpsSpeed,
            "heading" | "Yaw" => Self::Heading,

            _ if name.starts_with("gyroADC[") || name.starts_with("gyroData[") => {
                parse_axis_field(name).map_or_else(|| Self::Unknown(name.to_string()), Self::Gyro)
            }
            _ if name.starts_with("gyroUnfilt[") => parse_axis_field(name)
                .map_or_else(|| Self::Unknown(name.to_string()), Self::GyroUnfilt),
            _ if name.starts_with("motor[") => parse_index(name).map_or_else(
                || Self::Unknown(name.to_string()),
                |i| Self::Motor(MotorIndex(i)),
            ),
            _ if name.starts_with("rcCommand[") => {
                parse_rc_channel(name).map_or_else(|| Self::Unknown(name.to_string()), Self::Rc)
            }
            _ if name.starts_with("setpoint[") => parse_axis_field(name)
                .map_or_else(|| Self::Unknown(name.to_string()), Self::Setpoint),
            _ if name.starts_with("accSmooth[") => {
                parse_axis_field(name).map_or_else(|| Self::Unknown(name.to_string()), Self::Accel)
            }
            _ if name.starts_with("axisP[") => {
                parse_axis_field(name).map_or_else(|| Self::Unknown(name.to_string()), Self::PidP)
            }
            _ if name.starts_with("axisI[") => {
                parse_axis_field(name).map_or_else(|| Self::Unknown(name.to_string()), Self::PidI)
            }
            _ if name.starts_with("axisD[") => {
                parse_axis_field(name).map_or_else(|| Self::Unknown(name.to_string()), Self::PidD)
            }
            _ if name.starts_with("eRPM[") => parse_index(name).map_or_else(
                || Self::Unknown(name.to_string()),
                |i| Self::ERpm(MotorIndex(i)),
            ),
            other => Self::Unknown(other.to_string()),
        }
    }
}

impl fmt::Display for SensorField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Time => write!(f, "time"),
            Self::LoopIteration => write!(f, "loopIteration"),
            Self::Gyro(a) => write!(f, "gyroADC[{}]", a.index()),
            Self::Motor(m) => write!(f, "motor[{m}]"),
            Self::Rc(ch) => write!(f, "rcCommand[{}]", ch.index()),
            Self::Setpoint(a) => write!(f, "setpoint[{}]", a.index()),
            Self::Accel(a) => write!(f, "accSmooth[{}]", a.index()),
            Self::PidP(a) => write!(f, "axisP[{}]", a.index()),
            Self::PidI(a) => write!(f, "axisI[{}]", a.index()),
            Self::PidD(a) => write!(f, "axisD[{}]", a.index()),
            Self::ERpm(m) => write!(f, "eRPM[{m}]"),
            Self::GyroUnfilt(a) => write!(f, "gyroUnfilt[{}]", a.index()),
            Self::Vbat => write!(f, "vbatLatest"),
            Self::Altitude => write!(f, "altitude"),
            Self::GpsSpeed => write!(f, "gpsSpeed"),
            Self::GpsLat => write!(f, "gpsLat"),
            Self::GpsLng => write!(f, "gpsLng"),
            Self::Heading => write!(f, "heading"),
            Self::Unknown(s) => write!(f, "{s}"),
        }
    }
}

/// Extracts the axis from an indexed field name like `gyroADC[1]`.
fn parse_axis_field(name: &str) -> Option<Axis> {
    let idx = parse_index(name)?;
    match idx {
        0 => Some(Axis::Roll),
        1 => Some(Axis::Pitch),
        2 => Some(Axis::Yaw),
        _ => None,
    }
}

/// Extracts the RC channel from `rcCommand[N]`.
fn parse_rc_channel(name: &str) -> Option<RcChannel> {
    let idx = parse_index(name)?;
    match idx {
        0 => Some(RcChannel::Roll),
        1 => Some(RcChannel::Pitch),
        2 => Some(RcChannel::Yaw),
        3 => Some(RcChannel::Throttle),
        _ => None,
    }
}

/// Extracts the numeric index from a field name like `motor[3]`.
fn parse_index(name: &str) -> Option<usize> {
    let start = name.find('[')? + 1;
    let end = name.find(']')?;
    name[start..end].parse().ok()
}

/// A non-fatal diagnostic collected during parsing.
/// The parser never panics on corrupt data — it collects these instead.
#[derive(Debug, Clone)]
pub struct Warning {
    pub message: String,
    pub byte_offset: Option<usize>,
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.byte_offset {
            Some(offset) => write!(f, "[0x{offset:x}] {}", self.message),
            None => write!(f, "{}", self.message),
        }
    }
}

/// Fatal errors from the public API — I/O failures or completely
/// unrecognizable input. Parse corruption within a recognized format
/// is never an error; it produces warnings instead.
#[derive(Debug)]
pub enum ParseError {
    Io(std::io::Error),
    NoData,
    UnrecognizedFormat,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "I/O error: {e}"),
            Self::NoData => write!(f, "No data to parse"),
            Self::UnrecognizedFormat => write!(f, "No recognized blackbox format found in data"),
        }
    }
}

impl std::error::Error for ParseError {}

impl From<std::io::Error> for ParseError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Filter configuration extracted from flight controller parameters.
#[derive(Debug, Clone, Serialize)]
#[allow(clippy::struct_field_names)]
pub struct FilterConfig {
    pub gyro_lpf_hz: Option<f64>,
    pub gyro_lpf2_hz: Option<f64>,
    pub dterm_lpf_hz: Option<f64>,
    pub dyn_notch_min_hz: Option<f64>,
    pub dyn_notch_max_hz: Option<f64>,
    pub gyro_notch1_hz: Option<f64>,
    pub gyro_notch2_hz: Option<f64>,
}

/// A parsed flight session. The primary type consumers work with.
///
/// Call methods directly for format-agnostic sensor data access.
/// Pattern match on the enum variants when you need format-specific data.
///
/// Adding a variant is a breaking change — consumers should handle every format.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum Session {
    Betaflight(BfSession),
    ArduPilot(ApSession),
    Px4(Px4Session),
}

/// Dispatches a method call to the inner format-specific type.
macro_rules! dispatch {
    ($self:ident, $method:ident $(, $arg:expr)*) => {
        match $self {
            Self::Betaflight(s) => s.$method($($arg),*),
            Self::ArduPilot(s) => s.$method($($arg),*),
            Self::Px4(s) => s.$method($($arg),*),
        }
    };
}

impl Session {
    pub fn frame_count(&self) -> usize {
        dispatch!(self, frame_count)
    }
    pub fn field_names(&self) -> Vec<String> {
        dispatch!(self, field_names)
    }
    pub fn firmware_version(&self) -> &str {
        dispatch!(self, firmware_version)
    }
    pub fn craft_name(&self) -> &str {
        dispatch!(self, craft_name)
    }
    pub fn sample_rate_hz(&self) -> f64 {
        dispatch!(self, sample_rate_hz)
    }
    pub fn duration_seconds(&self) -> f64 {
        dispatch!(self, duration_seconds)
    }
    pub fn field(&self, field: &SensorField) -> Vec<f64> {
        dispatch!(self, field, field)
    }
    pub fn motor_count(&self) -> usize {
        dispatch!(self, motor_count)
    }
    pub fn motor_range(&self) -> (f64, f64) {
        dispatch!(self, motor_range)
    }
    pub fn warnings(&self) -> &[Warning] {
        dispatch!(self, warnings)
    }
    pub fn index(&self) -> usize {
        dispatch!(self, index)
    }

    pub fn filter_config(&self) -> FilterConfig {
        dispatch!(self, filter_config)
    }
    pub fn is_truncated(&self) -> bool {
        dispatch!(self, is_truncated)
    }
    pub fn has_rpm_telemetry(&self) -> bool {
        dispatch!(self, has_rpm_telemetry)
    }
    pub fn has_gyro_unfiltered(&self) -> bool {
        dispatch!(self, has_gyro_unfiltered)
    }
    pub fn corrupt_bytes(&self) -> usize {
        dispatch!(self, corrupt_bytes)
    }

    /// Extracts one field by header string name.
    /// Convenience for system boundaries (WASM bridge, CLI user input).
    pub fn field_by_name(&self, name: &str) -> Vec<f64> {
        self.field(&SensorField::from_header(name))
    }
}

/// Complete parsed log file.
#[derive(Debug)]
pub struct Log {
    pub sessions: Vec<Session>,
    pub warnings: Vec<Warning>,
}

impl Log {
    /// Returns the number of sessions in the log.
    pub fn session_count(&self) -> usize {
        self.sessions.len()
    }
}
