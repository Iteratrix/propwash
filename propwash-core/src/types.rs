use std::fmt;

use crate::format::ap::types::ApRawSession;
use crate::format::bf::types::BfRawSession;
use crate::format::px4::types::Px4RawSession;

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

    /// Returns whether this field is a motor field.
    pub fn is_motor(&self) -> bool {
        matches!(self, Self::Motor(_))
    }

    /// Returns whether this field is an eRPM field.
    pub fn is_erpm(&self) -> bool {
        matches!(self, Self::ERpm(_))
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

/// The only error the public API can return — file I/O failures.
/// Parse corruption is never an error; it produces warnings.
#[derive(Debug)]
pub enum ParseError {
    Io(std::io::Error),
    NoData,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "I/O error: {e}"),
            Self::NoData => write!(f, "No data to parse"),
        }
    }
}

impl std::error::Error for ParseError {}

impl From<std::io::Error> for ParseError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// A value decoded from a log frame. Formats differ in what types they produce.
#[derive(Debug, Clone)]
pub enum Value {
    Int(i64),
    Float(f64),
    Str(String),
    Bool(bool),
}

impl Value {
    /// Converts to an integer, truncating floats.
    #[allow(clippy::cast_possible_truncation)]
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Self::Int(v) => Some(*v),
            Self::Float(v) => Some(*v as i64),
            Self::Str(_) | Self::Bool(_) => None,
        }
    }

    /// Converts to a float, promoting integers.
    #[allow(clippy::cast_precision_loss)]
    pub fn as_float(&self) -> Option<f64> {
        match self {
            Self::Float(v) => Some(*v),
            Self::Int(v) => Some(*v as f64),
            Self::Str(_) | Self::Bool(_) => None,
        }
    }
}

/// Format-agnostic interface for accessing sensor data.
///
/// Implemented by each format's raw session type. This is the primary
/// abstraction consumers should use — it provides uniform access to
/// sensor data regardless of the underlying log format.
pub trait Unified {
    fn frame_count(&self) -> usize;
    fn field_names(&self) -> Vec<String>;
    fn firmware_version(&self) -> &str;
    fn craft_name(&self) -> &str;
    fn sample_rate_hz(&self) -> f64;
    fn duration_seconds(&self) -> f64;
    fn field(&self, field: &SensorField) -> Vec<f64>;
    fn motor_count(&self) -> usize;

    /// Returns the (min, max) output range for motor values.
    fn motor_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Extracts one field by header string name.
    /// Convenience for system boundaries (WASM bridge, CLI user input).
    fn field_by_name(&self, name: &str) -> Vec<f64> {
        self.field(&SensorField::from_header(name))
    }
}

/// Format-specific raw session data.
/// Adding a variant is a breaking change — consumers should handle every format.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum RawSession {
    Betaflight(BfRawSession),
    ArduPilot(ApRawSession),
    Px4(Px4RawSession),
}

impl RawSession {
    /// Single dispatch point: returns the Unified implementation for this format.
    /// Adding a new format = adding one line here.
    fn as_unified(&self) -> &dyn Unified {
        match self {
            Self::Betaflight(bf) => bf,
            Self::ArduPilot(ap) => ap,
            Self::Px4(px4) => px4,
        }
    }
}

/// One parsed session from a log file.
#[derive(Debug)]
pub struct Session {
    /// Format-specific raw data — always available.
    pub raw: RawSession,
    /// Non-fatal diagnostics from parsing.
    pub warnings: Vec<Warning>,
    /// 1-based session index within the file.
    pub index: usize,
}

impl Session {
    /// Returns the unified interface for format-agnostic sensor data access.
    pub fn unified(&self) -> &dyn Unified {
        self.raw.as_unified()
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
