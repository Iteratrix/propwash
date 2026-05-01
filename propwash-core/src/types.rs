use std::fmt;

use serde::Serialize;

// Re-export so existing `crate::types::Session` imports keep working during
// the refactor — Session itself lives in `crate::session`.
pub use crate::session::Session;

/// Rotational axis: roll, pitch, or yaw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub enum Axis {
    Roll,
    Pitch,
    Yaw,
}

impl Axis {
    /// All three axes in standard order.
    pub const ALL: [Self; 3] = [Self::Roll, Self::Pitch, Self::Yaw];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Roll => "roll",
            Self::Pitch => "pitch",
            Self::Yaw => "yaw",
        }
    }

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
pub(crate) enum RcChannel {
    Roll,
    Pitch,
    Yaw,
    Throttle,
}

impl fmt::Display for RcChannel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Roll => write!(f, "roll"),
            Self::Pitch => write!(f, "pitch"),
            Self::Yaw => write!(f, "yaw"),
            Self::Throttle => write!(f, "throttle"),
        }
    }
}

/// Typed motor index. Prevents mixing with axis indices or other ordinals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct MotorIndex(pub usize);

impl fmt::Display for MotorIndex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Format-agnostic sensor field identifier.
///
/// Crate-internal: external consumers use [`Session::field_by_name`]
/// to look up fields by string name. Kept inside the crate because
/// the BF parser uses it as a typed key for field-position lookup
/// tables.
///
/// Known fields get proper variants with typed indices.
/// Unknown fields preserve the original header string.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub(crate) enum SensorField {
    Time,
    Gyro(Axis),
    Motor(MotorIndex),
    Rc(RcChannel),
    Setpoint(Axis),
    Accel(Axis),
    PidP(Axis),
    PidI(Axis),
    PidD(Axis),
    Feedforward(Axis),
    ERpm(MotorIndex),
    GyroUnfilt(Axis),
    Vbat,
    Altitude,
    GpsSpeed,
    GpsLat,
    GpsLng,
    Heading,
    Rssi,
    Unknown(String),
}

impl SensorField {
    /// Parses a canonical field name into a `SensorField`.
    ///
    /// # Errors
    ///
    /// Returns `Err` if an indexed field has an invalid axis or index.
    pub fn parse(name: &str) -> Result<Self, String> {
        match name {
            "time" => Ok(Self::Time),
            "vbat" => Ok(Self::Vbat),
            "altitude" => Ok(Self::Altitude),
            "gps_speed" => Ok(Self::GpsSpeed),
            "gps_lat" => Ok(Self::GpsLat),
            "gps_lng" => Ok(Self::GpsLng),
            "heading" => Ok(Self::Heading),
            "rssi" => Ok(Self::Rssi),

            _ if name.starts_with("gyro[") => parse_axis_by_name(name)
                .map(Self::Gyro)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("gyro_unfilt[") => parse_axis_by_name(name)
                .map(Self::GyroUnfilt)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("motor[") => parse_index(name)
                .map(|i| Self::Motor(MotorIndex(i)))
                .ok_or_else(|| format!("invalid index in {name}")),
            _ if name.starts_with("rc[") => parse_rc_channel_by_name(name)
                .map(Self::Rc)
                .ok_or_else(|| format!("invalid channel in {name}")),
            _ if name.starts_with("setpoint[") => parse_axis_by_name(name)
                .map(Self::Setpoint)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("accel[") => parse_axis_by_name(name)
                .map(Self::Accel)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("pid_p[") => parse_axis_by_name(name)
                .map(Self::PidP)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("pid_i[") => parse_axis_by_name(name)
                .map(Self::PidI)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("pid_d[") => parse_axis_by_name(name)
                .map(Self::PidD)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("feedforward[") => parse_axis_by_name(name)
                .map(Self::Feedforward)
                .ok_or_else(|| format!("invalid axis in {name}")),
            _ if name.starts_with("erpm[") => parse_index(name)
                .map(|i| Self::ERpm(MotorIndex(i)))
                .ok_or_else(|| format!("invalid index in {name}")),
            other => Ok(Self::Unknown(other.to_string())),
        }
    }
}

impl fmt::Display for SensorField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Time => write!(f, "time"),
            Self::Gyro(a) => write!(f, "gyro[{a}]"),
            Self::Motor(m) => write!(f, "motor[{m}]"),
            Self::Rc(ch) => write!(f, "rc[{ch}]"),
            Self::Setpoint(a) => write!(f, "setpoint[{a}]"),
            Self::Accel(a) => write!(f, "accel[{a}]"),
            Self::PidP(a) => write!(f, "pid_p[{a}]"),
            Self::PidI(a) => write!(f, "pid_i[{a}]"),
            Self::PidD(a) => write!(f, "pid_d[{a}]"),
            Self::Feedforward(a) => write!(f, "feedforward[{a}]"),
            Self::ERpm(m) => write!(f, "erpm[{m}]"),
            Self::GyroUnfilt(a) => write!(f, "gyro_unfilt[{a}]"),
            Self::Vbat => write!(f, "vbat"),
            Self::Altitude => write!(f, "altitude"),
            Self::GpsSpeed => write!(f, "gps_speed"),
            Self::GpsLat => write!(f, "gps_lat"),
            Self::GpsLng => write!(f, "gps_lng"),
            Self::Heading => write!(f, "heading"),
            Self::Rssi => write!(f, "rssi"),
            Self::Unknown(s) => write!(f, "{s}"),
        }
    }
}

/// Extracts the axis from an indexed field name like `gyroADC[1]`.
pub(crate) fn parse_axis_field(name: &str) -> Option<Axis> {
    let idx = parse_index(name)?;
    match idx {
        0 => Some(Axis::Roll),
        1 => Some(Axis::Pitch),
        2 => Some(Axis::Yaw),
        _ => None,
    }
}

/// Extracts the RC channel from `rcCommand[N]`.
pub(crate) fn parse_rc_channel(name: &str) -> Option<RcChannel> {
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
pub(crate) fn parse_index(name: &str) -> Option<usize> {
    let start = name.find('[')? + 1;
    let end = name.find(']')?;
    name[start..end].parse().ok()
}

/// Extracts the axis from a canonical field name like `gyro[roll]`.
fn parse_axis_by_name(name: &str) -> Option<Axis> {
    let start = name.find('[')? + 1;
    let end = name.find(']')?;
    match &name[start..end] {
        "roll" => Some(Axis::Roll),
        "pitch" => Some(Axis::Pitch),
        "yaw" => Some(Axis::Yaw),
        _ => None,
    }
}

/// Extracts the RC channel from a canonical field name like `rc[throttle]`.
fn parse_rc_channel_by_name(name: &str) -> Option<RcChannel> {
    let start = name.find('[')? + 1;
    let end = name.find(']')?;
    match &name[start..end] {
        "roll" => Some(RcChannel::Roll),
        "pitch" => Some(RcChannel::Pitch),
        "yaw" => Some(RcChannel::Yaw),
        "throttle" => Some(RcChannel::Throttle),
        _ => None,
    }
}

/// A non-fatal diagnostic collected during parsing.
/// The parser never panics on corrupt data — it collects these instead.
#[derive(Debug, Clone, Serialize)]
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

/// PID gains for a single axis.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct AxisGains {
    pub p: Option<u32>,
    pub i: Option<u32>,
    pub d: Option<u32>,
}

/// PID gains extracted from flight controller configuration.
///
/// Values are firmware-native units (e.g., Betaflight uses integer gains,
/// ArduPilot/PX4 use floats scaled to milli-units).
#[derive(Debug, Clone, Default, Serialize)]
pub struct PidGains {
    axes: [AxisGains; 3],
}

impl PidGains {
    pub fn new(roll: AxisGains, pitch: AxisGains, yaw: AxisGains) -> Self {
        Self {
            axes: [roll, pitch, yaw],
        }
    }

    pub fn get(&self, axis: Axis) -> &AxisGains {
        &self.axes[axis.index()]
    }

    /// Returns true if any gains are available.
    pub fn has_data(&self) -> bool {
        self.axes.iter().any(|a| a.p.is_some())
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
