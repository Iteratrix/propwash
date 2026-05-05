//! Typed `Session` — the unified, format-agnostic representation of one
//! parsed flight log session.
//!
//! Each parser populates a `Session` directly. Storage is **columnar**:
//! every stream is a parallel pair of `time_us` (microseconds since session
//! start) and `values` (samples). Per-stream time axes preserve the
//! multi-rate nature of real flight data — no resampling or fabrication
//! happens at parse time. Resampling lives in [`crate::analysis`].
//!
//! Empty `Vec` fields mean "not present in this log" — consumers should
//! treat them as missing data, not zeroes.

use std::collections::HashMap;

use az::Az;
use serde::Serialize;

use crate::types::{Axis, FilterConfig, MotorIndex, PidGains, RcChannel, SensorField, Warning};
use crate::units::{
    Amps, Celsius, DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2,
    Normalized01, Volts,
};

// ── Generic building blocks ──────────────────────────────────────────────────

/// Three values bundled per rotational axis. Used both for per-sample
/// triples (`Triaxial<f64>`) and for parallel columns (`Triaxial<Vec<f64>>`).
#[derive(Debug, Clone, Default, Serialize)]
pub struct Triaxial<T> {
    pub roll: T,
    pub pitch: T,
    pub yaw: T,
}

impl<T> Triaxial<T> {
    pub const fn new(roll: T, pitch: T, yaw: T) -> Self {
        Self { roll, pitch, yaw }
    }

    pub fn get(&self, axis: Axis) -> &T {
        match axis {
            Axis::Roll => &self.roll,
            Axis::Pitch => &self.pitch,
            Axis::Yaw => &self.yaw,
        }
    }

    pub fn get_mut(&mut self, axis: Axis) -> &mut T {
        match axis {
            Axis::Roll => &mut self.roll,
            Axis::Pitch => &mut self.pitch,
            Axis::Yaw => &mut self.yaw,
        }
    }

    /// Iterates `(axis, value)` in canonical roll/pitch/yaw order.
    pub fn iter(&self) -> impl Iterator<Item = (Axis, &T)> {
        [
            (Axis::Roll, &self.roll),
            (Axis::Pitch, &self.pitch),
            (Axis::Yaw, &self.yaw),
        ]
        .into_iter()
    }

    /// Maps each axis through a function, preserving the bundle.
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Triaxial<U> {
        Triaxial {
            roll: f(self.roll),
            pitch: f(self.pitch),
            yaw: f(self.yaw),
        }
    }

    /// Maps each axis through a function that also sees the axis label.
    pub fn map_with_axis<U>(self, mut f: impl FnMut(Axis, T) -> U) -> Triaxial<U> {
        Triaxial {
            roll: f(Axis::Roll, self.roll),
            pitch: f(Axis::Pitch, self.pitch),
            yaw: f(Axis::Yaw, self.yaw),
        }
    }
}

impl<T: Default> Triaxial<Vec<T>> {
    /// Constructs a `Triaxial` of empty `Vec`s.
    pub fn empty_vecs() -> Self {
        Self {
            roll: Vec::new(),
            pitch: Vec::new(),
            yaw: Vec::new(),
        }
    }
}

/// A single-valued, time-stamped stream.
///
/// `time_us[i]` is the timestamp (μs since session start) of `values[i]`.
/// The two `Vec`s are always the same length. An empty `TimeSeries`
/// means the stream was not present in the source log.
#[derive(Debug, Clone, Default, Serialize)]
pub struct TimeSeries<T> {
    pub time_us: Vec<u64>,
    pub values: Vec<T>,
}

impl<T> TimeSeries<T> {
    pub const fn new() -> Self {
        Self {
            time_us: Vec::new(),
            values: Vec::new(),
        }
    }

    pub fn from_parts(time_us: Vec<u64>, values: Vec<T>) -> Self {
        debug_assert_eq!(
            time_us.len(),
            values.len(),
            "TimeSeries time_us and values length mismatch"
        );
        Self { time_us, values }
    }

    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    pub fn push(&mut self, t: u64, v: T) {
        self.time_us.push(t);
        self.values.push(v);
    }

    /// Total span in microseconds (last timestamp − first), or 0 if &lt; 2 samples.
    pub fn duration_us(&self) -> u64 {
        match (self.time_us.first(), self.time_us.last()) {
            (Some(&first), Some(&last)) if self.time_us.len() >= 2 && last >= first => last - first,
            _ => 0,
        }
    }
}

/// Three axis-co-sampled streams sharing one time axis.
///
/// Use this when the three axes come from a single sensor sampled together
/// (gyro, accelerometer, rate setpoints). `values.roll[i]`, `values.pitch[i]`,
/// `values.yaw[i]` all share the timestamp `time_us[i]`.
#[derive(Debug, Clone, Default, Serialize)]
pub struct TriaxialSeries<T> {
    pub time_us: Vec<u64>,
    pub values: Triaxial<Vec<T>>,
}

impl<T> TriaxialSeries<T> {
    pub fn new() -> Self {
        Self {
            time_us: Vec::new(),
            values: Triaxial {
                roll: Vec::new(),
                pitch: Vec::new(),
                yaw: Vec::new(),
            },
        }
    }

    pub fn len(&self) -> usize {
        self.time_us.len()
    }

    pub fn is_empty(&self) -> bool {
        self.time_us.is_empty()
    }

    /// Borrow the column for one axis as a slice.
    pub fn axis(&self, axis: Axis) -> &[T] {
        self.values.get(axis)
    }
}

// ── Composite groups ────────────────────────────────────────────────────────

/// Pilot stick inputs and throttle.
///
/// Sticks (roll/pitch/yaw) and throttle share one time axis — they're
/// sampled together by the receiver. Stick deflection is normalised to
/// −1.0..=1.0; throttle is normalised to 0.0..=1.0.
#[derive(Debug, Clone, Default, Serialize)]
pub struct RcCommand {
    pub time_us: Vec<u64>,
    pub sticks: Triaxial<Vec<f32>>,
    pub throttle: Vec<Normalized01>,
}

impl RcCommand {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.time_us.is_empty()
    }

    pub fn len(&self) -> usize {
        self.time_us.len()
    }
}

/// Mixer outputs and (optional) ESC telemetry.
///
/// `commands[motor_index]` is the per-sample command for one motor;
/// all motors share `time_us`. Values are normalised to 0.0..=1.0 (idle
/// to full). `esc` is `Some` only when BLHeli/DShot ESC telemetry is
/// available in the log.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Motors {
    pub time_us: Vec<u64>,
    pub commands: Vec<Vec<Normalized01>>,
    pub esc: Option<Esc>,
}

impl Motors {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn motor_count(&self) -> usize {
        self.commands.len()
    }

    pub fn is_empty(&self) -> bool {
        self.time_us.is_empty()
    }
}

/// Per-motor ESC telemetry (BLHeli32/DShot bidirectional).
///
/// Each `Vec<Vec<_>>` is indexed by motor first, then sample. All motors
/// share the same `time_us` axis. Empty inner `Vec`s indicate that
/// particular telemetry channel wasn't reported.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Esc {
    pub time_us: Vec<u64>,
    pub erpm: Vec<Vec<Erpm>>,
    pub temperature: Vec<Vec<Celsius>>,
    pub voltage: Vec<Vec<Volts>>,
    pub current: Vec<Vec<Amps>>,
}

/// GPS fix samples. Coordinates are decimal degrees, not raw integers.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Gps {
    pub time_us: Vec<u64>,
    pub lat: Vec<DecimalDegrees>,
    pub lng: Vec<DecimalDegrees>,
    pub alt: Vec<Meters>,
    pub speed: Vec<MetersPerSec>,
    /// **GPS course over ground**, degrees. This is the direction of
    /// horizontal motion — it differs from airframe heading whenever
    /// the vehicle yaws without translating (hover-and-rotate, crab in
    /// wind). For airframe heading, see [`Session::attitude`].
    pub heading: Vec<f32>,
    pub sats: Vec<u8>,
}

impl Gps {
    /// True when no GPS frames were observed AND no data column carries
    /// values. A schema with only timestamps (no lat/lng/alt/etc.)
    /// should still be considered absent — consumers checking
    /// `gps.is_empty()` shouldn't see "Some" merely because we logged
    /// time markers without payload.
    pub fn is_empty(&self) -> bool {
        self.time_us.is_empty()
            && self.lat.is_empty()
            && self.lng.is_empty()
            && self.alt.is_empty()
            && self.speed.is_empty()
            && self.heading.is_empty()
            && self.sats.is_empty()
    }

    /// True when at least one data column carries values. Strictly
    /// stronger than `!is_empty()` — a Gps with only `time_us` populated
    /// (e.g. a BF schema lacking all coord fields) returns false here
    /// but also returns true from `is_empty()` after the `bug_007` fix.
    pub fn has_data(&self) -> bool {
        !self.lat.is_empty()
            || !self.lng.is_empty()
            || !self.alt.is_empty()
            || !self.speed.is_empty()
            || !self.heading.is_empty()
            || !self.sats.is_empty()
    }
}

// ── Discrete events ─────────────────────────────────────────────────────────

/// Severity level for [`EventKind::LogMessage`] events.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum LogSeverity {
    Debug,
    Info,
    Notice,
    Warning,
    Error,
    Critical,
    Alert,
    Emergency,
}

/// Common flight modes across formats. Format-specific modes that don't
/// map cleanly land in [`FlightMode::Other`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
pub enum FlightMode {
    /// No mode active / disarmed.
    #[default]
    None,
    /// Stabilized / Angle / Self-level mode.
    Stabilize,
    /// Acro / Rate mode.
    Acro,
    /// Horizon (blended angle and acro).
    Horizon,
    /// Altitude hold.
    AltHold,
    /// Position hold (GPS-required).
    PosHold,
    /// Loiter.
    Loiter,
    /// Return to home / RTL / RTH.
    ReturnToHome,
    /// Auto / Mission.
    Auto,
    /// Manual.
    Manual,
    /// Land.
    Land,
    /// Failsafe.
    Failsafe,
    /// Format-specific mode not in this enum (preserves original name).
    Other(String),
}

/// Discriminated event types emitted at irregular times during flight.
#[derive(Debug, Clone, Serialize)]
pub enum EventKind {
    Armed,
    Disarmed,
    ModeChange { to: FlightMode },
    Crash,
    Failsafe { reason: String },
    GpsRescue { phase: String },
    LogMessage { severity: LogSeverity },
    Custom(String),
}

/// One event — point in time + discriminated kind + optional human-readable
/// text. Events carry their own `time_us` inline because they're irregular
/// by nature (no shared axis).
#[derive(Debug, Clone, Serialize)]
pub struct Event {
    pub time_us: u64,
    pub kind: EventKind,
    pub message: Option<String>,
}

// ── Per-session metadata ────────────────────────────────────────────────────

/// Source format of the parsed log.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum Format {
    Betaflight,
    ArduPilot,
    Px4,
    Mavlink,
}

impl Format {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Betaflight => "Betaflight",
            Self::ArduPilot => "ArduPilot",
            Self::Px4 => "PX4",
            Self::Mavlink => "MAVLink",
        }
    }
}

/// Static (non-time-series) information about the session.
#[derive(Debug, Clone, Serialize)]
pub struct SessionMeta {
    pub format: Format,
    pub firmware: String,
    pub craft_name: Option<String>,
    pub board: Option<String>,
    pub motor_count: usize,
    /// Motor pole count, used by FFT analysis to convert eRPM to
    /// mechanical Hz (`mechanical_hz = erpm / 60 / (poles / 2)`).
    /// Sourced from BF `motor_poles` header; AP/PX4/MAVLink leave it
    /// `None`. Default 14 (3-pole-pair brushless, the common 5" build)
    /// is a reasonable downstream fallback.
    pub motor_poles: Option<u32>,
    pub pid_gains: Option<PidGains>,
    pub filter_config: Option<FilterConfig>,
    /// 1-based index within the parent log (logs may contain multiple sessions).
    pub session_index: usize,
    /// True if the log ended without a clean stop marker (likely truncated/crashed).
    pub truncated: bool,
    /// Bytes the parser couldn't make sense of (corruption or unknown extensions).
    pub corrupt_bytes: usize,
    /// Non-fatal diagnostics gathered during parsing.
    pub warnings: Vec<Warning>,
}

impl Default for SessionMeta {
    fn default() -> Self {
        Self {
            format: Format::Betaflight,
            firmware: String::new(),
            craft_name: None,
            board: None,
            motor_count: 0,
            motor_poles: None,
            pid_gains: None,
            filter_config: None,
            session_index: 0,
            truncated: false,
            corrupt_bytes: 0,
            warnings: Vec::new(),
        }
    }
}

// ── Top-level Session ───────────────────────────────────────────────────────

/// The unified, typed flight session.
///
/// Each field reflects what was actually in the source log. Empty
/// vectors mean the stream wasn't present. Per-stream `time_us` axes
/// preserve multi-rate sampling (e.g., gyro at 4 kHz vs vbat at 50 Hz)
/// without fabricating samples.
///
/// For co-aligned analysis (FFT, step response, PID), use
/// [`crate::analysis::analyze`] which builds an `AlignedMain` view at
/// a chosen rate.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Session {
    // Sensors
    pub gyro: TriaxialSeries<DegPerSec>,
    pub accel: TriaxialSeries<MetersPerSec2>,
    pub setpoint: TriaxialSeries<DegPerSec>,
    pub pid_err: TriaxialSeries<DegPerSec>,
    /// Vehicle attitude (roll/pitch/yaw) in **degrees**. This is the
    /// firmware-reported airframe orientation — distinct from
    /// [`Gps::heading`] which is GPS course-over-ground (direction of
    /// motion, can differ from heading during hover/crab).
    /// Empty when the source format has no attitude topic.
    pub attitude: TriaxialSeries<f32>,

    // Inputs / outputs
    pub rc_command: RcCommand,
    pub motors: Motors,

    // Single-stream telemetry
    pub armed: TimeSeries<bool>,
    pub flight_mode: TimeSeries<FlightMode>,
    pub vbat: TimeSeries<Volts>,
    pub current: TimeSeries<Amps>,
    pub rssi: TimeSeries<f32>,

    // Optional / irregular
    pub gps: Option<Gps>,
    pub events: Vec<Event>,

    // Catch-all for format-specific numeric streams that don't fit a typed slot.
    pub extras: HashMap<String, TimeSeries<f64>>,

    pub meta: SessionMeta,
}

impl Session {
    /// Convenience: 1-based session index from `meta`.
    pub fn index(&self) -> usize {
        self.meta.session_index
    }

    /// Convenience: PID gains from `meta`. Returns the default (empty) gains
    /// if the format didn't surface any.
    pub fn pid_gains(&self) -> PidGains {
        self.meta.pid_gains.clone().unwrap_or_default()
    }

    /// Convenience: filter config from `meta`. Returns a config with all
    /// `None`s if the format didn't surface filter parameters.
    pub fn filter_config(&self) -> FilterConfig {
        self.meta
            .filter_config
            .clone()
            .unwrap_or_else(empty_filter_config)
    }

    /// Motor command range. Always `(0.0, 1.0)` because motor commands
    /// are normalised to that range on the new Session.
    pub fn motor_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Enumerate canonical names of populated streams (typed slots +
    /// `extras` keys) for runtime field-picker UIs and `propwash info`
    /// output. A name in this list, when round-tripped through
    /// [`SensorField::from_str`] and [`Self::field`], is guaranteed to
    /// resolve to a non-empty `Vec` (with the exception of names that
    /// have no `SensorField` variant — `attitude[*]`, `current`,
    /// `extras` keys — which fall back to [`SensorField::Unknown`]).
    ///
    /// Code that knows what it wants at compile time should reach into
    /// the typed fields directly (`session.gyro.values.roll`, etc.)
    /// instead of round-tripping through strings.
    pub fn field_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        // Time is the implicit anchor for any non-empty triaxial stream.
        if !self.gyro.is_empty()
            || !self.accel.is_empty()
            || !self.setpoint.is_empty()
            || !self.motors.is_empty()
            || !self.rc_command.is_empty()
        {
            names.push("time".into());
        }
        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            if !self.gyro.values.get(axis).is_empty() {
                names.push(format!("gyro[{axis}]"));
            }
        }
        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            if !self.accel.values.get(axis).is_empty() {
                names.push(format!("accel[{axis}]"));
            }
        }
        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            if !self.setpoint.values.get(axis).is_empty() {
                names.push(format!("setpoint[{axis}]"));
            }
        }
        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            if !self.attitude.values.get(axis).is_empty() {
                names.push(format!("attitude[{axis}]"));
            }
        }
        for (i, col) in self.motors.commands.iter().enumerate() {
            if !col.is_empty() {
                names.push(format!("motor[{i}]"));
            }
        }
        if let Some(esc) = &self.motors.esc {
            for (i, col) in esc.erpm.iter().enumerate() {
                if !col.is_empty() {
                    names.push(format!("erpm[{i}]"));
                }
            }
        }
        if !self.rc_command.sticks.roll.is_empty() {
            names.push("rc[roll]".into());
        }
        if !self.rc_command.sticks.pitch.is_empty() {
            names.push("rc[pitch]".into());
        }
        if !self.rc_command.sticks.yaw.is_empty() {
            names.push("rc[yaw]".into());
        }
        if !self.rc_command.throttle.is_empty() {
            names.push("rc[throttle]".into());
        }
        if !self.vbat.is_empty() {
            names.push("vbat".into());
        }
        if !self.current.is_empty() {
            names.push("current".into());
        }
        if !self.rssi.is_empty() {
            names.push("rssi".into());
        }
        if let Some(gps) = &self.gps {
            if !gps.lat.is_empty() {
                names.push("gps_lat".into());
            }
            if !gps.lng.is_empty() {
                names.push("gps_lng".into());
            }
            if !gps.alt.is_empty() {
                names.push("altitude".into());
            }
            if !gps.speed.is_empty() {
                names.push("gps_speed".into());
            }
            if !gps.heading.is_empty() {
                names.push("heading".into());
            }
        }
        names.extend(self.extras.keys().cloned());
        names
    }

    /// Look up a field by typed [`SensorField`] handle. Returns an
    /// empty `Vec` for variants that aren't populated in this session
    /// (e.g. asking for `Gyro(Roll)` on a session whose gyro stream is
    /// empty, or any [`SensorField::Unknown`] value).
    ///
    /// **Prefer the typed accessors directly** when the field is
    /// known at compile time — they're zero-cost and preserve unit
    /// information:
    ///
    /// ```ignore
    /// let roll: &[DegPerSec]    = &session.gyro.values.roll;
    /// let m0:   &[Normalized01] = &session.motors.commands[0];
    /// let vbat: &[Volts]        = &session.vbat.values;
    /// ```
    ///
    /// Use this method when the field handle came from a runtime
    /// source — a CLI argument parsed via [`SensorField::from_str`],
    /// a `SensorField` deserialized off the WASM boundary, etc. It
    /// allocates a `Vec<f64>` per call (copy + cast off the typed
    /// columns) and erases the unit type.
    #[allow(clippy::too_many_lines)] // declarative variant-to-typed-field routing
    pub fn field(&self, field: &SensorField) -> Vec<f64> {
        match field {
            SensorField::Time => self.gyro.time_us.iter().map(|&t| t.az::<f64>()).collect(),
            SensorField::Gyro(axis) => {
                bytemuck::cast_slice::<DegPerSec, f64>(self.gyro.values.get(*axis).as_slice())
                    .to_vec()
            }
            SensorField::GyroUnfilt(_) => Vec::new(), // not modelled yet
            SensorField::Setpoint(axis) => {
                bytemuck::cast_slice::<DegPerSec, f64>(self.setpoint.values.get(*axis).as_slice())
                    .to_vec()
            }
            SensorField::Accel(axis) => {
                bytemuck::cast_slice::<MetersPerSec2, f64>(self.accel.values.get(*axis).as_slice())
                    .to_vec()
            }
            SensorField::Motor(MotorIndex(i)) => self
                .motors
                .commands
                .get(*i)
                .map(|col| {
                    bytemuck::cast_slice::<Normalized01, f32>(col)
                        .iter()
                        .map(|&v| f64::from(v))
                        .collect()
                })
                .unwrap_or_default(),
            SensorField::ERpm(MotorIndex(i)) => self
                .motors
                .esc
                .as_ref()
                .and_then(|e| e.erpm.get(*i))
                .map(|col| {
                    bytemuck::cast_slice::<Erpm, u32>(col)
                        .iter()
                        .map(|&v| f64::from(v))
                        .collect()
                })
                .unwrap_or_default(),
            SensorField::Rc(RcChannel::Roll) => self
                .rc_command
                .sticks
                .roll
                .iter()
                .map(|&v| f64::from(v))
                .collect(),
            SensorField::Rc(RcChannel::Pitch) => self
                .rc_command
                .sticks
                .pitch
                .iter()
                .map(|&v| f64::from(v))
                .collect(),
            SensorField::Rc(RcChannel::Yaw) => self
                .rc_command
                .sticks
                .yaw
                .iter()
                .map(|&v| f64::from(v))
                .collect(),
            SensorField::Rc(RcChannel::Throttle) => {
                bytemuck::cast_slice::<Normalized01, f32>(&self.rc_command.throttle)
                    .iter()
                    .map(|&v| f64::from(v))
                    .collect()
            }
            SensorField::Vbat => bytemuck::cast_slice::<Volts, f32>(&self.vbat.values)
                .iter()
                .map(|&v| f64::from(v))
                .collect(),
            SensorField::Rssi => self.rssi.values.iter().map(|&v| f64::from(v)).collect(),
            SensorField::Heading => {
                // Prefer airframe heading (attitude.yaw), fall back to GPS
                // course-over-ground when attitude isn't available. The two
                // semantically differ during hover/crab; the airframe value
                // is what the legacy `field(Heading)` returned for AP/PX4/
                // MAVLink before the typed-Session refactor.
                if self.attitude.values.yaw.is_empty() {
                    self.gps
                        .as_ref()
                        .map(|g| g.heading.iter().map(|&v| f64::from(v)).collect())
                        .unwrap_or_default()
                } else {
                    self.attitude
                        .values
                        .yaw
                        .iter()
                        .map(|&v| f64::from(v))
                        .collect()
                }
            }
            SensorField::GpsLat => self
                .gps
                .as_ref()
                .map(|g| bytemuck::cast_slice::<DecimalDegrees, f64>(&g.lat).to_vec())
                .unwrap_or_default(),
            SensorField::GpsLng => self
                .gps
                .as_ref()
                .map(|g| bytemuck::cast_slice::<DecimalDegrees, f64>(&g.lng).to_vec())
                .unwrap_or_default(),
            SensorField::Altitude => self
                .gps
                .as_ref()
                .map(|g| {
                    bytemuck::cast_slice::<Meters, f32>(&g.alt)
                        .iter()
                        .map(|&v| f64::from(v))
                        .collect()
                })
                .unwrap_or_default(),
            SensorField::GpsSpeed => self
                .gps
                .as_ref()
                .map(|g| {
                    bytemuck::cast_slice::<MetersPerSec, f32>(&g.speed)
                        .iter()
                        .map(|&v| f64::from(v))
                        .collect()
                })
                .unwrap_or_default(),
            SensorField::PidP(_)
            | SensorField::PidI(_)
            | SensorField::PidD(_)
            | SensorField::Feedforward(_) => {
                Vec::new() // PID terms not modelled directly yet
            }
            SensorField::Unknown(name) => self
                .extras
                .get(name)
                .map(|ts| ts.values.clone())
                .unwrap_or_default(),
        }
    }

    /// Convenience: firmware string from `meta`.
    pub fn firmware_version(&self) -> &str {
        &self.meta.firmware
    }

    /// Convenience: craft name from `meta`, or empty string if absent.
    pub fn craft_name(&self) -> &str {
        self.meta.craft_name.as_deref().unwrap_or("")
    }

    /// Convenience: motor count from `meta`.
    pub fn motor_count(&self) -> usize {
        self.meta.motor_count
    }

    /// Convenience: truncated flag from `meta`.
    pub fn is_truncated(&self) -> bool {
        self.meta.truncated
    }

    /// Convenience: corrupt-byte count from `meta`.
    pub fn corrupt_bytes(&self) -> usize {
        self.meta.corrupt_bytes
    }

    /// Convenience: warning slice from `meta`.
    pub fn warnings(&self) -> &[Warning] {
        &self.meta.warnings
    }

    /// Number of gyro samples — the canonical "frame count" for the session.
    pub fn frame_count(&self) -> usize {
        self.gyro.len()
    }

    /// Sample rate of the gyro stream in Hz. Alias for [`Self::gyro_rate_hz`].
    pub fn sample_rate_hz(&self) -> f64 {
        self.gyro_rate_hz()
    }

    /// Total flight duration in seconds, computed from the longest-spanning
    /// stream (typically gyro).
    pub fn duration_seconds(&self) -> f64 {
        let candidates = [
            self.gyro.time_us.first().zip(self.gyro.time_us.last()),
            self.accel.time_us.first().zip(self.accel.time_us.last()),
            self.rc_command
                .time_us
                .first()
                .zip(self.rc_command.time_us.last()),
            self.motors.time_us.first().zip(self.motors.time_us.last()),
        ];
        candidates
            .into_iter()
            .filter_map(|pair| pair.map(|(a, b)| b.saturating_sub(*a)))
            .max()
            .map_or(0.0, |dt_us| dt_us.az::<f64>() / 1_000_000.0)
    }

    /// Sample rate of the gyro stream in Hz, or 0.0 if no gyro data.
    pub fn gyro_rate_hz(&self) -> f64 {
        rate_hz(&self.gyro.time_us)
    }
}

fn empty_filter_config() -> FilterConfig {
    FilterConfig {
        gyro_lpf_hz: None,
        gyro_lpf2_hz: None,
        dterm_lpf_hz: None,
        dyn_notch_min_hz: None,
        dyn_notch_max_hz: None,
        gyro_notch1_hz: None,
        gyro_notch2_hz: None,
    }
}

/// Estimate sample rate (Hz) of a uniformly-sampled time axis.
fn rate_hz(time_us: &[u64]) -> f64 {
    if time_us.len() < 2 {
        return 0.0;
    }
    let dt_us = time_us[time_us.len() - 1].saturating_sub(time_us[0]);
    if dt_us == 0 {
        return 0.0;
    }
    let n = time_us.len() - 1;
    n.az::<f64>() / (dt_us.az::<f64>() / 1_000_000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triaxial_iter_yields_all_three_axes() {
        let t = Triaxial::new(1, 2, 3);
        let collected: Vec<_> = t.iter().map(|(_, v)| *v).collect();
        assert_eq!(collected, vec![1, 2, 3]);
    }

    #[test]
    fn triaxial_map_preserves_order() {
        let t = Triaxial::new(1, 2, 3);
        let doubled = t.map(|v| v * 2);
        assert_eq!(doubled.roll, 2);
        assert_eq!(doubled.pitch, 4);
        assert_eq!(doubled.yaw, 6);
    }

    #[test]
    fn time_series_default_is_empty() {
        let ts: TimeSeries<f64> = TimeSeries::new();
        assert!(ts.is_empty());
        assert_eq!(ts.len(), 0);
        assert_eq!(ts.duration_us(), 0);
    }

    #[test]
    fn time_series_push_records_pair() {
        let mut ts: TimeSeries<f64> = TimeSeries::new();
        ts.push(0, 1.0);
        ts.push(1000, 2.0);
        assert_eq!(ts.len(), 2);
        assert_eq!(ts.duration_us(), 1000);
    }

    #[test]
    fn gyro_rate_estimate_is_reasonable() {
        let mut s = Session::default();
        s.gyro.time_us = (0..1000).map(|i| i * 250).collect(); // 4 kHz
        s.gyro.values.roll = vec![DegPerSec(0.0); 1000];
        s.gyro.values.pitch = vec![DegPerSec(0.0); 1000];
        s.gyro.values.yaw = vec![DegPerSec(0.0); 1000];
        let rate = s.gyro_rate_hz();
        assert!((rate - 4000.0).abs() < 5.0, "got {rate}");
    }

    #[test]
    fn cast_slice_works_through_triaxial_series() {
        let mut s: TriaxialSeries<DegPerSec> = TriaxialSeries::new();
        s.time_us = vec![0, 250, 500];
        s.values.roll = vec![DegPerSec(1.0), DegPerSec(2.0), DegPerSec(3.0)];
        let raw: &[f64] = bytemuck::cast_slice(&s.values.roll);
        assert_eq!(raw, &[1.0, 2.0, 3.0]);
    }
}
