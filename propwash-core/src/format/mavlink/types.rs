//! MAVLink parser-internal data types: MAV_TYPE, severity, status messages,
//! parse stats. No `MavlinkSession` — parser writes into
//! [`crate::session::Session`] via [`super::build`].

pub use crate::format::common::MsgColumns;

/// `MAV_TYPE` — vehicle type from HEARTBEAT.
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

/// MAVLink message severity from STATUSTEXT.
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
