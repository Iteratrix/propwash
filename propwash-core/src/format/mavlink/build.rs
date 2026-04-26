//! Translate a [`MavlinkParsed`] into a typed [`Session`].
#![allow(
    clippy::assigning_clones,        // Session fields start empty; clone_from buys nothing
    clippy::needless_pass_by_value,  // signature symmetry across format build()s
)]
//!
//! All MAVLink-specific unit conversions:
//!  - `ATTITUDE.{rollspeed,pitchspeed,yawspeed}` rad/s → `DegPerSec`
//!  - `RAW_IMU.{xgyro,ygyro,zgyro}` mrad/s → `DegPerSec` (only if no ATTITUDE)
//!  - `RAW_IMU.{xacc,yacc,zacc}` mG → `MetersPerSec2`
//!  - `SERVO_OUTPUT_RAW.servoN_raw` PWM → `Normalized01`
//!  - `RC_CHANNELS.chanN_raw` PWM → sticks/throttle
//!  - `VFR_HUD` doesn't carry vbat directly, but `BATTERY_STATUS` /
//!    `SYS_STATUS.voltage_battery` (mV) does
//!  - `GPS_RAW_INT.{lat,lon}` int×10⁷ → `DecimalDegrees`
//!  - `HEARTBEAT.base_mode` `MAV_MODE_FLAG_SAFETY_ARMED` bit → armed
//!  - `HEARTBEAT.custom_mode` → `FlightMode` (vehicle-type-aware)

use az::{Az, SaturatingAs};

use super::parser::MavlinkParsed;
use super::types::{MavType, MsgColumns};
use crate::format::common::{
    ardupilot_filter_config, ardupilot_pid_gains, px4_filter_config, px4_pid_gains, AutopilotFamily,
};
use crate::session::{
    Event, EventKind, FlightMode, Format, Gps, LogSeverity, Session, SessionMeta, TimeSeries,
};
use crate::types::Warning;
use crate::units::{
    Amps, DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2, Normalized01, Volts,
};

const RAD_TO_DEG: f64 = 57.295_779_513_082_32;

#[allow(clippy::too_many_lines)] // declarative message-to-Session mapping
pub(crate) fn session(
    parsed: MavlinkParsed,
    warnings: Vec<Warning>,
    session_index: usize,
) -> Session {
    let mut s = Session::default();

    let topic = |name: &str| -> Option<&MsgColumns> { parsed.topics.get(name) };

    // ── Gyro: ATTITUDE rollspeed/pitchspeed/yawspeed (rad/s → DegPerSec) ───
    if let Some(t) = topic("ATTITUDE") {
        if let (Some(r), Some(p), Some(y)) = (
            t.column("rollspeed"),
            t.column("pitchspeed"),
            t.column("yawspeed"),
        ) {
            s.gyro.time_us = t.timestamps.clone();
            s.gyro.values.roll = r.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
            s.gyro.values.pitch = p.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
            s.gyro.values.yaw = y.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        }
        // Attitude (airframe orientation) from the same ATTITUDE message,
        // radians → degrees. Falls back to VFR_HUD.heading for yaw if
        // ATTITUDE.{roll,pitch,yaw} aren't present.
        if let (Some(r), Some(p), Some(y)) = (t.column("roll"), t.column("pitch"), t.column("yaw"))
        {
            s.attitude.time_us = t.timestamps.clone();
            s.attitude.values.roll = r.iter().map(|v| (v * RAD_TO_DEG).az::<f32>()).collect();
            s.attitude.values.pitch = p.iter().map(|v| (v * RAD_TO_DEG).az::<f32>()).collect();
            s.attitude.values.yaw = y.iter().map(|v| (v * RAD_TO_DEG).az::<f32>()).collect();
        }
    }
    // VFR_HUD.heading (degrees) as a yaw-only fallback when ATTITUDE
    // lacks the absolute angles.
    if s.attitude.values.yaw.is_empty() {
        if let Some(t) = topic("VFR_HUD") {
            if let Some(hd) = t.column("heading") {
                s.attitude.time_us = t.timestamps.clone();
                s.attitude.values.yaw = hd.iter().map(|&v| v.az::<f32>()).collect();
            }
        }
    }

    // ── Accel: RAW_IMU then SCALED_IMU (mG → m/s²) ──────────────────────────
    let imu_source = topic("RAW_IMU").or_else(|| topic("SCALED_IMU"));
    if let Some(t) = imu_source {
        if let (Some(x), Some(y), Some(z)) = (t.column("xacc"), t.column("yacc"), t.column("zacc"))
        {
            s.accel.time_us = t.timestamps.clone();
            s.accel.values.roll = x
                .iter()
                .map(|v| MetersPerSec2(v * 0.001 * 9.806_65))
                .collect();
            s.accel.values.pitch = y
                .iter()
                .map(|v| MetersPerSec2(v * 0.001 * 9.806_65))
                .collect();
            s.accel.values.yaw = z
                .iter()
                .map(|v| MetersPerSec2(v * 0.001 * 9.806_65))
                .collect();
        }
        // Fallback gyro from RAW_IMU if no ATTITUDE:
        if s.gyro.is_empty() {
            if let (Some(x), Some(y), Some(z)) =
                (t.column("xgyro"), t.column("ygyro"), t.column("zgyro"))
            {
                s.gyro.time_us = t.timestamps.clone();
                s.gyro.values.roll = x
                    .iter()
                    .map(|v| DegPerSec(v * 0.001 * RAD_TO_DEG))
                    .collect();
                s.gyro.values.pitch = y
                    .iter()
                    .map(|v| DegPerSec(v * 0.001 * RAD_TO_DEG))
                    .collect();
                s.gyro.values.yaw = z
                    .iter()
                    .map(|v| DegPerSec(v * 0.001 * RAD_TO_DEG))
                    .collect();
            }
        }
    }

    // ── Motor outputs: SERVO_OUTPUT_RAW.servoN_raw ──────────────────────────
    if let Some(t) = topic("SERVO_OUTPUT_RAW") {
        s.motors.time_us = t.timestamps.clone();
        let mut motor_count = 0;
        for i in 1..=8 {
            let name = format!("servo{i}_raw");
            if let Some(col) = t.column(&name) {
                let normalized: Vec<Normalized01> = col
                    .iter()
                    .map(|&v| Normalized01((((v - 1000.0) / 1000.0).az::<f32>()).clamp(0.0, 1.0)))
                    .collect();
                s.motors.commands.push(normalized);
                motor_count += 1;
            } else {
                break;
            }
        }
        s.meta.motor_count = motor_count.max(parsed.vehicle_type.motor_count().unwrap_or(0));
    }

    // ── RC inputs: RC_CHANNELS.chanN_raw ───────────────────────────────────
    let rc_source = topic("RC_CHANNELS").or_else(|| topic("RC_CHANNELS_RAW"));
    if let Some(t) = rc_source {
        s.rc_command.time_us = t.timestamps.clone();
        if let Some(col) = t.column("chan1_raw") {
            s.rc_command.sticks.roll = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("chan2_raw") {
            s.rc_command.sticks.pitch = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("chan4_raw") {
            s.rc_command.sticks.yaw = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("chan3_raw") {
            s.rc_command.throttle = col
                .iter()
                .map(|&v| Normalized01((((v - 1000.0) / 1000.0).az::<f32>()).clamp(0.0, 1.0)))
                .collect();
        }
        if let Some(col) = t.column("rssi") {
            s.rssi = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| v.az::<f32>()).collect(),
            );
        }
    }

    // ── Battery: SYS_STATUS.voltage_battery (mV) / current_battery (cA) ────
    if let Some(t) = topic("SYS_STATUS") {
        if let Some(col) = t.column("voltage_battery") {
            s.vbat = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter()
                    .map(|&v| Volts((v * 0.001).az::<f32>()))
                    .collect(),
            );
        }
        if let Some(col) = t.column("current_battery") {
            // cA → A
            s.current = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| Amps((v * 0.01).az::<f32>())).collect(),
            );
        }
    }

    // ── GPS: GPS_RAW_INT ───────────────────────────────────────────────────
    if let Some(t) = topic("GPS_RAW_INT") {
        let mut gps = Gps {
            time_us: t.timestamps.clone(),
            ..Gps::default()
        };
        if let Some(col) = t.column("lat") {
            gps.lat = col.iter().map(|&v| DecimalDegrees(v * 1e-7)).collect();
        }
        if let Some(col) = t.column("lon") {
            gps.lng = col.iter().map(|&v| DecimalDegrees(v * 1e-7)).collect();
        }
        if let Some(col) = t.column("alt") {
            // alt mm
            gps.alt = col
                .iter()
                .map(|&v| Meters((v * 0.001).az::<f32>()))
                .collect();
        }
        if let Some(col) = t.column("vel") {
            // cm/s
            gps.speed = col
                .iter()
                .map(|&v| MetersPerSec((v * 0.01).az::<f32>()))
                .collect();
        }
        if let Some(col) = t.column("cog") {
            // 0.01 deg
            gps.heading = col.iter().map(|&v| (v * 0.01).az::<f32>()).collect();
        }
        if let Some(col) = t.column("satellites_visible") {
            gps.sats = col.iter().map(|&v| v.saturating_as::<u8>()).collect();
        }
        if !gps.is_empty() {
            s.gps = Some(gps);
        }
    }

    // ── HEARTBEAT → armed + flight_mode + edge events ─────────────────────
    if let Some(t) = topic("HEARTBEAT") {
        let base_mode_col = t.column("base_mode");
        let custom_mode_col = t.column("custom_mode");
        if let Some(base) = base_mode_col {
            let mut last_armed = false;
            for (&time, &b) in t.timestamps.iter().zip(base.iter()) {
                let is_armed = (b.az::<u32>() & 0b1000_0000) != 0; // MAV_MODE_FLAG_SAFETY_ARMED
                if is_armed != last_armed {
                    s.armed.push(time, is_armed);
                    s.events.push(Event {
                        time_us: time,
                        kind: if is_armed {
                            EventKind::Armed
                        } else {
                            EventKind::Disarmed
                        },
                        message: None,
                    });
                    last_armed = is_armed;
                }
            }
        }
        if let Some(cm) = custom_mode_col {
            let mut last_mode = u32::MAX;
            for (&time, &m) in t.timestamps.iter().zip(cm.iter()) {
                let id = m.az::<u32>();
                if id != last_mode {
                    let mode = mavlink_flight_mode(id, parsed.vehicle_type);
                    s.flight_mode.push(time, mode.clone());
                    s.events.push(Event {
                        time_us: time,
                        kind: EventKind::ModeChange { to: mode },
                        message: None,
                    });
                    last_mode = id;
                }
            }
        }
    }

    // ── STATUSTEXT → Session.events ───────────────────────────────────────
    for msg in &parsed.status_messages {
        let severity = match msg.severity {
            super::types::Severity::Emergency => LogSeverity::Emergency,
            super::types::Severity::Alert => LogSeverity::Alert,
            super::types::Severity::Critical => LogSeverity::Critical,
            super::types::Severity::Error => LogSeverity::Error,
            super::types::Severity::Warning => LogSeverity::Warning,
            super::types::Severity::Notice => LogSeverity::Notice,
            super::types::Severity::Info => LogSeverity::Info,
            super::types::Severity::Debug => LogSeverity::Debug,
        };
        s.events.push(Event {
            time_us: msg.timestamp_us,
            kind: EventKind::LogMessage { severity },
            message: Some(msg.text.clone()),
        });
    }

    // ── Detect autopilot family from HEARTBEAT.autopilot ──────────────────
    // MAVLink is the wire protocol for both ArduPilot and PX4. Parameter
    // naming differs (ATC_RAT_RLL_P vs MC_ROLLRATE_P), so the right
    // dispatch is chosen by the autopilot byte in HEARTBEAT. Mode = the
    // most-frequent value across the log to be robust against stray
    // GCS-sourced HEARTBEATs (autopilot=8 = MAV_AUTOPILOT_INVALID).
    let autopilot = detect_autopilot(&parsed);
    let (pid_gains, filter_config) = match autopilot {
        AutopilotFamily::ArduPilot => (
            Some(ardupilot_pid_gains(&parsed.params)),
            Some(ardupilot_filter_config(&parsed.params)),
        ),
        AutopilotFamily::Px4 => (
            Some(px4_pid_gains(&parsed.params)),
            Some(px4_filter_config(&parsed.params)),
        ),
        // Unknown / Generic / Other — return None instead of "Some(empty)"
        // so consumers can distinguish "no PID info" from "all gains
        // happened to be zero".
        AutopilotFamily::Generic | AutopilotFamily::Other(_) => (None, None),
    };

    // ── Metadata ──────────────────────────────────────────────────────────
    s.meta = SessionMeta {
        format: Format::Mavlink,
        firmware: parsed.firmware_version.clone(),
        craft_name: Some(parsed.vehicle_type.as_str().to_string()),
        board: None,
        motor_count: s.meta.motor_count,
        pid_gains,
        filter_config,
        session_index,
        truncated: parsed.stats.truncated,
        corrupt_bytes: parsed.stats.corrupt_bytes,
        warnings,
    };

    let _erpm: Vec<Erpm> = Vec::new(); // suppress unused-import if MAVLink doesn't have ESC RPM yet

    s
}

/// Pick the most common `HEARTBEAT.autopilot` value from the parsed log
/// and translate to [`AutopilotFamily`]. Mode-of-many is robust against
/// stray HEARTBEATs sent by the GCS itself (`MAV_AUTOPILOT_INVALID = 8`).
/// Returns [`AutopilotFamily::Generic`] when no HEARTBEATs were seen.
fn detect_autopilot(parsed: &MavlinkParsed) -> AutopilotFamily {
    let Some(t) = parsed.topics.get("HEARTBEAT") else {
        return AutopilotFamily::Generic;
    };
    let Some(col) = t.column("autopilot") else {
        return AutopilotFamily::Generic;
    };
    if col.is_empty() {
        return AutopilotFamily::Generic;
    }
    // Tally mode of u8 values (256 buckets).
    let mut counts = [0u32; 256];
    for &v in col {
        let id = v.saturating_as::<u8>();
        counts[id as usize] = counts[id as usize].saturating_add(1);
    }
    let (best, _) = counts
        .iter()
        .enumerate()
        .max_by_key(|(_, &n)| n)
        .unwrap_or((0, &0));
    AutopilotFamily::from_id(best.saturating_as::<u8>())
}

fn stick_norm(pwm: f64) -> f32 {
    (((pwm - 1500.0) / 500.0).az::<f32>()).clamp(-1.0, 1.0)
}

/// `MAVLink` `HEARTBEAT.custom_mode` interpretation depends on the vehicle's
/// firmware family. `ArduPilot` Copter `custom_mode` IDs ≈ MODE message IDs
/// from the AP build pass, so we reuse that mapping for Quadrotor/Hexarotor/etc.
/// PX4 / Plane / Rover use different IDs — fall back to Other.
fn mavlink_flight_mode(custom_mode: u32, vehicle: MavType) -> FlightMode {
    match vehicle {
        MavType::Quadrotor | MavType::Hexarotor | MavType::Octorotor | MavType::Tricopter => {
            // Same IDs as ArduCopter MODE messages
            match custom_mode {
                0 => FlightMode::Stabilize,
                1 => FlightMode::Acro,
                2 => FlightMode::AltHold,
                3 => FlightMode::Auto,
                5 => FlightMode::Loiter,
                6 => FlightMode::ReturnToHome,
                9 => FlightMode::Land,
                16 => FlightMode::PosHold,
                23 => FlightMode::Failsafe,
                other => FlightMode::Other(format!("MODE_{other}")),
            }
        }
        _ => FlightMode::Other(format!("CUSTOM_{custom_mode}")),
    }
}
