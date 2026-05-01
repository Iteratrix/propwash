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
    // lacks the absolute angles. Roll/pitch padded with NaN so all
    // three axes stay length-aligned with attitude.time_us.
    if s.attitude.values.yaw.is_empty() {
        if let Some(t) = topic("VFR_HUD") {
            if let Some(hd) = t.column("heading") {
                let n = t.timestamps.len();
                s.attitude.time_us = t.timestamps.clone();
                s.attitude.values.yaw = hd.iter().map(|&v| v.az::<f32>()).collect();
                s.attitude.values.roll = vec![f32::NAN; n];
                s.attitude.values.pitch = vec![f32::NAN; n];
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
        if gps.has_data() {
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
/// stray HEARTBEATs sent by the GCS itself.
///
/// `MAV_AUTOPILOT_INVALID = 8` is filtered out before tallying — a GCS
/// emitting an INVALID heartbeat shouldn't outvote the airframe's real
/// autopilot ID. On a tie between buckets, prefer any non-Generic
/// family so a single ArduPilot/PX4 heartbeat beats a flood of zeros.
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
    let mut counts = [0u32; 256];
    for &v in col {
        let id = v.saturating_as::<u8>();
        if id == 8 {
            continue; // MAV_AUTOPILOT_INVALID — never the airframe's truth
        }
        counts[id as usize] = counts[id as usize].saturating_add(1);
    }
    // Pick the bucket with the highest count, preferring non-Generic
    // (id != 0) on ties. Iterating in id order means id 0 wins ties
    // by default — flip that with a secondary key.
    let (best, _) = counts
        .iter()
        .enumerate()
        .max_by_key(|(id, &n)| (n, u32::from(*id != 0)))
        .unwrap_or((0, &0));
    if counts[best] == 0 {
        return AutopilotFamily::Generic;
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::mavlink::types::MavlinkParseStats;
    use std::collections::HashMap;

    fn synthetic_parsed(
        topics: HashMap<String, MsgColumns>,
        params: HashMap<String, f64>,
    ) -> MavlinkParsed {
        MavlinkParsed {
            topics,
            firmware_version: String::new(),
            vehicle_type: MavType::Quadrotor,
            params,
            status_messages: Vec::new(),
            stats: MavlinkParseStats::default(),
        }
    }

    fn one_topic(
        name: &str,
        fields: Vec<&str>,
        ts: Vec<u64>,
        cols: Vec<Vec<f64>>,
    ) -> HashMap<String, MsgColumns> {
        let mut topics = HashMap::new();
        let names: Vec<String> = fields.iter().map(|s| (*s).to_string()).collect();
        let mut mc = MsgColumns::new(names);
        for (i, &t) in ts.iter().enumerate() {
            let row: Vec<f64> = cols.iter().map(|c| c[i]).collect();
            mc.push_row(t, &row);
        }
        topics.insert(name.to_string(), mc);
        topics
    }

    /// bug_006: ATTITUDE.{roll,pitch,yaw} radians must convert to
    /// attitude.values.{roll,pitch,yaw} in degrees.
    #[test]
    fn mavlink_attitude_radians_to_degrees() {
        let topics = one_topic(
            "ATTITUDE",
            vec![
                "rollspeed",
                "pitchspeed",
                "yawspeed",
                "roll",
                "pitch",
                "yaw",
            ],
            vec![1000, 2000],
            vec![
                vec![0.0, 0.0],
                vec![0.0, 0.0],
                vec![0.0, 0.0],
                vec![std::f64::consts::FRAC_PI_2, std::f64::consts::PI], // 90°, 180°
                vec![0.0, std::f64::consts::FRAC_PI_4],                  // 0°, 45°
                vec![std::f64::consts::PI, 0.0],                         // 180°, 0°
            ],
        );
        let parsed = synthetic_parsed(topics, HashMap::new());
        let s = super::session(parsed, vec![], 1);
        assert!((s.attitude.values.roll[0] - 90.0).abs() < 0.01);
        assert!((s.attitude.values.roll[1] - 180.0).abs() < 0.01);
        assert!((s.attitude.values.pitch[1] - 45.0).abs() < 0.01);
        assert!((s.attitude.values.yaw[0] - 180.0).abs() < 0.01);
    }

    /// bug_016: a HEARTBEAT.autopilot=12 (PX4) tlog with PX4 parameter
    /// names must yield populated `pid_gains` from the PX4 parser, not
    /// an empty `Some` from the AP parser path.
    #[test]
    fn mavlink_px4_autopilot_dispatches_to_px4_pid_helper() {
        let topics = one_topic(
            "HEARTBEAT",
            vec!["autopilot"],
            vec![1000, 2000, 3000],
            vec![vec![12.0, 12.0, 12.0]], // MAV_AUTOPILOT_PX4
        );
        let mut params = HashMap::new();
        params.insert("MC_ROLLRATE_P".to_string(), 0.15);
        params.insert("MC_ROLLRATE_I".to_string(), 0.20);
        params.insert("MC_ROLLRATE_D".to_string(), 0.003);
        params.insert("MC_PITCHRATE_P".to_string(), 0.16);
        params.insert("MC_YAWRATE_P".to_string(), 0.30);
        // AP keys deliberately absent — pre-fix path would have read these
        // and returned all-None gains.
        let parsed = synthetic_parsed(topics, params);
        let s = super::session(parsed, vec![], 1);
        let pid = s
            .meta
            .pid_gains
            .as_ref()
            .expect("expected PX4 dispatch to populate pid_gains");
        assert!(
            pid.has_data(),
            "PX4 PID gains should be populated, not empty"
        );
        let roll = pid.get(crate::types::Axis::Roll);
        assert!(
            roll.p.is_some_and(|p| p > 0),
            "MC_ROLLRATE_P must populate roll P"
        );
    }

    /// And the inverse: HEARTBEAT.autopilot=3 (ArduPilot) must dispatch
    /// to the AP helper.
    #[test]
    fn mavlink_ardupilot_autopilot_dispatches_to_ap_pid_helper() {
        let topics = one_topic(
            "HEARTBEAT",
            vec!["autopilot"],
            vec![1000, 2000],
            vec![vec![3.0, 3.0]], // MAV_AUTOPILOT_ARDUPILOTMEGA
        );
        let mut params = HashMap::new();
        params.insert("ATC_RAT_RLL_P".to_string(), 0.25);
        params.insert("ATC_RAT_RLL_I".to_string(), 0.18);
        let parsed = synthetic_parsed(topics, params);
        let s = super::session(parsed, vec![], 1);
        let pid = s.meta.pid_gains.expect("AP dispatch should populate");
        let roll = pid.get(crate::types::Axis::Roll);
        assert!(roll.p.is_some_and(|p| p > 0));
    }

    /// Generic / unknown autopilot must yield None (not Some(empty)) so
    /// `Option::is_some` is a reliable "have PID info" signal.
    #[test]
    fn mavlink_unknown_autopilot_yields_none_not_empty_some() {
        let topics = one_topic(
            "HEARTBEAT",
            vec!["autopilot"],
            vec![1000],
            vec![vec![8.0]], // MAV_AUTOPILOT_INVALID
        );
        let parsed = synthetic_parsed(topics, HashMap::new());
        let s = super::session(parsed, vec![], 1);
        assert!(
            s.meta.pid_gains.is_none(),
            "unknown autopilot must yield None"
        );
        assert!(s.meta.filter_config.is_none());
    }
}
