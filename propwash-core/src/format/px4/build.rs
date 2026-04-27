//! Translate a [`Px4Parsed`] into a typed [`Session`].
#![allow(
    clippy::assigning_clones,        // Session fields start empty; clone_from buys nothing
    clippy::needless_pass_by_value,  // signature symmetry across format build()s
)]
//!
//! All PX4-specific unit conversions live here:
//!  - `vehicle_angular_velocity.xyz[i]` rad/s → `DegPerSec`
//!  - `sensor_combined.gyro_rad[i]` rad/s → `DegPerSec`
//!  - `sensor_accel.{x,y,z}` m/s² → `MetersPerSec2`
//!  - `vehicle_rates_setpoint.{roll,pitch,yaw}` rad/s → `DegPerSec`
//!  - `actuator_outputs.output[i]` already 0–1 (or 1000–2000 PWM in some configs)
//!  - `input_rc.values[i]` PWM 1000–2000 → sticks/throttle Normalized01
//!  - `battery_status.voltage_v / current_a` → typed
//!  - `vehicle_gps_position.{lat,lon}` int×10⁷ → `DecimalDegrees`

use az::{Az, SaturatingAs};

use super::parser::Px4Parsed;
use super::types::TopicData;
use crate::session::{
    Event, EventKind, FlightMode, Format, Gps, LogSeverity, Session, SessionMeta, TimeSeries,
};
use crate::types::Warning;
use crate::units::{
    Amps, DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2, Normalized01, Volts,
};

const RAD_TO_DEG: f64 = 57.295_779_513_082_32;

#[allow(clippy::too_many_lines)] // declarative topic-to-Session mapping
pub(crate) fn session(parsed: Px4Parsed, warnings: Vec<Warning>, session_index: usize) -> Session {
    let mut s = Session::default();

    let topic = |name: &str| -> Option<&TopicData> {
        let msg_id = parsed
            .subscriptions
            .values()
            .filter(|sub| sub.format_name == name)
            .min_by_key(|sub| sub.multi_id)
            .map(|sub| sub.msg_id)?;
        parsed.topics.get(&msg_id)
    };

    // ── Gyro: prefer vehicle_angular_velocity, fall back to sensor_combined / sensor_gyro
    let gyro_sources = [
        ("vehicle_angular_velocity", ["xyz[0]", "xyz[1]", "xyz[2]"]),
        (
            "sensor_combined",
            ["gyro_rad[0]", "gyro_rad[1]", "gyro_rad[2]"],
        ),
        ("sensor_gyro", ["x", "y", "z"]),
    ];
    for (name, fields) in &gyro_sources {
        if let Some(t) = topic(name) {
            if let (Some(rx), Some(ry), Some(rz)) = (
                t.column(fields[0]),
                t.column(fields[1]),
                t.column(fields[2]),
            ) {
                s.gyro.time_us = t.timestamps.clone();
                s.gyro.values.roll = rx.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
                s.gyro.values.pitch = ry.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
                s.gyro.values.yaw = rz.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
                break;
            }
        }
    }

    // ── Accel ─────────────────────────────────────────────────────────────
    if let Some(t) = topic("sensor_combined") {
        if let (Some(x), Some(y), Some(z)) = (
            t.column("accelerometer_m_s2[0]"),
            t.column("accelerometer_m_s2[1]"),
            t.column("accelerometer_m_s2[2]"),
        ) {
            s.accel.time_us = t.timestamps.clone();
            s.accel.values.roll = x.iter().copied().map(MetersPerSec2).collect();
            s.accel.values.pitch = y.iter().copied().map(MetersPerSec2).collect();
            s.accel.values.yaw = z.iter().copied().map(MetersPerSec2).collect();
        }
    }
    if s.accel.is_empty() {
        if let Some(t) = topic("sensor_accel") {
            if let (Some(x), Some(y), Some(z)) = (t.column("x"), t.column("y"), t.column("z")) {
                s.accel.time_us = t.timestamps.clone();
                s.accel.values.roll = x.iter().copied().map(MetersPerSec2).collect();
                s.accel.values.pitch = y.iter().copied().map(MetersPerSec2).collect();
                s.accel.values.yaw = z.iter().copied().map(MetersPerSec2).collect();
            }
        }
    }

    // ── Setpoint ──────────────────────────────────────────────────────────
    if let Some(t) = topic("vehicle_rates_setpoint") {
        if let (Some(r), Some(p), Some(y)) = (t.column("roll"), t.column("pitch"), t.column("yaw"))
        {
            s.setpoint.time_us = t.timestamps.clone();
            s.setpoint.values.roll = r.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
            s.setpoint.values.pitch = p.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
            s.setpoint.values.yaw = y.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        }
    }

    // ── Attitude (airframe orientation, degrees) ──────────────────────────
    // Prefer vehicle_attitude.q (quaternion → Euler), fall back to
    // vehicle_global_position.yaw (already EKF-filtered, but only yaw).
    if let Some(t) = topic("vehicle_attitude") {
        if let (Some(qw), Some(qx), Some(qy), Some(qz)) = (
            t.column("q[0]"),
            t.column("q[1]"),
            t.column("q[2]"),
            t.column("q[3]"),
        ) {
            s.attitude.time_us = t.timestamps.clone();
            s.attitude.values.roll = Vec::with_capacity(qw.len());
            s.attitude.values.pitch = Vec::with_capacity(qw.len());
            s.attitude.values.yaw = Vec::with_capacity(qw.len());
            for (((&w, &x), &y), &z) in qw.iter().zip(qx.iter()).zip(qy.iter()).zip(qz.iter()) {
                let (roll, pitch, yaw) = quat_to_euler_deg(w, x, y, z);
                s.attitude.values.roll.push(roll);
                s.attitude.values.pitch.push(pitch);
                s.attitude.values.yaw.push(yaw);
            }
        }
    }
    if s.attitude.is_empty() {
        if let Some(t) = topic("vehicle_global_position") {
            if let Some(yaw) = t.column("yaw") {
                s.attitude.time_us = t.timestamps.clone();
                s.attitude.values.yaw = yaw.iter().map(|&v| (v * RAD_TO_DEG).az::<f32>()).collect();
                // roll/pitch left empty; only yaw is fused into the global pos topic.
            }
        }
    }

    // ── Motor outputs ─────────────────────────────────────────────────────
    // PX4's `actuator_outputs.output[0..15]` is a fixed-width array; not
    // every channel is wired. Stop only on schema absence (`column` →
    // None) — never on runtime values, since pre-arm logs and
    // VTOL/quadplane mixers can legitimately leave low-index channels at
    // 0.0 throughout the recording while the actual motors live at
    // higher indices. Empty (all-zero) channels still get pushed so
    // motor indices stay aligned with the source schema.
    if let Some(t) = topic("actuator_outputs") {
        s.motors.time_us = t.timestamps.clone();
        let mut motor_count = 0;
        for i in 0..16 {
            let name = format!("output[{i}]");
            let Some(col) = t.column(&name) else { break };
            let normalized: Vec<Normalized01> = col
                .iter()
                .map(|&v| Normalized01(normalize_output(v)))
                .collect();
            s.motors.commands.push(normalized);
            motor_count += 1;
        }
        s.meta.motor_count = motor_count;
    }

    // ── RC inputs ─────────────────────────────────────────────────────────
    if let Some(t) = topic("input_rc") {
        s.rc_command.time_us = t.timestamps.clone();
        if let Some(col) = t.column("values[0]") {
            s.rc_command.sticks.roll = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("values[1]") {
            s.rc_command.sticks.pitch = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("values[3]") {
            s.rc_command.sticks.yaw = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("values[2]") {
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

    // ── Battery ───────────────────────────────────────────────────────────
    if let Some(t) = topic("battery_status") {
        if let Some(col) = t.column("voltage_v") {
            s.vbat = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| Volts(v.az::<f32>())).collect(),
            );
        }
        if let Some(col) = t.column("current_a") {
            s.current = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| Amps(v.az::<f32>())).collect(),
            );
        }
    }

    // ── GPS ───────────────────────────────────────────────────────────────
    if let Some(t) = topic("vehicle_gps_position") {
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
            // alt in mm
            gps.alt = col
                .iter()
                .map(|&v| Meters((v / 1000.0).az::<f32>()))
                .collect();
        }
        if let Some(col) = t.column("vel_m_s") {
            gps.speed = col.iter().map(|&v| MetersPerSec(v.az::<f32>())).collect();
        }
        if let Some(col) = t.column("cog_rad") {
            gps.heading = col.iter().map(|&v| (v * RAD_TO_DEG).az::<f32>()).collect();
        }
        if let Some(col) = t.column("satellites_used") {
            gps.sats = col.iter().map(|&v| v.saturating_as::<u8>()).collect();
        }
        if !gps.is_empty() {
            s.gps = Some(gps);
        }
    }

    // ── ESC telemetry ─────────────────────────────────────────────────────
    if let Some(t) = topic("esc_status") {
        let mut erpm: Vec<Vec<Erpm>> = Vec::new();
        for i in 0..8 {
            let name = format!("esc[{i}].esc_rpm");
            if let Some(col) = t.column(&name) {
                erpm.push(col.iter().map(|&v| Erpm(v.max(0.0).az::<u32>())).collect());
            } else {
                break;
            }
        }
        if !erpm.is_empty() {
            s.motors.esc = Some(crate::session::Esc {
                time_us: t.timestamps.clone(),
                erpm,
                ..crate::session::Esc::default()
            });
        }
    }

    // ── vehicle_status → armed + flight_mode + edge events ────────────────
    if let Some(t) = topic("vehicle_status") {
        let armed_col = t.column("arming_state");
        let mode_col = t.column("nav_state");
        if let Some(armed) = armed_col {
            let mut last_armed = false;
            for (&time, &a) in t.timestamps.iter().zip(armed.iter()) {
                let is_armed = a.az::<i64>() == 2; // ARMING_STATE_ARMED = 2
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
        if let Some(mode) = mode_col {
            let mut last_mode_id = u8::MAX;
            for (&time, &m) in t.timestamps.iter().zip(mode.iter()) {
                let id = m.saturating_as::<u8>();
                if id != last_mode_id {
                    let mode = px4_flight_mode(id);
                    s.flight_mode.push(time, mode.clone());
                    s.events.push(Event {
                        time_us: time,
                        kind: EventKind::ModeChange { to: mode },
                        message: None,
                    });
                    last_mode_id = id;
                }
            }
        }
    }

    // ── Firmware log messages → Session.events ────────────────────────────
    for msg in &parsed.log_messages {
        if msg.level >= 7 {
            continue; // skip debug
        }
        let severity = match msg.level {
            0 => LogSeverity::Emergency,
            1 => LogSeverity::Alert,
            2 => LogSeverity::Critical,
            3 => LogSeverity::Error,
            4 => LogSeverity::Warning,
            5 => LogSeverity::Notice,
            6 => LogSeverity::Info,
            _ => LogSeverity::Debug,
        };
        s.events.push(Event {
            time_us: msg.timestamp_us,
            kind: EventKind::LogMessage { severity },
            message: Some(msg.message.clone()),
        });
    }

    // ── Metadata ──────────────────────────────────────────────────────────
    s.meta = SessionMeta {
        format: Format::Px4,
        firmware: parsed.firmware_version.clone(),
        craft_name: if parsed.hardware_name.is_empty() {
            None
        } else {
            Some(parsed.hardware_name.clone())
        },
        board: None,
        motor_count: s.meta.motor_count,
        pid_gains: None,
        filter_config: None,
        session_index,
        truncated: parsed.stats.truncated,
        corrupt_bytes: parsed.stats.corrupt_bytes,
        warnings,
    };

    s
}

/// PX4's `actuator_outputs.output[i]` is either normalised 0-1 (newer
/// firmware in floating-point modes) or a PWM 1000-2000 (legacy). Detect
/// by magnitude and normalise.
fn normalize_output(raw: f64) -> f32 {
    if raw > 50.0 {
        // PWM
        (((raw - 1000.0) / 1000.0).az::<f32>()).clamp(0.0, 1.0)
    } else {
        raw.az::<f32>().clamp(0.0, 1.0)
    }
}

fn stick_norm(pwm: f64) -> f32 {
    (((pwm - 1500.0) / 500.0).az::<f32>()).clamp(-1.0, 1.0)
}

/// Convert a unit quaternion (PX4 `vehicle_attitude.q[0..3]` = w, x, y, z)
/// into Euler angles (roll, pitch, yaw) in degrees using the standard
/// aerospace ZYX convention.
#[allow(
    clippy::similar_names,   // canonical sin*_cos*_ pair names from the literature
    clippy::suboptimal_flops // mul_add rewrites would obscure the formula
)]
fn quat_to_euler_deg(w: f64, x: f64, y: f64, z: f64) -> (f32, f32, f32) {
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * std::f64::consts::FRAC_PI_2
    } else {
        sinp.asin()
    };

    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (
        (roll * RAD_TO_DEG).az::<f32>(),
        (pitch * RAD_TO_DEG).az::<f32>(),
        (yaw * RAD_TO_DEG).az::<f32>(),
    )
}

/// PX4 `vehicle_status.nav_state` (`NAVIGATION_STATE_*`) → common `FlightMode`.
///
/// Mapped against canonical PX4 `msg/vehicle_status.msg` constants stable
/// across PX4 v1.10+. Identical-body arms (`8`/`13` → Failsafe) are kept
/// separate to document the underlying PX4 nav state IDs.
#[allow(clippy::match_same_arms)]
fn px4_flight_mode(state: u8) -> FlightMode {
    match state {
        0 => FlightMode::Manual,
        1 => FlightMode::AltHold,
        2 => FlightMode::PosHold,
        3 => FlightMode::Auto, // AUTO_MISSION
        4 => FlightMode::Loiter,
        5 => FlightMode::ReturnToHome, // AUTO_RTL
        6 => FlightMode::Other("POSITION_SLOW".into()),
        7 => FlightMode::Other("FREE5".into()), // legacy AUTO_RTGS, removed
        8 => FlightMode::Failsafe,              // AUTO_LANDENGFAIL
        9 => FlightMode::Other("FREE3".into()), // legacy AUTO_LANDGPSFAIL, removed
        10 => FlightMode::Acro,
        11 => FlightMode::Other("FREE2".into()), // unused
        12 => FlightMode::Other("DESCEND".into()),
        13 => FlightMode::Failsafe, // TERMINATION
        14 => FlightMode::Other("OFFBOARD".into()),
        15 => FlightMode::Stabilize,
        16 => FlightMode::Other("RATTITUDE_LEGACY".into()), // removed in modern PX4
        17 => FlightMode::Other("AUTO_TAKEOFF".into()),
        18 => FlightMode::Land, // AUTO_LAND
        19 => FlightMode::Other("AUTO_FOLLOW_TARGET".into()),
        20 => FlightMode::Other("AUTO_PRECLAND".into()),
        21 => FlightMode::Other("ORBIT".into()),
        22 => FlightMode::Other("AUTO_VTOL_TAKEOFF".into()),
        other => FlightMode::Other(format!("NAV_{other}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::px4::types::{
        Px4ParseStats, ULogField, ULogFormat, ULogSubscription, ULogType,
    };
    use std::collections::HashMap;

    #[test]
    fn px4_canonical_modes() {
        assert_eq!(px4_flight_mode(0), FlightMode::Manual);
        assert_eq!(px4_flight_mode(1), FlightMode::AltHold);
        assert_eq!(px4_flight_mode(2), FlightMode::PosHold);
        assert_eq!(px4_flight_mode(3), FlightMode::Auto);
        assert_eq!(px4_flight_mode(4), FlightMode::Loiter);
        assert_eq!(px4_flight_mode(5), FlightMode::ReturnToHome);
        assert_eq!(px4_flight_mode(10), FlightMode::Acro);
        assert_eq!(px4_flight_mode(15), FlightMode::Stabilize);
        assert_eq!(px4_flight_mode(18), FlightMode::Land);
        // Acro/Stabilize must NOT be swapped:
        assert_ne!(px4_flight_mode(10), FlightMode::Stabilize);
        assert_ne!(px4_flight_mode(15), FlightMode::Acro);
    }

    /// Construct a minimal `Px4Parsed` containing one topic. The topic
    /// schema is dummy (not used by build's `topic()` lookup beyond
    /// format_name → msg_id matching).
    fn synthetic_parsed(
        topic_name: &str,
        field_names: Vec<&str>,
        timestamps: Vec<u64>,
        columns: Vec<Vec<f64>>,
    ) -> Px4Parsed {
        let mut formats = HashMap::new();
        let mut subscriptions = HashMap::new();
        let mut topics = HashMap::new();
        let names: Vec<String> = field_names.iter().map(|&s| s.to_string()).collect();

        formats.insert(
            topic_name.to_string(),
            ULogFormat {
                name: topic_name.to_string(),
                fields: field_names
                    .iter()
                    .map(|n| ULogField {
                        name: (*n).to_string(),
                        type_name: "uint64_t".into(),
                        primitive: Some(ULogType::UInt64),
                        array_size: None,
                        byte_size: 8,
                    })
                    .collect(),
                total_size: 8 * field_names.len(),
            },
        );
        let msg_id: u16 = 1;
        subscriptions.insert(
            msg_id,
            ULogSubscription {
                msg_id,
                multi_id: 0,
                format_name: topic_name.to_string(),
            },
        );
        let mut mc = TopicData::new(names);
        for (i, &t) in timestamps.iter().enumerate() {
            let row: Vec<f64> = columns.iter().map(|c| c[i]).collect();
            mc.push_row(t, &row);
        }
        topics.insert(msg_id, mc);

        Px4Parsed {
            formats,
            subscriptions,
            topics,
            info: HashMap::new(),
            params: HashMap::new(),
            log_messages: Vec::new(),
            firmware_version: String::new(),
            hardware_name: String::new(),
            stats: Px4ParseStats::default(),
        }
    }

    /// bug_008: actuator_outputs schema with output[0] all zeros must NOT
    /// truncate the motor list. Pre-fix this returned motor_count=0 for
    /// pre-arm logs and VTOL/quadplane configs with inert low-index
    /// channels.
    #[test]
    fn px4_motor_count_survives_all_zero_low_channel() {
        // 4 channels: output[0] all zeros (e.g. unused tilt servo),
        // output[1..3] populated.
        let parsed = synthetic_parsed(
            "actuator_outputs",
            vec!["output[0]", "output[1]", "output[2]", "output[3]"],
            vec![1000, 2000, 3000],
            vec![
                vec![0.0, 0.0, 0.0], // inert
                vec![0.5, 0.6, 0.7],
                vec![0.5, 0.6, 0.7],
                vec![0.5, 0.6, 0.7],
            ],
        );
        let s = super::session(parsed, vec![], 1);
        assert_eq!(
            s.meta.motor_count, 4,
            "all-zero output[0] must not truncate the motor list (got {})",
            s.meta.motor_count
        );
        assert_eq!(s.motors.commands.len(), 4);
        // Channel 0 still holds zeros — present, not silently dropped.
        assert!(s.motors.commands[0].iter().all(|n| n.0 == 0.0));
    }

    /// bug_006: vehicle_attitude.q quaternion must convert to attitude
    /// roll/pitch/yaw in degrees via the standard ZYX Euler formula.
    /// Test values: identity quaternion (w=1) → all zeros; 90° yaw
    /// quaternion (w=cos(45°), z=sin(45°)) → yaw ≈ 90°.
    #[test]
    fn px4_quaternion_to_euler_degrees() {
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        let parsed = synthetic_parsed(
            "vehicle_attitude",
            vec!["q[0]", "q[1]", "q[2]", "q[3]"],
            vec![1000, 2000],
            vec![
                vec![1.0, s2], // w
                vec![0.0, 0.0],
                vec![0.0, 0.0],
                vec![0.0, s2], // z
            ],
        );
        let s = super::session(parsed, vec![], 1);
        assert_eq!(s.attitude.values.yaw.len(), 2);
        // Identity quaternion → 0° yaw.
        assert!(
            s.attitude.values.yaw[0].abs() < 0.01,
            "identity quaternion should yield 0° yaw, got {}",
            s.attitude.values.yaw[0]
        );
        // 90° yaw quaternion.
        assert!(
            (s.attitude.values.yaw[1] - 90.0).abs() < 0.1,
            "90° yaw quaternion should yield ≈90° yaw, got {}",
            s.attitude.values.yaw[1]
        );
    }
}
