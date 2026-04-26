//! Translate a [`Px4Parsed`] into a typed [`Session`].
//!
//! All PX4-specific unit conversions live here:
//!  - `vehicle_angular_velocity.xyz[i]` rad/s → `DegPerSec`
//!  - `sensor_combined.gyro_rad[i]` rad/s → `DegPerSec`
//!  - `sensor_accel.{x,y,z}` m/s² → `MetersPerSec2`
//!  - `vehicle_rates_setpoint.{roll,pitch,yaw}` rad/s → `DegPerSec`
//!  - `actuator_outputs.output[i]` already 0–1 (or 1000–2000 PWM in some configs)
//!  - `input_rc.values[i]` PWM 1000–2000 → sticks/throttle Normalized01
//!  - `battery_status.voltage_v / current_a` → typed
//!  - `vehicle_gps_position.{lat,lon}` int×10⁷ → DecimalDegrees

use az::{Az, SaturatingAs};

use super::parser::Px4Parsed;
use super::types::TopicData;
use crate::session::{
    Event, EventKind, FlightMode, Format, Gps, LogSeverity, Session, SessionMeta, TimeSeries,
};
use crate::types::Warning;
use crate::units::{
    Amps, DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2, Normalized01,
    Volts,
};

const RAD_TO_DEG: f64 = 57.295_779_513_082_32;

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
            if let (Some(x), Some(y), Some(z)) =
                (t.column("x"), t.column("y"), t.column("z"))
            {
                s.accel.time_us = t.timestamps.clone();
                s.accel.values.roll = x.iter().copied().map(MetersPerSec2).collect();
                s.accel.values.pitch = y.iter().copied().map(MetersPerSec2).collect();
                s.accel.values.yaw = z.iter().copied().map(MetersPerSec2).collect();
            }
        }
    }

    // ── Setpoint ──────────────────────────────────────────────────────────
    if let Some(t) = topic("vehicle_rates_setpoint") {
        if let (Some(r), Some(p), Some(y)) =
            (t.column("roll"), t.column("pitch"), t.column("yaw"))
        {
            s.setpoint.time_us = t.timestamps.clone();
            s.setpoint.values.roll = r.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
            s.setpoint.values.pitch = p.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
            s.setpoint.values.yaw = y.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        }
    }

    // ── Motor outputs ─────────────────────────────────────────────────────
    if let Some(t) = topic("actuator_outputs") {
        s.motors.time_us = t.timestamps.clone();
        let mut motor_count = 0;
        for i in 0..16 {
            let name = format!("output[{i}]");
            if let Some(col) = t.column(&name) {
                let normalized: Vec<Normalized01> = col
                    .iter()
                    .map(|&v| Normalized01(normalize_output(v)))
                    .collect();
                if normalized.iter().all(|n| n.0 == 0.0) {
                    // Empty channel — probably not wired.
                    break;
                }
                s.motors.commands.push(normalized);
                motor_count += 1;
            } else {
                break;
            }
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
            gps.alt = col.iter().map(|&v| Meters((v / 1000.0).az::<f32>())).collect();
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

/// PX4 `vehicle_status.nav_state` (NAVIGATION_STATE_*) → common FlightMode.
fn px4_flight_mode(state: u8) -> FlightMode {
    match state {
        0 => FlightMode::Manual,
        1 => FlightMode::AltHold,
        2 => FlightMode::PosHold,
        3 => FlightMode::Auto,            // MISSION
        4 => FlightMode::Loiter,
        5 => FlightMode::ReturnToHome,
        6 => FlightMode::Other("AUTO_RCRECOVER".into()),
        7 => FlightMode::Failsafe,        // RTGS
        8 => FlightMode::Failsafe,        // LANDENGFAIL
        9 => FlightMode::Land,            // AUTO_LAND
        10 => FlightMode::Other("AUTO_TAKEOFF".into()),
        11 => FlightMode::Other("AUTO_FOLLOW_TARGET".into()),
        12 => FlightMode::Other("AUTO_PRECLAND".into()),
        14 => FlightMode::Stabilize,
        15 => FlightMode::Acro,
        17 => FlightMode::Manual,         // RATTITUDE
        other => FlightMode::Other(format!("NAV_{other}")),
    }
}
