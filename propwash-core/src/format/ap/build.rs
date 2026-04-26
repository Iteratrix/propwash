//! Translate an `ApParsed` (per-message-type columnar intermediate) into
//! a typed [`Session`].
//!
//! All AP-specific unit conversions live here:
//!  - `IMU.GyrX/Y/Z` rad/s → `DegPerSec`
//!  - `IMU.AccX/Y/Z` m/s² (already in SI)
//!  - `RCOU.Cn` motor PWM 1000-2000 → `Normalized01`
//!  - `RCIN.Cn` stick PWM 1000-2000 → `−1..1` (sticks) / `0..1` (throttle)
//!  - `BAT.Volt` already in V
//!  - `GPS.Lat/Lng` already in decimal degrees (AP stores ÷10⁷ pre-scaled by `L` field-type)
//!
//! Wait — that's not right. AP `L` is i32 raw degrees×10⁷; `decode_f64`
//! returns the i32 as f64 unchanged. So we DO scale ×1e-7 here.

use std::collections::HashMap;

use az::{Az, SaturatingAs};

use super::parser::ApParsed;
use super::types::MsgColumns;
use crate::format::common::{ardupilot_filter_config, ardupilot_pid_gains};
use crate::session::{
    Event, EventKind, FlightMode, Format, Gps, Session, SessionMeta, TimeSeries,
};
use crate::types::Warning;
use crate::units::{
    Amps, DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2, Normalized01,
    Volts,
};

const RAD_TO_DEG: f64 = 57.295_779_513_082_32;

pub(crate) fn session(parsed: ApParsed, warnings: Vec<Warning>, session_index: usize) -> Session {
    let mut s = Session::default();

    let topic_by_name = |name: &str| -> Option<&MsgColumns> {
        parsed
            .msg_defs
            .values()
            .find(|d| d.name == name)
            .and_then(|d| parsed.topics.get(&d.msg_type))
    };

    // ── Gyro: prefer IMU (filtered) then GYR (unfiltered fallback) ─────────
    if let Some(t) = topic_by_name("IMU") {
        s.gyro.time_us = t.timestamps.clone();
        let r = t.column("GyrX").unwrap_or(&[]);
        let p = t.column("GyrY").unwrap_or(&[]);
        let y = t.column("GyrZ").unwrap_or(&[]);
        s.gyro.values.roll = r.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        s.gyro.values.pitch = p.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        s.gyro.values.yaw = y.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
    } else if let Some(t) = topic_by_name("GYR") {
        s.gyro.time_us = t.timestamps.clone();
        let r = t.column("GyrX").unwrap_or(&[]);
        let p = t.column("GyrY").unwrap_or(&[]);
        let y = t.column("GyrZ").unwrap_or(&[]);
        s.gyro.values.roll = r.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        s.gyro.values.pitch = p.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
        s.gyro.values.yaw = y.iter().map(|v| DegPerSec(v * RAD_TO_DEG)).collect();
    }

    // ── Accel: prefer IMU then ACC ─────────────────────────────────────────
    if let Some(t) = topic_by_name("IMU") {
        s.accel.time_us = t.timestamps.clone();
        let x = t.column("AccX").unwrap_or(&[]);
        let y = t.column("AccY").unwrap_or(&[]);
        let z = t.column("AccZ").unwrap_or(&[]);
        s.accel.values.roll = x.iter().copied().map(MetersPerSec2).collect();
        s.accel.values.pitch = y.iter().copied().map(MetersPerSec2).collect();
        s.accel.values.yaw = z.iter().copied().map(MetersPerSec2).collect();
    } else if let Some(t) = topic_by_name("ACC") {
        s.accel.time_us = t.timestamps.clone();
        let x = t.column("AccX").unwrap_or(&[]);
        let y = t.column("AccY").unwrap_or(&[]);
        let z = t.column("AccZ").unwrap_or(&[]);
        s.accel.values.roll = x.iter().copied().map(MetersPerSec2).collect();
        s.accel.values.pitch = y.iter().copied().map(MetersPerSec2).collect();
        s.accel.values.yaw = z.iter().copied().map(MetersPerSec2).collect();
    }

    // ── Setpoint: RATE message (RDes / PDes / YDes are deg/s already) ─────
    if let Some(t) = topic_by_name("RATE") {
        s.setpoint.time_us = t.timestamps.clone();
        for (axis_field, store) in [
            ("RDes", &mut s.setpoint.values.roll),
            ("PDes", &mut s.setpoint.values.pitch),
            ("YDes", &mut s.setpoint.values.yaw),
        ] {
            *store = t
                .column(axis_field)
                .unwrap_or(&[])
                .iter()
                .copied()
                .map(DegPerSec)
                .collect();
        }
    }

    // ── Motor outputs: RCOU.C1..C8 PWM 1000-2000 → Normalized01 ───────────
    let (motor_min, motor_max) = ap_motor_range(&parsed.params);
    let pwm_span = (motor_max - motor_min).max(1.0);
    if let Some(t) = topic_by_name("RCOU") {
        s.motors.time_us = t.timestamps.clone();
        let mut motor_count = 0;
        for i in 1..=8 {
            let name = format!("C{i}");
            if let Some(col) = t.column(&name) {
                let normalized: Vec<Normalized01> = col
                    .iter()
                    .map(|&v| Normalized01((((v - motor_min) / pwm_span).az::<f32>()).clamp(0.0, 1.0)))
                    .collect();
                s.motors.commands.push(normalized);
                motor_count += 1;
            } else {
                break;
            }
        }
        s.meta.motor_count = motor_count;
    }

    // ── RC inputs: RCIN.C1..C4 PWM 1000-2000 → sticks (-1..1), throttle (0..1) ──
    if let Some(t) = topic_by_name("RCIN") {
        s.rc_command.time_us = t.timestamps.clone();
        if let Some(col) = t.column("C1") {
            s.rc_command.sticks.roll = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("C2") {
            s.rc_command.sticks.pitch = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("C4") {
            s.rc_command.sticks.yaw = col.iter().map(|&v| stick_norm(v)).collect();
        }
        if let Some(col) = t.column("C3") {
            s.rc_command.throttle = col
                .iter()
                .map(|&v| Normalized01((((v - motor_min) / pwm_span).az::<f32>()).clamp(0.0, 1.0)))
                .collect();
        }
        if let Some(col) = t.column("RSSI") {
            s.rssi = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| v.az::<f32>()).collect(),
            );
        }
    }

    // ── Battery ──────────────────────────────────────────────────────────────
    if let Some(t) = topic_by_name("BAT") {
        if let Some(col) = t.column("Volt") {
            s.vbat = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| Volts(v.az::<f32>())).collect(),
            );
        }
        if let Some(col) = t.column("Curr") {
            s.current = TimeSeries::from_parts(
                t.timestamps.clone(),
                col.iter().map(|&v| Amps(v.az::<f32>())).collect(),
            );
        }
    }

    // ── GPS ─────────────────────────────────────────────────────────────────
    if let Some(t) = topic_by_name("GPS") {
        let mut gps = Gps {
            time_us: t.timestamps.clone(),
            ..Gps::default()
        };
        if let Some(col) = t.column("Lat") {
            gps.lat = col.iter().map(|&v| DecimalDegrees(v * 1e-7)).collect();
        }
        if let Some(col) = t.column("Lng") {
            gps.lng = col.iter().map(|&v| DecimalDegrees(v * 1e-7)).collect();
        }
        if let Some(col) = t.column("Alt") {
            gps.alt = col.iter().map(|&v| Meters(v.az::<f32>())).collect();
        }
        if let Some(col) = t.column("Spd") {
            gps.speed = col.iter().map(|&v| MetersPerSec(v.az::<f32>())).collect();
        }
        if let Some(col) = t.column("GCrs") {
            gps.heading = col.iter().map(|&v| v.az::<f32>()).collect();
        }
        if let Some(col) = t.column("NSats") {
            gps.sats = col.iter().map(|&v| v.saturating_as::<u8>()).collect();
        }
        if !gps.is_empty() {
            s.gps = Some(gps);
        }
    }

    // ── ESC telemetry: ESC messages with Instance + RPM ─────────────────────
    if let Some(t) = topic_by_name("ESC") {
        let inst_col = t.column("Instance");
        let rpm_col = t.column("RPM");
        if let (Some(inst), Some(rpm)) = (inst_col, rpm_col) {
            let max_inst = inst.iter().copied().fold(0.0_f64, f64::max).az::<usize>() + 1;
            let mut erpm: Vec<Vec<Erpm>> = vec![Vec::new(); max_inst];
            for (&i, &r) in inst.iter().zip(rpm.iter()) {
                let idx = i.saturating_as::<usize>();
                if let Some(col) = erpm.get_mut(idx) {
                    col.push(Erpm(r.max(0.0).az::<u32>()));
                }
            }
            s.motors.esc = Some(crate::session::Esc {
                time_us: t.timestamps.clone(),
                erpm,
                ..crate::session::Esc::default()
            });
        }
    }

    // ── Mode + armed events from MODE message ──────────────────────────────
    if let Some(t) = topic_by_name("MODE") {
        if let Some(mode_col) = t.column("Mode") {
            for (&time, &mode_id) in t.timestamps.iter().zip(mode_col.iter()) {
                let mode = ap_flight_mode(mode_id.az::<u8>());
                s.flight_mode.push(time, mode.clone());
                s.events.push(Event {
                    time_us: time,
                    kind: EventKind::ModeChange { to: mode },
                    message: None,
                });
            }
        }
    }

    // ── EV (event) and ERR (error) → Session.events ───────────────────────
    if let Some(t) = topic_by_name("EV") {
        if let Some(id_col) = t.column("Id") {
            for (&time, &id) in t.timestamps.iter().zip(id_col.iter()) {
                let kind = match id.az::<i64>() {
                    10 | 11 => EventKind::Custom(format!("EV.Id={id}")),
                    _ => EventKind::Custom(format!("EV.Id={id}")),
                };
                s.events.push(Event {
                    time_us: time,
                    kind,
                    message: None,
                });
            }
        }
    }
    if let Some(t) = topic_by_name("ERR") {
        let subsys_col = t.column("Subsys");
        let ecode_col = t.column("ECode");
        if let (Some(subsys), Some(codes)) = (subsys_col, ecode_col) {
            for ((&time, &s_), &c) in t.timestamps.iter().zip(subsys.iter()).zip(codes.iter()) {
                s.events.push(Event {
                    time_us: time,
                    kind: EventKind::LogMessage {
                        severity: crate::session::LogSeverity::Error,
                    },
                    message: Some(format!(
                        "Subsystem {} error code {}",
                        s_.az::<i64>(),
                        c.az::<i64>()
                    )),
                });
            }
        }
    }

    // ── STATUSTEXT-style firmware messages: MSG.Message ────────────────────
    // (MSG carries Z[64] strings — skipped: our parser doesn't decode strings as columns yet.)

    // ── Metadata ───────────────────────────────────────────────────────────
    s.meta = SessionMeta {
        format: Format::ArduPilot,
        firmware: parsed.firmware_version.clone(),
        craft_name: if parsed.vehicle_name.is_empty() {
            None
        } else {
            Some(parsed.vehicle_name.clone())
        },
        board: None,
        motor_count: s.meta.motor_count,
        pid_gains: Some(ardupilot_pid_gains(&parsed.params)),
        filter_config: Some(ardupilot_filter_config(&parsed.params)),
        session_index,
        truncated: parsed.stats.truncated,
        corrupt_bytes: parsed.stats.corrupt_bytes,
        warnings,
    };

    s
}

fn ap_motor_range(params: &HashMap<String, f64>) -> (f64, f64) {
    let min = params.get("MOT_PWM_MIN").copied().unwrap_or(1000.0);
    let max = params.get("MOT_PWM_MAX").copied().unwrap_or(2000.0);
    (min, max)
}

fn stick_norm(pwm: f64) -> f32 {
    (((pwm - 1500.0) / 500.0).az::<f32>()).clamp(-1.0, 1.0)
}

/// Best-effort mapping of ArduCopter mode IDs to our common FlightMode.
/// Plane and Rover use different IDs; falls back to FlightMode::Other for unknowns.
fn ap_flight_mode(id: u8) -> FlightMode {
    match id {
        0 => FlightMode::Stabilize,
        1 => FlightMode::Acro,
        2 => FlightMode::AltHold,
        3 => FlightMode::Auto,
        4 => FlightMode::Other("GUIDED".into()),
        5 => FlightMode::Loiter,
        6 => FlightMode::ReturnToHome,
        7 => FlightMode::Other("CIRCLE".into()),
        9 => FlightMode::Land,
        13 => FlightMode::Other("SPORT".into()),
        16 => FlightMode::PosHold,
        17 => FlightMode::Other("BRAKE".into()),
        18 => FlightMode::Other("THROW".into()),
        19 => FlightMode::Other("AVOID_ADSB".into()),
        20 => FlightMode::Other("GUIDED_NOGPS".into()),
        21 => FlightMode::Other("SMART_RTL".into()),
        22 => FlightMode::Other("FLOWHOLD".into()),
        23 => FlightMode::Failsafe,
        24 => FlightMode::Other("ZIGZAG".into()),
        other => FlightMode::Other(format!("MODE_{other}")),
    }
}
