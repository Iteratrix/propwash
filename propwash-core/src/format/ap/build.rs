//! Translate an `ApParsed` (per-message-type columnar intermediate) into
//! a typed [`Session`].
#![allow(
    clippy::assigning_clones,        // Session fields start empty; clone_from buys nothing
    clippy::needless_pass_by_value,  // signature symmetry across format build()s
)]
//!
//! All AP-specific unit conversions live here:
//!  - `IMU.GyrX/Y/Z` rad/s → `DegPerSec` (parser doesn't scale `f` floats)
//!  - `IMU.AccX/Y/Z` m/s² (already in SI)
//!  - `RCOU.Cn` motor PWM 1000-2000 → `Normalized01`
//!  - `RCIN.Cn` stick PWM 1000-2000 → `−1..1` (sticks) / `0..1` (throttle)
//!  - `BAT.Volt` already in V
//!
//! AP FMT-type scaling (centidegrees, decimal degrees, etc.) is applied
//! by the parser via [`FieldType::I16Centi`]/[`FieldType::I32DegreesE7`]
//! /etc., so values reach this build step already in physical units —
//! no per-call `× 0.01` or `× 1e-7` needed.

use std::collections::HashMap;

use az::{Az, SaturatingAs};

use super::parser::ApParsed;
use super::types::MsgColumns;
use crate::format::common::{ardupilot_filter_config, ardupilot_pid_gains};
use crate::session::{Event, EventKind, FlightMode, Format, Gps, Session, SessionMeta, TimeSeries};
use crate::types::Warning;
use crate::units::{
    Amps, DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2, Normalized01, Volts,
};

const RAD_TO_DEG: f64 = 57.295_779_513_082_32;

#[allow(clippy::too_many_lines)] // declarative topic-to-Session mapping; splitting hides the routing
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

    // ── Attitude (airframe orientation, degrees) ──────────────────────────
    // ATT.{Roll,Pitch,Yaw} use FMT type `c` (centidegrees); the parser's
    // FieldType::I16Centi scales them to degrees during decode, so build
    // receives values already in degrees and just casts to f32.
    if let Some(t) = topic_by_name("ATT") {
        let to_f32 = |col: Option<&[f64]>| -> Vec<f32> {
            col.unwrap_or(&[]).iter().map(|&v| v.az::<f32>()).collect()
        };
        s.attitude.time_us = t.timestamps.clone();
        s.attitude.values.roll = to_f32(t.column("Roll"));
        s.attitude.values.pitch = to_f32(t.column("Pitch"));
        s.attitude.values.yaw = to_f32(t.column("Yaw"));
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
                    .map(|&v| {
                        Normalized01((((v - motor_min) / pwm_span).az::<f32>()).clamp(0.0, 1.0))
                    })
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
    // GPS.Lat/Lng use FMT type `L` (int32 × 1e7); the parser's
    // FieldType::I32DegreesE7 scales to decimal degrees during decode,
    // so build receives values already in decimal degrees.
    if let Some(t) = topic_by_name("GPS") {
        let mut gps = Gps {
            time_us: t.timestamps.clone(),
            ..Gps::default()
        };
        if let Some(col) = t.column("Lat") {
            gps.lat = col.iter().copied().map(DecimalDegrees).collect();
        }
        if let Some(col) = t.column("Lng") {
            gps.lng = col.iter().copied().map(DecimalDegrees).collect();
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
        if gps.has_data() {
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
            // Dedup consecutive identical mode IDs (matches PX4/MAVLink/BF
            // builds). ArduPilot mainline only writes MODE on transitions,
            // but legacy/forked firmware and replay tools can emit
            // duplicates; without the guard those cascade into double
            // ModeChange events through the analysis layer.
            let mut last_mode_id = u8::MAX;
            for (&time, &mode_id) in t.timestamps.iter().zip(mode_col.iter()) {
                let id = mode_id.az::<u8>();
                if id == last_mode_id {
                    continue;
                }
                let mode = ap_flight_mode(id);
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

    // ── EV (event) and ERR (error) → Session.events ───────────────────────
    if let Some(t) = topic_by_name("EV") {
        if let Some(id_col) = t.column("Id") {
            for (&time, &id) in t.timestamps.iter().zip(id_col.iter()) {
                s.events.push(Event {
                    time_us: time,
                    kind: EventKind::Custom(format!("EV.Id={id}")),
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
        motor_poles: None, // AP doesn't expose motor pole count via params
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

/// Best-effort mapping of `ArduCopter` mode IDs to our common `FlightMode`.
/// Plane and Rover use different IDs; falls back to `FlightMode::Other` for unknowns.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::ap::types::{ApMsgDef, ApParseStats, FieldType};
    use std::collections::HashMap;

    /// Construct a minimal `ApParsed` with one custom message type whose
    /// columns are populated from `(timestamps, columns)`. Convenience
    /// for build-step regression tests that don't need a real .bin
    /// fixture.
    fn synthetic_parsed(
        msg_name: &str,
        field_names: Vec<&str>,
        timestamps: Vec<u64>,
        columns: Vec<Vec<f64>>,
    ) -> ApParsed {
        let mut msg_defs = HashMap::new();
        let mut topics = HashMap::new();
        let msg_type = 100u8; // arbitrary non-FMT id
        let names: Vec<String> = field_names.iter().map(|&s| s.to_string()).collect();
        msg_defs.insert(
            msg_type,
            ApMsgDef {
                msg_type,
                name: msg_name.to_string(),
                field_types: vec![FieldType::U64; field_names.len()],
                field_names: names.clone(),
                msg_len: 0,
                format_str: String::new(),
            },
        );
        let mut mc = MsgColumns::new(names);
        // push_row takes (timestamp, all_columns_for_one_row); our `columns`
        // is column-major so transpose for push.
        for (i, &t) in timestamps.iter().enumerate() {
            let row: Vec<f64> = columns.iter().map(|c| c[i]).collect();
            mc.push_row(t, &row);
        }
        topics.insert(msg_type, mc);
        ApParsed {
            msg_defs,
            topics,
            firmware_version: String::new(),
            vehicle_name: String::new(),
            params: HashMap::new(),
            stats: ApParseStats::default(),
        }
    }

    /// bug_019: consecutive duplicate MODE samples must collapse to one
    /// ModeChange event + one flight_mode sample (matches PX4/MAVLink/BF
    /// dedup pattern).
    #[test]
    fn ap_mode_dedup_collapses_consecutive_duplicates() {
        let parsed = synthetic_parsed(
            "MODE",
            vec!["Mode"],
            vec![1000, 2000, 3000, 4000],
            vec![vec![0.0, 0.0, 5.0, 5.0]], // Stabilize, Stabilize, Loiter, Loiter
        );
        let s = super::session(parsed, vec![], 1);
        let mode_events: Vec<_> = s
            .events
            .iter()
            .filter(|e| matches!(e.kind, EventKind::ModeChange { .. }))
            .collect();
        assert_eq!(
            mode_events.len(),
            2,
            "expected 2 ModeChange events (Stabilize then Loiter), got {}",
            mode_events.len()
        );
        assert_eq!(s.flight_mode.values.len(), 2);
        assert_eq!(s.flight_mode.values[0], FlightMode::Stabilize);
        assert_eq!(s.flight_mode.values[1], FlightMode::Loiter);
    }

    /// bug_006 + F7: ATT.{Roll,Pitch,Yaw} use FMT type `c` (centidegrees);
    /// after F7 the parser scales them at decode time, so the build step
    /// receives values already in degrees and must just pass them through.
    /// (Prior to F7, the build step divided by 100 itself; pre-F7 this
    /// test's synthetic input was in raw centidegrees.)
    #[test]
    fn ap_attitude_passes_through_parser_scaled_degrees() {
        let parsed = synthetic_parsed(
            "ATT",
            vec!["Roll", "Pitch", "Yaw"],
            vec![1000, 2000],
            vec![
                vec![15.0, -5.0], // already-scaled degrees
                vec![-10.0, 20.0],
                vec![180.0, 359.99],
            ],
        );
        let s = super::session(parsed, vec![], 1);
        assert_eq!(s.attitude.values.roll.len(), 2);
        assert!((s.attitude.values.roll[0] - 15.0).abs() < 0.01);
        assert!((s.attitude.values.pitch[0] - (-10.0)).abs() < 0.01);
        assert!((s.attitude.values.yaw[0] - 180.0).abs() < 0.01);
        assert!((s.attitude.values.yaw[1] - 359.99).abs() < 0.05);
    }
}
