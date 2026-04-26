//! Translate a stream of [`BfFrame`]s into a typed [`Session`].
#![allow(
    clippy::assigning_clones,             // empty-on-init Session fields
    clippy::needless_pass_by_value,       // signature symmetry
    clippy::match_same_arms,              // explicit Time arm documents intent vs `_` catch-all
    clippy::unnecessary_wraps,            // map_bf_event returns Option for future variants
    clippy::doc_lazy_continuation,        // multi-line doc continuations
)]
//!
//! This is the only place BF-specific knowledge meets Session-shaped
//! data: unit conversions (vbat × 0.01 → Volts; GPS × 1e-7 → `DecimalDegrees`;
//! eRPM × 100 → Erpm), motor command normalisation against the header-
//! reported PWM range, FlightMode-event translation to typed
//! `FlightMode` + armed transitions.

use std::collections::HashMap;

use az::{Az, SaturatingAs, WrappingAs};

use super::frames::{BfFrame, BfFrames};
use super::types::{BfEvent, BfFrameDefs, BfHeaderValue};
use crate::session::{Event, EventKind, FlightMode, Format, Gps, Session, SessionMeta, TimeSeries};
use crate::types::{
    AxisGains, FilterConfig, MotorIndex, PidGains, RcChannel, SensorField, Warning,
};
use crate::units::{
    DecimalDegrees, DegPerSec, Erpm, Meters, MetersPerSec, MetersPerSec2, Normalized01, Volts,
};

/// Build a `Session` by streaming frames from the iterator.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(crate) fn session(
    mut frames: BfFrames<'_>,
    headers: &HashMap<String, BfHeaderValue>,
    main_defs: &BfFrameDefs,
    slow_defs: Option<&BfFrameDefs>,
    gps_defs: Option<&BfFrameDefs>,
    firmware_version: String,
    craft_name: String,
    session_index: usize,
    parse_warnings: Vec<Warning>,
) -> Session {
    let mut s = Session::default();

    let motor_count = main_defs
        .fields
        .iter()
        .filter(|f| matches!(f.name, SensorField::Motor(_)))
        .count();
    let erpm_count = main_defs
        .fields
        .iter()
        .filter(|f| matches!(f.name, SensorField::ERpm(_)))
        .count();

    s.motors.commands = vec![Vec::new(); motor_count];
    if erpm_count > 0 {
        s.motors.esc = Some(crate::session::Esc {
            erpm: vec![Vec::new(); erpm_count],
            ..crate::session::Esc::default()
        });
    }

    let main_time_idx = main_defs.index_of(&SensorField::Time);
    let acc_1g = i64::from(BfHeaderValue::int(headers, "acc_1G", 4096).max(1));
    let motor_min = pwm_min(headers);
    let motor_max = pwm_max(headers);
    let pwm_span = (motor_max - motor_min).max(1).az::<f32>();

    let mut main_time_us: Vec<u64> = Vec::new();
    let mut last_main_time: u64 = 0;
    let mut prev_flight_mode_flags: u32 = 0;
    let mut armed_state = false;

    for frame in frames.by_ref() {
        match frame {
            BfFrame::Main { values, .. } => {
                let t = main_time_idx
                    .and_then(|i| values.get(i).copied())
                    .unwrap_or(0)
                    .wrapping_as::<u64>();
                main_time_us.push(t);
                last_main_time = t;

                for (def, &raw) in main_defs.fields.iter().zip(values.iter()) {
                    match &def.name {
                        SensorField::Time => {}
                        SensorField::Gyro(axis) => s
                            .gyro
                            .values
                            .get_mut(*axis)
                            .push(DegPerSec(raw.az::<f64>())),
                        SensorField::Setpoint(axis) => s
                            .setpoint
                            .values
                            .get_mut(*axis)
                            .push(DegPerSec(raw.az::<f64>())),
                        SensorField::Accel(axis) => s
                            .accel
                            .values
                            .get_mut(*axis)
                            // BF stores acc as raw sensor counts; divide by acc_1G to get g, ×9.80665 for m/s²
                            .push(MetersPerSec2(
                                raw.az::<f64>() / acc_1g.az::<f64>() * 9.80665,
                            )),
                        SensorField::Motor(MotorIndex(i)) => {
                            if let Some(col) = s.motors.commands.get_mut(*i) {
                                col.push(normalize_motor(raw.az::<i32>(), motor_min, pwm_span));
                            }
                        }
                        SensorField::ERpm(MotorIndex(i)) => {
                            if let Some(esc) = s.motors.esc.as_mut() {
                                if let Some(col) = esc.erpm.get_mut(*i) {
                                    // BF stores eRPM/100; multiply back
                                    col.push(Erpm(raw.max(0).az::<u32>().saturating_mul(100)));
                                }
                            }
                        }
                        SensorField::Rc(ch) => match ch {
                            RcChannel::Roll => {
                                s.rc_command.sticks.roll.push(normalize_stick(raw));
                            }
                            RcChannel::Pitch => {
                                s.rc_command.sticks.pitch.push(normalize_stick(raw));
                            }
                            RcChannel::Yaw => {
                                s.rc_command.sticks.yaw.push(normalize_stick(raw));
                            }
                            RcChannel::Throttle => s
                                .rc_command
                                .throttle
                                .push(normalize_throttle(raw, motor_min, pwm_span)),
                        },
                        SensorField::Unknown(name) => {
                            s.extras
                                .entry(name.clone())
                                .or_default()
                                .push(t, raw.az::<f64>());
                        }
                        // Not currently routed: PidP/PidI/PidD, Feedforward, GyroUnfilt, Vbat (slow), Rssi (slow), GPS fields (gps frame)
                        _ => {}
                    }
                }
            }

            BfFrame::Slow { values } => {
                let Some(slow_defs) = slow_defs else { continue };
                for (def, &raw) in slow_defs.fields.iter().zip(values.iter()) {
                    match &def.name {
                        SensorField::Vbat => {
                            // BF stores vbat in centivolts (× 0.01 → V)
                            s.vbat
                                .push(last_main_time, Volts((raw.az::<f64>() * 0.01).az::<f32>()));
                        }
                        SensorField::Rssi => {
                            // 0-1023 → 0-100%
                            s.rssi.push(
                                last_main_time,
                                (raw.az::<f64>() / 1023.0 * 100.0).az::<f32>(),
                            );
                        }
                        SensorField::Unknown(name) if name == "flightModeFlags" => {
                            let flags = raw.wrapping_as::<u32>();
                            let (mode, armed) = parse_flight_mode_flags(flags);
                            if armed != armed_state {
                                s.armed.push(last_main_time, armed);
                                s.events.push(Event {
                                    time_us: last_main_time,
                                    kind: if armed {
                                        EventKind::Armed
                                    } else {
                                        EventKind::Disarmed
                                    },
                                    message: None,
                                });
                                armed_state = armed;
                            }
                            if flags != prev_flight_mode_flags {
                                s.flight_mode.push(last_main_time, mode.clone());
                                s.events.push(Event {
                                    time_us: last_main_time,
                                    kind: EventKind::ModeChange { to: mode },
                                    message: None,
                                });
                                prev_flight_mode_flags = flags;
                            }
                        }
                        SensorField::Unknown(name) => {
                            s.extras
                                .entry(name.clone())
                                .or_default()
                                .push(last_main_time, raw.az::<f64>());
                        }
                        _ => {}
                    }
                }
            }

            BfFrame::Gps { values } => {
                let Some(gps_defs) = gps_defs else { continue };
                let gps = s.gps.get_or_insert_with(Gps::default);

                let mut frame_time = last_main_time;
                if let Some(idx) = gps_defs.index_of(&SensorField::Time) {
                    if let Some(&v) = values.get(idx) {
                        frame_time = v.wrapping_as::<u64>();
                    }
                }
                gps.time_us.push(frame_time);

                // Push only fields whose schema is present in this BF
                // variant's GPS frame. Synthetic 0.0 padding would
                // violate the Session contract ("Empty Vec means absent
                // — not zero") and produce Null Island coordinates if
                // GpsLat/Lng were missing from the schema. Columns may
                // therefore have length < gps.time_us.len() (or 0)
                // when the field isn't reported by this firmware.
                for (def, &raw) in gps_defs.fields.iter().zip(values.iter()) {
                    match &def.name {
                        SensorField::Time => {}
                        SensorField::GpsLat => {
                            gps.lat.push(DecimalDegrees(raw.az::<f64>() * 1e-7));
                        }
                        SensorField::GpsLng => {
                            gps.lng.push(DecimalDegrees(raw.az::<f64>() * 1e-7));
                        }
                        SensorField::Altitude => {
                            // BF altitude in cm
                            gps.alt.push(Meters(raw.az::<f32>() / 100.0));
                        }
                        SensorField::GpsSpeed => {
                            // BF speed in cm/s
                            gps.speed.push(MetersPerSec(raw.az::<f32>() / 100.0));
                        }
                        SensorField::Heading => {
                            // BF heading in 0.1 deg
                            gps.heading.push(raw.az::<f32>() / 10.0);
                        }
                        SensorField::Unknown(name) if name == "GPS_numSat" => {
                            gps.sats.push(raw.saturating_as::<u8>());
                        }
                        _ => {}
                    }
                }
            }

            BfFrame::Event(ev) => {
                if let Some(event) = map_bf_event(ev, last_main_time) {
                    s.events.push(event);
                }
            }
        }
    }

    frames.finalize();
    let stats = std::mem::take(&mut frames.stats);
    let mut warnings = parse_warnings;
    warnings.append(&mut frames.warnings);

    // Attach time axes to all main-frame streams.
    s.gyro.time_us = main_time_us.clone();
    s.accel.time_us = main_time_us.clone();
    s.setpoint.time_us = main_time_us.clone();
    s.pid_err.time_us = main_time_us.clone();
    s.motors.time_us = main_time_us.clone();
    s.rc_command.time_us = main_time_us;

    // Populate metadata.
    s.meta = SessionMeta {
        format: Format::Betaflight,
        firmware: firmware_version,
        craft_name: if craft_name.is_empty() {
            None
        } else {
            Some(craft_name)
        },
        board: None,
        motor_count,
        pid_gains: Some(parse_pid_gains(headers)),
        filter_config: Some(parse_filter_config(headers)),
        session_index,
        truncated: !stats.clean_end,
        corrupt_bytes: stats.corrupt_bytes,
        warnings,
    };

    // Save remaining stats as extras under a synthetic key. (Optional —
    // keeps i/p/slow/gps frame counts visible to power users.)
    s.extras.insert(
        "_bf_i_frame_count".into(),
        TimeSeries::from_parts(vec![0], vec![stats.i_frame_count.az::<f64>()]),
    );
    s.extras.insert(
        "_bf_p_frame_count".into(),
        TimeSeries::from_parts(vec![0], vec![stats.p_frame_count.az::<f64>()]),
    );

    s
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn pwm_min(headers: &HashMap<String, BfHeaderValue>) -> i32 {
    let mo = BfHeaderValue::int_list(headers, "motorOutput");
    mo.first().copied().unwrap_or(1000)
}

fn pwm_max(headers: &HashMap<String, BfHeaderValue>) -> i32 {
    let mo = BfHeaderValue::int_list(headers, "motorOutput");
    mo.get(1).copied().unwrap_or(2000)
}

fn normalize_motor(raw: i32, motor_min: i32, pwm_span: f32) -> Normalized01 {
    let v = (raw - motor_min).az::<f32>() / pwm_span;
    Normalized01(v.clamp(0.0, 1.0))
}

fn normalize_throttle(raw: i64, motor_min: i32, pwm_span: f32) -> Normalized01 {
    let v = (raw.az::<i32>() - motor_min).az::<f32>() / pwm_span;
    Normalized01(v.clamp(0.0, 1.0))
}

fn normalize_stick(raw: i64) -> f32 {
    // Sticks centred around 1500 with range ±500 PWM-microseconds.
    ((raw.az::<f32>() - 1500.0) / 500.0).clamp(-1.0, 1.0)
}

fn parse_pid_gains(headers: &HashMap<String, BfHeaderValue>) -> PidGains {
    let parse = |key: &str| -> AxisGains {
        let vals = BfHeaderValue::int_list(headers, key);
        let to_u32 = |v: i32| -> Option<u32> {
            if v > 0 {
                Some(v.cast_unsigned())
            } else {
                None
            }
        };
        AxisGains {
            p: vals.first().copied().and_then(to_u32),
            i: vals.get(1).copied().and_then(to_u32),
            d: vals.get(2).copied().and_then(to_u32),
        }
    };
    PidGains::new(parse("rollPID"), parse("pitchPID"), parse("yawPID"))
}

fn parse_filter_config(headers: &HashMap<String, BfHeaderValue>) -> FilterConfig {
    let non_zero = |v: i32| -> Option<f64> {
        if v > 0 {
            Some(f64::from(v))
        } else {
            None
        }
    };
    FilterConfig {
        gyro_lpf_hz: non_zero(BfHeaderValue::int(headers, "gyro_lowpass_hz", 0)),
        gyro_lpf2_hz: non_zero(BfHeaderValue::int(headers, "gyro_lowpass2_hz", 0)),
        dterm_lpf_hz: non_zero(BfHeaderValue::int(headers, "dterm_lpf_hz", 0))
            .or_else(|| non_zero(BfHeaderValue::int(headers, "dterm_lowpass_hz", 0))),
        dyn_notch_min_hz: non_zero(BfHeaderValue::int(headers, "dyn_notch_min_hz", 0)),
        dyn_notch_max_hz: non_zero(BfHeaderValue::int(headers, "dyn_notch_max_hz", 0)),
        gyro_notch1_hz: non_zero(BfHeaderValue::int(headers, "gyro_notch_hz", 0)),
        gyro_notch2_hz: non_zero(BfHeaderValue::int(headers, "gyro_notch2_hz", 0)),
    }
}

fn map_bf_event(ev: BfEvent, fallback_time: u64) -> Option<Event> {
    match ev {
        BfEvent::Disarm { reason } => Some(Event {
            time_us: fallback_time,
            kind: EventKind::Disarmed,
            message: Some(format!("disarm reason={reason}")),
        }),
        BfEvent::FlightMode { flags, .. } => {
            let (mode, _) = parse_flight_mode_flags(flags);
            Some(Event {
                time_us: fallback_time,
                kind: EventKind::ModeChange { to: mode },
                message: None,
            })
        }
        BfEvent::SyncBeep { time_us } => Some(Event {
            time_us,
            kind: EventKind::Custom("SyncBeep".into()),
            message: None,
        }),
        BfEvent::InflightAdjustment { function, value } => Some(Event {
            time_us: fallback_time,
            kind: EventKind::Custom("InflightAdjustment".into()),
            message: Some(format!("function={function} value={value}")),
        }),
        BfEvent::LoggingResume {
            log_iteration,
            current_time,
        } => Some(Event {
            time_us: u64::from(current_time),
            kind: EventKind::Custom("LoggingResume".into()),
            message: Some(format!("iteration={log_iteration}")),
        }),
        BfEvent::LogEnd => Some(Event {
            time_us: fallback_time,
            kind: EventKind::Custom("LogEnd".into()),
            message: None,
        }),
        BfEvent::Unknown { type_id } => Some(Event {
            time_us: fallback_time,
            kind: EventKind::Custom(format!("Unknown({type_id})")),
            message: None,
        }),
    }
}

/// Decode Betaflight's `flightModeFlags` bitfield into a coarse `FlightMode`
/// + `armed` state. BF flag IDs vary between firmware versions; this is a
/// best-effort mapping covering common BF 4.x flags.
fn parse_flight_mode_flags(flags: u32) -> (FlightMode, bool) {
    const ARM: u32 = 1 << 0;
    const ANGLE: u32 = 1 << 1;
    const HORIZON: u32 = 1 << 2;
    const FAILSAFE: u32 = 1 << 5;

    let armed = (flags & ARM) != 0;
    let mode = if (flags & FAILSAFE) != 0 {
        FlightMode::Failsafe
    } else if (flags & ANGLE) != 0 {
        FlightMode::Stabilize
    } else if (flags & HORIZON) != 0 {
        FlightMode::Horizon
    } else if armed {
        FlightMode::Acro
    } else {
        FlightMode::None
    };
    (mode, armed)
}
