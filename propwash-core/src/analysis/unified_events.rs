use crate::types::{Axis, MotorIndex, RcChannel, SensorField, Session};

use super::events::{EventKind, FlightEvent};

/// Run all format-agnostic event detectors using the `Unified` trait.
#[allow(clippy::cast_precision_loss)]
pub fn detect_all(unified: &Session) -> Vec<FlightEvent> {
    let timestamps = unified.field(&SensorField::Time);
    if timestamps.len() < 2 {
        return Vec::new();
    }
    let first_t = timestamps[0];

    let mut events = Vec::new();

    detect_gyro_spikes(unified, &timestamps, first_t, &mut events);
    detect_throttle_events(unified, &timestamps, first_t, &mut events);
    detect_motor_saturation(unified, &timestamps, first_t, &mut events);
    detect_overshoot(unified, &timestamps, first_t, &mut events);
    detect_desync(unified, &timestamps, first_t, &mut events);

    events.sort_by(|a, b| a.time_seconds.total_cmp(&b.time_seconds));
    events
}

fn detect_gyro_spikes(
    unified: &Session,
    timestamps: &[f64],
    first_t: f64,
    events: &mut Vec<FlightEvent>,
) {
    let spike_threshold = 500.0; // deg/s
    let axis_names = ["roll", "pitch", "yaw"];

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let gyro = unified.field(&SensorField::Gyro(*axis));
        for (j, &val) in gyro.iter().enumerate() {
            if val.abs() > spike_threshold {
                let t = timestamps.get(j).copied().unwrap_or(0.0);
                events.push(FlightEvent {
                    frame_index: j,
                    #[allow(clippy::cast_possible_truncation)]
                    time_us: t as i64,
                    time_seconds: (t - first_t) / 1_000_000.0,
                    kind: EventKind::GyroSpike {
                        axis: axis_names[i],
                        magnitude: val,
                    },
                });
            }
        }
    }
}

/// Lookback window for throttle event detection (microseconds).
/// Compares throttle now vs 100ms ago to detect sustained chops/punches.
const THROTTLE_LOOKBACK_US: f64 = 100_000.0;
/// Minimum throttle change (percentage of range) to qualify as a chop/punch.
const THROTTLE_MIN_CHANGE_PCT: f64 = 30.0;
/// Minimum cooldown between events (microseconds) to avoid duplicates.
const THROTTLE_COOLDOWN_US: f64 = 500_000.0;

#[allow(clippy::cast_precision_loss)]
fn detect_throttle_events(
    unified: &Session,
    timestamps: &[f64],
    first_t: f64,
    events: &mut Vec<FlightEvent>,
) {
    let throttle = unified.field(&SensorField::Rc(RcChannel::Throttle));
    if throttle.len() < 2 {
        return;
    }

    let t_min = throttle.iter().copied().fold(f64::MAX, f64::min);
    let t_max = throttle.iter().copied().fold(f64::MIN, f64::max);
    let t_range = (t_max - t_min).max(1.0);
    let to_pct = |v: f64| (v - t_min) / t_range * 100.0;

    let len = throttle.len().min(timestamps.len());
    let mut lookback_idx = 0;
    let mut last_chop_t = f64::MIN;
    let mut last_punch_t = f64::MIN;

    for i in 1..len {
        let curr_t = timestamps[i];

        // Advance lookback pointer to maintain ~100ms window
        while lookback_idx < i && timestamps[lookback_idx] < curr_t - THROTTLE_LOOKBACK_US {
            lookback_idx += 1;
        }
        if lookback_idx >= i {
            continue;
        }

        let from_pct = to_pct(throttle[lookback_idx]);
        let now_pct = to_pct(throttle[i]);
        let change = now_pct - from_pct;
        let dt_ms = (curr_t - timestamps[lookback_idx]) / 1000.0;

        if change < -THROTTLE_MIN_CHANGE_PCT && curr_t - last_chop_t > THROTTLE_COOLDOWN_US {
            last_chop_t = curr_t;
            events.push(FlightEvent {
                frame_index: i,
                #[allow(clippy::cast_possible_truncation)]
                time_us: curr_t as i64,
                time_seconds: (curr_t - first_t) / 1_000_000.0,
                kind: EventKind::ThrottleChop {
                    from_percent: from_pct,
                    to_percent: now_pct,
                    duration_ms: dt_ms,
                },
            });
        } else if change > THROTTLE_MIN_CHANGE_PCT && curr_t - last_punch_t > THROTTLE_COOLDOWN_US {
            last_punch_t = curr_t;
            events.push(FlightEvent {
                frame_index: i,
                #[allow(clippy::cast_possible_truncation)]
                time_us: curr_t as i64,
                time_seconds: (curr_t - first_t) / 1_000_000.0,
                kind: EventKind::ThrottlePunch {
                    from_percent: from_pct,
                    to_percent: now_pct,
                    duration_ms: dt_ms,
                },
            });
        }
    }
}

fn detect_motor_saturation(
    unified: &Session,
    timestamps: &[f64],
    first_t: f64,
    events: &mut Vec<FlightEvent>,
) {
    let (_, motor_max) = unified.motor_range();
    let threshold = motor_max - (motor_max / 50.0);

    for mi in 0..unified.motor_count() {
        let motor = unified.field(&SensorField::Motor(MotorIndex(mi)));
        let mut saturated_start: Option<usize> = None;

        for (i, &val) in motor.iter().enumerate() {
            let is_saturated = val >= threshold;
            match (is_saturated, saturated_start) {
                (true, None) => saturated_start = Some(i),
                (false, Some(start)) => {
                    let duration = i - start;
                    if duration >= 5 {
                        let t = timestamps.get(start).copied().unwrap_or(0.0);
                        events.push(FlightEvent {
                            frame_index: start,
                            #[allow(clippy::cast_possible_truncation)]
                            time_us: t as i64,
                            time_seconds: (t - first_t) / 1_000_000.0,
                            kind: EventKind::MotorSaturation {
                                motor_index: mi,
                                duration_frames: duration,
                            },
                        });
                    }
                    saturated_start = None;
                }
                _ => {}
            }
        }
    }
}

fn detect_overshoot(
    unified: &Session,
    timestamps: &[f64],
    first_t: f64,
    events: &mut Vec<FlightEvent>,
) {
    let axis_names = ["roll", "pitch", "yaw"];
    let overshoot_threshold = 15.0;

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let gyro = unified.field(&SensorField::Gyro(*axis));
        let setpoint = unified.field(&SensorField::Setpoint(*axis));
        if gyro.is_empty() || setpoint.is_empty() {
            continue;
        }

        let len = gyro.len().min(setpoint.len()).min(timestamps.len());
        for j in 0..len {
            let sp = setpoint[j];
            let actual = gyro[j];

            if sp.abs() < 50.0 {
                continue;
            }

            let overshoot_pct = if sp.abs() > 0.0 {
                ((actual - sp) / sp * 100.0).abs()
            } else {
                0.0
            };

            if overshoot_pct > overshoot_threshold
                && actual.signum() == sp.signum()
                && actual.abs() > sp.abs()
            {
                let t = timestamps.get(j).copied().unwrap_or(0.0);
                events.push(FlightEvent {
                    frame_index: j,
                    #[allow(clippy::cast_possible_truncation)]
                    time_us: t as i64,
                    time_seconds: (t - first_t) / 1_000_000.0,
                    kind: EventKind::Overshoot {
                        axis: axis_names[i],
                        setpoint: sp,
                        actual,
                        overshoot_percent: overshoot_pct,
                    },
                });
            }
        }
    }
}

#[allow(clippy::cast_precision_loss)]
fn detect_desync(
    unified: &Session,
    timestamps: &[f64],
    first_t: f64,
    events: &mut Vec<FlightEvent>,
) {
    let n_motors = unified.motor_count();
    if n_motors < 2 {
        return;
    }

    let (_, motor_max) = unified.motor_range();
    let threshold = motor_max - (motor_max / 20.0);

    let motors: Vec<Vec<f64>> = (0..n_motors)
        .map(|i| unified.field(&SensorField::Motor(MotorIndex(i))))
        .collect();

    let len = motors
        .iter()
        .map(Vec::len)
        .min()
        .unwrap_or(0)
        .min(timestamps.len());

    for j in 0..len {
        let vals: Vec<f64> = motors.iter().map(|m| m[j]).collect();

        for (mi, &val) in vals.iter().enumerate() {
            if val < threshold {
                continue;
            }
            let others_sum: f64 = vals
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != mi)
                .map(|(_, &v)| v)
                .sum();
            let others_avg = others_sum / (n_motors - 1) as f64;

            if val > others_avg * 1.5 && others_avg > 0.0 {
                let t = timestamps.get(j).copied().unwrap_or(0.0);
                events.push(FlightEvent {
                    frame_index: j,
                    #[allow(clippy::cast_possible_truncation)]
                    time_us: t as i64,
                    time_seconds: (t - first_t) / 1_000_000.0,
                    kind: EventKind::Desync {
                        motor_index: mi,
                        #[allow(clippy::cast_possible_truncation)]
                        motor_value: val as i64,
                        average_others: others_avg,
                    },
                });
            }
        }
    }
}
