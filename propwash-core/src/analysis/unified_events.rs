//! Format-agnostic event detectors.
//!
//! **Per-stream time axes**: each detector takes its own stream's
//! `time_us` for indexing — earlier versions paired (e.g.) throttle
//! samples with gyro timestamps, which silently produced wrong event
//! times on AP/PX4/MAVLink where streams sample at different rates.
//! [`detect_overshoot`] is the only detector that genuinely needs to
//! cross-correlate two streams (gyro vs setpoint); it resamples
//! setpoint onto the gyro time axis via [`super::util::resample`].

use az::Az;

use crate::types::{Axis, Session};
use crate::units::DegPerSec;

use super::events::{EventKind, FlightEvent};
use super::util;

/// Run all format-agnostic event detectors over the typed Session.
pub fn detect_all(unified: &Session) -> Vec<FlightEvent> {
    if unified.gyro.time_us.len() < 2 {
        return Vec::new();
    }

    let mut events = Vec::new();
    detect_gyro_spikes(unified, &mut events);
    detect_throttle_events(unified, &mut events);
    detect_motor_saturation(unified, &mut events);
    detect_overshoot(unified, &mut events);
    detect_desync(unified, &mut events);

    events.sort_by(|a, b| a.time_seconds.total_cmp(&b.time_seconds));
    events
}

/// `(time_us as f64, first_t)` from a u64 axis. Returns `(_, 0.0)` for
/// an empty axis; callers gate on `len() < 2` separately.
fn axis_f64(time_us: &[u64]) -> (Vec<f64>, f64) {
    let v: Vec<f64> = time_us.iter().map(|&t| t.az::<f64>()).collect();
    let first = v.first().copied().unwrap_or(0.0);
    (v, first)
}

fn detect_gyro_spikes(unified: &Session, events: &mut Vec<FlightEvent>) {
    let spike_threshold = 500.0; // deg/s
    let (timestamps, first_t) = axis_f64(&unified.gyro.time_us);
    if timestamps.is_empty() {
        return;
    }

    for axis in Axis::ALL {
        let gyro: &[f64] = bytemuck::cast_slice(unified.gyro.values.get(axis).as_slice());
        for (j, &val) in gyro.iter().enumerate() {
            if val.abs() > spike_threshold {
                let t = timestamps.get(j).copied().unwrap_or(0.0);
                events.push(FlightEvent {
                    frame_index: j,
                    time_us: t.az::<i64>(),
                    time_seconds: (t - first_t) / 1_000_000.0,
                    kind: EventKind::GyroSpike {
                        axis,
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

fn detect_throttle_events(unified: &Session, events: &mut Vec<FlightEvent>) {
    let throttle: Vec<f64> = unified
        .rc_command
        .throttle
        .iter()
        .map(|n| f64::from(n.0))
        .collect();
    if throttle.len() < 2 {
        return;
    }
    // Use rc_command's own time axis — pairing throttle samples with
    // gyro timestamps was the AP/PX4/MAVLink-silent-bug.
    let (timestamps, first_t) = axis_f64(&unified.rc_command.time_us);
    if timestamps.len() < 2 {
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
                time_us: curr_t.az::<i64>(),
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
                time_us: curr_t.az::<i64>(),
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

fn detect_motor_saturation(unified: &Session, events: &mut Vec<FlightEvent>) {
    let (_, motor_max) = unified.motor_range();
    let threshold = motor_max - (motor_max / 50.0);
    let (timestamps, first_t) = axis_f64(&unified.motors.time_us);
    if timestamps.is_empty() {
        return;
    }

    for mi in 0..unified.motor_count() {
        let Some(motor_col) = unified.motors.commands.get(mi) else {
            continue;
        };
        let mut saturated_start: Option<usize> = None;

        for (i, n) in motor_col.iter().enumerate() {
            let val = f64::from(n.0);
            let is_saturated = val >= threshold;
            match (is_saturated, saturated_start) {
                (true, None) => saturated_start = Some(i),
                (false, Some(start)) => {
                    let duration = i - start;
                    if duration >= 5 {
                        let t = timestamps.get(start).copied().unwrap_or(0.0);
                        events.push(FlightEvent {
                            frame_index: start,
                            time_us: t.az::<i64>(),
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

fn detect_overshoot(unified: &Session, events: &mut Vec<FlightEvent>) {
    let overshoot_threshold = 15.0;
    let (timestamps, first_t) = axis_f64(&unified.gyro.time_us);
    if timestamps.is_empty() || unified.setpoint.is_empty() {
        return;
    }

    for axis in Axis::ALL {
        let gyro: &[f64] = bytemuck::cast_slice(unified.gyro.values.get(axis).as_slice());
        if gyro.is_empty() {
            continue;
        }
        // Resample setpoint onto the gyro time axis. Setpoint and gyro
        // share an axis on BF (parser populates both with main-frame
        // time) but not on AP/PX4/MAVLink, where setpoint comes from a
        // controller topic at a different rate than the IMU.
        let setpoint_axis_us: super::util::TimeSeriesView<DegPerSec> =
            super::util::TimeSeriesView {
                time_us: &unified.setpoint.time_us,
                values: unified.setpoint.values.get(axis),
            };
        let setpoint = util::resample_view(
            setpoint_axis_us,
            &unified.gyro.time_us,
            util::Strategy::Linear,
        );

        let len = gyro.len().min(setpoint.len()).min(timestamps.len());
        for j in 0..len {
            let sp = setpoint[j].0;
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
                let t = timestamps[j];
                events.push(FlightEvent {
                    frame_index: j,
                    time_us: t.az::<i64>(),
                    time_seconds: (t - first_t) / 1_000_000.0,
                    kind: EventKind::Overshoot {
                        axis,
                        setpoint: sp,
                        actual,
                        overshoot_percent: overshoot_pct,
                    },
                });
            }
        }
    }
}

fn detect_desync(unified: &Session, events: &mut Vec<FlightEvent>) {
    let n_motors = unified.motor_count();
    if n_motors < 2 {
        return;
    }
    let (timestamps, first_t) = axis_f64(&unified.motors.time_us);
    if timestamps.is_empty() {
        return;
    }

    let (_, motor_max) = unified.motor_range();
    let spike_threshold = motor_max * 0.95;

    let motors: Vec<Vec<f64>> = (0..n_motors)
        .map(|i| {
            unified
                .motors
                .commands
                .get(i)
                .map(|col| col.iter().map(|n| f64::from(n.0)).collect())
                .unwrap_or_default()
        })
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
            if val < spike_threshold {
                continue;
            }
            let others: Vec<f64> = vals
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != mi)
                .map(|(_, &v)| v)
                .collect();
            let others_avg = others.iter().sum::<f64>() / others.len().az::<f64>();
            let others_max = others.iter().copied().fold(0.0_f64, f64::max);

            if others_max < motor_max * 0.60 && val > others_avg * 3.0 && others_avg > 0.0 {
                let t = timestamps[j];
                events.push(FlightEvent {
                    frame_index: j,
                    time_us: t.az::<i64>(),
                    time_seconds: (t - first_t) / 1_000_000.0,
                    kind: EventKind::Desync {
                        motor_index: mi,
                        motor_value: val.az::<i64>(),
                        average_others: others_avg,
                    },
                });
            }
        }
    }
}

/// Check if a single motor value constitutes a desync given the motor values and max.
/// Extracted for testability.
#[cfg_attr(not(test), allow(dead_code))]
fn is_desync(motor_vals: &[f64], motor_idx: usize, motor_max: f64) -> bool {
    let val = motor_vals[motor_idx];
    let spike_threshold = motor_max * 0.95;
    if val < spike_threshold {
        return false;
    }
    let others: Vec<f64> = motor_vals
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != motor_idx)
        .map(|(_, &v)| v)
        .collect();
    let others_avg = others.iter().sum::<f64>() / others.len().az::<f64>();
    let others_max = others.iter().copied().fold(0.0_f64, f64::max);

    others_max < motor_max * 0.60 && val > others_avg * 3.0 && others_avg > 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    const MOTOR_MAX: f64 = 2047.0;

    #[test]
    fn desync_real_desync() {
        let vals = [2000.0, 400.0, 500.0, 450.0];
        assert!(
            is_desync(&vals, 0, MOTOR_MAX),
            "motor at max with others low should be desync"
        );
    }

    #[test]
    fn desync_normal_cornering() {
        let vals = [1950.0, 1800.0, 800.0, 900.0];
        assert!(
            !is_desync(&vals, 0, MOTOR_MAX),
            "two motors high = normal cornering"
        );
    }

    #[test]
    fn desync_throttle_punch() {
        let vals = [2000.0, 1500.0, 1600.0, 1400.0];
        assert!(
            !is_desync(&vals, 0, MOTOR_MAX),
            "all motors high = throttle punch"
        );
    }

    #[test]
    fn desync_motor_below_spike_threshold() {
        let vals = [1842.0, 400.0, 500.0, 450.0];
        assert!(
            !is_desync(&vals, 0, MOTOR_MAX),
            "motor below 95% should not trigger"
        );
    }

    #[test]
    fn desync_boundary_others_at_60_percent() {
        let vals = [2000.0, MOTOR_MAX * 0.60, 400.0, 400.0];
        assert!(
            !is_desync(&vals, 0, MOTOR_MAX),
            "other motor at 60% should block desync"
        );
    }

    #[test]
    fn desync_ratio_too_low() {
        let others_val = MOTOR_MAX * 0.59;
        let vals = [2000.0, others_val, others_val, others_val];
        assert!(
            !is_desync(&vals, 0, MOTOR_MAX),
            "ratio too low even though others < 60%"
        );
    }

    #[test]
    fn desync_all_conditions_met() {
        let vals = [2000.0, 500.0, 500.0, 500.0];
        assert!(is_desync(&vals, 0, MOTOR_MAX));
    }

    #[test]
    fn desync_idle_not_triggered() {
        let vals = [100.0, 50.0, 60.0, 55.0];
        assert!(!is_desync(&vals, 0, MOTOR_MAX), "idle should not trigger");
    }

    #[test]
    fn desync_two_motors() {
        let vals = [2000.0, 400.0];
        assert!(is_desync(&vals, 0, MOTOR_MAX));
    }
}
