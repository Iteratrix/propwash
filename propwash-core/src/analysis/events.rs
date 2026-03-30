use serde::Serialize;

use crate::format::bf::types::{BfFrame, BfRawSession};

#[derive(Debug, Clone, Serialize)]
pub struct FlightEvent {
    pub frame_index: usize,
    pub time_us: i64,
    pub time_seconds: f64,
    pub kind: EventKind,
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type")]
pub enum EventKind {
    ThrottleChop {
        from_percent: f64,
        to_percent: f64,
        duration_ms: f64,
    },
    ThrottlePunch {
        from_percent: f64,
        to_percent: f64,
        duration_ms: f64,
    },
    MotorSaturation {
        motor_index: usize,
        duration_frames: usize,
    },
    GyroSpike {
        axis: &'static str,
        magnitude: f64,
    },
    Overshoot {
        axis: &'static str,
        setpoint: f64,
        actual: f64,
        overshoot_percent: f64,
    },
}

struct FieldIndices {
    time: Option<usize>,
    throttle: Option<usize>,
    motors: Vec<usize>,
    gyro: [Option<usize>; 3],
    setpoint: [Option<usize>; 3],
}

impl FieldIndices {
    fn from_session(session: &BfRawSession) -> Self {
        let defs = &session.main_field_defs;
        let mut motors = Vec::new();
        for i in 0..8 {
            let name = format!("motor[{i}]");
            match defs.index_of(&name) {
                Some(idx) => motors.push(idx),
                None => break,
            }
        }
        Self {
            time: defs.index_of("time"),
            throttle: defs.index_of("rcCommand[3]"),
            motors,
            gyro: [
                defs.index_of("gyroADC[0]"),
                defs.index_of("gyroADC[1]"),
                defs.index_of("gyroADC[2]"),
            ],
            setpoint: [
                defs.index_of("setpoint[0]"),
                defs.index_of("setpoint[1]"),
                defs.index_of("setpoint[2]"),
            ],
        }
    }
}

fn frame_val(frame: &BfFrame, idx: Option<usize>) -> i64 {
    idx.and_then(|i| frame.values.get(i).copied()).unwrap_or(0)
}

fn frame_time_seconds(frame: &BfFrame, first_time: i64, time_idx: Option<usize>) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let t = (frame_val(frame, time_idx) - first_time) as f64 / 1_000_000.0;
    t
}

/// Detects events in a Betaflight session.
pub fn detect_events(session: &BfRawSession) -> Vec<FlightEvent> {
    let idx = FieldIndices::from_session(session);
    let frames = &session.frames;

    if frames.len() < 2 {
        return Vec::new();
    }

    let first_time = frame_val(&frames[0], idx.time);
    let motor_max = detect_motor_max(session);

    let mut events = Vec::new();

    detect_throttle_events(frames, &idx, first_time, &mut events);
    detect_motor_saturation(frames, &idx, first_time, motor_max, &mut events);
    detect_gyro_spikes(frames, &idx, first_time, &mut events);
    detect_overshoot(frames, &idx, first_time, &mut events);

    events.sort_by_key(|e| e.frame_index);
    events
}

fn detect_motor_max(session: &BfRawSession) -> i64 {
    let motor_output = session.get_header_int_list("motorOutput");
    match motor_output.len() {
        0 => 2047,
        1 => i64::from(motor_output[0]),
        _ => i64::from(motor_output[1]),
    }
}

#[allow(clippy::cast_precision_loss)]
fn detect_throttle_events(
    frames: &[BfFrame],
    idx: &FieldIndices,
    first_time: i64,
    events: &mut Vec<FlightEvent>,
) {
    let Some(throttle_idx) = idx.throttle else {
        return;
    };

    let throttle_range = 1000.0;

    for window in frames.windows(2) {
        let prev_throttle = frame_val(&window[0], Some(throttle_idx));
        let curr_throttle = frame_val(&window[1], Some(throttle_idx));
        let delta = curr_throttle - prev_throttle;

        let prev_time = frame_val(&window[0], idx.time);
        let curr_time = frame_val(&window[1], idx.time);
        let dt_ms = (curr_time - prev_time) as f64 / 1000.0;

        if dt_ms <= 0.0 {
            continue;
        }

        #[allow(clippy::cast_precision_loss)]
        let rate = delta as f64 / dt_ms;

        if rate < -200.0 {
            events.push(FlightEvent {
                frame_index: window[1].frame_index,
                time_us: frame_val(&window[1], idx.time),
                time_seconds: frame_time_seconds(&window[1], first_time, idx.time),
                kind: EventKind::ThrottleChop {
                    from_percent: (prev_throttle - 1000) as f64 / throttle_range * 100.0,
                    to_percent: (curr_throttle - 1000) as f64 / throttle_range * 100.0,
                    duration_ms: dt_ms,
                },
            });
        } else if rate > 200.0 {
            events.push(FlightEvent {
                frame_index: window[1].frame_index,
                time_us: frame_val(&window[1], idx.time),
                time_seconds: frame_time_seconds(&window[1], first_time, idx.time),
                kind: EventKind::ThrottlePunch {
                    from_percent: (prev_throttle - 1000) as f64 / throttle_range * 100.0,
                    to_percent: (curr_throttle - 1000) as f64 / throttle_range * 100.0,
                    duration_ms: dt_ms,
                },
            });
        }
    }
}

fn detect_motor_saturation(
    frames: &[BfFrame],
    idx: &FieldIndices,
    first_time: i64,
    motor_max: i64,
    events: &mut Vec<FlightEvent>,
) {
    let threshold = motor_max - (motor_max / 50);

    for (motor_index, &motor_idx) in idx.motors.iter().enumerate() {
        let mut saturated_start: Option<usize> = None;

        for (i, frame) in frames.iter().enumerate() {
            let val = frame_val(frame, Some(motor_idx));
            let is_saturated = val >= threshold;

            match (is_saturated, saturated_start) {
                (true, None) => saturated_start = Some(i),
                (false, Some(start)) => {
                    let duration = i - start;
                    if duration >= 5 {
                        events.push(FlightEvent {
                            frame_index: start,
                            time_us: frame_val(&frames[start], idx.time),
                            time_seconds: frame_time_seconds(&frames[start], first_time, idx.time),
                            kind: EventKind::MotorSaturation {
                                motor_index,
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

fn detect_gyro_spikes(
    frames: &[BfFrame],
    idx: &FieldIndices,
    first_time: i64,
    events: &mut Vec<FlightEvent>,
) {
    let axis_names = ["roll", "pitch", "yaw"];
    let spike_threshold = 500.0;

    for (axis, &gyro_idx) in idx.gyro.iter().enumerate() {
        let Some(gyro_idx) = gyro_idx else {
            continue;
        };

        for frame in frames {
            #[allow(clippy::cast_precision_loss)]
            let val = frame_val(frame, Some(gyro_idx)) as f64;
            if val.abs() > spike_threshold {
                events.push(FlightEvent {
                    frame_index: frame.frame_index,
                    time_us: frame_val(frame, idx.time),
                    time_seconds: frame_time_seconds(frame, first_time, idx.time),
                    kind: EventKind::GyroSpike {
                        axis: axis_names[axis],
                        magnitude: val,
                    },
                });
            }
        }
    }
}

fn detect_overshoot(
    frames: &[BfFrame],
    idx: &FieldIndices,
    first_time: i64,
    events: &mut Vec<FlightEvent>,
) {
    let axis_names = ["roll", "pitch", "yaw"];
    let overshoot_threshold = 15.0;

    for (axis, (&gyro_idx, &sp_idx)) in idx.gyro.iter().zip(idx.setpoint.iter()).enumerate() {
        let Some(gyro_idx) = gyro_idx else {
            continue;
        };
        let Some(sp_idx) = sp_idx else {
            continue;
        };

        for frame in frames {
            #[allow(clippy::cast_precision_loss)]
            let setpoint = frame_val(frame, Some(sp_idx)) as f64;
            #[allow(clippy::cast_precision_loss)]
            let actual = frame_val(frame, Some(gyro_idx)) as f64;

            if setpoint.abs() < 50.0 {
                continue;
            }

            let overshoot_pct = if setpoint.abs() > 0.0 {
                ((actual - setpoint) / setpoint * 100.0).abs()
            } else {
                0.0
            };

            if overshoot_pct > overshoot_threshold
                && actual.signum() == setpoint.signum()
                && actual.abs() > setpoint.abs()
            {
                events.push(FlightEvent {
                    frame_index: frame.frame_index,
                    time_us: frame_val(frame, idx.time),
                    time_seconds: frame_time_seconds(frame, first_time, idx.time),
                    kind: EventKind::Overshoot {
                        axis: axis_names[axis],
                        setpoint,
                        actual,
                        overshoot_percent: overshoot_pct,
                    },
                });
            }
        }
    }
}
