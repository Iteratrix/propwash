use serde::Serialize;

use crate::types::{Axis, SensorField, Session};

/// Step response quality metrics for one axis.
#[derive(Debug, Clone, Serialize)]
pub struct AxisStepResponse {
    pub axis: &'static str,
    /// Number of step events detected and analyzed.
    pub step_count: usize,
    /// Average time from 10% to 90% of target (milliseconds).
    pub rise_time_ms: f64,
    /// Average peak overshoot as percentage of step size.
    pub overshoot_percent: f64,
    /// Average time to settle within 5% of target (milliseconds).
    pub settling_time_ms: f64,
}

/// Complete step response analysis across all axes.
#[derive(Debug, Clone, Serialize)]
pub struct StepResponseAnalysis {
    pub axes: Vec<AxisStepResponse>,
}

/// Minimum setpoint change (deg/s) to qualify as a "step."
const MIN_STEP_SIZE: f64 = 50.0;
/// Lookback window for step detection (samples).
/// At 1kHz with RC smoothing, a stick input ramps over ~50ms.
const STEP_LOOKBACK: usize = 50;
/// Window before the step to establish the baseline (samples).
const PRE_SAMPLES: usize = 5;
/// Window after the step to measure the response (samples).
const POST_SAMPLES: usize = 200;
/// Minimum gap between detected steps (samples) to avoid duplicates.
const STEP_COOLDOWN: usize = 100;

/// Analyze step response quality from setpoint and gyro data.
#[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
pub fn analyze_step_response(session: &Session) -> Option<StepResponseAnalysis> {
    let sample_rate = session.sample_rate_hz();
    if sample_rate <= 0.0 {
        return None;
    }
    let ms_per_sample = 1000.0 / sample_rate;

    let axis_names = ["roll", "pitch", "yaw"];
    let mut axes = Vec::new();

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let setpoint = session.field(&SensorField::Setpoint(*axis));
        let gyro = session.field(&SensorField::Gyro(*axis));

        if setpoint.len() < POST_SAMPLES || gyro.len() < POST_SAMPLES {
            continue;
        }
        let len = setpoint.len().min(gyro.len());

        // Detect step events: significant setpoint changes
        let steps = detect_steps(&setpoint[..len]);
        if steps.is_empty() {
            continue;
        }

        let mut rise_times = Vec::new();
        let mut overshoots = Vec::new();
        let mut settling_times = Vec::new();

        for &step_idx in &steps {
            if step_idx + POST_SAMPLES > len {
                continue;
            }

            // Baseline: setpoint before the lookback window (stable before ramp)
            let pre_start = step_idx.saturating_sub(STEP_LOOKBACK + PRE_SAMPLES);
            let pre_end = step_idx.saturating_sub(STEP_LOOKBACK);
            let sp_before = if pre_end > pre_start {
                mean(&setpoint[pre_start..pre_end])
            } else {
                setpoint[step_idx.saturating_sub(1)]
            };
            // Target: setpoint well after the ramp settles (50-100 samples after detection)
            let settle_start = (step_idx + STEP_LOOKBACK).min(step_idx + POST_SAMPLES - 10);
            let settle_end = (settle_start + 20).min(step_idx + POST_SAMPLES);
            let sp_after = mean(&setpoint[settle_start..settle_end]);
            let step_size = sp_after - sp_before;

            if step_size.abs() < MIN_STEP_SIZE {
                continue;
            }

            let gyro_window = &gyro[step_idx..step_idx + POST_SAMPLES];

            // Skip windows with corrupt gyro data (spikes > 2000 deg/s)
            let max_abs = gyro_window.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
            if max_abs > 2000.0 {
                continue;
            }

            // Rise time: samples from 10% to 90% of step
            let target_10 = sp_before + step_size * 0.1;
            let target_90 = sp_before + step_size * 0.9;
            let rising = step_size > 0.0;

            let crossed_10 = gyro_window.iter().position(|&v| {
                if rising {
                    v >= target_10
                } else {
                    v <= target_10
                }
            });
            let crossed_90 = gyro_window.iter().position(|&v| {
                if rising {
                    v >= target_90
                } else {
                    v <= target_90
                }
            });

            if let (Some(t10), Some(t90)) = (crossed_10, crossed_90) {
                if t90 > t10 {
                    rise_times.push((t90 - t10) as f64 * ms_per_sample);
                }
            }

            // Overshoot: how far past the target the gyro goes
            let (peak, _) = if rising {
                gyro_window
                    .iter()
                    .enumerate()
                    .fold(
                        (f64::MIN, 0),
                        |(max, mi), (i, &v)| {
                            if v > max {
                                (v, i)
                            } else {
                                (max, mi)
                            }
                        },
                    )
            } else {
                gyro_window
                    .iter()
                    .enumerate()
                    .fold(
                        (f64::MAX, 0),
                        |(min, mi), (i, &v)| {
                            if v < min {
                                (v, i)
                            } else {
                                (min, mi)
                            }
                        },
                    )
            };

            let overshoot = if step_size.abs() > 0.0 {
                ((peak - sp_after) / step_size * 100.0).max(0.0)
            } else {
                0.0
            };
            // Skip unreliable measurements (corrupt data or bad baseline)
            if overshoot > 200.0 {
                continue;
            }
            overshoots.push(overshoot);

            // Settling time: first sample after which gyro stays within 5% of target
            let tolerance = step_size.abs() * 0.05;
            let settled_at = gyro_window.iter().enumerate().position(|(i, _)| {
                // Check if all remaining samples in a short window stay within tolerance
                let check_end = (i + 20).min(gyro_window.len());
                gyro_window[i..check_end]
                    .iter()
                    .all(|&v| (v - sp_after).abs() < tolerance)
            });
            if let Some(t) = settled_at {
                settling_times.push(t as f64 * ms_per_sample);
            }
        }

        if rise_times.is_empty() && overshoots.is_empty() {
            continue;
        }

        axes.push(AxisStepResponse {
            axis: axis_names[i],
            step_count: overshoots.len(),
            rise_time_ms: if rise_times.is_empty() {
                0.0
            } else {
                mean(&rise_times)
            },
            overshoot_percent: if overshoots.is_empty() {
                0.0
            } else {
                mean(&overshoots)
            },
            settling_time_ms: if settling_times.is_empty() {
                0.0
            } else {
                mean(&settling_times)
            },
        });
    }

    if axes.is_empty() {
        None
    } else {
        Some(StepResponseAnalysis { axes })
    }
}

/// Detect indices where the setpoint makes a significant step change.
///
/// Uses a lookback window to catch RC-smoothed ramps at high sample rates
/// (e.g., 1kHz with BF rate limiting produces ~1 deg/s per sample deltas).
fn detect_steps(setpoint: &[f64]) -> Vec<usize> {
    let mut steps = Vec::new();
    // Allow detection from the start by setting last_step far in the past
    let mut last_step = 0usize;
    let start = STEP_LOOKBACK;

    for i in start..setpoint.len() {
        if i > last_step && i - last_step < STEP_COOLDOWN && last_step >= start {
            continue;
        }
        // Compare current value vs value LOOKBACK samples ago
        let delta = (setpoint[i] - setpoint[i - STEP_LOOKBACK]).abs();
        if delta >= MIN_STEP_SIZE {
            steps.push(i);
            last_step = i;
        }
    }
    steps
}

#[allow(clippy::cast_precision_loss)]
fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f64>() / values.len() as f64
}
