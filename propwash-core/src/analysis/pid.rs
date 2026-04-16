use crate::types::{Axis, SensorField, Session};
use serde::Serialize;

/// Step response metrics for a single axis.
#[derive(Debug, Serialize)]
pub struct AxisStepResponse {
    pub axis: &'static str,
    /// Number of step events detected.
    pub step_count: usize,
    /// Median overshoot percentage across detected steps.
    pub median_overshoot_pct: f64,
    /// Median rise time in milliseconds (10% → 90% of setpoint).
    pub median_rise_time_ms: f64,
    /// Median settling time in milliseconds (within 5% of setpoint).
    pub median_settling_time_ms: f64,
    /// Rating: "tight", "good", "sluggish", or "oscillating".
    pub rating: &'static str,
    /// Concrete tuning suggestion, if any.
    pub suggestion: String,
}

/// PID analysis for the whole flight.
#[derive(Debug, Serialize)]
pub struct PidAnalysis {
    pub axes: Vec<AxisStepResponse>,
}

/// Minimum setpoint change (deg/s) to qualify as a "step".
const STEP_THRESHOLD: f64 = 100.0;

/// Minimum absolute setpoint at peak to analyse (avoids noise on idle).
const MIN_SETPOINT_ABS: f64 = 50.0;

/// A single detected step event with measured response characteristics.
struct StepMeasurement {
    overshoot_pct: f64,
    rise_time_ms: f64,
    settling_time_ms: f64,
}

/// Analyze PID step response characteristics for all axes.
pub fn analyze_pid(session: &Session) -> PidAnalysis {
    let sample_rate = session.sample_rate_hz();
    if sample_rate <= 0.0 {
        return PidAnalysis { axes: Vec::new() };
    }

    let ms_per_frame = 1000.0 / sample_rate;

    let axes = Axis::ALL
        .iter()
        .map(|&axis| {
            let setpoint = session.field(&SensorField::Setpoint(axis));
            let gyro = session.field(&SensorField::Gyro(axis));

            let measurements = detect_steps(&setpoint, &gyro, ms_per_frame);
            build_axis_result(axis, &measurements)
        })
        .collect();

    PidAnalysis { axes }
}

/// Detect setpoint steps and measure gyro response for each.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn detect_steps(setpoint: &[f64], gyro: &[f64], ms_per_frame: f64) -> Vec<StepMeasurement> {
    let n = setpoint.len().min(gyro.len());
    if n < 10 {
        return Vec::new();
    }

    let mut measurements = Vec::new();
    let max_response_frames = (200.0 / ms_per_frame) as usize; // look up to 200ms ahead

    let mut i = 1;
    while i < n.saturating_sub(max_response_frames) {
        let delta = setpoint[i] - setpoint[i - 1];

        // Detect a step: large setpoint change, significant absolute setpoint
        if delta.abs() >= STEP_THRESHOLD && setpoint[i].abs() >= MIN_SETPOINT_ABS {
            let target = setpoint[i];

            if let Some(m) = measure_response(gyro, setpoint, i, target, max_response_frames, ms_per_frame) {
                measurements.push(m);
            }

            // Skip past this step's response window to avoid double-counting
            i += max_response_frames;
        } else {
            i += 1;
        }
    }

    measurements
}

/// Measure response characteristics for a single step starting at frame `start`.
#[allow(clippy::cast_precision_loss)]
fn measure_response(
    gyro: &[f64],
    setpoint: &[f64],
    start: usize,
    target: f64,
    window: usize,
    ms_per_frame: f64,
) -> Option<StepMeasurement> {
    let end = (start + window).min(gyro.len()).min(setpoint.len());
    if start >= end {
        return None;
    }

    let baseline = gyro[start.saturating_sub(1)];
    let travel = target - baseline;
    if travel.abs() < 10.0 {
        return None;
    }

    let settle_band = travel.abs() * 0.05;

    let mut rise_start_frame = None;
    let mut rise_end_frame = None;
    let mut peak_overshoot: f64 = 0.0;
    let mut settled_frame = None;
    let sign = travel.signum();

    for j in start..end {
        let val = gyro[j];
        let progress = (val - baseline) * sign;
        let target_progress = travel.abs();

        // Rise time: first frame past 10%
        if rise_start_frame.is_none() && progress >= target_progress * 0.1 {
            rise_start_frame = Some(j);
        }

        // Rise time: first frame past 90%
        if rise_end_frame.is_none() && progress >= target_progress * 0.9 {
            rise_end_frame = Some(j);
        }

        // Track overshoot
        let overshoot = progress - target_progress;
        if overshoot > peak_overshoot {
            peak_overshoot = overshoot;
        }

        // Settling: first frame within band after the rise
        if rise_end_frame.is_some() && settled_frame.is_none() {
            let sp = setpoint[j];
            if (val - sp).abs() <= settle_band {
                settled_frame = Some(j);
            }
        }
    }

    let rise_time_ms = match (rise_start_frame, rise_end_frame) {
        (Some(s), Some(e)) => (e - s) as f64 * ms_per_frame,
        _ => return None, // Couldn't measure rise time — skip this step
    };

    let overshoot_pct = if travel.abs() > 0.0 {
        (peak_overshoot / travel.abs()) * 100.0
    } else {
        0.0
    };

    let settling_time_ms = settled_frame.map_or(
        window as f64 * ms_per_frame,
        |f| (f - start) as f64 * ms_per_frame,
    );

    Some(StepMeasurement {
        overshoot_pct,
        rise_time_ms,
        settling_time_ms,
    })
}

fn build_axis_result(axis: Axis, measurements: &[StepMeasurement]) -> AxisStepResponse {
    let axis_name = match axis {
        Axis::Roll => "roll",
        Axis::Pitch => "pitch",
        Axis::Yaw => "yaw",
    };

    if measurements.is_empty() {
        return AxisStepResponse {
            axis: axis_name,
            step_count: 0,
            median_overshoot_pct: 0.0,
            median_rise_time_ms: 0.0,
            median_settling_time_ms: 0.0,
            rating: "no data",
            suggestion: "Not enough stick input to analyze step response.".into(),
        };
    }

    let overshoot = median(measurements.iter().map(|m| m.overshoot_pct));
    let rise_time = median(measurements.iter().map(|m| m.rise_time_ms));
    let settling_time = median(measurements.iter().map(|m| m.settling_time_ms));

    let (rating, suggestion) = rate_axis(axis_name, overshoot, rise_time, settling_time);

    AxisStepResponse {
        axis: axis_name,
        step_count: measurements.len(),
        median_overshoot_pct: (overshoot * 10.0).round() / 10.0,
        median_rise_time_ms: (rise_time * 10.0).round() / 10.0,
        median_settling_time_ms: (settling_time * 10.0).round() / 10.0,
        rating,
        suggestion,
    }
}

fn rate_axis(
    axis: &str,
    overshoot_pct: f64,
    rise_time_ms: f64,
    settling_time_ms: f64,
) -> (&'static str, String) {
    // Oscillating: high overshoot AND slow settling
    if overshoot_pct > 30.0 && settling_time_ms > 80.0 {
        return (
            "oscillating",
            format!("Reduce P gain on {axis}. {overshoot_pct:.0}% overshoot with {settling_time_ms:.0}ms settling suggests P is too high relative to D."),
        );
    }

    // High overshoot but fast settling — P slightly high
    if overshoot_pct > 20.0 {
        return (
            "good",
            format!("Consider reducing P gain slightly on {axis} or increasing D — {overshoot_pct:.0}% overshoot is a bit high."),
        );
    }

    // Sluggish: low overshoot but slow rise
    if rise_time_ms > 30.0 || settling_time_ms > 100.0 {
        return (
            "sluggish",
            format!("Increase P gain on {axis} — {rise_time_ms:.0}ms rise time is slow."),
        );
    }

    // Tight: fast rise, low overshoot, fast settling
    if overshoot_pct < 10.0 && rise_time_ms < 20.0 && settling_time_ms < 60.0 {
        return (
            "tight",
            format!("{axis} response looks great — fast and clean."),
        );
    }

    // Good: reasonable balance
    (
        "good",
        format!("{axis} response is solid — {overshoot_pct:.0}% overshoot, {rise_time_ms:.0}ms rise."),
    )
}

fn median(iter: impl Iterator<Item = f64>) -> f64 {
    let mut values: Vec<f64> = iter.collect();
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = values.len() / 2;
    if values.len().is_multiple_of(2) {
        f64::midpoint(values[mid - 1], values[mid])
    } else {
        values[mid]
    }
}
