use serde::Serialize;

use crate::types::{Axis, AxisGains, PidGains, SensorField, Session};

use super::step_response::StepResponseAnalysis;

/// Step response quality rating for a single axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum TuningRating {
    /// Fast rise, low overshoot — well tuned.
    Tight,
    /// Reasonable response — no changes needed.
    Good,
    /// Slow rise, no overshoot — P too low.
    Sluggish,
    /// Moderate overshoot — P too high or D too low.
    Overshooting,
    /// High overshoot with long settling — unstable.
    Oscillating,
}

impl TuningRating {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Tight => "tight",
            Self::Good => "good",
            Self::Sluggish => "sluggish",
            Self::Overshooting => "overshooting",
            Self::Oscillating => "oscillating",
        }
    }
}

/// Concrete tuning suggestion for one axis with current and suggested gain values.
#[derive(Debug, Clone, Serialize)]
pub struct TuningSuggestion {
    pub axis: Axis,
    pub rating: TuningRating,
    pub current: AxisGains,
    pub suggested: AxisGains,
    pub overshoot_percent: f64,
    pub rise_time_ms: f64,
    pub settling_time_ms: f64,
    pub step_count: usize,
}

/// PID-specific analysis results.
#[derive(Debug, Clone, Serialize)]
pub struct PidAnalysis {
    /// Per-axis I-term windup assessment.
    pub windup: Vec<AxisWindup>,
    /// Per-axis oscillation frequency (Hz) detected from error signal around steps.
    pub oscillation: Vec<AxisOscillation>,
    /// Per-axis tuning suggestions with specific gain recommendations.
    pub tuning: Vec<TuningSuggestion>,
}

/// I-term windup assessment for one axis.
#[derive(Debug, Clone, Serialize)]
pub struct AxisWindup {
    pub axis: Axis,
    /// Fraction of frames where |I| > |P| (0.0–1.0).
    pub i_dominant_fraction: f64,
    /// Peak |I/P| ratio observed.
    pub peak_ratio: f64,
}

/// Oscillation frequency for one axis, detected from error signal FFT around steps.
#[derive(Debug, Clone, Serialize)]
pub struct AxisOscillation {
    pub axis: Axis,
    /// Dominant oscillation frequency in Hz, if detected.
    pub frequency_hz: Option<f64>,
    /// Magnitude of the oscillation peak in dB.
    pub magnitude_db: Option<f64>,
    /// Overshoot percent from step response (for correlation).
    pub overshoot_percent: f64,
}

/// Minimum frames for windup analysis to be meaningful.
const MIN_FRAMES_WINDUP: usize = 500;

/// Minimum step count for oscillation frequency detection.
const MIN_STEPS_OSCILLATION: usize = 3;

/// FFT window size for oscillation detection around steps.
const OSC_FFT_SIZE: usize = 128;

/// Minimum setpoint change to qualify as a step.
const MIN_STEP_SIZE: f64 = 50.0;

/// Post-step window in samples for oscillation FFT.
const OSC_POST_SAMPLES: usize = 200;

/// Step cooldown in samples.
const OSC_STEP_COOLDOWN: usize = 100;

/// Lookback window for step detection.
const OSC_STEP_LOOKBACK: usize = 50;

/// Analyze PID behavior from session data.
pub fn analyze_pid(
    session: &Session,
    step_response: Option<&StepResponseAnalysis>,
    gains: &PidGains,
) -> Option<PidAnalysis> {
    let windup = analyze_windup(session);
    let oscillation = analyze_oscillation(session, step_response);
    let tuning = compute_tuning(step_response, gains);

    if windup.is_empty() && oscillation.is_empty() && tuning.is_empty() {
        return None;
    }

    Some(PidAnalysis {
        windup,
        oscillation,
        tuning,
    })
}

// ---------------------------------------------------------------------------
// Tuning suggestions
// ---------------------------------------------------------------------------

/// Classify step response and compute suggested gain adjustments.
fn compute_tuning(
    step_response: Option<&StepResponseAnalysis>,
    gains: &PidGains,
) -> Vec<TuningSuggestion> {
    let Some(sr) = step_response else {
        return Vec::new();
    };

    let axis_names = ["roll", "pitch", "yaw"];
    let mut results = Vec::new();

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let Some(axis_sr) = sr.axes.iter().find(|a| a.axis == axis_names[i]) else {
            continue;
        };
        if axis_sr.step_count < 3 {
            continue;
        }

        let overshoot = axis_sr.overshoot_percent;
        let rise = axis_sr.rise_time_ms;
        let settling = axis_sr.settling_time_ms;
        let current = *gains.get(*axis);

        let (rating, suggested) = classify_and_suggest(overshoot, rise, settling, &current);

        results.push(TuningSuggestion {
            axis: *axis,
            rating,
            current,
            suggested,
            overshoot_percent: overshoot,
            rise_time_ms: rise,
            settling_time_ms: settling,
            step_count: axis_sr.step_count,
        });
    }

    results
}

/// Classify the response and compute suggested gains.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn classify_and_suggest(
    overshoot: f64,
    rise: f64,
    settling: f64,
    current: &AxisGains,
) -> (TuningRating, AxisGains) {
    let scale_p = |factor: f64| current.p.map(|p| (f64::from(p) * factor) as u32);
    let scale_d = |factor: f64| current.d.map(|d| (f64::from(d) * factor) as u32);

    if overshoot > 40.0 && settling > rise * 3.0 {
        // Oscillating: aggressively lower P, raise D
        (
            TuningRating::Oscillating,
            AxisGains {
                p: scale_p(0.75),
                i: current.i,
                d: scale_d(1.3),
            },
        )
    } else if overshoot > 25.0 {
        // Overshooting: moderate P decrease, slight D increase
        (
            TuningRating::Overshooting,
            AxisGains {
                p: scale_p(0.85),
                i: current.i,
                d: scale_d(1.15),
            },
        )
    } else if rise > 15.0 && overshoot < 5.0 {
        // Sluggish: raise P
        (
            TuningRating::Sluggish,
            AxisGains {
                p: scale_p(1.2),
                i: current.i,
                d: current.d,
            },
        )
    } else if rise < 5.0 && overshoot < 10.0 {
        // Tight: no changes
        (TuningRating::Tight, *current)
    } else {
        // Good: no changes
        (TuningRating::Good, *current)
    }
}

// ---------------------------------------------------------------------------
// I-term windup
// ---------------------------------------------------------------------------

/// Detect I-term windup: frames where |I-term| exceeds |P-term|.
#[allow(clippy::cast_precision_loss)]
fn analyze_windup(session: &Session) -> Vec<AxisWindup> {
    let mut results = Vec::new();

    for axis in Axis::ALL {
        let pid_p = session.field(&SensorField::PidP(axis));
        let pid_i = session.field(&SensorField::PidI(axis));

        if pid_p.len() < MIN_FRAMES_WINDUP || pid_i.len() < MIN_FRAMES_WINDUP {
            continue;
        }

        let len = pid_p.len().min(pid_i.len());
        let mut i_dominant_count = 0usize;
        let mut peak_ratio: f64 = 0.0;

        for j in 0..len {
            let p = pid_p[j].abs();
            let i_val = pid_i[j].abs();

            if p > 1.0 {
                let ratio = i_val / p;
                if ratio > 1.0 {
                    i_dominant_count += 1;
                }
                peak_ratio = peak_ratio.max(ratio);
            } else if i_val > 10.0 {
                i_dominant_count += 1;
            }
        }

        let fraction = i_dominant_count as f64 / len as f64;

        if fraction > 0.01 || peak_ratio > 2.0 {
            results.push(AxisWindup {
                axis,
                i_dominant_fraction: fraction,
                peak_ratio,
            });
        }
    }

    results
}

// ---------------------------------------------------------------------------
// Oscillation frequency
// ---------------------------------------------------------------------------

/// Detect oscillation frequency from error signal (setpoint - gyro) around steps.
#[allow(clippy::cast_precision_loss)]
fn analyze_oscillation(
    session: &Session,
    step_response: Option<&StepResponseAnalysis>,
) -> Vec<AxisOscillation> {
    let Some(sr) = step_response else {
        return Vec::new();
    };

    let sample_rate = session.sample_rate_hz();
    if sample_rate <= 0.0 {
        return Vec::new();
    }

    let axis_names = ["roll", "pitch", "yaw"];
    let mut results = Vec::new();

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let Some(axis_sr) = sr.axes.iter().find(|a| a.axis == axis_names[i]) else {
            continue;
        };

        if axis_sr.step_count < MIN_STEPS_OSCILLATION || axis_sr.overshoot_percent < 20.0 {
            continue;
        }

        let setpoint = session.field(&SensorField::Setpoint(*axis));
        let gyro = session.field(&SensorField::Gyro(*axis));

        if setpoint.len() < OSC_POST_SAMPLES || gyro.len() < OSC_POST_SAMPLES {
            continue;
        }

        let len = setpoint.len().min(gyro.len());
        let steps = detect_steps(&setpoint[..len]);

        let mut error_windows: Vec<Vec<f64>> = Vec::new();
        for &step_idx in &steps {
            let end = step_idx + OSC_POST_SAMPLES;
            if end > len {
                continue;
            }

            let error: Vec<f64> = (step_idx..end).map(|j| setpoint[j] - gyro[j]).collect();

            let max_err = error.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
            if max_err > 2000.0 {
                continue;
            }

            error_windows.push(error);
        }

        if error_windows.is_empty() {
            continue;
        }

        let (freq, mag) = find_dominant_oscillation(&error_windows, sample_rate);

        results.push(AxisOscillation {
            axis: *axis,
            frequency_hz: freq,
            magnitude_db: mag,
            overshoot_percent: axis_sr.overshoot_percent,
        });
    }

    results
}

/// Detect step indices using windowed lookback.
fn detect_steps(setpoint: &[f64]) -> Vec<usize> {
    let mut steps = Vec::new();
    let mut last_step = 0usize;
    let start = OSC_STEP_LOOKBACK;

    for i in start..setpoint.len() {
        if i > last_step && i - last_step < OSC_STEP_COOLDOWN && last_step >= start {
            continue;
        }
        let delta = (setpoint[i] - setpoint[i - OSC_STEP_LOOKBACK]).abs();
        if delta >= MIN_STEP_SIZE {
            steps.push(i);
            last_step = i;
        }
    }
    steps
}

/// Average error windows and find dominant frequency via FFT.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn find_dominant_oscillation(windows: &[Vec<f64>], sample_rate: f64) -> (Option<f64>, Option<f64>) {
    use rustfft::num_complex::Complex;
    use rustfft::FftPlanner;

    let n = OSC_FFT_SIZE.min(windows.iter().map(Vec::len).min().unwrap_or(0));
    if n < 16 {
        return (None, None);
    }

    let mut avg = vec![0.0f64; n];
    for w in windows {
        for (i, val) in w.iter().take(n).enumerate() {
            avg[i] += val;
        }
    }
    let count = windows.len() as f64;
    for v in &mut avg {
        *v /= count;
    }

    for (i, v) in avg.iter_mut().enumerate() {
        let w = 0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos());
        *v *= w;
    }

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(n);
    let mut buffer: Vec<Complex<f64>> = avg.iter().map(|&v| Complex { re: v, im: 0.0 }).collect();
    fft.process(&mut buffer);

    let freq_resolution = sample_rate / n as f64;
    let min_bin = (10.0 / freq_resolution).ceil() as usize;
    let max_bin = n / 2;

    if min_bin >= max_bin {
        return (None, None);
    }

    let mut best_bin = min_bin;
    let mut best_mag = 0.0f64;

    for (bin, c) in buffer.iter().enumerate().take(max_bin).skip(min_bin) {
        let mag = (c.re.powi(2) + c.im.powi(2)).sqrt();
        if mag > best_mag {
            best_mag = mag;
            best_bin = bin;
        }
    }

    if best_mag < 1.0 {
        return (None, None);
    }

    let freq = best_bin as f64 * freq_resolution;
    let mag_db = 20.0 * (best_mag / n as f64).max(1e-10).log10();

    (Some(freq), Some(mag_db))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_steps_finds_large_changes() {
        // Simulate a ramp: 0 for 100 samples, then 100 for the rest
        // The lookback window detects the change at index 100+LOOKBACK
        let mut sp = vec![0.0; 300];
        for i in 100..300 {
            sp[i] = 100.0;
        }
        let steps = detect_steps(&sp);
        assert!(!steps.is_empty(), "should detect at least one step");
        // Step detected when lookback window spans the transition
        assert!(steps[0] >= 100, "step should be at or after the transition");
    }

    #[test]
    fn detect_steps_respects_cooldown() {
        let mut sp = vec![0.0; 400];
        // First step at 100
        for i in 100..150 {
            sp[i] = 100.0;
        }
        // Second step at 150 — within cooldown (100 samples)
        for i in 150..400 {
            sp[i] = 200.0;
        }
        let steps = detect_steps(&sp);
        // Both transitions are within cooldown distance of each other
        assert!(steps.len() <= 2, "cooldown should limit detections");
    }

    #[test]
    fn find_dominant_oscillation_with_sine() {
        let n = 128;
        let sample_rate = 1000.0;
        let freq = 50.0;
        let window: Vec<f64> = (0..n)
            .map(|i| (2.0 * std::f64::consts::PI * freq * i as f64 / sample_rate).sin() * 100.0)
            .collect();

        let (detected_freq, detected_mag) = find_dominant_oscillation(&[window], sample_rate);
        assert!(detected_freq.is_some(), "should detect a frequency");
        let f = detected_freq.unwrap();
        assert!(
            (f - freq).abs() < 15.0,
            "detected {f:.1} Hz, expected ~{freq} Hz"
        );
        assert!(detected_mag.is_some());
    }

    #[test]
    fn classify_oscillating() {
        let gains = AxisGains {
            p: Some(40),
            i: Some(60),
            d: Some(30),
        };
        let (rating, suggested) = classify_and_suggest(50.0, 4.0, 60.0, &gains);
        assert_eq!(rating, TuningRating::Oscillating);
        assert_eq!(suggested.p, Some(30)); // 40 * 0.75
        assert_eq!(suggested.d, Some(39)); // 30 * 1.3
    }

    #[test]
    fn classify_overshooting() {
        let gains = AxisGains {
            p: Some(39),
            i: Some(80),
            d: Some(40),
        };
        let (rating, suggested) = classify_and_suggest(30.0, 5.0, 20.0, &gains);
        assert_eq!(rating, TuningRating::Overshooting);
        assert_eq!(suggested.p, Some(33)); // 39 * 0.85
        assert_eq!(suggested.d, Some(46)); // 40 * 1.15
    }

    #[test]
    fn classify_sluggish() {
        let gains = AxisGains {
            p: Some(35),
            i: Some(75),
            d: Some(35),
        };
        let (rating, suggested) = classify_and_suggest(2.0, 20.0, 25.0, &gains);
        assert_eq!(rating, TuningRating::Sluggish);
        assert_eq!(suggested.p, Some(42)); // 35 * 1.2
        assert_eq!(suggested.d, Some(35)); // unchanged
    }

    #[test]
    fn classify_tight() {
        let gains = AxisGains {
            p: Some(35),
            i: Some(75),
            d: Some(35),
        };
        let (rating, suggested) = classify_and_suggest(5.0, 3.0, 8.0, &gains);
        assert_eq!(rating, TuningRating::Tight);
        assert_eq!(suggested, gains); // no changes
    }

    #[test]
    fn classify_without_gains() {
        let gains = AxisGains::default();
        let (rating, suggested) = classify_and_suggest(30.0, 5.0, 20.0, &gains);
        assert_eq!(rating, TuningRating::Overshooting);
        assert_eq!(suggested.p, None); // can't suggest without current values
        assert_eq!(suggested.d, None);
    }
}
