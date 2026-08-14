use az::{Az, SaturatingAs};
use serde::Serialize;

use crate::session::Format;
use crate::types::{Axis, AxisGains, PidGains, Session};

use super::util;

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
    /// Ready-to-paste Betaflight CLI `set` commands applying every
    /// changed gain from `tuning`, terminated by `save`. `None` when
    /// the log isn't Betaflight-family or no suggestion changes a gain.
    pub betaflight_cli_diff: Option<String>,
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

/// |P| below this (mixer-output units) is treated as "P inactive";
/// I/P ratios against a near-zero P would be meaninglessly large.
const WINDUP_P_ACTIVE_MIN: f64 = 1.0;

/// With P inactive, |I| above this still counts as I-dominant —
/// a large I-term holding against nothing is the windup signature.
const WINDUP_I_STANDALONE_MIN: f64 = 10.0;

/// Report an axis when I dominates more than this fraction of frames…
const WINDUP_FRACTION_REPORT: f64 = 0.01;

/// …or when the peak |I/P| ratio exceeds this, even briefly.
const WINDUP_PEAK_RATIO_REPORT: f64 = 2.0;

/// Minimum step count for oscillation frequency detection.
const MIN_STEPS_OSCILLATION: usize = 3;

/// FFT window size for oscillation detection around steps.
const OSC_FFT_SIZE: usize = 128;

/// Post-step window in samples for oscillation FFT.
const OSC_POST_SAMPLES: usize = 200;

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

    // The `set p_roll = …` vocabulary is Betaflight's; other firmware
    // has entirely different parameter systems (and gain scales), so
    // emitting a diff there would be actively misleading.
    let betaflight_cli_diff = if session.meta.format == Format::Betaflight {
        build_betaflight_cli_diff(&tuning)
    } else {
        None
    };

    Some(PidAnalysis {
        windup,
        oscillation,
        tuning,
        betaflight_cli_diff,
    })
}

/// Render changed gains as Betaflight CLI commands
/// (`set p_roll = 38\n…\nsave`). Returns `None` when every suggestion
/// keeps the current gains (nothing worth pasting).
fn build_betaflight_cli_diff(tuning: &[TuningSuggestion]) -> Option<String> {
    let changes: Vec<String> = tuning
        .iter()
        .flat_map(|t| {
            [
                ("p", t.current.p, t.suggested.p),
                ("i", t.current.i, t.suggested.i),
                ("d", t.current.d, t.suggested.d),
            ]
            .into_iter()
            .filter_map(move |(term, current, suggested)| {
                let (current, suggested) = current.zip(suggested)?;
                (current != suggested)
                    .then(|| format!("set {term}_{axis} = {suggested}", axis = t.axis))
            })
        })
        .collect();

    if changes.is_empty() {
        return None;
    }

    let mut out = String::from("# propwash suggested PID changes\n");
    out.push_str(&changes.join("\n"));
    out.push_str("\nsave\n");
    Some(out)
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

    let mut results = Vec::new();

    for axis in &Axis::ALL {
        let Some(axis_sr) = sr.axes.iter().find(|a| a.axis == *axis) else {
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
fn classify_and_suggest(
    overshoot: f64,
    rise: f64,
    settling: f64,
    current: &AxisGains,
) -> (TuningRating, AxisGains) {
    let scale_p = |factor: f64| {
        current
            .p
            .map(|p| (f64::from(p) * factor).saturating_as::<u32>())
    };
    let scale_d = |factor: f64| {
        current
            .d
            .map(|d| (f64::from(d) * factor).saturating_as::<u32>())
    };

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
///
/// Reads the typed [`Session::pid_terms`] traces — populated by
/// Betaflight-family logs; empty for AP/PX4/MAVLink, where this
/// returns no findings. Only axes crossing a report threshold are
/// included, so a well-behaved log yields an empty `Vec`.
fn analyze_windup(session: &Session) -> Vec<AxisWindup> {
    let terms = &session.pid_terms;
    let mut results = Vec::new();

    for axis in Axis::ALL {
        let pid_p = terms.p.get(axis);
        let pid_i = terms.i.get(axis);

        if pid_p.len() < MIN_FRAMES_WINDUP || pid_i.len() < MIN_FRAMES_WINDUP {
            continue;
        }

        let len = pid_p.len().min(pid_i.len());
        let mut i_dominant_count = 0usize;
        let mut peak_ratio: f64 = 0.0;

        for (&p, &i_term) in pid_p.iter().zip(pid_i) {
            let p_abs = f64::from(p).abs();
            let i_abs = f64::from(i_term).abs();

            if p_abs > WINDUP_P_ACTIVE_MIN {
                let ratio = i_abs / p_abs;
                if ratio > 1.0 {
                    i_dominant_count += 1;
                }
                peak_ratio = peak_ratio.max(ratio);
            } else if i_abs > WINDUP_I_STANDALONE_MIN {
                i_dominant_count += 1;
            }
        }

        let fraction = i_dominant_count.az::<f64>() / len.az::<f64>();

        if fraction > WINDUP_FRACTION_REPORT || peak_ratio > WINDUP_PEAK_RATIO_REPORT {
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

    let mut results = Vec::new();

    for axis in &Axis::ALL {
        let Some(axis_sr) = sr.axes.iter().find(|a| a.axis == *axis) else {
            continue;
        };

        if axis_sr.step_count < MIN_STEPS_OSCILLATION || axis_sr.overshoot_percent < 20.0 {
            continue;
        }

        let setpoint: &[f64] = bytemuck::cast_slice(session.setpoint.values.get(*axis).as_slice());
        let gyro: &[f64] = bytemuck::cast_slice(session.gyro.values.get(*axis).as_slice());

        if setpoint.len() < OSC_POST_SAMPLES || gyro.len() < OSC_POST_SAMPLES {
            continue;
        }

        let len = setpoint.len().min(gyro.len());
        let steps = util::detect_steps(&setpoint[..len]);

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

/// Average error windows and find dominant frequency via FFT.
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
    let count = windows.len().az::<f64>();
    for v in &mut avg {
        *v /= count;
    }

    for (i, v) in avg.iter_mut().enumerate() {
        let w =
            0.5 * (1.0 - (2.0 * std::f64::consts::PI * i.az::<f64>() / (n - 1).az::<f64>()).cos());
        *v *= w;
    }

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(n);
    let mut buffer: Vec<Complex<f64>> = avg.iter().map(|&v| Complex { re: v, im: 0.0 }).collect();
    fft.process(&mut buffer);

    let freq_resolution = sample_rate / n.az::<f64>();
    let min_bin = (10.0 / freq_resolution).ceil().az::<usize>();
    let max_bin = n / 2;

    if min_bin >= max_bin {
        return (None, None);
    }

    let mut best_bin = min_bin;
    let mut best_mag = 0.0f64;

    for (bin, c) in buffer.iter().enumerate().take(max_bin).skip(min_bin) {
        let mag = c.re.hypot(c.im);
        if mag > best_mag {
            best_mag = mag;
            best_bin = bin;
        }
    }

    if best_mag < 1.0 {
        return (None, None);
    }

    let freq = best_bin.az::<f64>() * freq_resolution;
    let mag_db = 20.0 * (best_mag / n.az::<f64>()).max(1e-10).log10();

    (Some(freq), Some(mag_db))
}

#[cfg(test)]
mod tests {
    use super::*;

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

    /// Session with `n` frames of constant P and I terms on the roll axis.
    fn session_with_roll_terms(n: usize, p: f32, i: f32) -> Session {
        let mut s = Session::default();
        s.pid_terms.time_us = (0..n).map(|j| (j * 250).az::<u64>()).collect();
        s.pid_terms.p.roll = vec![p; n];
        s.pid_terms.i.roll = vec![i; n];
        s
    }

    fn suggestion(axis: Axis, current: AxisGains, suggested: AxisGains) -> TuningSuggestion {
        TuningSuggestion {
            axis,
            rating: TuningRating::Overshooting,
            current,
            suggested,
            overshoot_percent: 30.0,
            rise_time_ms: 5.0,
            settling_time_ms: 20.0,
            step_count: 5,
        }
    }

    #[test]
    fn cli_diff_emits_only_changed_gains() {
        let s = suggestion(
            Axis::Roll,
            AxisGains {
                p: Some(45),
                i: Some(80),
                d: Some(40),
            },
            AxisGains {
                p: Some(38),
                i: Some(80),
                d: Some(46),
            },
        );
        let diff = build_betaflight_cli_diff(&[s]).expect("changed gains produce a diff");
        assert_eq!(
            diff, "# propwash suggested PID changes\nset p_roll = 38\nset d_roll = 46\nsave\n",
            "unchanged I must not appear; save terminates"
        );
    }

    #[test]
    fn cli_diff_none_when_nothing_changes() {
        let gains = AxisGains {
            p: Some(45),
            i: Some(80),
            d: Some(40),
        };
        let s = suggestion(Axis::Pitch, gains, gains);
        assert!(build_betaflight_cli_diff(&[s]).is_none());
    }

    #[test]
    fn cli_diff_skips_axes_without_known_gains() {
        let s = suggestion(
            Axis::Yaw,
            AxisGains::default(),
            AxisGains {
                p: Some(30),
                i: None,
                d: None,
            },
        );
        assert!(
            build_betaflight_cli_diff(&[s]).is_none(),
            "no current value → nothing to diff against"
        );
    }

    #[test]
    fn cli_diff_gated_to_betaflight_sessions() {
        // A non-BF session with windup data still gets PidAnalysis,
        // but never a Betaflight CLI diff.
        let mut s = session_with_roll_terms(1000, 10.0, 30.0);
        s.meta.format = crate::session::Format::Px4;
        let analysis =
            analyze_pid(&s, None, &PidGains::default()).expect("windup yields an analysis");
        assert!(analysis.betaflight_cli_diff.is_none());
    }

    #[test]
    fn windup_detected_when_i_dominates() {
        let s = session_with_roll_terms(1000, 10.0, 30.0);
        let windup = analyze_windup(&s);
        assert_eq!(windup.len(), 1, "sustained |I| > |P| should be reported");
        assert_eq!(windup[0].axis, Axis::Roll);
        assert!((windup[0].i_dominant_fraction - 1.0).abs() < 1e-9);
        assert!((windup[0].peak_ratio - 3.0).abs() < 1e-9);
    }

    #[test]
    fn windup_quiet_when_p_dominates() {
        let s = session_with_roll_terms(1000, 30.0, 10.0);
        assert!(analyze_windup(&s).is_empty(), "healthy P-dominant trace");
    }

    #[test]
    fn windup_detected_with_inactive_p_and_large_i() {
        // P near zero (stick centred) while I holds a big offset —
        // the classic windup signature after a sustained disturbance.
        let s = session_with_roll_terms(1000, 0.0, 50.0);
        let windup = analyze_windup(&s);
        assert_eq!(windup.len(), 1);
        assert!((windup[0].i_dominant_fraction - 1.0).abs() < 1e-9);
    }

    #[test]
    fn windup_skips_short_traces() {
        let s = session_with_roll_terms(MIN_FRAMES_WINDUP - 1, 10.0, 30.0);
        assert!(
            analyze_windup(&s).is_empty(),
            "below MIN_FRAMES_WINDUP no assessment is made"
        );
    }

    #[test]
    fn windup_empty_for_sessions_without_pid_terms() {
        assert!(analyze_windup(&Session::default()).is_empty());
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
