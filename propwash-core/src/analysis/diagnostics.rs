use std::fmt::Write;

use serde::Serialize;

use super::events::{EventKind, FlightEvent};
use super::fft::VibrationAnalysis;
use super::pid::{PidAnalysis, TuningRating};
use super::step_response::StepResponseAnalysis;
use crate::types::FilterConfig;

#[derive(Debug, Clone, Serialize)]
pub struct Diagnostic {
    pub severity: Severity,
    pub category: &'static str,
    pub message: String,
    pub detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum Severity {
    Info,
    Warning,
    Problem,
}

pub fn diagnose(
    events: &[FlightEvent],
    vibration: Option<&VibrationAnalysis>,
    step_response: Option<&StepResponseAnalysis>,
    pid: Option<&PidAnalysis>,
    filter_config: &FilterConfig,
    motor_count: usize,
    duration_seconds: f64,
) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    diagnostics.extend(diagnose_motor_saturation(events, motor_count));
    diagnostics.extend(diagnose_desync(events));
    diagnostics.extend(diagnose_overshoot(events, duration_seconds));
    diagnostics.extend(diagnose_gyro_spikes(events));
    if let Some(vib) = vibration {
        diagnostics.extend(diagnose_vibration(vib));
        diagnostics.extend(diagnose_throttle_response(vib));
        diagnostics.extend(diagnose_fc_mounting(vib));
        diagnostics.extend(diagnose_filter_adequacy(vib, filter_config));
    }
    if let Some(sr) = step_response {
        if let Some(vib) = vibration {
            diagnostics.extend(diagnose_d_term_noise(sr, vib));
        }
    }
    if let Some(pid) = pid {
        diagnostics.extend(diagnose_tuning_from_analysis(pid));
        diagnostics.extend(diagnose_windup(pid));
        diagnostics.extend(diagnose_oscillation_frequency(pid));
    }

    diagnostics.sort_by_key(|d| std::cmp::Reverse(d.severity));
    diagnostics
}

fn diagnose_motor_saturation(events: &[FlightEvent], motor_count: usize) -> Vec<Diagnostic> {
    let mut motor_sat_frames = vec![0usize; motor_count];
    for event in events {
        if let EventKind::MotorSaturation {
            motor_index,
            duration_frames,
        } = &event.kind
        {
            if *motor_index < motor_count {
                motor_sat_frames[*motor_index] += duration_frames;
            }
        }
    }

    let total_sat: usize = motor_sat_frames.iter().sum();
    if total_sat == 0 {
        return Vec::new();
    }

    let max_sat = *motor_sat_frames.iter().max().unwrap_or(&0);
    let min_sat = *motor_sat_frames.iter().min().unwrap_or(&0);

    let sat_summary = || {
        motor_sat_frames
            .iter()
            .enumerate()
            .map(|(i, &f)| format!("motor[{i}]={f}"))
            .collect::<Vec<_>>()
            .join(", ")
    };

    if max_sat > 100 && min_sat > 0 {
        vec![Diagnostic {
            severity: Severity::Warning,
            category: "motors",
            message: "Motor saturation detected on all motors".into(),
            detail: format!(
                "Saturated frames per motor: {}. This suggests the motor/prop combo is underpowered for the flying style.",
                sat_summary()
            ),
        }]
    } else if max_sat > 50 && max_sat > min_sat * 3 {
        let worst = motor_sat_frames
            .iter()
            .enumerate()
            .max_by_key(|(_, &f)| f)
            .map_or(0, |(i, _)| i);
        vec![Diagnostic {
            severity: Severity::Problem,
            category: "motors",
            message: format!("Motor[{worst}] saturating significantly more than others"),
            detail: format!(
                "Saturated frames: {}. Asymmetric saturation suggests a mechanical issue (damaged prop, bent motor shaft, loose mount) or CG imbalance on that side.",
                sat_summary()
            ),
        }]
    } else if total_sat > 20 {
        vec![Diagnostic {
            severity: Severity::Info,
            category: "motors",
            message: "Minor motor saturation detected".into(),
            detail: format!(
                "Saturated frames: {}. Brief saturation during aggressive maneuvers is normal.",
                sat_summary()
            ),
        }]
    } else {
        Vec::new()
    }
}

fn diagnose_desync(events: &[FlightEvent]) -> Vec<Diagnostic> {
    let desync_count: usize = events
        .iter()
        .filter(|e| matches!(&e.kind, EventKind::Desync { .. }))
        .count();

    if desync_count > 50 {
        vec![Diagnostic {
            severity: Severity::Problem,
            category: "esc",
            message: format!("ESC desync detected ({desync_count} events)"),
            detail: "One motor repeatedly spiking to max while others are at normal levels indicates ESC desync. Check motor timing, reduce DShot speed, or try a different ESC protocol. Also check for damaged motor wires or bad solder joints.".into(),
        }]
    } else if desync_count > 5 {
        vec![Diagnostic {
            severity: Severity::Warning,
            category: "esc",
            message: format!("Possible ESC desync ({desync_count} events)"),
            detail: "Brief single-motor spikes detected. May be desync or aggressive PID correction. If twitches are visible in flight, check ESC settings.".into(),
        }]
    } else {
        Vec::new()
    }
}

fn diagnose_overshoot(events: &[FlightEvent], duration_seconds: f64) -> Vec<Diagnostic> {
    let mut axis_overshoots: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];

    for event in events {
        if let EventKind::Overshoot {
            axis,
            overshoot_percent,
            ..
        } = &event.kind
        {
            let idx = match *axis {
                "roll" => 0,
                "pitch" => 1,
                _ => 2,
            };
            axis_overshoots[idx].push(*overshoot_percent);
        }
    }

    let mut diagnostics = Vec::new();
    let axis_names = ["roll", "pitch", "yaw"];
    for (idx, overshoots) in axis_overshoots.iter().enumerate() {
        if overshoots.is_empty() {
            continue;
        }

        #[allow(clippy::cast_precision_loss)]
        let count_f = overshoots.len() as f64;
        let rate = count_f / duration_seconds;
        let avg = overshoots.iter().sum::<f64>() / count_f;
        let max = overshoots.iter().copied().reduce(f64::max).unwrap_or(0.0);

        if max > 100.0 {
            diagnostics.push(Diagnostic {
                severity: Severity::Problem,
                category: "pid",
                message: format!(
                    "Severe {} overshoot (peak {:.0}%, avg {:.0}%)",
                    axis_names[idx], max, avg
                ),
                detail: format!(
                    "{} overshoot events ({:.1}/sec). Peak overshoot over 100% indicates P gain is too high or D gain is too low on {} axis.",
                    overshoots.len(), rate, axis_names[idx]
                ),
            });
        } else if rate > 5.0 && avg > 20.0 {
            diagnostics.push(Diagnostic {
                severity: Severity::Warning,
                category: "pid",
                message: format!(
                    "Frequent {} overshoot ({:.0}/sec, avg {:.0}%)",
                    axis_names[idx], rate, avg
                ),
                detail: format!(
                    "Consider increasing D gain or decreasing P gain slightly on {} axis.",
                    axis_names[idx]
                ),
            });
        }
    }
    diagnostics
}

fn diagnose_gyro_spikes(events: &[FlightEvent]) -> Vec<Diagnostic> {
    let mut max_spike: [f64; 3] = [0.0; 3];
    let mut spike_count: [usize; 3] = [0; 3];

    for event in events {
        if let EventKind::GyroSpike { axis, magnitude } = &event.kind {
            let idx = match *axis {
                "roll" => 0,
                "pitch" => 1,
                _ => 2,
            };
            max_spike[idx] = max_spike[idx].max(magnitude.abs());
            spike_count[idx] += 1;
        }
    }

    let mut diagnostics = Vec::new();
    let axis_names = ["roll", "pitch", "yaw"];
    for (idx, (&peak, &count)) in max_spike.iter().zip(spike_count.iter()).enumerate() {
        if peak > 1500.0 {
            diagnostics.push(Diagnostic {
                severity: Severity::Problem,
                category: "mechanical",
                message: format!(
                    "Extreme {} gyro spike ({:.0}°/s, {} events)",
                    axis_names[idx], peak, count
                ),
                detail: "Gyro readings above 1500°/s typically indicate a crash, prop strike, or gyro clipping. Check for damage.".into(),
            });
        }
    }
    diagnostics
}

fn diagnose_vibration(vib: &VibrationAnalysis) -> Vec<Diagnostic> {
    let axis_names = ["roll", "pitch", "yaw"];
    let mut diagnostics = Vec::new();

    for spectrum in &vib.spectra {
        let axis_idx = match spectrum.axis {
            "roll" => 0,
            "pitch" => 1,
            _ => 2,
        };
        let noise_floor = vib.noise_floor_db[axis_idx];

        if let Some(peak) = spectrum.peaks.first() {
            let prominence = peak.magnitude_db - noise_floor;

            if prominence > 25.0 && peak.frequency_hz > 100.0 && peak.frequency_hz < 400.0 {
                diagnostics.push(Diagnostic {
                    severity: Severity::Warning,
                    category: "vibration",
                    message: format!(
                        "Strong resonance at {:.0} Hz on {} ({:.1} dB above floor)",
                        peak.frequency_hz, axis_names[axis_idx], prominence
                    ),
                    detail: format!(
                        "A prominent peak at {:.0} Hz suggests frame or prop resonance. Consider adding a notch filter at this frequency if not already configured.",
                        peak.frequency_hz
                    ),
                });
            }

            if prominence > 15.0 && peak.frequency_hz >= 20.0 && peak.frequency_hz <= 100.0 {
                diagnostics.push(Diagnostic {
                    severity: Severity::Warning,
                    category: "vibration",
                    message: format!(
                        "Low-frequency noise at {:.0} Hz on {} ({:.1} dB above floor)",
                        peak.frequency_hz, axis_names[axis_idx], prominence
                    ),
                    detail: "Low-frequency peaks (20-100 Hz) often indicate propwash oscillation, PID oscillation, or mechanical looseness.".into(),
                });
            }
        }
    }
    diagnostics
}

fn diagnose_throttle_response(vib: &VibrationAnalysis) -> Vec<Diagnostic> {
    if vib.throttle_bands.len() < 2 {
        return Vec::new();
    }

    let mut diagnostics = Vec::new();
    for spectrum_idx in 0..3 {
        let axis_name = match spectrum_idx {
            0 => "roll",
            1 => "pitch",
            _ => "yaw",
        };

        let mut band_peaks: Vec<(f64, f64)> = Vec::new();
        for band in &vib.throttle_bands {
            if let Some(spectrum) = band.spectra.get(spectrum_idx) {
                if let Some(peak) = spectrum.peaks.first() {
                    band_peaks.push((band.throttle_min, peak.frequency_hz));
                }
            }
        }

        if band_peaks.len() >= 2 {
            let freqs: Vec<f64> = band_peaks.iter().map(|(_, f)| *f).collect();
            let min_f = freqs.iter().copied().reduce(f64::min).unwrap_or(0.0);
            let max_f = freqs.iter().copied().reduce(f64::max).unwrap_or(0.0);

            if max_f > min_f * 2.0 && min_f > 10.0 {
                diagnostics.push(Diagnostic {
                    severity: Severity::Info,
                    category: "vibration",
                    message: format!(
                        "Dominant {axis_name} frequency shifts with throttle ({min_f:.0}-{max_f:.0} Hz)"
                    ),
                    detail: "Frequency that scales with throttle indicates motor noise (RPM-dependent). This is normal and addressed by RPM filtering or dynamic notch filters.".into(),
                });
            }
        }
    }
    diagnostics
}

fn diagnose_fc_mounting(vib: &VibrationAnalysis) -> Vec<Diagnostic> {
    let Some(accel) = &vib.accel else {
        return Vec::new();
    };

    let mut diagnostics = Vec::new();
    let accel_rms_total =
        (accel.rms[0].powi(2) + accel.rms[1].powi(2) + accel.rms[2].powi(2)).sqrt();

    if accel_rms_total > 200.0 {
        diagnostics.push(Diagnostic {
            severity: Severity::Problem,
            category: "mounting",
            message: format!(
                "High FC board vibration (accel RMS: X={:.0} Y={:.0} Z={:.0})",
                accel.rms[0], accel.rms[1], accel.rms[2]
            ),
            detail: "The accelerometer shows excessive vibration of the FC board itself. Check soft mount grommets, stack screws, and standoffs for looseness or wear. If using hard mount, consider switching to soft mounting.".into(),
        });
    } else if accel_rms_total > 100.0 {
        diagnostics.push(Diagnostic {
            severity: Severity::Warning,
            category: "mounting",
            message: format!(
                "Moderate FC board vibration (accel RMS: X={:.0} Y={:.0} Z={:.0})",
                accel.rms[0], accel.rms[1], accel.rms[2]
            ),
            detail: "FC board vibration is elevated. Check that soft mount grommets are intact and stack hardware is tight.".into(),
        });
    }

    if accel.rms[2] > accel.rms[0] * 2.0 && accel.rms[2] > accel.rms[1] * 2.0 && accel.rms[2] > 50.0
    {
        diagnostics.push(Diagnostic {
            severity: Severity::Warning,
            category: "mounting",
            message: "FC vibration dominated by Z-axis (vertical bounce)".into(),
            detail: "The FC board is bouncing vertically more than it vibrates laterally. This pattern suggests the soft mount grommets are too soft or the FC stack has vertical play. Check standoff height and grommet condition.".into(),
        });
    }
    diagnostics
}

fn diagnose_filter_adequacy(vib: &VibrationAnalysis, config: &FilterConfig) -> Vec<Diagnostic> {
    let gyro_lpf = config.gyro_lpf_hz.unwrap_or(0.0);
    if gyro_lpf <= 0.0 {
        return Vec::new(); // No filter config available
    }

    let mut diagnostics = Vec::new();

    for spectrum in &vib.spectra {
        let noise_floor = vib.noise_floor_db[match spectrum.axis {
            "roll" => 0,
            "pitch" => 1,
            _ => 2,
        }];

        for peak in &spectrum.peaks {
            let prominence = peak.magnitude_db - noise_floor;
            if prominence < 15.0 {
                continue; // Not significant enough
            }

            let freq = peak.frequency_hz;

            // Check if a significant peak is below the gyro LPF cutoff (not being filtered)
            if freq < gyro_lpf * 0.9 && freq > 20.0 {
                // Check if any notch filter covers this frequency
                let covered_by_notch = [config.gyro_notch1_hz, config.gyro_notch2_hz]
                    .iter()
                    .any(|n| n.is_some_and(|nf| (nf - freq).abs() < freq * 0.2));

                let covered_by_dyn_notch = config.dyn_notch_min_hz.is_some_and(|min| freq >= min)
                    && config.dyn_notch_max_hz.is_none_or(|max| freq <= max);

                if !covered_by_notch && !covered_by_dyn_notch {
                    diagnostics.push(Diagnostic {
                        severity: Severity::Warning,
                        category: "filters",
                        message: format!(
                            "Noise peak at {freq:.0} Hz on {} is below LPF cutoff ({gyro_lpf:.0} Hz)",
                            spectrum.axis
                        ),
                        detail: format!(
                            "A {prominence:.0} dB peak at {freq:.0} Hz passes through the gyro LPF (set at {gyro_lpf:.0} Hz) and is not covered by any notch filter. Consider adding a notch filter at {freq:.0} Hz or lowering the LPF cutoff."
                        ),
                    });
                }
            }
        }
    }

    diagnostics
}

fn diagnose_tuning_from_analysis(pid: &PidAnalysis) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    for t in &pid.tuning {
        let axis = t.axis.as_str();
        let rating = t.rating;

        let severity = match rating {
            TuningRating::Oscillating => Severity::Problem,
            TuningRating::Overshooting | TuningRating::Sluggish => Severity::Warning,
            TuningRating::Tight | TuningRating::Good => Severity::Info,
        };

        let mut detail = match rating {
            TuningRating::Oscillating => format!(
                "Lower P gain on {axis} axis. If already low, raise D gain to dampen oscillation."
            ),
            TuningRating::Overshooting => {
                format!("Reduce P gain or increase D gain on {axis} axis to reduce overshoot.")
            }
            TuningRating::Sluggish => {
                format!("Increase P gain on {axis} axis for faster response.")
            }
            TuningRating::Tight => format!("{axis} axis tuning looks tight."),
            TuningRating::Good => format!("{axis} axis tuning is good."),
        };

        // Append specific gain suggestions when available
        if t.current.p.is_some() || t.suggested.p.is_some() {
            let gains_changed = t.current != t.suggested;
            if gains_changed {
                detail += " Try";
                if let (Some(cur), Some(sug)) = (t.current.p, t.suggested.p) {
                    if cur != sug {
                        let _ = write!(detail, " P: {cur} → {sug}");
                    }
                }
                if let (Some(cur), Some(sug)) = (t.current.d, t.suggested.d) {
                    if cur != sug {
                        let _ = write!(detail, ", D: {cur} → {sug}");
                    }
                }
                detail += ".";
            } else if let (Some(p), Some(d)) = (t.current.p, t.current.d) {
                let _ = write!(detail, " Current gains — P: {p}, D: {d}.");
            }
        }

        let _ = write!(
            detail,
            " ({:.0}% overshoot, {:.1}ms rise, {} steps)",
            t.overshoot_percent, t.rise_time_ms, t.step_count
        );

        diagnostics.push(Diagnostic {
            severity,
            category: "tuning",
            message: format!("{} axis response: {}", capitalize(axis), rating.as_str()),
            detail,
        });
    }

    diagnostics
}

fn diagnose_d_term_noise(sr: &StepResponseAnalysis, vib: &VibrationAnalysis) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    // Average noise floor across axes
    let avg_noise_floor =
        (vib.noise_floor_db[0] + vib.noise_floor_db[1] + vib.noise_floor_db[2]) / 3.0;

    // Check for axes that are overshooting AND have a high noise floor
    // This means the pilot can't safely raise D to control overshoot
    let overshooting_axes: Vec<&str> = sr
        .axes
        .iter()
        .filter(|a| a.step_count >= 3 && a.overshoot_percent > 25.0)
        .map(|a| a.axis)
        .collect();

    if !overshooting_axes.is_empty() && avg_noise_floor > 35.0 {
        let axes_str = overshooting_axes.join(", ");
        diagnostics.push(Diagnostic {
            severity: Severity::Warning,
            category: "tuning",
            message: format!("High noise floor ({avg_noise_floor:.0} dB) limits D-gain headroom"),
            detail: format!(
                "{axes_str} axis overshoot could be reduced with more D gain, but the noise floor \
                 is too high ({avg_noise_floor:.0} dB) — raising D would amplify motor noise. \
                 Improve filtering first (lower gyro LPF or add notch filters), then increase D."
            ),
        });
    }

    diagnostics
}

fn diagnose_windup(pid: &PidAnalysis) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    for w in &pid.windup {
        let axis = w.axis.as_str();
        if w.i_dominant_fraction > 0.3 {
            diagnostics.push(Diagnostic {
                severity: Severity::Warning,
                category: "tuning",
                message: format!(
                    "{} I-term dominates P-term {:.0}% of the time",
                    capitalize(axis),
                    w.i_dominant_fraction * 100.0
                ),
                detail: format!(
                    "The I-term exceeds the P-term for {:.0}% of the flight on {axis} axis \
                     (peak ratio {:.1}x). This suggests I gain is too high — the integrator \
                     is doing work that P should handle. Try reducing I gain.",
                    w.i_dominant_fraction * 100.0,
                    w.peak_ratio
                ),
            });
        } else if w.peak_ratio > 5.0 {
            diagnostics.push(Diagnostic {
                severity: Severity::Info,
                category: "tuning",
                message: format!(
                    "{} I-term spike detected (peak {:.1}x P-term)",
                    capitalize(axis),
                    w.peak_ratio
                ),
                detail: format!(
                    "The {axis} I-term briefly reached {:.1}x the P-term value. \
                     Occasional I-term spikes during sustained maneuvers are normal, \
                     but frequent spikes may indicate I gain is slightly high.",
                    w.peak_ratio
                ),
            });
        }
    }

    diagnostics
}

fn diagnose_oscillation_frequency(pid: &PidAnalysis) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    for osc in &pid.oscillation {
        let axis = osc.axis.as_str();
        if let (Some(freq), Some(mag)) = (osc.frequency_hz, osc.magnitude_db) {
            if mag > -20.0 && osc.overshoot_percent > 20.0 {
                diagnostics.push(Diagnostic {
                    severity: Severity::Info,
                    category: "tuning",
                    message: format!("{} axis oscillating at ~{:.0} Hz", capitalize(axis), freq),
                    detail: format!(
                        "Step response error signal shows a dominant oscillation at {freq:.0} Hz \
                         on {axis} axis ({mag:.0} dB, {overshoot:.0}% overshoot). \
                         If this is below ~30 Hz, it's likely PID oscillation — reduce P or \
                         increase D. If 30-100 Hz, it may be propwash. Above 100 Hz suggests \
                         motor/frame vibration leaking into the control loop.",
                        overshoot = osc.overshoot_percent
                    ),
                });
            }
        }
    }

    diagnostics
}

fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        None => String::new(),
        Some(f) => f.to_uppercase().to_string() + c.as_str(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::pid::{PidAnalysis, TuningRating, TuningSuggestion};
    use crate::analysis::step_response::{AxisStepResponse, StepResponseAnalysis};
    use crate::types::{Axis, AxisGains, PidGains};

    fn make_sr(
        axis: &'static str,
        rise: f64,
        overshoot: f64,
        settling: f64,
        steps: usize,
    ) -> StepResponseAnalysis {
        StepResponseAnalysis {
            axes: vec![AxisStepResponse {
                axis,
                step_count: steps,
                rise_time_ms: rise,
                overshoot_percent: overshoot,
                settling_time_ms: settling,
            }],
        }
    }

    fn bf_gains() -> PidGains {
        PidGains::new(
            AxisGains {
                p: Some(35),
                i: Some(75),
                d: Some(35),
            },
            AxisGains {
                p: Some(39),
                i: Some(80),
                d: Some(40),
            },
            AxisGains {
                p: Some(40),
                i: Some(60),
                d: None,
            },
        )
    }

    fn make_pid_with_tuning(suggestions: Vec<TuningSuggestion>) -> PidAnalysis {
        PidAnalysis {
            windup: Vec::new(),
            oscillation: Vec::new(),
            tuning: suggestions,
        }
    }

    fn make_suggestion(
        axis: Axis,
        rating: TuningRating,
        current: AxisGains,
        suggested: AxisGains,
        overshoot: f64,
        rise: f64,
        settling: f64,
        steps: usize,
    ) -> TuningSuggestion {
        TuningSuggestion {
            axis,
            rating,
            current,
            suggested,
            overshoot_percent: overshoot,
            rise_time_ms: rise,
            settling_time_ms: settling,
            step_count: steps,
        }
    }

    #[test]
    fn tuning_tight() {
        let gains = *bf_gains().get(Axis::Roll);
        let pid = make_pid_with_tuning(vec![make_suggestion(
            Axis::Roll,
            TuningRating::Tight,
            gains,
            gains,
            5.0,
            3.0,
            8.0,
            10,
        )]);
        let diags = diagnose_tuning_from_analysis(&pid);
        assert_eq!(diags.len(), 1);
        assert!(diags[0].message.contains("tight"));
        assert_eq!(diags[0].severity, Severity::Info);
        assert!(
            diags[0].detail.contains("P: 35"),
            "should show current gains"
        );
    }

    #[test]
    fn tuning_overshooting_with_suggestion() {
        let current = *bf_gains().get(Axis::Pitch);
        let suggested = AxisGains {
            p: Some(33),
            i: Some(80),
            d: Some(46),
        };
        let pid = make_pid_with_tuning(vec![make_suggestion(
            Axis::Pitch,
            TuningRating::Overshooting,
            current,
            suggested,
            30.0,
            5.0,
            20.0,
            6,
        )]);
        let diags = diagnose_tuning_from_analysis(&pid);
        assert_eq!(diags.len(), 1);
        assert!(diags[0].message.contains("overshooting"));
        assert_eq!(diags[0].severity, Severity::Warning);
        assert!(
            diags[0].detail.contains("P: 39 → 33"),
            "should suggest P decrease"
        );
        assert!(
            diags[0].detail.contains("D: 40 → 46"),
            "should suggest D increase"
        );
    }

    #[test]
    fn tuning_without_gains() {
        let empty = AxisGains::default();
        let pid = make_pid_with_tuning(vec![make_suggestion(
            Axis::Roll,
            TuningRating::Overshooting,
            empty,
            empty,
            30.0,
            5.0,
            20.0,
            6,
        )]);
        let diags = diagnose_tuning_from_analysis(&pid);
        assert_eq!(diags.len(), 1);
        assert!(diags[0].message.contains("overshooting"));
        assert!(
            !diags[0].detail.contains("→"),
            "no suggestions without gains"
        );
    }

    fn make_vib(noise_floor: [f64; 3]) -> VibrationAnalysis {
        VibrationAnalysis {
            spectra: Vec::new(),
            noise_floor_db: noise_floor,
            throttle_bands: Vec::new(),
            avg_motor_hz: None,
            accel: None,
            propwash: None,
        }
    }

    #[test]
    fn d_term_noise_fires_with_overshoot_and_high_noise() {
        let sr = make_sr("roll", 5.0, 35.0, 20.0, 10);
        let vib = make_vib([40.0, 38.0, 36.0]);
        let diags = diagnose_d_term_noise(&sr, &vib);
        assert_eq!(diags.len(), 1);
        assert!(diags[0].message.contains("noise floor"));
        assert!(diags[0].detail.contains("filtering"));
    }

    #[test]
    fn d_term_noise_quiet_with_low_noise() {
        let sr = make_sr("roll", 5.0, 35.0, 20.0, 10);
        let vib = make_vib([20.0, 18.0, 16.0]);
        let diags = diagnose_d_term_noise(&sr, &vib);
        assert!(diags.is_empty(), "low noise floor should not trigger");
    }

    #[test]
    fn d_term_noise_quiet_with_no_overshoot() {
        let sr = make_sr("roll", 5.0, 10.0, 12.0, 10);
        let vib = make_vib([40.0, 38.0, 36.0]);
        let diags = diagnose_d_term_noise(&sr, &vib);
        assert!(diags.is_empty(), "no overshoot means no D-gain concern");
    }
}
