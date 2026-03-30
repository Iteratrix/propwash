use serde::Serialize;

use super::events::{EventKind, FlightEvent};
use super::fft::VibrationAnalysis;

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
    motor_count: usize,
    duration_seconds: f64,
) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    diagnose_motor_saturation(events, motor_count, &mut diagnostics);
    diagnose_overshoot(events, duration_seconds, &mut diagnostics);
    diagnose_gyro_spikes(events, &mut diagnostics);
    if let Some(vib) = vibration {
        diagnose_vibration(vib, &mut diagnostics);
        diagnose_throttle_response(vib, &mut diagnostics);
        diagnose_fc_mounting(vib, &mut diagnostics);
    }

    diagnostics.sort_by(|a, b| b.severity.cmp(&a.severity));
    diagnostics
}

fn diagnose_motor_saturation(
    events: &[FlightEvent],
    motor_count: usize,
    diagnostics: &mut Vec<Diagnostic>,
) {
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
        return;
    }

    let max_sat = *motor_sat_frames.iter().max().unwrap_or(&0);
    let min_sat = *motor_sat_frames.iter().min().unwrap_or(&0);

    if max_sat > 100 && min_sat > 0 {
        diagnostics.push(Diagnostic {
            severity: Severity::Warning,
            category: "motors",
            message: "Motor saturation detected on all motors".into(),
            detail: format!(
                "Saturated frames per motor: {}. This suggests the motor/prop combo is underpowered for the flying style.",
                motor_sat_frames
                    .iter()
                    .enumerate()
                    .map(|(i, &f)| format!("motor[{i}]={f}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        });
    } else if max_sat > 50 && max_sat > min_sat * 3 {
        let worst = motor_sat_frames
            .iter()
            .enumerate()
            .max_by_key(|(_, &f)| f)
            .map_or(0, |(i, _)| i);
        diagnostics.push(Diagnostic {
            severity: Severity::Problem,
            category: "motors",
            message: format!("Motor[{worst}] saturating significantly more than others"),
            detail: format!(
                "Saturated frames: {}. Asymmetric saturation suggests a mechanical issue (damaged prop, bent motor shaft, loose mount) or CG imbalance on that side.",
                motor_sat_frames
                    .iter()
                    .enumerate()
                    .map(|(i, &f)| format!("motor[{i}]={f}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        });
    } else if total_sat > 20 {
        diagnostics.push(Diagnostic {
            severity: Severity::Info,
            category: "motors",
            message: "Minor motor saturation detected".into(),
            detail: format!(
                "Saturated frames: {}. Brief saturation during aggressive maneuvers is normal.",
                motor_sat_frames
                    .iter()
                    .enumerate()
                    .map(|(i, &f)| format!("motor[{i}]={f}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        });
    }
}

fn diagnose_overshoot(
    events: &[FlightEvent],
    duration_seconds: f64,
    diagnostics: &mut Vec<Diagnostic>,
) {
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
}

fn diagnose_gyro_spikes(events: &[FlightEvent], diagnostics: &mut Vec<Diagnostic>) {
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
}

fn diagnose_vibration(vib: &VibrationAnalysis, diagnostics: &mut Vec<Diagnostic>) {
    let axis_names = ["roll", "pitch", "yaw"];

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
}

fn diagnose_throttle_response(vib: &VibrationAnalysis, diagnostics: &mut Vec<Diagnostic>) {
    if vib.throttle_bands.len() < 2 {
        return;
    }

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
}

fn diagnose_fc_mounting(vib: &VibrationAnalysis, diagnostics: &mut Vec<Diagnostic>) {
    let Some(accel) = &vib.accel else {
        return;
    };

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
}
