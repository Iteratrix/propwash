use az::{Az, SaturatingAs};
use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;

use super::events::{EventKind, FlightEvent};
use crate::session::SampleBatch;
use crate::types::{Axis, Session};
use crate::units::DegPerSec;

#[derive(Debug, Clone, Serialize)]
pub struct FrequencySpectrum {
    pub axis: Axis,
    pub sample_rate_hz: f64,
    pub frequencies_hz: Vec<f64>,
    pub magnitudes_db: Vec<f64>,
    pub peaks: Vec<FrequencyPeak>,
}

#[derive(Debug, Clone, Serialize)]
pub enum NoiseClass {
    MotorNoise,
    FrameResonance,
    Unknown,
}

#[derive(Debug, Clone, Serialize)]
pub struct FrequencyPeak {
    pub frequency_hz: f64,
    pub magnitude_db: f64,
    pub rank: usize,
    pub classification: Option<NoiseClass>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ThrottleBand {
    pub label: String,
    pub throttle_min: f64,
    pub throttle_max: f64,
    pub frame_count: usize,
    pub spectra: Vec<FrequencySpectrum>,
    /// Average motor RPM frequency (Hz = RPM/60) in this throttle band, if eRPM data is available.
    pub avg_motor_hz: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct AccelVibration {
    pub rms: [f64; 3],
    pub spectra: Vec<FrequencySpectrum>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PropwashAnalysis {
    /// Per-axis FFT of gyro data in the recovery windows after throttle chops.
    pub spectra: Vec<FrequencySpectrum>,
    /// Number of throttle chop events used.
    pub chop_count: usize,
    /// Dominant propwash frequency (Hz) across all axes, if detected.
    pub dominant_frequency_hz: Option<f64>,
    /// Magnitude of the dominant propwash peak (dB).
    pub dominant_magnitude_db: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct VibrationAnalysis {
    pub spectra: Vec<FrequencySpectrum>,
    pub noise_floor_db: [f64; 3],
    pub throttle_bands: Vec<ThrottleBand>,
    /// Average motor RPM frequency (Hz) across the full flight, if eRPM data is available.
    pub avg_motor_hz: Option<f64>,
    pub accel: Option<AccelVibration>,
    pub propwash: Option<PropwashAnalysis>,
}

impl VibrationAnalysis {
    /// Gyro sample rate the spectra were computed at, if any were.
    pub fn sample_rate_hz(&self) -> Option<f64> {
        self.spectra.first().map(|s| s.sample_rate_hz)
    }
}

const FFT_WINDOW_SIZE: usize = 1024;
const FFT_OVERLAP: usize = 512;
const PEAK_COUNT: usize = 5;
const MIN_FREQ_HZ: f64 = 10.0;

const SPEC_WINDOW: usize = 512;
const SPEC_OVERLAP: usize = 384;
const SPEC_MAX_FREQ: f64 = 1000.0;

const THROTTLE_BANDS: [(f64, f64, &str); 4] = [
    (0.0, 25.0, "0-25%"),
    (25.0, 50.0, "25-50%"),
    (50.0, 75.0, "50-75%"),
    (75.0, 100.0, "75-100%"),
];

pub fn compute_spectrum_from_samples(
    samples: &[f64],
    sample_rate: f64,
    axis: Axis,
) -> FrequencySpectrum {
    compute_spectrum_from_segments(&[samples], sample_rate, axis)
}

/// Welch-averaged magnitude spectrum over every full window inside each
/// segment. Windows never span two segments, so non-contiguous bursts
/// (`ArduPilot`'s batch sampler) average without seams. Segments shorter
/// than one window contribute nothing; with no full window at all, every
/// bin reads -120 dB.
pub fn compute_spectrum_from_segments(
    segments: &[&[f64]],
    sample_rate: f64,
    axis: Axis,
) -> FrequencySpectrum {
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(FFT_WINDOW_SIZE);

    let hann = hann_window(FFT_WINDOW_SIZE);
    let n_bins = FFT_WINDOW_SIZE / 2;

    let mut avg_magnitudes = vec![0.0_f64; n_bins];
    let mut n_windows = 0_usize;

    for segment in segments.iter().filter(|s| s.len() >= FFT_WINDOW_SIZE) {
        for start in (0..=segment.len() - FFT_WINDOW_SIZE).step_by(FFT_WINDOW_SIZE - FFT_OVERLAP) {
            let window_samples = &segment[start..start + FFT_WINDOW_SIZE];

            let mut buffer: Vec<Complex<f64>> = window_samples
                .iter()
                .zip(hann.iter())
                .map(|(&s, &h)| Complex::new(s * h, 0.0))
                .collect();

            fft.process(&mut buffer);

            for (i, mag) in avg_magnitudes.iter_mut().enumerate() {
                let c = buffer[i];
                *mag += c.re.hypot(c.im);
            }
            n_windows += 1;
        }
    }

    if n_windows > 0 {
        let scale = 1.0 / n_windows.az::<f64>();
        for mag in &mut avg_magnitudes {
            *mag *= scale;
        }
    }

    let freq_resolution = sample_rate / FFT_WINDOW_SIZE.az::<f64>();
    let frequencies_hz: Vec<f64> = (0..n_bins)
        .map(|i| i.az::<f64>() * freq_resolution)
        .collect();

    let magnitudes_db: Vec<f64> = avg_magnitudes
        .iter()
        .map(|&m| if m > 0.0 { 20.0 * m.log10() } else { -120.0 })
        .collect();

    let peaks = find_peaks(&frequencies_hz, &magnitudes_db);

    FrequencySpectrum {
        axis,
        sample_rate_hz: sample_rate,
        frequencies_hz,
        magnitudes_db,
        peaks,
    }
}

fn hann_window(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| {
            let x = std::f64::consts::PI * 2.0 * i.az::<f64>() / (size - 1).az::<f64>();
            0.5 * (1.0 - x.cos())
        })
        .collect()
}

fn find_peaks(frequencies: &[f64], magnitudes_db: &[f64]) -> Vec<FrequencyPeak> {
    let min_bin = frequencies
        .iter()
        .position(|&f| f >= MIN_FREQ_HZ)
        .unwrap_or(1);

    let mut peaks: Vec<(usize, f64)> = Vec::new();
    for i in min_bin.max(1)..magnitudes_db.len() - 1 {
        if magnitudes_db[i] > magnitudes_db[i - 1] && magnitudes_db[i] > magnitudes_db[i + 1] {
            peaks.push((i, magnitudes_db[i]));
        }
    }

    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    peaks.truncate(PEAK_COUNT);

    peaks
        .iter()
        .enumerate()
        .map(|(rank, &(bin, db))| FrequencyPeak {
            frequency_hz: frequencies[bin],
            magnitude_db: db,
            rank: rank + 1,
            classification: None,
        })
        .collect()
}

fn classify_peaks(spectra: &mut [FrequencySpectrum], bands: &[ThrottleBand]) {
    if bands.len() < 2 {
        return;
    }

    for spectrum in spectra.iter_mut() {
        let band_peaks: Vec<Option<f64>> = bands
            .iter()
            .map(|band| {
                band.spectra
                    .iter()
                    .find(|s| s.axis == spectrum.axis)
                    .and_then(|s| s.peaks.first())
                    .map(|p| p.frequency_hz)
            })
            .collect();

        let valid_peaks: Vec<f64> = band_peaks.iter().filter_map(|p| *p).collect();
        if valid_peaks.len() < 2 {
            continue;
        }

        let min_f = valid_peaks.iter().copied().reduce(f64::min).unwrap_or(0.0);
        let max_f = valid_peaks.iter().copied().reduce(f64::max).unwrap_or(0.0);
        let shifts_with_throttle = max_f > min_f * 1.5 && min_f > 10.0;

        for peak in &mut spectrum.peaks {
            let freq = peak.frequency_hz;
            let appears_in_bands: Vec<bool> = bands
                .iter()
                .map(|band| {
                    band.spectra
                        .iter()
                        .find(|s| s.axis == spectrum.axis)
                        .is_some_and(|s| {
                            s.peaks
                                .iter()
                                .any(|p| (p.frequency_hz - freq).abs() < freq * 0.15)
                        })
                })
                .collect();

            let appears_count = appears_in_bands.iter().filter(|&&b| b).count();

            if appears_count >= bands.len() / 2 && !shifts_with_throttle {
                peak.classification = Some(NoiseClass::FrameResonance);
            } else if shifts_with_throttle && freq > 50.0 {
                peak.classification = Some(NoiseClass::MotorNoise);
            }
        }
    }
}

fn compute_noise_floor(magnitudes_db: &[f64]) -> f64 {
    if magnitudes_db.is_empty() {
        return -120.0;
    }
    let mut sorted = magnitudes_db.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    sorted[sorted.len() / 2]
}

/// Format-agnostic vibration analysis using the `Unified` trait.
/// Includes throttle-banded FFT, accelerometer analysis, and propwash analysis.
pub fn analyze_vibration_unified(
    unified: &Session,
    events: &[FlightEvent],
) -> Option<VibrationAnalysis> {
    let sample_rate = unified.sample_rate_hz();
    if sample_rate <= 0.0 {
        return None;
    }

    // Get motor pole count for eRPM → frequency conversion.
    // BF logs surface motor pole count via the `motor_poles` header.
    // AP/PX4/MAVLink don't expose it; default to 14 (3-pole-pair 5"
    // brushless, the common build) so eRPM→Hz still produces a
    // reasonable number on those formats.
    let motor_poles: u32 = unified.meta.motor_poles.unwrap_or(14);

    let full_rate = full_rate_gyro(unified);
    let mut spectra = Vec::new();

    for axis in Axis::ALL {
        if let Some((bursts, rate)) = full_rate {
            let segments = burst_segments(bursts.iter(), axis);
            if segments.iter().any(|s| s.len() >= FFT_WINDOW_SIZE) {
                spectra.push(compute_spectrum_from_segments(&segments, rate, axis));
            }
        } else {
            let gyro: &[f64] = bytemuck::cast_slice(unified.gyro.values.get(axis).as_slice());
            if gyro.len() >= FFT_WINDOW_SIZE {
                spectra.push(compute_spectrum_from_samples(gyro, sample_rate, axis));
            }
        }
    }

    if spectra.is_empty() {
        return None;
    }

    let noise_floor_db = std::array::from_fn(|i| {
        spectra
            .get(i)
            .map_or(-120.0, |s| compute_noise_floor(&s.magnitudes_db))
    });

    let erpm_data = erpm_on_gyro_axis(unified);
    let throttle_bands = match full_rate {
        Some((bursts, rate)) => {
            compute_throttle_bands_from_bursts(unified, bursts, rate, &erpm_data, motor_poles)
        }
        None => compute_throttle_bands_unified(unified, sample_rate, &erpm_data, motor_poles),
    };
    let accel = analyze_accel_unified(unified, sample_rate);

    classify_peaks(&mut spectra, &throttle_bands);

    let propwash = analyze_propwash(unified, events, sample_rate);

    // eRPM is held onto the gyro axis, so gyro frame indices address it.
    let all_indices: Vec<usize> = (0..unified.frame_count()).collect();
    let avg_motor_hz = if erpm_data.iter().any(|e| !e.is_empty()) {
        compute_avg_motor_hz(&erpm_data, &all_indices, motor_poles)
    } else {
        None
    };

    Some(VibrationAnalysis {
        spectra,
        noise_floor_db,
        throttle_bands,
        avg_motor_hz,
        accel,
        propwash,
    })
}

/// The full-rate gyro bursts and their shared rate, when the log has a burst
/// long enough for one FFT window (`ArduPilot`'s batch sampler).
fn full_rate_gyro(session: &Session) -> Option<(&[SampleBatch<DegPerSec>], f64)> {
    let bursts = session.gyro_batches.as_slice();
    let rate = bursts
        .iter()
        .find(|b| b.len() >= FFT_WINDOW_SIZE)?
        .sample_rate_hz;
    (rate > 0.0).then_some((bursts, rate))
}

/// One axis of each burst, as FFT segments.
fn burst_segments<'a>(
    bursts: impl Iterator<Item = &'a SampleBatch<DegPerSec>>,
    axis: Axis,
) -> Vec<&'a [f64]> {
    bursts
        .map(|b| bytemuck::cast_slice::<DegPerSec, f64>(b.values.get(axis).as_slice()))
        .collect()
}

/// eRPM per motor, held onto the main gyro time axis so that gyro sample
/// indices also index eRPM. Motors without telemetry yield empty columns.
fn erpm_on_gyro_axis(session: &Session) -> Vec<Vec<f64>> {
    let Some(esc) = session.motors.esc.as_ref() else {
        return Vec::new();
    };
    esc.erpm
        .iter()
        .map(|col| {
            if col.is_empty() {
                return Vec::new();
            }
            let view = super::util::TimeSeriesView {
                time_us: &esc.time_us,
                values: col.as_slice(),
            };
            super::util::resample_zoh_view(view, &session.gyro.time_us)
                .into_iter()
                .map(|r| f64::from(r.0))
                .collect()
        })
        .collect()
}

/// Throttle as a percentage above the flight's lowest throttle. Throttle is
/// already normalized to 0-1, so flooring the range at 1.0 keeps the scale
/// absolute (a flight spanning 30-60% stays 0-30, not 0-100).
fn throttle_percent(throttle: &[f64]) -> Vec<f64> {
    let t_min = throttle.iter().copied().fold(f64::MAX, f64::min);
    let t_max = throttle.iter().copied().fold(f64::MIN, f64::max);
    let t_range = (t_max - t_min).max(1.0);
    throttle
        .iter()
        .map(|&v| (v - t_min) / t_range * 100.0)
        .collect()
}

/// Throttle bands from full-rate bursts. A burst lasts a fraction of a
/// second, so it joins the band of its mean throttle whole, and each band's
/// spectrum averages windows inside its own bursts only.
fn compute_throttle_bands_from_bursts(
    session: &Session,
    bursts: &[SampleBatch<DegPerSec>],
    rate: f64,
    erpm_data: &[Vec<f64>],
    motor_poles: u32,
) -> Vec<ThrottleBand> {
    let rc = &session.rc_command;
    let throttle: Vec<f64> = rc.throttle.iter().map(|n| f64::from(n.0)).collect();
    if throttle.is_empty() || throttle.len() != rc.time_us.len() {
        return Vec::new();
    }
    let pct = throttle_percent(&throttle);
    let span =
        |b: &SampleBatch<DegPerSec>| (b.start_us, b.start_us.saturating_add(b.duration_us()));
    let burst_pct: Vec<f64> = bursts
        .iter()
        .map(|b| {
            let (start, end) = span(b);
            let lo = rc.time_us.partition_point(|&t| t < start);
            let hi = rc.time_us.partition_point(|&t| t < end);
            match pct.get(lo..hi) {
                Some(window) if !window.is_empty() => {
                    window.iter().sum::<f64>() / window.len().az::<f64>()
                }
                // No RC sample inside the burst: hold the last one before it.
                _ => pct.get(lo.saturating_sub(1)).copied().unwrap_or(0.0),
            }
        })
        .collect();

    let gyro_time = &session.gyro.time_us;
    let has_erpm = erpm_data.iter().any(|e| !e.is_empty());
    let mut bands = Vec::new();

    for &(band_min, band_max, label) in &THROTTLE_BANDS {
        let members: Vec<&SampleBatch<DegPerSec>> = bursts
            .iter()
            .zip(&burst_pct)
            .filter(|(_, &p)| p >= band_min && p < band_max)
            .map(|(b, _)| b)
            .collect();
        let frame_count: usize = members.iter().map(|b| b.len()).sum();
        if frame_count < FFT_WINDOW_SIZE {
            continue;
        }

        let spectra = Axis::ALL
            .iter()
            .filter_map(|&axis| {
                let segments = burst_segments(members.iter().copied(), axis);
                segments
                    .iter()
                    .any(|s| s.len() >= FFT_WINDOW_SIZE)
                    .then(|| compute_spectrum_from_segments(&segments, rate, axis))
            })
            .collect();

        // eRPM lives on the main gyro axis: average it over the gyro
        // samples that fall inside this band's bursts.
        let avg_motor_hz = if has_erpm {
            let gyro_indices: Vec<usize> = members
                .iter()
                .flat_map(|b| {
                    let (start, end) = span(b);
                    gyro_time.partition_point(|&t| t < start)
                        ..gyro_time.partition_point(|&t| t < end)
                })
                .collect();
            compute_avg_motor_hz(erpm_data, &gyro_indices, motor_poles)
        } else {
            None
        };

        bands.push(ThrottleBand {
            label: label.to_string(),
            throttle_min: band_min,
            throttle_max: band_max,
            frame_count,
            spectra,
            avg_motor_hz,
        });
    }

    bands
}

fn compute_throttle_bands_unified(
    unified: &Session,
    sample_rate: f64,
    erpm_data: &[Vec<f64>],
    motor_poles: u32,
) -> Vec<ThrottleBand> {
    if unified.rc_command.throttle.is_empty() {
        return Vec::new();
    }
    let target_axis = &unified.gyro.time_us;
    if target_axis.is_empty() {
        return Vec::new();
    }

    // Resample raw throttle (Normalized01 on rc_command.time_us) onto
    // the gyro time axis so band-membership indices and gyro/ERPM
    // sample lookups all refer to the same moments.
    let throttle_view = super::util::TimeSeriesView {
        time_us: &unified.rc_command.time_us,
        values: unified.rc_command.throttle.as_slice(),
    };
    let throttle: Vec<f64> = super::util::resample_zoh_view(throttle_view, target_axis)
        .into_iter()
        .map(|n| f64::from(n.0))
        .collect();

    let throttle_pct = throttle_percent(&throttle);

    let gyro_data: Vec<Vec<f64>> = Axis::ALL
        .iter()
        .map(|a| {
            bytemuck::cast_slice::<DegPerSec, f64>(unified.gyro.values.get(*a).as_slice()).to_vec()
        })
        .collect();

    let has_erpm = erpm_data.iter().any(|e| !e.is_empty());

    let mut bands = Vec::new();

    for &(band_min, band_max, label) in &THROTTLE_BANDS {
        let band_indices: Vec<usize> = throttle_pct
            .iter()
            .enumerate()
            .filter(|(_, &pct)| pct >= band_min && pct < band_max)
            .map(|(i, _)| i)
            .collect();

        if band_indices.len() < FFT_WINDOW_SIZE {
            continue;
        }

        let mut band_spectra = Vec::new();
        for (axis_idx, gyro) in gyro_data.iter().enumerate() {
            if gyro.is_empty() {
                continue;
            }
            let samples: Vec<f64> = band_indices
                .iter()
                .filter_map(|&i| gyro.get(i).copied())
                .collect();

            if samples.len() >= FFT_WINDOW_SIZE {
                band_spectra.push(compute_spectrum_from_samples(
                    &samples,
                    sample_rate,
                    Axis::ALL[axis_idx],
                ));
            }
        }

        // Compute average motor RPM in this throttle band
        let avg_motor_hz = if has_erpm {
            compute_avg_motor_hz(erpm_data, &band_indices, motor_poles)
        } else {
            None
        };

        bands.push(ThrottleBand {
            label: label.to_string(),
            throttle_min: band_min,
            throttle_max: band_max,
            frame_count: band_indices.len(),
            spectra: band_spectra,
            avg_motor_hz,
        });
    }

    bands
}

/// Compute average motor vibration frequency (Hz) from eRPM data at the given frame indices.
///
/// Assumes eRPM values are already in actual electrical RPM (each format's
/// `field()` handles scaling — e.g., BF multiplies raw ×100 in `field()`).
/// Vibration frequency = eRPM / 60 / (poles/2).
fn compute_avg_motor_hz(
    erpm_data: &[Vec<f64>],
    indices: &[usize],
    motor_poles: u32,
) -> Option<f64> {
    let mut sum = 0.0f64;
    let mut count = 0usize;

    for motor in erpm_data {
        if motor.is_empty() {
            continue;
        }
        for &i in indices {
            if let Some(&erpm) = motor.get(i) {
                if erpm > 0.0 {
                    sum += erpm;
                    count += 1;
                }
            }
        }
    }

    if count > 0 {
        let pole_pairs = f64::from(motor_poles.max(2)) / 2.0;
        Some(sum / count.az::<f64>() / 60.0 / pole_pairs)
    } else {
        None
    }
}

fn analyze_accel_unified(unified: &Session, sample_rate: f64) -> Option<AccelVibration> {
    let mut rms = [0.0; 3];
    let mut spectra = Vec::new();
    let mut has_data = false;

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let samples: &[f64] = bytemuck::cast_slice(unified.accel.values.get(*axis).as_slice());
        if samples.is_empty() {
            continue;
        }
        has_data = true;

        let mean = samples.iter().sum::<f64>() / samples.len().az::<f64>();
        let ac_coupled: Vec<f64> = samples.iter().map(|&s| s - mean).collect();

        let variance = ac_coupled.iter().map(|v| v * v).sum::<f64>() / ac_coupled.len().az::<f64>();
        rms[i] = variance.sqrt();

        if ac_coupled.len() >= FFT_WINDOW_SIZE {
            spectra.push(compute_spectrum_from_samples(
                &ac_coupled,
                sample_rate,
                *axis,
            ));
        }
    }

    if has_data {
        Some(AccelVibration { rms, spectra })
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Propwash analysis — FFT of gyro in recovery windows after throttle chops
// ---------------------------------------------------------------------------

/// Duration of the post-chop recovery window in seconds.
const PROPWASH_WINDOW_SECS: f64 = 0.75;
/// Propwash oscillations typically occur in this frequency range.
const PROPWASH_MIN_HZ: f64 = 20.0;
const PROPWASH_MAX_HZ: f64 = 100.0;

#[allow(clippy::too_many_lines)]
fn analyze_propwash(
    session: &Session,
    events: &[FlightEvent],
    sample_rate: f64,
) -> Option<PropwashAnalysis> {
    let chop_times: Vec<f64> = events
        .iter()
        .filter_map(|e| match &e.kind {
            EventKind::ThrottleChop { .. } => Some(e.time_seconds),
            _ => None,
        })
        .collect();

    if chop_times.is_empty() {
        return None;
    }

    let time: Vec<f64> = session
        .gyro
        .time_us
        .iter()
        .map(|&t| t.az::<f64>())
        .collect();
    if time.is_empty() {
        return None;
    }
    let t0_us = time[0];

    let window_samples = (PROPWASH_WINDOW_SECS * sample_rate).saturating_as::<usize>();
    if window_samples < 64 {
        return None;
    }
    // Use a power-of-2 FFT size that fits within the window
    let fft_size = window_samples.next_power_of_two().min(FFT_WINDOW_SIZE);

    let gyro_data: Vec<Vec<f64>> = Axis::ALL
        .iter()
        .map(|a| {
            bytemuck::cast_slice::<DegPerSec, f64>(session.gyro.values.get(*a).as_slice()).to_vec()
        })
        .collect();

    if gyro_data.iter().all(Vec::is_empty) {
        return None;
    }

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(fft_size);
    let hann = hann_window(fft_size);
    let n_bins = fft_size / 2;
    let freq_resolution = sample_rate / fft_size.az::<f64>();
    let frequencies_hz: Vec<f64> = (0..n_bins)
        .map(|i| i.az::<f64>() * freq_resolution)
        .collect();

    let mut spectra = Vec::new();

    for (axis_idx, gyro) in gyro_data.iter().enumerate() {
        if gyro.is_empty() {
            continue;
        }

        let mut avg_magnitudes = vec![0.0_f64; n_bins];
        let mut window_count = 0usize;

        for &chop_t in &chop_times {
            // Convert chop time (seconds) to sample index
            let chop_us = chop_t.mul_add(1_000_000.0, t0_us);
            let start_idx = time.partition_point(|&t| t < chop_us);

            if start_idx + fft_size > gyro.len() {
                continue;
            }

            let mut buffer: Vec<Complex<f64>> = (0..fft_size)
                .map(|i| Complex::new(gyro[start_idx + i] * hann[i], 0.0))
                .collect();

            fft.process(&mut buffer);

            for (i, mag) in avg_magnitudes.iter_mut().enumerate() {
                let c = buffer[i];
                *mag += c.re.hypot(c.im);
            }
            window_count += 1;
        }

        if window_count == 0 {
            continue;
        }

        let scale = 1.0 / window_count.az::<f64>();
        for mag in &mut avg_magnitudes {
            *mag *= scale;
        }

        let magnitudes_db: Vec<f64> = avg_magnitudes
            .iter()
            .map(|&m| if m > 0.0 { 20.0 * m.log10() } else { -120.0 })
            .collect();

        let peaks = find_peaks(&frequencies_hz, &magnitudes_db);

        spectra.push(FrequencySpectrum {
            axis: Axis::ALL[axis_idx],
            sample_rate_hz: sample_rate,
            frequencies_hz: frequencies_hz.clone(),
            magnitudes_db,
            peaks,
        });
    }

    if spectra.is_empty() {
        return None;
    }

    // Find the dominant peak in the propwash frequency range across all axes
    let (dominant_frequency_hz, dominant_magnitude_db) = spectra
        .iter()
        .flat_map(|s| &s.peaks)
        .filter(|p| p.frequency_hz >= PROPWASH_MIN_HZ && p.frequency_hz <= PROPWASH_MAX_HZ)
        .max_by(|a, b| {
            a.magnitude_db
                .partial_cmp(&b.magnitude_db)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map_or((None, None), |p| {
            (Some(p.frequency_hz), Some(p.magnitude_db))
        });

    Some(PropwashAnalysis {
        spectra,
        chop_count: chop_times.len(),
        dominant_frequency_hz,
        dominant_magnitude_db,
    })
}

#[derive(Debug, Clone, Serialize)]
pub struct SpectrogramAxis {
    pub axis: String,
    pub time_s: Vec<f64>,
    pub frequencies_hz: Vec<f64>,
    pub magnitudes_db: Vec<Vec<f64>>,
    pub max_freq_hz: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct Spectrogram {
    pub axes: Vec<SpectrogramAxis>,
    pub sample_rate_hz: f64,
}

/// Computes a time-frequency spectrogram for the given axes.
///
/// Each entry in `axes` is `(axis_label, samples)` — the caller pulls
/// the samples out of typed Session fields (e.g. via
/// `bytemuck::cast_slice` on `session.gyro.values.roll`) and labels
/// them however it wants for the result. `sample_rate_hz` and
/// `time_us_axis` come from the same Session field the samples are
/// taken from, typically the gyro axis.
///
/// Returns `None` if `sample_rate_hz` is non-positive or no axis
/// produced enough samples for one window.
///
/// **Caveat (multi-rate sources):** for axes whose source stream
/// samples at a different rate than `time_us_axis`, the FFT magnitudes
/// are correct (each window is contiguous samples of the source) but
/// the per-window x-axis labels linearly project onto `time_us_axis`,
/// so they're approximate. Pass the same axis whose `samples` you're
/// FFT-ing to keep labels exact.
pub fn compute_spectrogram(
    sample_rate_hz: f64,
    time_us_axis: &[u64],
    axes: &[(&str, &[f64])],
) -> Option<Spectrogram> {
    if sample_rate_hz <= 0.0 {
        return None;
    }

    let freq_res = sample_rate_hz / SPEC_WINDOW.az::<f64>();
    let max_bin = (SPEC_MAX_FREQ / freq_res)
        .saturating_as::<usize>()
        .min(SPEC_WINDOW / 2);
    let frequencies_hz: Vec<f64> = (0..max_bin).map(|i| i.az::<f64>() * freq_res).collect();

    let time_raw: Vec<f64> = time_us_axis.iter().map(|&t| t.az::<f64>()).collect();
    let t0 = time_raw.first().copied().unwrap_or(0.0);

    let hann = hann_window(SPEC_WINDOW);
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(SPEC_WINDOW);

    let step = SPEC_WINDOW - SPEC_OVERLAP;
    let mut result_axes = Vec::new();

    for &(axis_name, raw) in axes {
        if raw.len() < SPEC_WINDOW {
            continue;
        }

        let n_windows = (raw.len() - SPEC_WINDOW) / step + 1;
        let mut time_s = Vec::with_capacity(n_windows);
        let mut magnitudes_db = Vec::with_capacity(n_windows);

        for w in 0..n_windows {
            let start = w * step;
            let mid = start + SPEC_WINDOW / 2;

            let t = if mid < time_raw.len() {
                (time_raw[mid] - t0) / 1_000_000.0
            } else {
                0.0
            };
            time_s.push(t);

            let mut buffer: Vec<Complex<f64>> = (0..SPEC_WINDOW)
                .map(|i| Complex::new(raw[start + i] * hann[i], 0.0))
                .collect();

            fft.process(&mut buffer);

            let row: Vec<f64> = (0..max_bin)
                .map(|i| {
                    let c = buffer[i];
                    let mag = c.re.hypot(c.im);
                    if mag > 0.0 {
                        20.0 * mag.log10()
                    } else {
                        -120.0
                    }
                })
                .collect();
            magnitudes_db.push(row);
        }

        result_axes.push(SpectrogramAxis {
            axis: axis_name.to_string(),
            time_s,
            frequencies_hz: frequencies_hz.clone(),
            magnitudes_db,
            max_freq_hz: SPEC_MAX_FREQ,
        });
    }

    if result_axes.is_empty() {
        return None;
    }

    Some(Spectrogram {
        axes: result_axes,
        sample_rate_hz,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tone split across non-contiguous bursts with phase jumps between
    /// them still peaks at its frequency: windows stay inside each burst.
    /// A segment shorter than one window is skipped, not a panic.
    #[test]
    fn segments_average_within_bursts() {
        let rate = 2000.0;
        let burst = |phase: f64| -> Vec<f64> {
            (0..FFT_WINDOW_SIZE)
                .map(|i| {
                    (2.0 * std::f64::consts::PI * 300.0 * i.az::<f64>() / rate + phase).sin()
                        * 100.0
                })
                .collect()
        };
        let (a, b, c) = (burst(0.0), burst(1.3), burst(2.9));
        let short = vec![1.0; 100];
        let spectrum = compute_spectrum_from_segments(&[&a, &short, &b, &c], rate, Axis::Roll);
        let top = spectrum
            .peaks
            .first()
            .expect("the tone should produce a peak");
        let bin = rate / FFT_WINDOW_SIZE.az::<f64>();
        assert!(
            (top.frequency_hz - 300.0).abs() <= bin,
            "expected 300 Hz, got {}",
            top.frequency_hz
        );
    }

    #[test]
    fn segments_without_a_full_window_read_as_silence() {
        let spectrum = compute_spectrum_from_segments(&[&[1.0; 10]], 1000.0, Axis::Roll);
        assert!(spectrum.magnitudes_db.iter().all(|&m| m <= -119.0));
    }

    /// Generate a pure sine wave at the given frequency and sample rate.
    fn sine_wave(freq_hz: f64, sample_rate: f64, n_samples: usize) -> Vec<f64> {
        (0..n_samples)
            .map(|i| {
                (2.0 * std::f64::consts::PI * freq_hz * i.az::<f64>() / sample_rate).sin() * 100.0
            })
            .collect()
    }

    #[test]
    fn spectrum_detects_single_sine() {
        let samples = sine_wave(200.0, 1000.0, 4096);
        let spectrum = compute_spectrum_from_samples(&samples, 1000.0, Axis::Roll);

        assert!(!spectrum.peaks.is_empty(), "should detect a peak");
        let peak = &spectrum.peaks[0];
        assert!(
            (peak.frequency_hz - 200.0).abs() < 10.0,
            "peak should be near 200 Hz, got {:.1} Hz",
            peak.frequency_hz
        );
    }

    #[test]
    fn spectrum_detects_two_sines() {
        // Two frequencies: 100 Hz and 300 Hz
        let mut samples = sine_wave(100.0, 1000.0, 4096);
        let second = sine_wave(300.0, 1000.0, 4096);
        for (i, v) in second.iter().enumerate() {
            samples[i] += v;
        }
        let spectrum = compute_spectrum_from_samples(&samples, 1000.0, Axis::Roll);

        assert!(
            spectrum.peaks.len() >= 2,
            "should detect at least 2 peaks, got {}",
            spectrum.peaks.len()
        );

        let freqs: Vec<f64> = spectrum.peaks.iter().map(|p| p.frequency_hz).collect();
        let has_100 = freqs.iter().any(|f| (f - 100.0).abs() < 10.0);
        let has_300 = freqs.iter().any(|f| (f - 300.0).abs() < 10.0);
        assert!(has_100, "should find peak near 100 Hz, got {freqs:?}");
        assert!(has_300, "should find peak near 300 Hz, got {freqs:?}");
    }

    #[test]
    fn spectrum_frequency_resolution() {
        // At 1000 Hz sample rate with 1024-point FFT, resolution is ~0.98 Hz
        let resolution = 1000.0 / FFT_WINDOW_SIZE.az::<f64>();
        assert!(
            resolution < 2.0,
            "frequency resolution should be < 2 Hz, got {resolution:.2}"
        );
    }

    #[test]
    fn noise_floor_of_sine() {
        let samples = sine_wave(200.0, 1000.0, 4096);
        let spectrum = compute_spectrum_from_samples(&samples, 1000.0, Axis::Roll);
        let floor = compute_noise_floor(&spectrum.magnitudes_db);

        // Noise floor should be well below the peak
        let peak_db = spectrum.peaks.first().map_or(0.0, |p| p.magnitude_db);
        assert!(
            peak_db - floor > 20.0,
            "peak ({peak_db:.0} dB) should be >20 dB above floor ({floor:.0} dB)"
        );
    }

    #[test]
    fn noise_floor_of_noise() {
        // White noise — noise floor should be relatively high (no clear peaks)
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        let samples: Vec<f64> = (0..4096)
            .map(|i| {
                let mut h = DefaultHasher::new();
                i.hash(&mut h);
                (h.finish().az::<f64>() / u64::MAX.az::<f64>() - 0.5) * 100.0
            })
            .collect();
        let spectrum = compute_spectrum_from_samples(&samples, 1000.0, Axis::Roll);
        let floor = compute_noise_floor(&spectrum.magnitudes_db);
        // White noise has a relatively flat spectrum — floor should be near peak
        let max_db = spectrum
            .magnitudes_db
            .iter()
            .copied()
            .fold(f64::MIN, f64::max);
        assert!(
            max_db - floor < 20.0,
            "white noise: peak ({max_db:.0} dB) should be <20 dB above floor ({floor:.0} dB)"
        );
    }

    #[test]
    fn hann_window_properties() {
        let w = hann_window(1024);
        assert_eq!(w.len(), 1024);
        // Hann window is zero at endpoints
        assert!(w[0].abs() < 1e-10, "first sample should be ~0");
        // Peak at center
        let mid = w[512];
        assert!((mid - 1.0).abs() < 0.01, "center should be ~1.0, got {mid}");
    }

    #[test]
    fn find_peaks_returns_sorted_by_magnitude() {
        let samples = sine_wave(200.0, 1000.0, 4096);
        let spectrum = compute_spectrum_from_samples(&samples, 1000.0, Axis::Roll);

        for pair in spectrum.peaks.windows(2) {
            assert!(
                pair[0].magnitude_db >= pair[1].magnitude_db,
                "peaks should be sorted by magnitude descending"
            );
        }
    }

    #[test]
    fn avg_motor_hz_basic() {
        // 4 motors at 6000 eRPM (already scaled by format's field())
        // With 14 poles (7 pole pairs): freq = 6000 / 60 / 7 = 14.3 Hz
        let erpm = vec![vec![6000.0; 100]; 4];
        let indices: Vec<usize> = (0..100).collect();
        let result = compute_avg_motor_hz(&erpm, &indices, 14);
        assert!(result.is_some());
        let hz = result.unwrap();
        assert!((hz - 14.3).abs() < 0.5, "expected ~14.3 Hz, got {hz:.1}");
    }

    #[test]
    fn avg_motor_hz_empty() {
        let erpm: Vec<Vec<f64>> = vec![vec![]; 4];
        let indices: Vec<usize> = (0..100).collect();
        let result = compute_avg_motor_hz(&erpm, &indices, 14);
        assert!(result.is_none());
    }

    #[test]
    fn avg_motor_hz_zeros_filtered() {
        // Mix of zero and non-zero eRPM — zeros should be excluded
        let mut motor = vec![0.0; 100];
        motor[50..].fill(6000.0);
        let erpm = vec![motor; 4];
        let indices: Vec<usize> = (0..100).collect();
        let result = compute_avg_motor_hz(&erpm, &indices, 14);
        assert!(result.is_some());
        let hz = result.unwrap();
        // Only non-zero samples contribute
        assert!(
            (hz - 14.3).abs() < 0.5,
            "should be ~14.3 Hz (ignoring zeros), got {hz:.1}"
        );
    }

    /// `bug_005` regression: `compute_throttle_bands_unified` must classify
    /// gyro samples by their *real-time-correlated* throttle, not by
    /// indexing throttle and gyro at the same array position. A Session
    /// where throttle is at 50 Hz and gyro is at 4 kHz exercises the
    /// resample path; if the path regressed to direct indexing, the
    /// throttle would drive band selection on only the first ~50 of
    /// the 4000 gyro samples and the rest would all classify as the
    /// last-known band — yielding wildly skewed `frame_count`s.
    #[test]
    fn throttle_bands_use_resampled_throttle_axis() {
        use crate::session::{Format, RcCommand, Session, SessionMeta, TriaxialSeries};
        use crate::units::{DegPerSec, Normalized01};

        // 1 second @ 4 kHz gyro, 50 Hz throttle.
        let gyro_n = 4_000usize;
        let throttle_n = 50usize;
        let gyro_time: Vec<u64> = (0..gyro_n).map(|i| (i * 250) as u64).collect();
        let throttle_time: Vec<u64> = (0..throttle_n).map(|i| (i * 20_000) as u64).collect();

        // Throttle ramp: 0%-50% over the first half, 50%-100% over the
        // second half. After resample-to-gyro, the first 2000 gyro
        // samples should fall in 0-25%/25-50% bands and the second 2000
        // in 50-75%/75-100% bands. Direct-indexing regression would
        // flatten this to throttle_n=50 samples and leave the remaining
        // ~3950 gyro samples mis-classified.
        let throttle: Vec<Normalized01> = (0..throttle_n)
            .map(|i| Normalized01(i.az::<f32>() / (throttle_n.az::<f32>() - 1.0)))
            .collect();

        let mut s = Session {
            meta: SessionMeta {
                format: Format::ArduPilot,
                ..SessionMeta::default()
            },
            ..Session::default()
        };
        // Gyro: a sine wave on roll so the FFT actually has content
        // (compute_throttle_bands_unified internally calls compute_spectrum
        // which needs ≥ FFT_WINDOW_SIZE samples and non-zero magnitudes).
        let sine: Vec<DegPerSec> = (0..gyro_n)
            .map(|i| {
                let v =
                    (2.0 * std::f64::consts::PI * 200.0 * i.az::<f64>() / 4_000.0).sin() * 100.0;
                DegPerSec(v)
            })
            .collect();
        s.gyro = TriaxialSeries {
            time_us: gyro_time,
            values: crate::session::Triaxial::new(
                sine.clone(),
                vec![DegPerSec(0.0); gyro_n],
                vec![DegPerSec(0.0); gyro_n],
            ),
        };
        s.rc_command = RcCommand {
            time_us: throttle_time,
            sticks: crate::session::Triaxial::new(vec![], vec![], vec![]),
            throttle,
        };

        let erpm = super::erpm_on_gyro_axis(&s);
        let bands = super::compute_throttle_bands_unified(&s, 4_000.0, &erpm, 14);
        // Each band's frame_count must reflect membership of gyro
        // samples (which span the full ~4000), not the raw 50
        // throttle samples. Sum across bands ≥ FFT_WINDOW_SIZE means
        // the resample expanded throttle correctly onto gyro time.
        let total: usize = bands.iter().map(|b| b.frame_count).sum();
        assert!(
            total >= 1024,
            "expected ≥1024 gyro samples binned across throttle bands; \
             got {total} (likely regressed to direct-index throttle)"
        );
    }
}
