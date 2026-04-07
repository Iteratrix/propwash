use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;

use crate::types::{Axis, RcChannel, SensorField, Session};

#[derive(Debug, Clone, Serialize)]
pub struct FrequencySpectrum {
    pub axis: &'static str,
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
}

#[derive(Debug, Clone, Serialize)]
pub struct AccelVibration {
    pub rms: [f64; 3],
    pub spectra: Vec<FrequencySpectrum>,
}

#[derive(Debug, Clone, Serialize)]
pub struct VibrationAnalysis {
    pub spectra: Vec<FrequencySpectrum>,
    pub noise_floor_db: [f64; 3],
    pub throttle_bands: Vec<ThrottleBand>,
    pub accel: Option<AccelVibration>,
}

const FFT_WINDOW_SIZE: usize = 1024;
const FFT_OVERLAP: usize = 512;
const PEAK_COUNT: usize = 5;
const MIN_FREQ_HZ: f64 = 10.0;

const THROTTLE_BANDS: [(f64, f64, &str); 4] = [
    (0.0, 25.0, "0-25%"),
    (25.0, 50.0, "25-50%"),
    (50.0, 75.0, "50-75%"),
    (75.0, 100.0, "75-100%"),
];

#[allow(clippy::cast_precision_loss)]
pub(crate) fn compute_spectrum_from_samples(
    samples: &[f64],
    sample_rate: f64,
    axis: &'static str,
) -> FrequencySpectrum {
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(FFT_WINDOW_SIZE);

    let hann = hann_window(FFT_WINDOW_SIZE);
    let n_windows = (samples.len() - FFT_WINDOW_SIZE) / (FFT_WINDOW_SIZE - FFT_OVERLAP) + 1;
    let n_bins = FFT_WINDOW_SIZE / 2;

    let mut avg_magnitudes = vec![0.0_f64; n_bins];

    for w in 0..n_windows {
        let start = w * (FFT_WINDOW_SIZE - FFT_OVERLAP);
        let window_samples = &samples[start..start + FFT_WINDOW_SIZE];

        let mut buffer: Vec<Complex<f64>> = window_samples
            .iter()
            .zip(hann.iter())
            .map(|(&s, &h)| Complex::new(s * h, 0.0))
            .collect();

        fft.process(&mut buffer);

        for (i, mag) in avg_magnitudes.iter_mut().enumerate() {
            let c = buffer[i];
            *mag += (c.re * c.re + c.im * c.im).sqrt();
        }
    }

    let scale = 1.0 / n_windows as f64;
    for mag in &mut avg_magnitudes {
        *mag *= scale;
    }

    let freq_resolution = sample_rate / FFT_WINDOW_SIZE as f64;
    let frequencies_hz: Vec<f64> = (0..n_bins).map(|i| i as f64 * freq_resolution).collect();

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

#[allow(clippy::cast_precision_loss)]
fn hann_window(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| {
            let x = std::f64::consts::PI * 2.0 * i as f64 / (size - 1) as f64;
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

    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
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
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    sorted[sorted.len() / 2]
}

/// Format-agnostic vibration analysis using the `Unified` trait.
/// Includes throttle-banded FFT and accelerometer analysis.
#[allow(clippy::cast_precision_loss)]
pub fn analyze_vibration_unified(unified: &Session) -> Option<VibrationAnalysis> {
    let sample_rate = unified.sample_rate_hz();
    if sample_rate <= 0.0 {
        return None;
    }

    let axis_names = ["roll", "pitch", "yaw"];
    let mut spectra = Vec::new();

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let gyro = unified.field(&SensorField::Gyro(*axis));
        if gyro.len() >= FFT_WINDOW_SIZE {
            spectra.push(compute_spectrum_from_samples(
                &gyro,
                sample_rate,
                axis_names[i],
            ));
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

    let throttle_bands = compute_throttle_bands_unified(unified, sample_rate);
    let accel = analyze_accel_unified(unified, sample_rate);

    classify_peaks(&mut spectra, &throttle_bands);

    Some(VibrationAnalysis {
        spectra,
        noise_floor_db,
        throttle_bands,
        accel,
    })
}

#[allow(clippy::cast_precision_loss)]
fn compute_throttle_bands_unified(unified: &Session, sample_rate: f64) -> Vec<ThrottleBand> {
    let throttle = unified.field(&SensorField::Rc(RcChannel::Throttle));
    if throttle.is_empty() {
        return Vec::new();
    }

    // Normalize throttle to percentage using data range
    let t_min_val = throttle.iter().copied().fold(f64::MAX, f64::min);
    let t_max_val = throttle.iter().copied().fold(f64::MIN, f64::max);
    let t_range = (t_max_val - t_min_val).max(1.0);
    let throttle_pct: Vec<f64> = throttle
        .iter()
        .map(|&v| (v - t_min_val) / t_range * 100.0)
        .collect();

    let axis_names: [&str; 3] = ["roll", "pitch", "yaw"];
    let gyro_data: Vec<Vec<f64>> = Axis::ALL
        .iter()
        .map(|a| unified.field(&SensorField::Gyro(*a)))
        .collect();

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
                    axis_names[axis_idx],
                ));
            }
        }

        bands.push(ThrottleBand {
            label: label.to_string(),
            throttle_min: band_min,
            throttle_max: band_max,
            frame_count: band_indices.len(),
            spectra: band_spectra,
        });
    }

    bands
}

#[allow(clippy::cast_precision_loss)]
fn analyze_accel_unified(unified: &Session, sample_rate: f64) -> Option<AccelVibration> {
    let accel_names = ["X", "Y", "Z"];
    let mut rms = [0.0; 3];
    let mut spectra = Vec::new();
    let mut has_data = false;

    for (i, axis) in Axis::ALL.iter().enumerate() {
        let samples = unified.field(&SensorField::Accel(*axis));
        if samples.is_empty() {
            continue;
        }
        has_data = true;

        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let ac_coupled: Vec<f64> = samples.iter().map(|&s| s - mean).collect();

        let variance = ac_coupled.iter().map(|v| v * v).sum::<f64>() / ac_coupled.len() as f64;
        rms[i] = variance.sqrt();

        if ac_coupled.len() >= FFT_WINDOW_SIZE {
            spectra.push(compute_spectrum_from_samples(
                &ac_coupled,
                sample_rate,
                accel_names[i],
            ));
        }
    }

    if has_data {
        Some(AccelVibration { rms, spectra })
    } else {
        None
    }
}
