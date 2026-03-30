use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;

use crate::format::bf::types::BfRawSession;

#[derive(Debug, Clone, Serialize)]
pub struct FrequencySpectrum {
    pub axis: &'static str,
    pub sample_rate_hz: f64,
    pub frequencies_hz: Vec<f64>,
    pub magnitudes_db: Vec<f64>,
    pub peaks: Vec<FrequencyPeak>,
}

#[derive(Debug, Clone, Serialize)]
pub struct FrequencyPeak {
    pub frequency_hz: f64,
    pub magnitude_db: f64,
    pub rank: usize,
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
pub struct VibrationAnalysis {
    pub spectra: Vec<FrequencySpectrum>,
    pub noise_floor_db: [f64; 3],
    pub throttle_bands: Vec<ThrottleBand>,
}

const FFT_WINDOW_SIZE: usize = 1024;
const FFT_OVERLAP: usize = 512;
const PEAK_COUNT: usize = 5;
const MIN_FREQ_HZ: f64 = 10.0;

/// Runs FFT vibration analysis on gyro data from a Betaflight session.
#[allow(clippy::cast_precision_loss)]
pub fn analyze_vibration(session: &BfRawSession, sample_rate: f64) -> VibrationAnalysis {
    let axis_names = ["roll", "pitch", "yaw"];
    let gyro_indices = [
        session.main_field_defs.index_of("gyroADC[0]"),
        session.main_field_defs.index_of("gyroADC[1]"),
        session.main_field_defs.index_of("gyroADC[2]"),
    ];

    let mut spectra = Vec::new();
    let mut noise_floor_db = [0.0; 3];

    for (axis_idx, gyro_idx) in gyro_indices.iter().enumerate() {
        let Some(idx) = *gyro_idx else {
            continue;
        };

        let samples: Vec<f64> = session
            .frames
            .iter()
            .map(|f| f.values.get(idx).copied().unwrap_or(0) as f64)
            .collect();

        if samples.len() < FFT_WINDOW_SIZE {
            continue;
        }

        let spectrum = compute_spectrum(&samples, sample_rate, axis_names[axis_idx]);
        noise_floor_db[axis_idx] = compute_noise_floor(&spectrum.magnitudes_db);
        spectra.push(spectrum);
    }

    let throttle_bands = compute_throttle_bands(session, sample_rate, &gyro_indices, &axis_names);

    VibrationAnalysis {
        spectra,
        noise_floor_db,
        throttle_bands,
    }
}

const THROTTLE_BANDS: [(f64, f64, &str); 4] = [
    (0.0, 25.0, "0-25%"),
    (25.0, 50.0, "25-50%"),
    (50.0, 75.0, "50-75%"),
    (75.0, 100.0, "75-100%"),
];

#[allow(clippy::cast_precision_loss)]
fn compute_throttle_bands(
    session: &BfRawSession,
    sample_rate: f64,
    gyro_indices: &[Option<usize>; 3],
    axis_names: &[&'static str; 3],
) -> Vec<ThrottleBand> {
    let Some(throttle_idx) = session.main_field_defs.index_of("rcCommand[3]") else {
        return Vec::new();
    };

    let mut bands = Vec::new();

    for &(t_min, t_max, label) in &THROTTLE_BANDS {
        let band_frames: Vec<usize> = session
            .frames
            .iter()
            .enumerate()
            .filter(|(_, f)| {
                let throttle = f.values.get(throttle_idx).copied().unwrap_or(0);
                let pct = (throttle - 1000) as f64 / 10.0;
                pct >= t_min && pct < t_max
            })
            .map(|(i, _)| i)
            .collect();

        if band_frames.len() < FFT_WINDOW_SIZE {
            continue;
        }

        let mut spectra = Vec::new();
        for (axis_idx, gyro_idx) in gyro_indices.iter().enumerate() {
            let Some(idx) = *gyro_idx else {
                continue;
            };

            let samples: Vec<f64> = band_frames
                .iter()
                .map(|&i| session.frames[i].values.get(idx).copied().unwrap_or(0) as f64)
                .collect();

            if samples.len() >= FFT_WINDOW_SIZE {
                spectra.push(compute_spectrum(
                    &samples,
                    sample_rate,
                    axis_names[axis_idx],
                ));
            }
        }

        bands.push(ThrottleBand {
            label: label.to_string(),
            throttle_min: t_min,
            throttle_max: t_max,
            frame_count: band_frames.len(),
            spectra,
        });
    }

    bands
}

#[allow(clippy::cast_precision_loss)]
fn compute_spectrum(samples: &[f64], sample_rate: f64, axis: &'static str) -> FrequencySpectrum {
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
        })
        .collect()
}

fn compute_noise_floor(magnitudes_db: &[f64]) -> f64 {
    if magnitudes_db.is_empty() {
        return -120.0;
    }
    let mut sorted = magnitudes_db.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    sorted[sorted.len() / 2]
}
