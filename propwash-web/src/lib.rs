use std::cell::RefCell;
use std::collections::HashMap;

use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;
use wasm_bindgen::prelude::*;

use propwash_core::analysis::{self, FlightAnalysis};
use propwash_core::types::{Log, RawSession, Session};

thread_local! {
    static CURRENT_LOG: RefCell<Option<Log>> = const { RefCell::new(None) };
}

#[derive(Serialize)]
struct AnalysisResult {
    sessions: Vec<SessionResult>,
    warnings: Vec<String>,
}

#[derive(Serialize)]
struct SessionResult {
    index: usize,
    firmware: String,
    craft: String,
    duration_seconds: f64,
    sample_rate_hz: f64,
    frame_count: usize,
    analysis: FlightAnalysis,
}

#[derive(Serialize)]
struct TimeseriesResult {
    time_s: Vec<f64>,
    fields: HashMap<String, Vec<f64>>,
    sample_rate_hz: f64,
    total_frames: usize,
    decimation: usize,
}

#[derive(Serialize)]
struct SpectrogramResult {
    axis: String,
    time_s: Vec<f64>,
    frequencies_hz: Vec<f64>,
    magnitudes_db: Vec<Vec<f64>>,
    max_freq_hz: f64,
}

#[derive(Serialize)]
struct SpectrogramResponse {
    axes: Vec<SpectrogramResult>,
    sample_rate_hz: f64,
}

#[derive(Serialize)]
#[allow(clippy::struct_field_names)]
struct FilterConfig {
    gyro_lpf_hz: Option<f64>,
    gyro_lpf2_hz: Option<f64>,
    dterm_lpf_hz: Option<f64>,
    dyn_notch_min_hz: Option<f64>,
    dyn_notch_max_hz: Option<f64>,
    gyro_notch1_hz: Option<f64>,
    gyro_notch2_hz: Option<f64>,
}

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn analyze(data: &[u8]) -> String {
    let log = match propwash_core::decode(data) {
        Ok(log) => log,
        Err(e) => {
            return serde_json::to_string(&AnalysisResult {
                sessions: Vec::new(),
                warnings: vec![e.to_string()],
            })
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#));
        }
    };

    let mut sessions = Vec::new();
    for session in &log.sessions {
        let flight_analysis = analysis::analyze(session);

        sessions.push(SessionResult {
            index: session.index(),
            firmware: session.firmware_version().to_string(),
            craft: session.craft_name().to_string(),
            duration_seconds: session.duration_seconds(),
            sample_rate_hz: session.sample_rate_hz(),
            frame_count: session.frame_count(),
            analysis: flight_analysis,
        });
    }

    let mut warnings = Vec::new();
    for w in &log.warnings {
        warnings.push(w.to_string());
    }

    let result = AnalysisResult { sessions, warnings };

    CURRENT_LOG.with(|cell| {
        *cell.borrow_mut() = Some(log);
    });

    serde_json::to_string(&result)
        .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
}

#[wasm_bindgen]
#[allow(clippy::cast_precision_loss)]
pub fn get_timeseries(session_idx: usize, max_points: usize, field_list: &str) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let total_frames = session.frame_count();
        let sample_rate = session.sample_rate_hz();

        let step = if total_frames > max_points {
            total_frames / max_points
        } else {
            1
        };

        let requested: Vec<&str> = field_list.split(',').collect();

        let mut fields: HashMap<String, Vec<f64>> = HashMap::new();

        for &name in &requested {
            let raw = session.field_by_name(name);
            let decimated: Vec<f64> = raw.iter().step_by(step).copied().collect();
            fields.insert(name.to_string(), decimated);
        }

        let time_raw = session.field_by_name("time");
        let t0 = time_raw.first().copied().unwrap_or(0.0);
        let time_s: Vec<f64> = time_raw
            .iter()
            .step_by(step)
            .map(|&v| (v - t0) / 1_000_000.0)
            .collect();

        let result = TimeseriesResult {
            time_s,
            fields,
            sample_rate_hz: sample_rate,
            total_frames,
            decimation: step,
        };

        serde_json::to_string(&result)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

const SPEC_WINDOW: usize = 512;
const SPEC_OVERLAP: usize = 384;
const SPEC_MAX_FREQ: f64 = 1000.0;

#[wasm_bindgen]
#[allow(clippy::cast_precision_loss)]
pub fn get_spectrogram(session_idx: usize, axis_list: &str) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let sample_rate = session.sample_rate_hz();
        if sample_rate <= 0.0 {
            return r#"{"error":"no sample rate"}"#.to_string();
        }

        let freq_res = sample_rate / SPEC_WINDOW as f64;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let max_bin = ((SPEC_MAX_FREQ / freq_res) as usize).min(SPEC_WINDOW / 2);
        let frequencies_hz: Vec<f64> = (0..max_bin).map(|i| i as f64 * freq_res).collect();

        let time_raw = session.field_by_name("time");
        let t0 = time_raw.first().copied().unwrap_or(0.0);

        let hann = hann_window(SPEC_WINDOW);
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(SPEC_WINDOW);

        let axis_fields: Vec<(&str, &str)> = axis_list
            .split(',')
            .map(|a| {
                let field = match a {
                    "roll" => "gyroADC[0]",
                    "pitch" => "gyroADC[1]",
                    "yaw" => "gyroADC[2]",
                    other => other,
                };
                (a, field)
            })
            .collect();

        let step = SPEC_WINDOW - SPEC_OVERLAP;
        let mut axes = Vec::new();

        for &(axis_name, field_name) in &axis_fields {
            let raw = session.field_by_name(field_name);
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
                        let mag = (c.re * c.re + c.im * c.im).sqrt();
                        if mag > 0.0 {
                            20.0 * mag.log10()
                        } else {
                            -120.0
                        }
                    })
                    .collect();
                magnitudes_db.push(row);
            }

            axes.push(SpectrogramResult {
                axis: axis_name.to_string(),
                time_s,
                frequencies_hz: frequencies_hz.clone(),
                magnitudes_db,
                max_freq_hz: SPEC_MAX_FREQ,
            });
        }

        let response = SpectrogramResponse {
            axes,
            sample_rate_hz: sample_rate,
        };

        serde_json::to_string(&response)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

#[wasm_bindgen]
pub fn get_filter_config(session_idx: usize) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let non_zero_f64 = |v: f64| -> Option<f64> {
            if v > 0.0 {
                Some(v)
            } else {
                None
            }
        };

        let config = match session {
            RawSession::Betaflight(bf) => {
                let non_zero = |v: i32| -> Option<f64> {
                    if v > 0 {
                        Some(f64::from(v))
                    } else {
                        None
                    }
                };
                FilterConfig {
                    gyro_lpf_hz: non_zero(bf.get_header_int("gyro_lowpass_hz", 0)),
                    gyro_lpf2_hz: non_zero(bf.get_header_int("gyro_lowpass2_hz", 0)),
                    dterm_lpf_hz: non_zero(bf.get_header_int("dterm_lpf_hz", 0))
                        .or_else(|| non_zero(bf.get_header_int("dterm_lowpass_hz", 0))),
                    dyn_notch_min_hz: non_zero(bf.get_header_int("dyn_notch_min_hz", 0)),
                    dyn_notch_max_hz: non_zero(bf.get_header_int("dyn_notch_max_hz", 0)),
                    gyro_notch1_hz: non_zero(bf.get_header_int("gyro_notch_hz", 0)),
                    gyro_notch2_hz: non_zero(bf.get_header_int("gyro_notch2_hz", 0)),
                }
            }
            RawSession::ArduPilot(ap) => {
                let p = |k: &str| ap.params.get(k).copied().unwrap_or(0.0);
                FilterConfig {
                    gyro_lpf_hz: non_zero_f64(p("INS_GYRO_FILTER")),
                    gyro_lpf2_hz: None,
                    dterm_lpf_hz: non_zero_f64(p("ATC_RAT_RLL_FLTE")),
                    dyn_notch_min_hz: if p("INS_HNTCH_ENABLE") > 0.0 {
                        non_zero_f64(p("INS_HNTCH_FREQ"))
                    } else {
                        None
                    },
                    dyn_notch_max_hz: None,
                    gyro_notch1_hz: if p("INS_NOTCH_ENABLE") > 0.0 {
                        non_zero_f64(p("INS_NOTCH_FREQ"))
                    } else {
                        None
                    },
                    gyro_notch2_hz: if p("INS_NOTC2_ENABLE") > 0.0 {
                        non_zero_f64(p("INS_NOTC2_FREQ"))
                    } else {
                        None
                    },
                }
            }
            RawSession::Px4(px4) => {
                let p = |k: &str| px4.params.get(k).copied().unwrap_or(0.0);
                FilterConfig {
                    gyro_lpf_hz: non_zero_f64(p("IMU_GYRO_CUTOFF")),
                    gyro_lpf2_hz: None,
                    dterm_lpf_hz: non_zero_f64(p("IMU_DGYRO_CUTOFF")),
                    dyn_notch_min_hz: None,
                    dyn_notch_max_hz: None,
                    gyro_notch1_hz: non_zero_f64(p("IMU_GYRO_NF0_FRQ")),
                    gyro_notch2_hz: non_zero_f64(p("IMU_GYRO_NF1_FRQ")),
                }
            }
        };

        serde_json::to_string(&config)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
}

#[derive(Serialize)]
struct RawFramesResult {
    field_names: Vec<String>,
    frames: Vec<Vec<f64>>,
    start: usize,
    total: usize,
}

#[wasm_bindgen]
pub fn get_raw_frames(session_idx: usize, start: usize, count: usize, field_list: &str) -> String {
    CURRENT_LOG.with(|cell| {
        let borrow = cell.borrow();
        let Some(log) = borrow.as_ref() else {
            return r#"{"error":"no log loaded"}"#.to_string();
        };

        let Some(session) = log.sessions.get(session_idx) else {
            return r#"{"error":"invalid session index"}"#.to_string();
        };

        let total = session.frame_count();
        let requested: Vec<&str> = field_list.split(',').collect();

        let end = (start + count).min(total);
        let mut frames = Vec::with_capacity(end - start);

        for frame_idx in start..end {
            let mut row = Vec::with_capacity(requested.len());
            for &name in &requested {
                let field_data = session.field_by_name(name);
                row.push(field_data.get(frame_idx).copied().unwrap_or(0.0));
            }
            frames.push(row);
        }

        let result = RawFramesResult {
            field_names: requested.iter().map(|s| (*s).to_string()).collect(),
            frames,
            start,
            total,
        };

        serde_json::to_string(&result)
            .unwrap_or_else(|e| format!(r#"{{"error":"serialization failed: {e}"}}"#))
    })
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
