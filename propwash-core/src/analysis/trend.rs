use serde::Serialize;

use super::FlightAnalysis;

/// Key metrics extracted from one session for cross-session trend analysis.
#[derive(Debug, Clone, Serialize)]
pub struct TrendPoint {
    /// Human-readable label (e.g., `btfl_003.bbl` / Session 1).
    pub label: String,
    pub duration_seconds: f64,
    pub sample_rate_hz: f64,
    pub frame_count: usize,
    /// Per-axis noise floor in dB (roll, pitch, yaw). `None` if no vibration data.
    pub noise_floor_db: Option<[f64; 3]>,
    /// Maximum motor balance deviation (%). `None` if no motor data.
    pub motor_balance_max_deviation: Option<f64>,
    /// Average step response rise time in ms. `None` if no step response data.
    pub step_response_rise_ms: Option<f64>,
    /// Average step response overshoot %. `None` if no step response data.
    pub step_response_overshoot: Option<f64>,
    /// Total event count.
    pub total_events: usize,
    /// Desync event count.
    pub desyncs: usize,
    /// Number of diagnostics at warning or problem severity.
    pub diagnostic_count: usize,
}

/// Extract trend points from a list of labeled analyses.
///
/// Each entry is `(label, analysis)`. Returns one `TrendPoint` per entry
/// in the same order.
#[allow(clippy::cast_precision_loss)]
pub fn compute_trend(analyses: &[(String, &FlightAnalysis)]) -> Vec<TrendPoint> {
    analyses
        .iter()
        .map(|(label, analysis)| {
            let noise_floor_db = analysis.vibration.as_ref().map(|v| v.noise_floor_db);

            let motor_balance_max_deviation = if analysis.summary.motor_balance.is_empty() {
                None
            } else {
                Some(
                    analysis
                        .summary
                        .motor_balance
                        .iter()
                        .map(|m| m.deviation_percent.abs())
                        .fold(0.0_f64, f64::max),
                )
            };

            let step_response_rise_ms = analysis.step_response.as_ref().and_then(|sr| {
                let rises: Vec<f64> = sr.axes.iter().map(|a| a.rise_time_ms).collect();
                if rises.is_empty() {
                    None
                } else {
                    Some(rises.iter().sum::<f64>() / rises.len() as f64)
                }
            });

            let step_response_overshoot = analysis.step_response.as_ref().and_then(|sr| {
                let os: Vec<f64> = sr.axes.iter().map(|a| a.overshoot_percent).collect();
                if os.is_empty() {
                    None
                } else {
                    Some(os.iter().sum::<f64>() / os.len() as f64)
                }
            });

            let diagnostic_count = analysis
                .diagnostics
                .iter()
                .filter(|d| d.severity >= super::diagnostics::Severity::Warning)
                .count();

            TrendPoint {
                label: label.clone(),
                duration_seconds: analysis.summary.duration_seconds,
                sample_rate_hz: analysis.summary.sample_rate_hz,
                frame_count: analysis.summary.frame_count,
                noise_floor_db,
                motor_balance_max_deviation,
                step_response_rise_ms,
                step_response_overshoot,
                total_events: analysis.summary.total_events,
                desyncs: analysis.summary.desyncs,
                diagnostic_count,
            }
        })
        .collect()
}
