pub mod diagnostics;
pub mod episodes;
pub mod events;
pub mod fft;
pub mod pid;
pub mod step_response;
pub mod summary;
pub mod trend;
pub mod unified_events;
pub(crate) mod util;

use diagnostics::Diagnostic;
use events::FlightEvent;
use fft::VibrationAnalysis;
use pid::PidAnalysis;
use step_response::StepResponseAnalysis;
use summary::FlightSummary;

use crate::types::Session;

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct FlightAnalysis {
    pub summary: FlightSummary,
    pub events: Vec<FlightEvent>,
    pub vibration: Option<VibrationAnalysis>,
    pub step_response: Option<StepResponseAnalysis>,
    pub pid: Option<PidAnalysis>,
    pub diagnostics: Vec<Diagnostic>,
}

/// Analyzes a parsed session, detecting events and producing a summary.
///
/// TODO(refactor/session-typed): all format-specific event extraction
/// (PX4 `log_messages`, AP `EV`/`ERR`, `MAVLink` `STATUSTEXT`) now lives
/// inside each parser's `build` step and lands in `session.events`.
/// Once parsers are filled in, fold those into `FlightEvents` here.
pub fn analyze(session: &Session) -> FlightAnalysis {
    let detected = unified_events::detect_all(session);

    let vibration = fft::analyze_vibration_unified(session, &detected);
    let summary = summary::summarize(session, &detected);
    let filter_config = session.filter_config();
    let step_resp = step_response::analyze_step_response(session);
    let pid_gains = session.pid_gains();
    let pid_analysis = pid::analyze_pid(session, step_resp.as_ref(), &pid_gains);
    let diags = diagnostics::diagnose(
        &detected,
        vibration.as_ref(),
        step_resp.as_ref(),
        pid_analysis.as_ref(),
        &filter_config,
        summary.motor_count,
        summary.duration_seconds,
    );

    FlightAnalysis {
        summary,
        events: detected,
        vibration,
        step_response: step_resp,
        pid: pid_analysis,
        diagnostics: diags,
    }
}
