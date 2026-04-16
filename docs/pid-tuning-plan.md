# PID Tuning Feature — Plan & Roadmap

## What's Done

### Core Analysis (`propwash-core/src/analysis/pid.rs`)
- Step detection: finds setpoint jumps >100 deg/s where target >50 deg/s absolute
- Response measurement per step: overshoot %, rise time (10%→90%), settling time (within 5% band)
- Per-axis rating: "tight", "good", "sluggish", "oscillating" with concrete tuning suggestions
- Uses median across detected steps for robustness against outliers
- `PidAnalysis` included in `FlightAnalysis` — auto-serialized through WASM bridge

### Web UI (`web/src/app.ts`, `web/index.html`, `web/style.css`)
- "PID Tuning" tab between Overview and Timeline
- Step response cards: per-axis overshoot %, rise time, settling time, step count, color-coded rating, suggestion text
- Setpoint vs Gyro overlay charts: per axis, synced zoom, setpoint as dashed gray, gyro in axis color, overshoot event markers
- P/I/D term breakdown charts: per-axis with roll/pitch/yaw tab selector

## What's Missing

### Analysis Improvements
- [ ] **Feedforward field**: add `SensorField::Feedforward(Axis)` to the type system and parse `axisF[N]` from Betaflight logs. Show in PID breakdown charts.
- [ ] **Oscillation frequency detection**: when overshoot is high, use FFT on the error signal (setpoint - gyro) around steps to identify the oscillation frequency. Report "oscillating at ~45 Hz" rather than just "oscillating."
- [ ] **D-term noise analysis**: compare D-term RMS at different throttle levels. High D-term noise suggests D is too high or D-term LPF is too aggressive.
- [ ] **I-term windup detection**: detect sustained I-term growth during maneuvers (e.g., I-term exceeding P-term for extended periods). Suggests I gain is too high.
- [ ] **Settling time verification**: current settling heuristic uses "first frame within 5% band after rise." A better approach: verify all subsequent frames (or a window of N frames) stay within the band. Avoids false early-settle detection during oscillation.
- [ ] **Per-step detail export**: expose individual step measurements (not just median) so the UI could show a distribution or overlay multiple step responses.

### Web UI Improvements
- [ ] **Error signal chart**: computed `setpoint - gyro` per axis, shown as a separate timeseries. Makes overshoot/lag/oscillation immediately visible without mental subtraction.
- [ ] **Step response overlay**: zoom to individual steps and overlay them aligned at t=0, showing the average response shape. This is what Betaflight's PID Toolbox does.
- [ ] **D-term noise chart**: D-term alongside gyro unfilt, showing how much D-term noise exists relative to the signal.
- [ ] **Tuning suggestions panel**: aggregate the per-axis suggestions into a prioritized "top 3 things to change" list at the top of the PID view.
- [ ] **Filter interaction**: on the spectrum tab, overlay filter cutoff frequencies from `FilterConfig` on the FFT plots so pilots can see whether their filters are cutting the right frequencies.

### ArduPilot / PX4 Specifics
- [ ] **AP PID data**: ArduPilot logs P/I/D in separate PIDR/PIDP/PIDY messages with different sample rates than IMU. The current step detection assumes uniform sample rate across setpoint and gyro — verify this holds for AP or interpolate.
- [ ] **PX4 rate controller output**: PX4 logs rate controller output in `rate_ctrl_status` topic. Map this to the PID fields.
- [ ] **Rate setpoint sources**: ArduPilot `RATE` message `RDes`/`PDes`/`YDes` fields; PX4 `vehicle_rates_setpoint`. Verify both are correctly mapped to `SensorField::Setpoint`.

### Testing
- [ ] **Golden value tests for PID analysis**: add test fixtures with known step responses and verify the computed metrics match expected values.
- [ ] **Edge cases**: empty setpoint data, constant setpoint (no steps), very short flights, yaw-only flights.
- [ ] **Browser testing**: verify the full PID Tuning tab renders correctly with real flight logs across Betaflight, ArduPilot, and PX4.
