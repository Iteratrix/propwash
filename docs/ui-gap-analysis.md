# UI Gap Analysis: Top 20 Blackbox Use Cases

Assessed 2026-04-13 against propwash web UI and analysis pipeline.

## Rating Key

- **Supported** — UI directly helps with this
- **Partial** — some data shown but workflow is incomplete
- **Data available** — parser has the data but UI doesn't expose it
- **Not supported** — neither parser nor UI handles this

## Assessment

| # | Use Case | Rating | Notes |
|---|----------|--------|-------|
| 1 | PID tuning (setpoint vs gyro) | Partial | Parser has both fields. No overlay view — gyro and PIDs are separate tabs. Overshoot detector compares them but pilot can't visually inspect tracking. |
| 2 | Oscillation diagnosis | Partial | Spectrum peaks classified as motor/frame. Overshoot flags P-too-high. No D-term time-series, no interactive frequency-to-PID correlation. |
| 3 | Noise floor / filter effectiveness | **Supported** | FFT per axis with filter cutoff overlays (LPF1/2, D-LPF, notches, dyn notch range). Peak badges, noise floor computation. Gap: no unfiltered-vs-filtered gyro comparison. |
| 4 | Propwash handling | Partial | ThrottleChop events detected, clickable. Throttle-banded FFT. No post-chop gyro isolation, no propwash-specific spectrum (30-90 Hz band after chops). |
| 5 | Motor noise / RPM harmonics | Partial | Throttle-banded FFT shows shifting peaks. `ERpm` field exists in parser. eRPM not exposed in any timeline or spectrum overlay. Peak classification is heuristic, not eRPM-correlated. |
| 6 | Frame resonance | **Supported** | `classify_peaks` identifies fixed-frequency peaks across throttle bands. Red badges in spectrum. Diagnostics with notch filter advice. |
| 7 | Hot motors / D-term noise | Partial | PID tab shows P-term only. `PidD(Axis)` exists in parser but not in any FIELD_GROUPS. No D-term spectrum or D-term-to-motor correlation. Filter overlay shows D-LPF cutoff on gyro spectrum. |
| 8 | ESC desync | **Supported** | Full pipeline: detection (motor spike >1.5x average), events table, severity classification, diagnostics with DShot/ESC advice, click-to-navigate. Summary card shows count. |
| 9 | Crash reconstruction | Partial | Timeline zoom works. GyroSpike events flag >500 deg/s. No "last 5 seconds" shortcut, no multi-channel overlay (gyro + motors + RC on one view), no crash-finder auto-navigation. `is_truncated()` not surfaced. |
| 10 | Accel vibration levels | **Supported** | RMS per axis with cards, accel spectrum, diagnostics with thresholds (>100 warning, >200 problem), Z-axis dominance detection, soft-mount advice. |
| 11 | Motor balance / thrust asymmetry | Partial | Motor traces in timeline. Asymmetric saturation flagged in diagnostics. No motor comparison bar chart or average-output-per-motor summary. |
| 12 | Feedforward / RC smoothing | Data available | RC commands in timeline. Setpoint exists in parser but not in any FIELD_GROUPS — never charted. No RC-vs-setpoint comparison for stair-stepping diagnosis. |
| 13 | Battery voltage sag | Data available | `Vbat` parsed for all formats. Not in any FIELD_GROUPS, only accessible via Raw Data table. No voltage-vs-throttle overlay, no sag summary, no voltage diagnostic. |
| 14 | Failsafe investigation | Partial | FirmwareMessage events from AP/PX4/MAVLink shown in events table. No failsafe-specific event type. No RC-link-loss detection from data patterns. |
| 15 | Compass interference | Not supported | `Heading` field exists. No magnetometer raw data in SensorField. No compass-vs-throttle analysis. |
| 16 | GPS glitch / position accuracy | Data available | `GpsLat`, `GpsLng`, `GpsSpeed` parsed. Not in any FIELD_GROUPS. No map, no position jump detection, no sat count or HDOP fields. |
| 17 | Altitude hold performance | Data available | `Altitude` field parsed. Not charted. No desired-vs-actual altitude comparison. |
| 18 | Step response quality | Partial | Setpoint and gyro exist. Overshoot detector quantifies overshoot %. No step response plot, no rise time / settling time calculation. |
| 19 | Power system brownout | Partial | `is_truncated()` and `corrupt_bytes()` in parser. Vbat parsed. Neither surfaced in UI. No "log ended abruptly" warning. |
| 20 | RSSI / link quality | Not supported | No RSSI or LQ field in SensorField. No RC link quality time-series. |

## Top 5 Highest-Impact Gaps

Ranked by (frequency of use case) x (feasibility given existing data).

### 1. Setpoint vs Gyro overlay (fixes #1, #4, #12, #18)

The single biggest gap. The #1 reason pilots open a blackbox viewer. Parser already has `Setpoint(Axis)` and `Gyro(Axis)`. Add a FIELD_GROUPS entry pairing setpoint and gyro per axis on the same chart, with setpoint as a semi-transparent trace. Add a timeline tab for it.

**Effort:** ~5 lines of TypeScript, zero backend changes.

### 2. Battery voltage timeline (fixes #13, #19)

Add `vbat` and `rc[throttle]` as a timeline group. Battery sag is the third most common thing pilots check. Also surface `is_truncated()` in overview as a warning card.

**Effort:** ~10 lines of TypeScript.

### 3. Full PID breakdown — P/I/D terms (fixes #2, #7)

Current PIDs tab only shows P-term. Add I and D terms. D-term traces are how pilots diagnose hot motors — if D oscillates at high frequency, it's amplifying noise into motor outputs.

**Effort:** FIELD_GROUPS additions only.

### 4. Motor balance summary (fixes #11, improves #9)

Add average motor output per motor as bar/card view in Overview. Compute mean and deviation from existing motor columns. Highlight the hardest-working motor and the delta.

**Effort:** ~30 lines in renderSummary().

### 5. Propwash-specific analysis (fixes #4)

The tool's namesake. We already detect ThrottleChop events with timestamps. For each chop, extract gyro in the 0.5-1.0s recovery window, compute FFT, show average propwash spectrum highlighting 30-90 Hz. Add a diagnostic for propwash severity.

**Effort:** ~40 lines in fft.rs + spectrum UI section.

## Future Considerations

- **eRPM overlay on spectrum**: verify RPM filter tracking visually
- **Unfiltered vs filtered gyro**: show pre/post-filter spectra on same plot
- **GPS/altitude timeline groups**: trivial additions, mainly useful for AP/PX4 autonomous modes
- **RSSI/LQ field**: requires adding to SensorField enum and format parsers
- **Magnetometer data**: requires new SensorField variants for raw mag axes
