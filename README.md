# propwash

Drone flight log analyzer. Parses Betaflight blackbox (`.bbl`), ArduPilot DataFlash (`.bin`), PX4 ULog (`.ulg`) and MAVLink telemetry (`.tlog`) logs to detect vibration issues, PID tuning problems, motor saturation, gyro spikes, and mechanical faults - with frequency spectrum analysis and actionable diagnostics.

**Try it now: [propwash.deltave.org](https://propwash.deltave.org)** - drop a log file in your browser. No install, no uploads. Runs locally via WebAssembly.

![Overview - summary, diagnostics, and events](docs/img/overview.png)

![Multi-flight diagnostics and spectrum comparison](docs/img/diagnostics.png)

```
$ propwash analyze flight.bbl

── Session 1 ──
  Firmware:    Betaflight 4.2.9
  Craft:       DIATONE ROMA F5
  Duration:    89.0s
  Sample rate: 249 Hz
  Motors:      4

  Timeline (68 episodes):
      10.486s  GYRO SPIKE     pitch peak 677°/s (47 frames) (185ms)
      10.647s  OVERSHOOT      pitch peak 205% (14 frames) (52ms)
      25.429s  MOTOR SAT      motor[0] for 8 frames
      88.255s  GYRO SPIKE     pitch peak 2032°/s (103 frames) (414ms)

  Vibration Analysis (full flight):
    roll axis — noise floor 22.3 dB:
      #1: 205 Hz at 45.2 dB
      #2: 51 Hz at 32.0 dB
    pitch axis — noise floor 21.3 dB:
      #1: 218 Hz at 41.4 dB

  Vibration by Throttle:
    0-25%:   roll dominant 34 Hz    ← propwash
    25-50%:  roll dominant 205 Hz   ← frame resonance
    75-100%: roll dominant 21 Hz    ← PID oscillation

  Diagnostics:
    !!  [motors] Motor[0] saturating significantly more than others
    !!  [pid] Severe pitch overshoot (peak 372%, avg 78%)
    !!  [mechanical] Extreme pitch gyro spike (2032°/s)
```

## Web UI

Features: views for Overview, Timeline, Spectrum, Compare, Trend and Raw Data; PID tuning with step-response overlays and a paste-ready Betaflight CLI diff; interactive ECharts and uPlot charts with zoom/pan; spectrogram heatmaps; filter-cutoff and RPM-harmonic overlays; throttle-band analysis; side-by-side comparison and trends across flights. Works offline once loaded.

## Install (CLI)

Download a prebuilt binary from [Releases](https://github.com/Iteratrix/propwash/releases) (archives are named `propwash-cli-<platform>`), or build from source:

```bash
cargo install --git https://github.com/Iteratrix/propwash propwash-cli
```

Either way, the command is `propwash`.

## Usage

```bash
# Full analysis with event timeline, vibration FFT, and diagnostics
propwash analyze flight.bbl

# JSON output for programmatic consumption
propwash analyze flight.bbl --output json

# Log metadata and field inventory
propwash info flight.bbl

# Batch triage of multiple files
propwash scan *.bbl *.bin *.ulg

# Side-by-side comparison of two flights
propwash compare before.bbl after.bbl

# Raw frame dump (for debugging / agentic analysis)
propwash dump flight.bbl --session 2 --frames 100-200 --fields gyroADC,motor
```

## What it detects

**Events:**
- Throttle chops and punches (rapid throttle changes)
- Motor saturation (motor hitting max output)
- Gyro spikes (extreme rotation rates)
- Setpoint overshoot (PID tracking errors)
- ESC desync (single motor spike)
- Firmware messages (PX4 log messages, ArduPilot error codes)

**Vibration:**
- Frequency spectrum per axis (FFT with Hann windowing)
- Top 5 frequency peaks with noise classification (motor noise vs frame resonance)
- Throttle-windowed analysis (how vibration changes with motor speed)
- Spectrogram (frequency vs time heatmap)
- Accelerometer vibration (FC mounting quality)

**Tuning:**
- Step response per axis (overshoot, rise and settling time) with overlaid step traces
- P/I/D gain suggestions, exported as a paste-ready Betaflight CLI diff
- I-term windup and oscillation frequency

**Diagnostics:**
- Asymmetric motor saturation → mechanical issue on one side
- Severe overshoot → P/D imbalance with axis-specific recommendation
- Extreme gyro spikes → crash/damage detection
- Frame resonance → notch filter recommendation
- Throttle-dependent frequency shift → motor noise (RPM filtering)
- FC mounting issues → accelerometer RMS analysis

## Supported formats

| Firmware | Format | Status |
|----------|--------|--------|
| Betaflight | `.bbl` blackbox | Full support |
| EmuFlight | `.bbl` blackbox | Full support |
| Rotorflight | `.bbl` blackbox | Full support |
| INAV | `.bbl` blackbox | Full support |
| Cleanflight | `.bbl` blackbox | Full support |
| ArduPilot (Copter/Plane/Rover) | `.bin` DataFlash | Full support; vibration from the batch sampler* |
| PX4 | `.ulg` ULog | Full support |
| ArduPilot / PX4 ground station | `.tlog` MAVLink | Events, attitude and tuning; too slow (~10 Hz) for vibration |

\* Vibration spectra use the batch sampler's full-rate gyro bursts (`ISBH`/`ISBD`, enabled with `INS_LOG_BAT_MASK`). Without them, the `IMU` message is often logged at only 25-50 Hz; a spectrum only reaches half the sample rate, so propwash skips vibration checks on such logs and says why.

## Architecture

```
propwash-core/          Parser and analysis library (usable on its own)
  format/bf/            Betaflight-family blackbox decoder
  format/ap/            ArduPilot DataFlash decoder
  format/px4/           PX4 ULog decoder
  format/mavlink/       MAVLink telemetry (.tlog) decoder
  analysis/             Events, FFT, step response, PID tuning, diagnostics
propwash-cli/           CLI (installs the `propwash` command)
propwash-web/           WASM bridge (powers the web UI)
web/                    Browser frontend (TypeScript + ECharts + uPlot)
```

Each format decodes into one typed `Session`: per-stream time axes, with units carried as types (`DegPerSec`, `Volts`, `Normalized01`, ...).

```rust
use propwash_core::types::SensorField;

let log = propwash_core::decode_file("flight.bbl")?;
for session in &log.sessions {
    let roll = &session.gyro.values.roll; // Vec<DegPerSec>
    let motor0 = session.field(&"motor[0]".parse::<SensorField>()?); // by name, for user input
    let analysis = propwash_core::analysis::analyze(session);
    for d in &analysis.diagnostics {
        println!("{}: {}", d.category, d.message);
    }
}
```

## Performance

Parses a 15MB Betaflight log (472K frames) in 250ms. 45x faster than Python parsers.

## License

MIT
