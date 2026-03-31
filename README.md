# propwash

Flight log vibration analyzer for FPV drones. Parses Betaflight-family blackbox logs and tells you what's wrong.

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

**[propwash.deltave.org](https://propwash.deltave.org)** — drop a `.bbl` file in your browser and get instant analysis. No install, no uploads. Everything runs locally via WebAssembly.

## Install (CLI)

Download a prebuilt binary from [Releases](https://github.com/Iteratrix/propwash/releases), or build from source:

```bash
cargo install --git https://github.com/Iteratrix/propwash propwash
```

## Usage

```bash
# Full analysis with event timeline, vibration FFT, and diagnostics
propwash analyze flight.bbl

# JSON output for programmatic consumption
propwash analyze flight.bbl --output json

# Log metadata and field inventory
propwash info flight.bbl
propwash info flight.bbl --json

# Raw frame dump (for agentic analysis / debugging)
propwash dump flight.bbl --session 2 --frames 100-200 --fields gyroADC,motor
```

## What it detects

**Events:**
- Throttle chops and punches (rapid throttle changes)
- Motor saturation (motor hitting max output)
- Gyro spikes (extreme rotation rates)
- Setpoint overshoot (PID tracking errors)

**Vibration:**
- Frequency spectrum per axis (FFT with Hann windowing)
- Top 5 frequency peaks in dB
- Throttle-windowed analysis (how vibration changes with motor speed)
- Noise floor measurement

**Diagnostics:**
- Asymmetric motor saturation → mechanical issue on one side
- Severe overshoot → P/D imbalance with axis-specific recommendation
- Extreme gyro spikes → crash/damage detection
- Frame resonance → notch filter recommendation
- Throttle-dependent frequency shift → motor noise (RPM filtering)

## Supported formats

| Firmware | Status |
|----------|--------|
| Betaflight | Full support |
| EmuFlight | Full support |
| Rotorflight | Full support |
| INAV | Full support |
| Cleanflight | Partial (legacy format) |

## Architecture

```
propwash-core/          Parser library (can be used independently)
  format/bf/            Betaflight-family format decoder
  analysis/             Event detection, FFT, diagnostics
propwash/               CLI binary
propwash-web/           WASM bridge (powers the web UI)
web/                    Browser frontend (vanilla JS + uPlot)
```

Three-layer API for library consumers:
- **`session.unified()`** — format-agnostic sensor data
- **`session.analyzed()`** — firmware-specific insights
- **`session.raw`** — direct access to parsed frames

## Performance

Parses a 15MB log (472K frames) in 250ms. 45x faster than Python parsers.

## License

MIT
