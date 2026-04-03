# Known Issues

## Legacy Cleanflight (Data version 1) — improved

**Affected files:** `cleanflight/LOG00568-572.TXT` (2015-era logs).

Cleanflight Data version 1 logs use older encoding/predictor conventions.
Most frames parse correctly. Some files may still diverge from the official
decoder after several thousand frames due to subtle predictor differences.

**Impact:** Only affects 10-year-old log files. All modern Betaflight,
EmuFlight, Rotorflight, INAV, and ArduPilot files parse correctly.
