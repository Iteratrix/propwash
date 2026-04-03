# Format Support

## Supported

| Format | Extension | Firmware | Status |
|--------|-----------|----------|--------|
| Betaflight Blackbox | `.bbl` | Betaflight, EmuFlight, Rotorflight, INAV, Cleanflight | Full support |
| ArduPilot DataFlash | `.bin` | ArduCopter, ArduPlane, ArduRover, ArduSub | Full support |

## Planned

| Priority | Format | Extension | Firmware | Difficulty | Notes |
|----------|--------|-----------|----------|------------|-------|
| 1 | PX4 ULog | `.ulg` | PX4 Autopilot | Medium | Second largest open-source autopilot. Self-describing, no prediction encoding. `pyulog` reference parser. |
| 2 | DJI Flight Logs | `.DAT`, `.txt` | DJI Phantom/Mavic/Mini/Air/FPV | Hard | 70%+ market share. Encrypted in newer versions. `DatCon` (Java) reverse-engineered parser exists. |
| 3 | MAVLink Telemetry | `.tlog` | ArduPilot, PX4 (ground station side) | Easy | Timestamped MAVLink packets logged by Mission Planner/QGC. Low rate (~10Hz). |
| 4 | KISS FC | binary | KISS FC, KISS Ultra | Easy | Simple fixed-width fields. Racing niche, no good standalone parser. |
| 5 | Gyroflow | `.gcsv` | N/A (camera gyro data) | Trivial | Text CSV. Pure gyro/accel for video stabilization. |

## Not Planned

| Format | Why |
|--------|-----|
| Parrot PUD | Proprietary, small user base |
| Autel DAT | Proprietary, encrypted, no public documentation |
| Skydio | Completely proprietary, logs not user-accessible |
| LibrePilot/dRonin UAVTalk | Dead communities |
| FlightOne/FalcoX | Dead or Betaflight-compatible |
| RaceFlight | Dead |
| MultiWii MSP | Historical, no onboard logging |
| Spektrum/FrSky/CRSF telemetry | Radio telemetry, not FC logs |
| Paparazzi UAV | Academic niche |
| Crazyflie CRTP | Research niche |
| PX4 sdlog2 | Obsolete, superseded by ULog |

## Format Detection

All format detection uses magic bytes, not file extensions:

| Format | Magic | Offset |
|--------|-------|--------|
| Betaflight | `H Product:Blackbox flight data recorder` | First session start |
| ArduPilot | `0xA3 0x95 0x80` | Byte 0 (HEAD1 HEAD2 FMT_TYPE) |
| PX4 ULog | `ULog` + version byte | Byte 0 |
| DJI DAT | TBD (version-dependent) | TBD |
| MAVLink tlog | `0xFE` or `0xFD` (MAVLink v1/v2 STX) | After 8-byte timestamp |
