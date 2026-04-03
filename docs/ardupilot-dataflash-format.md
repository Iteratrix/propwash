# ArduPilot DataFlash Binary Log Format (.bin) -- Technical Specification

Derived from ArduPilot source code (libraries/AP_Logger/) and pymavlink/DFReader.py.

---

## 1. File Structure Overview

A `.bin` file is a flat stream of back-to-back binary messages with no file-level
header, footer, or index. Each message is self-framed by a 3-byte header. The
file begins with a sequence of metadata messages written during log startup:

```
[FMT messages -- one per message type]
[PARM messages -- one per parameter]
[UNIT messages -- one per unit character]
[MULT messages -- one per multiplier character]
[FMTU messages -- one per message type, linking fields to units/multipliers]
[SysInfo: MSG (firmware string), VER, MSG (system id), MSG (param space), MSG (RC protocol)]
[Mission CMD messages]
[Rally RALY messages]
[Fence messages]
[Vehicle-specific startup messages]
... normal flight data messages ...
```

There is no explicit end-of-file marker. The file simply stops when logging ends.

---

## 2. Message Framing

Every message has the same 3-byte header:

```
Offset  Size  Value
0       1     HEAD_BYTE1 = 0xA3 (163)
1       1     HEAD_BYTE2 = 0x95 (149)
2       1     msg_type   (uint8_t, 0-255)
```

The total message length (including the 3-byte header) is defined per message
type in the corresponding FMT message. Maximum message length is 255 bytes
(`LOG_PACKET_MAX_LEN = UINT8_MAX`).

```
+------+------+---------+---------------------------+
| 0xA3 | 0x95 | msg_type|       payload ...         |
+------+------+---------+---------------------------+
  1 byte 1 byte  1 byte    (msg_len - 3) bytes
```

**There are no checksums, CRCs, or length fields in the wire format.** Message
length is determined entirely by the FMT definition for each msg_type.

---

## 3. Byte Order

**Little-endian throughout.** This is an ARM Cortex-M platform. The pymavlink
DFReader uses `struct.unpack("<...")` for all fields. Multi-byte integers and
floats are stored in little-endian byte order.

---

## 4. FMT Messages (Type 128 = 0x80)

FMT is the self-describing bootstrap message. Its own format is hardcoded --
you must know FMT's layout to parse anything else. msg_type 128 is reserved
for FMT and must not be used for any other message.

### Wire layout (89 bytes total):

```
Offset  Size  Field       Type        Description
0       1     head1       uint8       0xA3
1       1     head2       uint8       0x95
2       1     msgid       uint8       128 (LOG_FORMAT_MSG)
3       1     type        uint8       Message type being defined (0-255)
4       1     length      uint8       Total message length including header
5       4     name        char[4]     Message name (null-padded, NOT null-terminated)
9       16    format      char[16]    Format string (one char per field)
25      64    labels      char[64]    Comma-separated field names
```

Total: 3 + 1 + 1 + 4 + 16 + 64 = **89 bytes**

The FMT message for FMT itself is:
```
type=128, length=89, name="FMT", format="BBnNZ", labels="Type,Length,Name,Format,Columns"
```

### Format string characters and their sizes:

| Char | C Type            | Wire Size | Semantic                              |
|------|-------------------|-----------|---------------------------------------|
| `a`  | int16_t[32]       | 64 bytes  | Array of 32 signed 16-bit integers    |
| `b`  | int8_t            | 1 byte    | Signed byte                           |
| `B`  | uint8_t           | 1 byte    | Unsigned byte                         |
| `h`  | int16_t           | 2 bytes   | Signed 16-bit integer                 |
| `H`  | uint16_t          | 2 bytes   | Unsigned 16-bit integer               |
| `i`  | int32_t           | 4 bytes   | Signed 32-bit integer                 |
| `I`  | uint32_t          | 4 bytes   | Unsigned 32-bit integer               |
| `f`  | float             | 4 bytes   | IEEE 754 single-precision float       |
| `d`  | double            | 8 bytes   | IEEE 754 double-precision float       |
| `n`  | char[4]           | 4 bytes   | Fixed-length string (null-padded)     |
| `N`  | char[16]          | 16 bytes  | Fixed-length string (null-padded)     |
| `Z`  | char[64]          | 64 bytes  | Fixed-length string (null-padded)     |
| `c`  | int16_t * 100     | 2 bytes   | Centi-value (divide by 100 for real)  |
| `C`  | uint16_t * 100    | 2 bytes   | Unsigned centi-value                  |
| `e`  | int32_t * 100     | 4 bytes   | Centi-value (divide by 100)           |
| `E`  | uint32_t * 100    | 4 bytes   | Unsigned centi-value                  |
| `L`  | int32_t           | 4 bytes   | Lat/Lng (multiply by 1e-7 for deg)    |
| `M`  | uint8_t           | 1 byte    | Flight mode enum                      |
| `q`  | int64_t           | 8 bytes   | Signed 64-bit integer                 |
| `Q`  | uint64_t          | 8 bytes   | Unsigned 64-bit integer               |
| `g`  | Float16_t         | 2 bytes   | IEEE 754 half-precision float         |

**Key parsing detail:** `c`, `C`, `e`, `E` are stored as integers on the wire.
The multiplier (0.01) is *semantic* -- the raw bytes are just int16/uint16/int32/uint32.
Similarly, `L` is stored as int32 but represents degrees * 1e7.

**Maximum format string length:** 16 characters (max 16 fields per message).
**Maximum labels string length:** 64 characters.

### Computing message payload size from format string:

```rust
fn format_char_size(c: char) -> usize {
    match c {
        'a' => 64,          // int16_t[32]
        'b' | 'B' | 'M' => 1,
        'h' | 'H' | 'c' | 'C' | 'g' => 2,
        'i' | 'I' | 'f' | 'e' | 'E' | 'L' | 'n' => 4,
        'd' | 'q' | 'Q' => 8,
        'N' => 16,
        'Z' => 64,
        _ => panic!("unknown format char"),
    }
}

// Total message length = 3 (header) + sum of format_char_size for each char
```

---

## 5. UNIT Messages

Define a mapping from a single-character ID to a unit string.

### Wire layout (76 bytes total):

```
Offset  Size  Field       Type        Description
0-2     3     header      -           0xA3 0x95 <UNIT_msg_type>
3       8     time_us     uint64      Timestamp (microseconds since boot)
11      1     type        char        Single-character unit ID
12      64    unit        char[64]    Unit string (e.g. "m/s", "deg", "Pa")
```

### Unit ID table (from source):

| ID  | Unit          | Description                    |
|-----|---------------|--------------------------------|
| `-` | (empty)       | No units (dimensionless)       |
| `?` | UNKNOWN       | Not yet determined             |
| `A` | A             | Ampere                         |
| `a` | Ah            | Ampere hours                   |
| `d` | deg           | Degrees (angular)              |
| `b` | B             | Bytes                          |
| `B` | B/s           | Bytes per second               |
| `k` | deg/s         | Degrees per second             |
| `D` | deglatitude   | Degrees of latitude            |
| `e` | deg/s/s       | Degrees per second squared     |
| `E` | rad/s         | Radians per second             |
| `G` | Gauss         | Magnetic field strength        |
| `h` | degheading    | Heading (0-360)                |
| `i` | A.s           | Ampere-seconds                 |
| `J` | W.s           | Joules (Watt-seconds)          |
| `l` | l             | Litres                         |
| `L` | rad/s/s       | Radians per second squared     |
| `m` | m             | Metres                         |
| `n` | m/s           | Metres per second              |
| `o` | m/s/s         | Metres per second squared      |
| `O` | degC          | Degrees Celsius                |
| `%` | %             | Percent                        |
| `S` | satellites    | Number of satellites           |
| `s` | s             | Seconds                        |
| `t` | N.m           | Newton-metres (torque)         |
| `q` | rpm           | Revolutions per minute         |
| `r` | rad           | Radians                        |
| `U` | deglongitude  | Degrees of longitude           |
| `u` | ppm           | Pulses per minute              |
| `v` | V             | Volt                           |
| `P` | Pa            | Pascal                         |
| `w` | Ohm           | Ohm                            |
| `W` | Watt          | Watt                           |
| `X` | W.h           | Watt-hours                     |
| `y` | l/s           | Litres per second              |
| `Y` | us            | PWM microseconds               |
| `z` | Hz            | Hertz                          |
| `#` | instance      | Sensor instance number         |

---

## 6. MULT Messages

Define a mapping from a single-character ID to a numeric multiplier.

### Wire layout (20 bytes total):

```
Offset  Size  Field         Type        Description
0-2     3     header        -           0xA3 0x95 <MULT_msg_type>
3       8     time_us       uint64      Timestamp
11      1     type          char        Single-character multiplier ID
12      8     multiplier    double      Numeric multiplier value
```

### Multiplier ID table (from source):

| ID  | Multiplier | Description                                    |
|-----|------------|------------------------------------------------|
| `-` | 0          | No multiplier (strings, etc.)                  |
| `?` | 1          | Unknown/not yet determined                     |
| `2` | 1e2        | Multiply by 100                                |
| `1` | 1e1        | Multiply by 10                                 |
| `0` | 1e0        | No scaling (1.0)                               |
| `A` | 1e-1       | Divide by 10                                   |
| `B` | 1e-2       | Divide by 100 (centi-)                         |
| `C` | 1e-3       | Divide by 1000 (milli-)                        |
| `D` | 1e-4       | Divide by 10000                                |
| `E` | 1e-5       | Divide by 100000                               |
| `F` | 1e-6       | Divide by 1000000 (micro-)                     |
| `G` | 1e-7       | Divide by 10000000 (lat/lng scaling)           |
| `I` | 1e-9       | Divide by 1000000000 (nano-)                   |
| `!` | 3.6        | km/h to m/s, or A.s to mA.h                   |
| `/` | 3600       | A.s to A.h                                     |

**Important:** The multiplier applies to the *raw integer value* in the log.
Any scaling implied by the format character (e.g., the "centi" in `c` format)
is IGNORED for MULT purposes. The format character only tells you the C type.

---

## 7. FMTU Messages (Format Units)

Link a message type to its per-field units and multipliers.

### Wire layout (44 bytes total):

```
Offset  Size  Field         Type        Description
0-2     3     header        -           0xA3 0x95 <FMTU_msg_type>
3       8     time_us       uint64      Timestamp
11      1     format_type   uint8       Message type this applies to
12      16    units         char[16]    Unit ID per field (chars from UNIT table)
28      16    multipliers   char[16]    Multiplier ID per field (chars from MULT table)
```

Each character position in `units` and `multipliers` corresponds to the same
position in the FMT `format` string. This tells you what physical unit and
scale each field has.

---

## 8. Timestamps

### TimeUS field

Nearly every message begins with a `uint64_t time_us` field (format char `Q`)
immediately after the 3-byte header. This is the value of `AP_HAL::micros64()`
-- microseconds since system boot.

The FMT message itself is an exception -- it has no timestamp.

### Interpreting timestamps

- TimeUS is monotonically increasing (within a single log)
- TimeUS wraps at 2^64 (effectively never)
- TimeUS starts from 0 at boot
- For wall-clock time, look for the **RTC** message:
  - `RTC.Epoch` = Unix timestamp in microseconds (uint64)
  - Correlate `RTC.TimeUS` with other messages' TimeUS to get absolute time
- GPS time is available via `GPS.GMS` (ms since start of GPS week) and `GPS.GWk` (GPS week number)

---

## 9. Session / Flight Boundaries

ArduPilot creates a new log file for each arming, so typically one .bin file = one flight session. However, multiple arm/disarm cycles can occur within a single file. Detect boundaries via:

### EV (Event) messages:
- `EV.Id == 10` --> ARMED
- `EV.Id == 11` --> DISARMED

### ARM messages:
- `ARM.ArmState == 1` --> Armed
- `ARM.ArmState == 0` --> Disarmed

### MODE messages:
- Mode changes indicate flight phase transitions

### Other event IDs:
| ID | Event                    |
|----|--------------------------|
| 10 | ARMED                    |
| 11 | DISARMED                 |
| 15 | AUTO_ARMED               |
| 17 | LAND_COMPLETE_MAYBE      |
| 18 | LAND_COMPLETE            |
| 19 | LOST_GPS                 |
| 25 | SET_HOME                 |
| 28 | NOT_LANDED               |
| 54 | MOTORS_EMERGENCY_STOPPED |

---

## 10. Sample Rates (ArduCopter typical)

Rates depend on `LOG_BITMASK` parameter and scheduler configuration. ArduCopter
main loop runs at 400Hz. Typical rates:

| Message    | Rate (Hz) | Notes                                        |
|------------|-----------|----------------------------------------------|
| IMU        | 25        | Default; 400 with MASK_LOG_IMU_FAST          |
| ACC, GYR   | 400       | Raw sensor, only with IMU_FAST or IMU_RAW    |
| ATT        | 10        | Default (ATTITUDE_MED); 400 with ATT_FAST    |
| RATE       | 10-400    | Same bitmask as ATT                          |
| AHR2       | 10        | Always at 10Hz                               |
| GPS        | 5-50      | Follows GPS update rate (typically 5-10 Hz)  |
| GPA        | 5-50      | Same as GPS                                  |
| BARO       | 10        | Logged from update_batt_compass (10Hz task)  |
| MAG        | 10        | Logged from update_batt_compass              |
| RCIN       | 10-25     | Depends on LOG_BITMASK; typically 10Hz       |
| RCOU       | 10-25     | Same as RCIN                                 |
| BAT        | 10        | Battery data from 10Hz task                  |
| ESC        | 10-100    | ESC telemetry rate                           |
| PIDR/P/Y   | 10-400    | Same rate as ATT logging                     |
| PSCN/E/D   | varies    | Position controller, rate-limited             |
| PM         | 1         | Performance monitor, 1Hz                     |
| MODE       | event     | Only on mode change                          |
| EV         | event     | Only on events                               |
| ERR        | event     | Only on errors                               |
| PARM       | startup   | All parameters logged once at log start      |
| CMD        | startup   | Mission logged once at log start             |

Rate limiting is configurable via `LOG_FILE_RATEMAX` and `LOG_DARM_RATEMAX`
parameters. Messages marked `streaming: true` in the LogStructure definition
can be rate-limited.

---

## 11. Key Message Types for Flight Analysis

### Message Type ID Assignments

Message type IDs are assigned via the `LogMessages` enum. IDs 0-31 are reserved
for vehicle-specific use. Common messages start at 32. FMT is always 128.

**You should NOT hardcode these IDs** -- always parse the FMT messages to discover
the mapping between type IDs and message names. The enum values can change
between firmware versions. The format is self-describing by design.

### Critical messages and their wire formats:

#### ATT -- Attitude (35 bytes)
```
Format: "QffffffB"
Fields: TimeUS, DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw, AEKF
Units:  degrees for roll/pitch, heading degrees for yaw
```

#### IMU -- Inertial Measurement Unit (59 bytes)
```
Format: "QBffffffIIfBBHH"
Fields: TimeUS, I, GyrX, GyrY, GyrZ, AccX, AccY, AccZ, EG, EA, T, GH, AH, GHz, AHz
Units:  rad/s (gyro), m/s/s (accel), degC (temp), Hz (rates)
```

#### ACC -- Raw Accelerometer (31 bytes)
```
Format: "QBQfff"
Fields: TimeUS, I, SampleUS, AccX, AccY, AccZ
Units:  m/s/s
```

#### GYR -- Raw Gyroscope (31 bytes)
```
Format: "QBQfff"
Fields: TimeUS, I, SampleUS, GyrX, GyrY, GyrZ
Units:  rad/s
```

#### GPS (55 bytes)
```
Format: "QBBIHBcLLeffffB"
Fields: TimeUS, I, Status, GMS, GWk, NSats, HDop, Lat, Lng, Alt, Spd, GCrs, VZ, Yaw, U
- Status: 0=no GPS, 1=no fix, 2=2D, 3=3D, 4=3D+DGPS, 5=RTK float, 6=RTK fixed
- Lat/Lng: int32 * 1e-7 = degrees
- Alt: int32 * 1e-2 = meters (centimeters stored)
- HDop: int16 * 0.01
```

#### GPA -- GPS Accuracy (39 bytes)
```
Format: "QBCCCCfBIHeHH"
Fields: TimeUS, I, VDop, HAcc, VAcc, SAcc, YAcc, VV, SMS, Delta, AEl, RTCMFU, RTCMFD
```

#### RCIN -- RC Input (31 bytes)
```
Format: "QHHHHHHHHHHHHHH"
Fields: TimeUS, C1-C14
Units:  PWM microseconds (typically 1000-2000)
```

#### RCOU -- Servo/Motor Output (31 bytes)
```
Format: "QHHHHHHHHHHHHHH"
Fields: TimeUS, C1-C14
Units:  PWM microseconds
```

#### BARO -- Barometer (43 bytes)
```
Format: "QBfffcfIffBf"
Fields: TimeUS, I, Alt, AltAMSL, Press, Temp, CRt, SMS, Offset, GndTemp, H, CPress
Units:  m (altitude), Pa (pressure), degC (temp), m/s (climb rate)
- Temp: int16 * 0.01 = degC
```

#### MAG -- Magnetometer (33 bytes)
```
Format: "QBhhhhhhhhhBI"
Fields: TimeUS, I, MagX, MagY, MagZ, OfsX, OfsY, OfsZ, MOX, MOY, MOZ, Health, S
Units:  Gauss * 1000 (milligauss)
```

#### BAT -- Battery (39 bytes)
```
Format: "QBfffffcfBBB"
Fields: TimeUS, Inst, Volt, VoltR, Curr, CurrTot, EnrgTot, Temp, Res, RemPct, H, SH
Units:  V, A, Ah, Wh, degC, Ohm, %
```

#### ESC -- ESC Telemetry (35 bytes)
```
Format: "QBffffcfcf"
Fields: TimeUS, Instance, RPM, RawRPM, Volt, Curr, Temp, CTot, MotTemp, Err
Units:  rpm, V, A, centi-degC, mAh, centi-degC, %
```

#### MODE -- Flight Mode (14 bytes)
```
Format: "QMBB"
Fields: TimeUS, Mode, ModeNum, Rsn
```

#### EV -- Event (12 bytes)
```
Format: "QB"
Fields: TimeUS, Id
```

#### ERR -- Error (13 bytes)
```
Format: "QBB"
Fields: TimeUS, Subsys, ECode
```

#### PARM -- Parameter (31 bytes)
```
Format: "QNff"
Fields: TimeUS, Name, Value, Default
```

#### RATE -- Attitude Rates (59 bytes)
```
Format: "Qfffffffffffff"
Fields: TimeUS, RDes, R, ROut, PDes, P, POut, YDes, Y, YOut, ADes, A, AOut, AOutSlew
```

#### PIDR/PIDP/PIDY -- PID Loops (52 bytes)
```
Format: "QffffffffffB"
Fields: TimeUS, Tar, Act, Err, P, I, D, FF, DFF, Dmod, SRate, Flags
```

#### PSCN/PSCE/PSCD -- Position Controller (47 bytes)
```
Format: "Qfffffffff"
Fields: TimeUS, DPx, TPx, Px, DVx, TVx, Vx, DAx, TAx, Ax
Units:  m (position), m/s (velocity), m/s/s (accel)
```

#### MOTB -- Motor/Battery Interaction (30 bytes)
```
Format: "QfffffB"
Fields: TimeUS, LiftMax, BatVolt, ThLimit, ThrAvMx, ThrOut, FailFlags
```

#### VIBE -- Vibration Data (24 bytes)
```
Format: "QBfffI"
Fields: TimeUS, IMU, VibeX, VibeY, VibeZ, Clip
Units:  m/s/s (vibration), count (clipping events)
```

#### VER -- Firmware Version (89 bytes)
```
Format: "QBHBBBBIZHBBII"
Fields: TimeUS, BT, BST, Maj, Min, Pat, FWT, GH, FWS, APJ, BU, FV, IMI, ICI
```

#### RTC -- Real-Time Clock (20 bytes)
```
Format: "QQB"
Fields: TimeUS, Epoch, SourceType
- Epoch: Unix time * 1e6 (microseconds since 1970-01-01)
```

---

## 12. Parsing Strategy

### Pseudocode for a binary parser:

```
1. Scan for sync bytes: 0xA3 0x95
2. Read msg_type byte
3. If msg_type == 128 (FMT):
   - Parse FMT with hardcoded 89-byte layout
   - Register: formats[type] = { name, length, format_string, labels }
4. Else:
   - Look up format for this msg_type
   - If unknown, SKIP (you don't know the length -- resync needed)
   - Read (length - 3) payload bytes
   - Unpack fields according to format string
5. Handle UNIT, MULT, FMTU messages to build unit/multiplier tables
6. Repeat until EOF
```

### Resynchronization on corrupt data:

If you encounter bytes that don't match a valid header (0xA3 0x95 followed by
a known msg_type), advance byte-by-byte until you find the next valid header.
pymavlink's DFReader does exactly this, emitting warnings about skipped bytes.

### Important edge cases:

- Some FMT messages may be emitted mid-stream (not just at file start) if a
  message type is first used after the initial FMT dump
- The FMT for FMT itself is always the first message in the file
- Message types 0-31 are vehicle-specific and may not be present
- Type 255 is reserved for future use
- The `streaming` flag in LogStructure is not present in the wire format

---

## 13. Existing Parsers (Reference Implementations)

### pymavlink DFReader.py
- Python, canonical reference parser
- Uses `struct.unpack("<...")` -- little-endian
- FORMAT_TO_STRUCT mapping handles type conversion
- Applies multipliers (0.01 for c/C/e/E, 1e-7 for L)
- Handles corrupt data with byte-by-byte resync
- Source: https://github.com/ArduPilot/pymavlink/blob/master/DFReader.py

### MAVExplorer / MAVProxy
- Built on pymavlink
- Adds graphing, FFT analysis, map display
- Source: https://github.com/ArduPilot/MAVProxy

### Mission Planner
- C# implementation
- Handles both .bin (DataFlash) and .tlog (MAVLink telemetry)
- Source: https://github.com/ArduPilot/MissionPlanner

### MATLAB ardupilotreader
- MathWorks built-in function for reading ArduPilot logs
- Documentation: https://www.mathworks.com/help/uav/ref/ardupilotreader.html

### dronecan/libcanard log tools
- Various C implementations in the ArduPilot ecosystem

---

## 14. Summary: Minimum Viable Parser

To parse a .bin file, you need:

1. **Hardcode FMT layout** (type=128, length=89, format="BBnNZ")
2. **Build format table** from FMT messages
3. **Scan for 0xA3 0x95** header, read msg_type
4. **Look up length** from format table, read payload
5. **Decode payload** using format string -> size mapping (little-endian)
6. **Apply multipliers** for c/C/e/E/L format chars if you want real values
7. **Optionally parse** UNIT/MULT/FMTU for rich unit metadata

For Betaflight-style analysis (PID tuning, vibration, motor analysis), the
essential messages are: **ATT, RATE, PIDR/P/Y, IMU, RCIN, RCOU, ESC, BARO,
GPS, BAT, MOTB, VIBE, MODE, EV, PARM**.
