# Betaflight Blackbox Binary Format Specification

Technical reference for implementing a parser. Derived from:
- https://betaflight.com/docs/development/Blackbox-Internals
- https://github.com/betaflight/betaflight/blob/master/src/main/blackbox/blackbox.c
- https://github.com/blackbox-log/blackbox-log (Rust crate)

Code examples are Python-style pseudocode. The reference implementation is
`propwash-core/src/format/bf/`.

---

## 1. File Structure

A `.bbl`/`.bfl` file contains one or more **log sessions** concatenated together.
Non-blackbox data (garbage, partial writes) may appear between sessions and must be skipped.

Each session:
```
[Log Start Marker]       -- ASCII text, exact string below
[Header Lines]           -- ASCII text, "H key:value\n" lines
[Binary Frame Data]      -- concatenated binary frames (I, P, S, G, H, E)
[Optional End Event]     -- E-frame with event type 0xFF
```

### Log Start Marker (Session Boundary)

Exact bytes (ASCII):
```
H Product:Blackbox flight data recorder by Nicholas Sherlock\n
```
This is always the first header line. To find sessions in a file, scan for all
occurrences of this byte sequence. Each occurrence marks the start of a new session.

### Data Version

The second header line must be:
```
H Data version:2\n
```

---

## 2. Header Format

Each header line is ASCII text:
```
H <space><name>:<value>\n
```

The header section ends when the next byte is NOT `H` (0x48). That byte is the
first frame marker of the binary data section.

### 2.1 Field Definition Headers

For each frame type, four (or six) CSV header lines define the fields:

**For main frames (I/P) -- 6 headers (delta fields):**
```
H Field I name:loopIteration,time,axisP[0],axisP[1],...
H Field I signed:0,0,1,1,...
H Field I predictor:0,0,0,0,...
H Field I encoding:1,1,0,0,...
H Field P predictor:6,2,1,1,...
H Field P encoding:9,0,0,0,...
```

The first 4 lines use the "main frame char" (I). The last 2 lines use the
"delta frame char" (P). This means I-frames and P-frames share the same field
names and signedness, but have DIFFERENT predictor and encoding per field.

**For simple frames (S, G, H) -- 4 headers:**
```
H Field S name:flightModeFlags,stateFlags,failsafePhase,...
H Field S signed:0,0,0,...
H Field S predictor:0,0,0,...
H Field S encoding:1,1,7,...
```

### 2.2 Field Name Format

Field names with indices use brackets: `axisP[0]`, `motor[3]`, `GPS_coord[0]`.
Fields with `fieldNameIndex == -1` have no brackets: `time`, `vbatLatest`.

### 2.3 System Info Headers (Key Metadata)

| Header Name | Format | Parser Relevance |
|---|---|---|
| `Firmware type` | String | "Betaflight", "INAV", etc. |
| `Firmware revision` | String | Version string |
| `I interval` | Integer | Loop iterations between I-frames |
| `P interval` | Integer | Loop iterations between P-frames |
| `P ratio` | Integer | I_interval / P_interval |
| `minthrottle` | Integer | For predictor ID 4 (legacy, now mostly unused) |
| `motorOutput` | "low,high" | For predictor ID 11 (MINMOTOR); low = motor idle value |
| `vbatref` | Integer | For predictor ID 9 (VBATREF) |
| `acc_1G` | Integer | Accelerometer scale factor |
| `gyro_scale` | Hex `0x%x` | Float-as-int-hex: `castFloatBytesToInt(1.0f)` |
| `looptime` | Integer | Loop period in microseconds |
| `debug_mode` | Integer | Debug mode enum value |
| `features` | Integer | Feature bitmask |
| `fields_disabled_mask` | Integer | Bitmask of disabled field groups |

---

## 3. Frame Types

Each binary frame starts with a single-byte ASCII marker:

| Marker | Byte | Name | Role |
|--------|------|------|------|
| `I` | 0x49 | Intraframe | Keyframe, self-contained, no history needed |
| `P` | 0x50 | Interframe | Delta frame, requires previous I + P frames |
| `S` | 0x53 | Slow | Infrequently-changing state (flight modes, flags) |
| `G` | 0x47 | GPS | GPS position/velocity data |
| `H` | 0x48 | GPS Home | GPS home reference point |
| `E` | 0x45 | Event | Discrete events (sync beep, disarm, etc.) |

Frames have NO length prefix, NO checksum, NO end delimiter. The parser must
decode each field according to the header definitions and trust that the next
byte after a frame is another valid frame marker.

### 3.1 Frame Validation

On encountering invalid data:
1. Check that the byte after a decoded frame is a valid marker (I/P/S/G/H/E)
2. Check that `loopIteration` and `time` don't jump unreasonably
3. On error, scan forward for the next valid marker byte
4. After sync loss, discard all P-frames until the next I-frame

### 3.2 Frame Scheduling

Given `I_interval` and `P_interval` from headers:
```python
def frame_type(iteration: int) -> str:
    if iteration % I_interval == 0:
        return 'I'
    if iteration % P_interval == 0:
        return 'P'
    return None  # not logged
```

---

## 4. Encoding Schemes

### Encoding ID Table

| ID | Name | Grouped? | Description |
|----|------|----------|-------------|
| 0 | SIGNED_VB | No | ZigZag + unsigned variable byte |
| 1 | UNSIGNED_VB | No | Unsigned variable byte |
| 3 | NEG_14BIT | No | Negate, mask to 14 bits, unsigned VB |
| 6 | TAG8_8SVB | Yes (up to 8) | 1-byte bitmask + signed VB for non-zero fields |
| 7 | TAG2_3S32 | Yes (exactly 3) | 2-bit tag + 3 packed signed 32-bit values |
| 8 | TAG8_4S16 | Yes (exactly 4) | 8-bit tag + 4 packed signed 16-bit values |
| 9 | NULL | No | No bytes written; value is always 0 |
| 10 | TAG2_3SVARIABLE | Yes (exactly 3) | 2-bit tag + 3 values with asymmetric bit widths |

**Grouped encodings**: When consecutive fields share the same encoding ID, they
are read together as a single group. For TAG2_3S32: groups of exactly 3.
For TAG8_4S16: groups of exactly 4. For TAG8_8SVB: groups of up to 8.

### 4.1 Unsigned Variable Byte (ID: 1)

Standard LEB128-style encoding. Each byte stores 7 data bits in bits [6:0].
Bit 7 = 1 means "more bytes follow". Bit 7 = 0 means "this is the last byte".

**Encoding (write):**
```python
def write_unsigned_vb(value: int) -> bytes:
    result = bytearray()
    while value > 127:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value)
    return bytes(result)
```

**Decoding (read):**
```python
def read_unsigned_vb(stream) -> int:
    result = 0
    offset = 0
    while True:
        byte = stream.read(1)[0]
        result |= (byte & 0x7F) << offset
        offset += 7
        if (byte & 0x80) == 0:
            break
        if offset >= 32:
            raise ValueError("VB overflow")
    return result
```

**Examples:**
- 0 -> `[0x00]` (1 byte)
- 127 -> `[0x7F]` (1 byte)
- 128 -> `[0x80, 0x01]` (2 bytes)
- 16384 -> `[0x80, 0x80, 0x01]` (3 bytes)

### 4.2 Signed Variable Byte (ID: 0)

ZigZag-encode the signed value to an unsigned value, then write as unsigned VB.

**ZigZag encoding:**
```python
def zigzag_encode(value: int) -> int:
    return (value << 1) ^ (value >> 31)  # arithmetic right shift
```

**ZigZag decoding:**
```python
def zigzag_decode(value: int) -> int:
    return (value >> 1) ^ -(value & 1)
```

**Mapping:**
| Signed | Unsigned |
|--------|----------|
| 0 | 0 |
| -1 | 1 |
| 1 | 2 |
| -2 | 3 |
| 2 | 4 |
| -3 | 5 |

### 4.3 NEG_14BIT (ID: 3)

Used for battery voltage in I-frames. The value is `vbatref - vbatLatest`,
which is typically positive (voltage drops). The firmware writes:
```c
blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
```

**Decoding:**
```python
def read_neg_14bit(stream) -> int:
    raw = read_unsigned_vb(stream) & 0xFFFF  # treat as u16
    if raw & 0x2000:  # bit 13 set = sign extension needed
        result = int.from_bytes((raw | 0xC000).to_bytes(2, 'little', signed=False), 'little', signed=True)
    else:
        result = raw
    return -result
```

The result is the negated 14-bit signed value. Since the predictor is VBATREF
and the value stored is `(vbatref - actual) & 0x3FFF`, after applying the
predictor (`decoded + vbatref`), you get the actual voltage.

### 4.4 TAG8_8SVB (ID: 6)

Encodes up to 8 signed values. Most values are expected to be zero.

**Special case: if the group has exactly 1 field**, no header byte is written;
the single value is encoded directly as signed VB.

**Normal case (2-8 fields):**
1. Read 1 header byte. Each bit (LSB = field 0) indicates non-zero (1) or zero (0).
2. For each bit that is 1, read a signed VB value.
3. Bits that are 0 produce value 0.
4. After reading all expected fields, remaining header bits must be 0 (else invalid frame).

**Decoding:**
```python
def read_tag8_8svb(stream, count: int) -> list[int]:
    if count == 1:
        return [read_signed_vb(stream)]

    header = stream.read(1)[0]
    values = []
    for i in range(count):
        if header & (1 << i):
            values.append(read_signed_vb(stream))
        else:
            values.append(0)
    # Verify no extra bits set
    if header >> count:
        raise ValueError("Extra bits in TAG8_8SVB header")
    return values
```

### 4.5 TAG2_3S32 (ID: 7)

Packs exactly 3 signed 32-bit values. A 2-bit selector in bits [7:6] of the
first byte chooses the packing format:

**Selector 0 -- 2 bits per field (1 byte total):**
```
Byte: SS AA BB CC
      [7:6]=selector  [5:4]=value0  [3:2]=value1  [1:0]=value2
```
Each 2-bit field is sign-extended: range [-2, 1].

**Selector 1 -- 4 bits per field (2 bytes total):**
```
Byte 0: SS AAAA        [7:6]=selector  [3:0]=value0
Byte 1: BBBB CCCC      [7:4]=value1    [3:0]=value2
```
Each 4-bit field is sign-extended: range [-8, 7].

**Selector 2 -- 6 bits per field (3 bytes total):**
```
Byte 0: SS AAAAAA      [7:6]=selector  [5:0]=value0
Byte 1: XX BBBBBB      [5:0]=value1 (upper 2 bits ignored per Rust impl)
Byte 2: XX CCCCCC      [5:0]=value2 (upper 2 bits ignored per Rust impl)
```
Each 6-bit field is sign-extended: range [-32, 31].

Note: The C encoder writes full bytes for values 1 and 2 (`(uint8_t)values[x]`),
meaning all 8 bits are present. But only the low 6 bits carry the value; the
decoder should mask with 0x3F and sign-extend from 6 bits.

**Selector 3 -- Variable bytes per field (1 + N bytes):**
```
Byte 0: 11 TTTTTT      [7:6]=0b11  [5:0]=3 sub-selectors (2 bits each)
```
Sub-selector layout in bits [5:0]: `CC BB AA` (field 0 in lowest bits).
Each 2-bit sub-selector:
- 0 = 1 byte (signed i8, range [-128, 127])
- 1 = 2 bytes (signed i16 LE, range [-32768, 32767])
- 2 = 3 bytes (i24 LE, sign-extended from 24 bits)
- 3 = 4 bytes (signed i32 LE)

Multi-byte values are **little-endian** (low byte first).

**Decoding pseudocode:**
```python
def read_tag2_3s32(stream) -> list[int]:
    byte0 = stream.read(1)[0]
    selector = (byte0 & 0xC0) >> 6

    if selector == 0:
        return [sign_extend((byte0 >> 4) & 3, 2),
                sign_extend((byte0 >> 2) & 3, 2),
                sign_extend(byte0 & 3, 2)]
    elif selector == 1:
        v0 = sign_extend(byte0 & 0x0F, 4)
        byte1 = stream.read(1)[0]
        v1 = sign_extend((byte1 >> 4) & 0x0F, 4)
        v2 = sign_extend(byte1 & 0x0F, 4)
        return [v0, v1, v2]
    elif selector == 2:
        v0 = sign_extend(byte0 & 0x3F, 6)
        v1 = sign_extend(stream.read(1)[0] & 0x3F, 6)
        v2 = sign_extend(stream.read(1)[0] & 0x3F, 6)
        return [v0, v1, v2]
    else:  # selector == 3
        tags = byte0 & 0x3F
        values = []
        for _ in range(3):
            tag = tags & 3
            tags >>= 2
            if tag == 0:
                values.append(int.from_bytes(stream.read(1), 'little', signed=True))
            elif tag == 1:
                values.append(int.from_bytes(stream.read(2), 'little', signed=True))
            elif tag == 2:
                raw = int.from_bytes(stream.read(3), 'little', signed=False)
                values.append(sign_extend(raw, 24))
            else:  # tag == 3
                values.append(int.from_bytes(stream.read(4), 'little', signed=True))
        return values

def sign_extend(value: int, bits: int) -> int:
    if value & (1 << (bits - 1)):
        value -= (1 << bits)
    return value
```

### 4.6 TAG8_4S16 (ID: 8)

Packs exactly 4 signed 16-bit values using a bitstream approach.

**Format:**
1. Read 1 tag byte. 2 bits per field (field 0 in bits [1:0]).
2. Each 2-bit selector:
   - 0 = zero (no data bits)
   - 1 = 4-bit signed nibble (range [-8, 7])
   - 2 = 8-bit signed byte (range [-128, 127])
   - 3 = 16-bit signed (range [-32768, 32767])

**Bitstream packing:** Values are packed as a **nibble-aligned bitstream**, MSB-first:

- 4-bit values occupy one nibble
- 8-bit values occupy two nibbles (high nibble first)
- 16-bit values occupy four nibbles (high byte first, MSB)
- Data is packed sequentially with no gaps between fields
- If the total nibble count is odd, the final nibble is padded (low nibble of last byte)

**Critical detail from C source:** The encoder maintains a `nibbleIndex` and `buffer`.
When `nibbleIndex == 0`, the current field's high bits go into the buffer's high nibble.
When `nibbleIndex == 1`, the current field's bits are combined with the buffered nibble.

For 16-bit fields: when aligned, writes `[high_byte, low_byte]`. When unaligned,
the high 4 bits merge with the buffer, then middle 8 bits, then low 4 bits go into buffer.

**Decoding (from Rust crate):**
```python
def read_tag8_4s16(stream) -> list[int]:
    tags = stream.read(1)[0]
    if tags == 0:
        return [0, 0, 0, 0]

    result = [0, 0, 0, 0]
    aligned = True
    buffer = 0

    for i in range(4):
        tag = (tags >> (i * 2)) & 3
        if tag == 0:
            result[i] = 0
        elif tag == 1:  # 4-bit nibble
            if aligned:
                buffer = stream.read(1)[0]
                nibble = buffer >> 4
            else:
                nibble = buffer & 0x0F
            aligned = not aligned
            result[i] = sign_extend(nibble, 4)
        elif tag == 2:  # 8-bit
            if aligned:
                result[i] = int.from_bytes(stream.read(1), 'big', signed=True)
            else:
                upper = (buffer & 0x0F) << 4
                buffer = stream.read(1)[0]
                result[i] = int.from_bytes(bytes([(upper | (buffer >> 4)) & 0xFF]), 'big', signed=True)
        else:  # tag == 3: 16-bit
            if aligned:
                # Big-endian (high byte first) then swap => read as big-endian i16
                raw = stream.read(2)
                result[i] = int.from_bytes(raw, 'big', signed=True)
            else:
                upper = (buffer & 0x0F) << 12
                raw = stream.read(2)
                middle = raw[0]
                lower = raw[1]
                buffer = lower
                result[i] = sign_extend_16(upper | (middle << 4) | (lower >> 4))
                # Note: aligned stays False
                continue  # don't flip aligned
            # aligned stays True after full 16-bit read
    return result
```

**IMPORTANT NOTE on byte order**: The C encoder writes 16-bit values as `[high_byte, low_byte]`
(big-endian), which differs from the little-endian convention used by TAG2_3S32.
The Rust decoder calls `.swap_bytes()` on aligned i16 reads, confirming big-endian wire format.

### 4.7 TAG2_3SVARIABLE (ID: 10)

Similar to TAG2_3S32 but with **asymmetric bit widths** per selector.
Used less commonly. Same 2-bit selector in bits [7:6]:

**Selector 0 -- 2 bits per field (1 byte):** Same as TAG2_3S32 selector 0.
```
Byte: SS AA BB CC    (identical layout)
```

**Selector 1 -- 5/5/4 bits (2 bytes):**
```
Byte 0: SS AAAAA B     [7:6]=selector, [5:1]=value0(5 bits), [0]=value1 bit4
Byte 1: BBBB CCCC      [7:4]=value1 bits[3:0], [3:0]=value2(4 bits)
```
Encoding from C: `(selector << 6) | ((values[0] & 0x1F) << 1) | ((values[1] & 0x1F) >> 4)`
Then: `((values[1] & 0x0F) << 4) | (values[2] & 0x0F)`

Ranges: value0 [-16,15], value1 [-16,15], value2 [-8,7]

**Selector 2 -- 8/7/7 bits (3 bytes):**
```
Byte 0: SS AAAAAA      [7:6]=selector, [5:0]=value0 bits[7:2]
Byte 1: AA BBBBBB B    [7:6]=value0 bits[1:0], [5:1]= value1 bits[6:1], [0]=value1 bit0...
```
Encoding from C:
```
byte0 = (selector << 6) | ((values[0] & 0xFF) >> 2)
byte1 = ((values[0] & 0x03) << 6) | ((values[1] & 0x7F) >> 1)
byte2 = ((values[1] & 0x01) << 7) | (values[2] & 0x7F)
```
Ranges: value0 [-256,255], value1 [-128,127], value2 [-128,127]

**Selector 3 -- Variable bytes:** Identical to TAG2_3S32 selector 3.

### 4.8 NULL (ID: 9)

No bytes are read or written. The decoded value is always 0. Used when the
predictor perfectly predicts the value (e.g., `loopIteration` in P-frames
uses predictor INCREMENT, which always adds 1, so the residual is always 0).

---

## 5. Predictor Types

Before encoding, the firmware computes a **predicted** value and stores only
the **residual** (actual - predicted). The decoder must reverse this:
`actual = decoded_residual + predicted`.

| ID | Name | Prediction Formula | Used In |
|----|------|-------------------|---------|
| 0 | ZERO | `0` | I-frames (raw value stored as-is) |
| 1 | PREVIOUS | `history[n-1]` | P-frames for most fields |
| 2 | STRAIGHT_LINE | `2 * history[n-1] - history[n-2]` | P-frame `time` field |
| 3 | AVERAGE_2 | `(history[n-1] + history[n-2]) / 2` | P-frame gyro, acc, motor, debug |
| 4 | MINTHROTTLE | `header["minthrottle"]` | I-frame motor[0] (legacy) |
| 5 | MOTOR_0 | `motor[0]` from current frame | I-frame motor[1..N] |
| 6 | INCREMENT | `previous + 1` | P-frame loopIteration |
| 7 | HOME_COORD | GPS home lat/lon from H-frame | G-frame GPS_coord |
| 8 | 1500 | `1500` | I-frame servo fields |
| 9 | VBATREF | `header["vbatref"]` | I-frame vbatLatest |
| 10 | LAST_MAIN_FRAME_TIME | time from previous main frame | G-frame time |
| 11 | MINMOTOR | `header["motorOutput"].split(",")[0]` | I-frame motor[0] |

### Predictor Application

**For I-frames:** `actual = decoded + predictor`
- Predictor 0: `actual = decoded`
- Predictor 4/11: `actual = decoded + minthrottle/minmotor`
- Predictor 5: `actual = decoded + motor[0]` (already decoded in this frame)
- Predictor 8: `actual = decoded + 1500`
- Predictor 9: `actual = decoded + vbatref`

**For P-frames:** `actual = decoded + predictor`
- Predictor 1: `actual = decoded + previous_value`
- Predictor 2: `actual = decoded + (2 * prev1 - prev2)` (straight line extrapolation)
- Predictor 3: `actual = decoded + (prev1 + prev2) // 2` (integer division)
- Predictor 6: `actual = decoded + (previous + 1)` (but encoded as NULL, so decoded=0, actual = previous + 1)

### History Management

The firmware maintains a ring buffer of 3 states: `history[0]` = current,
`history[1]` = previous, `history[2]` = two frames ago.

After an **I-frame**: `history[1] = history[2] = history[0]` (no prior state).
After a **P-frame**: `history[2] = history[1]; history[1] = history[0]`.

---

## 6. Frame Decode Details

### 6.1 I-Frame

Marker byte: `I` (0x49)

Fields are encoded sequentially using the I-predictor and I-encoding from the
header definitions. Each field is independent (no grouping dependencies beyond
what the encoding specifies).

Typical field order (depends on active conditions):
1. `loopIteration` - unsigned VB, predictor 0
2. `time` - unsigned VB, predictor 0
3. `axisP[0..2]` - signed VB, predictor 0
4. `axisI[0..2]` - signed VB, predictor 0
5. `axisD[0..2]` - signed VB, predictor 0 (only if PID D != 0)
6. `axisF[0..2]` - signed VB, predictor 0
7. `rcCommand[0..2]` - signed VB, predictor 0
8. `rcCommand[3]` (throttle) - unsigned VB, predictor 0
9. `setpoint[0..3]` - signed VB, predictor 0
10. `vbatLatest` - NEG_14BIT, predictor VBATREF
11. `amperageLatest` - signed VB, predictor 0
12. `magADC[0..2]` - signed VB, predictor 0 (if mag present)
13. `baroAlt` - signed VB, predictor 0 (if baro present)
14. `surfaceRaw` - signed VB, predictor 0 (if rangefinder)
15. `rssi` - unsigned VB, predictor 0
16. `gyroADC[0..2]` - signed VB, predictor 0
17. `gyroUnfilt[0..2]` - signed VB, predictor 0
18. `accSmooth[0..2]` - signed VB, predictor 0 (if acc present)
19. `imuQuaternion[0..2]` - signed VB, predictor 0 (if attitude)
20. `debug[0..7]` - signed VB, predictor 0
21. `motor[0]` - unsigned VB, predictor MINMOTOR
22. `motor[1..N]` - signed VB, predictor MOTOR_0
23. `servo[0..7]` - TAG8_8SVB, predictor 1500 (if servos)
24. `eRPM[0..N]` - unsigned VB, predictor 0 (if DSHOT telemetry)

**The actual field list is determined entirely by the header definitions.**
The parser should not hardcode this order -- read the `Field I name/signed/predictor/encoding`
headers and decode accordingly.

### 6.2 P-Frame

Marker byte: `P` (0x50)

Uses the P-predictor and P-encoding columns from the header definitions.
Key differences from I-frames:
- `loopIteration` uses NULL encoding (no bytes; increment by 1)
- `time` uses STRAIGHT_LINE predictor + signed VB
- PID I terms use TAG2_3S32 (packed 3 values)
- RC commands use TAG8_4S16 (packed 4 values)
- Battery/sensor fields use TAG8_8SVB (bitmask grouping)
- Gyro/acc/motor use AVERAGE_2 predictor + signed VB

### 6.3 S-Frame (Slow)

Marker byte: `S` (0x53)

Always uses "simple" (intra) encoding -- no delta from previous S-frame.
Typical fields:
1. `flightModeFlags` - unsigned VB
2. `stateFlags` - unsigned VB
3. `failsafePhase`, `rxSignalReceived`, `rxFlightChannelsValid` - TAG2_3S32 (packed)

### 6.4 H-Frame (GPS Home)

Marker byte: `H` (0x48)

**WARNING:** This collides with header lines (which also start with `H`).
In the binary section, `H` is only valid as a GPS home frame marker.

Fields:
1. `GPS_home[0]` (latitude) - signed VB
2. `GPS_home[1]` (longitude) - signed VB
3. `GPS_home[2]` (altitude in 0.1m) - signed VB
4. `GPS_home_epoch` - unsigned VB

### 6.5 G-Frame (GPS Data)

Marker byte: `G` (0x47)

Fields:
1. `time` - unsigned VB (only if NOT logging every frame; predictor LAST_MAIN_FRAME_TIME)
2. `GPS_numSat` - unsigned VB
3. `GPS_coord[0]` (lat) - signed VB, predictor HOME_COORD (delta from GPS_home lat)
4. `GPS_coord[1]` (lon) - signed VB, predictor HOME_COORD (delta from GPS_home lon)
5. `GPS_altitude` - signed VB (in 0.1m increments)
6. `GPS_speed` - unsigned VB
7. `GPS_ground_course` - unsigned VB
8. `GPS_velned[0..2]` (N/E/D) - signed VB
9. `GPS_time` - unsigned VB (epoch time)

### 6.6 E-Frame (Event)

Marker byte: `E` (0x45), followed by event type byte.

```
[0x45] [event_type: u8] [payload...]
```

| Event ID | Name | Payload |
|----------|------|---------|
| 0 | SYNC_BEEP | `time: unsigned_vb` (u32 microseconds) |
| 13 | INFLIGHT_ADJUSTMENT | `function: u8`, then if `function & 0x80`: `value: f32` (4 bytes LE), else `value: signed_vb` |
| 14 | LOGGING_RESUME | `logIteration: unsigned_vb`, `currentTime: unsigned_vb` |
| 15 | DISARM | `reason: unsigned_vb` |
| 30 | FLIGHT_MODE | `flags: unsigned_vb`, `lastFlags: unsigned_vb` |
| 40 | IMU_FAILURE | `error_code: unsigned_vb` (Rust crate only, may not be in all firmware) |
| 255 | LOG_END | `"End of log\0"` (null-terminated ASCII string) |

**INFLIGHT_ADJUSTMENT detail:**
- Read 1 byte for `adjustmentFunction`
- If bit 7 is set: strip bit 7, read next 4 bytes as IEEE 754 float (little-endian)
- If bit 7 is clear: read next value as signed VB integer

**LOG_END detail:**
The firmware writes: `blackboxWriteString("End of log"); blackboxWrite(0);`
This produces the ASCII bytes `End of log` followed by a null byte.
The Rust crate also looks for an optional `(disarm reason:X)` suffix.

---

## 7. Complete Decode Pipeline

For each field in a frame:

```
1. Read raw value using the field's ENCODING
   - For grouped encodings, read all fields in the group at once

2. Compute the PREDICTOR value:
   - Use the appropriate history and header values

3. actual_value = raw_decoded + predictor_value

4. Store actual_value in history for future predictions
```

### 7.1 Grouped Encoding Handling

When parsing fields sequentially, detect runs of consecutive fields with the
same encoding and decode them as a group:

- **TAG2_3S32 / TAG2_3SVARIABLE**: Read 3 values at once, assign to 3 consecutive fields
- **TAG8_4S16**: Read 4 values at once, assign to 4 consecutive fields
- **TAG8_8SVB**: Count consecutive fields with this encoding (max 8), read group

If fewer fields remain than the group size (e.g., only 2 fields left but encoding
is TAG2_3S32), the behavior is undefined in the spec. The firmware guarantees
correct grouping in practice.

### 7.2 Important Edge Cases

1. **TAG8_8SVB with 1 field**: No header byte; reads directly as signed VB.
2. **NEG_14BIT predictor chain**: In I-frames with VBATREF predictor, the stored
   value is `(vbatref - actual) & 0x3FFF`. The neg_14bit decoder negates this,
   giving `-(vbatref - actual) = actual - vbatref`. Adding the VBATREF predictor
   gives: `(actual - vbatref) + vbatref = actual`. Correct.
3. **Motor predictor in I-frames**: motor[0] uses MINMOTOR predictor. motor[1..N]
   use MOTOR_0 predictor (= the already-decoded motor[0] value from this same frame).
   Fields must be decoded in order for this to work.
4. **P-frame loopIteration**: Uses INCREMENT predictor + NULL encoding. This means
   no bytes are consumed, and the value is `previous_loopIteration + 1`.
5. **TAG8_4S16 byte order is BIG-ENDIAN** while TAG2_3S32 byte order is LITTLE-ENDIAN.
6. **H frame marker collision**: In the binary section, an `H` byte could be either
   a GPS Home frame or the start of a new log session's header. Disambiguate by
   checking if the following bytes look like a header line (`H Product:...`).

---

## 8. Data Types Summary

```python
# All values are 32-bit integers internally
# Signedness is tracked per-field in the header (0 = unsigned, 1 = signed)
# The encoding determines how to read bytes
# The predictor determines how to reconstruct the actual value
# Float values only appear in INFLIGHT_ADJUSTMENT events (4 bytes IEEE 754 LE)
```

---

## 9. Quick Reference: Typical Default Field Definitions

### I/P Main Frame Fields

| Field | Signed | I-Pred | I-Enc | P-Pred | P-Enc |
|-------|--------|--------|-------|--------|-------|
| loopIteration | U | 0 | UNSIGNED_VB(1) | INC(6) | NULL(9) |
| time | U | 0 | UNSIGNED_VB(1) | STRAIGHT_LINE(2) | SIGNED_VB(0) |
| axisP[0..2] | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | SIGNED_VB(0) |
| axisI[0..2] | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | TAG2_3S32(7) |
| axisD[0..2] | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | SIGNED_VB(0) |
| axisF[0..2] | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | SIGNED_VB(0) |
| rcCommand[0..2] | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | TAG8_4S16(8) |
| rcCommand[3] | U | 0 | UNSIGNED_VB(1) | PREVIOUS(1) | TAG8_4S16(8) |
| setpoint[0..3] | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | TAG8_4S16(8) |
| vbatLatest | U | VBATREF(9) | NEG_14BIT(3) | PREVIOUS(1) | TAG8_8SVB(6) |
| amperageLatest | S | 0 | SIGNED_VB(0) | PREVIOUS(1) | TAG8_8SVB(6) |
| gyroADC[0..2] | S | 0 | SIGNED_VB(0) | AVERAGE_2(3) | SIGNED_VB(0) |
| motor[0] | U | MINMOTOR(11) | UNSIGNED_VB(1) | AVERAGE_2(3) | SIGNED_VB(0) |
| motor[1..N] | U | MOTOR_0(5) | SIGNED_VB(0) | AVERAGE_2(3) | SIGNED_VB(0) |
| debug[0..7] | S | 0 | SIGNED_VB(0) | AVERAGE_2(3) | SIGNED_VB(0) |
| servo[0..7] | U | 1500(8) | TAG8_8SVB(6) | PREVIOUS(1) | TAG8_8SVB(6) |
| eRPM[0..N] | U | 0 | UNSIGNED_VB(1) | PREVIOUS(1) | SIGNED_VB(0) |

### Slow Frame Fields

| Field | Signed | Pred | Enc |
|-------|--------|------|-----|
| flightModeFlags | U | 0 | UNSIGNED_VB(1) |
| stateFlags | U | 0 | UNSIGNED_VB(1) |
| failsafePhase | U | 0 | TAG2_3S32(7) |
| rxSignalReceived | U | 0 | TAG2_3S32(7) |
| rxFlightChannelsValid | U | 0 | TAG2_3S32(7) |
