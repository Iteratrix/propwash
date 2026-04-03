# PX4 ULog Binary Log Format Specification

Technical reference for writing a Rust parser. Derived from the official PX4 spec,
cross-referenced against the `pyulog` (Python) and `ulog_cpp` (C++) reference parsers,
and the PX4-Autopilot firmware logger source.

Sources:
- https://docs.px4.io/main/en/dev_log/ulog_file_format.html
- https://github.com/PX4/pyulog (core.py)
- https://github.com/PX4/ulog_cpp (raw_messages.hpp, messages.hpp/cpp, reader.cpp)
- https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/logger

---

## 1. Overall File Structure

```
+-----------------+
| File Header     |  16 bytes, fixed
+-----------------+
| Definitions     |  Variable length: Flag Bits, Format, Info, Multi-Info,
|   Section       |  Parameter, Default Parameter messages
+-----------------+
| Data Section    |  Variable length: Add Logged, Data, Logging, Dropout,
|                 |  Sync, Remove Logged, Parameter (changed) messages
+-----------------+
| Appended Data   |  Optional. Additional Data section messages appended
|   (optional)    |  after incomplete log close (crash recovery)
+-----------------+
```

**Byte order: Little-endian throughout.**

The file is a sequence of the 16-byte header followed by a stream of
length-prefixed messages. Each message starts with a 3-byte message header.
The Definitions section ends (and Data section begins) when the first
`'A'` (Add Logged), `'L'` (Logging), or `'C'` (Logging Tagged) message is encountered.

---

## 2. File Header

**Total size: 16 bytes. Always at file offset 0.**

```
Offset  Size  Type      Field       Description
------  ----  --------  ----------  -------------------------------------------
0       7     u8[7]     magic       Magic bytes: 0x55 0x4C 0x6F 0x67 0x01 0x12 0x35
                                    (ASCII "ULog" + 0x01 0x12 0x35)
7       1     u8        version     File format version (currently 0x01)
8       8     u64       timestamp   Microseconds since epoch (system start or
                                    UTC, depending on configuration)
```

**Pseudocode:**
```rust
let magic = &buf[0..7];
assert_eq!(magic, b"\x55\x4c\x6f\x67\x01\x12\x35");
let version = buf[7];
let timestamp = u64::from_le_bytes(buf[8..16].try_into().unwrap());
```

**Notes:**
- pyulog accepts version > 1 with a warning and attempts to parse anyway.
- ulog_cpp stores magic as `['U', 'L', 'o', 'g', 0x01, 0x12, 0x35]` (7 bytes),
  then `magic[7]` is the version byte (stored in the 8th byte of the `magic` array
  in the C struct, which is really the version field).

---

## 3. Message Header

Every message in the Definitions and Data sections is prefixed with this 3-byte header:

```
Offset  Size  Type   Field     Description
------  ----  -----  --------  -------------------------------------------
0       2     u16    msg_size  Payload size in bytes (EXCLUDES this 3-byte header)
2       1     u8     msg_type  Single ASCII character identifying the message type
```

**Total message size on disk = 3 + msg_size.**

**Important:** `msg_size` does NOT include the 3 header bytes. The next message
starts at `current_offset + 3 + msg_size`.

---

## 4. Message Types Summary

| Code | Char | Name                 | Section      | Description                        |
|------|------|----------------------|--------------|------------------------------------|
| 0x42 | 'B'  | FLAG_BITS            | Definitions  | Compat/incompat flags, offsets     |
| 0x46 | 'F'  | FORMAT               | Definitions  | Message format definition          |
| 0x49 | 'I'  | INFO                 | Both         | Key-value info (single)            |
| 0x4D | 'M'  | INFO_MULTIPLE        | Both         | Key-value info (multi-part)        |
| 0x50 | 'P'  | PARAMETER            | Both         | Parameter value (int32/float)      |
| 0x51 | 'Q'  | PARAMETER_DEFAULT    | Definitions  | Default parameter value            |
| 0x41 | 'A'  | ADD_LOGGED_MSG       | Data         | Subscribe: bind msg_id to format   |
| 0x52 | 'R'  | REMOVE_LOGGED_MSG    | Data         | Unsubscribe a msg_id               |
| 0x44 | 'D'  | DATA                 | Data         | Logged data payload                |
| 0x4C | 'L'  | LOGGING              | Data         | Logged string (untagged)           |
| 0x43 | 'C'  | LOGGING_TAGGED       | Data         | Logged string (tagged)             |
| 0x53 | 'S'  | SYNC                 | Data         | Sync marker for corruption recovery|
| 0x4F | 'O'  | DROPOUT              | Data         | Data loss indicator                |

**Parser rules:**
- Unknown message types: skip (warn), advance by `3 + msg_size`.
- Unknown file versions: parse anyway (warn).
- Unknown `incompat_flags` bits set: **refuse to parse** (fatal).
- Truncated final message: discard and stop.

---

## 5. Message Type Details

### 5.1 Flag Bits ('B' = 0x42)

**Must be the very first message after the file header** (at file offset 16).
If this message is not present, the parser should proceed without flags.

**Payload layout (msg_size = 40 typically):**

```
Offset  Size  Type       Field              Description
------  ----  ---------  -----------------  ------------------------------------
0       8     u8[8]      compat_flags       Compatible feature flags
8       8     u8[8]      incompat_flags     Incompatible feature flags
16      24    u64[3]     appended_offsets    File byte offsets for appended data
```

(Offsets are relative to the payload start, i.e., after the 3-byte message header.)

**compat_flags[0] bits:**
| Bit | Mask | Name               | Description                           |
|-----|------|--------------------|---------------------------------------|
| 0   | 0x01 | DEFAULT_PARAMETERS | Default parameter ('Q') messages exist|

**incompat_flags[0] bits:**
| Bit | Mask | Name           | Description                              |
|-----|------|----------------|------------------------------------------|
| 0   | 0x01 | DATA_APPENDED  | Appended data sections exist at offsets   |

**Rules:**
- If any unknown bits are set in `incompat_flags`, **stop parsing** -- the file
  uses features this parser cannot understand.
- Unknown bits in `compat_flags` can be safely ignored.
- `appended_offsets`: up to 3 file offsets where appended data sections begin.
  Zero means unused. When `DATA_APPENDED` is set, parse the main data section
  up to the first offset, then seek to each offset and parse as additional
  data sections.

**Hex example** (from `sample_appended_multiple.ulg`):
```
Offset 0x10: 28 00 42                              # msg_size=40, type='B'
             00 00 00 00 00 00 00 00                # compat_flags (all zero)
             01 00 00 00 00 00 00 00                # incompat_flags[0]=0x01 (DATA_APPENDED)
             c1 a0 06 00 00 00 00 00                # appended_offsets[0] = 0x0006A0C1
             f1 e4 06 00 00 00 00 00                # appended_offsets[1] = 0x0006E4F1
             21 29 07 00 00 00 00 00                # appended_offsets[2] = 0x00072921
```

---

### 5.2 Format Definition ('F' = 0x46)

Defines the schema for a message type. All formats must appear in the
Definitions section before any 'A' messages reference them.

**Payload:** A plain-text format string (no null terminator).

```
Offset  Size       Type     Field    Description
------  ---------  -------  -------  ------------------------------------------
0       msg_size   char[]   format   "name:type1 field1;type2 field2;..."
```

**Format string grammar:**
```
format_string = message_name ":" field_list
field_list    = field (";" field)* ";"
field         = type_spec " " field_name
type_spec     = basic_type | basic_type "[" array_len "]" | nested_name | nested_name "[" array_len "]"
message_name  = [a-zA-Z0-9_-/]+
field_name    = [a-zA-Z0-9_]+
array_len     = integer > 0
```

**Example:**
```
sensor_combined:uint64_t timestamp;float[3] gyro_rad;uint32_t gyro_integral_dt;int32_t accelerometer_timestamp_relative;float[3] accelerometer_m_s2;uint32_t accelerometer_integral_dt;uint8_t accelerometer_clipping;uint8_t gyro_clipping;uint8_t accel_calibration_count;uint8_t gyro_calibration_count;
```

**Parsing pseudocode:**
```rust
let (name, fields_str) = format.split_once(':').unwrap();
for field_str in fields_str.split(';') {
    if field_str.is_empty() { continue; }
    let (type_part, name) = field_str.split_once(' ').unwrap();
    if let Some(bracket) = type_part.find('[') {
        let base_type = &type_part[..bracket];
        let array_len: usize = type_part[bracket+1..type_part.len()-1].parse().unwrap();
        // array field
    } else {
        // scalar field
    }
}
```

**Special fields:**
- `timestamp` (uint64_t): **Required as the first field** of every message format.
  Monotonic microseconds. Every 'D' message begins with this 8-byte timestamp.
- `_padding[N]` (uint8_t[N]): Alignment padding. Parsers should **ignore** these.
  Trailing padding may be omitted from logged data (the payload can be shorter
  than the format's total size by the trailing padding amount).

---

### 5.3 Information Message ('I' = 0x49)

Stores a single key-value pair. Keys are unique (last value wins if duplicated).

**Payload layout:**

```
Offset  Size                    Type     Field    Description
------  ----------------------  -------  -------  --------------------------
0       1                       u8       key_len  Length of the key string
1       key_len                 char[]   key      "type_spec field_name"
1+kl    msg_size - 1 - key_len  u8[]     value    Binary value data
```

The `key` field uses the same `"type field_name"` format as format fields.
The value bytes are the binary representation of that type.

**Common keys in PX4 logs:**

| Key                          | Type           | Description                          |
|------------------------------|----------------|--------------------------------------|
| `char[N] sys_name`           | string         | System name ("PX4")                  |
| `char[N] ver_hw`             | string         | Hardware version string              |
| `char[N] ver_sw`             | string         | Git hash of firmware                 |
| `uint32_t ver_sw_release`    | u32            | Version: 0xAABBCCTT (major.minor.patch.type) |
| `char[N] sys_os_name`       | string         | OS name ("NuttX")                    |
| `char[N] sys_os_ver`        | string         | OS version string                    |
| `char[N] sys_toolchain`     | string         | Toolchain name                       |
| `char[N] sys_toolchain_ver` | string         | Toolchain version                    |
| `int32_t time_ref_utc`      | i32            | UTC time offset in seconds           |
| `uint64_t sys_startup_time` | u64            | System startup time                  |
| `char[N] replay`            | string         | Replay file name (if replay)         |
| `char[N] sys_uuid`          | string         | Board UUID                           |

**Version encoding (ver_sw_release):**
```
Byte 3 (MSB): major
Byte 2:       minor
Byte 1:       patch
Byte 0 (LSB): type  (0-63=dev, 64-127=alpha, 128-191=beta, 192-254=RC, 255=release)
```

---

### 5.4 Multi Information Message ('M' = 0x4D)

Like 'I' but allows multiple values for the same key, and values that span
multiple messages.

**Payload layout:**

```
Offset  Size                    Type     Field         Description
------  ----------------------  -------  ------------  --------------------------
0       1                       u8       is_continued  0 = new list item, 1 = continuation
1       1                       u8       key_len       Length of the key string
2       key_len                 char[]   key           "type_spec field_name"
2+kl    msg_size - 2 - key_len  u8[]     value         Binary value data
```

**Semantics:**
- `is_continued = 0`: Start a new entry in the list for this key.
- `is_continued = 1`: Append this value to the previous entry with the same key.

This is used for long strings that exceed a single message (e.g., replay file paths,
hardfault logs, etc.).

---

### 5.5 Parameter Message ('P' = 0x50)

Stores a parameter name and value. In the Definitions section, these are
initial parameter values. In the Data section, these represent parameter
changes (the current `last_timestamp` applies).

**Payload layout:**

```
Offset  Size                    Type     Field    Description
------  ----------------------  -------  -------  --------------------------
0       1                       u8       key_len  Length of the key string
1       key_len                 char[]   key      "type field_name"
1+kl    msg_size - 1 - key_len  u8[]     value    Binary value
```

**Allowed types:** Only `int32_t` (4 bytes) or `float` (4 bytes).

**Example key:** `"float MC_ROLLRATE_P"` followed by 4 bytes (IEEE 754 float, LE).

---

### 5.6 Default Parameter Message ('Q' = 0x51)

Like 'P' but stores the default value of a parameter, plus a bitfield
indicating which default category it belongs to.

**Payload layout:**

```
Offset  Size                    Type     Field          Description
------  ----------------------  -------  -------------  --------------------------
0       1                       u8       default_types  Bitmask of default categories
1       1                       u8       key_len        Length of key string
2       key_len                 char[]   key            "type field_name"
2+kl    msg_size - 2 - key_len  u8[]     value          Binary value
```

**default_types bits:**
| Bit | Value | Category      |
|-----|-------|---------------|
| 0   | 0x01  | system        |
| 1   | 0x02  | current_setup |

Both bits can be set simultaneously. Only present when `compat_flags[0] & 0x01`.

---

### 5.7 Add Logged Message ('A' = 0x41)

Creates a subscription: binds a `msg_id` to a format name and multi-instance ID.
Must appear before any 'D' messages referencing that `msg_id`.

**Payload layout:**

```
Offset  Size          Type     Field          Description
------  -----------   -------  -------------  -----------------------------------
0       1             u8       multi_id       Instance index (0 = first/default)
1       2             u16      msg_id         Unique subscription ID (sequential)
3       msg_size - 3  char[]   message_name   Format name (matches 'F' message name)
```

**Notes:**
- `msg_id` values start at 0 and increment sequentially.
- `multi_id` differentiates multiple instances of the same topic
  (e.g., `sensor_gyro` instance 0, 1, 2, 3).
- The `message_name` must match a format name from a prior 'F' message.

---

### 5.8 Remove Logged Message ('R' = 0x52)

Unsubscribes a previously added `msg_id`. Rare in practice.

**Payload layout:**

```
Offset  Size  Type   Field    Description
------  ----  -----  -------  ---------------------------
0       2     u16    msg_id   Subscription ID to remove
```

---

### 5.9 Data Message ('D' = 0x44)

The actual logged data. This is the most common message type in a file.

**Payload layout:**

```
Offset  Size          Type   Field   Description
------  -----------   -----  ------  ----------------------------------------
0       2             u16    msg_id  References a subscription from 'A'
2       msg_size - 2  u8[]   data    Binary payload matching the format def
```

The `data` bytes are laid out exactly according to the format definition
referenced by `msg_id` (via the 'A' message). Fields are packed in order,
with no additional padding between them (the format itself may include
`_padding` fields for alignment).

**The first 8 bytes of `data` are always `timestamp` (u64, microseconds).**

**Trailing padding:** The data payload may be shorter than the format's total
computed size. If so, the missing bytes are trailing `_padding` fields and
should be treated as zero.

**Data payload size:** `msg_size - 2` (subtracting the 2-byte msg_id).

**Pseudocode for reading a data message:**
```rust
let msg_id = u16::from_le_bytes(payload[0..2]);
let data = &payload[2..];
let subscription = subscriptions.get(&msg_id).unwrap();
let timestamp = u64::from_le_bytes(data[0..8]);
// Remaining fields at offsets defined by the format
```

---

### 5.10 Logging Message ('L' = 0x4C)

A logged string (from `PX4_INFO()`, `PX4_ERR()`, etc.).

**Payload layout:**

```
Offset  Size          Type     Field      Description
------  -----------   -------  ---------  ---------------------------
0       1             u8       log_level  ASCII '0'..'7' (0x30..0x37)
1       8             u64      timestamp  Microseconds
9       msg_size - 9  char[]   message    UTF-8 string (no null term)
```

**Log levels:**

| Value | Char | Level     |
|-------|------|-----------|
| 0x30  | '0'  | EMERGENCY |
| 0x31  | '1'  | ALERT     |
| 0x32  | '2'  | CRITICAL  |
| 0x33  | '3'  | ERROR     |
| 0x34  | '4'  | WARNING   |
| 0x35  | '5'  | NOTICE    |
| 0x36  | '6'  | INFO      |
| 0x37  | '7'  | DEBUG     |

---

### 5.11 Tagged Logging Message ('C' = 0x43)

Like 'L' but with an additional 16-bit tag for source identification.

**Payload layout:**

```
Offset  Size           Type     Field      Description
------  ------------   -------  ---------  ---------------------------
0       1              u8       log_level  ASCII '0'..'7'
1       2              u16      tag        Source identifier
3       8              u64      timestamp  Microseconds
11      msg_size - 11  char[]   message    UTF-8 string (no null term)
```

---

### 5.12 Sync Message ('S' = 0x53)

Contains 8 magic bytes. Used for recovery after corruption: a parser can
scan forward for this byte sequence to find a valid message boundary.

**Payload layout:**

```
Offset  Size  Type     Field       Description
------  ----  -------  ----------  ---------------------------
0       8     u8[8]    sync_magic  0x2F 0x73 0x13 0x20 0x25 0x0C 0xBB 0x12
```

`msg_size` = 8.

---

### 5.13 Dropout Message ('O' = 0x4F)

Indicates that the logger dropped data (could not write fast enough).

**Payload layout:**

```
Offset  Size  Type   Field     Description
------  ----  -----  --------  ---------------------------
0       2     u16    duration  Duration of data loss in milliseconds
```

`msg_size` = 2.

The dropout's timestamp is inferred from the last seen data message timestamp.

---

## 6. Type System

### 6.1 Primitive Types

| Type Name  | Size (bytes) | Rust Equivalent | struct pack |
|------------|:------------:|-----------------|:-----------:|
| `int8_t`   | 1            | `i8`            | `b`         |
| `uint8_t`  | 1            | `u8`            | `B`         |
| `int16_t`  | 2            | `i16`           | `h`         |
| `uint16_t` | 2            | `u16`           | `H`         |
| `int32_t`  | 4            | `i32`           | `i`         |
| `uint32_t` | 4            | `u32`           | `I`         |
| `int64_t`  | 8            | `i64`           | `q`         |
| `uint64_t` | 8            | `u64`           | `Q`         |
| `float`    | 4            | `f32`           | `f`         |
| `double`   | 8            | `f64`           | `d`         |
| `bool`     | 1            | `bool` (0/1)    | `?`         |
| `char`     | 1            | `u8` (ASCII)    | `c`         |

### 6.2 Arrays

Arrays are declared as `type[N]` where N is a positive integer.

- `float[3]` = 3 consecutive `float` values = 12 bytes
- `char[40]` = 40 bytes of character data (used as strings, no null terminator guaranteed)
- `uint8_t[5]` = 5 bytes

**Size in bytes = element_size * N.**

### 6.3 Nested Types

A field type that is not a recognized primitive is treated as a nested message reference.
The type name must match a format defined by a prior 'F' message.

Example format string:
```
vehicle_imu:uint64_t timestamp;sensor_accel_data accel;sensor_gyro_data gyro;...
```

Where `sensor_accel_data` is defined by its own 'F' message.

**Nested arrays** are also possible: `my_substruct[4] data` would be 4 consecutive
instances of the `my_substruct` format laid out sequentially.

**Resolution order:** All 'F' messages are in the Definitions section. Forward
references are allowed (type used before defined), but all must be resolved
by the end of the Definitions section. **Circular dependencies are not allowed.**

### 6.4 Field Size Calculation

```rust
fn field_size(field: &Field, formats: &HashMap<String, Format>) -> usize {
    let base_size = match field.type_name.as_str() {
        "int8_t" | "uint8_t" | "bool" | "char" => 1,
        "int16_t" | "uint16_t" => 2,
        "int32_t" | "uint32_t" | "float" => 4,
        "int64_t" | "uint64_t" | "double" => 8,
        nested => formats[nested].total_size(formats), // recursive
    };
    let count = field.array_length.unwrap_or(1);
    base_size * count
}
```

### 6.5 Padding

PX4 uses `_paddingN` fields (e.g., `uint8_t[5] _padding0`) for struct alignment.
These exist in the format definition but:
- Parsers should ignore their values
- **Trailing padding may be omitted** from the actual data payload. The data
  payload may be shorter than the computed format size by exactly the size
  of trailing padding fields.
- pyulog strips trailing `_padding` fields when computing the minimum expected
  data size.

---

## 7. Subscription Model

The subscription model maps `msg_id` values to format definitions:

```
'F' messages define schemas:    "sensor_combined:uint64_t timestamp;float[3] gyro_rad;..."
                                "vehicle_attitude:uint64_t timestamp;float[4] q;..."

'A' messages create subscriptions:
    msg_id=0, multi_id=0, name="sensor_combined"
    msg_id=1, multi_id=0, name="vehicle_attitude"
    msg_id=2, multi_id=0, name="actuator_outputs"
    msg_id=3, multi_id=1, name="sensor_gyro"     <- second instance

'D' messages reference subscriptions:
    msg_id=0, data=[timestamp, gyro_rad[3], ...]
    msg_id=1, data=[timestamp, q[4], ...]
```

**Parser state machine:**
1. Parse all 'F' messages -> build format registry (name -> field list)
2. Resolve nested types (compute offsets and total sizes)
3. On 'A' message -> create subscription: `msg_id` -> (format, multi_id, name)
4. On 'D' message -> look up subscription by `msg_id`, decode data using format
5. On 'R' message -> remove subscription for `msg_id`

---

## 8. Multi-Info Messages

The 'M' (INFO_MULTIPLE) message type allows:

1. **Multiple values for the same key** (like a list/array)
2. **Large values split across multiple messages** (continuation)

**Assembly algorithm:**

```
dict: HashMap<String, Vec<Vec<u8>>>

on message_info_multiple(key, value, is_continued):
    if !is_continued:
        dict.entry(key).or_default().push(value)  // new list item
    else:
        dict.get_mut(key).last_mut().extend(value) // append to last item
```

Common use cases:
- `char[N] hardfault_plain`: crash dump text (can be very long)
- `char[N] ver_sw_branch`: git branch name
- `char[N] perf_top`: performance counters

---

## 9. Appended Data Section

When a log file is not properly closed (e.g., crash, power loss), PX4 can append
additional data sections after the original end of file.

**How it works:**

1. The 'B' message's `incompat_flags[0]` bit 0 is set (`DATA_APPENDED`)
2. `appended_offsets[0..2]` contain up to 3 file byte offsets
3. Non-zero offsets point to the start of additional data sections
4. These sections contain valid Data section messages ('A', 'D', 'L', etc.)

**Parsing procedure:**

```rust
if incompat_flags[0] & 0x01 != 0 {
    let offsets: Vec<u64> = appended_offsets.iter()
        .filter(|&&o| o != 0)
        .copied()
        .collect();

    // Parse main data section up to the first appended offset
    parse_data_section(current_pos..offsets[0]);

    // Parse each appended section
    for offset in &offsets {
        seek_to(*offset);
        parse_data_section(*offset..next_offset_or_eof);
    }
}
```

**Important:** The main data section should be read only up to the first
appended offset (to avoid reading into potentially corrupt padding between
the incomplete original data and the appended data).

---

## 10. Timestamps

### 10.1 File Start Timestamp

The 8-byte timestamp in the file header. This is typically the time since
system boot in microseconds, or a UTC epoch timestamp depending on configuration.

### 10.2 Data Message Timestamps

Every data message ('D') payload begins with a `uint64_t timestamp` field
(the first field defined in every format). This is a **monotonic** timestamp
in microseconds since system start.

The `timestamp` field is always at offset 0 within the data payload
(offset 2 from the start of the 'D' message payload, after the `msg_id`).

### 10.3 Relative Timestamps

Some formats use `int32_t xxx_timestamp_relative` fields. These are offsets
in microseconds relative to the main `timestamp` field. A value of
`0x7FFFFFFF` (2147483647, `RELATIVE_TIMESTAMP_INVALID`) means the
associated sensor data is invalid.

### 10.4 Logging Timestamps

'L' and 'C' messages have their own embedded `uint64_t timestamp`.

### 10.5 Dropout Timestamps

'O' messages do NOT contain a timestamp. The dropout is associated with
the timestamp of the most recently parsed data message.

---

## 11. Corruption Recovery

### 11.1 Detection Heuristics

Both reference parsers use these heuristics to detect corruption:
- `msg_type == 0` or `msg_size == 0`: definitely corrupt
- `msg_size > 10000`: likely corrupt (normal messages are much smaller)
- Unknown `msg_type` that is not a valid ASCII letter: likely corrupt
- Data payload size doesn't match expected format size

### 11.2 Sync-Based Recovery

The parser can scan forward for the Sync message magic bytes:
`0x2F 0x73 0x13 0x20 0x25 0x0C 0xBB 0x12`

After finding sync bytes, the parser knows the next byte after the sync
sequence is a valid message header.

### 11.3 Heuristic Recovery (ulog_cpp)

The C++ parser scans byte-by-byte looking for a valid message header:
```
for each byte offset in buffer:
    header = read_header(offset)
    if header.msg_size != 0
       && header.msg_type != 0
       && header.msg_size < 10000
       && msg_type in KNOWN_TYPES:
        // possible recovery point
```

### 11.4 Truncated Files

If the file ends mid-message (fewer bytes available than `msg_size` indicates),
discard the partial message and stop parsing. This is normal for crash logs.

---

## 12. Key Message Names for Flight Analysis

These are the most important topic names logged by PX4 for flight analysis:

### 12.1 IMU / Sensors

| Topic Name               | Typical Rate | Key Fields                                            |
|--------------------------|:------------:|-------------------------------------------------------|
| `sensor_combined`        | full rate*   | `gyro_rad[3]`, `accelerometer_m_s2[3]`                |
| `sensor_gyro`            | 1 Hz (logged)| `x`, `y`, `z` (rad/s)                                 |
| `sensor_accel`           | 1 Hz (logged)| `x`, `y`, `z` (m/s^2)                                 |
| `sensor_mag`             | 1 Hz (logged)| `x`, `y`, `z` (gauss)                                 |
| `sensor_baro`            | 1 Hz (logged)| `pressure`, `temperature`                              |
| `vehicle_imu`            | 2 Hz (logged)| `delta_angle[3]`, `delta_velocity[3]`                  |

*`sensor_combined` has no interval specified = logged at full publish rate (typically 250-1000 Hz).

### 12.2 Attitude / Position

| Topic Name                    | Typical Rate | Key Fields                                      |
|-------------------------------|:------------:|-------------------------------------------------|
| `vehicle_attitude`            | 20 Hz        | `q[4]` (quaternion w,x,y,z), `delta_q_reset[4]`|
| `vehicle_attitude_setpoint`   | 20 Hz        | `q_d[4]` or roll/pitch/yaw setpoints            |
| `vehicle_angular_velocity`    | 50 Hz        | `xyz[3]` (rad/s body frame)                     |
| `vehicle_rates_setpoint`      | 50 Hz        | `roll`, `pitch`, `yaw` (rad/s)                  |
| `vehicle_local_position`      | 10 Hz        | `x`, `y`, `z`, `vx`, `vy`, `vz`                |
| `vehicle_global_position`     | 5 Hz         | `lat`, `lon`, `alt`                             |
| `vehicle_gps_position`        | 10 Hz        | `latitude_deg`, `longitude_deg`, `fix_type`     |

### 12.3 Actuators / Motors

| Topic Name               | Typical Rate | Key Fields                                     |
|--------------------------|:------------:|------------------------------------------------|
| `actuator_outputs`       | 10 Hz        | `output[16]` (PWM or normalized), `noutputs`   |
| `actuator_motors`        | 10 Hz        | Motor commands                                  |
| `actuator_servos`        | 10 Hz        | Servo commands                                  |
| `esc_status`             | 10 Hz        | ESC RPM, voltage, current per motor             |

### 12.4 Battery / Power

| Topic Name               | Typical Rate | Key Fields                                     |
|--------------------------|:------------:|------------------------------------------------|
| `battery_status`         | 5 Hz         | `voltage_v`, `current_a`, `remaining`, `cell_count` |
| `system_power`           | 2 Hz         | System voltage rails                            |

### 12.5 State / Mode

| Topic Name               | Typical Rate | Key Fields                                     |
|--------------------------|:------------:|------------------------------------------------|
| `vehicle_status`         | on change    | `arming_state`, `nav_state`                    |
| `vehicle_control_mode`   | on change    | `flag_armed`, `flag_control_*`                 |
| `vehicle_land_detected`  | on change    | `landed`, `freefall`                            |
| `commander_state`        | on change    | `main_state`                                    |
| `failsafe_flags`         | on change    | Failsafe condition flags                        |
| `input_rc`               | 2 Hz         | `values[N]`, `channel_count`, `rssi`            |
| `manual_control_setpoint`| 5 Hz         | `roll`, `pitch`, `yaw`, `throttle`              |

### 12.6 EKF / Estimation

| Topic Name                | Typical Rate | Key Fields                                    |
|---------------------------|:------------:|-----------------------------------------------|
| `estimator_status`        | 5 Hz         | `states[N]`, innovation test ratios            |
| `estimator_sensor_bias`   | 1 Hz         | `gyro_bias[3]`, `accel_bias[3]`, `mag_bias[3]`|

### 12.7 Typical Logging Rates

The `add_topic("name", interval_ms)` call specifies the **minimum interval**
between logged samples. A value of 0 or no interval means "log every update"
(full rate).

From the PX4 default profile:
- **Full rate (no throttle):** `sensor_combined`, `vehicle_command`, `parameter_update`
- **20 ms (50 Hz):** `vehicle_angular_velocity`, `vehicle_rates_setpoint`
- **50 ms (20 Hz):** `vehicle_attitude`, `vehicle_acceleration`
- **100 ms (10 Hz):** `vehicle_gps_position`, `vehicle_local_position`, `actuator_outputs`, `esc_status`
- **200 ms (5 Hz):** `vehicle_global_position`, `manual_control_setpoint`, `battery_status`, `vehicle_air_data`
- **500 ms (2 Hz):** `system_power`, `vehicle_imu`, `input_rc`
- **1000 ms (1 Hz):** `sensor_gyro`, `sensor_accel`, `sensor_mag`, `sensor_baro`, `wind`, `airspeed`

High-rate profile (for racing/acro analysis) logs `sensor_combined`,
`vehicle_angular_velocity`, `vehicle_attitude`, and `actuator_motors`
at full rate with no throttling.

---

## 13. Complete Binary Layout Examples

### 13.1 File Start (annotated hex dump)

From `sample_log_small.ulg`:

```
00000000: 55 4c 6f 67 01 12 35 01   ULog..5.     <- magic[7] + version=0x01
00000008: 5a e4 35 01 00 00 00 00                 <- timestamp=0x000000_0135E45A (20_374_618 us)

0000000d:                                         --- Messages begin ---

00000010: 28 00 42                                <- msg_size=40, type='B' (FLAG_BITS)
00000013: 00 00 00 00 00 00 00 00                 <- compat_flags[8] = all zeros
0000001b: 00 00 00 00 00 00 00 00                 <- incompat_flags[8] = all zeros
00000023: 00 00 00 00 00 00 00 00                 <- appended_offsets[0] = 0
0000002b: 00 00 00 00 00 00 00 00                 <- appended_offsets[1] = 0
00000033: 00 00 00 00 00 00 00 00                 <- appended_offsets[2] = 0

0000003b: 38 00 49                                <- msg_size=56, type='I' (INFO)
0000003e: 0f                                      <- key_len=15
0000003f: 63 68 61 72 5b 34 30 5d 20 76 65 72 5f 73 77  "char[40] ver_sw"
0000004e: 38 35 38 33 66 31 64 61 ...             <- value: "8583f1da..." (git hash)
```

### 13.2 Data Message Layout

For `sensor_combined` with format:
```
sensor_combined:uint64_t timestamp;float[3] gyro_rad;uint32_t gyro_integral_dt;
int32_t accelerometer_timestamp_relative;float[3] accelerometer_m_s2;
uint32_t accelerometer_integral_dt;uint8_t accelerometer_clipping;
uint8_t gyro_clipping;uint8_t accel_calibration_count;uint8_t gyro_calibration_count;
```

Assuming `msg_id=0` for this subscription:

```
3-byte header:
  XX XX 44              <- msg_size=XX, type='D'

2-byte msg_id:
  00 00                 <- msg_id=0 (sensor_combined)

Data payload (in field order):
  Offset  Size  Field
  0       8     timestamp (u64)
  8       12    gyro_rad[3] (3x f32)
  20      4     gyro_integral_dt (u32)
  24      4     accelerometer_timestamp_relative (i32)
  28      12    accelerometer_m_s2[3] (3x f32)
  40      4     accelerometer_integral_dt (u32)
  44      1     accelerometer_clipping (u8)
  45      1     gyro_clipping (u8)
  46      1     accel_calibration_count (u8)
  47      1     gyro_calibration_count (u8)
                Total: 48 bytes (+ possibly _padding stripped from end)
```

So `msg_size` = 2 (msg_id) + 48 (data) = 50 = 0x32.
Total bytes on disk: 3 (header) + 50 = 53 bytes per sample.

---

## 14. Parser State Machine

```
                  +-----------+
                  | ReadMagic |  Read 16 bytes, validate magic
                  +-----+-----+
                        |
                  +-----v-------+
                  | ReadFlagBits|  Check if first msg is 'B', parse if so
                  +-----+-------+
                        |
                  +-----v-------+
                  |  ReadHeader |  Parse 'F', 'I', 'M', 'P', 'Q' messages
                  |             |  Until 'A', 'L', or 'C' is encountered
                  +-----+-------+
                        |
                  +-----v------+
                  |  ReadData  |  Parse 'A', 'D', 'L', 'C', 'S', 'O', 'R',
                  |            |  'P' (changed params), 'I', 'M' messages
                  +-----+------+
                        |
                  +-----v------+
                  | HandleEOF  |  Discard partial message, finalize
                  +------------+
```

**Transition from ReadHeader to ReadData:**
The first 'A', 'L', or 'C' message signals the start of the Data section.
When this happens:
1. Resolve all nested type definitions (compute field offsets and sizes)
2. Seek back so the triggering message will be re-parsed in ReadData mode
3. Switch to ReadData state

**Note from pyulog:** When the transition-triggering message is encountered,
pyulog seeks backward by `3 + msg_size` bytes so the message is re-read
in the data section parser.

---

## 15. Test Fixtures

Three fixture files are provided in `propwash-core/tests/fixtures/px4/`:

| File                                           | Size    | Features Covered                              |
|------------------------------------------------|---------|-----------------------------------------------|
| `sample_log_small.ulg`                         | 900 KB  | Basic: formats, data, info, params            |
| `sample_appended_multiple.ulg`                 | 475 KB  | DATA_APPENDED flag, 3 appended offsets         |
| `sample_logging_tagged_and_default_params.ulg` | 1.9 MB  | DEFAULT_PARAMETERS flag, 'Q' + 'C' messages  |

These are sourced from the pyulog test suite (https://github.com/PX4/pyulog).

---

## 16. Implementation Notes for Rust

### 16.1 Endianness

All multi-byte values are little-endian. Use `u16::from_le_bytes()`,
`u64::from_le_bytes()`, `f32::from_le_bytes()`, etc.

### 16.2 String Handling

Strings are UTF-8 encoded with **no null terminator**. Length is always
determined from `msg_size` and field offsets, never from a null byte.
For `char[N]` arrays in format definitions, `strnlen` semantics apply
(the C++ parser truncates at the first null byte within the array).

### 16.3 Alignment

ULog data is **packed** (no alignment padding between fields). The
C/C++ structs use `#pragma pack(push, 1)`. In Rust, use
`#[repr(C, packed)]` for zero-copy parsing or just read field-by-field
from a byte slice.

### 16.4 Suggested Data Structures

```rust
struct ULogHeader {
    version: u8,
    timestamp: u64,
}

struct FlagBits {
    compat_flags: [u8; 8],
    incompat_flags: [u8; 8],
    appended_offsets: [u64; 3],
}

enum FieldType {
    Int8, UInt8, Int16, UInt16, Int32, UInt32,
    Int64, UInt64, Float, Double, Bool, Char,
    Nested(String),
}

struct FieldDef {
    name: String,
    field_type: FieldType,
    array_length: Option<usize>,  // None = scalar
    offset: usize,                // byte offset within message data
    size: usize,                  // total bytes (element_size * count)
}

struct FormatDef {
    name: String,
    fields: Vec<FieldDef>,
    size: usize,  // total data size in bytes
}

struct Subscription {
    msg_id: u16,
    multi_id: u8,
    format: Arc<FormatDef>,
    message_name: String,
}
```

### 16.5 Zero-Copy Considerations

For high-performance parsing, the data payload of 'D' messages can be
accessed as a borrowed byte slice without copying. Field values can be
read directly from the slice using the precomputed offsets from format
resolution.

### 16.6 Error Recovery Strategy

```rust
enum ParseAction {
    Continue,           // Valid message parsed
    Skip(usize),        // Skip N bytes (unknown message type)
    ScanForSync,        // Corruption detected, scan for sync magic
    Stop,               // Unrecoverable error or EOF
}
```

### 16.7 Multi-Instance Topics

Multiple sensors of the same type (e.g., 4 gyros) each get a separate
subscription with the same `message_name` but different `multi_id` values
(0, 1, 2, 3) and different `msg_id` values.

When extracting data, a topic is uniquely identified by the
`(message_name, multi_id)` tuple.
