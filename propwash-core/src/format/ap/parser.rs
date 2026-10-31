use std::collections::HashMap;

use az::{Az, SaturatingAs};

use crate::types::Warning;

use super::types::{ApMsgDef, ApParseStats, FieldType, IsbBatch, MsgColumns};

/// Output of the AP parser: just the columnar intermediate that
/// [`super::build`] folds into a typed Session.
pub(crate) struct ApParsed {
    pub msg_defs: HashMap<u8, ApMsgDef>,
    pub topics: HashMap<u8, MsgColumns>,
    pub firmware_version: String,
    pub vehicle_name: String,
    pub params: HashMap<String, f64>,
    /// Complete batch-sampler bursts, in log order.
    pub batches: Vec<IsbBatch>,
    pub stats: ApParseStats,
}

const HEAD1: u8 = 0xA3;
const HEAD2: u8 = 0x95;
const FMT_TYPE: u8 = 128;
const FMT_LEN: usize = 89;

// ---------------------------------------------------------------------------
// Pre-computed field layout
// ---------------------------------------------------------------------------

struct FieldSlot {
    byte_offset: usize,
    field_type: FieldType,
}

struct MsgLayout {
    /// Slots for column storage (excludes timestamp field).
    slots: Vec<FieldSlot>,
    /// Field names parallel to `slots`.
    field_names: Vec<String>,
    /// Byte offset of the timestamp field.
    timestamp_byte_offset: usize,
    /// True if timestamp is `TimeMS` (needs ×1000 conversion to microseconds).
    timestamp_is_ms: bool,
    /// Byte offsets of ALL fields (including timestamp, strings) for metadata extraction.
    all_offsets: Vec<usize>,
}

impl MsgLayout {
    fn from_def(def: &ApMsgDef) -> Self {
        let mut all_offsets = Vec::with_capacity(def.field_types.len());
        let mut offset = 0;
        for &ft in &def.field_types {
            all_offsets.push(offset);
            offset += ft.wire_size();
        }

        let first_name = def.field_names.first().map_or("", String::as_str);
        let timestamp_is_ms = first_name == "TimeMS";
        let timestamp_byte_offset = 0; // timestamp is always first field

        // Build slots for all non-timestamp fields
        let mut slots = Vec::new();
        let mut field_names = Vec::new();
        for (i, (name, &ft)) in def.field_names.iter().zip(&def.field_types).enumerate() {
            if i == 0 && (name == "TimeUS" || name == "TimeMS") {
                continue; // timestamp stored separately
            }
            slots.push(FieldSlot {
                byte_offset: all_offsets[i],
                field_type: ft,
            });
            field_names.push(name.clone());
        }

        Self {
            slots,
            field_names,
            timestamp_byte_offset,
            timestamp_is_ms,
            all_offsets,
        }
    }
}

// ---------------------------------------------------------------------------
// Main parse entry point
// ---------------------------------------------------------------------------

/// Parse an `ArduPilot` `DataFlash` binary log into the columnar
/// intermediate. The build step in `super::build` translates this into
/// a typed Session.
#[allow(clippy::too_many_lines)]
pub(crate) fn parse(data: &[u8], warnings: &mut Vec<Warning>) -> ApParsed {
    let mut msg_defs: HashMap<u8, ApMsgDef> = HashMap::new();
    let mut topics: HashMap<u8, MsgColumns> = HashMap::new();
    let mut layouts: HashMap<u8, MsgLayout> = HashMap::new();
    let mut params: HashMap<String, f64> = HashMap::new();
    let mut firmware_version = String::new();
    let mut vehicle_name = String::new();
    let mut batches = BatchAssembler::default();
    let mut stats = ApParseStats::default();

    let mut row_buf: Vec<f64> = Vec::new();
    let mut pos = 0;

    while pos + 3 <= data.len() {
        if data[pos] != HEAD1 || data[pos + 1] != HEAD2 {
            stats.corrupt_bytes += 1;
            pos += 1;
            continue;
        }

        let msg_type = data[pos + 2];

        if msg_type == FMT_TYPE {
            if pos + FMT_LEN > data.len() {
                stats.truncated = true;
                break;
            }
            if let Some(def) = parse_fmt(&data[pos..pos + FMT_LEN]) {
                msg_defs.insert(def.msg_type, def);
                stats.fmt_count += 1;
            }
            pos += FMT_LEN;
            stats.total_messages += 1;
            continue;
        }

        let Some(def) = msg_defs.get(&msg_type) else {
            stats.unknown_types += 1;
            stats.corrupt_bytes += 1;
            pos += 1;
            continue;
        };

        let msg_len = def.msg_len;
        if pos + msg_len > data.len() {
            stats.truncated = true;
            break;
        }

        let payload = &data[pos + 3..pos + msg_len];

        // Lazy layout initialization
        if let std::collections::hash_map::Entry::Vacant(e) = topics.entry(msg_type) {
            let layout = MsgLayout::from_def(def);
            e.insert(MsgColumns::new(layout.field_names.clone()));
            layouts.insert(msg_type, layout);
        }

        if let Some(layout) = layouts.get(&msg_type) {
            // Decode timestamp
            let raw_time = decode_u64(payload, layout.timestamp_byte_offset, def.field_types[0]);
            let time_us = if layout.timestamp_is_ms {
                raw_time * 1000
            } else {
                raw_time
            };

            // Decode all fields into scratch buffer
            row_buf.clear();
            for slot in &layout.slots {
                if slot.byte_offset + slot.field_type.wire_size() <= payload.len() {
                    row_buf.push(decode_f64(slot.field_type, &payload[slot.byte_offset..]));
                } else {
                    row_buf.push(0.0);
                }
            }

            if let Some(mc) = topics.get_mut(&msg_type) {
                mc.push_row(time_us, &row_buf);
                stats.total_messages += 1;
            }
        }

        // Extract metadata from special message types
        if def.name == "PARM" {
            if let Some(layout) = layouts.get(&msg_type) {
                extract_param(payload, def, layout, &mut params);
            }
        } else if def.name == "MSG" {
            if let Some(layout) = layouts.get(&msg_type) {
                extract_msg_info(
                    payload,
                    def,
                    layout,
                    &mut firmware_version,
                    &mut vehicle_name,
                );
            }
        } else if def.name == "VER" {
            if let Some(layout) = layouts.get(&msg_type) {
                extract_version(payload, def, layout, &mut firmware_version);
            }
        } else if def.name == "ISBH" {
            if let Some(layout) = layouts.get(&msg_type) {
                batches.header(payload, def, layout);
            }
        } else if def.name == "ISBD" {
            if let Some(layout) = layouts.get(&msg_type) {
                batches.chunk(payload, def, layout);
            }
        }

        pos += msg_len;
    }

    if stats.corrupt_bytes > 0 {
        warnings.push(Warning {
            message: format!("{} corrupt bytes skipped", stats.corrupt_bytes),
            byte_offset: None,
        });
    }

    ApParsed {
        msg_defs,
        topics,
        firmware_version,
        vehicle_name,
        params,
        batches: batches.into_batches(),
        stats,
    }
}

// ---------------------------------------------------------------------------
// Batch sampler (ISBH + ISBD)
// ---------------------------------------------------------------------------

/// `ISBD` carries 32 samples per axis per message.
const ISBD_SAMPLES: usize = 32;

/// Raw bytes of one named field, if the message defines it and the payload
/// is long enough to hold it.
fn field_bytes<'a>(
    payload: &'a [u8],
    def: &ApMsgDef,
    layout: &MsgLayout,
    name: &str,
) -> Option<(&'a [u8], FieldType)> {
    let idx = def.field_names.iter().position(|n| n == name)?;
    let ft = *def.field_types.get(idx)?;
    let offset = *layout.all_offsets.get(idx)?;
    Some((payload.get(offset..offset + ft.wire_size())?, ft))
}

fn field_f64(payload: &[u8], def: &ApMsgDef, layout: &MsgLayout, name: &str) -> Option<f64> {
    field_bytes(payload, def, layout, name).map(|(bytes, ft)| decode_f64(ft, bytes))
}

struct OpenBatch {
    /// `ISBH.N`, which its `ISBD` chunks repeat.
    number: u16,
    expected: usize,
    next_seqno: u16,
    batch: IsbBatch,
}

/// Stitches `ISBH` headers and their `ISBD` chunks into [`IsbBatch`]es in
/// log order. `N` is a u16 that wraps on long logs, so chunks are matched to
/// the header that precedes them, not by number alone. A burst with a
/// missing chunk or cut short by the end of the log is dropped: shifting
/// later samples into the gap would corrupt its spectrum.
#[derive(Default)]
struct BatchAssembler {
    open: Option<OpenBatch>,
    done: Vec<IsbBatch>,
}

impl BatchAssembler {
    fn header(&mut self, payload: &[u8], def: &ApMsgDef, layout: &MsgLayout) {
        self.finish();
        let field = |name| field_f64(payload, def, layout, name);
        let (Some(number), Some(sensor_type), Some(instance), Some(mul), Some(count), Some(rate)) = (
            field("N"),
            field("type"),
            field("instance"),
            field("mul"),
            field("smp_cnt"),
            field("smp_rate"),
        ) else {
            return;
        };
        let Some((start, FieldType::U64)) = field_bytes(payload, def, layout, "SampleUS") else {
            return;
        };
        if mul <= 0.0 || !rate.is_finite() || rate <= 0.0 {
            return;
        }
        let expected = count.saturating_as::<usize>();
        self.open = Some(OpenBatch {
            number: number.saturating_as::<u16>(),
            expected,
            next_seqno: 0,
            batch: IsbBatch {
                sensor_type: sensor_type.saturating_as::<u8>(),
                instance: instance.saturating_as::<u8>(),
                multiplier: mul,
                sample_us: decode_u64(start, 0, FieldType::U64),
                sample_rate_hz: rate,
                samples: std::array::from_fn(|_| Vec::with_capacity(expected)),
            },
        });
    }

    fn chunk(&mut self, payload: &[u8], def: &ApMsgDef, layout: &MsgLayout) {
        let Some(open) = self.open.as_mut() else {
            return;
        };
        let field = |name| field_f64(payload, def, layout, name);
        let (Some(number), Some(seqno)) = (field("N"), field("seqno")) else {
            return;
        };
        if number.saturating_as::<u16>() != open.number {
            return;
        }
        if seqno.saturating_as::<u16>() != open.next_seqno {
            self.open = None;
            return;
        }
        for (axis, name) in open.batch.samples.iter_mut().zip(["x", "y", "z"]) {
            let Some((bytes, FieldType::I16Array32)) = field_bytes(payload, def, layout, name)
            else {
                self.open = None;
                return;
            };
            axis.extend(
                bytes
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .take(ISBD_SAMPLES)
                    .map(|&b| i16::from_le_bytes(b)),
            );
        }
        open.next_seqno = open.next_seqno.wrapping_add(1);
    }

    fn finish(&mut self) {
        let Some(mut open) = self.open.take() else {
            return;
        };
        if open.expected == 0 || open.batch.samples[0].len() < open.expected {
            return;
        }
        for axis in &mut open.batch.samples {
            axis.truncate(open.expected);
        }
        self.done.push(open.batch);
    }

    fn into_batches(mut self) -> Vec<IsbBatch> {
        self.finish();
        self.done
    }
}

// ---------------------------------------------------------------------------
// Primitive decoders
// ---------------------------------------------------------------------------

#[inline]
fn decode_f64(ft: FieldType, data: &[u8]) -> f64 {
    match ft {
        FieldType::I8 => f64::from(data[0].cast_signed()),
        FieldType::U8 | FieldType::FlightMode => f64::from(data[0]),
        FieldType::I16 | FieldType::I16Array32 => f64::from(i16::from_le_bytes([data[0], data[1]])),
        FieldType::I16Centi => f64::from(i16::from_le_bytes([data[0], data[1]])) * 0.01,
        FieldType::U16 => f64::from(u16::from_le_bytes([data[0], data[1]])),
        FieldType::U16Centi => f64::from(u16::from_le_bytes([data[0], data[1]])) * 0.01,
        FieldType::I32 => f64::from(i32::from_le_bytes(data[..4].try_into().unwrap_or([0; 4]))),
        FieldType::I32Centi => {
            f64::from(i32::from_le_bytes(data[..4].try_into().unwrap_or([0; 4]))) * 0.01
        }
        FieldType::I32DegreesE7 => {
            f64::from(i32::from_le_bytes(data[..4].try_into().unwrap_or([0; 4]))) * 1e-7
        }
        FieldType::U32 => f64::from(u32::from_le_bytes(data[..4].try_into().unwrap_or([0; 4]))),
        FieldType::U32Centi => {
            f64::from(u32::from_le_bytes(data[..4].try_into().unwrap_or([0; 4]))) * 0.01
        }
        FieldType::I64 => i64::from_le_bytes(data[..8].try_into().unwrap_or([0; 8])).az::<f64>(),
        FieldType::U64 => u64::from_le_bytes(data[..8].try_into().unwrap_or([0; 8])).az::<f64>(),
        FieldType::Float => f64::from(f32::from_le_bytes(data[..4].try_into().unwrap_or([0; 4]))),
        FieldType::Double => f64::from_le_bytes(data[..8].try_into().unwrap_or([0; 8])),
        FieldType::Float16 => {
            let bits = u16::from_le_bytes([data[0], data[1]]);
            f64::from(decode_f16(bits))
        }
        FieldType::Char4 | FieldType::Char16 | FieldType::Char64 | FieldType::Unknown(_) => 0.0,
    }
}

fn decode_u64(data: &[u8], offset: usize, ft: FieldType) -> u64 {
    let d = &data[offset..];
    match ft {
        FieldType::U64 | FieldType::I64 => u64::from_le_bytes(d[..8].try_into().unwrap_or([0; 8])),
        FieldType::U32 | FieldType::I32 => {
            u64::from(u32::from_le_bytes(d[..4].try_into().unwrap_or([0; 4])))
        }
        _ => 0,
    }
}

// ---------------------------------------------------------------------------
// Format / metadata extraction
// ---------------------------------------------------------------------------

fn parse_fmt(data: &[u8]) -> Option<ApMsgDef> {
    if data.len() < FMT_LEN {
        return None;
    }

    let msg_type = data[3];
    let msg_len = data[4] as usize;
    let name = read_fixed_str(&data[5..9]);
    let format_str = read_fixed_str(&data[9..25]);
    let labels = read_fixed_str(&data[25..89]);

    let field_types: Vec<FieldType> = format_str
        .bytes()
        .map(FieldType::from_format_char)
        .collect();

    let field_names: Vec<String> = labels
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect();

    Some(ApMsgDef {
        msg_type,
        name,
        field_types,
        field_names,
        msg_len,
        format_str,
    })
}

fn read_fixed_str(data: &[u8]) -> String {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    String::from_utf8_lossy(&data[..end]).to_string()
}

fn read_str_at(payload: &[u8], layout: &MsgLayout, field_idx: usize, ft: FieldType) -> String {
    let offset = layout.all_offsets[field_idx];
    let size = ft.wire_size();
    if offset + size <= payload.len() {
        read_fixed_str(&payload[offset..offset + size])
    } else {
        String::new()
    }
}

fn extract_param(
    payload: &[u8],
    def: &ApMsgDef,
    layout: &MsgLayout,
    params: &mut HashMap<String, f64>,
) {
    let name_idx = def.field_names.iter().position(|n| n == "Name");
    let val_idx = def.field_names.iter().position(|n| n == "Value");
    if let (Some(ni), Some(vi)) = (name_idx, val_idx) {
        let name = read_str_at(payload, layout, ni, def.field_types[ni]);
        if !name.is_empty() {
            let offset = layout.all_offsets[vi];
            let val = decode_f64(def.field_types[vi], &payload[offset..]);
            params.insert(name, val);
        }
    }
}

fn extract_msg_info(
    payload: &[u8],
    def: &ApMsgDef,
    layout: &MsgLayout,
    firmware_version: &mut String,
    vehicle_name: &mut String,
) {
    let msg_idx = def.field_names.iter().position(|n| n == "Message");
    if let Some(idx) = msg_idx {
        let text = read_str_at(payload, layout, idx, def.field_types[idx]);
        if text.contains("ArduCopter")
            || text.contains("ArduPlane")
            || text.contains("ArduRover")
            || text.contains("ArduSub")
            || text.contains("Rover")
            || text.contains("Copter")
            || text.contains("Plane")
            || text.contains("AntennaTracker")
        {
            if firmware_version.is_empty() {
                *firmware_version = text;
            }
        } else if vehicle_name.is_empty()
            && !text.is_empty()
            && !text.contains("ChibiOS")
            && !text.contains("NuttX")
            && !text.contains("SITL")
            && !text.starts_with("Param ")
            && !text.starts_with("RC Protocol")
            && !text.starts_with("Throttle ")
            && !text.starts_with("Frame")
        {
            *vehicle_name = text;
        }
    }
}

fn extract_version(
    payload: &[u8],
    def: &ApMsgDef,
    layout: &MsgLayout,
    firmware_version: &mut String,
) {
    let fws_idx = def.field_names.iter().position(|n| n == "FWS");
    if let Some(idx) = fws_idx {
        let text = read_str_at(payload, layout, idx, def.field_types[idx]);
        if !text.is_empty() {
            *firmware_version = text;
        }
    }
}

fn decode_f16(bits: u16) -> f32 {
    let sign = (bits >> 15) & 1;
    let exponent = (bits >> 10) & 0x1F;
    let mantissa = bits & 0x3FF;

    let val = if exponent == 0 {
        let m = f32::from(mantissa) / 1024.0;
        m * 2.0f32.powi(-14)
    } else if exponent == 31 {
        if mantissa == 0 {
            f32::INFINITY
        } else {
            f32::NAN
        }
    } else {
        let m = 1.0 + f32::from(mantissa) / 1024.0;
        m * 2.0f32.powi(i32::from(exponent) - 15)
    };

    if sign == 1 {
        -val
    } else {
        val
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// F7 red→green: `ArduPilot` FMT type `c` means "int16 × 100"
    /// (centidegrees / fixed-point). The parser must apply the ÷100
    /// scaling so consumers don't see raw centidegrees and compute
    /// values 100× wrong. Pre-fix, both `h` and `c` collapsed to
    /// `FieldType::I16` and `decode_f64` returned the raw int16 as f64.
    #[test]
    fn fmt_c_centidegree_field_scaled() {
        let centi_type = FieldType::from_format_char(b'c');
        // 4500 centidegrees = 45.00°
        let bytes = 4500_i16.to_le_bytes();
        let v = decode_f64(centi_type, &bytes);
        assert!(
            (v - 45.0).abs() < 1e-9,
            "FMT type 'c' (centidegree) should decode to 45.0°, got {v}"
        );
    }

    /// `C` = uint16 × 100 (e.g. RFND.Dist in cm-as-deca-mm? No —
    /// some AP versions use `C` for cm-scaled distances; semantics
    /// are "raw / 100").
    #[test]
    fn fmt_capital_c_centi_uint_scaled() {
        let centi_type = FieldType::from_format_char(b'C');
        let bytes = 12000_u16.to_le_bytes();
        let v = decode_f64(centi_type, &bytes);
        assert!((v - 120.0).abs() < 1e-9, "got {v}, expected 120.0");
    }

    /// `e` = int32 × 100. Used for some altitude / centidegree fields.
    #[test]
    fn fmt_e_centi_int32_scaled() {
        let e_type = FieldType::from_format_char(b'e');
        let bytes = (-99_999_i32).to_le_bytes();
        let v = decode_f64(e_type, &bytes);
        assert!((v - (-999.99)).abs() < 1e-6, "got {v}, expected -999.99");
    }

    /// `L` = int32 with × 1e7 scale (lat/lng). Parser must divide
    /// by 1e7 to yield decimal degrees so consumers don't have to
    /// remember the convention per-field.
    #[test]
    fn fmt_capital_l_lat_lng_scaled_to_decimal_degrees() {
        let l_type = FieldType::from_format_char(b'L');
        let bytes = 502_075_967_i32.to_le_bytes(); // Sweden, ~50.2076°
        let v = decode_f64(l_type, &bytes);
        assert!((v - 50.207_596_7).abs() < 1e-6, "got {v}, expected ~50.21");
    }

    /// Plain `h` (no scaling) must still decode raw — regression guard
    /// against accidentally scaling everything.
    #[test]
    fn fmt_h_int16_unchanged() {
        let h_type = FieldType::from_format_char(b'h');
        let bytes = (-1234_i16).to_le_bytes();
        let v = decode_f64(h_type, &bytes);
        assert!((v - (-1234.0)).abs() < 1e-9, "got {v}");
    }
}

#[cfg(test)]
mod batch_tests {
    use super::*;

    const ISBH_TYPE: u8 = 200;
    const ISBD_TYPE: u8 = 201;

    fn fmt_record(msg_type: u8, name: &str, format: &str, labels: &str) -> Vec<u8> {
        let payload_len: usize = format
            .bytes()
            .map(|c| FieldType::from_format_char(c).wire_size())
            .sum();
        let mut out = vec![
            HEAD1,
            HEAD2,
            FMT_TYPE,
            msg_type,
            (payload_len + 3).saturating_as::<u8>(),
        ];
        for (text, width) in [(name, 4), (format, 16), (labels, 64)] {
            let mut field = text.as_bytes().to_vec();
            field.resize(width, 0);
            out.extend(field);
        }
        out
    }

    fn header_defs() -> Vec<u8> {
        let mut out = fmt_record(
            ISBH_TYPE,
            "ISBH",
            "QHBBHHQf",
            "TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate",
        );
        out.extend(fmt_record(
            ISBD_TYPE,
            "ISBD",
            "QHHaaa",
            "TimeUS,N,seqno,x,y,z",
        ));
        out
    }

    fn isbh(number: u16, sensor_type: u8, instance: u8, mul: u16, count: u16) -> Vec<u8> {
        let mut out = vec![HEAD1, HEAD2, ISBH_TYPE];
        out.extend(1_000_u64.to_le_bytes());
        out.extend(number.to_le_bytes());
        out.extend([sensor_type, instance]);
        out.extend(mul.to_le_bytes());
        out.extend(count.to_le_bytes());
        out.extend(5_000_000_u64.to_le_bytes());
        out.extend(1000.0_f32.to_le_bytes());
        out
    }

    /// One chunk whose x samples are `base..base + 32` (y = -x, z = 0).
    fn isbd(number: u16, seqno: u16, base: i16) -> Vec<u8> {
        let mut out = vec![HEAD1, HEAD2, ISBD_TYPE];
        out.extend(2_000_u64.to_le_bytes());
        out.extend(number.to_le_bytes());
        out.extend(seqno.to_le_bytes());
        let x: Vec<i16> = (0..32).map(|i| base + i).collect();
        for axis in [x.clone(), x.iter().map(|v| -v).collect(), vec![0; 32]] {
            out.extend(axis.iter().flat_map(|v| v.to_le_bytes()));
        }
        out
    }

    fn parse_bytes(parts: &[Vec<u8>]) -> ApParsed {
        parse(&parts.concat(), &mut Vec::new())
    }

    #[test]
    fn stitches_chunks_and_trims_to_sample_count() {
        let parsed = parse_bytes(&[
            header_defs(),
            isbh(7, 1, 2, 938, 40),
            isbd(7, 0, 0),
            isbd(7, 1, 32),
        ]);
        assert_eq!(parsed.batches.len(), 1);
        let b = &parsed.batches[0];
        assert_eq!((b.sensor_type, b.instance), (1, 2));
        assert_eq!(b.sample_us, 5_000_000);
        assert!((b.multiplier - 938.0).abs() < f64::EPSILON);
        assert!((b.sample_rate_hz - 1000.0).abs() < f64::EPSILON);
        assert_eq!(b.samples[0], (0..40).collect::<Vec<i16>>());
        assert_eq!(b.samples[1][39], -39);
    }

    /// A lost chunk would shift every later sample, so the burst is dropped.
    #[test]
    fn drops_burst_with_missing_chunk() {
        let parsed = parse_bytes(&[
            header_defs(),
            isbh(7, 1, 0, 938, 64),
            isbd(7, 0, 0),
            isbd(7, 2, 64),
        ]);
        assert!(parsed.batches.is_empty());
    }

    #[test]
    fn drops_burst_cut_short_by_end_of_log() {
        let parsed = parse_bytes(&[header_defs(), isbh(7, 1, 0, 938, 64), isbd(7, 0, 0)]);
        assert!(parsed.batches.is_empty());
    }

    /// `N` wraps on long logs; chunks belong to the header right before them.
    #[test]
    fn ignores_chunks_for_another_burst() {
        let parsed = parse_bytes(&[
            header_defs(),
            isbh(7, 1, 0, 938, 32),
            isbd(6, 0, 100),
            isbd(7, 0, 0),
            isbh(8, 0, 0, 104, 32),
            isbd(8, 0, 5),
        ]);
        assert_eq!(parsed.batches.len(), 2);
        assert_eq!(parsed.batches[0].samples[0][0], 0);
        assert_eq!(parsed.batches[1].sensor_type, 0);
        assert_eq!(parsed.batches[1].samples[0][0], 5);
    }
}
