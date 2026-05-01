use std::collections::HashMap;

use az::Az;

use crate::types::Warning;

use super::types::{ApMsgDef, ApParseStats, FieldType, MsgColumns};

/// Output of the AP parser: just the columnar intermediate that
/// [`super::build`] folds into a typed Session.
pub(crate) struct ApParsed {
    pub msg_defs: HashMap<u8, ApMsgDef>,
    pub topics: HashMap<u8, MsgColumns>,
    pub firmware_version: String,
    pub vehicle_name: String,
    pub params: HashMap<String, f64>,
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
        stats,
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

    /// F7 red→green: ArduPilot FMT type `c` means "int16 × 100"
    /// (centidegrees / fixed-point). The parser must apply the ÷100
    /// scaling so consumers don't see raw centidegrees and compute
    /// values 100× wrong. Pre-fix, both `h` and `c` collapsed to
    /// `FieldType::I16` and decode_f64 returned the raw int16 as f64.
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
        assert!((v - 50.2_075_967).abs() < 1e-6, "got {v}, expected ~50.21");
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
