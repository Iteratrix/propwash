use std::collections::HashMap;

use crate::types::Warning;

use super::types::{ApMessage, ApMsgDef, ApParseStats, ApRawSession, ApValue, FieldType};

const HEAD1: u8 = 0xA3;
const HEAD2: u8 = 0x95;
const FMT_TYPE: u8 = 128;
const FMT_LEN: usize = 89;

/// Parse an `ArduPilot` `DataFlash` binary log.
pub(crate) fn parse(data: &[u8], warnings: &mut Vec<Warning>) -> ApRawSession {
    let mut msg_defs: HashMap<u8, ApMsgDef> = HashMap::new();
    let mut messages: Vec<ApMessage> = Vec::new();
    let mut params: HashMap<String, f64> = HashMap::new();
    let mut firmware_version = String::new();
    let mut vehicle_name = String::new();
    let mut stats = ApParseStats::default();

    let mut pos = 0;

    while pos + 3 <= data.len() {
        // Scan for message header
        if data[pos] != HEAD1 || data[pos + 1] != HEAD2 {
            stats.corrupt_bytes += 1;
            pos += 1;
            continue;
        }

        let msg_type = data[pos + 2];

        if msg_type == FMT_TYPE {
            // FMT message — fixed 89-byte layout
            if pos + FMT_LEN > data.len() {
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

        // Look up message definition
        let Some(def) = msg_defs.get(&msg_type) else {
            stats.unknown_types += 1;
            stats.corrupt_bytes += 1;
            pos += 1;
            continue;
        };

        let msg_len = def.msg_len;
        if pos + msg_len > data.len() {
            break;
        }

        let payload = &data[pos + 3..pos + msg_len];
        let values = decode_payload(payload, &def.field_types);

        // Extract timestamp — first field is typically TimeUS (u64) or TimeMS (u32)
        let first_field_name = def.field_names.first().map_or("", String::as_str);
        let raw_time = values.first().map_or(0u64, |v| match v {
            ApValue::UInt(t) => *t,
            ApValue::Int(t) => {
                #[allow(clippy::cast_sign_loss)]
                {
                    *t as u64
                }
            }
            _ => 0,
        });
        let time_us = if first_field_name == "TimeMS" {
            raw_time * 1000 // Convert ms to us
        } else {
            raw_time
        };

        let msg = ApMessage {
            msg_type,
            time_us,
            values,
        };

        // Extract metadata from special message types
        if def.name == "PARM" {
            extract_param(&msg, def, &mut params);
        } else if def.name == "MSG" {
            extract_msg_info(&msg, def, &mut firmware_version, &mut vehicle_name);
        } else if def.name == "VER" {
            extract_version(&msg, def, &mut firmware_version);
        }

        messages.push(msg);
        stats.total_messages += 1;
        pos += msg_len;
    }

    if stats.corrupt_bytes > 0 {
        warnings.push(Warning {
            message: format!("{} corrupt bytes skipped", stats.corrupt_bytes),
            byte_offset: None,
        });
    }

    ApRawSession {
        msg_defs,
        messages,
        firmware_version,
        vehicle_name,
        params,
        stats,
    }
}

/// Parse a FMT message (89 bytes) into a message definition.
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

/// Decode a message payload according to its field types.
#[allow(clippy::cast_possible_wrap)]
fn decode_payload(payload: &[u8], field_types: &[FieldType]) -> Vec<ApValue> {
    let mut values = Vec::with_capacity(field_types.len());
    let mut offset = 0;

    for &ft in field_types {
        let size = ft.wire_size();
        if offset + size > payload.len() {
            values.push(ApValue::Int(0));
            continue;
        }
        let bytes = &payload[offset..offset + size];

        let value = match ft {
            FieldType::I8 => ApValue::Int(i64::from(bytes[0] as i8)),
            FieldType::U8 | FieldType::FlightMode => ApValue::UInt(u64::from(bytes[0])),
            FieldType::I16 => ApValue::Int(i64::from(i16::from_le_bytes([bytes[0], bytes[1]]))),
            FieldType::U16 => ApValue::UInt(u64::from(u16::from_le_bytes([bytes[0], bytes[1]]))),
            FieldType::I32 => ApValue::Int(i64::from(i32::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3],
            ]))),
            FieldType::U32 => ApValue::UInt(u64::from(u32::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3],
            ]))),
            FieldType::I64 => ApValue::Int(i64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            FieldType::U64 => ApValue::UInt(u64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            FieldType::Float => ApValue::Float(f64::from(f32::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3],
            ]))),
            FieldType::Double => ApValue::Float(f64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            FieldType::Float16 => {
                let bits = u16::from_le_bytes([bytes[0], bytes[1]]);
                ApValue::Float(f64::from(decode_f16(bits)))
            }
            FieldType::Char4 | FieldType::Char16 | FieldType::Char64 => {
                ApValue::Str(read_fixed_str(bytes))
            }
            FieldType::I16Array32 => {
                // Store as the first element for now — arrays are rarely needed
                let v = i16::from_le_bytes([bytes[0], bytes[1]]);
                ApValue::Int(i64::from(v))
            }
            FieldType::Unknown(_) => ApValue::Int(0),
        };

        values.push(value);
        offset += size;
    }

    values
}

/// Decode IEEE 754 half-precision float to f32.
fn decode_f16(bits: u16) -> f32 {
    let sign = (bits >> 15) & 1;
    let exponent = (bits >> 10) & 0x1F;
    let mantissa = bits & 0x3FF;

    let val = if exponent == 0 {
        // Subnormal
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

/// Read a null-padded fixed-length string.
fn read_fixed_str(data: &[u8]) -> String {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    String::from_utf8_lossy(&data[..end]).to_string()
}

fn extract_param(msg: &ApMessage, def: &ApMsgDef, params: &mut HashMap<String, f64>) {
    let name_idx = def.field_names.iter().position(|n| n == "Name");
    let val_idx = def.field_names.iter().position(|n| n == "Value");
    if let (Some(ni), Some(vi)) = (name_idx, val_idx) {
        if let (Some(ApValue::Str(name)), Some(val)) = (msg.values.get(ni), msg.values.get(vi)) {
            params.insert(name.clone(), val.as_f64());
        }
    }
}

fn extract_msg_info(
    msg: &ApMessage,
    def: &ApMsgDef,
    firmware_version: &mut String,
    vehicle_name: &mut String,
) {
    let msg_idx = def.field_names.iter().position(|n| n == "Message");
    if let Some(idx) = msg_idx {
        if let Some(ApValue::Str(text)) = msg.values.get(idx) {
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
                    firmware_version.clone_from(text);
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
                vehicle_name.clone_from(text);
            }
        }
    }
}

fn extract_version(msg: &ApMessage, def: &ApMsgDef, firmware_version: &mut String) {
    // VER message has FWS (firmware string) field
    let fws_idx = def.field_names.iter().position(|n| n == "FWS");
    if let Some(idx) = fws_idx {
        if let Some(ApValue::Str(text)) = msg.values.get(idx) {
            if !text.is_empty() {
                firmware_version.clone_from(text);
            }
        }
    }
}
