use std::collections::HashMap;

use crate::types::Warning;

use super::types::{
    Px4ParseStats, Px4RawSession, ULogDataMsg, ULogField, ULogFormat, ULogSubscription, ULogType,
    ULogValue,
};

const ULOG_MAGIC: &[u8; 7] = b"\x55\x4c\x6f\x67\x01\x12\x35";
const HEADER_SIZE: usize = 16;

// Message type codes
const MSG_FLAG_BITS: u8 = b'B';
const MSG_FORMAT: u8 = b'F';
const MSG_INFO: u8 = b'I';
const MSG_INFO_MULTI: u8 = b'M';
const MSG_PARAM: u8 = b'P';
const MSG_PARAM_DEFAULT: u8 = b'Q';
const MSG_ADD_LOGGED: u8 = b'A';
const MSG_DATA: u8 = b'D';
const MSG_LOGGING: u8 = b'L';
const MSG_LOGGING_TAGGED: u8 = b'C';
const MSG_DROPOUT: u8 = b'O';

/// Parse a PX4 `ULog` binary file.
pub(crate) fn parse(data: &[u8], warnings: &mut Vec<Warning>) -> Px4RawSession {
    let mut formats: HashMap<String, ULogFormat> = HashMap::new();
    let mut subscriptions: HashMap<u16, ULogSubscription> = HashMap::new();
    let mut data_messages: Vec<ULogDataMsg> = Vec::new();
    let mut info: HashMap<String, String> = HashMap::new();
    let mut params: HashMap<String, f64> = HashMap::new();
    let mut stats = Px4ParseStats::default();

    if data.len() < HEADER_SIZE || data[..7] != *ULOG_MAGIC {
        warnings.push(Warning {
            message: "Invalid ULog header".into(),
            byte_offset: Some(0),
        });
        return empty_session(stats);
    }

    let mut pos = HEADER_SIZE;

    // Check for flag bits message
    let mut appended_offsets: Vec<u64> = Vec::new();
    if pos + 3 <= data.len() && data[pos + 2] == MSG_FLAG_BITS {
        let msg_size = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        if pos + 3 + msg_size <= data.len() {
            let payload = &data[pos + 3..pos + 3 + msg_size];
            parse_flag_bits(payload, &mut appended_offsets, warnings);
        }
        pos += 3 + msg_size;
    }

    // Parse definitions and data sections
    parse_messages(
        data,
        &mut pos,
        &mut formats,
        &mut subscriptions,
        &mut data_messages,
        &mut info,
        &mut params,
        &mut stats,
        warnings,
    );

    // Parse appended data sections
    for offset in appended_offsets {
        #[allow(clippy::cast_possible_truncation)]
        let mut apos = offset as usize;
        if apos < data.len() {
            parse_messages(
                data,
                &mut apos,
                &mut formats,
                &mut subscriptions,
                &mut data_messages,
                &mut info,
                &mut params,
                &mut stats,
                warnings,
            );
        }
    }

    let firmware_version = info
        .get("ver_sw")
        .or_else(|| info.get("sys_name"))
        .cloned()
        .unwrap_or_default();

    let hardware_name = info
        .get("sys_hw")
        .or_else(|| info.get("ver_hw"))
        .cloned()
        .unwrap_or_default();

    Px4RawSession {
        formats,
        subscriptions,
        data_messages,
        info,
        params,
        firmware_version,
        hardware_name,
        stats,
    }
}

fn parse_flag_bits(payload: &[u8], appended_offsets: &mut Vec<u64>, warnings: &mut Vec<Warning>) {
    if payload.len() < 40 {
        return;
    }

    // Check incompat flags
    let incompat = payload[8];
    let known_incompat = 0x01; // DATA_APPENDED
    if incompat & !known_incompat != 0 {
        warnings.push(Warning {
            message: format!(
                "Unknown incompatible flags: 0x{:02x}. Parsing may be incorrect.",
                incompat & !known_incompat
            ),
            byte_offset: None,
        });
    }

    // Extract appended offsets
    if incompat & 0x01 != 0 {
        for i in 0..3 {
            let base = 16 + i * 8;
            if base + 8 <= payload.len() {
                let offset =
                    u64::from_le_bytes(payload[base..base + 8].try_into().unwrap_or([0; 8]));
                if offset > 0 {
                    appended_offsets.push(offset);
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn parse_messages(
    data: &[u8],
    pos: &mut usize,
    formats: &mut HashMap<String, ULogFormat>,
    subscriptions: &mut HashMap<u16, ULogSubscription>,
    data_messages: &mut Vec<ULogDataMsg>,
    info: &mut HashMap<String, String>,
    params: &mut HashMap<String, f64>,
    stats: &mut Px4ParseStats,
    warnings: &mut Vec<Warning>,
) {
    while *pos + 3 <= data.len() {
        let msg_size = u16::from_le_bytes([data[*pos], data[*pos + 1]]) as usize;
        let msg_type = data[*pos + 2];
        let msg_end = *pos + 3 + msg_size;

        if msg_end > data.len() {
            break;
        }

        let payload = &data[*pos + 3..msg_end];
        stats.total_messages += 1;

        match msg_type {
            MSG_FORMAT => {
                if let Some(fmt) = parse_format(payload, formats) {
                    formats.insert(fmt.name.clone(), fmt);
                    stats.format_count += 1;
                }
            }
            MSG_INFO | MSG_INFO_MULTI => {
                parse_info(payload, msg_type, info);
            }
            MSG_PARAM | MSG_PARAM_DEFAULT => {
                parse_param(payload, msg_type, params);
            }
            MSG_ADD_LOGGED => {
                if let Some(sub) = parse_add_logged(payload) {
                    subscriptions.insert(sub.msg_id, sub);
                    stats.subscription_count += 1;
                }
            }
            MSG_DATA => {
                if let Some(msg) = parse_data(payload, subscriptions, formats) {
                    data_messages.push(msg);
                    stats.data_count += 1;
                }
            }
            MSG_DROPOUT => {
                stats.dropout_count += 1;
            }
            MSG_LOGGING | MSG_LOGGING_TAGGED => {}
            _ => {
                if stats.total_messages < 10 {
                    warnings.push(Warning {
                        message: format!("Unknown ULog message type: 0x{msg_type:02x}"),
                        byte_offset: Some(*pos),
                    });
                }
            }
        }

        *pos = msg_end;
    }
}

fn parse_format(
    payload: &[u8],
    existing_formats: &HashMap<String, ULogFormat>,
) -> Option<ULogFormat> {
    let text = std::str::from_utf8(payload).ok()?;
    let (name, fields_str) = text.split_once(':')?;
    let name = name.trim().to_string();

    let mut fields = Vec::new();
    let mut total_size = 0;

    for field_def in fields_str.split(';') {
        let field_def = field_def.trim();
        if field_def.is_empty() {
            continue;
        }

        let (type_str, field_name) = field_def.rsplit_once(' ')?;
        let type_str = type_str.trim();
        let field_name = field_name.trim();

        // Skip padding fields
        if field_name.starts_with("_padding") {
            let byte_size = compute_type_size(type_str, existing_formats);
            total_size += byte_size;
            continue;
        }

        let (base_type, array_size) = parse_array_type(type_str);
        let primitive = ULogType::from_name(base_type);
        let byte_size = compute_type_size(type_str, existing_formats);

        fields.push(ULogField {
            name: field_name.to_string(),
            type_name: type_str.to_string(),
            primitive,
            array_size,
            byte_size,
        });

        total_size += byte_size;
    }

    Some(ULogFormat {
        name,
        fields,
        total_size,
    })
}

fn parse_array_type(type_str: &str) -> (&str, Option<usize>) {
    if let Some(bracket_start) = type_str.find('[') {
        let base = &type_str[..bracket_start];
        let count_str = &type_str[bracket_start + 1..type_str.len() - 1];
        let count = count_str.parse().ok();
        (base, count)
    } else {
        (type_str, None)
    }
}

fn compute_type_size(type_str: &str, formats: &HashMap<String, ULogFormat>) -> usize {
    let (base, array_size) = parse_array_type(type_str);
    let count = array_size.unwrap_or(1);

    if let Some(prim) = ULogType::from_name(base) {
        prim.size() * count
    } else if let Some(nested) = formats.get(base) {
        nested.total_size * count
    } else {
        0
    }
}

fn parse_info(payload: &[u8], msg_type: u8, info: &mut HashMap<String, String>) {
    let offset = usize::from(msg_type == MSG_INFO_MULTI);
    if payload.len() < offset + 1 {
        return;
    }
    let key_len = payload[offset] as usize;
    if payload.len() < offset + 1 + key_len {
        return;
    }
    let key_bytes = &payload[offset + 1..offset + 1 + key_len];
    let value_bytes = &payload[offset + 1 + key_len..];

    let Ok(key_str) = std::str::from_utf8(key_bytes) else {
        return;
    };

    // Key format: "type key_name"
    let key_name = key_str.split_once(' ').map_or(key_str, |(_, name)| name);

    // Try to interpret value as string
    let value = if key_str.starts_with("char[") {
        String::from_utf8_lossy(value_bytes)
            .trim_end_matches('\0')
            .to_string()
    } else if key_str.starts_with("int32_t") && value_bytes.len() >= 4 {
        let v = i32::from_le_bytes(value_bytes[..4].try_into().unwrap_or([0; 4]));
        v.to_string()
    } else if key_str.starts_with("float") && value_bytes.len() >= 4 {
        let v = f32::from_le_bytes(value_bytes[..4].try_into().unwrap_or([0; 4]));
        v.to_string()
    } else {
        String::from_utf8_lossy(value_bytes)
            .trim_end_matches('\0')
            .to_string()
    };

    info.insert(key_name.to_string(), value);
}

fn parse_param(payload: &[u8], msg_type: u8, params: &mut HashMap<String, f64>) {
    let offset = usize::from(msg_type == MSG_PARAM_DEFAULT);
    if payload.len() < offset + 1 {
        return;
    }
    let key_len = payload[offset] as usize;
    if payload.len() < offset + 1 + key_len + 4 {
        return;
    }
    let key_bytes = &payload[offset + 1..offset + 1 + key_len];
    let value_bytes = &payload[offset + 1 + key_len..];

    let Ok(key_str) = std::str::from_utf8(key_bytes) else {
        return;
    };

    let key_name = key_str.split_once(' ').map_or(key_str, |(_, name)| name);

    let value = if key_str.starts_with("float") && value_bytes.len() >= 4 {
        f64::from(f32::from_le_bytes(
            value_bytes[..4].try_into().unwrap_or([0; 4]),
        ))
    } else if key_str.starts_with("int32_t") && value_bytes.len() >= 4 {
        f64::from(i32::from_le_bytes(
            value_bytes[..4].try_into().unwrap_or([0; 4]),
        ))
    } else {
        return;
    };

    params.insert(key_name.to_string(), value);
}

fn parse_add_logged(payload: &[u8]) -> Option<ULogSubscription> {
    if payload.len() < 3 {
        return None;
    }
    let multi_id = payload[0];
    let msg_id = u16::from_le_bytes([payload[1], payload[2]]);
    let format_name = std::str::from_utf8(&payload[3..])
        .ok()?
        .trim_end_matches('\0')
        .to_string();

    Some(ULogSubscription {
        msg_id,
        multi_id,
        format_name,
    })
}

fn parse_data(
    payload: &[u8],
    subscriptions: &HashMap<u16, ULogSubscription>,
    formats: &HashMap<String, ULogFormat>,
) -> Option<ULogDataMsg> {
    if payload.len() < 2 {
        return None;
    }
    let msg_id = u16::from_le_bytes([payload[0], payload[1]]);
    let data = &payload[2..];

    let sub = subscriptions.get(&msg_id)?;
    let fmt = formats.get(&sub.format_name)?;

    let mut values = HashMap::new();
    let mut offset = 0;

    for field in &fmt.fields {
        if offset + field.byte_size > data.len() {
            break;
        }

        let field_data = &data[offset..offset + field.byte_size];

        if let Some(array_size) = field.array_size {
            if let Some(prim) = field.primitive {
                let elem_size = prim.size();
                for i in 0..array_size {
                    let elem_offset = i * elem_size;
                    if elem_offset + elem_size <= field_data.len() {
                        let val = decode_primitive(
                            prim,
                            &field_data[elem_offset..elem_offset + elem_size],
                        );
                        values.insert(format!("{}[{i}]", field.name), val);
                    }
                }
            }
        } else if let Some(prim) = field.primitive {
            let val = decode_primitive(prim, field_data);
            values.insert(field.name.clone(), val);
        }

        offset += field.byte_size;
    }

    let timestamp_us = values.get("timestamp").map_or(0, |v| {
        #[allow(clippy::cast_sign_loss)]
        match v {
            ULogValue::UInt(t) => *t,
            ULogValue::Int(t) => *t as u64,
            _ => 0,
        }
    });

    Some(ULogDataMsg {
        msg_id,
        timestamp_us,
        values,
    })
}

fn decode_primitive(prim: ULogType, data: &[u8]) -> ULogValue {
    match prim {
        ULogType::Int8 => ULogValue::Int(i64::from(data[0].cast_signed())),
        ULogType::UInt8 | ULogType::Bool | ULogType::Char => ULogValue::UInt(u64::from(data[0])),
        ULogType::Int16 => ULogValue::Int(i64::from(i16::from_le_bytes([data[0], data[1]]))),
        ULogType::UInt16 => ULogValue::UInt(u64::from(u16::from_le_bytes([data[0], data[1]]))),
        ULogType::Int32 => ULogValue::Int(i64::from(i32::from_le_bytes(
            data[..4].try_into().unwrap_or([0; 4]),
        ))),
        ULogType::UInt32 => ULogValue::UInt(u64::from(u32::from_le_bytes(
            data[..4].try_into().unwrap_or([0; 4]),
        ))),
        ULogType::Int64 => {
            ULogValue::Int(i64::from_le_bytes(data[..8].try_into().unwrap_or([0; 8])))
        }
        ULogType::UInt64 => {
            ULogValue::UInt(u64::from_le_bytes(data[..8].try_into().unwrap_or([0; 8])))
        }
        ULogType::Float => ULogValue::Float(f64::from(f32::from_le_bytes(
            data[..4].try_into().unwrap_or([0; 4]),
        ))),
        ULogType::Double => {
            ULogValue::Float(f64::from_le_bytes(data[..8].try_into().unwrap_or([0; 8])))
        }
    }
}

fn empty_session(stats: Px4ParseStats) -> Px4RawSession {
    Px4RawSession {
        formats: HashMap::new(),
        subscriptions: HashMap::new(),
        data_messages: Vec::new(),
        info: HashMap::new(),
        params: HashMap::new(),
        firmware_version: String::new(),
        hardware_name: String::new(),
        stats,
    }
}
