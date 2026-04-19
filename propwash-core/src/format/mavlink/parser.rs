use std::collections::HashMap;

use az::Az;

use crate::types::Warning;

use super::types::{
    MavType, MavlinkParseStats, MavlinkSession, MsgColumns, Severity, StatusMessage,
};

const MARKER_V1: u8 = 0xFE;
const MARKER_V2: u8 = 0xFD;
const GCS_SYSID: u8 = 255;

// ---------------------------------------------------------------------------
// MAVLink X.25 CRC-16
// ---------------------------------------------------------------------------

fn mavlink_crc(data: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        let tmp = u16::from(b) ^ (crc & 0xFF);
        let tmp = (tmp ^ (tmp << 4)) & 0xFF;
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    // Append the virtual CRC-extra byte
    let tmp = u16::from(crc_extra) ^ (crc & 0xFF);
    let tmp = (tmp ^ (tmp << 4)) & 0xFF;
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

// ---------------------------------------------------------------------------
// CRC-extra lookup — covers all standard + ArduPilot MAVLink v1 messages
// ---------------------------------------------------------------------------

/// CRC-extra for each `MAVLink` message ID (0-255). 0 = unknown/unused.
const CRC_EXTRA: [u8; 256] = [
    /*   0 */ 50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, /*  16 */ 0,
    0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, /*  32 */ 185, 104, 237,
    244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, /*  48 */ 41, 39, 78, 196, 0,
    0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, /*  64 */ 191, 118, 148, 21, 0, 243, 124, 0, 0,
    38, 20, 158, 152, 143, 0, 0, /*  80 */ 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63,
    54, 47, 0, 0, /*  96 */ 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34,
    /* 112 */ 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25,
    /* 128 */ 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131,
    /* 144 */ 127, 0, 103, 154, 178, 200, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0,
    /* 160 */ 78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138,
    /* 176 */ 234, 240, 47, 189, 52, 174, 229, 85, 159, 186, 72, 0, 0, 0, 0, 92,
    /* 192 */ 36, 71, 98, 120, 0, 0, 0, 0, 134, 205, 0, 0, 0, 0, 0, 0, /* 208 */ 0, 0, 0,
    0, 0, 0, 69, 101, 50, 202, 17, 162, 0, 0, 0, 0, /* 224 */ 0, 208, 207, 0, 0, 0, 163, 105,
    151, 35, 150, 179, 0, 0, 0, 0, /* 240 */ 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49,
    170, 44, 83, 46, 0,
];

/// Bitmap of which message IDs (0-255) have a known CRC extra.
const CRC_KNOWN: [u64; 4] = [
    0xE0CF_FFFF_FFFF_08F7,
    0xFFFF_FFF0_3EFE_3E6F,
    0x87FF_FFFF_7FFD_FFFF,
    0x7FFE_0FC6_0FC0_030F,
];

fn crc_extra_for(msg_id: u32) -> Option<u8> {
    if msg_id >= 256 {
        return None;
    }
    let idx = msg_id as usize;
    let word = CRC_KNOWN[idx / 64];
    if word & (1u64 << (idx % 64)) != 0 {
        Some(CRC_EXTRA[idx])
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Wire type decoders
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
#[allow(dead_code)]
enum W {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    F32,
}

#[inline]
fn decode_w(data: &[u8], offset: usize, w: W) -> f64 {
    match w {
        W::U8 => f64::from(data[offset]),
        W::I8 => f64::from(data[offset].cast_signed()),
        W::U16 => f64::from(u16::from_le_bytes([data[offset], data[offset + 1]])),
        W::I16 => f64::from(i16::from_le_bytes([data[offset], data[offset + 1]])),
        W::U32 => f64::from(u32::from_le_bytes(
            data[offset..offset + 4].try_into().unwrap_or([0; 4]),
        )),
        W::I32 => f64::from(i32::from_le_bytes(
            data[offset..offset + 4].try_into().unwrap_or([0; 4]),
        )),
        W::U64 => {
            u64::from_le_bytes(data[offset..offset + 8].try_into().unwrap_or([0; 8])).az::<f64>()
        }
        W::F32 => f64::from(f32::from_le_bytes(
            data[offset..offset + 4].try_into().unwrap_or([0; 4]),
        )),
    }
}

// ---------------------------------------------------------------------------
// Message specifications (wire-order field layouts)
// ---------------------------------------------------------------------------

struct MsgSpec {
    name: &'static str,
    min_payload: usize,
    /// Column fields: (name, byte offset in payload, wire type).
    fields: &'static [(&'static str, usize, W)],
}

// -- Telemetry messages we decode into columns --

const ATTITUDE: MsgSpec = MsgSpec {
    name: "ATTITUDE",
    min_payload: 28,

    fields: &[
        ("roll", 4, W::F32),
        ("pitch", 8, W::F32),
        ("yaw", 12, W::F32),
        ("rollspeed", 16, W::F32),
        ("pitchspeed", 20, W::F32),
        ("yawspeed", 24, W::F32),
    ],
};

const RAW_IMU: MsgSpec = MsgSpec {
    name: "RAW_IMU",
    min_payload: 26,

    fields: &[
        ("xacc", 8, W::I16),
        ("yacc", 10, W::I16),
        ("zacc", 12, W::I16),
        ("xgyro", 14, W::I16),
        ("ygyro", 16, W::I16),
        ("zgyro", 18, W::I16),
        ("xmag", 20, W::I16),
        ("ymag", 22, W::I16),
        ("zmag", 24, W::I16),
    ],
};

const SCALED_IMU: MsgSpec = MsgSpec {
    name: "SCALED_IMU",
    min_payload: 22,

    fields: &[
        ("xacc", 4, W::I16),
        ("yacc", 6, W::I16),
        ("zacc", 8, W::I16),
        ("xgyro", 10, W::I16),
        ("ygyro", 12, W::I16),
        ("zgyro", 14, W::I16),
        ("xmag", 16, W::I16),
        ("ymag", 18, W::I16),
        ("zmag", 20, W::I16),
    ],
};

const GPS_RAW_INT: MsgSpec = MsgSpec {
    name: "GPS_RAW_INT",
    min_payload: 30,

    fields: &[
        ("lat", 8, W::I32),
        ("lon", 12, W::I32),
        ("alt", 16, W::I32),
        ("eph", 20, W::U16),
        ("epv", 22, W::U16),
        ("vel", 24, W::U16),
        ("cog", 26, W::U16),
        ("fix_type", 28, W::U8),
        ("satellites_visible", 29, W::U8),
    ],
};

const GLOBAL_POSITION_INT: MsgSpec = MsgSpec {
    name: "GLOBAL_POSITION_INT",
    min_payload: 28,

    fields: &[
        ("lat", 4, W::I32),
        ("lon", 8, W::I32),
        ("alt", 12, W::I32),
        ("relative_alt", 16, W::I32),
        ("vx", 20, W::I16),
        ("vy", 22, W::I16),
        ("vz", 24, W::I16),
        ("hdg", 26, W::U16),
    ],
};

const SERVO_OUTPUT_RAW: MsgSpec = MsgSpec {
    name: "SERVO_OUTPUT_RAW",
    min_payload: 21,

    fields: &[
        ("servo1_raw", 4, W::U16),
        ("servo2_raw", 6, W::U16),
        ("servo3_raw", 8, W::U16),
        ("servo4_raw", 10, W::U16),
        ("servo5_raw", 12, W::U16),
        ("servo6_raw", 14, W::U16),
        ("servo7_raw", 16, W::U16),
        ("servo8_raw", 18, W::U16),
    ],
};

const RC_CHANNELS_RAW: MsgSpec = MsgSpec {
    name: "RC_CHANNELS_RAW",
    min_payload: 22,

    fields: &[
        ("chan1_raw", 4, W::U16),
        ("chan2_raw", 6, W::U16),
        ("chan3_raw", 8, W::U16),
        ("chan4_raw", 10, W::U16),
        ("chan5_raw", 12, W::U16),
        ("chan6_raw", 14, W::U16),
        ("chan7_raw", 16, W::U16),
        ("chan8_raw", 18, W::U16),
    ],
};

const RC_CHANNELS: MsgSpec = MsgSpec {
    name: "RC_CHANNELS",
    min_payload: 42,

    fields: &[
        ("chan1_raw", 4, W::U16),
        ("chan2_raw", 6, W::U16),
        ("chan3_raw", 8, W::U16),
        ("chan4_raw", 10, W::U16),
        ("chan5_raw", 12, W::U16),
        ("chan6_raw", 14, W::U16),
        ("chan7_raw", 16, W::U16),
        ("chan8_raw", 18, W::U16),
        ("chan9_raw", 20, W::U16),
        ("chan10_raw", 22, W::U16),
        ("chan11_raw", 24, W::U16),
        ("chan12_raw", 26, W::U16),
        ("chan13_raw", 28, W::U16),
        ("chan14_raw", 30, W::U16),
        ("chan15_raw", 32, W::U16),
        ("chan16_raw", 34, W::U16),
        ("chan17_raw", 36, W::U16),
        ("chan18_raw", 38, W::U16),
        ("chancount", 40, W::U8),
    ],
};

const VFR_HUD: MsgSpec = MsgSpec {
    name: "VFR_HUD",
    min_payload: 20,

    fields: &[
        ("airspeed", 0, W::F32),
        ("groundspeed", 4, W::F32),
        ("alt", 8, W::F32),
        ("climb", 12, W::F32),
        ("heading", 16, W::I16),
        ("throttle", 18, W::U16),
    ],
};

const SYS_STATUS: MsgSpec = MsgSpec {
    name: "SYS_STATUS",
    min_payload: 31,

    fields: &[
        ("voltage_battery", 14, W::U16),
        ("current_battery", 16, W::I16),
        ("battery_remaining", 30, W::I8),
    ],
};

const HEARTBEAT: MsgSpec = MsgSpec {
    name: "HEARTBEAT",
    min_payload: 9,

    fields: &[
        ("custom_mode", 0, W::U32),
        ("type", 4, W::U8),
        ("autopilot", 5, W::U8),
        ("base_mode", 6, W::U8),
        ("system_status", 7, W::U8),
    ],
};

const VIBRATION: MsgSpec = MsgSpec {
    name: "VIBRATION",
    min_payload: 32,

    fields: &[
        ("vibration_x", 8, W::F32),
        ("vibration_y", 12, W::F32),
        ("vibration_z", 16, W::F32),
        ("clipping_0", 20, W::U32),
        ("clipping_1", 24, W::U32),
        ("clipping_2", 28, W::U32),
    ],
};

fn msg_spec(msg_id: u32) -> Option<&'static MsgSpec> {
    match msg_id {
        0 => Some(&HEARTBEAT),
        1 => Some(&SYS_STATUS),
        24 => Some(&GPS_RAW_INT),
        26 => Some(&SCALED_IMU),
        27 => Some(&RAW_IMU),
        30 => Some(&ATTITUDE),
        33 => Some(&GLOBAL_POSITION_INT),
        35 => Some(&RC_CHANNELS_RAW),
        36 => Some(&SERVO_OUTPUT_RAW),
        65 => Some(&RC_CHANNELS),
        74 => Some(&VFR_HUD),
        241 => Some(&VIBRATION),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Generic field decoder
// ---------------------------------------------------------------------------

fn decode_fields(
    payload: &[u8],
    spec: &MsgSpec,
    tlog_ts: u64,
    topics: &mut HashMap<String, MsgColumns>,
    row_buf: &mut Vec<f64>,
) {
    if payload.len() < spec.min_payload {
        return;
    }

    // Always use tlog receive timestamp (absolute, monotonically increasing).
    // Boot-relative timestamps (time_boot_ms, time_usec) can wrap across
    // reboots within a single tlog file.
    let ts = tlog_ts;

    let mc = topics.entry(spec.name.to_string()).or_insert_with(|| {
        MsgColumns::new(
            spec.fields
                .iter()
                .map(|(name, _, _)| (*name).to_string())
                .collect(),
        )
    });

    row_buf.clear();
    for &(_, offset, wt) in spec.fields {
        row_buf.push(decode_w(payload, offset, wt));
    }
    mc.push_row(ts, row_buf);
}

// ---------------------------------------------------------------------------
// Special message handlers
// ---------------------------------------------------------------------------

fn decode_statustext(payload: &[u8], tlog_ts: u64) -> Option<StatusMessage> {
    if payload.len() < 2 {
        return None;
    }
    let text_end = payload[1..]
        .iter()
        .position(|&b| b == 0)
        .map_or(payload.len() - 1, |p| p);
    Some(StatusMessage {
        timestamp_us: tlog_ts,
        severity: Severity::from_id(payload[0]),
        text: String::from_utf8_lossy(&payload[1..=text_end]).to_string(),
    })
}

fn is_firmware_version(text: &str) -> bool {
    [
        "APM:",
        "ArduCopter",
        "ArduPlane",
        "ArduRover",
        "ArduSub",
        "Copter",
        "Plane",
    ]
    .iter()
    .any(|prefix| text.contains(prefix))
}

fn decode_param_value(payload: &[u8]) -> Option<(String, f64)> {
    if payload.len() < 22 {
        return None;
    }
    let value = f64::from(f32::from_le_bytes(
        payload[0..4].try_into().unwrap_or([0; 4]),
    ));
    let name_end = payload[8..24].iter().position(|&b| b == 0).unwrap_or(16);
    let name = String::from_utf8_lossy(&payload[8..8 + name_end]).to_string();
    if name.is_empty() {
        None
    } else {
        Some((name, value))
    }
}

// ---------------------------------------------------------------------------
// Main parse loop
// ---------------------------------------------------------------------------

/// Parse a `MAVLink` telemetry log (.tlog) file.
#[allow(clippy::too_many_lines)]
pub(crate) fn parse(data: &[u8], warnings: &mut Vec<Warning>) -> MavlinkSession {
    let mut topics: HashMap<String, MsgColumns> = HashMap::new();
    let mut params: HashMap<String, f64> = HashMap::new();
    let mut firmware_version = String::new();
    let mut vehicle_type = MavType::default();
    let mut status_messages: Vec<StatusMessage> = Vec::new();
    let mut stats = MavlinkParseStats::default();
    let mut row_buf: Vec<f64> = Vec::new();

    let mut pos: usize = 0;

    while pos + 9 <= data.len() {
        let marker = data[pos + 8];
        if marker != MARKER_V1 && marker != MARKER_V2 {
            stats.corrupt_bytes += 1;
            pos += 1;
            continue;
        }

        let tlog_ts = u64::from_be_bytes(data[pos..pos + 8].try_into().unwrap_or([0; 8]));
        let frame_start = pos + 8;

        // Parse frame header and validate CRC
        let (payload, frame_end, sysid, msg_id, is_v2) = match marker {
            MARKER_V1 => {
                if frame_start + 6 > data.len() {
                    stats.truncated = true;
                    break;
                }
                let payload_len = data[frame_start + 1] as usize;
                let frame_end = frame_start + 6 + payload_len + 2;
                if frame_end > data.len() {
                    stats.truncated = true;
                    break;
                }
                let crc_received = u16::from_le_bytes([data[frame_end - 2], data[frame_end - 1]]);
                let msg_id = u32::from(data[frame_start + 5]);
                if let Some(extra) = crc_extra_for(msg_id) {
                    let crc = mavlink_crc(&data[frame_start + 1..frame_end - 2], extra);
                    if crc != crc_received {
                        stats.crc_errors += 1;
                        pos += 1;
                        continue;
                    }
                }
                let payload = &data[frame_start + 6..frame_end - 2];
                (payload, frame_end, data[frame_start + 3], msg_id, false)
            }
            MARKER_V2 => {
                if frame_start + 10 > data.len() {
                    stats.truncated = true;
                    break;
                }
                let payload_len = data[frame_start + 1] as usize;
                let incompat_flags = data[frame_start + 2];
                let sig_len = if incompat_flags & 0x01 != 0 { 13 } else { 0 };
                let frame_end = frame_start + 10 + payload_len + 2 + sig_len;
                if frame_end > data.len() {
                    stats.truncated = true;
                    break;
                }
                let msg_id = u32::from_le_bytes([
                    data[frame_start + 7],
                    data[frame_start + 8],
                    data[frame_start + 9],
                    0,
                ]);
                let crc_off = frame_start + 10 + payload_len;
                let crc_received = u16::from_le_bytes([data[crc_off], data[crc_off + 1]]);
                if let Some(extra) = crc_extra_for(msg_id) {
                    let crc = mavlink_crc(&data[frame_start + 1..crc_off], extra);
                    if crc != crc_received {
                        stats.crc_errors += 1;
                        pos += 1;
                        continue;
                    }
                }
                let payload = &data[frame_start + 10..frame_start + 10 + payload_len];
                (payload, frame_end, data[frame_start + 5], msg_id, true)
            }
            _ => unreachable!(),
        };

        // Dispatch to message-specific decoders
        match msg_id {
            0 if sysid != GCS_SYSID => {
                if payload.len() >= 9 && vehicle_type == MavType::Generic {
                    vehicle_type = MavType::from_id(payload[4]);
                }
                decode_fields(payload, &HEARTBEAT, tlog_ts, &mut topics, &mut row_buf);
            }
            22 => {
                if let Some((name, value)) = decode_param_value(payload) {
                    params.insert(name, value);
                }
            }
            253 => {
                if let Some(msg) = decode_statustext(payload, tlog_ts) {
                    if firmware_version.is_empty() && is_firmware_version(&msg.text) {
                        firmware_version.clone_from(&msg.text);
                    }
                    status_messages.push(msg);
                }
            }
            _ => {
                if let Some(spec) = msg_spec(msg_id) {
                    decode_fields(payload, spec, tlog_ts, &mut topics, &mut row_buf);
                }
            }
        }

        stats.total_packets += 1;
        if is_v2 {
            stats.v2_packets += 1;
        } else {
            stats.v1_packets += 1;
        }
        pos = frame_end;
    }

    if stats.corrupt_bytes > 0 {
        warnings.push(Warning {
            message: format!("{} corrupt bytes skipped", stats.corrupt_bytes),
            byte_offset: None,
        });
    }
    if stats.crc_errors > 0 {
        warnings.push(Warning {
            message: format!("{} CRC errors", stats.crc_errors),
            byte_offset: None,
        });
    }

    MavlinkSession {
        topics,
        firmware_version,
        vehicle_type,
        params,
        status_messages,
        stats,
        warnings: Vec::new(),
        session_index: 0,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_heartbeat() {
        // Manually verify CRC for the first HEARTBEAT in our fixture.
        // v1 frame bytes (after the 8-byte tlog timestamp):
        // FE 09 00 FF 00 00 | 00 00 00 00 06 08 00 00 03 | A1 DF
        let header_and_payload: &[u8] = &[
            0x09, // len
            0x00, // seq
            0xFF, // sysid
            0x00, // compid
            0x00, // msgid = HEARTBEAT
            // payload:
            0x00, 0x00, 0x00, 0x00, // custom_mode
            0x06, // type
            0x08, // autopilot
            0x00, // base_mode
            0x00, // system_status
            0x03, // mavlink_version
        ];
        let expected_crc = u16::from_le_bytes([0xA1, 0xDF]);
        let crc = mavlink_crc(header_and_payload, CRC_EXTRA[0]);
        assert_eq!(
            crc, expected_crc,
            "CRC mismatch: got {crc:#06X}, expected {expected_crc:#06X}"
        );
    }

    #[test]
    fn crc_extra_lookup() {
        assert_eq!(crc_extra_for(0), Some(50)); // HEARTBEAT
        assert_eq!(crc_extra_for(30), Some(39)); // ATTITUDE
        assert_eq!(crc_extra_for(253), Some(83)); // STATUSTEXT
        assert_eq!(crc_extra_for(3), None); // unused ID
        assert_eq!(crc_extra_for(256), None); // out of range
    }
}
