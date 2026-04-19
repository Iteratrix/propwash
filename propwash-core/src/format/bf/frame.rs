use super::encoding::{
    read_neg_14bit, read_signed_vb, read_tag2_3s32, read_tag2_3svariable, read_tag8_4s16,
    read_tag8_8svb, read_unsigned_vb,
};
use super::predictor::{apply_i_predictor, apply_p_predictor, DecodeContext};
use super::types::{
    BfEvent, BfFieldDef, BfFrame, BfFrameKind, BfParseStats, BfSession, Encoding, Predictor,
};
use crate::reader::{InternalError, Reader};
use crate::types::Warning;
use az::WrappingAs;

pub(crate) struct ParsedFrames {
    pub main: Vec<BfFrame>,
    pub slow: Vec<BfFrame>,
    pub gps: Vec<BfFrame>,
    pub gps_home: Option<Vec<i64>>,
    pub events: Vec<BfEvent>,
    pub stats: BfParseStats,
}
use crate::types::{MotorIndex, SensorField};

/// Frame marker bytes in Betaflight blackbox logs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameMarker {
    Intra,
    Inter,
    Slow,
    Gps,
    GpsHome,
    Event,
}

impl TryFrom<u8> for FrameMarker {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            b'I' => Ok(Self::Intra),
            b'P' => Ok(Self::Inter),
            b'S' => Ok(Self::Slow),
            b'G' => Ok(Self::Gps),
            b'H' => Ok(Self::GpsHome),
            b'E' => Ok(Self::Event),
            other => Err(other),
        }
    }
}

// Event type IDs
const EVENT_SYNC_BEEP: u8 = 0;
const EVENT_INFLIGHT_ADJUSTMENT: u8 = 13;
const EVENT_LOGGING_RESUME: u8 = 14;
const EVENT_DISARM: u8 = 15;
const EVENT_FLIGHT_MODE: u8 = 30;
const EVENT_LOG_END: u8 = 255;

/// Parse all frames from binary data for one session.
/// Never panics — collects warnings on corruption and continues.
#[allow(clippy::too_many_lines)]
pub(crate) fn parse_session_frames(
    data: &[u8],
    base_offset: usize,
    session: &BfSession,
    warnings: &mut Vec<Warning>,
) -> ParsedFrames {
    let mut reader = Reader::with_offset(data, base_offset);
    let mut main_frames: Vec<BfFrame> = Vec::new();
    let mut slow_frames: Vec<BfFrame> = Vec::new();
    let mut gps_frames: Vec<BfFrame> = Vec::new();
    let mut gps_home: Option<Vec<i64>> = None;
    let mut events: Vec<BfEvent> = Vec::new();

    let fields = &session.main_field_defs.fields;
    let p_encodings = &session.p_encodings;
    let p_predictors = &session.p_predictors;

    // Pre-cache I-frame encodings (was .collect() per frame)
    let i_encodings: Vec<Encoding> = fields.iter().map(|f| f.encoding).collect();
    // Pre-cache simple frame encodings
    let slow_encodings: Vec<Encoding> = session
        .slow_field_defs
        .as_ref()
        .map_or_else(Vec::new, |d| d.fields.iter().map(|f| f.encoding).collect());
    let gps_encodings: Vec<Encoding> = session
        .gps_field_defs
        .as_ref()
        .map_or_else(Vec::new, |d| d.fields.iter().map(|f| f.encoding).collect());
    let gps_home_encodings: Vec<Encoding> = session
        .gps_home_field_defs
        .as_ref()
        .map_or_else(Vec::new, |d| d.fields.iter().map(|f| f.encoding).collect());

    let motor0_idx = session
        .main_field_defs
        .index_of(&SensorField::Motor(MotorIndex(0)));
    let time_idx = session.main_field_defs.index_of(&SensorField::Time);
    let iter_idx = session
        .main_field_defs
        .index_of(&SensorField::Unknown("loopIteration".to_string()));

    let mut ctx = DecodeContext::new(session);
    let mut stats = BfParseStats::default();
    // Reusable scratch buffer for raw decoded values
    let mut raw_buf: Vec<i32> = Vec::with_capacity(fields.len());

    while !reader.is_exhausted() {
        let frame_start = reader.save_point();

        let marker = match reader.read_byte().map(FrameMarker::try_from) {
            Ok(Ok(m)) => m,
            Err(InternalError::Eof) => break,
            Ok(Err(_)) | Err(_) => {
                stats.corrupt_bytes += 1;
                continue;
            }
        };

        match marker {
            FrameMarker::Intra => {
                let result = decode_i_frame(
                    &mut reader,
                    fields,
                    &i_encodings,
                    session,
                    motor0_idx,
                    &mut raw_buf,
                );
                match result {
                    Ok(frame) => {
                        if validate_next_marker(&reader) {
                            let iteration = iter_idx
                                .and_then(|i| frame.values.get(i).copied())
                                .unwrap_or(0)
                                .wrapping_as::<u32>();
                            ctx.reset_from_i_frame(&frame.values, iteration);
                            main_frames.push(frame);
                            stats.i_frame_count += 1;
                        } else {
                            stats.corrupt_bytes += 1;
                            reader.restore(frame_start);
                            reader.skip(1);
                        }
                    }
                    Err(InternalError::Eof) => break,
                    Err(InternalError::Corrupt) => {
                        stats.corrupt_bytes += 1;
                        reader.restore(frame_start);
                        reader.skip(1);
                        ctx.invalidate();
                    }
                }
            }

            FrameMarker::Inter => {
                if !ctx.is_ready() {
                    stats.corrupt_bytes += 1;
                    continue;
                }
                let skipped = ctx.skipped_frames();
                let (prev1, prev2) = ctx.slices();
                let result = decode_p_frame(
                    &mut reader,
                    fields,
                    p_encodings,
                    p_predictors,
                    prev1,
                    prev2,
                    session,
                    time_idx,
                    skipped,
                    &mut raw_buf,
                );
                match result {
                    Ok(frame) => {
                        if validate_next_marker(&reader) {
                            ctx.advance_from_p_frame(&frame.values);
                            main_frames.push(frame);
                            stats.p_frame_count += 1;
                        } else {
                            stats.corrupt_bytes += 1;
                            reader.restore(frame_start);
                            reader.skip(1);
                        }
                    }
                    Err(InternalError::Eof) => break,
                    Err(InternalError::Corrupt) => {
                        stats.corrupt_bytes += 1;
                        reader.restore(frame_start);
                        reader.skip(1);
                        ctx.invalidate();
                    }
                }
            }

            FrameMarker::Slow => {
                if let Some(slow_defs) = &session.slow_field_defs {
                    match decode_simple_frame(
                        &mut reader,
                        &slow_encodings,
                        &mut raw_buf,
                        slow_defs.len(),
                    ) {
                        Ok(values) => {
                            slow_frames.push(BfFrame {
                                values,
                                kind: BfFrameKind::Intra,
                            });
                            stats.slow_frame_count += 1;
                        }
                        Err(InternalError::Eof) => break,
                        Err(_) => {
                            reader.restore(frame_start);
                            reader.skip(1);
                        }
                    }
                }
            }

            FrameMarker::Gps => {
                if let Some(gps_defs) = &session.gps_field_defs {
                    match decode_simple_frame(
                        &mut reader,
                        &gps_encodings,
                        &mut raw_buf,
                        gps_defs.len(),
                    ) {
                        Ok(values) => {
                            gps_frames.push(BfFrame {
                                values,
                                kind: BfFrameKind::Intra,
                            });
                            stats.gps_frame_count += 1;
                        }
                        Err(InternalError::Eof) => break,
                        Err(_) => {
                            reader.restore(frame_start);
                            reader.skip(1);
                        }
                    }
                }
            }

            FrameMarker::GpsHome => {
                if let Some(home_defs) = &session.gps_home_field_defs {
                    match decode_simple_frame(
                        &mut reader,
                        &gps_home_encodings,
                        &mut raw_buf,
                        home_defs.len(),
                    ) {
                        Ok(values) => {
                            gps_home = Some(values);
                        }
                        Err(InternalError::Eof) => break,
                        Err(_) => {
                            reader.restore(frame_start);
                            reader.skip(1);
                        }
                    }
                }
            }

            FrameMarker::Event => match parse_event(&mut reader) {
                Ok(Some(event)) => {
                    let is_end = match event {
                        BfEvent::LogEnd => true,
                        BfEvent::SyncBeep { .. }
                        | BfEvent::Disarm { .. }
                        | BfEvent::FlightMode { .. }
                        | BfEvent::InflightAdjustment { .. }
                        | BfEvent::LoggingResume { .. }
                        | BfEvent::Unknown { .. } => false,
                    };
                    events.push(event);
                    stats.event_count += 1;
                    if is_end {
                        stats.clean_end = true;
                        break;
                    }
                }
                Ok(None) => {}
                Err(InternalError::Eof) => break,
                Err(_) => {
                    reader.restore(frame_start);
                    reader.skip(1);
                }
            },
        }
    }

    if stats.corrupt_bytes > 0 {
        warnings.push(Warning {
            message: format!(
                "Skipped {} corrupt/invalid bytes during parsing",
                stats.corrupt_bytes
            ),
            byte_offset: None,
        });
    }

    // Apply GPS home offset to reconstruct absolute coordinates
    if let (Some(ref home), Some(gps_defs)) = (&gps_home, &session.gps_field_defs) {
        let coord0_idx = gps_defs.index_of_str("GPS_coord[0]");
        let coord1_idx = gps_defs.index_of_str("GPS_coord[1]");
        for frame in &mut gps_frames {
            if let Some(idx) = coord0_idx {
                if let Some(val) = frame.values.get_mut(idx) {
                    *val = val.wrapping_add(home[0]);
                }
            }
            if let Some(idx) = coord1_idx {
                if let Some(val) = frame.values.get_mut(idx) {
                    *val = val.wrapping_add(home[1]);
                }
            }
        }
    }

    ParsedFrames {
        main: main_frames,
        slow: slow_frames,
        gps: gps_frames,
        gps_home,
        events,
        stats,
    }
}

/// Decode raw field values from the stream, handling grouped encodings.
/// Writes directly into the caller-provided `values` buffer (must be `fields.len()` long).
fn decode_fields(
    reader: &mut Reader<'_>,
    encodings: &[Encoding],
    values: &mut [i32],
) -> Result<(), InternalError> {
    let n = values.len();
    let mut i = 0;

    while i < n {
        let enc = encodings[i];

        match enc {
            Encoding::Null => {
                values[i] = 0;
                i += 1;
            }
            Encoding::UnsignedVb => {
                values[i] = read_unsigned_vb(reader)?.wrapping_as::<i32>();
                i += 1;
            }
            Encoding::Neg14Bit => {
                values[i] = read_neg_14bit(reader)?;
                i += 1;
            }
            Encoding::Tag2_3S32 => {
                let count = count_consecutive(encodings, i, enc, 3);
                let raw = read_tag2_3s32(reader)?;
                values[i..i + count].copy_from_slice(&raw[..count]);
                i += count;
            }
            Encoding::Tag8_4S16 => {
                let count = count_consecutive(encodings, i, enc, 4);
                let raw = read_tag8_4s16(reader)?;
                values[i..i + count].copy_from_slice(&raw[..count]);
                reader.byte_align();
                i += count;
            }
            Encoding::Tag8_8Svb => {
                let count = count_consecutive(encodings, i, enc, 8);
                read_tag8_8svb(reader, count, &mut values[i..i + count])?;
                i += count;
            }
            Encoding::Tag2_3SVariable => {
                let count = count_consecutive(encodings, i, enc, 3);
                let raw = read_tag2_3svariable(reader)?;
                values[i..i + count].copy_from_slice(&raw[..count]);
                i += count;
            }
            Encoding::SignedVb | Encoding::Unknown(_) => {
                values[i] = read_signed_vb(reader)?;
                i += 1;
            }
        }
    }
    Ok(())
}

fn count_consecutive(
    encodings: &[Encoding],
    start: usize,
    target: Encoding,
    max_n: usize,
) -> usize {
    let mut count = 0;
    for &enc in encodings.iter().skip(start).take(max_n) {
        if enc == target {
            count += 1;
        } else {
            break;
        }
    }
    count.max(1)
}

/// Decode an I-frame (keyframe).
#[allow(clippy::too_many_arguments)]
fn decode_i_frame(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
    i_encodings: &[Encoding],
    session: &BfSession,
    motor0_idx: Option<usize>,
    raw_buf: &mut Vec<i32>,
) -> Result<BfFrame, InternalError> {
    raw_buf.resize(fields.len(), 0);
    decode_fields(reader, i_encodings, raw_buf)?;

    let mut values: Vec<i64> = Vec::with_capacity(fields.len());
    for (i, field) in fields.iter().enumerate() {
        let predicted = apply_i_predictor(
            field.predictor,
            raw_buf[i],
            field.value_sign,
            &values,
            motor0_idx,
            session,
        );
        values.push(predicted);
    }

    Ok(BfFrame {
        values,
        kind: BfFrameKind::Intra,
    })
}

/// Decode a P-frame (delta frame).
#[allow(clippy::too_many_arguments)]
fn decode_p_frame(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
    p_encodings: &[Encoding],
    p_predictors: &[Predictor],
    prev1: &[i64],
    prev2: &[i64],
    session: &BfSession,
    time_idx: Option<usize>,
    skipped_frames: u32,
    raw_buf: &mut Vec<i32>,
) -> Result<BfFrame, InternalError> {
    raw_buf.resize(fields.len(), 0);
    decode_fields(reader, p_encodings, raw_buf)?;

    let mut values: Vec<i64> = Vec::with_capacity(fields.len());
    for (j, field) in fields.iter().enumerate() {
        let predictor = p_predictors.get(j).copied().unwrap_or(Predictor::Zero);
        let predicted = apply_p_predictor(
            predictor,
            raw_buf[j],
            j,
            field.value_sign,
            prev1,
            prev2,
            session,
            time_idx,
            skipped_frames,
        );
        values.push(predicted);
    }

    Ok(BfFrame {
        values,
        kind: BfFrameKind::Inter,
    })
}

/// Decode a simple frame (S, G, H) — no predictors, just raw decode.
fn decode_simple_frame(
    reader: &mut Reader<'_>,
    encodings: &[Encoding],
    raw_buf: &mut Vec<i32>,
    n_fields: usize,
) -> Result<Vec<i64>, InternalError> {
    raw_buf.resize(n_fields, 0);
    decode_fields(reader, encodings, raw_buf)?;
    Ok(raw_buf.iter().map(|&v| i64::from(v)).collect())
}

/// Parse an event frame.
fn parse_event(reader: &mut Reader<'_>) -> Result<Option<BfEvent>, InternalError> {
    let event_type = reader.read_byte()?;

    match event_type {
        EVENT_SYNC_BEEP => {
            let time = read_unsigned_vb(reader)?;
            Ok(Some(BfEvent::SyncBeep {
                time_us: u64::from(time),
            }))
        }
        EVENT_FLIGHT_MODE => {
            let flags = read_unsigned_vb(reader)?;
            let last_flags = read_unsigned_vb(reader)?;
            Ok(Some(BfEvent::FlightMode { flags, last_flags }))
        }
        EVENT_INFLIGHT_ADJUSTMENT => {
            let func = reader.read_byte()?;
            if func & 0x80 != 0 {
                let bytes = reader.read_bytes(4)?;
                let float_val = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
                Ok(Some(BfEvent::InflightAdjustment {
                    function: func & 0x7F,
                    value: f64::from(float_val),
                }))
            } else {
                let value = read_signed_vb(reader)?;
                Ok(Some(BfEvent::InflightAdjustment {
                    function: func,
                    value: f64::from(value),
                }))
            }
        }
        EVENT_LOGGING_RESUME => {
            let log_iteration = read_unsigned_vb(reader)?;
            let current_time = read_unsigned_vb(reader)?;
            Ok(Some(BfEvent::LoggingResume {
                log_iteration,
                current_time,
            }))
        }
        EVENT_DISARM => {
            let reason = read_unsigned_vb(reader)?;
            Ok(Some(BfEvent::Disarm { reason }))
        }
        EVENT_LOG_END => {
            const END_MARKER: &[u8] = b"End of log\0";
            if reader.remaining() >= END_MARKER.len() {
                let bytes = reader.read_bytes(END_MARKER.len())?;
                if bytes == END_MARKER {
                    return Ok(Some(BfEvent::LogEnd));
                }
            }
            Err(InternalError::Corrupt)
        }
        _ => Ok(Some(BfEvent::Unknown {
            type_id: event_type,
        })),
    }
}

fn validate_next_marker(reader: &Reader<'_>) -> bool {
    match reader.peek() {
        None => true,
        Some(b) => FrameMarker::try_from(b).is_ok(),
    }
}
