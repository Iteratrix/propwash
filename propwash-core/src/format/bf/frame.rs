use super::encoding::{
    read_neg_14bit, read_signed_vb, read_tag2_3s32, read_tag2_3svariable, read_tag8_4s16,
    read_tag8_8svb, read_unsigned_vb,
};
use super::predictor::{apply_i_predictor, apply_p_predictor, DecodeContext};
use super::types::{BfEvent, BfFieldDef, BfFrame, BfFrameKind, BfParseStats, BfRawSession, Encoding, Predictor};
use crate::types::{MotorIndex, SensorField};
use crate::reader::{InternalError, Reader};
use crate::types::Warning;

// Frame marker bytes
const MARKER_I: u8 = b'I';
const MARKER_P: u8 = b'P';
const MARKER_S: u8 = b'S';
const MARKER_G: u8 = b'G';
const MARKER_H: u8 = b'H';
const MARKER_E: u8 = b'E';

// Event type IDs
const EVENT_SYNC_BEEP: u8 = 0;
const EVENT_INFLIGHT_ADJUSTMENT: u8 = 13;
const EVENT_LOGGING_RESUME: u8 = 14;
const EVENT_DISARM: u8 = 15;
const EVENT_FLIGHT_MODE: u8 = 30;
const EVENT_LOG_END: u8 = 255;

fn is_valid_marker(b: u8) -> bool {
    matches!(
        b,
        MARKER_I | MARKER_P | MARKER_S | MARKER_G | MARKER_H | MARKER_E
    )
}

/// Parse all frames from binary data for one session.
/// Never panics — collects warnings on corruption and continues.
#[allow(clippy::too_many_lines)]
pub(crate) fn parse_session_frames(
    data: &[u8],
    base_offset: usize,
    session: &BfRawSession,
    warnings: &mut Vec<Warning>,
) -> (
    Vec<BfFrame>,
    Vec<BfFrame>,
    Vec<BfFrame>,
    Vec<BfEvent>,
    BfParseStats,
) {
    let mut reader = Reader::with_offset(data, base_offset);
    let mut main_frames: Vec<BfFrame> = Vec::new();
    let mut slow_frames: Vec<BfFrame> = Vec::new();
    let mut gps_frames: Vec<BfFrame> = Vec::new();
    let mut events: Vec<BfEvent> = Vec::new();

    let fields = &session.main_field_defs.fields;
    let p_encodings = &session.p_encodings;
    let p_predictors = &session.p_predictors;

    let motor0_idx = session.main_field_defs.index_of(&SensorField::Motor(MotorIndex(0)));
    let time_idx = session.main_field_defs.index_of(&SensorField::Time);
    let iter_idx = session.main_field_defs.index_of(&SensorField::LoopIteration);

    let mut ctx = DecodeContext::new(session);
    let mut frame_index: usize = 0;
    let mut stats = BfParseStats::default();

    while !reader.is_exhausted() {
        let frame_start = reader.save_point();
        let frame_abs_offset = reader.abs_pos();

        let marker = match reader.read_byte() {
            Ok(b) => b,
            Err(InternalError::Eof) => break,
            Err(_) => {
                stats.corrupt_bytes += 1;
                continue;
            }
        };

        if !is_valid_marker(marker) {
            stats.corrupt_bytes += 1;
            continue;
        }

        match marker {
            MARKER_I => {
                let result = decode_i_frame(
                    &mut reader,
                    fields,
                    session,
                    motor0_idx,
                    frame_abs_offset,
                    frame_index,
                );
                match result {
                    Ok(frame) => {
                        if validate_next_marker(&reader) {
                            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                            let iteration = iter_idx
                                .and_then(|i| frame.values.get(i).copied())
                                .unwrap_or(0) as u32;
                            ctx.reset_from_i_frame(&frame.values, iteration);
                            main_frames.push(frame);
                            frame_index += 1;
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

            MARKER_P => {
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
                    frame_abs_offset,
                    frame_index,
                );
                match result {
                    Ok(frame) => {
                        if validate_next_marker(&reader) {
                            ctx.advance_from_p_frame(&frame.values);
                            main_frames.push(frame);
                            frame_index += 1;
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

            MARKER_S => {
                if let Some(slow_defs) = &session.slow_field_defs {
                    match decode_simple_frame(&mut reader, &slow_defs.fields) {
                        Ok(values) => {
                            slow_frames.push(BfFrame {
                                values,
                                kind: BfFrameKind::Intra,
                                byte_offset: frame_abs_offset,
                                frame_index: slow_frames.len(),
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

            MARKER_G => {
                if let Some(gps_defs) = &session.gps_field_defs {
                    match decode_simple_frame(&mut reader, &gps_defs.fields) {
                        Ok(values) => {
                            gps_frames.push(BfFrame {
                                values,
                                kind: BfFrameKind::Intra,
                                byte_offset: frame_abs_offset,
                                frame_index: gps_frames.len(),
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

            MARKER_H => {
                if let Some(home_defs) = &session.gps_home_field_defs {
                    if decode_simple_frame(&mut reader, &home_defs.fields).is_err() {
                        reader.restore(frame_start);
                        reader.skip(1);
                    }
                }
            }

            MARKER_E => match parse_event(&mut reader) {
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

            _ => {}
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

    (main_frames, slow_frames, gps_frames, events, stats)
}

/// Decode raw field values from the stream, handling grouped encodings.
/// Returns a Vec of i32 values in field-definition order.
fn decode_fields(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
    encodings: &[Encoding],
) -> Result<Vec<i32>, InternalError> {
    let n = fields.len();
    let mut values = vec![0i32; n];
    let mut i = 0;

    while i < n {
        let enc = encodings.get(i).copied().unwrap_or(Encoding::SignedVb);

        match enc {
            Encoding::Null => {
                i += 1;
            }
            Encoding::UnsignedVb => {
                #[allow(clippy::cast_possible_wrap)]
                {
                    values[i] = read_unsigned_vb(reader)? as i32;
                }
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
                i += count;
            }
            Encoding::Tag8_8Svb => {
                let count = count_consecutive(encodings, i, enc, 8);
                let raw = read_tag8_8svb(reader, count)?;
                values[i..i + count].copy_from_slice(&raw[..count]);
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
    Ok(values)
}

fn count_consecutive(encodings: &[Encoding], start: usize, target: Encoding, max_n: usize) -> usize {
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
fn decode_i_frame(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
    session: &BfRawSession,
    motor0_idx: Option<usize>,
    byte_offset: usize,
    frame_index: usize,
) -> Result<BfFrame, InternalError> {
    let encodings: Vec<Encoding> = fields.iter().map(|f| f.encoding).collect();
    let raw = decode_fields(reader, fields, &encodings)?;

    // Apply I-frame predictors in order (MOTOR_0 depends on motor[0])
    let mut values: Vec<i64> = Vec::with_capacity(fields.len());
    for (i, field) in fields.iter().enumerate() {
        let predicted = apply_i_predictor(
            field.predictor,
            raw[i],
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
        byte_offset,
        frame_index,
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
    session: &BfRawSession,
    time_idx: Option<usize>,
    skipped_frames: u32,
    byte_offset: usize,
    frame_index: usize,
) -> Result<BfFrame, InternalError> {
    let raw = decode_fields(reader, fields, p_encodings)?;

    let mut values: Vec<i64> = Vec::with_capacity(fields.len());
    for (j, field) in fields.iter().enumerate() {
        let predictor = p_predictors.get(j).copied().unwrap_or(Predictor::Zero);
        let predicted = apply_p_predictor(
            predictor,
            raw[j],
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
        byte_offset,
        frame_index,
    })
}

/// Decode a simple frame (S, G, H) — no predictors, just raw decode.
fn decode_simple_frame(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
) -> Result<Vec<i64>, InternalError> {
    let encodings: Vec<Encoding> = fields.iter().map(|f| f.encoding).collect();
    let raw = decode_fields(reader, fields, &encodings)?;
    Ok(raw.into_iter().map(i64::from).collect())
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
                reader.read_bytes(4)?;
                Ok(Some(BfEvent::InflightAdjustment {
                    function: func & 0x7F,
                    value: 0,
                }))
            } else {
                let value = read_signed_vb(reader)?;
                Ok(Some(BfEvent::InflightAdjustment {
                    function: func,
                    value,
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
        Some(b) => is_valid_marker(b),
    }
}
