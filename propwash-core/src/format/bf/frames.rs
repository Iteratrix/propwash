//! BF frame iterator: yields a stream of [`BfFrame`] from the binary
//! body of one log session.
//!
//! Stateful: holds the per-frame `prev` buffer for delta predictors,
//! the GPS home reference, the iteration counter, and per-session parse
//! statistics. After iteration the consumer reads `stats`, `gps_home`,
//! and `warnings` off the iterator before dropping it.

use az::WrappingAs;

use super::encoding::{
    read_neg_14bit, read_signed_vb, read_tag2_3s32, read_tag2_3svariable, read_tag8_4s16,
    read_tag8_8svb, read_unsigned_vb,
};
use super::predictor::{
    apply_i_predictor, apply_p_predictor, DecodeContext, FrameSchedule, PredictorRefs,
};
use super::types::{
    BfEvent, BfFieldDef, BfFrameDefs, BfFrameKind, BfHeaderValue, BfParseStats, Encoding,
    Predictor,
};
use crate::reader::{InternalError, Reader};
use crate::types::{SensorField, Warning};

/// One decoded frame from the BF binary stream.
#[derive(Debug, Clone)]
pub(crate) enum BfFrame {
    /// I-frame (keyframe) or P-frame (delta) — `values` parallel to main field defs.
    Main {
        kind: BfFrameKind,
        values: Vec<i64>,
    },
    /// S-frame (slow telemetry) — `values` parallel to slow field defs.
    Slow { values: Vec<i64> },
    /// G-frame (GPS) — `values` parallel to GPS field defs, with the home
    /// offset already applied to coord fields.
    Gps { values: Vec<i64> },
    /// Discrete event marker.
    Event(BfEvent),
}

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

const EVENT_SYNC_BEEP: u8 = 0;
const EVENT_INFLIGHT_ADJUSTMENT: u8 = 13;
const EVENT_LOGGING_RESUME: u8 = 14;
const EVENT_DISARM: u8 = 15;
const EVENT_FLIGHT_MODE: u8 = 30;
const EVENT_LOG_END: u8 = 255;

/// Stateful iterator that lazily decodes frames from the binary body.
pub(crate) struct BfFrames<'a> {
    reader: Reader<'a>,
    fields: &'a [BfFieldDef],
    p_encodings: &'a [Encoding],
    p_predictors: &'a [Predictor],
    slow_defs: Option<&'a BfFrameDefs>,
    gps_defs: Option<&'a BfFrameDefs>,
    gps_home_defs: Option<&'a BfFrameDefs>,
    i_encodings: Vec<Encoding>,
    slow_encodings: Vec<Encoding>,
    gps_encodings: Vec<Encoding>,
    gps_home_encodings: Vec<Encoding>,
    iter_idx: Option<usize>,
    refs: PredictorRefs,
    ctx: DecodeContext,
    raw_buf: Vec<i32>,
    /// GPS home reference coordinates (set by H-frame). The iterator applies
    /// this offset to coord fields of subsequent G-frames before yielding.
    pub gps_home: Option<Vec<i64>>,
    pub stats: BfParseStats,
    pub warnings: Vec<Warning>,
    finished: bool,
}

impl<'a> BfFrames<'a> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        binary: &'a [u8],
        base_offset: usize,
        headers: &std::collections::HashMap<String, BfHeaderValue>,
        main_defs: &'a BfFrameDefs,
        p_encodings: &'a [Encoding],
        p_predictors: &'a [Predictor],
        slow_defs: Option<&'a BfFrameDefs>,
        gps_defs: Option<&'a BfFrameDefs>,
        gps_home_defs: Option<&'a BfFrameDefs>,
        min_motor_override: i32,
    ) -> Self {
        let i_encodings: Vec<Encoding> = main_defs.fields.iter().map(|f| f.encoding).collect();
        let slow_encodings: Vec<Encoding> = slow_defs
            .map_or_else(Vec::new, |d| d.fields.iter().map(|f| f.encoding).collect());
        let gps_encodings: Vec<Encoding> = gps_defs
            .map_or_else(Vec::new, |d| d.fields.iter().map(|f| f.encoding).collect());
        let gps_home_encodings: Vec<Encoding> = gps_home_defs
            .map_or_else(Vec::new, |d| d.fields.iter().map(|f| f.encoding).collect());

        let iter_idx = main_defs.index_of(&SensorField::Unknown("loopIteration".to_string()));
        let refs = PredictorRefs::from_headers(headers, main_defs, min_motor_override);
        let schedule = FrameSchedule::from_headers(headers);

        Self {
            reader: Reader::with_offset(binary, base_offset),
            fields: &main_defs.fields,
            p_encodings,
            p_predictors,
            slow_defs,
            gps_defs,
            gps_home_defs,
            i_encodings,
            slow_encodings,
            gps_encodings,
            gps_home_encodings,
            iter_idx,
            refs,
            ctx: DecodeContext::new(schedule),
            raw_buf: Vec::with_capacity(main_defs.fields.len()),
            gps_home: None,
            stats: BfParseStats::default(),
            warnings: Vec::new(),
            finished: false,
        }
    }

    /// Drain end-of-iteration diagnostics into the warnings collector.
    /// Call after the iterator is exhausted.
    pub fn finalize(&mut self) {
        if self.stats.corrupt_bytes > 0 {
            self.warnings.push(Warning {
                message: format!(
                    "Skipped {} corrupt/invalid bytes during parsing",
                    self.stats.corrupt_bytes
                ),
                byte_offset: None,
            });
        }
    }
}

impl Iterator for BfFrames<'_> {
    type Item = BfFrame;

    #[allow(clippy::too_many_lines)]
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.finished || self.reader.is_exhausted() {
                return None;
            }
            let frame_start = self.reader.save_point();

            let marker = match self.reader.read_byte().map(FrameMarker::try_from) {
                Ok(Ok(m)) => m,
                Err(InternalError::Eof) => return None,
                Ok(Err(_)) | Err(_) => {
                    self.stats.corrupt_bytes += 1;
                    continue;
                }
            };

            match marker {
                FrameMarker::Intra => {
                    let result = decode_i_frame(
                        &mut self.reader,
                        self.fields,
                        &self.i_encodings,
                        &self.refs,
                        &mut self.raw_buf,
                    );
                    match result {
                        Ok((kind, values)) => {
                            if validate_next_marker(&self.reader) {
                                let iteration = self
                                    .iter_idx
                                    .and_then(|i| values.get(i).copied())
                                    .unwrap_or(0)
                                    .wrapping_as::<u32>();
                                self.ctx.reset_from_i_frame(&values, iteration);
                                self.stats.i_frame_count += 1;
                                return Some(BfFrame::Main { kind, values });
                            }
                            self.stats.corrupt_bytes += 1;
                            self.reader.restore(frame_start);
                            self.reader.skip(1);
                        }
                        Err(InternalError::Eof) => return None,
                        Err(InternalError::Corrupt) => {
                            self.stats.corrupt_bytes += 1;
                            self.reader.restore(frame_start);
                            self.reader.skip(1);
                            self.ctx.invalidate();
                        }
                    }
                }

                FrameMarker::Inter => {
                    if !self.ctx.is_ready() {
                        self.stats.corrupt_bytes += 1;
                        continue;
                    }
                    let skipped = self.ctx.skipped_frames();
                    let (prev1, prev2) = self.ctx.slices();
                    let result = decode_p_frame(
                        &mut self.reader,
                        self.fields,
                        self.p_encodings,
                        self.p_predictors,
                        prev1,
                        prev2,
                        &self.refs,
                        skipped,
                        &mut self.raw_buf,
                    );
                    match result {
                        Ok((kind, values)) => {
                            if validate_next_marker(&self.reader) {
                                self.ctx.advance_from_p_frame(&values);
                                self.stats.p_frame_count += 1;
                                return Some(BfFrame::Main { kind, values });
                            }
                            self.stats.corrupt_bytes += 1;
                            self.reader.restore(frame_start);
                            self.reader.skip(1);
                        }
                        Err(InternalError::Eof) => return None,
                        Err(InternalError::Corrupt) => {
                            self.stats.corrupt_bytes += 1;
                            self.reader.restore(frame_start);
                            self.reader.skip(1);
                            self.ctx.invalidate();
                        }
                    }
                }

                FrameMarker::Slow => {
                    if let Some(slow_defs) = self.slow_defs {
                        match decode_simple_frame(
                            &mut self.reader,
                            &self.slow_encodings,
                            &mut self.raw_buf,
                            slow_defs.len(),
                        ) {
                            Ok(values) => {
                                self.stats.slow_frame_count += 1;
                                return Some(BfFrame::Slow { values });
                            }
                            Err(InternalError::Eof) => return None,
                            Err(_) => {
                                self.reader.restore(frame_start);
                                self.reader.skip(1);
                            }
                        }
                    }
                }

                FrameMarker::Gps => {
                    if let Some(gps_defs) = self.gps_defs {
                        match decode_simple_frame(
                            &mut self.reader,
                            &self.gps_encodings,
                            &mut self.raw_buf,
                            gps_defs.len(),
                        ) {
                            Ok(mut values) => {
                                // Apply GPS home offset to absolute coordinates
                                if let Some(home) = self.gps_home.as_deref() {
                                    if let Some(idx) = gps_defs.index_of_str("GPS_coord[0]") {
                                        if let Some(val) = values.get_mut(idx) {
                                            *val = val.wrapping_add(home.first().copied().unwrap_or(0));
                                        }
                                    }
                                    if let Some(idx) = gps_defs.index_of_str("GPS_coord[1]") {
                                        if let Some(val) = values.get_mut(idx) {
                                            *val = val.wrapping_add(home.get(1).copied().unwrap_or(0));
                                        }
                                    }
                                }
                                self.stats.gps_frame_count += 1;
                                return Some(BfFrame::Gps { values });
                            }
                            Err(InternalError::Eof) => return None,
                            Err(_) => {
                                self.reader.restore(frame_start);
                                self.reader.skip(1);
                            }
                        }
                    }
                }

                FrameMarker::GpsHome => {
                    if let Some(home_defs) = self.gps_home_defs {
                        match decode_simple_frame(
                            &mut self.reader,
                            &self.gps_home_encodings,
                            &mut self.raw_buf,
                            home_defs.len(),
                        ) {
                            Ok(values) => {
                                self.gps_home = Some(values);
                            }
                            Err(InternalError::Eof) => return None,
                            Err(_) => {
                                self.reader.restore(frame_start);
                                self.reader.skip(1);
                            }
                        }
                    }
                }

                FrameMarker::Event => match parse_event(&mut self.reader) {
                    Ok(Some(event)) => {
                        let is_end = matches!(event, BfEvent::LogEnd);
                        self.stats.event_count += 1;
                        if is_end {
                            self.stats.clean_end = true;
                            self.finished = true;
                        }
                        return Some(BfFrame::Event(event));
                    }
                    Ok(None) => {}
                    Err(InternalError::Eof) => return None,
                    Err(_) => {
                        self.reader.restore(frame_start);
                        self.reader.skip(1);
                    }
                },
            }
        }
    }
}

// ── Frame decoders ──────────────────────────────────────────────────────────

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

fn decode_i_frame(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
    i_encodings: &[Encoding],
    refs: &PredictorRefs,
    raw_buf: &mut Vec<i32>,
) -> Result<(BfFrameKind, Vec<i64>), InternalError> {
    raw_buf.resize(fields.len(), 0);
    decode_fields(reader, i_encodings, raw_buf)?;
    let mut values: Vec<i64> = Vec::with_capacity(fields.len());
    for (i, field) in fields.iter().enumerate() {
        let predicted =
            apply_i_predictor(field.predictor, raw_buf[i], field.value_sign, &values, refs);
        values.push(predicted);
    }
    Ok((BfFrameKind::Intra, values))
}

#[allow(clippy::too_many_arguments)]
fn decode_p_frame(
    reader: &mut Reader<'_>,
    fields: &[BfFieldDef],
    p_encodings: &[Encoding],
    p_predictors: &[Predictor],
    prev1: &[i64],
    prev2: &[i64],
    refs: &PredictorRefs,
    skipped: u32,
    raw_buf: &mut Vec<i32>,
) -> Result<(BfFrameKind, Vec<i64>), InternalError> {
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
            refs,
            skipped,
        );
        values.push(predicted);
    }
    Ok((BfFrameKind::Inter, values))
}

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
