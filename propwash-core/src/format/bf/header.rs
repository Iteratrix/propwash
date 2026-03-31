use std::collections::HashMap;

use super::types::{BfFieldDef, BfFieldSign, BfFrameDefs, BfHeaderValue, Encoding, Predictor};
use crate::types::Warning;

/// The exact marker string that begins every Betaflight-family blackbox log session.
const LOG_START_MARKER: &[u8] = b"H Product:Blackbox flight data recorder by Nicholas Sherlock";

/// Find byte offsets of all log sessions in the file.
/// Uses `memchr` for SIMD-accelerated scanning.
pub(crate) fn find_sessions(data: &[u8]) -> Vec<usize> {
    memchr::memmem::find_iter(data, LOG_START_MARKER).collect()
}

/// Result of parsing one session's headers.
pub(crate) struct ParsedHeaders {
    pub raw: HashMap<String, BfHeaderValue>,
    pub firmware_type: String,
    pub firmware_version: String,
    pub craft_name: String,
    pub main_field_defs: BfFrameDefs,
    pub p_encodings: Vec<Encoding>,
    pub p_predictors: Vec<Predictor>,
    pub slow_field_defs: Option<BfFrameDefs>,
    pub gps_field_defs: Option<BfFrameDefs>,
    pub gps_home_field_defs: Option<BfFrameDefs>,
    /// Byte offset where binary frame data begins.
    pub binary_start: usize,
}

/// Parse header lines for one session.
///
/// Reads ASCII `H key:value\n` lines starting at `start`, stopping when
/// a line doesn't begin with `H` (that byte is the first frame marker).
pub(crate) fn parse_headers(
    data: &[u8],
    start: usize,
    end: usize,
    warnings: &mut Vec<Warning>,
) -> ParsedHeaders {
    let mut raw: HashMap<String, BfHeaderValue> = HashMap::new();
    // Collect field definition headers separately: "Field_I" -> { "name" -> [...], "signed" -> [...] }
    let mut field_headers: HashMap<String, HashMap<String, Vec<String>>> = HashMap::new();
    let mut binary_start = end;

    let mut pos = start;
    while pos < end {
        // Find end of line
        let nl = match memchr::memchr(b'\n', &data[pos..end]) {
            Some(offset) => pos + offset,
            None => break,
        };

        let line = &data[pos..nl];
        pos = nl + 1;

        // Header lines start with 'H'
        if line.is_empty() || line[0] != b'H' {
            // This byte is the start of binary data
            binary_start = nl - line.len();
            break;
        }

        // Decode line as ASCII (replace invalid bytes)
        let line_str = String::from_utf8_lossy(line);

        // Strip "H " prefix
        let content = if let Some(stripped) = line_str.strip_prefix("H ") {
            stripped
        } else {
            &line_str[1..] // Just strip the 'H'
        };

        // Split on first ':'
        let Some(colon) = content.find(':') else {
            warnings.push(Warning {
                message: format!("Header line missing colon: {content:?}"),
                byte_offset: Some(pos - line.len() - 1),
            });
            continue;
        };

        let key = content[..colon].trim();
        let value_str = content[colon + 1..].trim();

        // Field definition headers get special handling
        if let Some(rest) = key.strip_prefix("Field ") {
            let parts: Vec<&str> = rest.splitn(2, ' ').collect();
            if parts.len() == 2 {
                let frame_char = parts[0]; // I, P, S, G, H
                let prop = parts[1]; // name, signed, predictor, encoding
                let fkey = format!("Field_{frame_char}");

                let csv_parts: Vec<String> =
                    value_str.split(',').map(|v| v.trim().to_string()).collect();

                field_headers
                    .entry(fkey)
                    .or_default()
                    .insert(prop.to_string(), csv_parts);
            }
            continue;
        }

        // Regular header
        raw.insert(key.to_string(), parse_header_value(value_str));
    }

    // If we didn't find a non-H line, re-scan to find where headers end
    if binary_start == end {
        binary_start = find_binary_start(data, start, end);
    }

    // Build field definitions from collected headers
    let i_defs = field_headers.get("Field_I");
    let p_defs = field_headers.get("Field_P");

    let main_names = i_defs
        .and_then(|d| d.get("name"))
        .cloned()
        .unwrap_or_default();
    let n = main_names.len();

    let main_signed = get_u8_list(i_defs, "signed", n);
    let main_predictors = get_u8_list(i_defs, "predictor", n);
    let main_encodings = get_u8_list(i_defs, "encoding", n);
    let p_predictors: Vec<Predictor> = get_u8_list(p_defs, "predictor", n).into_iter().map(Predictor::from).collect();
    let p_encodings: Vec<Encoding> = get_u8_list(p_defs, "encoding", n).into_iter().map(Encoding::from).collect();

    let main_field_defs =
        build_field_defs(&main_names, &main_signed, &main_predictors, &main_encodings);
    let slow_field_defs = build_frame_defs(&field_headers, "Field_S", warnings);
    let gps_field_defs = build_frame_defs(&field_headers, "Field_G", warnings);
    let gps_home_field_defs = build_frame_defs(&field_headers, "Field_H", warnings);

    // Extract firmware info
    let firmware_type = match raw.get("Firmware type") {
        Some(BfHeaderValue::Str(s)) => s.clone(),
        _ => "unknown".to_string(),
    };
    let firmware_version = match raw.get("Firmware revision") {
        Some(BfHeaderValue::Str(s)) => s.clone(),
        _ => "unknown".to_string(),
    };
    let craft_name = match raw.get("Craft name") {
        Some(BfHeaderValue::Str(s)) => s.clone(),
        _ => String::new(),
    };

    ParsedHeaders {
        raw,
        firmware_type,
        firmware_version,
        craft_name,
        main_field_defs,
        p_encodings,
        p_predictors,
        slow_field_defs,
        gps_field_defs,
        gps_home_field_defs,
        binary_start,
    }
}

/// Find where binary data starts by scanning for the first line after a newline
/// that doesn't begin with 'H'.
fn find_binary_start(data: &[u8], start: usize, end: usize) -> usize {
    let mut scan = start;
    while scan < end {
        let nl = match memchr::memchr(b'\n', &data[scan..end]) {
            Some(offset) => scan + offset,
            None => return scan,
        };
        let next_pos = nl + 1;
        if next_pos >= end {
            return end;
        }
        if data[next_pos] != b'H' {
            return next_pos;
        }
        scan = next_pos;
    }
    end
}

/// Parse a header value string into a typed `BfHeaderValue`.
fn parse_header_value(s: &str) -> BfHeaderValue {
    // Try CSV int list
    if s.contains(',') {
        let parts: Vec<&str> = s.split(',').collect();
        let ints: Result<Vec<i64>, _> = parts.iter().map(|p| p.trim().parse()).collect();
        if let Ok(list) = ints {
            return BfHeaderValue::IntList(list);
        }
        return BfHeaderValue::Str(s.to_string());
    }

    // Try single int
    if let Ok(v) = s.parse::<i64>() {
        return BfHeaderValue::Int(v);
    }

    BfHeaderValue::Str(s.to_string())
}

/// Extract a list of `u8` values from field header CSV, with fallback to zeros.
fn get_u8_list(
    defs: Option<&HashMap<String, Vec<String>>>,
    key: &str,
    expected_len: usize,
) -> Vec<u8> {
    let Some(defs) = defs else {
        return vec![0; expected_len];
    };
    let Some(vals) = defs.get(key) else {
        return vec![0; expected_len];
    };

    let mut result: Vec<u8> = vals.iter().map(|v| v.parse::<u8>().unwrap_or(0)).collect();

    result.resize(expected_len, 0);
    result
}

/// Build `BfFrameDefs` from parallel name/signed/predictor/encoding arrays.
fn build_field_defs(
    names: &[String],
    signed: &[u8],
    predictors: &[u8],
    encodings: &[u8],
) -> BfFrameDefs {
    let fields = names
        .iter()
        .enumerate()
        .map(|(i, name)| {
            let value_sign = match name.as_str() {
                "time" | "loopIteration" => BfFieldSign::Unsigned,
                _ => BfFieldSign::Signed,
            };
            BfFieldDef {
                name: name.clone(),
                signed: signed.get(i).copied().unwrap_or(0) != 0,
                predictor: Predictor::from(predictors.get(i).copied().unwrap_or(0)),
                encoding: Encoding::from(encodings.get(i).copied().unwrap_or(0)),
                value_sign,
            }
        })
        .collect();
    BfFrameDefs::new(fields)
}

/// Build `BfFrameDefs` for a secondary frame type (S, G, H).
fn build_frame_defs(
    field_headers: &HashMap<String, HashMap<String, Vec<String>>>,
    key: &str,
    _warnings: &mut Vec<Warning>,
) -> Option<BfFrameDefs> {
    let defs = field_headers.get(key)?;
    let names = defs.get("name")?;
    if names.is_empty() {
        return None;
    }

    let n = names.len();
    let signed = get_u8_list(Some(defs), "signed", n);
    let predictors = get_u8_list(Some(defs), "predictor", n);
    let encodings = get_u8_list(Some(defs), "encoding", n);

    Some(build_field_defs(names, &signed, &predictors, &encodings))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn fixtures_dir() -> &'static Path {
        Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
    }

    #[test]
    fn find_sessions_in_multi_log() {
        let data = std::fs::read(fixtures_dir().join("fc-blackbox/btfl_all.bbl")).unwrap();
        let offsets = find_sessions(&data);
        assert!(
            offsets.len() >= 10,
            "btfl_all.bbl should have many sessions, got {}",
            offsets.len()
        );
    }

    #[test]
    fn find_sessions_single_log() {
        let data = std::fs::read(fixtures_dir().join("fc-blackbox/btfl_002.bbl")).unwrap();
        let offsets = find_sessions(&data);
        assert!(!offsets.is_empty());
    }

    #[test]
    fn parse_betaflight_headers() {
        let data = std::fs::read(fixtures_dir().join("fc-blackbox/btfl_001.bbl")).unwrap();
        let offsets = find_sessions(&data);
        let end = offsets.get(1).copied().unwrap_or(data.len());

        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        assert!(h.firmware_type.contains("Cleanflight") || h.firmware_type.contains("Betaflight"));
        assert!(h.firmware_version.contains("Betaflight"));
        assert!(
            !h.main_field_defs.is_empty(),
            "should have main field definitions"
        );
        assert!(
            h.binary_start > offsets[0],
            "binary_start should be past the header"
        );
        assert!(
            h.binary_start < end,
            "binary_start should be before session end"
        );

        // Check expected fields
        let names = h.main_field_defs.names();
        assert!(names.contains(&"loopIteration"));
        assert!(names.contains(&"time"));
        assert!(names.contains(&"gyroADC[0]"));
        assert!(names.contains(&"motor[0]"));
    }

    #[test]
    fn parse_emuflight_headers() {
        let data = std::fs::read(fixtures_dir().join("gimbal-ghost/emuf_001.bbl")).unwrap();
        let offsets = find_sessions(&data);
        let end = offsets.get(1).copied().unwrap_or(data.len());

        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        assert!(
            h.firmware_version.contains("EmuFlight"),
            "got: {}",
            h.firmware_version
        );
        assert!(!h.main_field_defs.is_empty());
    }

    #[test]
    fn parse_rotorflight_headers() {
        let data = std::fs::read(fixtures_dir().join("gimbal-ghost/rtfl_001.bbl")).unwrap();
        let offsets = find_sessions(&data);
        let end = offsets.get(1).copied().unwrap_or(data.len());

        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        assert!(
            h.firmware_version.contains("Rotorflight"),
            "got: {}",
            h.firmware_version
        );

        // Rotorflight: should have motor[0] but not motor[1]
        let names = h.main_field_defs.names();
        assert!(names.contains(&"motor[0]"));
    }

    #[test]
    fn parse_cleanflight_headers() {
        let data = std::fs::read(fixtures_dir().join("cleanflight/LOG00568.TXT")).unwrap();
        let offsets = find_sessions(&data);
        assert!(!offsets.is_empty(), "should find at least one session");

        let end = offsets.get(1).copied().unwrap_or(data.len());
        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        assert!(!h.main_field_defs.is_empty());
        let names = h.main_field_defs.names();
        assert!(names.contains(&"time"));
        // Old Cleanflight uses "gyroData" instead of "gyroADC"
        assert!(
            names.contains(&"gyroADC[0]") || names.contains(&"gyroData[0]"),
            "expected gyro field, got: {names:?}"
        );
    }

    #[test]
    fn parse_inav_headers() {
        let data = std::fs::read(fixtures_dir().join("inav/LOG00001.TXT")).unwrap();
        let offsets = find_sessions(&data);
        assert!(!offsets.is_empty());

        let end = offsets.get(1).copied().unwrap_or(data.len());
        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        assert!(!h.main_field_defs.is_empty());
    }

    #[test]
    fn parse_crash_log_headers() {
        let data = std::fs::read(fixtures_dir().join("fc-blackbox/crashing-LOG00002.BFL")).unwrap();
        let offsets = find_sessions(&data);
        assert!(!offsets.is_empty());

        let end = offsets.get(1).copied().unwrap_or(data.len());
        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        // Should still parse headers even if log has corruption later
        assert!(!h.main_field_defs.is_empty());
    }

    #[test]
    fn p_encodings_populated() {
        let data = std::fs::read(fixtures_dir().join("fc-blackbox/btfl_001.bbl")).unwrap();
        let offsets = find_sessions(&data);
        let end = offsets.get(1).copied().unwrap_or(data.len());

        let mut warnings = Vec::new();
        let h = parse_headers(&data, offsets[0], end, &mut warnings);

        assert_eq!(h.p_encodings.len(), h.main_field_defs.len());
        assert_eq!(h.p_predictors.len(), h.main_field_defs.len());
    }

    #[test]
    fn header_value_parsing() {
        assert!(matches!(parse_header_value("42"), BfHeaderValue::Int(42)));
        assert!(matches!(parse_header_value("hello"), BfHeaderValue::Str(_)));
        match parse_header_value("1,2,3") {
            BfHeaderValue::IntList(v) => assert_eq!(v, vec![1, 2, 3]),
            other => panic!("expected IntList, got {other:?}"),
        }
    }
}
