use proptest::prelude::*;

// ═══════════════════════════════════════════════════════════════════
// Property: decode() NEVER panics, regardless of input
// ═══════════════════════════════════════════════════════════════════

proptest! {
    #[test]
    fn decode_never_panics(data in proptest::collection::vec(any::<u8>(), 0..4096)) {
        // This is the single most important property: no input can crash the parser.
        // UnrecognizedFormat is fine for random data.
        if let Ok(log) = propwash_core::decode(&data) {
            let _ = log.session_count();
        }
    }

    #[test]
    fn decode_never_panics_large(data in proptest::collection::vec(any::<u8>(), 0..65536)) {
        if let Ok(log) = propwash_core::decode(&data) {
            let _ = log.session_count();
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // Property: decode() with a valid header prefix + random body never panics
    // ═══════════════════════════════════════════════════════════════

    #[test]
    fn decode_with_header_prefix_never_panics(
        body in proptest::collection::vec(any::<u8>(), 0..4096)
    ) {
        // Prepend a valid header so the parser actually enters the Betaflight path
        let mut data = b"H Product:Blackbox flight data recorder by Nicholas Sherlock\n".to_vec();
        data.extend_from_slice(b"H Data version:2\n");
        data.extend_from_slice(b"H Field I name:loopIteration,time,gyroADC[0]\n");
        data.extend_from_slice(b"H Field I signed:0,0,1\n");
        data.extend_from_slice(b"H Field I predictor:0,0,0\n");
        data.extend_from_slice(b"H Field I encoding:1,1,0\n");
        data.extend_from_slice(b"H Field P predictor:6,2,1\n");
        data.extend_from_slice(b"H Field P encoding:9,0,0\n");
        data.extend_from_slice(&body);

        let log = propwash_core::decode(&data).expect("valid BF header should be recognized");
        // Should find at least 1 session (we gave it a valid header)
        assert!(log.session_count() >= 1);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Property: encoding decoders never panic on arbitrary bytes
// ═══════════════════════════════════════════════════════════════════

proptest! {
    #[test]
    fn random_bytes_after_i_marker_never_panics(
        body in proptest::collection::vec(any::<u8>(), 0..512)
    ) {
        let mut data = b"H Product:Blackbox flight data recorder by Nicholas Sherlock\n".to_vec();
        data.extend_from_slice(b"H Data version:2\n");
        data.extend_from_slice(b"H Field I name:loopIteration,time\n");
        data.extend_from_slice(b"H Field I signed:0,0\n");
        data.extend_from_slice(b"H Field I predictor:0,0\n");
        data.extend_from_slice(b"H Field I encoding:1,1\n");
        data.extend_from_slice(b"H Field P predictor:6,2\n");
        data.extend_from_slice(b"H Field P encoding:9,0\n");
        data.push(b'I');
        data.extend_from_slice(&body);

        let log = propwash_core::decode(&data).expect("valid BF header should be recognized");
        let _ = log.session_count();
    }
}

// ═══════════════════════════════════════════════════════════════════
// Property: ArduPilot parser never panics on random data
// ═══════════════════════════════════════════════════════════════════
// AP DataFlash format starts with FMT messages (header 0xA3 0x95).

proptest! {
    #[test]
    fn ap_parser_never_panics(
        body in proptest::collection::vec(any::<u8>(), 0..4096)
    ) {
        // AP header: 0xA3 0x95 + type 0x80 (FMT) + length 89
        // Minimal FMT message defining the FMT format itself
        let mut data: Vec<u8> = vec![0xA3, 0x95, 0x80];
        // FMT message body: type=128, length=89, name="FMT\0", format="BBnNZ"
        // (simplified — just enough bytes to be recognized as AP)
        data.resize(92, 0); // pad to FMT message length
        data[3] = 128; // type = FMT
        data[4] = 89;  // length = 89
        data[5..9].copy_from_slice(b"FMT\0");
        data.extend_from_slice(&body);

        // Should not panic regardless of body content
        let _ = propwash_core::decode(&data);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Property: PX4 ULog parser never panics on random data
// ═══════════════════════════════════════════════════════════════════

proptest! {
    #[test]
    fn px4_parser_never_panics(
        body in proptest::collection::vec(any::<u8>(), 0..4096)
    ) {
        // ULog magic: 7 bytes + 1 version byte + 8 timestamp bytes = 16 byte header
        let mut data = b"\x55\x4c\x6f\x67\x01\x12\x35".to_vec(); // "ULog" magic
        data.push(1); // version
        data.extend_from_slice(&[0u8; 8]); // timestamp
        data.extend_from_slice(&body);

        let _ = propwash_core::decode(&data);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Property: MAVLink tlog parser never panics on random data
// ═══════════════════════════════════════════════════════════════════

proptest! {
    #[test]
    fn mavlink_parser_never_panics(
        body in proptest::collection::vec(any::<u8>(), 0..4096)
    ) {
        // MAVLink tlog: 8-byte timestamp (reasonable Unix timestamp) + 0xFE marker
        let mut data = Vec::new();
        // Timestamp: 2024-01-01 in microseconds (within 2000-2100 detection range)
        let ts: u64 = 1_704_067_200_000_000;
        data.extend_from_slice(&ts.to_le_bytes());
        data.push(0xFE); // MAVLink v1 marker
        data.extend_from_slice(&body);

        let _ = propwash_core::decode(&data);
    }
}
