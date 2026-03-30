use proptest::prelude::*;

// ═══════════════════════════════════════════════════════════════════
// Property: decode() NEVER panics, regardless of input
// ═══════════════════════════════════════════════════════════════════

proptest! {
    #[test]
    fn decode_never_panics(data in proptest::collection::vec(any::<u8>(), 0..4096)) {
        // This is the single most important property: no input can crash the parser
        let log = propwash_core::decode(&data);
        // We don't assert anything about the result — just that we got one
        let _ = log.session_count();
    }

    #[test]
    fn decode_never_panics_large(data in proptest::collection::vec(any::<u8>(), 0..65536)) {
        let log = propwash_core::decode(&data);
        let _ = log.session_count();
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

        let log = propwash_core::decode(&data);
        // Should find at least 1 session (we gave it a valid header)
        assert!(log.session_count() >= 1);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Property: encoding decoders never panic on arbitrary bytes
// ═══════════════════════════════════════════════════════════════════
// These test the internal encoding functions directly via the public API.
// Since we can't access pub(crate) functions from integration tests,
// we test them indirectly through decode().

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
        // Inject an I-frame marker followed by random bytes
        data.push(b'I');
        data.extend_from_slice(&body);

        let log = propwash_core::decode(&data);
        let _ = log.session_count();
    }
}
