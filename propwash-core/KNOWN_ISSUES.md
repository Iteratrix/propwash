# Known Issues

## P-frame decoding differs from official blackbox_decode on some files

**Affected files:** `btfl_002.bbl`, `btfl_all.bbl`, and any file where the
binary data starts with Event/Slow frames before the first I-frame.

**Root cause:** The official Betaflight `blackbox_decode` tool uses bit-level
stream reading with `streamByteAlign()` calls between encoding reads. Our
parser reads whole bytes only. For most encodings this produces identical
results, but `TAG8_4S16` (nibble-aligned bitstream) can leave the official
decoder's stream at a sub-byte position. The subsequent `streamByteAlign()`
skips bits that our parser doesn't, causing P-frame values to diverge slightly.

When the diverged values fail the official decoder's validation check
(`loopIteration` or `time` jump too large), it invalidates the stream and
skips to the next I-frame. Our parser accepts the P-frames, producing
different frame sequences.

**Impact:** Affects ~5% of frames in affected files. Core sensor values
(gyro, motor, time) are close but not identical to the official decoder.
Does not affect FFT/vibration analysis — frequency peaks are the same
regardless of small value differences in individual frames.

**Fix:** Implement bit-level stream reading in `reader.rs` with
`byte_align()` calls matching the official decoder's positions. This is
a substantial change to the Reader abstraction.

## Legacy Cleanflight (Data version 1) partial support

**Affected files:** `cleanflight/LOG00568-572.TXT` (2015-era logs).

**Root cause:** These files use Data version 1, which has different
encoding/predictor behavior in some edge cases. Our parser handles most
frames correctly but diverges after several thousand frames on some files.
`cleanflight/LOG00570.TXT` matches the official decoder perfectly;
others diverge mid-flight.

**Impact:** Only affects 10-year-old log files. All modern Betaflight,
EmuFlight, Rotorflight, and INAV files parse correctly.
