/// Internal parse errors — never exposed to callers.
/// These drive the retry/skip/abort logic inside the frame loop.
#[derive(Debug, Clone)]
pub(crate) enum InternalError {
    /// Ran out of bytes.
    Eof,
    /// Data didn't match expected pattern.
    Corrupt,
}

use az::Az;

/// Zero-copy byte reader over a borrowed slice with bit-level positioning.
/// Supports sub-byte reads for `TAG8_4S16` nibble-aligned encoding,
/// and `byte_align()` to match the firmware's `streamByteAlign()`.
#[allow(dead_code)]
pub(crate) struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
    /// Bit offset within the current byte (0-7). 0 = byte-aligned.
    bit_pos: u8,
    /// Absolute offset of `data[0]` in the original file.
    base_offset: usize,
}

impl<'a> Reader<'a> {
    #[cfg(test)]
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            bit_pos: 0,
            base_offset: 0,
        }
    }

    /// Create a reader with a base offset (for reporting absolute file positions).
    pub fn with_offset(data: &'a [u8], base_offset: usize) -> Self {
        Self {
            data,
            pos: 0,
            bit_pos: 0,
            base_offset,
        }
    }

    /// Current position within the slice.
    #[cfg(test)]
    pub fn pos(&self) -> usize {
        self.pos
    }

    /// Absolute byte offset in the original file.
    #[allow(dead_code)]
    pub fn abs_pos(&self) -> usize {
        self.base_offset + self.pos
    }

    /// Bytes remaining (rounds down if mid-byte).
    pub fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }

    /// True when no bytes remain.
    pub fn is_exhausted(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Align to the next byte boundary. If already aligned, does nothing.
    /// Matches the firmware's `streamByteAlign()`.
    pub fn byte_align(&mut self) {
        if self.bit_pos > 0 {
            self.bit_pos = 0;
            self.pos += 1;
        }
    }

    /// Read one byte, advancing position. Auto-aligns to byte boundary first.
    #[inline]
    pub fn read_byte(&mut self) -> Result<u8, InternalError> {
        if self.bit_pos > 0 {
            self.bit_pos = 0;
            self.pos += 1;
        }
        if self.pos >= self.data.len() {
            return Err(InternalError::Eof);
        }
        let b = self.data[self.pos];
        self.pos += 1;
        Ok(b)
    }

    /// Read exactly `n` bytes as a slice. Auto-aligns to byte boundary first.
    pub fn read_bytes(&mut self, n: usize) -> Result<&'a [u8], InternalError> {
        self.byte_align();
        let end = self.pos.checked_add(n).ok_or(InternalError::Eof)?;
        if end > self.data.len() {
            return Err(InternalError::Eof);
        }
        let slice = &self.data[self.pos..end];
        self.pos = end;
        Ok(slice)
    }

    /// Read `n` bits (1-8) from the stream. Returns them right-aligned in a `u8`.
    /// Used by `TAG8_4S16` for nibble-aligned reads.
    pub fn read_bits(&mut self, n: u8) -> Result<u8, InternalError> {
        debug_assert!(n > 0 && n <= 8);

        if self.pos >= self.data.len() {
            return Err(InternalError::Eof);
        }

        let bits_left_in_byte = 8 - self.bit_pos;

        if n <= bits_left_in_byte {
            // All bits come from the current byte
            let shift = bits_left_in_byte - n;
            let mask = (1u16 << n) - 1;
            let result = (u16::from(self.data[self.pos]) >> shift) & mask;
            self.bit_pos += n;
            if self.bit_pos >= 8 {
                self.bit_pos = 0;
                self.pos += 1;
            }
            Ok(result.az::<u8>())
        } else {
            // Bits span two bytes
            let from_first = bits_left_in_byte;
            let from_second = n - from_first;
            let first_mask = (1u16 << from_first) - 1;
            let first_bits = u16::from(self.data[self.pos]) & first_mask;

            self.pos += 1;
            if self.pos >= self.data.len() {
                return Err(InternalError::Eof);
            }

            let second_shift = 8 - from_second;
            let second_bits = u16::from(self.data[self.pos]) >> second_shift;

            self.bit_pos = from_second;
            if self.bit_pos >= 8 {
                self.bit_pos = 0;
                self.pos += 1;
            }

            Ok(((first_bits << from_second) | second_bits).az::<u8>())
        }
    }

    /// Peek at the next byte without consuming it. Auto-aligns.
    pub fn peek(&self) -> Option<u8> {
        let pos = if self.bit_pos > 0 {
            self.pos + 1
        } else {
            self.pos
        };
        self.data.get(pos).copied()
    }

    /// Save current position for potential rollback (includes bit position).
    pub fn save_point(&self) -> (usize, u8) {
        (self.pos, self.bit_pos)
    }

    /// Restore to a previously saved position.
    pub fn restore(&mut self, save: (usize, u8)) {
        self.pos = save.0;
        self.bit_pos = save.1;
    }

    /// Skip forward by `n` bytes. Aligns first.
    pub fn skip(&mut self, n: usize) {
        self.byte_align();
        self.pos = (self.pos + n).min(self.data.len());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_byte_basic() {
        let mut r = Reader::new(&[0x42, 0xFF]);
        assert_eq!(r.read_byte().unwrap(), 0x42);
        assert_eq!(r.read_byte().unwrap(), 0xFF);
        assert!(r.read_byte().is_err());
    }

    #[test]
    fn read_bytes_basic() {
        let mut r = Reader::new(&[1, 2, 3, 4, 5]);
        assert_eq!(r.read_bytes(3).unwrap(), &[1, 2, 3]);
        assert_eq!(r.read_bytes(2).unwrap(), &[4, 5]);
        assert!(r.read_bytes(1).is_err());
    }

    #[test]
    fn peek_does_not_advance() {
        let r = Reader::new(&[0xAB]);
        assert_eq!(r.peek(), Some(0xAB));
        assert_eq!(r.peek(), Some(0xAB));
        assert_eq!(r.remaining(), 1);
    }

    #[test]
    fn peek_empty() {
        let r = Reader::new(&[]);
        assert_eq!(r.peek(), None);
    }

    #[test]
    fn save_restore() {
        let mut r = Reader::new(&[1, 2, 3]);
        let save = r.save_point();
        r.read_byte().unwrap();
        r.read_byte().unwrap();
        assert_eq!(r.pos(), 2);
        r.restore(save);
        assert_eq!(r.pos(), 0);
        assert_eq!(r.read_byte().unwrap(), 1);
    }

    #[test]
    fn exhausted() {
        let mut r = Reader::new(&[1]);
        assert!(!r.is_exhausted());
        r.read_byte().unwrap();
        assert!(r.is_exhausted());
    }

    #[test]
    fn abs_offset() {
        let r = Reader::with_offset(&[1, 2, 3], 1000);
        assert_eq!(r.abs_pos(), 1000);
    }

    #[test]
    fn skip_basic() {
        let mut r = Reader::new(&[1, 2, 3, 4, 5]);
        r.skip(3);
        assert_eq!(r.read_byte().unwrap(), 4);
    }

    #[test]
    fn skip_past_end_clamps() {
        let mut r = Reader::new(&[1, 2]);
        r.skip(100);
        assert!(r.is_exhausted());
    }

    #[test]
    fn remaining_tracks_correctly() {
        let mut r = Reader::new(&[1, 2, 3]);
        assert_eq!(r.remaining(), 3);
        r.read_byte().unwrap();
        assert_eq!(r.remaining(), 2);
    }

    #[test]
    fn byte_align_noop_when_aligned() {
        let mut r = Reader::new(&[0xFF, 0x00]);
        r.byte_align();
        assert_eq!(r.pos(), 0);
        assert_eq!(r.read_byte().unwrap(), 0xFF);
    }

    #[test]
    fn read_bits_4_from_aligned() {
        // 0xAB = 1010_1011
        let mut r = Reader::new(&[0xAB]);
        assert_eq!(r.read_bits(4).unwrap(), 0x0A); // top 4 bits: 1010
        assert_eq!(r.read_bits(4).unwrap(), 0x0B); // bottom 4 bits: 1011
        assert!(r.is_exhausted());
    }

    #[test]
    fn read_bits_spanning_bytes() {
        // 0xAB = 1010_1011, 0xCD = 1100_1101
        let mut r = Reader::new(&[0xAB, 0xCD]);
        assert_eq!(r.read_bits(4).unwrap(), 0x0A); // 1010
                                                   // Now at bit_pos=4, read 8 bits spanning both bytes
        assert_eq!(r.read_bits(8).unwrap(), 0xBC); // 1011_1100
        assert_eq!(r.read_bits(4).unwrap(), 0x0D); // 1101
    }

    #[test]
    fn byte_align_after_nibble_read() {
        let mut r = Reader::new(&[0xAB, 0xCD]);
        r.read_bits(4).unwrap(); // read top nibble of 0xAB
        r.byte_align(); // skip remaining 4 bits
        assert_eq!(r.read_byte().unwrap(), 0xCD); // next full byte
    }

    #[test]
    fn read_byte_auto_aligns() {
        let mut r = Reader::new(&[0xAB, 0xCD]);
        r.read_bits(4).unwrap(); // mid-byte
                                 // read_byte should align past remainder of 0xAB
        assert_eq!(r.read_byte().unwrap(), 0xCD);
    }
}
