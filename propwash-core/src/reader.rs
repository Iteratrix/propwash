/// Internal parse errors — never exposed to callers.
/// These drive the retry/skip/abort logic inside the frame loop.
#[derive(Debug, Clone)]
pub(crate) enum InternalError {
    /// Ran out of bytes.
    Eof,
    /// Data didn't match expected pattern.
    Corrupt,
}

/// Zero-copy byte reader over a borrowed slice.
/// All parsing operates on `&[u8]` — no allocations for reading.
pub(crate) struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
    /// Absolute offset of `data[0]` in the original file.
    /// Used for `byte_offset` reporting on frames.
    base_offset: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            base_offset: 0,
        }
    }

    /// Create a reader with a base offset (for reporting absolute file positions).
    pub fn with_offset(data: &'a [u8], base_offset: usize) -> Self {
        Self {
            data,
            pos: 0,
            base_offset,
        }
    }

    /// Current position within the slice.
    pub fn pos(&self) -> usize {
        self.pos
    }

    /// Absolute byte offset in the original file.
    pub fn abs_pos(&self) -> usize {
        self.base_offset + self.pos
    }

    /// Bytes remaining.
    pub fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }

    /// True when no bytes remain.
    pub fn is_exhausted(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Read one byte, advancing position.
    pub fn read_byte(&mut self) -> Result<u8, InternalError> {
        if self.pos >= self.data.len() {
            return Err(InternalError::Eof);
        }
        let b = self.data[self.pos];
        self.pos += 1;
        Ok(b)
    }

    /// Read exactly `n` bytes as a slice.
    pub fn read_bytes(&mut self, n: usize) -> Result<&'a [u8], InternalError> {
        let end = self.pos.checked_add(n).ok_or(InternalError::Eof)?;
        if end > self.data.len() {
            return Err(InternalError::Eof);
        }
        let slice = &self.data[self.pos..end];
        self.pos = end;
        Ok(slice)
    }

    /// Peek at the next byte without consuming it.
    pub fn peek(&self) -> Option<u8> {
        self.data.get(self.pos).copied()
    }

    /// Save current position for potential rollback.
    pub fn save_point(&self) -> usize {
        self.pos
    }

    /// Restore to a previously saved position.
    pub fn restore(&mut self, save: usize) {
        self.pos = save;
    }

    /// Skip forward by `n` bytes.
    pub fn skip(&mut self, n: usize) {
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
}
