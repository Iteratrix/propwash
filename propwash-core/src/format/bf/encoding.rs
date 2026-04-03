use crate::reader::{InternalError, Reader};

/// Sign-extend a value from `bits` width to `i32`.
fn sign_extend(value: u32, bits: u8) -> i32 {
    let shift = 32 - u32::from(bits);
    (value << shift).cast_signed() >> shift
}

/// Read an unsigned variable-byte (LEB128) integer.
pub(crate) fn read_unsigned_vb(r: &mut Reader<'_>) -> Result<u32, InternalError> {
    let mut result: u32 = 0;
    let mut shift: u32 = 0;
    for _ in 0..5 {
        let byte = r.read_byte()?;
        result |= u32::from(byte & 0x7F) << shift;
        if byte & 0x80 == 0 {
            return Ok(result);
        }
        shift += 7;
    }
    Ok(result)
}

/// Read a signed variable-byte integer (zigzag-decoded).
pub(crate) fn read_signed_vb(r: &mut Reader<'_>) -> Result<i32, InternalError> {
    let raw = read_unsigned_vb(r)?;
    // ZigZag decode: (raw >> 1) ^ -(raw & 1)
    Ok((raw >> 1).cast_signed() ^ -((raw & 1).cast_signed()))
}

/// Read a `NEG_14BIT` encoded value (encoding ID 3).
/// Used for battery voltage: stores `(vbatref - actual) & 0x3FFF`, negated.
pub(crate) fn read_neg_14bit(r: &mut Reader<'_>) -> Result<i32, InternalError> {
    let raw = read_unsigned_vb(r)? & 0x3FFF;
    let value = sign_extend(raw, 14);
    Ok(-value)
}

/// Read `TAG8_8SVB` encoded values (encoding ID 6).
/// Up to 8 signed values with a bitmask indicating which are non-zero.
/// Special case: if count == 1, no bitmask — reads directly as signed VB.
pub(crate) fn read_tag8_8svb(r: &mut Reader<'_>, count: usize) -> Result<Vec<i32>, InternalError> {
    if count == 1 {
        return Ok(vec![read_signed_vb(r)?]);
    }

    let header = r.read_byte()?;
    let mut values = Vec::with_capacity(count);
    for i in 0..count {
        if header & (1 << i) != 0 {
            values.push(read_signed_vb(r)?);
        } else {
            values.push(0);
        }
    }
    Ok(values)
}

/// Read `TAG2_3S32` encoded values (encoding ID 7).
/// Packs exactly 3 signed 32-bit values. 2-bit selector chooses packing.
/// Multi-byte values are LITTLE-ENDIAN.
pub(crate) fn read_tag2_3s32(r: &mut Reader<'_>) -> Result<[i32; 3], InternalError> {
    let byte0 = r.read_byte()?;
    let selector = (byte0 >> 6) & 3;

    match selector {
        // 2 bits per value, 1 byte total
        0 => Ok([
            sign_extend(u32::from((byte0 >> 4) & 3), 2),
            sign_extend(u32::from((byte0 >> 2) & 3), 2),
            sign_extend(u32::from(byte0 & 3), 2),
        ]),

        // 4 bits per value, 2 bytes total
        1 => {
            let byte1 = r.read_byte()?;
            Ok([
                sign_extend(u32::from(byte0 & 0x0F), 4),
                sign_extend(u32::from((byte1 >> 4) & 0x0F), 4),
                sign_extend(u32::from(byte1 & 0x0F), 4),
            ])
        }

        // 6 bits per value, 3 bytes total
        2 => {
            let byte1 = r.read_byte()?;
            let byte2 = r.read_byte()?;
            Ok([
                sign_extend(u32::from(byte0 & 0x3F), 6),
                sign_extend(u32::from(byte1 & 0x3F), 6),
                sign_extend(u32::from(byte2 & 0x3F), 6),
            ])
        }

        // Variable bytes per value
        _ => read_tag_variable_3(r, byte0 & 0x3F),
    }
}

/// Read `TAG8_4S16` encoded values (encoding ID 8).
/// Packs exactly 4 signed 16-bit values using nibble-aligned bitstream.
/// Uses bit-level reads to match the firmware's stream positioning.
#[allow(clippy::needless_range_loop)]
pub(crate) fn read_tag8_4s16(r: &mut Reader<'_>) -> Result<[i32; 4], InternalError> {
    let tags = r.read_byte()?;
    if tags == 0 {
        return Ok([0, 0, 0, 0]);
    }

    let mut result = [0i32; 4];

    for i in 0..4 {
        let tag = (tags >> (i * 2)) & 3;

        match tag {
            0 => {} // result[i] already 0

            1 => {
                // 4-bit signed nibble
                let nibble = r.read_bits(4)?;
                result[i] = sign_extend(u32::from(nibble), 4);
            }

            2 => {
                // 8-bit signed
                let byte = r.read_bits(8)?;
                result[i] = i32::from(byte as i8);
            }

            _ => {
                // 16-bit signed, big-endian
                let hi = r.read_bits(8)?;
                let lo = r.read_bits(8)?;
                result[i] = i32::from(i16::from_be_bytes([hi, lo]));
            }
        }
    }
    Ok(result)
}

/// Read `TAG2_3SVARIABLE` encoded values (encoding ID 10).
/// Similar to `TAG2_3S32` but with asymmetric bit widths per selector.
pub(crate) fn read_tag2_3svariable(r: &mut Reader<'_>) -> Result<[i32; 3], InternalError> {
    let byte0 = r.read_byte()?;
    let selector = (byte0 >> 6) & 3;

    match selector {
        // Same as `TAG2_3S32` selector 0
        0 => Ok([
            sign_extend(u32::from((byte0 >> 4) & 3), 2),
            sign_extend(u32::from((byte0 >> 2) & 3), 2),
            sign_extend(u32::from(byte0 & 3), 2),
        ]),

        // 5/5/4 bits
        1 => {
            let byte1 = r.read_byte()?;
            let v0 = sign_extend(u32::from((byte0 >> 1) & 0x1F), 5);
            let v1 = sign_extend(u32::from(((byte0 & 1) << 4) | (byte1 >> 4)), 5);
            let v2 = sign_extend(u32::from(byte1 & 0x0F), 4);
            Ok([v0, v1, v2])
        }

        // 8/7/7 bits
        2 => {
            let byte1 = r.read_byte()?;
            let byte2 = r.read_byte()?;
            let v0 = sign_extend((u32::from(byte0 & 0x3F) << 2) | (u32::from(byte1) >> 6), 8);
            let v1 = sign_extend(
                ((u32::from(byte1) & 0x3F) << 1) | (u32::from(byte2) >> 7),
                7,
            );
            let v2 = sign_extend(u32::from(byte2 & 0x7F), 7);
            Ok([v0, v1, v2])
        }

        // Variable bytes — same as `TAG2_3S32` selector 3
        _ => read_tag_variable_3(r, byte0 & 0x3F),
    }
}

/// Shared variable-length decoder for selector 3 of `TAG2_3S32` and `TAG2_3SVARIABLE`.
fn read_tag_variable_3(r: &mut Reader<'_>, tags: u8) -> Result<[i32; 3], InternalError> {
    let mut values = [0i32; 3];
    let mut tag_bits = tags;
    for value in &mut values {
        let tag = tag_bits & 3;
        tag_bits >>= 2;
        *value = match tag {
            0 => {
                let b = r.read_bytes(1)?;
                i32::from(b[0].cast_signed())
            }
            1 => {
                let b = r.read_bytes(2)?;
                i32::from(i16::from_le_bytes([b[0], b[1]]))
            }
            2 => {
                let b = r.read_bytes(3)?;
                let raw = u32::from(b[0]) | (u32::from(b[1]) << 8) | (u32::from(b[2]) << 16);
                sign_extend(raw, 24)
            }
            _ => {
                let b = r.read_bytes(4)?;
                i32::from_le_bytes([b[0], b[1], b[2], b[3]])
            }
        };
    }
    Ok(values)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unsigned_vb_zero() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_unsigned_vb(&mut r).unwrap(), 0);
    }

    #[test]
    fn unsigned_vb_one_byte_max() {
        let mut r = Reader::new(&[0x7F]);
        assert_eq!(read_unsigned_vb(&mut r).unwrap(), 127);
    }

    #[test]
    fn unsigned_vb_two_bytes() {
        let mut r = Reader::new(&[0x80, 0x01]);
        assert_eq!(read_unsigned_vb(&mut r).unwrap(), 128);
    }

    #[test]
    fn unsigned_vb_large() {
        let mut r = Reader::new(&[0x80, 0x80, 0x01]);
        assert_eq!(read_unsigned_vb(&mut r).unwrap(), 16384);
    }

    #[test]
    fn unsigned_vb_eof() {
        let mut r = Reader::new(&[]);
        assert!(read_unsigned_vb(&mut r).is_err());
    }

    #[test]
    fn signed_vb_zero() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_signed_vb(&mut r).unwrap(), 0);
    }

    #[test]
    fn signed_vb_neg_one() {
        let mut r = Reader::new(&[0x01]);
        assert_eq!(read_signed_vb(&mut r).unwrap(), -1);
    }

    #[test]
    fn signed_vb_pos_one() {
        let mut r = Reader::new(&[0x02]);
        assert_eq!(read_signed_vb(&mut r).unwrap(), 1);
    }

    #[test]
    fn signed_vb_neg_two() {
        let mut r = Reader::new(&[0x03]);
        assert_eq!(read_signed_vb(&mut r).unwrap(), -2);
    }

    #[test]
    fn signed_vb_pos_two() {
        let mut r = Reader::new(&[0x04]);
        assert_eq!(read_signed_vb(&mut r).unwrap(), 2);
    }

    #[test]
    fn neg_14bit_zero() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_neg_14bit(&mut r).unwrap(), 0);
    }

    #[test]
    fn neg_14bit_small() {
        let mut r = Reader::new(&[0x05]);
        assert_eq!(read_neg_14bit(&mut r).unwrap(), -5);
    }

    #[test]
    fn tag8_8svb_single_field() {
        let mut r = Reader::new(&[0x02]);
        assert_eq!(read_tag8_8svb(&mut r, 1).unwrap(), vec![1]);
    }

    #[test]
    fn tag8_8svb_all_zeros() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_tag8_8svb(&mut r, 3).unwrap(), vec![0, 0, 0]);
    }

    #[test]
    fn tag8_8svb_one_nonzero() {
        let mut r = Reader::new(&[0x02, 0x02]);
        assert_eq!(read_tag8_8svb(&mut r, 3).unwrap(), vec![0, 1, 0]);
    }

    #[test]
    fn tag2_3s32_selector0_zeros() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_tag2_3s32(&mut r).unwrap(), [0, 0, 0]);
    }

    #[test]
    fn tag2_3s32_selector0_values() {
        let mut r = Reader::new(&[0x24]);
        assert_eq!(read_tag2_3s32(&mut r).unwrap(), [-2, 1, 0]);
    }

    #[test]
    fn tag2_3s32_selector3_i8() {
        let mut r = Reader::new(&[0xC0, 0x01, 0xFF, 0x80]);
        assert_eq!(read_tag2_3s32(&mut r).unwrap(), [1, -1, -128]);
    }

    #[test]
    fn tag8_4s16_all_zeros() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_tag8_4s16(&mut r).unwrap(), [0, 0, 0, 0]);
    }

    #[test]
    fn tag8_4s16_single_nibble() {
        let mut r = Reader::new(&[0x01, 0x30]);
        let result = read_tag8_4s16(&mut r).unwrap();
        assert_eq!(result[0], 3);
        assert_eq!(result[1], 0);
    }

    #[test]
    fn tag8_4s16_negative_nibble() {
        let mut r = Reader::new(&[0x01, 0xF0]);
        let result = read_tag8_4s16(&mut r).unwrap();
        assert_eq!(result[0], -1);
    }

    #[test]
    fn tag2_3svariable_selector0_zeros() {
        let mut r = Reader::new(&[0x00]);
        assert_eq!(read_tag2_3svariable(&mut r).unwrap(), [0, 0, 0]);
    }

    #[test]
    fn sign_extend_positive() {
        assert_eq!(sign_extend(0b01, 2), 1);
        assert_eq!(sign_extend(0b0111, 4), 7);
    }

    #[test]
    fn sign_extend_negative() {
        assert_eq!(sign_extend(0b10, 2), -2);
        assert_eq!(sign_extend(0b11, 2), -1);
        assert_eq!(sign_extend(0b1000, 4), -8);
    }

    #[test]
    fn sign_extend_6bit() {
        assert_eq!(sign_extend(0b011111, 6), 31);
        assert_eq!(sign_extend(0b100000, 6), -32);
    }
}
