use super::types::{BfFieldSign, BfRawSession};

const MINTHROTTLE: u8 = 4;
const MOTOR_0: u8 = 5;
const FIFTEEN_HUNDRED: u8 = 8;
const VBATREF: u8 = 9;
const LAST_MAIN_FRAME_TIME: u8 = 10;
const MINMOTOR: u8 = 11;

const PREVIOUS: u8 = 1;
const STRAIGHT_LINE: u8 = 2;
const AVERAGE_2: u8 = 3;
const INCREMENT: u8 = 6;

/// Frame prediction history. Encodes the invariant that I-frames reset
/// both slots, while P-frames shift them forward.
pub(crate) enum FrameHistory {
    Empty,
    Ready { prev1: Vec<i64>, prev2: Vec<i64> },
}

impl FrameHistory {
    pub fn new(n_fields: usize) -> Self {
        let _ = n_fields;
        Self::Empty
    }

    /// Resets both history slots to the I-frame values.
    pub fn reset_from_i_frame(&mut self, values: &[i64]) {
        match self {
            Self::Empty => {
                *self = Self::Ready {
                    prev1: values.to_vec(),
                    prev2: values.to_vec(),
                };
            }
            Self::Ready { prev1, prev2 } => {
                prev1.copy_from_slice(values);
                prev2.copy_from_slice(values);
            }
        }
    }

    /// Advances history: prev2 takes prev1's old values, prev1 takes the new values.
    pub fn advance_from_p_frame(&mut self, values: &[i64]) {
        match self {
            Self::Empty => panic!("advance_from_p_frame called without I-frame"),
            Self::Ready { prev1, prev2 } => {
                prev2.copy_from_slice(prev1);
                prev1.copy_from_slice(values);
            }
        }
    }

    /// Returns whether history is ready for P-frame decoding.
    pub fn is_ready(&self) -> bool {
        match self {
            Self::Empty => false,
            Self::Ready { .. } => true,
        }
    }

    /// Invalidates history (after corruption).
    pub fn invalidate(&mut self) {
        *self = Self::Empty;
    }

    /// Borrows prev1 and prev2 for prediction. Panics if empty.
    pub fn slices(&self) -> (&[i64], &[i64]) {
        match self {
            Self::Empty => panic!("slices() called on empty history"),
            Self::Ready { prev1, prev2 } => (prev1, prev2),
        }
    }
}

/// Applies predictor for an I-frame field.
pub(crate) fn apply_i_predictor(
    predictor_id: u8,
    decoded: i32,
    sign: BfFieldSign,
    current_values: &[i64],
    motor0_idx: Option<usize>,
    session: &BfRawSession,
) -> i64 {
    let predicted: i32 = match predictor_id {
        MINTHROTTLE => decoded.wrapping_add(session.min_throttle()),
        MOTOR_0 => {
            #[allow(clippy::cast_possible_truncation)]
            let motor0 = motor0_idx
                .and_then(|idx| current_values.get(idx))
                .copied()
                .map_or(0, |v| v as i32);
            decoded.wrapping_add(motor0)
        }
        FIFTEEN_HUNDRED => decoded.wrapping_add(1500),
        VBATREF => decoded.wrapping_add(session.vbat_ref()),
        MINMOTOR => decoded.wrapping_add(session.min_motor()),
        _ => decoded,
    };
    to_i64(predicted, sign)
}

/// Applies predictor for a P-frame field.
#[allow(clippy::too_many_arguments)]
pub(crate) fn apply_p_predictor(
    predictor_id: u8,
    decoded: i32,
    field_idx: usize,
    sign: BfFieldSign,
    prev1: &[i64],
    prev2: &[i64],
    session: &BfRawSession,
    time_idx: Option<usize>,
) -> i64 {
    let p1 = to_i32(prev1.get(field_idx).copied().unwrap_or(0));
    let p2 = to_i32(prev2.get(field_idx).copied().unwrap_or(0));

    let predicted: i32 = match predictor_id {
        PREVIOUS => decoded.wrapping_add(p1),
        STRAIGHT_LINE => {
            let prediction = p1.wrapping_mul(2).wrapping_sub(p2);
            decoded.wrapping_add(prediction)
        }
        AVERAGE_2 => {
            let avg = p1.wrapping_add(p2) / 2;
            decoded.wrapping_add(avg)
        }
        INCREMENT => p1.wrapping_add(1).wrapping_add(decoded),
        MOTOR_0 => {
            let motor0_idx = session.main_field_defs.index_of("motor[0]");
            let motor0 = motor0_idx
                .and_then(|idx| prev1.get(idx))
                .copied()
                .map_or(0, to_i32);
            decoded.wrapping_add(motor0)
        }
        MINTHROTTLE => decoded.wrapping_add(session.min_throttle()),
        MINMOTOR => decoded.wrapping_add(session.min_motor()),
        VBATREF => decoded.wrapping_add(session.vbat_ref()),
        FIFTEEN_HUNDRED => decoded.wrapping_add(1500),
        LAST_MAIN_FRAME_TIME => {
            let t = time_idx
                .and_then(|idx| prev1.get(idx))
                .copied()
                .map_or(0, to_i32);
            decoded.wrapping_add(t)
        }
        _ => decoded,
    };
    to_i64(predicted, sign)
}

#[allow(clippy::cast_sign_loss)]
fn to_i64(value: i32, sign: BfFieldSign) -> i64 {
    match sign {
        BfFieldSign::Unsigned => i64::from(value as u32),
        BfFieldSign::Signed => i64::from(value),
    }
}

#[allow(clippy::cast_possible_truncation)]
fn to_i32(value: i64) -> i32 {
    value as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_i64_signed_positive() {
        assert_eq!(to_i64(42, BfFieldSign::Signed), 42);
    }

    #[test]
    fn to_i64_signed_negative() {
        assert_eq!(to_i64(-1, BfFieldSign::Signed), -1);
    }

    #[test]
    fn to_i64_unsigned_large() {
        assert_eq!(to_i64(-1, BfFieldSign::Unsigned), 0xFFFF_FFFF);
    }

    #[test]
    fn straight_line_wrapping() {
        let p1: i32 = 0x7FFF_FFFF_u32 as i32;
        let p2: i32 = p1.wrapping_sub(4000);
        let prediction = p1.wrapping_mul(2).wrapping_sub(p2);
        let result = 4000_i32.wrapping_add(prediction);
        assert_eq!(
            to_i64(result, BfFieldSign::Unsigned) as u32,
            p1.wrapping_add(8000) as u32
        );
    }

    #[test]
    fn history_reset_sets_both_slots() {
        let mut h = FrameHistory::new(3);
        assert!(!h.is_ready());

        h.reset_from_i_frame(&[10, 20, 30]);
        assert!(h.is_ready());

        let (p1, p2) = h.slices();
        assert_eq!(p1, &[10, 20, 30]);
        assert_eq!(p2, &[10, 20, 30]);
    }

    #[test]
    fn history_advance_shifts() {
        let mut h = FrameHistory::new(2);
        h.reset_from_i_frame(&[100, 200]);
        h.advance_from_p_frame(&[110, 210]);

        let (p1, p2) = h.slices();
        assert_eq!(p1, &[110, 210]);
        assert_eq!(p2, &[100, 200]);
    }

    #[test]
    fn history_invalidate() {
        let mut h = FrameHistory::new(2);
        h.reset_from_i_frame(&[1, 2]);
        assert!(h.is_ready());
        h.invalidate();
        assert!(!h.is_ready());
    }
}
