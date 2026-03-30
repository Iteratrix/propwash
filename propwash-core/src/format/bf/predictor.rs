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

/// Frame scheduling: how loopIteration advances between logged frames.
#[derive(Debug, Clone)]
pub(crate) struct FrameSchedule {
    p_ratio: u32,
}

impl FrameSchedule {
    pub fn from_headers(session: &BfRawSession) -> Self {
        #[allow(clippy::cast_sign_loss)]
        let p_ratio = session.get_header_int("P ratio", 1).max(1) as u32;
        Self { p_ratio }
    }

    fn skipped_frames(&self) -> u32 {
        self.p_ratio - 1
    }
}

/// Decode context: frame history + iteration tracking.
/// Encodes the invariant that I-frames reset everything,
/// P-frames advance, and corruption invalidates.
pub(crate) enum DecodeContext {
    Empty {
        schedule: FrameSchedule,
    },
    Ready {
        prev1: Vec<i64>,
        prev2: Vec<i64>,
        iteration: u32,
        schedule: FrameSchedule,
    },
}

impl DecodeContext {
    pub fn new(session: &BfRawSession) -> Self {
        Self::Empty {
            schedule: FrameSchedule::from_headers(session),
        }
    }

    /// Resets from an I-frame. Sets both history slots and the iteration counter.
    pub fn reset_from_i_frame(&mut self, values: &[i64], loop_iteration: u32) {
        let schedule = self.take_schedule();
        *self = Self::Ready {
            prev1: values.to_vec(),
            prev2: values.to_vec(),
            iteration: loop_iteration,
            schedule,
        };
    }

    /// Advances from a P-frame. Shifts history and increments iteration.
    /// Returns the number of skipped frames (for the INCREMENT predictor).
    pub fn advance_from_p_frame(&mut self, values: &[i64]) -> u32 {
        match self {
            Self::Empty { .. } => panic!("advance_from_p_frame called without I-frame"),
            Self::Ready {
                prev1,
                prev2,
                iteration,
                schedule,
            } => {
                let skipped = schedule.skipped_frames();
                *iteration = iteration.wrapping_add(skipped + 1);
                prev2.copy_from_slice(prev1);
                prev1.copy_from_slice(values);
                skipped
            }
        }
    }

    /// Returns whether the context is ready for P-frame decoding.
    pub fn is_ready(&self) -> bool {
        match self {
            Self::Empty { .. } => false,
            Self::Ready { .. } => true,
        }
    }

    /// Invalidates after corruption.
    pub fn invalidate(&mut self) {
        let schedule = self.take_schedule();
        *self = Self::Empty { schedule };
    }

    /// Borrows prev1 and prev2 for prediction.
    pub fn slices(&self) -> (&[i64], &[i64]) {
        match self {
            Self::Empty { .. } => panic!("slices() called on empty context"),
            Self::Ready { prev1, prev2, .. } => (prev1, prev2),
        }
    }

    /// Returns the current `skipped_frames` count for the INCREMENT predictor.
    pub fn skipped_frames(&self) -> u32 {
        self.schedule().skipped_frames()
    }

    fn schedule(&self) -> &FrameSchedule {
        match self {
            Self::Empty { schedule } | Self::Ready { schedule, .. } => schedule,
        }
    }

    fn take_schedule(&mut self) -> FrameSchedule {
        self.schedule().clone()
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
    skipped_frames: u32,
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
        INCREMENT => {
            #[allow(clippy::cast_possible_wrap)]
            let skip = skipped_frames as i32;
            p1.wrapping_add(skip + 1).wrapping_add(decoded)
        }
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
    fn context_reset_sets_both_slots() {
        let mut ctx = make_test_context();
        assert!(!ctx.is_ready());

        ctx.reset_from_i_frame(&[10, 20, 30], 0);
        assert!(ctx.is_ready());

        let (p1, p2) = ctx.slices();
        assert_eq!(p1, &[10, 20, 30]);
        assert_eq!(p2, &[10, 20, 30]);
    }

    #[test]
    fn context_advance_shifts_and_returns_skipped() {
        let mut ctx = make_test_context_with_ratio(8);
        ctx.reset_from_i_frame(&[100, 200], 0);
        let skipped = ctx.advance_from_p_frame(&[110, 210]);

        assert_eq!(skipped, 7); // p_ratio=8, so 7 skipped
        let (p1, p2) = ctx.slices();
        assert_eq!(p1, &[110, 210]);
        assert_eq!(p2, &[100, 200]);
    }

    #[test]
    fn context_invalidate() {
        let mut ctx = make_test_context();
        ctx.reset_from_i_frame(&[1, 2], 0);
        assert!(ctx.is_ready());
        ctx.invalidate();
        assert!(!ctx.is_ready());
    }

    #[test]
    fn increment_with_p_ratio_1() {
        let schedule = FrameSchedule { p_ratio: 1 };
        assert_eq!(schedule.skipped_frames(), 0);
    }

    #[test]
    fn increment_with_p_ratio_8() {
        let schedule = FrameSchedule { p_ratio: 8 };
        assert_eq!(schedule.skipped_frames(), 7);
    }

    #[test]
    fn increment_with_p_ratio_16() {
        let schedule = FrameSchedule { p_ratio: 16 };
        assert_eq!(schedule.skipped_frames(), 15);
    }

    fn make_test_context() -> DecodeContext {
        make_test_context_with_ratio(1)
    }

    fn make_test_context_with_ratio(p_ratio: u32) -> DecodeContext {
        DecodeContext::Empty {
            schedule: FrameSchedule { p_ratio },
        }
    }
}
