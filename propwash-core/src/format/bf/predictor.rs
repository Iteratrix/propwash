use az::WrappingAs;

use super::types::{BfFieldSign, BfFrameDefs, BfHeaderValue, Predictor};
use crate::types::{MotorIndex, SensorField};
use std::collections::HashMap;

/// Read-only context for predictor application — the small handful of
/// header-derived values that the I-frame and P-frame predictors need.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PredictorRefs {
    pub min_throttle: i32,
    pub min_motor: i32,
    pub vbat_ref: i32,
    pub motor0_idx: Option<usize>,
    pub time_idx: Option<usize>,
}

impl PredictorRefs {
    pub fn from_headers(
        headers: &HashMap<String, BfHeaderValue>,
        main_defs: &BfFrameDefs,
        min_motor_override: i32,
    ) -> Self {
        Self {
            min_throttle: BfHeaderValue::int(headers, "minthrottle", 0),
            min_motor: min_motor_override,
            vbat_ref: BfHeaderValue::int(headers, "vbatref", 0),
            motor0_idx: main_defs.index_of(&SensorField::Motor(MotorIndex(0))),
            time_idx: main_defs.index_of(&SensorField::Time),
        }
    }
}

/// Frame scheduling: determines which PID loop iterations are logged.
#[derive(Debug, Clone)]
pub(crate) struct FrameSchedule {
    i_interval: u32,
    p_num: u32,
    p_denom: u32,
}

impl FrameSchedule {
    pub fn from_headers(headers: &HashMap<String, BfHeaderValue>) -> Self {
        Self {
            i_interval: BfHeaderValue::int(headers, "I interval", 1)
                .max(1)
                .cast_unsigned(),
            p_num: BfHeaderValue::int(headers, "P interval", 1)
                .max(1)
                .cast_unsigned(),
            p_denom: BfHeaderValue::int(headers, "P ratio", 1)
                .max(1)
                .cast_unsigned(),
        }
    }

    fn should_have_frame(&self, iteration: u32) -> bool {
        (iteration % self.i_interval + self.p_num - 1) % self.p_denom < self.p_num
    }

    pub fn skipped_after(&self, last_iteration: u32) -> u32 {
        let mut count = 0;
        let mut idx = last_iteration.wrapping_add(1);
        while !self.should_have_frame(idx) {
            count += 1;
            idx = idx.wrapping_add(1);
            if count > self.i_interval {
                break;
            }
        }
        count
    }
}

/// Decode context: frame history + iteration tracking.
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
    pub fn new(schedule: FrameSchedule) -> Self {
        Self::Empty { schedule }
    }

    pub fn reset_from_i_frame(&mut self, values: &[i64], loop_iteration: u32) {
        let schedule = self.take_schedule();
        *self = Self::Ready {
            prev1: values.to_vec(),
            prev2: values.to_vec(),
            iteration: loop_iteration,
            schedule,
        };
    }

    pub fn advance_from_p_frame(&mut self, values: &[i64]) -> u32 {
        match self {
            Self::Empty { .. } => panic!("advance_from_p_frame called without I-frame"),
            Self::Ready {
                prev1,
                prev2,
                iteration,
                schedule,
            } => {
                let skipped = schedule.skipped_after(*iteration);
                *iteration = iteration.wrapping_add(skipped + 1);
                prev2.copy_from_slice(prev1);
                prev1.copy_from_slice(values);
                skipped
            }
        }
    }

    pub fn is_ready(&self) -> bool {
        matches!(self, Self::Ready { .. })
    }

    pub fn invalidate(&mut self) {
        let schedule = self.take_schedule();
        *self = Self::Empty { schedule };
    }

    pub fn slices(&self) -> (&[i64], &[i64]) {
        match self {
            Self::Empty { .. } => panic!("slices() called on empty context"),
            Self::Ready { prev1, prev2, .. } => (prev1, prev2),
        }
    }

    pub fn skipped_frames(&self) -> u32 {
        match self {
            Self::Empty { .. } => 0,
            Self::Ready {
                iteration,
                schedule,
                ..
            } => schedule.skipped_after(*iteration),
        }
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
    predictor: Predictor,
    decoded: i32,
    sign: BfFieldSign,
    current_values: &[i64],
    refs: &PredictorRefs,
) -> i64 {
    let predicted: i32 = match predictor {
        Predictor::MinThrottle => decoded.wrapping_add(refs.min_throttle),
        Predictor::Motor0 => {
            let motor0 = refs
                .motor0_idx
                .and_then(|idx| current_values.get(idx))
                .copied()
                .map_or(0, WrappingAs::wrapping_as);
            decoded.wrapping_add(motor0)
        }
        Predictor::FifteenHundred => decoded.wrapping_add(1500),
        Predictor::VbatRef => decoded.wrapping_add(refs.vbat_ref),
        Predictor::MinMotor => decoded.wrapping_add(refs.min_motor),
        _ => decoded,
    };
    to_i64(predicted, sign)
}

/// Applies predictor for a P-frame field.
#[allow(clippy::too_many_arguments)]
pub(crate) fn apply_p_predictor(
    predictor: Predictor,
    decoded: i32,
    field_idx: usize,
    sign: BfFieldSign,
    prev1: &[i64],
    prev2: &[i64],
    refs: &PredictorRefs,
    skipped_frames: u32,
) -> i64 {
    let p1 = to_i32(prev1.get(field_idx).copied().unwrap_or(0));
    let p2 = to_i32(prev2.get(field_idx).copied().unwrap_or(0));

    let predicted: i32 = match predictor {
        Predictor::Previous => decoded.wrapping_add(p1),
        Predictor::StraightLine => {
            let prediction = p1.wrapping_mul(2).wrapping_sub(p2);
            decoded.wrapping_add(prediction)
        }
        Predictor::Average2 => {
            let avg = p1.wrapping_add(p2) / 2;
            decoded.wrapping_add(avg)
        }
        Predictor::Increment => {
            let skip = skipped_frames.wrapping_as::<i32>();
            p1.wrapping_add(skip + 1).wrapping_add(decoded)
        }
        Predictor::Motor0 => {
            let motor0 = refs
                .motor0_idx
                .and_then(|idx| prev1.get(idx))
                .copied()
                .map_or(0, to_i32);
            decoded.wrapping_add(motor0)
        }
        Predictor::MinThrottle => decoded.wrapping_add(refs.min_throttle),
        Predictor::MinMotor => decoded.wrapping_add(refs.min_motor),
        Predictor::VbatRef => decoded.wrapping_add(refs.vbat_ref),
        Predictor::FifteenHundred => decoded.wrapping_add(1500),
        Predictor::LastMainFrameTime => {
            let t = refs
                .time_idx
                .and_then(|idx| prev1.get(idx))
                .copied()
                .map_or(0, to_i32);
            decoded.wrapping_add(t)
        }
        _ => decoded,
    };
    to_i64(predicted, sign)
}

fn to_i64(value: i32, sign: BfFieldSign) -> i64 {
    match sign {
        BfFieldSign::Unsigned => i64::from(value.cast_unsigned()),
        BfFieldSign::Signed => i64::from(value),
    }
}

fn to_i32(value: i64) -> i32 {
    value.wrapping_as::<i32>()
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

    fn make_test_schedule() -> FrameSchedule {
        FrameSchedule {
            i_interval: 256,
            p_num: 16,
            p_denom: 16,
        }
    }

    #[test]
    fn context_reset_sets_both_slots() {
        let mut ctx = DecodeContext::new(make_test_schedule());
        assert!(!ctx.is_ready());
        ctx.reset_from_i_frame(&[10, 20, 30], 0);
        assert!(ctx.is_ready());
        let (p1, p2) = ctx.slices();
        assert_eq!(p1, &[10, 20, 30]);
        assert_eq!(p2, &[10, 20, 30]);
    }

    #[test]
    fn context_advance_shifts() {
        let mut ctx = DecodeContext::new(make_test_schedule());
        ctx.reset_from_i_frame(&[100, 200], 0);
        let _skipped = ctx.advance_from_p_frame(&[110, 210]);
        let (p1, p2) = ctx.slices();
        assert_eq!(p1, &[110, 210]);
        assert_eq!(p2, &[100, 200]);
    }

    #[test]
    fn context_invalidate() {
        let mut ctx = DecodeContext::new(make_test_schedule());
        ctx.reset_from_i_frame(&[1, 2], 0);
        assert!(ctx.is_ready());
        ctx.invalidate();
        assert!(!ctx.is_ready());
    }

    #[test]
    fn should_have_frame_every_iteration() {
        let schedule = FrameSchedule {
            i_interval: 256,
            p_num: 16,
            p_denom: 16,
        };
        assert_eq!(schedule.skipped_after(0), 0);
        assert_eq!(schedule.skipped_after(1), 0);
        assert_eq!(schedule.skipped_after(100), 0);
    }

    #[test]
    fn should_have_frame_every_other() {
        let schedule = FrameSchedule {
            i_interval: 32,
            p_num: 1,
            p_denom: 2,
        };
        assert!(schedule.should_have_frame(0));
        assert!(!schedule.should_have_frame(1));
        assert!(schedule.should_have_frame(2));
        assert_eq!(schedule.skipped_after(0), 1);
        assert_eq!(schedule.skipped_after(2), 1);
    }

    #[test]
    fn should_have_frame_every_eighth() {
        let schedule = FrameSchedule {
            i_interval: 128,
            p_num: 1,
            p_denom: 8,
        };
        assert!(schedule.should_have_frame(0));
        for i in 1..8 {
            assert!(
                !schedule.should_have_frame(i),
                "frame {i} should be skipped"
            );
        }
        assert!(schedule.should_have_frame(8));
        assert_eq!(schedule.skipped_after(0), 7);
    }
}
