use crate::types::BfRawSession;

// Predictor IDs from the blackbox format spec.
const MINTHROTTLE: u8 = 4;
const MOTOR_0: u8 = 5;
const FIFTEEN_HUNDRED: u8 = 8;
const VBATREF: u8 = 9;
const LAST_MAIN_FRAME_TIME: u8 = 10;
const MINMOTOR: u8 = 11;

// P-frame only predictor IDs
const PREVIOUS: u8 = 1;
const STRAIGHT_LINE: u8 = 2;
const AVERAGE_2: u8 = 3;
const INCREMENT: u8 = 6;

/// Apply predictor for an I-frame field.
/// `current_values` contains already-decoded fields earlier in the same frame.
/// `motor0_idx` is the index of `motor[0]` for the `MOTOR_0` predictor.
pub(crate) fn apply_i_predictor(
    predictor_id: u8,
    decoded: i32,
    field_name: &str,
    current_values: &[i64],
    motor0_idx: Option<usize>,
    session: &BfRawSession,
) -> i64 {
    let d = i64::from(decoded);
    match predictor_id {
        MINTHROTTLE => wrap(d + i64::from(session.min_throttle()), field_name),
        MOTOR_0 => {
            let motor0 = motor0_idx
                .and_then(|idx| current_values.get(idx).copied())
                .unwrap_or(0);
            wrap(d + motor0, field_name)
        }
        FIFTEEN_HUNDRED => wrap(d + 1500, field_name),
        VBATREF => wrap(d + i64::from(session.vbat_ref()), field_name),
        MINMOTOR => wrap(d + i64::from(session.min_motor()), field_name),
        _ => d,
    }
}

/// Apply predictor for a P-frame field.
#[allow(clippy::too_many_arguments)]
pub(crate) fn apply_p_predictor(
    predictor_id: u8,
    decoded: i32,
    field_idx: usize,
    field_name: &str,
    prev1: &[i64],
    prev2: &[i64],
    session: &BfRawSession,
    time_idx: Option<usize>,
) -> i64 {
    let d = i64::from(decoded);
    let p1 = prev1.get(field_idx).copied().unwrap_or(0);
    let p2 = prev2.get(field_idx).copied().unwrap_or(0);

    match predictor_id {
        PREVIOUS => wrap(d + p1, field_name),
        STRAIGHT_LINE => wrap(d + 2 * p1 - p2, field_name),
        AVERAGE_2 => wrap(d + i64::midpoint(p1, p2), field_name),
        INCREMENT => wrap(p1 + 1 + d, field_name),
        MOTOR_0 => {
            // In P-frames, MOTOR_0 references motor[0] from the previous frame
            let motor0_idx = session.main_field_defs.index_of("motor[0]");
            let motor0 = motor0_idx
                .and_then(|idx| prev1.get(idx).copied())
                .unwrap_or(0);
            wrap(d + motor0, field_name)
        }
        MINTHROTTLE => wrap(d + i64::from(session.min_throttle()), field_name),
        MINMOTOR => wrap(d + i64::from(session.min_motor()), field_name),
        VBATREF => wrap(d + i64::from(session.vbat_ref()), field_name),
        FIFTEEN_HUNDRED => wrap(d + 1500, field_name),
        LAST_MAIN_FRAME_TIME => {
            let t = time_idx
                .and_then(|idx| prev1.get(idx).copied())
                .unwrap_or(0);
            wrap(d + t, field_name)
        }
        _ => d,
    }
}

/// Wrap a value to 32-bit range.
/// `time` and `loopIteration` are unsigned; everything else is signed.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn wrap(value: i64, field_name: &str) -> i64 {
    let masked = value as u32;
    if is_unsigned_field(field_name) {
        i64::from(masked)
    } else {
        i64::from(masked.cast_signed())
    }
}

fn is_unsigned_field(name: &str) -> bool {
    matches!(name, "time" | "loopIteration")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_signed_positive() {
        assert_eq!(wrap(42, "gyroADC[0]"), 42);
    }

    #[test]
    fn wrap_signed_negative() {
        assert_eq!(wrap(-1, "gyroADC[0]"), -1);
    }

    #[test]
    fn wrap_signed_overflow() {
        assert_eq!(wrap(0x1_0000_0000, "motor[0]"), 0);
    }

    #[test]
    fn wrap_unsigned_large() {
        assert_eq!(wrap(0xFFFF_FFFF, "time"), 0xFFFF_FFFF);
    }

    #[test]
    fn wrap_unsigned_overflow() {
        assert_eq!(wrap(0x1_0000_0000, "time"), 0);
    }

    #[test]
    fn wrap_loop_iteration_unsigned() {
        assert_eq!(wrap(0xFFFF_FFFE, "loopIteration"), 0xFFFF_FFFE);
    }
}
