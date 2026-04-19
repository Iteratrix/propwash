/// Minimum setpoint change (deg/s) to qualify as a "step."
const MIN_STEP_SIZE: f64 = 50.0;
/// Lookback window for step detection (samples).
/// At 1kHz with RC smoothing, a stick input ramps over ~50ms.
const STEP_LOOKBACK: usize = 50;
/// Minimum gap between detected steps (samples) to avoid duplicates.
const STEP_COOLDOWN: usize = 100;

/// Detect indices where the setpoint makes a significant step change.
///
/// Uses a windowed lookback to catch RC-smoothed ramps at high sample rates
/// (e.g., 1kHz with BF rate limiting produces ~1 deg/s per sample deltas).
pub fn detect_steps(setpoint: &[f64]) -> Vec<usize> {
    let mut steps = Vec::new();
    let mut last_step = 0usize;
    let start = STEP_LOOKBACK;

    for i in start..setpoint.len() {
        if i > last_step && i - last_step < STEP_COOLDOWN && last_step >= start {
            continue;
        }
        let delta = (setpoint[i] - setpoint[i - STEP_LOOKBACK]).abs();
        if delta >= MIN_STEP_SIZE {
            steps.push(i);
            last_step = i;
        }
    }
    steps
}

/// Returns the lookback window size (samples before the detected step index
/// where the ramp started). Used for baseline calculation.
pub const fn step_lookback() -> usize {
    STEP_LOOKBACK
}

/// Returns the minimum step size threshold (deg/s).
pub const fn min_step_size() -> f64 {
    MIN_STEP_SIZE
}

/// Returns the step detection cooldown (samples).
#[cfg(test)]
pub const fn step_cooldown() -> usize {
    STEP_COOLDOWN
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_steps_instant_jump() {
        let mut sp = vec![0.0; 200];
        for i in 100..200 {
            sp[i] = 200.0;
        }
        let steps = detect_steps(&sp);
        assert!(!steps.is_empty(), "should detect the step");
        assert!(
            (100..=150).contains(&steps[0]),
            "step should be near the transition"
        );
    }

    #[test]
    fn detect_steps_below_threshold() {
        let mut sp = vec![0.0; 200];
        for i in 100..200 {
            sp[i] = 30.0;
        }
        assert!(detect_steps(&sp).is_empty());
    }

    #[test]
    fn detect_steps_too_short() {
        assert!(detect_steps(&vec![0.0; 10]).is_empty());
    }
}
