use az::Az;

use crate::session::TimeSeries;
use crate::units::{Amps, DegPerSec, MetersPerSec, MetersPerSec2, Volts};

// ── Resampling ──────────────────────────────────────────────────────────────
// TODO(refactor/session-typed): drop these `allow(dead_code)`s once the
// analysis layer migrates and starts calling resample() / resample_zoh().

/// Strategy for filling samples at target timestamps that don't align with
/// the source time axis.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Strategy {
    /// Linear interpolation between bracketing samples. Best for continuous
    /// signals (gyro, setpoint, accel). Requires `T: Lerp`.
    Linear,
    /// Zero-order hold: each target sample takes the most-recent source
    /// value at-or-before its timestamp. Right for step-like or
    /// categorical signals (armed, flight mode, vbat readings between
    /// updates).
    StepFill,
    /// Take the temporally nearest source sample.
    Nearest,
}

/// Linear interpolation contract. Implemented for numeric scalars and the
/// unit-typed newtypes built on them.
#[allow(dead_code)]
pub trait Lerp: Copy {
    /// Returns `a + (b - a) * t` where `t ∈ [0.0, 1.0]`.
    fn lerp(a: Self, b: Self, t: f64) -> Self;
}

impl Lerp for f64 {
    #[inline]
    fn lerp(a: Self, b: Self, t: Self) -> Self {
        (b - a).mul_add(t, a)
    }
}

impl Lerp for f32 {
    #[inline]
    fn lerp(a: Self, b: Self, t: f64) -> Self {
        (b - a).mul_add(t.az::<f32>(), a)
    }
}

macro_rules! impl_lerp_via_inner {
    ($newtype:ty, $inner:ty) => {
        impl Lerp for $newtype {
            #[inline]
            fn lerp(a: Self, b: Self, t: f64) -> Self {
                Self(<$inner as Lerp>::lerp(a.0, b.0, t))
            }
        }
    };
}

impl_lerp_via_inner!(DegPerSec, f64);
impl_lerp_via_inner!(MetersPerSec2, f64);
impl_lerp_via_inner!(Volts, f32);
impl_lerp_via_inner!(Amps, f32);
impl_lerp_via_inner!(MetersPerSec, f32);

/// Read-only view over a time-stamped stream — slices instead of an
/// owned `TimeSeries`, so callers with non-`TimeSeries`-shaped storage
/// (e.g. `TriaxialSeries` axes, foreign columns) can resample without
/// allocating.
#[derive(Debug, Clone, Copy)]
pub struct TimeSeriesView<'a, T> {
    pub time_us: &'a [u64],
    pub values: &'a [T],
}

impl<'a, T> From<&'a TimeSeries<T>> for TimeSeriesView<'a, T> {
    fn from(ts: &'a TimeSeries<T>) -> Self {
        Self {
            time_us: &ts.time_us,
            values: &ts.values,
        }
    }
}

/// Resample a time series onto a target timestamp axis.
///
/// `target` must be sorted ascending. The returned `Vec` has the same
/// length as `target`. If `src` is empty, the result is an empty `Vec`
/// (caller should handle missing data — we don't fabricate zeroes).
///
/// For target timestamps before the first source sample, all strategies
/// return the first sample value. For target timestamps after the last
/// source sample, all strategies return the last sample value.
#[allow(dead_code)] // thin TimeSeries-taking wrapper; analysis call sites use resample_view directly
pub fn resample<T: Lerp>(src: &TimeSeries<T>, target: &[u64], strategy: Strategy) -> Vec<T> {
    resample_view(src.into(), target, strategy)
}

/// Same as [`resample`] but takes a [`TimeSeriesView`] (separate
/// `time_us` and `values` slices). Useful when the stream isn't stored
/// as a `TimeSeries` (e.g. one axis of a `TriaxialSeries`).
pub fn resample_view<T: Lerp>(
    src: TimeSeriesView<'_, T>,
    target: &[u64],
    strategy: Strategy,
) -> Vec<T> {
    let n = src.time_us.len();
    if n == 0 || src.values.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(target.len());
    let mut cursor = 0usize;

    for &t in target {
        while cursor + 1 < n && src.time_us[cursor + 1] <= t {
            cursor += 1;
        }

        // Saturating subtractions: defensive against the rare case where
        // a target axis isn't strictly monotonic (corrupt-frame recovery
        // in BF, occasional out-of-order vehicle_status in PX4) so the
        // forward-only cursor outruns t. Result clamps to the last valid
        // sample, matching the boundary clamp at either end.
        let n_values = src.values.len().min(n);
        out.push(if n_values == 0 {
            return Vec::new();
        } else if t <= src.time_us[0] {
            src.values[0]
        } else if cursor + 1 >= n_values || t >= src.time_us[n_values - 1] {
            src.values[n_values - 1]
        } else {
            let t0 = src.time_us[cursor];
            let t1 = src.time_us[cursor + 1];
            let v0 = src.values[cursor];
            let v1 = src.values[cursor + 1];
            let dt_left = t.saturating_sub(t0);
            let dt_right = t1.saturating_sub(t);
            match strategy {
                Strategy::StepFill => v0,
                Strategy::Nearest => {
                    if dt_left <= dt_right {
                        v0
                    } else {
                        v1
                    }
                }
                Strategy::Linear => {
                    let span = t1.saturating_sub(t0);
                    if span == 0 {
                        v0
                    } else {
                        let frac = dt_left.az::<f64>() / span.az::<f64>();
                        T::lerp(v0, v1, frac)
                    }
                }
            }
        });
    }
    out
}

/// Resample for any `Copy` type using zero-order hold (step fill). Use this
/// for bool, enum, and other non-interpolatable streams.
#[allow(dead_code)]
pub fn resample_zoh<T: Copy>(src: &TimeSeries<T>, target: &[u64]) -> Vec<T> {
    resample_zoh_view(src.into(), target)
}

#[allow(dead_code)]
pub fn resample_zoh_view<T: Copy>(src: TimeSeriesView<'_, T>, target: &[u64]) -> Vec<T> {
    let n = src.time_us.len();
    if n == 0 || src.values.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(target.len());
    let mut cursor = 0usize;

    for &t in target {
        while cursor + 1 < n && src.time_us[cursor + 1] <= t {
            cursor += 1;
        }
        out.push(if t < src.time_us[0] {
            src.values[0]
        } else {
            src.values[cursor]
        });
    }
    out
}

// ── Step detection (pre-existing) ───────────────────────────────────────────

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

    fn ts<T>(time_us: Vec<u64>, values: Vec<T>) -> TimeSeries<T> {
        TimeSeries::from_parts(time_us, values)
    }

    #[test]
    fn resample_empty_source_returns_empty() {
        let src: TimeSeries<f64> = TimeSeries::new();
        assert!(resample(&src, &[0, 100, 200], Strategy::Linear).is_empty());
    }

    #[test]
    fn resample_linear_midpoint() {
        let src = ts(vec![0, 1000], vec![0.0, 10.0]);
        let out = resample(&src, &[0, 250, 500, 750, 1000], Strategy::Linear);
        assert_eq!(out, vec![0.0, 2.5, 5.0, 7.5, 10.0]);
    }

    #[test]
    fn resample_step_fill_holds_previous_value() {
        let src = ts(vec![0, 1000, 2000], vec![1.0, 5.0, 9.0]);
        let out = resample(&src, &[0, 500, 1000, 1500, 2000], Strategy::StepFill);
        assert_eq!(out, vec![1.0, 1.0, 5.0, 5.0, 9.0]);
    }

    #[test]
    fn resample_nearest_picks_closer_neighbour() {
        let src = ts(vec![0, 1000], vec![1.0, 5.0]);
        let out = resample(&src, &[100, 400, 500, 600, 900], Strategy::Nearest);
        // tie at 500 → take left (500-0 == 1000-500)
        assert_eq!(out, vec![1.0, 1.0, 1.0, 5.0, 5.0]);
    }

    #[test]
    fn resample_clamps_outside_source_range() {
        let src = ts(vec![100, 200], vec![3.0, 7.0]);
        let out = resample(&src, &[0, 50, 150, 300, 1000], Strategy::Linear);
        assert_eq!(out, vec![3.0, 3.0, 5.0, 7.0, 7.0]);
    }

    #[test]
    fn resample_handles_unit_typed_values() {
        let src = ts(vec![0, 1000], vec![DegPerSec(0.0), DegPerSec(100.0)]);
        let out = resample(&src, &[0, 500, 1000], Strategy::Linear);
        assert_eq!(out, vec![DegPerSec(0.0), DegPerSec(50.0), DegPerSec(100.0)]);
    }

    #[test]
    fn resample_zoh_handles_bool() {
        let src = ts(vec![0, 500, 1000], vec![false, true, false]);
        let out = resample_zoh(&src, &[0, 200, 500, 750, 1000, 1500]);
        assert_eq!(out, vec![false, false, true, true, false, false]);
    }

    #[test]
    fn resample_to_higher_rate_preserves_endpoints() {
        // Source at 1 kHz, target at 4 kHz over the same span.
        let src_t: Vec<u64> = (0..10).map(|i| i * 1000).collect();
        let src_v: Vec<f64> = src_t.iter().map(|&t| t.az::<f64>() / 1000.0).collect();
        let src = ts(src_t, src_v);
        let target: Vec<u64> = (0..=36).map(|i| i * 250).collect();
        let out = resample(&src, &target, Strategy::Linear);
        assert_eq!(out.first().copied(), Some(0.0));
        assert!((out.last().copied().unwrap() - 9.0).abs() < 1e-9);
        assert_eq!(out.len(), target.len());
    }

    #[test]
    fn resample_onto_source_axis_is_identity() {
        use proptest::prelude::*;

        proptest!(ProptestConfig::with_cases(64), |(
            samples in proptest::collection::vec(any::<i32>(), 1..50)
        )| {
            let time_us: Vec<u64> = (0..samples.len()).map(|i| i.az::<u64>() * 1000).collect();
            let values: Vec<f64> = samples.iter().map(|v| f64::from(*v)).collect();
            let src = ts(time_us.clone(), values.clone());
            for strat in [super::Strategy::Linear, super::Strategy::StepFill, super::Strategy::Nearest] {
                let out = super::resample(&src, &time_us, strat);
                prop_assert_eq!(out, values.clone(), "{:?}", strat);
            }
        });
    }

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
