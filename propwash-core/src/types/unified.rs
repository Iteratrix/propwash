use std::collections::HashMap;
use std::fmt;

use super::raw::{BfFrame, BfRawSession, RawSession};

/// Format-agnostic unified view over any session.
/// Provides sensor data in canonical units regardless of source format.
/// This is the recommended API for most consumers.
pub struct UnifiedView<'a> {
    raw: &'a RawSession,
}

impl fmt::Debug for UnifiedView<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UnifiedView").finish_non_exhaustive()
    }
}

impl<'a> UnifiedView<'a> {
    pub(super) fn new(raw: &'a RawSession) -> Self {
        Self { raw }
    }

    /// Returns the number of main frames.
    pub fn frame_count(&self) -> usize {
        self.bf_parts().1.len()
    }

    /// Returns field names from header definitions.
    pub fn field_names(&self) -> Vec<&str> {
        match self.raw {
            RawSession::Betaflight(bf) => bf.main_field_defs.names(),
        }
    }

    /// Returns the firmware version string.
    pub fn firmware_version(&self) -> &str {
        match self.raw {
            RawSession::Betaflight(bf) => &bf.firmware_version,
        }
    }

    /// Returns the craft name.
    pub fn craft_name(&self) -> &str {
        match self.raw {
            RawSession::Betaflight(bf) => &bf.craft_name,
        }
    }

    /// Computes sample rate from first/last frame timestamps.
    pub fn sample_rate_hz(&self) -> f64 {
        let (session, frames) = self.bf_parts();
        if frames.len() < 2 {
            return 0.0;
        }
        let Some(time_idx) = session.main_field_defs.index_of("time") else {
            return 0.0;
        };
        let t0 = frames
            .first()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let tn = frames
            .last()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let dt_us = tn - t0;
        if dt_us <= 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)]
        let rate = (frames.len() - 1) as f64 / (dt_us as f64 / 1_000_000.0);
        rate
    }

    /// Returns flight duration in seconds.
    pub fn duration_seconds(&self) -> f64 {
        let (session, frames) = self.bf_parts();
        if frames.len() < 2 {
            return 0.0;
        }
        let Some(time_idx) = session.main_field_defs.index_of("time") else {
            return 0.0;
        };
        let t0 = frames
            .first()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let tn = frames
            .last()
            .and_then(|f| f.values.get(time_idx).copied())
            .unwrap_or(0);
        let dt_us = tn - t0;
        if dt_us <= 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)]
        let dur = dt_us as f64 / 1_000_000.0;
        dur
    }

    /// Extracts one field as a `Vec<i64>` across all main frames.
    pub fn field(&self, name: &str) -> Vec<i64> {
        let (session, frames) = self.bf_parts();
        let Some(idx) = session.main_field_defs.index_of(name) else {
            return vec![0; frames.len()];
        };
        frames
            .iter()
            .map(|f| f.values.get(idx).copied().unwrap_or(0))
            .collect()
    }

    /// Extracts all fields matching a name prefix.
    pub fn fields(&self, prefix: &str) -> HashMap<String, Vec<i64>> {
        let (session, frames) = self.bf_parts();
        session
            .main_field_defs
            .fields
            .iter()
            .enumerate()
            .filter(|(_, f)| f.name.starts_with(prefix))
            .map(|(idx, f)| {
                let values: Vec<i64> = frames
                    .iter()
                    .map(|frame| frame.values.get(idx).copied().unwrap_or(0))
                    .collect();
                (f.name.clone(), values)
            })
            .collect()
    }

    /// Returns the number of motors detected.
    pub fn motor_count(&self) -> usize {
        self.field_names()
            .iter()
            .filter(|n| n.starts_with("motor["))
            .count()
    }

    fn bf_parts(&self) -> (&BfRawSession, &[BfFrame]) {
        match self.raw {
            RawSession::Betaflight(bf) => (bf, &bf.frames),
        }
    }
}
