use std::fmt;

use super::raw::{BfParseStats, BfRawSession, RawSession};

/// Format-specific analyzed view.
#[non_exhaustive]
pub enum Analyzed<'a> {
    Betaflight(BfAnalyzedView<'a>),
}

/// Betaflight-specific analyzed view. Borrows raw data.
pub struct BfAnalyzedView<'a> {
    pub raw: &'a BfRawSession,
}

impl fmt::Debug for BfAnalyzedView<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BfAnalyzedView").finish_non_exhaustive()
    }
}

impl BfAnalyzedView<'_> {
    /// Returns the number of motors detected from field definitions.
    pub fn motor_count(&self) -> usize {
        self.raw
            .main_field_defs
            .fields
            .iter()
            .filter(|f| f.name.starts_with("motor["))
            .count()
    }

    /// Returns whether bidirectional `DShot` RPM telemetry is present.
    pub fn has_rpm_telemetry(&self) -> bool {
        self.raw
            .main_field_defs
            .fields
            .iter()
            .any(|f| f.name.starts_with("eRPM["))
    }

    /// Returns whether unfiltered gyro data is logged.
    pub fn has_gyro_unfiltered(&self) -> bool {
        self.raw
            .main_field_defs
            .fields
            .iter()
            .any(|f| f.name.starts_with("gyroUnfilt["))
    }

    /// Returns the debug mode from headers (determines what `debug[0-3]` fields mean).
    pub fn debug_mode(&self) -> i32 {
        self.raw.get_header_int("debug_mode", 0)
    }

    /// Returns whether the log ended cleanly (vs truncation/crash).
    pub fn is_truncated(&self) -> bool {
        !self.raw.stats.clean_end
    }

    /// Returns the parse statistics.
    pub fn stats(&self) -> &BfParseStats {
        &self.raw.stats
    }
}

pub(super) fn analyzed_from_raw(raw: &RawSession) -> Analyzed<'_> {
    match raw {
        RawSession::Betaflight(bf) => Analyzed::Betaflight(BfAnalyzedView { raw: bf }),
    }
}
