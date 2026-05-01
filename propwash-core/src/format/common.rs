use std::collections::HashMap;

use az::SaturatingAs;

use crate::types::{AxisGains, FilterConfig, PidGains};

/// Columnar storage for one message type's data.
///
/// All column vectors have the same length as `timestamps`.
/// Shared across AP, PX4, and `MAVLink` format modules.
#[derive(Debug)]
pub struct MsgColumns {
    /// Timestamps in microseconds, one per message.
    pub timestamps: Vec<u64>,
    /// Parallel column vectors of decoded field values.
    pub columns: Vec<Vec<f64>>,
    /// Field names, parallel to `columns`. Useful for diagnostics; the build
    /// step looks up by `column(name)` which uses the index map below.
    #[allow(dead_code)]
    pub field_names: Vec<String>,
    /// name -> column index for O(1) field lookup.
    field_index: HashMap<String, usize>,
}

impl MsgColumns {
    pub(crate) fn new(field_names: Vec<String>) -> Self {
        let field_index = field_names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.clone(), i))
            .collect();
        let columns = vec![Vec::new(); field_names.len()];
        Self {
            timestamps: Vec::new(),
            columns,
            field_names,
            field_index,
        }
    }

    /// Returns the column for a given field name.
    pub fn column(&self, field: &str) -> Option<&[f64]> {
        let idx = self.field_index.get(field)?;
        Some(&self.columns[*idx])
    }

    /// Pushes one row of decoded values into the columns.
    #[inline]
    pub(crate) fn push_row(&mut self, timestamp: u64, values: &[f64]) {
        self.timestamps.push(timestamp);
        for (col, &val) in self.columns.iter_mut().zip(values.iter()) {
            col.push(val);
        }
    }
}

/// Identifies the autopilot family that produced a `MAVLink` tlog so the
/// build step can select the right parameter-name table for PID gains
/// and filter config. `PX4` and `ArduPilot` both speak `MAVLink` but use
/// disjoint parameter names (`MC_ROLLRATE_P` vs `ATC_RAT_RLL_P`).
///
/// Mirrors the relevant `MAV_AUTOPILOT_*` enum values from the `MAVLink`
/// `common.xml` schema.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AutopilotFamily {
    #[default]
    Generic,
    /// `MAV_AUTOPILOT_ARDUPILOTMEGA = 3`
    ArduPilot,
    /// `MAV_AUTOPILOT_PX4 = 12`
    Px4,
    Other(u8),
}

impl AutopilotFamily {
    pub fn from_id(id: u8) -> Self {
        match id {
            3 => Self::ArduPilot,
            12 => Self::Px4,
            0 => Self::Generic,
            other => Self::Other(other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::AutopilotFamily;

    #[test]
    fn autopilot_family_canonical_ids() {
        assert_eq!(AutopilotFamily::from_id(3), AutopilotFamily::ArduPilot);
        assert_eq!(AutopilotFamily::from_id(12), AutopilotFamily::Px4);
        assert_eq!(AutopilotFamily::from_id(0), AutopilotFamily::Generic);
        assert_eq!(AutopilotFamily::from_id(8), AutopilotFamily::Other(8));
    }
}

/// Extract `ArduPilot` PID gains from a parameter map.
///
/// Shared by AP `DataFlash` and `MAVLink` tlog sessions, which use the same
/// `ATC_RAT_*` parameter names.
#[allow(clippy::implicit_hasher)]
pub fn ardupilot_pid_gains(params: &HashMap<String, f64>) -> PidGains {
    let parse = |p_key: &str, i_key: &str, d_key: &str| -> AxisGains {
        let get = |k: &str| -> Option<u32> {
            params
                .get(k)
                .copied()
                .filter(|&v| v > 0.0)
                .map(|v| (v * 1000.0).saturating_as::<u32>())
        };
        AxisGains {
            p: get(p_key),
            i: get(i_key),
            d: get(d_key),
        }
    };
    PidGains::new(
        parse("ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D"),
        parse("ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D"),
        parse("ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D"),
    )
}

/// Extract `ArduPilot` filter configuration from a parameter map.
///
/// Shared by AP `DataFlash` and `MAVLink` tlog sessions, which use the same
/// `INS_*` and `ATC_*` parameter names.
#[allow(clippy::implicit_hasher)]
pub fn ardupilot_filter_config(params: &HashMap<String, f64>) -> FilterConfig {
    let p = |k: &str| params.get(k).copied().unwrap_or(0.0);
    let non_zero = |v: f64| if v > 0.0 { Some(v) } else { None };

    FilterConfig {
        gyro_lpf_hz: non_zero(p("INS_GYRO_FILTER")),
        gyro_lpf2_hz: None,
        dterm_lpf_hz: non_zero(p("ATC_RAT_RLL_FLTE")),
        dyn_notch_min_hz: if p("INS_HNTCH_ENABLE") > 0.0 {
            non_zero(p("INS_HNTCH_FREQ"))
        } else {
            None
        },
        dyn_notch_max_hz: None,
        gyro_notch1_hz: if p("INS_NOTCH_ENABLE") > 0.0 {
            non_zero(p("INS_NOTCH_FREQ"))
        } else {
            None
        },
        gyro_notch2_hz: if p("INS_NOTC2_ENABLE") > 0.0 {
            non_zero(p("INS_NOTC2_FREQ"))
        } else {
            None
        },
    }
}

/// Extract `PX4` PID gains from a parameter map.
///
/// Shared by `PX4` `ULog` sessions and PX4-sourced `MAVLink` tlogs, which both
/// use the `MC_*RATE_{P,I,D}` multicopter rate-controller naming.
#[allow(clippy::implicit_hasher)]
pub fn px4_pid_gains(params: &HashMap<String, f64>) -> PidGains {
    let parse = |p_key: &str, i_key: &str, d_key: &str| -> AxisGains {
        let get = |k: &str| -> Option<u32> {
            params
                .get(k)
                .copied()
                .filter(|&v| v > 0.0)
                .map(|v| (v * 1000.0).saturating_as::<u32>())
        };
        AxisGains {
            p: get(p_key),
            i: get(i_key),
            d: get(d_key),
        }
    };
    PidGains::new(
        parse("MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D"),
        parse("MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D"),
        parse("MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D"),
    )
}

/// Extract `PX4` filter configuration from a parameter map.
///
/// `PX4` tunes filters via `IMU_GYRO_*` / `IMU_DGYRO_*` parameters, distinct
/// from the `ArduPilot` `INS_*` set.
#[allow(clippy::implicit_hasher)]
pub fn px4_filter_config(params: &HashMap<String, f64>) -> FilterConfig {
    let p = |k: &str| params.get(k).copied().unwrap_or(0.0);
    let non_zero = |v: f64| if v > 0.0 { Some(v) } else { None };

    FilterConfig {
        gyro_lpf_hz: non_zero(p("IMU_GYRO_CUTOFF")),
        gyro_lpf2_hz: None,
        dterm_lpf_hz: non_zero(p("IMU_DGYRO_CUTOFF")),
        dyn_notch_min_hz: non_zero(p("IMU_GYRO_DNF_MIN")),
        // PX4 has no upper-bound parameter for the dynamic notch — its
        // peak detection is data-driven. `IMU_GYRO_DNF_HMC` is the
        // harmonic count, not a frequency. Leave None until PX4 ships
        // a real upper bound.
        dyn_notch_max_hz: None,
        gyro_notch1_hz: non_zero(p("IMU_GYRO_NF0_FRQ")),
        gyro_notch2_hz: non_zero(p("IMU_GYRO_NF1_FRQ")),
    }
}
