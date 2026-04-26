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
