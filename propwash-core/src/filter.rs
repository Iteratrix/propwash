/// Filters frame indices by index range and/or time range.
///
/// Returns the indices of frames that satisfy both the frame range and
/// time range constraints. When both are specified, a frame must pass
/// both filters to be included.
///
/// # Arguments
///
/// * `total_frames` — Total number of frames in the session.
/// * `time_data` — Per-frame timestamps in microseconds (as `f64`).
///   Pass an empty slice if time filtering is not needed.
/// * `frame_range` — Optional `(start, optional_end)` for frame index filtering.
///   When `None`, all frames are considered. `start` is inclusive, `end` is inclusive.
/// * `time_range_us` — Optional `(start_us, optional_end_us)` for time filtering.
///   When `None`, no time filtering is applied. `start_us` is inclusive, `end_us` is inclusive.
#[allow(clippy::cast_possible_truncation)]
pub fn filter_frame_indices(
    total_frames: usize,
    time_data: &[f64],
    frame_range: Option<(usize, Option<usize>)>,
    time_range_us: Option<(i64, Option<i64>)>,
) -> Vec<usize> {
    let (frame_start, frame_end) = frame_range.unwrap_or((0, None));
    let upper = match frame_end {
        Some(end) => total_frames.min(end + 1),
        None => total_frames,
    };

    (frame_start..upper)
        .filter(|&i| {
            let Some((t_start, t_end)) = time_range_us else {
                return true;
            };
            let t = time_data.get(i).copied().unwrap_or(0.0) as i64;
            if t < t_start {
                return false;
            }
            if let Some(end) = t_end {
                if t > end {
                    return false;
                }
            }
            true
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_filters_returns_all() {
        let indices = filter_frame_indices(5, &[], None, None);
        assert_eq!(indices, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn frame_range_only() {
        let indices = filter_frame_indices(10, &[], Some((2, Some(5))), None);
        assert_eq!(indices, vec![2, 3, 4, 5]);
    }

    #[test]
    fn frame_range_open_end() {
        let indices = filter_frame_indices(5, &[], Some((3, None)), None);
        assert_eq!(indices, vec![3, 4]);
    }

    #[test]
    fn time_range_only() {
        let times: Vec<f64> = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let indices = filter_frame_indices(5, &times, None, Some((200, Some(400))));
        assert_eq!(indices, vec![1, 2, 3]);
    }

    #[test]
    fn time_range_open_end() {
        let times: Vec<f64> = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let indices = filter_frame_indices(5, &times, None, Some((300, None)));
        assert_eq!(indices, vec![2, 3, 4]);
    }

    #[test]
    fn both_filters() {
        let times: Vec<f64> = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let indices = filter_frame_indices(5, &times, Some((1, Some(3))), Some((250, None)));
        assert_eq!(indices, vec![2, 3]);
    }

    #[test]
    fn empty_result() {
        let indices = filter_frame_indices(0, &[], None, None);
        assert!(indices.is_empty());
    }

    #[test]
    fn frame_range_beyond_total() {
        let indices = filter_frame_indices(3, &[], Some((0, Some(10))), None);
        assert_eq!(indices, vec![0, 1, 2]);
    }
}
