use propwash_core::analysis::events::{EventKind, FlightEvent};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct Episode {
    pub start_time: f64,
    pub end_time: f64,
    pub duration_seconds: f64,
    pub event_count: usize,
    pub kind: EpisodeKind,
}

#[derive(Debug, Serialize)]
#[serde(tag = "type")]
pub enum EpisodeKind {
    ThrottleChop {
        from_percent: f64,
        to_percent: f64,
    },
    ThrottlePunch {
        from_percent: f64,
        to_percent: f64,
    },
    MotorSaturation {
        motor_index: usize,
        duration_frames: usize,
    },
    GyroSpikes {
        axis: &'static str,
        peak_magnitude: f64,
        count: usize,
    },
    Overshoot {
        axis: &'static str,
        peak_overshoot_percent: f64,
        count: usize,
    },
    Desync {
        motor_index: usize,
        count: usize,
    },
}

const EPISODE_GAP_SECONDS: f64 = 0.1;

pub fn consolidate(events: &[FlightEvent]) -> Vec<Episode> {
    let mut episodes = Vec::new();

    consolidate_throttle_chops(events, &mut episodes);
    consolidate_throttle_punches(events, &mut episodes);
    consolidate_motor_saturations(events, &mut episodes);
    consolidate_gyro_spikes(events, &mut episodes);
    consolidate_overshoots(events, &mut episodes);
    consolidate_desyncs(events, &mut episodes);

    episodes.sort_by(|a, b| a.start_time.partial_cmp(&b.start_time).unwrap());
    episodes
}

fn consolidate_throttle_chops(events: &[FlightEvent], episodes: &mut Vec<Episode>) {
    for event in events {
        if let EventKind::ThrottleChop {
            from_percent,
            to_percent,
            ..
        } = &event.kind
        {
            episodes.push(Episode {
                start_time: event.time_seconds,
                end_time: event.time_seconds,
                duration_seconds: 0.0,
                event_count: 1,
                kind: EpisodeKind::ThrottleChop {
                    from_percent: *from_percent,
                    to_percent: *to_percent,
                },
            });
        }
    }
}

fn consolidate_throttle_punches(events: &[FlightEvent], episodes: &mut Vec<Episode>) {
    for event in events {
        if let EventKind::ThrottlePunch {
            from_percent,
            to_percent,
            ..
        } = &event.kind
        {
            episodes.push(Episode {
                start_time: event.time_seconds,
                end_time: event.time_seconds,
                duration_seconds: 0.0,
                event_count: 1,
                kind: EpisodeKind::ThrottlePunch {
                    from_percent: *from_percent,
                    to_percent: *to_percent,
                },
            });
        }
    }
}

fn consolidate_motor_saturations(events: &[FlightEvent], episodes: &mut Vec<Episode>) {
    for event in events {
        if let EventKind::MotorSaturation {
            motor_index,
            duration_frames,
        } = &event.kind
        {
            episodes.push(Episode {
                start_time: event.time_seconds,
                end_time: event.time_seconds,
                duration_seconds: 0.0,
                event_count: 1,
                kind: EpisodeKind::MotorSaturation {
                    motor_index: *motor_index,
                    duration_frames: *duration_frames,
                },
            });
        }
    }
}

fn consolidate_gyro_spikes(events: &[FlightEvent], episodes: &mut Vec<Episode>) {
    let axes = ["roll", "pitch", "yaw"];
    for axis in &axes {
        let mut axis_events: Vec<&FlightEvent> = events
            .iter()
            .filter(|e| matches!(&e.kind, EventKind::GyroSpike { axis: a, .. } if a == axis))
            .collect();
        axis_events.sort_by(|a, b| a.time_seconds.partial_cmp(&b.time_seconds).unwrap());

        let mut i = 0;
        while i < axis_events.len() {
            let start = axis_events[i];
            let mut end = start;
            let mut peak = match &start.kind {
                EventKind::GyroSpike { magnitude, .. } => magnitude.abs(),
                _ => 0.0,
            };
            let mut count = 1;

            while i + 1 < axis_events.len()
                && axis_events[i + 1].time_seconds - end.time_seconds < EPISODE_GAP_SECONDS
            {
                i += 1;
                end = axis_events[i];
                count += 1;
                if let EventKind::GyroSpike { magnitude, .. } = &end.kind {
                    peak = peak.max(magnitude.abs());
                }
            }

            episodes.push(Episode {
                start_time: start.time_seconds,
                end_time: end.time_seconds,
                duration_seconds: end.time_seconds - start.time_seconds,
                event_count: count,
                kind: EpisodeKind::GyroSpikes {
                    axis,
                    peak_magnitude: peak,
                    count,
                },
            });
            i += 1;
        }
    }
}

fn consolidate_overshoots(events: &[FlightEvent], episodes: &mut Vec<Episode>) {
    let axes = ["roll", "pitch", "yaw"];
    for axis in &axes {
        let mut axis_events: Vec<&FlightEvent> = events
            .iter()
            .filter(|e| matches!(&e.kind, EventKind::Overshoot { axis: a, .. } if a == axis))
            .collect();
        axis_events.sort_by(|a, b| a.time_seconds.partial_cmp(&b.time_seconds).unwrap());

        let mut i = 0;
        while i < axis_events.len() {
            let start = axis_events[i];
            let mut end = start;
            let mut peak = match &start.kind {
                EventKind::Overshoot {
                    overshoot_percent, ..
                } => *overshoot_percent,
                _ => 0.0,
            };
            let mut count = 1;

            while i + 1 < axis_events.len()
                && axis_events[i + 1].time_seconds - end.time_seconds < EPISODE_GAP_SECONDS
            {
                i += 1;
                end = axis_events[i];
                count += 1;
                if let EventKind::Overshoot {
                    overshoot_percent, ..
                } = &end.kind
                {
                    peak = peak.max(*overshoot_percent);
                }
            }

            episodes.push(Episode {
                start_time: start.time_seconds,
                end_time: end.time_seconds,
                duration_seconds: end.time_seconds - start.time_seconds,
                event_count: count,
                kind: EpisodeKind::Overshoot {
                    axis,
                    peak_overshoot_percent: peak,
                    count,
                },
            });
            i += 1;
        }
    }
}

fn consolidate_desyncs(events: &[FlightEvent], episodes: &mut Vec<Episode>) {
    for motor_idx in 0..8 {
        let mut motor_events: Vec<&FlightEvent> = events
            .iter()
            .filter(|e| {
                matches!(&e.kind, EventKind::Desync { motor_index, .. } if *motor_index == motor_idx)
            })
            .collect();
        if motor_events.is_empty() {
            continue;
        }
        motor_events.sort_by(|a, b| a.time_seconds.partial_cmp(&b.time_seconds).unwrap());

        let mut i = 0;
        while i < motor_events.len() {
            let start = motor_events[i];
            let mut end = start;
            let mut count = 1;

            while i + 1 < motor_events.len()
                && motor_events[i + 1].time_seconds - end.time_seconds < EPISODE_GAP_SECONDS
            {
                i += 1;
                end = motor_events[i];
                count += 1;
            }

            episodes.push(Episode {
                start_time: start.time_seconds,
                end_time: end.time_seconds,
                duration_seconds: end.time_seconds - start.time_seconds,
                event_count: count,
                kind: EpisodeKind::Desync {
                    motor_index: motor_idx,
                    count,
                },
            });
            i += 1;
        }
    }
}
