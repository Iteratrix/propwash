use std::process;

use clap::{Parser, Subcommand};
use propwash_core::Analyzed;
use serde::Serialize;

#[derive(Parser)]
#[command(name = "propwash", about = "Flight log vibration analyzer")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Show metadata and field inventory for a blackbox log.
    Info {
        /// Path to a .bbl or .bfl log file.
        log_file: String,
        /// Output as JSON.
        #[arg(long)]
        json: bool,
    },
    /// Analyze vibration characteristics from a blackbox log.
    Analyze {
        /// Path to a .bbl or .bfl log file.
        log_file: String,
    },
    /// Dump raw frame data for programmatic consumption.
    Dump {
        /// Path to a .bbl or .bfl log file.
        log_file: String,
        /// Session index (1-based, default: all).
        #[arg(long)]
        session: Option<usize>,
        /// Frame range (e.g., "100-200").
        #[arg(long)]
        frames: Option<String>,
        /// Time range in seconds (e.g., "3.4-3.8").
        #[arg(long)]
        time: Option<String>,
        /// Comma-separated field name prefixes (e.g., "gyroADC,motor").
        #[arg(long)]
        fields: Option<String>,
    },
}

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Command::Info { log_file, json } => cmd_info(&log_file, json),
        Command::Analyze { log_file } => cmd_analyze(&log_file),
        Command::Dump {
            log_file,
            session,
            frames,
            time,
            fields,
        } => cmd_dump(
            &log_file,
            session,
            frames.as_deref(),
            time.as_deref(),
            fields.as_deref(),
        ),
    }
}

fn load_log(path: &str) -> propwash_core::Log {
    match propwash_core::decode_file(path) {
        Ok(log) => log,
        Err(e) => {
            eprintln!("Error: {e}");
            process::exit(1);
        }
    }
}

fn cmd_info(path: &str, json: bool) {
    let log = load_log(path);

    if json {
        let output = build_info_json(&log);
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        return;
    }

    println!("File:        {path}");
    println!("Sessions:    {}", log.session_count());
    println!();

    for session in &log.sessions {
        let unified = session.unified();
        println!("── Session {} ──", session.index);
        println!("  Firmware:       {}", unified.firmware_version());
        println!("  Craft:          {}", unified.craft_name());
        println!("  Duration:       {:.1}s", unified.duration_seconds());
        println!("  Sample rate:    {:.1} Hz", unified.sample_rate_hz());
        println!("  Frames:         {}", unified.frame_count());
        println!("  Motors:         {}", unified.motor_count());

        if let Analyzed::Betaflight(bf) = session.analyzed() {
            println!(
                "  RPM telemetry:  {}",
                if bf.has_rpm_telemetry() { "yes" } else { "no" }
            );
            println!(
                "  Gyro unfilt:    {}",
                if bf.has_gyro_unfiltered() {
                    "yes"
                } else {
                    "no"
                }
            );
            println!(
                "  Truncated:      {}",
                if bf.is_truncated() { "yes" } else { "no" }
            );
            if bf.stats().corrupt_bytes > 0 {
                println!("  Corrupt bytes:  {}", bf.stats().corrupt_bytes);
            }
        }

        let names = unified.field_names();
        println!("  Fields ({}):", names.len());
        for name in &names {
            println!("    {name}");
        }

        if !session.warnings.is_empty() {
            println!("  Warnings:");
            for w in &session.warnings {
                println!("    {w}");
            }
        }
        println!();
    }

    if !log.warnings.is_empty() {
        println!("Global warnings:");
        for w in &log.warnings {
            println!("  {w}");
        }
    }
}

fn cmd_analyze(path: &str) {
    let log = load_log(path);

    for session in &log.sessions {
        let unified = session.unified();
        if unified.frame_count() == 0 {
            continue;
        }

        let gyro_roll = unified.field("gyroADC[0]");
        let gyro_pitch = unified.field("gyroADC[1]");
        let gyro_yaw = unified.field("gyroADC[2]");

        let result = AnalyzeResult {
            session: session.index,
            firmware: unified.firmware_version().to_string(),
            sample_rate_hz: unified.sample_rate_hz(),
            duration_seconds: unified.duration_seconds(),
            frames: unified.frame_count(),
            motors: unified.motor_count(),
            gyro_roll_std: std_dev(&gyro_roll),
            gyro_pitch_std: std_dev(&gyro_pitch),
            gyro_yaw_std: std_dev(&gyro_yaw),
        };

        println!("{}", serde_json::to_string_pretty(&result).unwrap());
    }
}

fn cmd_dump(
    path: &str,
    session_filter: Option<usize>,
    frame_range: Option<&str>,
    time_range: Option<&str>,
    field_filter: Option<&str>,
) {
    let log = load_log(path);

    let field_prefixes: Vec<&str> = match field_filter {
        Some(f) => f.split(',').map(str::trim).collect(),
        None => Vec::new(),
    };

    let (frame_start, frame_end) = parse_range(frame_range);
    let (time_start_us, time_end_us) = parse_time_range(time_range);

    let mut output = DumpOutput {
        sessions: Vec::new(),
    };

    for session in &log.sessions {
        if let Some(target) = session_filter {
            if session.index != target {
                continue;
            }
        }

        let propwash_core::RawSession::Betaflight(bf) = &session.raw else {
            continue;
        };

        let field_names: Vec<&str> = bf.main_field_defs.names();
        let time_idx = bf.main_field_defs.index_of("time");

        let selected_fields: Vec<(usize, &str)> = field_names
            .iter()
            .enumerate()
            .filter(|(_, name)| {
                if field_prefixes.is_empty() {
                    return true;
                }
                field_prefixes.iter().any(|prefix| name.starts_with(prefix))
            })
            .map(|(i, name)| (i, *name))
            .collect();

        let mut frames = Vec::new();
        for frame in &bf.frames {
            if frame.frame_index < frame_start {
                continue;
            }
            if let Some(end) = frame_end {
                if frame.frame_index > end {
                    break;
                }
            }

            if let (Some(t_idx), Some(t_start)) = (time_idx, time_start_us) {
                let t = frame.values.get(t_idx).copied().unwrap_or(0);
                if t < t_start {
                    continue;
                }
                if let Some(t_end) = time_end_us {
                    if t > t_end {
                        break;
                    }
                }
            }

            let mut field_values = serde_json::Map::new();
            for &(idx, name) in &selected_fields {
                let val = frame.values.get(idx).copied().unwrap_or(0);
                field_values.insert(
                    name.to_string(),
                    serde_json::Value::Number(serde_json::Number::from(val)),
                );
            }

            frames.push(DumpFrame {
                index: frame.frame_index,
                byte_offset: frame.byte_offset,
                kind: match frame.kind {
                    propwash_core::format::bf::types::BfFrameKind::Intra => "I",
                    propwash_core::format::bf::types::BfFrameKind::Inter => "P",
                },
                values: field_values,
            });
        }

        output.sessions.push(DumpSession {
            index: session.index,
            firmware: bf.firmware_version.clone(),
            total_frames: bf.frames.len(),
            dumped_frames: frames.len(),
            fields: selected_fields
                .iter()
                .map(|(_, n)| (*n).to_string())
                .collect(),
            frames,
        });
    }

    println!("{}", serde_json::to_string_pretty(&output).unwrap());
}

fn parse_range(s: Option<&str>) -> (usize, Option<usize>) {
    let Some(s) = s else {
        return (0, None);
    };
    let Some((start, end)) = s.split_once('-') else {
        let n = s.parse().unwrap_or(0);
        return (n, Some(n));
    };
    let start = start.parse().unwrap_or(0);
    let end = end.parse().ok();
    (start, end)
}

fn parse_time_range(s: Option<&str>) -> (Option<i64>, Option<i64>) {
    let Some(s) = s else {
        return (None, None);
    };
    let s = s.trim_end_matches('s');
    let Some((start, end)) = s.split_once('-') else {
        return (None, None);
    };
    let to_us = |v: &str| -> Option<i64> {
        let v = v.trim_end_matches('s');
        #[allow(clippy::cast_possible_truncation)]
        v.parse::<f64>().ok().map(|f| (f * 1_000_000.0) as i64)
    };
    (to_us(start), to_us(end))
}

#[allow(clippy::cast_precision_loss)]
fn std_dev(values: &[i64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let n = values.len() as f64;
    let mean = values.iter().sum::<i64>() as f64 / n;
    let variance = values
        .iter()
        .map(|&v| (v as f64 - mean).powi(2))
        .sum::<f64>()
        / n;
    variance.sqrt()
}

#[derive(Serialize)]
struct AnalyzeResult {
    session: usize,
    firmware: String,
    sample_rate_hz: f64,
    duration_seconds: f64,
    frames: usize,
    motors: usize,
    gyro_roll_std: f64,
    gyro_pitch_std: f64,
    gyro_yaw_std: f64,
}

#[derive(Serialize)]
struct InfoJson {
    sessions: Vec<SessionInfo>,
}

#[derive(Serialize)]
struct SessionInfo {
    index: usize,
    firmware_version: String,
    craft_name: String,
    duration_seconds: f64,
    sample_rate_hz: f64,
    frame_count: usize,
    motor_count: usize,
    field_names: Vec<String>,
    warnings: Vec<String>,
}

#[derive(Serialize)]
struct DumpOutput {
    sessions: Vec<DumpSession>,
}

#[derive(Serialize)]
struct DumpSession {
    index: usize,
    firmware: String,
    total_frames: usize,
    dumped_frames: usize,
    fields: Vec<String>,
    frames: Vec<DumpFrame>,
}

#[derive(Serialize)]
struct DumpFrame {
    index: usize,
    byte_offset: usize,
    kind: &'static str,
    values: serde_json::Map<String, serde_json::Value>,
}

fn build_info_json(log: &propwash_core::Log) -> InfoJson {
    let sessions = log
        .sessions
        .iter()
        .map(|s| {
            let unified = s.unified();
            SessionInfo {
                index: s.index,
                firmware_version: unified.firmware_version().to_string(),
                craft_name: unified.craft_name().to_string(),
                duration_seconds: unified.duration_seconds(),
                sample_rate_hz: unified.sample_rate_hz(),
                frame_count: unified.frame_count(),
                motor_count: unified.motor_count(),
                field_names: unified
                    .field_names()
                    .iter()
                    .map(|s| (*s).to_string())
                    .collect(),
                warnings: s.warnings.iter().map(ToString::to_string).collect(),
            }
        })
        .collect();
    InfoJson { sessions }
}
