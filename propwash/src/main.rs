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
}

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Command::Info { log_file, json } => cmd_info(&log_file, json),
        Command::Analyze { log_file } => cmd_analyze(&log_file),
    }
}

fn cmd_info(path: &str, json: bool) {
    let log = match propwash_core::decode_file(path) {
        Ok(log) => log,
        Err(e) => {
            eprintln!("Error: {e}");
            process::exit(1);
        }
    };

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
    let log = match propwash_core::decode_file(path) {
        Ok(log) => log,
        Err(e) => {
            eprintln!("Error: {e}");
            process::exit(1);
        }
    };

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

// ── JSON output types ───────────────────────────────────────────────

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
