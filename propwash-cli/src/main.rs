use std::process;

use clap::{Parser, Subcommand};
use propwash_core::analysis::episodes;
use propwash_core::types::SensorField;
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
    /// Analyze flight events and vibration characteristics.
    Analyze {
        /// Path to a .bbl or .bfl log file.
        log_file: String,
        /// Output format.
        #[arg(long, default_value = "summary")]
        output: String,
    },
    /// Compare two log files side by side.
    Compare {
        /// First log file (before).
        log_a: String,
        /// Second log file (after).
        log_b: String,
    },
    /// Scan multiple log files and surface the worst diagnostics.
    Scan {
        /// Log files to scan.
        #[arg(required = true)]
        files: Vec<String>,
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
        Command::Analyze { log_file, output } => cmd_analyze(&log_file, &output),
        Command::Compare { log_a, log_b } => cmd_compare(&log_a, &log_b),
        Command::Scan { files } => cmd_scan(&files),
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
        println!("── Session {} ──", session.index());
        println!("  Firmware:       {}", session.firmware_version());
        println!("  Craft:          {}", session.craft_name());
        println!("  Duration:       {:.1}s", session.duration_seconds());
        println!("  Sample rate:    {:.1} Hz", session.sample_rate_hz());
        println!("  Frames:         {}", session.frame_count());
        println!("  Motors:         {}", session.motor_count());

        println!(
            "  RPM telemetry:  {}",
            if session
                .field(&propwash_core::types::SensorField::ERpm(
                    propwash_core::types::MotorIndex(0),
                ))
                .is_empty()
            {
                "no"
            } else {
                "yes"
            }
        );
        println!(
            "  Gyro unfilt:    {}",
            if session
                .field(&propwash_core::types::SensorField::GyroUnfilt(
                    propwash_core::types::Axis::Roll,
                ))
                .is_empty()
            {
                "no"
            } else {
                "yes"
            }
        );
        println!(
            "  Truncated:      {}",
            if session.is_truncated() { "yes" } else { "no" }
        );
        if session.corrupt_bytes() > 0 {
            println!("  Corrupt bytes:  {}", session.corrupt_bytes());
        }

        let names = session.field_names();
        println!("  Fields ({}):", names.len());
        for name in &names {
            println!("    {name}");
        }

        if !session.warnings().is_empty() {
            println!("  Warnings:");
            for w in session.warnings() {
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

#[allow(clippy::too_many_lines)]
fn cmd_compare(path_a: &str, path_b: &str) {
    let log_a = load_log(path_a);
    let log_b = load_log(path_b);

    let session_a = log_a.sessions.iter().find(|s| s.frame_count() > 0);
    let session_b = log_b.sessions.iter().find(|s| s.frame_count() > 0);

    let (Some(sa), Some(sb)) = (session_a, session_b) else {
        eprintln!("Both files must contain at least one session with frames.");
        process::exit(1);
    };

    let aa = propwash_core::analysis::analyze(sa);
    let ab = propwash_core::analysis::analyze(sb);
    let ea = episodes::consolidate(&aa.events);
    let eb = episodes::consolidate(&ab.events);

    println!("                        {path_a:>20}  {path_b:>20}");
    println!("  ─────────────────────────────────────────────────────────────");

    compare_line(
        "Duration",
        &format!("{:.1}s", aa.summary.duration_seconds),
        &format!("{:.1}s", ab.summary.duration_seconds),
    );
    compare_line(
        "Sample rate",
        &format!("{:.0} Hz", aa.summary.sample_rate_hz),
        &format!("{:.0} Hz", ab.summary.sample_rate_hz),
    );
    compare_line(
        "Frames",
        &format!("{}", aa.summary.frame_count),
        &format!("{}", ab.summary.frame_count),
    );
    compare_line(
        "Episodes",
        &format!("{}", ea.len()),
        &format!("{}", eb.len()),
    );
    println!();

    compare_val(
        "Throttle chops",
        aa.summary.throttle_chops,
        ab.summary.throttle_chops,
    );
    compare_val(
        "Motor saturations",
        aa.summary.motor_saturations,
        ab.summary.motor_saturations,
    );
    compare_val(
        "Gyro spikes",
        aa.summary.gyro_spikes,
        ab.summary.gyro_spikes,
    );
    compare_val("Overshoots", aa.summary.overshoots, ab.summary.overshoots);
    compare_val("Desyncs", aa.summary.desyncs, ab.summary.desyncs);
    println!();

    if let (Some(va), Some(vb)) = (&aa.vibration, &ab.vibration) {
        println!("  Noise floor (dB):");
        let axes = ["roll", "pitch", "yaw"];
        for (i, axis) in axes.iter().enumerate() {
            let da = va.noise_floor_db[i];
            let db_val = vb.noise_floor_db[i];
            let diff = db_val - da;
            let arrow = if diff < -1.0 {
                " ▼ improved"
            } else if diff > 1.0 {
                " ▲ worse"
            } else {
                ""
            };
            println!("    {axis:6} {da:>20.1}  {db_val:>20.1}{arrow}");
        }
        println!();

        println!("  Dominant frequency:");
        for (i, axis) in axes.iter().enumerate() {
            let freq_a = va
                .spectra
                .get(i)
                .and_then(|s| s.peaks.first())
                .map(|p| p.frequency_hz);
            let freq_b = vb
                .spectra
                .get(i)
                .and_then(|s| s.peaks.first())
                .map(|p| p.frequency_hz);
            let fa = freq_a.map_or_else(|| "—".into(), |f| format!("{f:.0} Hz"));
            let fb = freq_b.map_or_else(|| "—".into(), |f| format!("{f:.0} Hz"));
            println!("    {axis:6} {fa:>20}  {fb:>20}");
        }
    }
    println!();

    println!("  Diagnostics A:");
    if aa.diagnostics.is_empty() {
        println!("    None");
    }
    for d in &aa.diagnostics {
        println!("    [{}] {}", d.category, d.message);
    }

    println!("  Diagnostics B:");
    if ab.diagnostics.is_empty() {
        println!("    None");
    }
    for d in &ab.diagnostics {
        println!("    [{}] {}", d.category, d.message);
    }
    println!();
}

fn compare_line(label: &str, a: &str, b: &str) {
    println!("  {label:20} {a:>20}  {b:>20}");
}

fn compare_val(label: &str, a: usize, b: usize) {
    #[allow(clippy::cast_possible_wrap)]
    let diff = b as i64 - a as i64;
    let arrow = match diff.cmp(&0) {
        std::cmp::Ordering::Less => format!(" (▼{diff})"),
        std::cmp::Ordering::Greater => format!(" (▲+{diff})"),
        std::cmp::Ordering::Equal => String::new(),
    };
    println!("  {label:20} {a:>20}  {b:>20}{arrow}");
}

fn cmd_scan(files: &[String]) {
    for path in files {
        let log = match propwash_core::decode_file(path) {
            Ok(log) => log,
            Err(e) => {
                println!("ERR  {path}: {e}");
                continue;
            }
        };

        for session in &log.sessions {
            if session.frame_count() == 0 {
                continue;
            }

            let analysis = propwash_core::analysis::analyze(session);
            let episodes = episodes::consolidate(&analysis.events);

            let worst = analysis.diagnostics.first().map_or_else(
                || "OK".into(),
                |d| {
                    let icon = match d.severity {
                        propwash_core::analysis::diagnostics::Severity::Problem => "!!",
                        propwash_core::analysis::diagnostics::Severity::Warning => " !",
                        propwash_core::analysis::diagnostics::Severity::Info => "  ",
                    };
                    format!("{icon} {}", d.message)
                },
            );

            println!(
                "{worst:60}  {path} s{} ({:.0}s, {} events)",
                session.index(),
                session.duration_seconds(),
                episodes.len(),
            );
        }
    }
}

fn cmd_analyze(path: &str, output: &str) {
    let log = load_log(path);

    for session in &log.sessions {
        if session.frame_count() == 0 {
            continue;
        }

        let analysis = propwash_core::analysis::analyze(session);
        let episodes = episodes::consolidate(&analysis.events);

        match output {
            "json" => {
                #[derive(Serialize)]
                struct JsonOutput<'a> {
                    summary: &'a propwash_core::analysis::summary::FlightSummary,
                    episodes: &'a [episodes::Episode],
                    vibration: Option<&'a propwash_core::analysis::fft::VibrationAnalysis>,
                    diagnostics: &'a [propwash_core::analysis::diagnostics::Diagnostic],
                }
                let out = JsonOutput {
                    summary: &analysis.summary,
                    episodes: &episodes,
                    vibration: analysis.vibration.as_ref(),
                    diagnostics: &analysis.diagnostics,
                };
                println!("{}", serde_json::to_string_pretty(&out).unwrap());
            }
            "summary" => {
                print_summary(
                    &analysis.summary,
                    &episodes,
                    analysis.vibration.as_ref(),
                    &analysis.diagnostics,
                );
            }
            _ => {
                eprintln!("Unknown output format: {output}. Use 'json' or 'summary'.");
                process::exit(1);
            }
        }
    }
}

#[allow(clippy::too_many_lines)]
fn print_summary(
    s: &propwash_core::analysis::summary::FlightSummary,
    episodes: &[episodes::Episode],
    vibration: Option<&propwash_core::analysis::fft::VibrationAnalysis>,
    diagnostics: &[propwash_core::analysis::diagnostics::Diagnostic],
) {
    println!("── Session {} ──", s.session_index);
    println!("  Firmware:    {}", s.firmware);
    if !s.craft.is_empty() {
        println!("  Craft:       {}", s.craft);
    }
    println!("  Duration:    {:.1}s", s.duration_seconds);
    println!("  Sample rate: {:.0} Hz", s.sample_rate_hz);
    println!("  Frames:      {}", s.frame_count);
    println!("  Motors:      {}", s.motor_count);
    println!();

    if episodes.is_empty() {
        println!("  No events detected.");
    } else {
        println!("  Timeline ({} episodes):", episodes.len());
        for ep in episodes {
            let time = format!("{:>8.3}s", ep.start_time);
            let dur = if ep.duration_seconds > 0.001 {
                format!(" ({:.0}ms)", ep.duration_seconds * 1000.0)
            } else {
                String::new()
            };
            match &ep.kind {
                episodes::EpisodeKind::ThrottleChop {
                    from_percent,
                    to_percent,
                } => {
                    println!("    {time}  THROTTLE CHOP  {from_percent:.0}% → {to_percent:.0}%");
                }
                episodes::EpisodeKind::ThrottlePunch {
                    from_percent,
                    to_percent,
                } => {
                    println!("    {time}  THROTTLE PUNCH {from_percent:.0}% → {to_percent:.0}%");
                }
                episodes::EpisodeKind::MotorSaturation {
                    motor_index,
                    duration_frames,
                } => {
                    println!(
                        "    {time}  MOTOR SAT      motor[{motor_index}] for {duration_frames} frames"
                    );
                }
                episodes::EpisodeKind::GyroSpikes {
                    axis,
                    peak_magnitude,
                    count,
                } => {
                    println!(
                        "    {time}  GYRO SPIKE     {axis} peak {peak_magnitude:.0}°/s ({count} frames){dur}"
                    );
                }
                episodes::EpisodeKind::Overshoot {
                    axis,
                    peak_overshoot_percent,
                    count,
                } => {
                    println!(
                        "    {time}  OVERSHOOT      {axis} peak {peak_overshoot_percent:.0}% ({count} frames){dur}"
                    );
                }
                episodes::EpisodeKind::Desync { motor_index, count } => {
                    println!(
                        "    {time}  DESYNC         motor[{motor_index}] ({count} frames){dur}"
                    );
                }
            }
        }
    }

    if let Some(vib) = &vibration {
        println!();
        println!("  Vibration Analysis (full flight):");
        print_spectra(&vib.spectra, &vib.noise_floor_db);

        if let Some(accel) = &vib.accel {
            println!();
            println!("  FC Board Vibration (accelerometer):");
            println!(
                "    RMS: X={:.1}  Y={:.1}  Z={:.1}",
                accel.rms[0], accel.rms[1], accel.rms[2]
            );
            for spectrum in &accel.spectra {
                if let Some(peak) = spectrum.peaks.first() {
                    println!(
                        "    {}: dominant {:.0} Hz at {:.1} dB",
                        spectrum.axis, peak.frequency_hz, peak.magnitude_db
                    );
                }
            }
        }

        if !vib.throttle_bands.is_empty() {
            println!();
            println!("  Vibration by Throttle:");
            for band in &vib.throttle_bands {
                println!("    {} ({} frames):", band.label, band.frame_count);
                for spectrum in &band.spectra {
                    if let Some(peak) = spectrum.peaks.first() {
                        println!(
                            "      {}: dominant {:.0} Hz at {:.1} dB",
                            spectrum.axis, peak.frequency_hz, peak.magnitude_db
                        );
                    }
                }
            }
        }

        if let Some(pw) = &vib.propwash {
            println!();
            println!("  Propwash Analysis ({} chop windows):", pw.chop_count);
            for spectrum in &pw.spectra {
                if let Some(peak) = spectrum.peaks.first() {
                    println!(
                        "    {}: dominant {:.0} Hz at {:.1} dB",
                        spectrum.axis, peak.frequency_hz, peak.magnitude_db
                    );
                }
            }
            if let (Some(freq), Some(mag)) = (pw.dominant_frequency_hz, pw.dominant_magnitude_db) {
                println!("    Propwash frequency: {freq:.0} Hz ({mag:.1} dB)");
            } else {
                println!("    No dominant propwash frequency detected (20-100 Hz range)");
            }
        }
    }

    println!();
    println!("  Summary (raw event counts):");
    println!("    Throttle chops:    {}", s.throttle_chops);
    println!("    Throttle punches:  {}", s.throttle_punches);
    println!("    Motor saturations: {}", s.motor_saturations);
    println!("    Gyro spikes:       {}", s.gyro_spikes);
    println!("    Overshoots:        {}", s.overshoots);
    println!("    Desyncs:           {}", s.desyncs);

    if !diagnostics.is_empty() {
        println!();
        println!("  Diagnostics:");
        for diag in diagnostics {
            let icon = match diag.severity {
                propwash_core::analysis::diagnostics::Severity::Info => "   ",
                propwash_core::analysis::diagnostics::Severity::Warning => " ! ",
                propwash_core::analysis::diagnostics::Severity::Problem => "!! ",
            };
            println!("    {icon} [{:}] {}", diag.category, diag.message);
            println!("         {}", diag.detail);
        }
    }
    println!();
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
            if session.index() != target {
                continue;
            }
        }

        let field_names = session.field_names();

        let selected_fields: Vec<&str> = field_names
            .iter()
            .filter(|name| {
                if field_prefixes.is_empty() {
                    return true;
                }
                field_prefixes.iter().any(|prefix| name.starts_with(prefix))
            })
            .map(String::as_str)
            .collect();

        // Fetch all selected field data as columns
        let columns: Vec<Vec<f64>> = selected_fields
            .iter()
            .filter_map(|name| SensorField::parse(name).ok())
            .map(|field| session.field(&field))
            .collect();

        let n_frames = session.frame_count();
        let time_data = session.field(&SensorField::Time);

        let frame_range_arg = if frame_start == 0 && frame_end.is_none() {
            None
        } else {
            Some((frame_start, frame_end))
        };
        let time_range_arg = time_start_us.map(|t_start| (t_start, time_end_us));

        let indices = propwash_core::filter::filter_frame_indices(
            n_frames,
            &time_data,
            frame_range_arg,
            time_range_arg,
        );

        let frames: Vec<DumpFrame> = indices
            .into_iter()
            .map(|i| {
                let mut field_values = serde_json::Map::new();
                for (col_idx, &name) in selected_fields.iter().enumerate() {
                    let val = columns[col_idx].get(i).copied().unwrap_or(0.0);
                    let json_val = serde_json::Number::from_f64(val)
                        .map_or(serde_json::Value::Null, serde_json::Value::Number);
                    field_values.insert(name.to_string(), json_val);
                }
                DumpFrame {
                    index: i,
                    byte_offset: None,
                    kind: None,
                    values: field_values,
                }
            })
            .collect();

        output.sessions.push(DumpSession {
            index: session.index(),
            firmware: session.firmware_version().to_string(),
            total_frames: n_frames,
            dumped_frames: frames.len(),
            fields: selected_fields.iter().map(|s| (*s).to_string()).collect(),
            frames,
        });
    }

    println!("{}", serde_json::to_string_pretty(&output).unwrap());
}

fn print_spectra(
    spectra: &[propwash_core::analysis::fft::FrequencySpectrum],
    noise_floor: &[f64; 3],
) {
    for spectrum in spectra {
        let floor_idx = match spectrum.axis {
            "roll" => 0,
            "pitch" => 1,
            _ => 2,
        };
        println!(
            "    {} axis — noise floor {:.1} dB:",
            spectrum.axis, noise_floor[floor_idx]
        );
        if spectrum.peaks.is_empty() {
            println!("      No significant peaks");
        } else {
            for peak in &spectrum.peaks {
                let class = match &peak.classification {
                    Some(propwash_core::analysis::fft::NoiseClass::MotorNoise) => " (motor noise)",
                    Some(propwash_core::analysis::fft::NoiseClass::FrameResonance) => {
                        " (frame resonance)"
                    }
                    Some(propwash_core::analysis::fft::NoiseClass::Unknown) | None => "",
                };
                println!(
                    "      #{}: {:.0} Hz at {:.1} dB{class}",
                    peak.rank, peak.frequency_hz, peak.magnitude_db
                );
            }
        }
    }
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
    #[serde(skip_serializing_if = "Option::is_none")]
    byte_offset: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    kind: Option<&'static str>,
    values: serde_json::Map<String, serde_json::Value>,
}

fn build_info_json(log: &propwash_core::Log) -> InfoJson {
    let sessions = log
        .sessions
        .iter()
        .map(|s| SessionInfo {
            index: s.index(),
            firmware_version: s.firmware_version().to_string(),
            craft_name: s.craft_name().to_string(),
            duration_seconds: s.duration_seconds(),
            sample_rate_hz: s.sample_rate_hz(),
            frame_count: s.frame_count(),
            motor_count: s.motor_count(),
            field_names: s.field_names(),
            warnings: s.warnings().iter().map(ToString::to_string).collect(),
        })
        .collect();
    InfoJson { sessions }
}
