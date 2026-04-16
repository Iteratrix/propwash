use std::time::Instant;

use propwash_core::types::{Axis, SensorField};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: bench <file>");
        return;
    }
    let path = &args[1];
    let data = std::fs::read(path).unwrap();
    let size_mb = data.len() as f64 / 1_048_576.0;

    // Warm up
    let _ = propwash_core::decode(&data);

    // Benchmark parse only (3 runs, take best)
    let mut times = Vec::new();
    for _ in 0..3 {
        let start = Instant::now();
        let log = propwash_core::decode(&data);
        let elapsed = start.elapsed();
        std::hint::black_box(&log);
        times.push(elapsed);
    }
    let best = *times.iter().min().unwrap();
    let ms = best.as_secs_f64() * 1000.0;
    let mb_s = size_mb / best.as_secs_f64();

    // Parse once more to get stats
    let log = propwash_core::decode(&data).unwrap();
    let sessions = log.sessions.len();
    let frames: usize = log.sessions.iter().map(|s| s.frame_count()).sum();

    println!("{size_mb:.1}MB, {sessions} sessions, {frames} frames");
    println!(
        "Parse: {ms:.1}ms ({mb_s:.0} MB/s, {:.0} kframes/s)",
        frames as f64 / ms
    );

    // Benchmark field extraction
    let s = &log.sessions[0];
    let fields = [
        SensorField::Time,
        SensorField::Gyro(Axis::Roll),
        SensorField::Gyro(Axis::Pitch),
        SensorField::Gyro(Axis::Yaw),
    ];
    let start = Instant::now();
    for _ in 0..10 {
        for f in &fields {
            std::hint::black_box(s.field(f));
        }
    }
    let field_elapsed = start.elapsed();
    let field_ms = field_elapsed.as_secs_f64() * 1000.0 / 10.0;
    println!("Field extract (4 fields x10): {field_ms:.2}ms per iteration");
}
