# Test Data

Large flight log files for local performance benchmarking. These are **not** committed to git (gitignored) — they're too large (1.2GB total).

The smaller curated fixtures used by CI live in `propwash-core/tests/fixtures/`.

## Populating

Download representative large files into subdirectories by format:

```
test-data/
├── ardupilot/    # .bin DataFlash logs (10-250MB)
├── betaflight/   # .bbl/.bfl blackbox logs (5-30MB)
└── px4/          # .ulg ULog files (10-120MB)
```

### Sources

- **PX4**: [PX4 Flight Review](https://review.px4.io/browse) — download via `https://logs.px4.io/download?log=LOG_ID`
- **ArduPilot**: Community logs shared on [discuss.ardupilot.org](https://discuss.ardupilot.org) (search crash analysis threads for Drive/Dropbox links)
- **Betaflight**: No centralized database — source from GitHub repos like [PyFlightCoach](https://github.com/PyFlightCoach) or [pichim's tune repos](https://github.com/pichim)

## Running benchmarks

```bash
cargo run --release --example bench -- test-data/px4/some_large_file.ulg
```

Or use the automated perf regression tests (which use the smaller CI fixtures):

```bash
cargo test --release -p propwash-core --test perf -- --ignored
```
