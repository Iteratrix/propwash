# Propwash Library Code Review

Reviewer notes collected during a walkthrough of the codebase.
Each item is tagged with a category and file location for later triage.

Categories:
- **[bug]** — Correctness issue
- **[perf]** — Performance concern
- **[style]** — Code style / readability
- **[arch]** — Architecture / design
- **[safety]** — Safety / robustness
- **[docs]** — Documentation gap
- **[test]** — Testing gap
- **[nit]** — Minor nitpick
- **[question]** — Needs clarification

---

## Ground Rules

- **No backwards compatibility constraints.** No external users yet. Breaking changes are fine.

---

## Comments

### 1. [arch] `compare_official.py` (root)
Dev-only validation script living at the repo root. Should be moved to `scripts/` or `tools/` to keep the root clean.

### 2. [arch] Parser implementations — use enums/newtypes for sealed option sets
When reviewing the format parsers (bf, ap, px4), verify that sealed sets of options (message types, encoding types, field types, etc.) are represented as enums/newtypes rather than raw integers or strings. Makes invalid states unrepresentable.

### 3. [style] `Cargo.toml:12` — pedantic = warn
Pedantic is set to `warn` locally but CI enforces `-D warnings` (ci.yml:20), so warnings are effectively errors in CI. This is a reasonable pattern — local dev stays ergonomic while CI is strict. No action needed unless we want local parity.

### 4. [nit] `Cargo.toml:19` — `lto = "thin"` on dist profile
Fine as-is. Thin LTO gives most perf/size benefits without the compile time cost of fat LTO. Only applies to distribution builds.

### 5. [arch] `web/app.js` — should be TypeScript
The web frontend is ~44KB of vanilla JS. Given WASM interop and UI complexity, TypeScript would add meaningful type safety, especially for enforcing the WASM bridge contract. Tradeoff: adds a build step to what is currently a zero-build frontend.

### 6. [nit] `propwash-web/src/lib.rs:12-14` — `thread_local!` + `RefCell` for WASM state
Canonical pattern for single-threaded WASM. No action needed.

### 7. [arch] `propwash-web/src/lib.rs` + `propwash/src/main.rs` — DTO duplication between web and CLI
Both crates define nearly identical session-info structs (`SessionResult` in web, `SessionInfo`/`DumpSession` in CLI) to serialize the same core data. Consider a shared serialization layer in core or a common DTO crate.

### 8. [style] `propwash-web/src/lib.rs:78-96` — for-loops building collections
Several loops push into Vecs where `.map().collect()` would be more idiomatic:
- Lines 78-91: session result construction
- Lines 94-96: warning string conversion
- Lines 384-390: raw frame row building

### 9. [arch] `propwash-web/src/lib.rs:167-272` — spectrogram computation belongs in core
Full FFT windowing/spectrogram generation is implemented in the WASM bridge. Core already has `analysis/fft.rs` for FFT work. This should live in core so CLI and other consumers can also generate spectrograms. Currently WASM-only.

### 10. [arch] `propwash-web/src/lib.rs:294-350` — filter config bypasses Unified trait
`get_filter_config` matches on `RawSession::Betaflight`/`ArduPilot`/`Px4` directly and uses format-specific header strings/param maps. Breaks the abstraction — every new format requires updating this function. Filter configuration should be a method on the `Unified` trait (e.g., `fn filter_config(&self) -> FilterConfig`). Same pattern seen in CLI at `main.rs:121-141` for Betaflight-specific info display.

### 11. [nit] `propwash/` directory — should be renamed `propwash-cli`
The binary crate directory is `propwash/` while sibling crates follow the `propwash-core`, `propwash-web` naming convention. Rename to `propwash-cli/` for consistency. Keep the binary name as `propwash` via `[[bin]] name = "propwash"` so the installed command doesn't change.

### 12. [arch] Both CLI and web should drop the `raw` feature and use only `Unified`
Both consumer crates enable `features = ["raw"]` and reach into format-specific types. This defeats the purpose of the `Unified` abstraction. To eliminate raw access, the `Unified` trait needs to grow:
- `fn filter_config(&self) -> FilterConfig` (covers web `get_filter_config` and is generally useful)
- `fn has_rpm_telemetry(&self) -> bool`
- `fn has_gyro_unfiltered(&self) -> bool`
- `fn is_truncated(&self) -> bool`
- `fn corrupt_bytes(&self) -> usize` (or a `fn parse_stats(&self) -> ParseStats`)
Consider grouping the metadata methods into a `Capabilities` or `ParseStats` struct to avoid trait bloat. Once done, remove `features = ["raw"]` from both `propwash/Cargo.toml` and `propwash-web/Cargo.toml`. Supersedes #10.

### 13. [arch] `propwash/src/episodes.rs` — episode consolidation belongs in core
The entire 263-line `episodes.rs` module is pure analysis logic: grouping temporally-adjacent `FlightEvent`s into higher-level episodes. It's format-independent and reusable — the web timeline view would benefit from it. Should move to `propwash-core/src/analysis/episodes.rs`. Additionally, `consolidate_gyro_spikes`, `consolidate_overshoots`, and `consolidate_desyncs` share nearly identical "group events within a time gap" logic. A generic consolidation helper would eliminate the repetition.

### 14. [arch] `propwash/src/main.rs:582-616` — frame range/time filtering could be a core utility
The dump command implements its own frame iteration with time range and frame range filtering. This is reusable logic that could be a core iterator/utility, similar to the decimation logic noted in #9.
