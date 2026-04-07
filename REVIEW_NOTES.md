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

### 15. [arch] `propwash-core/src/analysis/mod.rs` — expose granular analysis, not just `analyze()`
`analyze()` is all-or-nothing: events + FFT + diagnostics + summary. But consumers have different needs — e.g., `cmd_scan` pays for FFT vibration analysis on every file but never displays it. The sub-functions (`unified_events::detect_all`, `fft::analyze_vibration_unified`, `diagnostics::diagnose`, `summary::summarize`) are already modular internally but not publicly composable. Make them part of the public API so consumers can pick what they need. Keep `analyze()` as a convenience "do everything" function.

### 16. [arch] `propwash-core/src/analysis/mod.rs:34-43` — format-specific event detection bypasses Unified
`analyze()` matches on `RawSession` to call `detect_ardupilot_events` and `detect_px4_log_events`. Same Unified bypass as #10/#12. Format-specific events (ArduPilot EV/ERR messages, PX4 log messages) should be surfaced through the Unified trait or via format-specific `detect_events` implementations, not via raw match in the analysis orchestrator.

### 17. [nit] `propwash-core/KNOWN_ISSUES.md` — delete, use GitHub Issues instead
File is effectively empty ("no known issues"). A static markdown file for issue tracking goes stale; GitHub Issues with a `known-issue` label is the right tool for this.

### 18. [arch] `propwash-core/src/lib.rs:17-43` — format detection and dispatch belongs in `format/mod.rs`
Magic byte constants and the `decode()` dispatch logic currently live in the crate root. They should move to `format/mod.rs` — it's already the parent of `ap`, `bf`, `px4`. The crate root should just re-export. Additionally, expose a `format::detect(data) -> Option<Format>` enum so consumers can identify the format without parsing (useful for CLI info, web UI file type display). Each format's magic bytes should live with its respective parser module.

### 19. [arch] `propwash-core/src/lib.rs:36-43` — unrecognized format should be a `ParseError`, not a warning
`decode()` returns a valid `Log` with zero sessions and a warning when the input is unrecognizable. This is inconsistent: `decode_file()` returns `Err(ParseError::NoData)` for empty files, but unrecognized data gets silently swallowed. Add a `ParseError::UnrecognizedFormat` variant and make `decode()` return `Result<Log, ParseError>`. Callers shouldn't have to inspect `sessions.is_empty()` to detect failure.

### 20. [nit] `propwash-core/src/types.rs:74` — `#[non_exhaustive]` on `SensorField` is premature
`Unknown(String)` already handles unrecognized field names at the data level. `#[non_exhaustive]` guards against new *variants* being added, but with no external consumers yet, it just forces unnecessary `_ =>` wildcard arms in internal matches. Remove it; add it back when the API stabilizes and there are real downstream users.

### 21. [arch] `propwash-core/src/types.rs:100` — `SensorField::from_header` is Betaflight-specific
`from_header` parses Betaflight's naming convention (`gyroADC[0]`, `axisP[1]`, `rcCommand[3]`) as if it's the canonical representation. ArduPilot and PX4 have entirely different native names but their `Unified` impls launder everything through BF-style names (constructing `SensorField` variants directly, then `Display` outputs BF names, then `from_header` parses them back). Betaflight's conventions have leaked into the format-agnostic layer. Fix: rename `from_header` to `from_betaflight_header` and move it to the BF parser, or make it explicitly a display-name parser for the `field_by_name` convenience method. The `SensorField` enum itself is the right abstraction — the string roundtrip through BF names is the problem.

### 22. [nit] `propwash-core/src/types.rs:144-152` — `is_motor()` and `is_erpm()` are unnecessary
Trivial wrappers around `matches!()` with only 2 call sites, both in the BF parser. They clutter the public API of `SensorField` for no real value. Delete them; callers can use `matches!` directly.

### 23. [style] `propwash-core/src/types.rs:253-281` — `Value` enum is dead code
The `Value` enum (`Int`, `Float`, `Str`, `Bool`) and its `as_int()`/`as_float()` methods are defined but never constructed or consumed anywhere. Delete it.

### 24. [arch] `propwash-core/src/types.rs:304-307` — `field_by_name` is a code smell
`field_by_name` encourages stringly-typed field access when `SensorField` already provides a type-safe enum. Internal code uses hardcoded strings like `"time"` and `"gyroADC[0]"` instead of `field(&SensorField::Time)`. The method's own doc comment acknowledges it's for "system boundaries (WASM bridge, CLI user input)" — but it's leaked into internal usage. Fix: delete `field_by_name` entirely. At system boundaries, callers do the conversion explicitly: `let sf = SensorField::parse(input)?; unified.field(&sf);`. This makes the stringly-typed → typed conversion visible, and lets callers handle unknown fields (e.g., return an error to the user) instead of silently getting an empty vec. Rename `from_header` to `SensorField::parse` (see #21). Internal code always uses `field()` with typed variants.

### 25. [arch] `propwash-core/src/types.rs:314-348` — `RawSession` enum: keep but make it a power-user escape hatch
~~Initially considered replacing `RawSession` with `Box<dyn Unified>`, but that loses the ability to access format-specific data entirely.~~ The current design (enum + `raw` feature gate) is actually the right shape. The problem is that `Unified` isn't rich enough, so consumers enable `raw` out of necessity. Fix: enrich `Unified` (#12) so CLI and web can drop `raw`. The `raw` feature then becomes a deliberate opt-in for power users (e.g., `compare_official.py`, custom Betaflight tuning tools) that genuinely need format internals.

### 26. [arch] `propwash-core/src/types.rs:332-348` — `Session` should `impl Unified`
`Session` doesn't implement `Unified` — it has a `unified()` method that returns `&dyn Unified`. So you always write `session.unified().frame_count()` instead of `session.frame_count()`. `Session` should `impl Unified` by delegating to `self.raw`. This makes `Session` itself the sealed-interface type: consumers call trait methods directly on it, and only reach into `.raw` when they need format-specific access via the `raw` feature.

### 27. [arch] `propwash-core/src/format/bf/frame.rs:14-27` — frame markers should be an enum
`MARKER_I`, `MARKER_P`, `MARKER_S`, `MARKER_G`, `MARKER_H`, `MARKER_E` are bare `u8` consts representing a sealed set of frame types. The `is_valid_marker()` function is a code smell — should be a `FrameMarker` enum with `TryFrom<u8>`. Event type IDs (lines 22-27) are lower priority since they're already parsed into the `BfEvent` enum, but could also benefit from being an enum for documentation value.

### 28. [arch] `propwash-core/src/format/px4/parser.rs:14-26` — ULog message types should be an enum
13 bare `u8` consts for ULog message type codes (`MSG_FORMAT`, `MSG_DATA`, `MSG_INFO`, etc.). This is a sealed set — should be a `ULogMsgType` enum with `TryFrom<u8>`. The `is_known_msg_type()` function at line 588 has the same smell as BF's `is_valid_marker`. Fulfils #2 (enums/newtypes for sealed option sets).

### 29. [style] Format parsers — prefer iterators over push loops for collection building
The `field_names()` implementations in all three formats (AP `types.rs:207-233`, PX4 `types.rs:229-261`, BF `types.rs:365`) use conditional `for` loops pushing into a `Vec`. These could use chained iterators (`flat_map`/`chain`/`filter`). The `field_names()` code across all three formats is also near-identical — potential for a shared helper. Parser core loops (frame decoding, error recovery) are fine as `for` — complex control flow doesn't benefit from forced iterator style.

### 30. [style] `&mut Vec` out parameters throughout parsers and diagnostics
Multiple functions use `&mut Vec<T>` as an output parameter instead of returning the data:
- **Parsers**: `ap/parser.rs:13`, `px4/parser.rs:29`, `px4/parser.rs:118` take `warnings: &mut Vec<Warning>`
- **Diagnostics**: `diagnostics.rs:122,210,242,286` take `diagnostics: &mut Vec<Diagnostic>`

This is C-style "out parameter" pattern that obscures data flow. Idiomatic Rust: return `Vec<T>` and let the caller `extend`. Each `diagnose_*` function returns its findings; the caller chains them. For warnings during parsing, return them alongside the parsed result (tuple or struct). Makes it clear at the call site how much data each function contributes.
