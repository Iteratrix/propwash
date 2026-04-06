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
