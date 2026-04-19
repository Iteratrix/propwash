import { bucketDiagnostics, classifyDelta } from "../format.js";
import type { SessionResult, Spectrum } from "../types.js";
import { $, chartWidth, escHtml, workspace, lookupSession } from "../state.js";
import { AXIS_COLORS } from "../chart-config.js";

export function populateComparePickers(): void {
  const selectA = $("#compare-a") as HTMLSelectElement;
  const selectB = $("#compare-b") as HTMLSelectElement;
  const prevA = selectA.value;
  const prevB = selectB.value;

  for (const sel of [selectA, selectB]) {
    sel.innerHTML = "";
    for (const file of workspace) {
      const name = file.filename || `File ${file.file_id}`;
      const group = document.createElement("optgroup");
      group.label = name;
      for (let i = 0; i < file.sessions.length; i++) {
        const s = file.sessions[i];
        const opt = document.createElement("option");
        opt.value = `${file.file_id}:${i}`;
        opt.textContent = file.sessions.length === 1
          ? `${name} (${s.duration_seconds.toFixed(1)}s)`
          : `Session ${s.index} (${s.duration_seconds.toFixed(1)}s)`;
        group.appendChild(opt);
      }
      sel.appendChild(group);
    }
  }

  // Restore previous selections or default A=first, B=second
  const allOpts = Array.from(selectA.options);
  if (prevA && allOpts.some(o => o.value === prevA)) {
    selectA.value = prevA;
  }
  if (prevB && allOpts.some(o => o.value === prevB)) {
    selectB.value = prevB;
  } else if (allOpts.length >= 2) {
    selectB.selectedIndex = 1;
  }

  selectA.onchange = renderCompare;
  selectB.onchange = renderCompare;
}

export function renderCompare(): void {
  const a = lookupSession(($("#compare-a") as HTMLSelectElement).value);
  const b = lookupSession(($("#compare-b") as HTMLSelectElement).value);

  if (!a || !b) {
    $("#compare-summary-table").innerHTML = '<p class="hint">Select two sessions to compare.</p>';
    $("#compare-diag-content").innerHTML = "";
    $("#compare-spectrum-plots").innerHTML = "";
    return;
  }

  renderComparisonSummary(a, b);
  renderComparisonTuning(a, b);
  renderComparisonDiagnostics(a, b);
  renderComparisonSpectra(a, b);
}

export function renderComparisonSummary(a: SessionResult, b: SessionResult): void {
  const rows = [
    ["Firmware", a.firmware, b.firmware],
    ["Craft", a.craft, b.craft],
    ["Duration", `${a.duration_seconds.toFixed(1)}s`, `${b.duration_seconds.toFixed(1)}s`],
    ["Sample Rate", `${a.sample_rate_hz.toFixed(0)} Hz`, `${b.sample_rate_hz.toFixed(0)} Hz`],
    ["Frames", a.frame_count.toLocaleString(), b.frame_count.toLocaleString()],
  ];

  const eventRows: [string, number, number, "lower" | "higher"][] = [
    ["Events", a.analysis.summary.total_events, b.analysis.summary.total_events, "lower"],
    ["Desyncs", a.analysis.summary.desyncs, b.analysis.summary.desyncs, "lower"],
    ["Gyro Spikes", a.analysis.summary.gyro_spikes, b.analysis.summary.gyro_spikes, "lower"],
    ["Motor Sats", a.analysis.summary.motor_saturations, b.analysis.summary.motor_saturations, "lower"],
    ["Overshoots", a.analysis.summary.overshoots, b.analysis.summary.overshoots, "lower"],
  ];

  let html = `<table class="compare-table">
    <thead><tr><th>Metric</th><th>Session A</th><th>Session B</th><th>Delta</th></tr></thead><tbody>`;

  for (const [label, va, vb] of rows) {
    html += `<tr><td>${label}</td><td>${va}</td><td>${vb}</td><td></td></tr>`;
  }

  for (const [label, va, vb, better] of eventRows) {
    const diff = vb - va;
    const cls = classifyDelta(diff, better);
    const sign = diff > 0 ? "+" : "";
    html += `<tr><td>${label}</td><td>${va}</td><td>${vb}</td><td class="delta ${cls}">${sign}${diff}</td></tr>`;
  }

  html += "</tbody></table>";
  $("#compare-summary-table").innerHTML = html;
}

export function renderComparisonTuning(a: SessionResult, b: SessionResult): void {
  const panel = $("#compare-tuning-panel");
  const content = $("#compare-tuning-content");

  const pidA = a.analysis.pid;
  const pidB = b.analysis.pid;

  if ((!pidA || pidA.tuning.length === 0) && (!pidB || pidB.tuning.length === 0)) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");

  // Collect all axes present in either session
  const axisSet = new Set<string>();
  for (const t of pidA?.tuning ?? []) axisSet.add(t.axis);
  for (const t of pidB?.tuning ?? []) axisSet.add(t.axis);

  const ratingColor: Record<string, string> = {
    Tight: "var(--green)", Good: "var(--green)",
    Sluggish: "var(--yellow, #e8b84a)",
    Overshooting: "var(--red)", Oscillating: "var(--red)",
  };

  let html = `<table class="compare-table"><thead><tr>
    <th>Axis</th><th>Rating A</th><th>Rating B</th>
    <th>Overshoot A</th><th>Overshoot B</th>
    <th>P</th><th>D</th>
  </tr></thead><tbody>`;

  for (const axis of axisSet) {
    const ta = pidA?.tuning.find(t => t.axis === axis);
    const tb = pidB?.tuning.find(t => t.axis === axis);

    const rA = ta?.rating ?? "\u2014";
    const rB = tb?.rating ?? "\u2014";
    const cA = ratingColor[rA] || "var(--text-dim)";
    const cB = ratingColor[rB] || "var(--text-dim)";

    const osA = ta ? `${ta.overshoot_percent.toFixed(0)}%` : "\u2014";
    const osB = tb ? `${tb.overshoot_percent.toFixed(0)}%` : "\u2014";

    const pA = ta?.current.p;
    const pB = tb?.current.p;
    const dA = ta?.current.d;
    const dB = tb?.current.d;

    const fmtGainPair = (a: number | null | undefined, b: number | null | undefined) => {
      if (a == null && b == null) return "\u2014";
      const sa = a != null ? `${a}` : "\u2014";
      const sb = b != null ? `${b}` : "\u2014";
      if (a === b) return sa;
      return `${sa} / ${sb}`;
    };

    html += `<tr>
      <td>${axis}</td>
      <td style="color:${cA};font-weight:600">${rA}</td>
      <td style="color:${cB};font-weight:600">${rB}</td>
      <td>${osA}</td><td>${osB}</td>
      <td>${fmtGainPair(pA, pB)}</td>
      <td>${fmtGainPair(dA, dB)}</td>
    </tr>`;
  }

  html += "</tbody></table>";
  content.innerHTML = html;
}

export function renderComparisonDiagnostics(a: SessionResult, b: SessionResult): void {
  const diagA = a.analysis.diagnostics || [];
  const diagB = b.analysis.diagnostics || [];

  const { fixed, newIssues, unchanged } = bucketDiagnostics(diagA, diagB);

  let html = "";

  if (fixed.length > 0) {
    html += `<div class="diag-diff-section"><h3 class="fixed">Fixed (${fixed.length})</h3>`;
    for (const d of fixed) {
      html += `<div class="diagnostic Info"><div class="message">${escHtml(d.message)}</div></div>`;
    }
    html += "</div>";
  }

  if (newIssues.length > 0) {
    html += `<div class="diag-diff-section"><h3 class="new-issue">New Issues (${newIssues.length})</h3>`;
    for (const d of newIssues) {
      html += `<div class="diagnostic ${d.severity}"><div class="message">${escHtml(d.message)}</div></div>`;
    }
    html += "</div>";
  }

  if (unchanged.length > 0) {
    html += `<div class="diag-diff-section"><h3 class="unchanged">Unchanged (${unchanged.length})</h3>`;
    for (const d of unchanged) {
      html += `<div class="diagnostic ${d.severity}"><div class="message">${escHtml(d.message)}</div></div>`;
    }
    html += "</div>";
  }

  if (!html) html = '<p class="hint">No diagnostics to compare.</p>';
  $("#compare-diag-content").innerHTML = html;
}

export function renderComparisonSpectra(a: SessionResult, b: SessionResult): void {
  const container = $("#compare-spectrum-plots");
  container.innerHTML = "";

  const vibA = a.analysis.vibration;
  const vibB = b.analysis.vibration;
  if (!vibA?.spectra || !vibB?.spectra) {
    container.innerHTML = '<p class="hint">No spectrum data to compare.</p>';
    return;
  }

  const axes = ["Roll", "Pitch", "Yaw"];
  for (const axisName of axes) {
    const specA = vibA.spectra.find((s: Spectrum) => s.axis === axisName);
    const specB = vibB.spectra.find((s: Spectrum) => s.axis === axisName);
    if (!specA || !specB) continue;

    const row = document.createElement("div");
    row.className = "spectrum-row";
    const title = document.createElement("h3");
    title.textContent = axisName;
    row.appendChild(title);

    const plotDiv = document.createElement("div");
    row.appendChild(plotDiv);
    container.appendChild(row);

    const maxFreq = Math.min(specA.sample_rate_hz / 2, 1000);
    const endA = specA.frequencies_hz.findIndex((f: number) => f > maxFreq);
    const nA = endA > 0 ? endA : specA.frequencies_hz.length;
    const endB = specB.frequencies_hz.findIndex((f: number) => f > maxFreq);
    const nB = endB > 0 ? endB : specB.frequencies_hz.length;

    const color = AXIS_COLORS[axisName] || "#5b8def";

    const opts = {
      width: chartWidth(),
      height: 200,
      cursor: { show: true },
      scales: { x: { time: false } },
      axes: [
        { label: "Hz", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
        { label: "dB", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
      ],
      series: [
        {},
        { label: "Session A", stroke: color, width: 1.5 },
        { label: "Session B", stroke: color, width: 1.5, dash: [4, 4] },
      ],
    };

    new uPlot(opts, [
      specA.frequencies_hz.slice(0, nA),
      specA.magnitudes_db.slice(0, nA),
      specB.magnitudes_db.slice(0, nB),
    ], plotDiv);
  }
}
