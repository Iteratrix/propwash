import init, { add_file, clear_workspace, get_timeseries, get_spectrogram, get_filter_config, get_raw_frames, get_trend, get_step_overlay } from "../pkg/propwash_web.js";
import type { WorkspaceFile, SessionRef, SessionResult, TrendPoint, FlightEvent, Diagnostic, VibrationAnalysis, Spectrum, FilterConfig, PidAnalysis, StepOverlay } from "./types.js";
import { formatEventType, formatEventDetails, eventColor, heatColor, bucketDiagnostics, classifyDelta } from "./format.js";

declare const echarts: any;
declare const uPlot: any;
declare const __WASM_URL__: string;

const $ = (sel: string): HTMLElement => document.querySelector(sel) as HTMLElement;
const $$ = (sel: string): NodeListOf<HTMLElement> => document.querySelectorAll(sel) as NodeListOf<HTMLElement>;

declare global {
  interface Window {
    _rawPage: (fileId: number, sessionIdx: number, start: number) => void;
  }
}

function chartWidth(): number {
  const padding = window.innerWidth <= 640 ? 24 : 64;
  return Math.min($("main").clientWidth - padding, 880);
}

let wasmReady = false;
let workspace: WorkspaceFile[] = [];
let activeSession: SessionRef | null = null;

const AXIS_COLORS: Record<string, string> = {
  Roll:  "#5b8def",
  Pitch: "#4ec88c",
  Yaw:   "#e8b84a",
  X: "#5b8def",
  Y: "#4ec88c",
  Z: "#e8b84a",
};

const FIELD_GROUPS = {
  gyro: {
    label: "Gyro (deg/s)",
    fields: ["gyro[roll]", "gyro[pitch]", "gyro[yaw]"],
    names:  ["Roll", "Pitch", "Yaw"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
  },
  spVsGyro: {
    label: "Setpoint vs Gyro (deg/s)",
    fields: ["setpoint[roll]", "gyro[roll]", "setpoint[pitch]", "gyro[pitch]", "setpoint[yaw]", "gyro[yaw]"],
    names:  ["SP Roll", "Gyro Roll", "SP Pitch", "Gyro Pitch", "SP Yaw", "Gyro Yaw"],
    colors: ["#5b8def66", "#5b8def", "#4ec88c66", "#4ec88c", "#e8b84a66", "#e8b84a"],
  },
  gyroFilt: {
    label: "Filtered vs Unfiltered Gyro (deg/s)",
    fields: ["gyro[roll]", "gyro_unfilt[roll]", "gyro[pitch]", "gyro_unfilt[pitch]", "gyro[yaw]", "gyro_unfilt[yaw]"],
    names:  ["Filt Roll", "Raw Roll", "Filt Pitch", "Raw Pitch", "Filt Yaw", "Raw Yaw"],
    colors: ["#5b8def", "#5b8def44", "#4ec88c", "#4ec88c44", "#e8b84a", "#e8b84a44"],
  },
  motors: {
    label: "Motors",
    fields: ["motor[0]", "motor[1]", "motor[2]", "motor[3]"],
    names:  ["Motor 1", "Motor 2", "Motor 3", "Motor 4"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e85454"],
  },
  rc: {
    label: "RC Commands",
    fields: ["rc[roll]", "rc[pitch]", "rc[yaw]", "rc[throttle]"],
    names:  ["Roll", "Pitch", "Yaw", "Throttle"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e8944a"],
  },
  voltage: {
    label: "Battery / Throttle",
    fields: ["vbat", "rc[throttle]"],
    names:  ["Battery (V)", "Throttle"],
    colors: ["#e85454", "#e8944a"],
  },
  pidError: {
    label: "PID Error — Setpoint minus Gyro (deg/s)",
    fields: ["setpoint[roll]", "gyro[roll]", "setpoint[pitch]", "gyro[pitch]", "setpoint[yaw]", "gyro[yaw]"],
    names:  ["Roll", "Pitch", "Yaw"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
    computed: "error",
  },
  pids: {
    label: "PIDs (Roll)",
    fields: ["pid_p[roll]", "pid_i[roll]", "pid_d[roll]", "feedforward[roll]"],
    names:  ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  },
  pidsPitch: {
    label: "PIDs (Pitch)",
    fields: ["pid_p[pitch]", "pid_i[pitch]", "pid_d[pitch]", "feedforward[pitch]"],
    names:  ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  },
  pidsYaw: {
    label: "PIDs (Yaw)",
    fields: ["pid_p[yaw]", "pid_i[yaw]", "pid_d[yaw]", "feedforward[yaw]"],
    names:  ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  },
  erpm: {
    label: "Motor RPM",
    fields: ["erpm[0]", "erpm[1]", "erpm[2]", "erpm[3]"],
    names:  ["Motor 1", "Motor 2", "Motor 3", "Motor 4"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e85454"],
  },
  altitude: {
    label: "Altitude / GPS Speed",
    fields: ["altitude", "gps_speed"],
    names:  ["Altitude (m)", "GPS Speed (m/s)"],
    colors: ["#5b8def", "#4ec88c"],
  },
  gps: {
    label: "GPS Position",
    fields: ["gps_lat", "gps_lng", "heading"],
    names:  ["Latitude", "Longitude", "Heading"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
  },
  rssi: {
    label: "RSSI / Link Quality (%)",
    fields: ["rssi"],
    names:  ["RSSI"],
    colors: ["#4ec88c"],
  },
};

const TS_MAX_POINTS = 4000;
let tsPlots: any[] = [];
let navigateTarget: number | null = null;
let tsSync: any = null;
let filterConfig: FilterConfig | null = null;

let renderedViews = new Set<string>();

/** Look up the active session's analysis result. */
function activeSessionResult(): SessionResult | null {
  if (!activeSession) return null;
  const file = workspace.find(f => f.file_id === activeSession!.fileId);
  return file?.sessions[activeSession.sessionIdx] ?? null;
}

/** Build a flat list of all sessions across workspace files. */
function allSessionRefs(): SessionRef[] {
  const refs: SessionRef[] = [];
  for (const file of workspace) {
    const name = file.filename || `File ${file.file_id}`;
    for (let i = 0; i < file.sessions.length; i++) {
      const s = file.sessions[i];
      const label = file.sessions.length === 1
        ? name
        : `${name} / Session ${s.index}`;
      refs.push({ fileId: file.file_id, sessionIdx: i, label });
    }
  }
  return refs;
}

async function boot() {
  await init({ module_or_path: __WASM_URL__ });
  wasmReady = true;
  setupDropZone();
  setupTimeseriesControls();
  setupViewTabs();
  $("#reset-btn").addEventListener("click", reset);
  $("#add-files-btn")?.addEventListener("click", () => {
    ($("#file-input") as HTMLInputElement).click();
  });
}

function setupViewTabs() {
  const tabs = $$("#view-tabs .view-tab");
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      $$(".view-content").forEach((v) => v.classList.remove("active"));
      const view = tab.dataset.view!;
      $(`.view-content[data-view="${view}"]`)!.classList.add("active");
      renderViewIfNeeded(view);
    });
  });
}

function reset() {
  workspace = [];
  activeSession = null;
  clear_workspace();
  disposeEcharts();
  renderedViews.clear();
  $("#results").classList.add("hidden");
  $("#drop-zone").classList.remove("hidden");
  ($("#file-input") as HTMLInputElement).value = "";
}


function populateComparePickers(): void {
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

function lookupSession(val: string): SessionResult | null {
  const [fileId, sessionIdx] = val.split(":").map(Number);
  const file = workspace.find(f => f.file_id === fileId);
  return file?.sessions[sessionIdx] ?? null;
}

function renderCompare(): void {
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

function renderComparisonSummary(a: SessionResult, b: SessionResult): void {
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

function renderComparisonTuning(a: SessionResult, b: SessionResult): void {
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

function renderComparisonDiagnostics(a: SessionResult, b: SessionResult): void {
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

function renderComparisonSpectra(a: SessionResult, b: SessionResult): void {
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

function setupDropZone(): void {
  const zone = $("#drop-zone");
  const input = $("#file-input") as HTMLInputElement;

  zone.addEventListener("click", (e: Event) => {
    if (e.target !== input) input.click();
  });
  input.addEventListener("click", (e: Event) => e.stopPropagation());
  input.addEventListener("change", () => {
    if (input.files && input.files.length > 0) handleFiles(input.files);
  });

  zone.addEventListener("dragover", (e: Event) => {
    e.preventDefault();
    zone.classList.add("drag-over");
  });
  zone.addEventListener("dragleave", () => zone.classList.remove("drag-over"));
  zone.addEventListener("drop", (e: Event) => {
    e.preventDefault();
    zone.classList.remove("drag-over");
    const dt = (e as DragEvent).dataTransfer;
    if (dt && dt.files.length > 0) handleFiles(dt.files);
  });
}

async function handleFiles(files: FileList): Promise<void> {
  if (!wasmReady) return;

  $("#drop-zone").classList.add("hidden");
  $("#loading").classList.remove("hidden");

  await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));

  for (let i = 0; i < files.length; i++) {
    const file = files[i];
    const buffer = await file.arrayBuffer();
    const data = new Uint8Array(buffer);
    const json = add_file(data, file.name);
    const result: WorkspaceFile = JSON.parse(json);
    if (result.sessions.length > 0) {
      workspace.push(result);
    }
  }

  $("#loading").classList.add("hidden");

  if (workspace.length === 0) {
    $("#drop-zone").classList.remove("hidden");
    alert("No sessions found in the loaded files.");
    return;
  }

  renderSessionSelector();
  $("#results").classList.remove("hidden");

  // Select first session
  const refs = allSessionRefs();
  if (refs.length > 0) {
    showSession(refs[0]);
  }
}

function renderSessionSelector() {
  const nav = $("#session-tabs");
  nav.innerHTML = "";

  const refs = allSessionRefs();
  if (refs.length <= 1) return;

  const select = document.createElement("select");
  select.className = "session-select";

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
    select.appendChild(group);
  }

  select.addEventListener("change", () => {
    const [fileId, sessionIdx] = select.value.split(":").map(Number);
    const ref = refs.find(r => r.fileId === fileId && r.sessionIdx === sessionIdx);
    if (ref) showSession(ref);
  });

  nav.appendChild(select);
}

function showSession(ref: SessionRef): void {
  activeSession = ref;
  filterConfig = JSON.parse(get_filter_config(ref.fileId, ref.sessionIdx));
  renderedViews.clear();
  const activeView = $(".view-tab.active")?.dataset.view || "overview";
  renderViewIfNeeded(activeView);
}

function renderViewIfNeeded(view: string): void {
  if (!activeSession) return;
  const key = `${view}-${activeSession.fileId}-${activeSession.sessionIdx}`;
  if (renderedViews.has(key)) return;
  renderedViews.add(key);

  const s = activeSessionResult();
  if (!s) return;

  switch (view) {
    case "overview":
      renderSummary(s);
      renderTuning(s.analysis.pid);
      renderStepOverlay(activeSession);
      renderDiagnostics(s.analysis.diagnostics);
      renderEvents(s.analysis.events);
      break;
    case "timeline":
      renderTimeseries(activeSession, $(".ts-tab.active")?.dataset.group || "gyro");
      break;
    case "spectrum":
      renderSpectraEcharts(s.analysis.vibration);
      renderSpectrogram(activeSession);
      renderThrottleBandsEcharts(s.analysis.vibration);
      renderAccel(s.analysis.vibration);
      renderPropwash(s.analysis.vibration);
      break;
    case "compare":
      populateComparePickers();
      renderCompare();
      break;
    case "trend":
      renderTrend();
      break;
    case "raw":
      renderRawData(activeSession);
      break;
  }
}

function navigateToTime(seconds: number): void {
  // Switch to timeline tab
  const tabs = $$("#view-tabs .view-tab");
  tabs.forEach((t) => t.classList.remove("active"));
  const timelineTab = $(".view-tab[data-view='timeline']");
  timelineTab.classList.add("active");
  $$(".view-content").forEach((v) => v.classList.remove("active"));
  $(`.view-content[data-view="timeline"]`)!.classList.add("active");

  // Ensure timeline is rendered
  renderViewIfNeeded("timeline");

  // Zoom to ±1.5s window around the event
  const win = 1.5;
  const min = Math.max(0, seconds - win);
  const max = seconds + win;
  for (const p of tsPlots) {
    p.setScale("x", { min, max });
  }

  // Flash the navigation target marker
  navigateTarget = seconds;
  setTimeout(() => { navigateTarget = null; for (const p of tsPlots) p.redraw(); }, 2000);
  for (const p of tsPlots) p.redraw();
}

function setupTimeseriesControls() {
  const tabs = $$("#ts-tabs .ts-tab");
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      if (!activeSession) return;
      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      renderedViews.delete(`timeline-${activeSession.fileId}-${activeSession.sessionIdx}`);
      renderTimeseries(activeSession, tab.dataset.group!);
    });
  });
  $("#ts-reset-zoom").addEventListener("click", () => {
    if (!activeSession) return;
    const activeGroup = $(".ts-tab.active")?.dataset.group || "gyro";
    renderedViews.delete(`timeline-${activeSession.fileId}-${activeSession.sessionIdx}`);
    renderTimeseries(activeSession, activeGroup);
  });
}

function renderTimeseries(ref: SessionRef, group: string): void {
  const container = $("#ts-charts");
  container.innerHTML = "";
  tsPlots = [];

  const g = FIELD_GROUPS[group as keyof typeof FIELD_GROUPS];
  if (!g) return;

  const allFields = g.fields.join(",");
  const json = get_timeseries(ref.fileId, ref.sessionIdx, TS_MAX_POINTS, allFields);
  const ts = JSON.parse(json);
  if (ts.error) {
    container.innerHTML = `<p class="hint">${ts.error}</p>`;
    return;
  }

  const time = ts.time_s;
  if (!time || time.length === 0) {
    container.innerHTML = '<p class="hint">No time-series data.</p>';
    return;
  }

  const syncKey = "ts-sync";
  tsSync = uPlot.sync(syncKey);

  const sr = activeSessionResult();
  const events = sr?.analysis?.events || [];

  const width = chartWidth();

  const datasets = [time];
  const series = [{}];
  let hasData = false;

  if ("computed" in g && (g as any).computed === "error") {
    // Compute setpoint - gyro per axis (fields are [sp_r, g_r, sp_p, g_p, sp_y, g_y])
    for (let i = 0; i < g.fields.length; i += 2) {
      const sp = ts.fields[g.fields[i]];
      const gy = ts.fields[g.fields[i + 1]];
      if (sp && gy && sp.length > 0 && gy.length > 0) {
        const err = sp.map((v: number, j: number) => v - (gy[j] ?? 0));
        datasets.push(err);
        series.push({
          label: g.names[i / 2],
          stroke: g.colors[i / 2],
          width: 1,
        });
        hasData = true;
      }
    }
  } else {
    for (let i = 0; i < g.fields.length; i++) {
      const vals = ts.fields[g.fields[i]];
      if (vals && vals.length > 0) {
        datasets.push(vals);
        series.push({
          label: g.names[i],
          stroke: g.colors[i],
          width: 1,
        });
        hasData = true;
      }
    }
  }

  if (!hasData) {
    container.innerHTML = `<p class="hint">No ${g.label} data in this log.</p>`;
    return;
  }

  const opts = {
    width,
    height: 280,
    cursor: {
      lock: true,
      sync: { key: syncKey, setSeries: true },
    },
    scales: { x: { time: false } },
    axes: [
      {
        label: "Time (s)",
        stroke: "#8888a0",
        grid: { stroke: "rgba(42,45,58,0.6)" },
        ticks: { stroke: "rgba(42,45,58,0.6)" },
        font: "11px JetBrains Mono, monospace",
        labelFont: "11px JetBrains Mono, monospace",
      },
      {
        label: g.label,
        stroke: "#8888a0",
        grid: { stroke: "rgba(42,45,58,0.6)" },
        ticks: { stroke: "rgba(42,45,58,0.6)" },
        font: "11px JetBrains Mono, monospace",
        labelFont: "11px JetBrains Mono, monospace",
      },
    ],
    series,
    plugins: [eventMarkersPlugin(events), navigateMarkerPlugin()],
    select: { show: true },
    hooks: {
      setSelect: [
        (u: any) => {
          const min = u.posToVal(u.select.left, "x");
          const max = u.posToVal(u.select.left + u.select.width, "x");
          if (max - min > 0.01) {
            for (const p of tsPlots) {
              p.setScale("x", { min, max });
            }
          }
        },
      ],
    },
  };

  const row = document.createElement("div");
  row.className = "ts-chart-row";
  container.appendChild(row);

  const plot = new uPlot(opts, datasets, row);
  tsPlots.push(plot);
}

function eventMarkersPlugin(events: FlightEvent[]): any {
  return {
    hooks: {
      draw: [
        (u: any) => {
          const ctx = u.ctx;
          const [xMin, xMax] = [u.scales.x.min, u.scales.x.max];
          for (const e of events) {
            const t = e.time_seconds;
            if (t < xMin || t > xMax) continue;
            const x = u.valToPos(t, "x", true);
            ctx.save();
            ctx.strokeStyle = eventColor(e.kind.type);
            ctx.globalAlpha = 0.4;
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(x, u.bbox.top);
            ctx.lineTo(x, u.bbox.top + u.bbox.height);
            ctx.stroke();
            ctx.restore();
          }
        },
      ],
    },
  };
}

function navigateMarkerPlugin(): any {
  return {
    hooks: {
      draw: [
        (u: any) => {
          if (navigateTarget == null) return;
          const t = navigateTarget;
          const [xMin, xMax] = [u.scales.x.min, u.scales.x.max];
          if (t < xMin || t > xMax) return;
          const x = u.valToPos(t, "x", true);
          const ctx = u.ctx;
          ctx.save();
          ctx.strokeStyle = "#ffffff";
          ctx.lineWidth = 2;
          ctx.globalAlpha = 0.8;
          ctx.setLineDash([4, 4]);
          ctx.beginPath();
          ctx.moveTo(x, u.bbox.top);
          ctx.lineTo(x, u.bbox.top + u.bbox.height);
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillStyle = "#ffffff";
          ctx.globalAlpha = 0.9;
          ctx.font = "10px JetBrains Mono, monospace";
          ctx.textAlign = "center";
          ctx.fillText(`${t.toFixed(2)}s`, x, u.bbox.top - 4);
          ctx.restore();
        },
      ],
    },
  };
}

function renderSummary(session: SessionResult): void {
  const grid = $("#summary-grid");
  const cards = [
    ["Firmware", session.firmware || "Unknown"],
    ["Craft", session.craft || "Unknown"],
    ["Duration", `${session.duration_seconds.toFixed(1)}s`],
    ["Sample Rate", `${session.sample_rate_hz.toFixed(0)} Hz`],
    ["Frames", session.frame_count.toLocaleString()],
    ["Motors", session.analysis.summary.motor_count],
    ["Events", session.analysis.summary.total_events],
    ["Desyncs", session.analysis.summary.desyncs],
  ];

  if (session.is_truncated) {
    cards.push(["Status", "TRUNCATED"]);
  }
  if (session.corrupt_bytes > 0) {
    cards.push(["Corrupt", `${session.corrupt_bytes.toLocaleString()} bytes`]);
  }

  grid.innerHTML = cards
    .map(([label, value]) => {
      const warn = label === "Status" || label === "Corrupt";
      return `<div class="summary-card${warn ? " summary-warn" : ""}">
        <div class="label">${label}</div>
        <div class="value" title="${value}">${value}</div>
      </div>`;
    })
    .join("");

  // Motor balance bars (remove previous if re-rendering)
  document.querySelector(".motor-balance-section")?.remove();
  const balance = session.analysis.summary.motor_balance;
  if (balance.length > 0) {
    const maxMean = Math.max(...balance.map(m => m.mean));
    const balanceHtml = balance.map(m => {
      const pct = maxMean > 0 ? (m.mean / maxMean * 100) : 0;
      const dev = m.deviation_percent;
      const color = Math.abs(dev) > 5 ? "#e85454" : Math.abs(dev) > 2 ? "#e8b84a" : "#4ec88c";
      const sign = dev > 0 ? "+" : "";
      return `<div class="motor-bar-row">
        <span class="motor-bar-label">Motor ${m.index + 1}</span>
        <div class="motor-bar-track">
          <div class="motor-bar-fill" style="width:${pct.toFixed(1)}%;background:${color}"></div>
        </div>
        <span class="motor-bar-value">${sign}${dev.toFixed(1)}%</span>
      </div>`;
    }).join("");
    grid.insertAdjacentHTML("afterend",
      `<div class="motor-balance-section">
        <h3>Motor Balance</h3>
        ${balanceHtml}
      </div>`
    );
  }
}

const echartsInstances: any[] = [];

function renderTuning(pid: PidAnalysis | null): void {
  const panel = $("#tuning-panel");
  const content = $("#tuning-content");

  if (!pid || pid.tuning.length === 0) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");

  const ratingColor: Record<string, string> = {
    Tight: "var(--green)",
    Good: "var(--green)",
    Sluggish: "var(--yellow, #e8b84a)",
    Overshooting: "var(--red, #e85454)",
    Oscillating: "var(--red, #e85454)",
  };

  const ratingLabel: Record<string, string> = {
    Tight: "Tight",
    Good: "Good",
    Sluggish: "Sluggish",
    Overshooting: "Overshooting",
    Oscillating: "Oscillating",
  };

  let html = `<table class="tuning-table">
    <thead><tr>
      <th>Axis</th><th>Rating</th>
      <th>P</th><th>I</th><th>D</th>
      <th>Overshoot</th><th>Rise</th><th>Steps</th>
    </tr></thead><tbody>`;

  for (const t of pid.tuning) {
    const color = ratingColor[t.rating] || "var(--text-dim)";
    const changed = t.current.p !== t.suggested.p || t.current.d !== t.suggested.d || t.current.i !== t.suggested.i;

    const fmtGain = (cur: number | null, sug: number | null) => {
      if (cur == null) return "\u2014";
      if (sug == null || cur === sug) return `${cur}`;
      return `${cur} <span class="gain-arrow">\u2192</span> <strong>${sug}</strong>`;
    };

    html += `<tr>
      <td>${t.axis}</td>
      <td style="color:${color};font-weight:600">${ratingLabel[t.rating] || t.rating}</td>
      <td class="gain-cell">${fmtGain(t.current.p, changed ? t.suggested.p : t.current.p)}</td>
      <td class="gain-cell">${fmtGain(t.current.i, changed ? t.suggested.i : t.current.i)}</td>
      <td class="gain-cell">${fmtGain(t.current.d, changed ? t.suggested.d : t.current.d)}</td>
      <td>${t.overshoot_percent.toFixed(0)}%</td>
      <td>${t.rise_time_ms.toFixed(1)}ms</td>
      <td>${t.step_count}</td>
    </tr>`;
  }

  html += "</tbody></table>";

  if (pid.tuning.some(t => t.rating === "Overshooting" || t.rating === "Oscillating")) {
    html += `<p class="hint" style="margin-top:0.5rem">Suggested values are conservative estimates based on step response analysis. Test changes one axis at a time.</p>`;
  }

  content.innerHTML = html;
}

function renderStepOverlay(ref: SessionRef): void {
  const panel = $("#step-overlay-panel");
  const container = $("#step-overlay-plots");

  const json = get_step_overlay(ref.fileId, ref.sessionIdx);
  const data: StepOverlay | { error: string } = JSON.parse(json);

  if ("error" in data || !data.axes || data.axes.length === 0) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");
  container.innerHTML = "";

  const width = chartWidth();
  const axisColor: Record<string, string> = {
    Roll: "#5b8def",
    Pitch: "#4ec88c",
    Yaw: "#e8b84a",
  };

  for (const axis of data.axes) {
    const row = document.createElement("div");
    row.className = "step-overlay-row";

    const title = document.createElement("h3");
    title.textContent = `${axis.axis} (${axis.gyro_steps.length} steps)`;
    row.appendChild(title);

    const plotDiv = document.createElement("div");
    row.appendChild(plotDiv);
    container.appendChild(row);

    const color = axisColor[axis.axis] || "#5b8def";
    const time = axis.time_ms;

    // Build datasets: time, then individual gyro traces, then average, then setpoint average
    const datasets: (number[] | Float64Array)[] = [time];
    const series: any[] = [{}];

    // Individual gyro traces — thin, semi-transparent
    for (let i = 0; i < axis.gyro_steps.length; i++) {
      datasets.push(axis.gyro_steps[i]);
      series.push({
        label: i === 0 ? "Steps" : "",
        stroke: color + "30",
        width: 0.5,
      });
    }

    // Average gyro — thick, solid
    datasets.push(axis.gyro_average);
    series.push({
      label: "Average",
      stroke: color,
      width: 2.5,
    });

    // Average setpoint — dashed gray
    // Use first setpoint trace as representative (they're all normalized similarly)
    if (axis.setpoint_steps.length > 0) {
      // Average the setpoint traces
      const spAvg = new Array(time.length).fill(0);
      for (const sp of axis.setpoint_steps) {
        for (let j = 0; j < time.length; j++) {
          spAvg[j] += (sp[j] ?? 0);
        }
      }
      const n = axis.setpoint_steps.length;
      for (let j = 0; j < time.length; j++) {
        spAvg[j] /= n;
      }
      datasets.push(spAvg);
      series.push({
        label: "Setpoint",
        stroke: "#888888",
        width: 1.5,
        dash: [6, 3],
      });
    }

    const opts = {
      width,
      height: 220,
      cursor: { show: true },
      scales: { x: { time: false } },
      legend: { show: true },
      axes: [
        {
          label: "Time (ms)",
          stroke: "#8888a0",
          grid: { stroke: "rgba(42,45,58,0.6)" },
          ticks: { stroke: "rgba(42,45,58,0.6)" },
          font: "11px JetBrains Mono, monospace",
          labelFont: "11px JetBrains Mono, monospace",
        },
        {
          label: "Normalized",
          stroke: "#8888a0",
          grid: { stroke: "rgba(42,45,58,0.6)" },
          ticks: { stroke: "rgba(42,45,58,0.6)" },
          font: "11px JetBrains Mono, monospace",
          labelFont: "11px JetBrains Mono, monospace",
        },
      ],
      series,
    };

    new uPlot(opts, datasets, plotDiv);
  }
}

function disposeEcharts() {
  for (const inst of echartsInstances) {
    inst.dispose();
  }
  echartsInstances.length = 0;
}

function renderSpectraEcharts(vibration: VibrationAnalysis | null): void {
  const container = $("#spectrum-plots");
  container.innerHTML = "";

  if (!vibration || !vibration.spectra || vibration.spectra.length === 0) {
    container.innerHTML = '<p class="hint">No spectrum data available.</p>';
    return;
  }

  for (const spectrum of vibration.spectra) {
    const row = document.createElement("div");
    row.className = "spectrum-row";

    const title = document.createElement("h3");
    title.textContent = spectrum.axis;
    row.appendChild(title);

    const chartDiv = document.createElement("div");
    chartDiv.style.width = "100%";
    chartDiv.style.height = "250px";
    row.appendChild(chartDiv);
    container.appendChild(row);

    const maxFreq = Math.min(spectrum.sample_rate_hz / 2, 1000);
    const endIdx = spectrum.frequencies_hz.findIndex((f: number) => f > maxFreq);
    const n = endIdx > 0 ? endIdx : spectrum.frequencies_hz.length;
    const freqs = spectrum.frequencies_hz.slice(0, n);
    const mags = spectrum.magnitudes_db.slice(0, n);
    const color = AXIS_COLORS[spectrum.axis] || "#5b8def";

    const chart = echarts.init(chartDiv, null, { renderer: "canvas" });
    echartsInstances.push(chart);

    const markLines = [];

    if (filterConfig && !filterConfig.error) {
      const filters = [
        { val: filterConfig.gyro_lpf_hz, label: "LPF1", color: "#e8944a" },
        { val: filterConfig.gyro_lpf2_hz, label: "LPF2", color: "#e8944a" },
        { val: filterConfig.dterm_lpf_hz, label: "D-LPF", color: "#9b59b6" },
        { val: filterConfig.gyro_notch1_hz, label: "Notch1", color: "#e85454" },
        { val: filterConfig.gyro_notch2_hz, label: "Notch2", color: "#e85454" },
      ];
      for (const f of filters) {
        if (f.val && f.val > 0 && f.val <= maxFreq) {
          markLines.push({ xAxis: f.val, label: { formatter: f.label, position: "end", fontSize: 9, color: f.color }, lineStyle: { color: f.color, type: "dashed", width: 1 } });
        }
      }
    }

    // RPM harmonic lines (1x, 2x, 3x of average motor frequency)
    if (vibration.avg_motor_hz) {
      const rpm = vibration.avg_motor_hz;
      for (let h = 1; h <= 3; h++) {
        const freq = rpm * h;
        if (freq > 0 && freq <= maxFreq) {
          markLines.push({ xAxis: freq, label: { formatter: `${h}x RPM`, position: "start", fontSize: 9, color: "#4ec88c" }, lineStyle: { color: "#4ec88c", type: "dotted", width: 1 } });
        }
      }
    }

    const peakPoints = (spectrum.peaks || []).slice(0, 3).map((p: any) => ({
      coord: [p.frequency_hz, p.magnitude_db],
      label: { formatter: `${p.frequency_hz.toFixed(0)} Hz`, fontSize: 10, color: "#e0e0e6", position: "top" },
      symbol: "circle",
      symbolSize: 6,
      itemStyle: { color: p.classification === "MotorNoise" ? "#e8944a" : p.classification === "FrameResonance" ? "#e85454" : color },
    }));

    chart.setOption({
      grid: { left: 50, right: 20, top: 40, bottom: 50 },
      toolbox: {
        show: true,
        right: 10,
        top: 0,
        feature: {
          dataZoom: { title: { zoom: "Zoom", back: "Reset" }, iconStyle: { borderColor: "#8888a0" }, emphasis: { iconStyle: { borderColor: "#5b8def" } } },
          restore: { title: "Reset", iconStyle: { borderColor: "#8888a0" }, emphasis: { iconStyle: { borderColor: "#5b8def" } } },
          saveAsImage: { title: "Save", iconStyle: { borderColor: "#8888a0" }, emphasis: { iconStyle: { borderColor: "#5b8def" } } },
        },
        iconStyle: { borderColor: "#8888a0" },
      },
      xAxis: { type: "value", name: "Hz", nameLocation: "center", nameGap: 25, min: 0, max: maxFreq, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
      yAxis: { type: "value", name: "dB", nameLocation: "center", nameGap: 35, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
      tooltip: { trigger: "axis", backgroundColor: "#1a1d27", borderColor: "#2a2d3a", textStyle: { color: "#e0e0e6", fontFamily: "JetBrains Mono, monospace", fontSize: 11 } },
      dataZoom: [
        { type: "inside", xAxisIndex: 0 },
        { type: "slider", xAxisIndex: 0, height: 18, bottom: 2, borderColor: "#2a2d3a", fillerColor: "rgba(91,141,239,0.15)", handleStyle: { color: "#5b8def" }, textStyle: { color: "#8888a0", fontSize: 9 } },
      ],
      series: [{
        type: "line",
        data: freqs.map((f: number, i: number) => [f, mags[i]]),
        smooth: false,
        symbol: "none",
        lineStyle: { color, width: 1.5 },
        areaStyle: { color: color + "18" },
        markLine: markLines.length > 0 ? { data: markLines, symbol: "none", silent: true } : undefined,
        markPoint: peakPoints.length > 0 ? { data: peakPoints } : undefined,
      }],
    });

    if (spectrum.peaks && spectrum.peaks.length > 0) {
      const peaks = document.createElement("div");
      peaks.className = "peaks-list";
      for (const p of spectrum.peaks) {
        const badge = document.createElement("span");
        badge.className = "peak-badge";
        let inner = `<span class="freq">${p.frequency_hz.toFixed(0)} Hz</span> (${p.magnitude_db.toFixed(1)} dB)`;
        if (p.classification) {
          const cls = p.classification === "MotorNoise" ? "Motor" : p.classification === "FrameResonance" ? "Frame" : "";
          if (cls) inner += `<span class="class">${cls}</span>`;
        }
        badge.innerHTML = inner;
        peaks.appendChild(badge);
      }
      row.appendChild(peaks);
    }
  }
}

function renderThrottleBandsEcharts(vibration: VibrationAnalysis | null): void {
  const container = $("#throttle-plots");
  container.innerHTML = "";

  if (!vibration || !vibration.throttle_bands || vibration.throttle_bands.length === 0) {
    container.innerHTML = '<p class="hint">No throttle band data.</p>';
    return;
  }

  for (const band of vibration.throttle_bands) {
    const section = document.createElement("div");
    section.className = "throttle-band";

    const title = document.createElement("h3");
    title.textContent = `Throttle ${band.label}`;
    section.appendChild(title);

    const meta = document.createElement("div");
    meta.className = "band-meta";
    const rpmInfo = band.avg_motor_hz ? ` — avg ${(band.avg_motor_hz * 60).toFixed(0)} RPM (${band.avg_motor_hz.toFixed(0)} Hz)` : "";
    meta.textContent = `${band.frame_count.toLocaleString()} frames${rpmInfo}`;
    section.appendChild(meta);

    if (band.spectra && band.spectra.length > 0) {
      const chartDiv = document.createElement("div");
      chartDiv.style.width = "100%";
      chartDiv.style.height = "200px";
      section.appendChild(chartDiv);
      container.appendChild(section);

      const maxFreq = Math.min(band.spectra[0].sample_rate_hz / 2, 1000);
      const endIdx = band.spectra[0].frequencies_hz.findIndex((f: number) => f > maxFreq);
      const n = endIdx > 0 ? endIdx : band.spectra[0].frequencies_hz.length;

      const chart = echarts.init(chartDiv, null, { renderer: "canvas" });
      echartsInstances.push(chart);

      // RPM harmonic markLines for this throttle band
      const rpmMarkLines: any[] = [];
      if (band.avg_motor_hz) {
        for (let h = 1; h <= 3; h++) {
          const freq = band.avg_motor_hz * h;
          if (freq > 0 && freq <= maxFreq) {
            rpmMarkLines.push({ xAxis: freq, label: { formatter: `${h}x`, position: "start", fontSize: 8, color: "#4ec88c" }, lineStyle: { color: "#4ec88c", type: "dotted", width: 1 } });
          }
        }
      }

      const series = [];
      for (let si = 0; si < band.spectra.length; si++) {
        const s = band.spectra[si];
        const color = AXIS_COLORS[s.axis] || "#5b8def";
        const entry: any = {
          type: "line",
          name: s.axis,
          data: s.frequencies_hz.slice(0, n).map((f: number, i: number) => [f, s.magnitudes_db[i]]),
          smooth: false,
          symbol: "none",
          lineStyle: { color, width: 1.5 },
        };
        // Attach RPM lines to the first series
        if (si === 0 && rpmMarkLines.length > 0) {
          entry.markLine = { data: rpmMarkLines, symbol: "none", silent: true };
        }
        series.push(entry);
      }

      chart.setOption({
        grid: { left: 50, right: 20, top: 30, bottom: 40 },
        legend: { show: true, textStyle: { color: "#8888a0", fontSize: 10 }, top: 0 },
        toolbox: {
          show: true,
          right: 10,
          top: 0,
          feature: {
            dataZoom: { title: { zoom: "Zoom", back: "Reset" }, iconStyle: { borderColor: "#8888a0" } },
            restore: { title: "Reset", iconStyle: { borderColor: "#8888a0" } },
          },
          iconStyle: { borderColor: "#8888a0" },
        },
        xAxis: { type: "value", name: "Hz", nameLocation: "center", nameGap: 25, min: 0, max: maxFreq, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
        yAxis: { type: "value", name: "dB", nameLocation: "center", nameGap: 35, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
        tooltip: { trigger: "axis", backgroundColor: "#1a1d27", borderColor: "#2a2d3a", textStyle: { color: "#e0e0e6", fontFamily: "JetBrains Mono, monospace", fontSize: 11 } },
        dataZoom: [{ type: "inside", xAxisIndex: 0 }],
        series,
      });
    } else {
      container.appendChild(section);
    }
  }
}

function renderDiagnostics(diagnostics: Diagnostic[]): void {
  const list = $("#diagnostics-list");
  if (!diagnostics || diagnostics.length === 0) {
    list.innerHTML = '<p class="hint">No issues detected.</p>';
    return;
  }

  // Sort by severity: Problem first, then Warning, then Info
  const order: Record<string, number> = { Problem: 0, Warning: 1, Info: 2 };
  const sorted = [...diagnostics].sort(
    (a, b) => (order[a.severity] ?? 3) - (order[b.severity] ?? 3)
  );

  list.innerHTML = sorted
    .map((d) =>
      `<div class="diagnostic ${d.severity}">
        <div class="diag-header">
          <span class="severity">${d.severity}</span>
          <span class="category">${d.category}</span>
        </div>
        <div class="message">${escHtml(d.message)}</div>
        <div class="detail">${escHtml(d.detail)}</div>
      </div>`
    )
    .join("");
}

function renderSpectrogram(ref: SessionRef): void {
  const container = $("#spectrogram-plots");
  container.innerHTML = "";

  const json = get_spectrogram(ref.fileId, ref.sessionIdx, "roll,pitch,yaw");
  const data = JSON.parse(json);
  if (data.error || !data.axes || data.axes.length === 0) {
    container.innerHTML = '<p class="hint">No spectrogram data available.</p>';
    return;
  }

  for (const axis of data.axes) {
    const row = document.createElement("div");
    row.className = "spectrogram-row";

    const title = document.createElement("h3");
    title.textContent = axis.axis;
    row.appendChild(title);

    const canvas = document.createElement("canvas");
    const width = chartWidth();
    const height = 160;
    canvas.width = width;
    canvas.height = height;
    canvas.style.width = width + "px";
    canvas.style.height = height + "px";
    row.appendChild(canvas);

    drawSpectrogram(canvas, axis);

    const legend = document.createElement("div");
    legend.className = "spectrogram-legend";
    legend.innerHTML = `<span>0s</span><span>${axis.time_s[axis.time_s.length - 1]?.toFixed(1) || 0}s</span>`;
    row.appendChild(legend);

    container.appendChild(row);
  }
}

function drawSpectrogram(canvas: HTMLCanvasElement, axis: any): void {
  const ctx = canvas.getContext("2d")!;
  const { width, height } = canvas;
  const nTime = axis.time_s.length;
  const nFreq = axis.frequencies_hz.length;

  if (nTime === 0 || nFreq === 0) return;

  let dbMin = Infinity;
  let dbMax = -Infinity;
  for (const row of axis.magnitudes_db) {
    for (const v of row) {
      if (v > -120) {
        if (v < dbMin) dbMin = v;
        if (v > dbMax) dbMax = v;
      }
    }
  }

  const dbRange = dbMax - dbMin || 1;
  const img = ctx.createImageData(nTime, nFreq);

  for (let t = 0; t < nTime; t++) {
    const row = axis.magnitudes_db[t];
    for (let f = 0; f < nFreq; f++) {
      const val = (row[f] - dbMin) / dbRange;
      const clamped = Math.max(0, Math.min(1, val));
      const [r, g, b] = heatColor(clamped);
      const yFlipped = nFreq - 1 - f;
      const idx = (yFlipped * nTime + t) * 4;
      img.data[idx] = r;
      img.data[idx + 1] = g;
      img.data[idx + 2] = b;
      img.data[idx + 3] = 255;
    }
  }

  const tmpCanvas = document.createElement("canvas");
  tmpCanvas.width = nTime;
  tmpCanvas.height = nFreq;
  tmpCanvas.getContext("2d")!.putImageData(img, 0, 0);

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tmpCanvas, 0, 0, width, height);

  ctx.fillStyle = "#e0e0e6";
  ctx.font = "10px JetBrains Mono, monospace";
  ctx.textAlign = "right";
  ctx.fillText(`${axis.frequencies_hz[nFreq - 1]?.toFixed(0) || ""} Hz`, width - 4, 12);
  ctx.fillText("0 Hz", width - 4, height - 4);
}


function createMultiAxisPlot(container: HTMLElement, spectra: Spectrum[]): void {
  if (spectra.length === 0) return;

  const maxFreq = Math.min(spectra[0].sample_rate_hz / 2, 1000);
  const endIdx = spectra[0].frequencies_hz.findIndex((f: number) => f > maxFreq);
  const n = endIdx > 0 ? endIdx : spectra[0].frequencies_hz.length;

  const xData = spectra[0].frequencies_hz.slice(0, n);
  const datasets = [xData];
  const series = [{}];

  for (const s of spectra) {
    datasets.push(s.magnitudes_db.slice(0, n));
    series.push({
      label: s.axis,
      stroke: AXIS_COLORS[s.axis] || "#5b8def",
      width: 1.5,
    });
  }

  const width = chartWidth();

  const opts = {
    width,
    height: 180,
    cursor: { show: true },
    scales: { x: { time: false } },
    axes: [
      { label: "Hz", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
      { label: "dB", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
    ],
    series,
  };

  new uPlot(opts, datasets, container);
}

function renderAccel(vibration: VibrationAnalysis | null): void {
  const panel = $("#accel-panel");
  const infoDiv = $("#accel-info");
  const plotsDiv = $("#accel-plots");

  if (!vibration || !vibration.accel) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");
  const accel = vibration.accel;

  const axes = ["X", "Y", "Z"];
  infoDiv.innerHTML = `<div class="accel-rms">` +
    accel.rms.map((v: number, i: number) =>
      `<div class="rms-card">
        <div class="axis">${axes[i]}</div>
        <div class="rms-value">${v.toFixed(1)}</div>
      </div>`
    ).join("") +
    `</div>`;

  plotsDiv.innerHTML = "";
  if (accel.spectra && accel.spectra.length > 0) {
    const plotDiv = document.createElement("div");
    plotsDiv.appendChild(plotDiv);
    createMultiAxisPlot(plotDiv, accel.spectra);
  }
}

function renderPropwash(vibration: VibrationAnalysis | null): void {
  const panel = $("#propwash-panel");
  const infoDiv = $("#propwash-info");
  const plotsDiv = $("#propwash-plots");

  if (!vibration || !vibration.propwash) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");
  const pw = vibration.propwash;

  let infoHtml = `<p class="propwash-summary">${pw.chop_count} throttle chop${pw.chop_count === 1 ? "" : "s"} analyzed`;
  if (pw.dominant_frequency_hz != null && pw.dominant_magnitude_db != null) {
    infoHtml += ` — dominant propwash frequency: <strong>${pw.dominant_frequency_hz.toFixed(0)} Hz</strong> (${pw.dominant_magnitude_db.toFixed(1)} dB)`;
  } else {
    infoHtml += ` — no dominant frequency in 20-100 Hz range`;
  }
  infoHtml += `</p>`;
  infoDiv.innerHTML = infoHtml;

  plotsDiv.innerHTML = "";
  if (pw.spectra && pw.spectra.length > 0) {
    const plotDiv = document.createElement("div");
    plotsDiv.appendChild(plotDiv);
    createMultiAxisPlot(plotDiv, pw.spectra);
  }
}

function renderTrend(): void {
  const container = $("#trend-table");

  if (workspace.length === 0) {
    container.innerHTML = '<p class="hint">Load files to see trends across sessions.</p>';
    return;
  }

  const json = get_trend();
  const points: TrendPoint[] = JSON.parse(json);

  if (points.length === 0) {
    container.innerHTML = '<p class="hint">No sessions with data found.</p>';
    return;
  }

  const fmt = (v: number | null, decimals = 1) =>
    v != null ? v.toFixed(decimals) : "\u2014";

  const fmtNf = (nf: [number, number, number] | null) =>
    nf != null ? ((nf[0] + nf[1] + nf[2]) / 3).toFixed(0) : "\u2014";

  let html = `<table class="event-table"><thead><tr>
    <th>Session</th>
    <th>Duration</th>
    <th>Noise Floor</th>
    <th>Motor Bal.</th>
    <th>Rise Time</th>
    <th>Overshoot</th>
    <th>Events</th>
    <th>Warnings</th>
  </tr></thead><tbody>`;

  for (const pt of points) {
    const mbColor = pt.motor_balance_max_deviation != null && pt.motor_balance_max_deviation > 5
      ? ' style="color:var(--red)"' : "";
    const osColor = pt.step_response_overshoot != null && pt.step_response_overshoot > 25
      ? ' style="color:var(--red)"'
      : pt.step_response_overshoot != null && pt.step_response_overshoot > 15
      ? ' style="color:var(--yellow, #e8b84a)"' : "";
    html += `<tr>
      <td>${escHtml(pt.label)}</td>
      <td>${pt.duration_seconds.toFixed(1)}s</td>
      <td>${fmtNf(pt.noise_floor_db)} dB</td>
      <td${mbColor}>${fmt(pt.motor_balance_max_deviation)}%</td>
      <td>${fmt(pt.step_response_rise_ms)} ms</td>
      <td${osColor}>${fmt(pt.step_response_overshoot, 0)}%</td>
      <td>${pt.total_events}</td>
      <td>${pt.diagnostic_count}</td>
    </tr>`;
  }

  html += "</tbody></table>";
  container.innerHTML = html;
}

const RAW_PAGE_SIZE = 200;
const RAW_DEFAULT_FIELDS = ["time", "gyro[roll]", "gyro[pitch]", "gyro[yaw]", "motor[0]", "motor[1]", "motor[2]", "motor[3]", "rc[throttle]"];

function renderRawData(ref: SessionRef): void {
  const select = $("#raw-field-select") as HTMLSelectElement;
  select.innerHTML = "";

  const defaultFields = RAW_DEFAULT_FIELDS;
  for (const name of defaultFields) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    opt.selected = true;
    select.appendChild(opt);
  }

  select.onchange = () => loadRawPage(ref, 0);
  loadRawPage(ref, 0);
}

function loadRawPage(ref: SessionRef, start: number): void {
  const select = $("#raw-field-select") as HTMLSelectElement;
  const selected = Array.from(select.selectedOptions).map((o) => o.value);
  if (selected.length === 0) return;

  const json = get_raw_frames(ref.fileId, ref.sessionIdx, start, RAW_PAGE_SIZE, selected.join(","));
  const data = JSON.parse(json);
  if (data.error) return;

  $("#raw-frame-info").textContent = `Frames ${data.start}–${data.start + data.frames.length} of ${data.total.toLocaleString()}`;

  const wrap = $("#raw-table-wrap");
  let html = `<table class="raw-table"><thead><tr><th>#</th>`;
  for (const name of data.field_names) {
    html += `<th>${name}</th>`;
  }
  html += "</tr></thead><tbody>";

  for (let i = 0; i < data.frames.length; i++) {
    html += `<tr><td>${data.start + i}</td>`;
    for (const val of data.frames[i]) {
      html += `<td>${val}</td>`;
    }
    html += "</tr>";
  }
  html += "</tbody></table>";

  if (data.total > RAW_PAGE_SIZE) {
    html += `<div style="display:flex;gap:0.5rem;margin-top:0.5rem;justify-content:center;">`;
    if (start > 0) {
      html += `<button class="ts-tab" onclick="window._rawPage(${ref.fileId},${ref.sessionIdx},${Math.max(0, start - RAW_PAGE_SIZE)})">Prev</button>`;
    }
    if (start + RAW_PAGE_SIZE < data.total) {
      html += `<button class="ts-tab" onclick="window._rawPage(${ref.fileId},${ref.sessionIdx},${start + RAW_PAGE_SIZE})">Next</button>`;
    }
    html += "</div>";
  }

  wrap.innerHTML = html;
}

window._rawPage = (fileId: number, sessionIdx: number, start: number) => {
  loadRawPage({ fileId, sessionIdx, label: "" }, start);
};

function renderEvents(events: FlightEvent[]): void {
  const list = $("#events-list");
  if (!events || events.length === 0) {
    list.innerHTML = '<p class="hint">No events detected.</p>';
    return;
  }

  // Collect unique event types for filter pills
  const types = [...new Set(events.map((e) => e.kind.type))];
  const activeTypes = new Set(types);

  const container = document.createElement("div");

  // Filter pills
  const filters = document.createElement("div");
  filters.className = "event-filters";
  for (const type of types) {
    const pill = document.createElement("button");
    pill.className = `event-pill active ${type}`;
    pill.textContent = `${formatEventType(type)} (${events.filter((e) => e.kind.type === type).length})`;
    pill.dataset.type = type;
    pill.addEventListener("click", () => {
      if (activeTypes.has(type)) {
        activeTypes.delete(type);
        pill.classList.remove("active");
      } else {
        activeTypes.add(type);
        pill.classList.add("active");
      }
      renderTable();
    });
    filters.appendChild(pill);
  }
  container.appendChild(filters);

  // Table container
  const tableWrap = document.createElement("div");
  container.appendChild(tableWrap);

  const MAX_DISPLAY = 200;

  function renderTable() {
    const filtered = events.filter((e) => activeTypes.has(e.kind.type));
    const shown = filtered.slice(0, MAX_DISPLAY);

    let html = `<table class="event-table">
      <thead><tr><th>Time</th><th>Type</th><th>Details</th></tr></thead>
      <tbody>`;

    for (const e of shown) {
      html += `<tr class="event-row" data-time="${e.time_seconds}">
        <td>${e.time_seconds.toFixed(2)}s</td>
        <td><span class="event-type ${e.kind.type}">${formatEventType(e.kind.type)}</span></td>
        <td>${formatEventDetails(e.kind)}</td>
      </tr>`;
    }

    html += "</tbody></table>";

    if (filtered.length > MAX_DISPLAY) {
      html += `<p class="events-truncated">Showing ${MAX_DISPLAY} of ${filtered.length} events</p>`;
    }

    tableWrap.innerHTML = html;

    // Wire up click-to-navigate on each row
    tableWrap.querySelectorAll(".event-row").forEach((row) => {
      row.addEventListener("click", () => {
        const time = parseFloat((row as HTMLElement).dataset.time!);
        navigateToTime(time);
      });
    });
  }

  renderTable();
  list.innerHTML = "";
  list.appendChild(container);
}


function escHtml(str: string): string {
  const div = document.createElement("div");
  div.textContent = str;
  return div.innerHTML;
}

let resizeTimer: ReturnType<typeof setTimeout> | null = null;
window.addEventListener("resize", () => {
  if (resizeTimer) clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => {
    for (const inst of echartsInstances) {
      inst.resize();
    }
    if (activeSession && !$("#results").classList.contains("hidden")) {
      const view = $(".view-tab.active")?.dataset.view;
      if (view === "timeline") {
        renderedViews.delete(`timeline-${activeSession.fileId}-${activeSession.sessionIdx}`);
        renderViewIfNeeded("timeline");
      }
    }
  }, 250);
});

boot().catch((err) => {
  console.error("Failed to initialize WASM:", err);
  $(".drop-zone-content p").textContent = "Failed to load analyzer. Check console.";
});
