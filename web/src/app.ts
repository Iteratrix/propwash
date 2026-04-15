import init, { analyze, get_timeseries, get_spectrogram, get_filter_config, get_raw_frames } from "../pkg/propwash_web.js";
import type { AnalysisResult, SessionResult, FlightEvent, Diagnostic, VibrationAnalysis, Spectrum, FilterConfig, TimeseriesResponse, SpectrogramResponse, RawFramesResponse, EventKind } from "./types.js";
import { formatEventType, formatEventDetails, eventColor, heatColor, bucketDiagnostics, classifyDelta } from "./format.js";

declare const echarts: any;
declare const uPlot: any;
declare const __WASM_URL__: string;

const $ = (sel: string): HTMLElement => document.querySelector(sel) as HTMLElement;
const $$ = (sel: string): NodeListOf<HTMLElement> => document.querySelectorAll(sel) as NodeListOf<HTMLElement>;

declare global {
  interface Window {
    _rawPage: (sessionIdx: number, start: number) => void;
  }
}

function chartWidth(): number {
  const padding = window.innerWidth <= 640 ? 24 : 64;
  return Math.min($("main").clientWidth - padding, 880);
}

let wasmReady = false;
let result: AnalysisResult | null = null;

const AXIS_COLORS: Record<string, string> = {
  roll:  "#5b8def",
  pitch: "#4ec88c",
  yaw:   "#e8b84a",
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
  pids: {
    label: "PIDs (Roll)",
    fields: ["pid_p[roll]", "pid_i[roll]", "pid_d[roll]"],
    names:  ["P", "I", "D"],
    colors: ["#5b8def", "#4ec88c", "#e85454"],
  },
  pidsPitch: {
    label: "PIDs (Pitch)",
    fields: ["pid_p[pitch]", "pid_i[pitch]", "pid_d[pitch]"],
    names:  ["P", "I", "D"],
    colors: ["#5b8def", "#4ec88c", "#e85454"],
  },
  pidsYaw: {
    label: "PIDs (Yaw)",
    fields: ["pid_p[yaw]", "pid_i[yaw]", "pid_d[yaw]"],
    names:  ["P", "I", "D"],
    colors: ["#5b8def", "#4ec88c", "#e85454"],
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
};

const TS_MAX_POINTS = 4000;
let tsPlots: any[] = [];
let tsSync: any = null;
let filterConfig: FilterConfig | null = null;

let compareResult: AnalysisResult | null = null;

let renderedViews = new Set<string>();

async function boot() {
  await init({ module_or_path: __WASM_URL__ });
  wasmReady = true;
  setupDropZone();
  setupCompareDropZone();
  setupTimeseriesControls();
  setupViewTabs();
  $("#reset-btn").addEventListener("click", reset);
  $("#compare-btn").addEventListener("click", startCompare);
  $("#compare-reset-btn").addEventListener("click", reset);
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
  result = null;
  compareResult = null;
  disposeEcharts();
  renderedViews.clear();
  $("#results").classList.add("hidden");
  $("#compare-drop").classList.add("hidden");
  $("#compare-results").classList.add("hidden");
  $("#drop-zone").classList.remove("hidden");
  ($("#file-input") as HTMLInputElement).value = "";
  ($("#compare-file-input") as HTMLInputElement).value = "";
}

function startCompare() {
  $("#results").classList.add("hidden");
  $("#compare-drop").classList.remove("hidden");
}

function setupCompareDropZone(): void {
  const zone = $("#compare-drop");
  const input = $("#compare-file-input") as HTMLInputElement;

  zone.addEventListener("click", () => input.click());
  input.addEventListener("change", () => {
    if (input.files && input.files.length > 0) handleCompareFile(input.files[0]);
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
    if (dt && dt.files.length > 0) handleCompareFile(dt.files[0]);
  });
}

async function handleCompareFile(file: File): Promise<void> {
  if (!wasmReady) return;

  $("#compare-drop").classList.add("hidden");
  $("#loading").classList.remove("hidden");

  await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));

  const buffer = await file.arrayBuffer();
  const data = new Uint8Array(buffer);
  const json = analyze(data);
  compareResult = JSON.parse(json);

  $("#loading").classList.add("hidden");

  if (compareResult!.sessions.length === 0) {
    $("#compare-drop").classList.remove("hidden");
    alert("No sessions found in comparison file.");
    return;
  }

  $("#compare-results").classList.remove("hidden");
  renderComparison();
}

function renderComparison(): void {
  const a = result!.sessions[activeSessionIdx] || result!.sessions[0];
  const b = compareResult!.sessions[0];

  renderComparisonSummary(a, b);
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

  const eventRows = [
    ["Events", a.analysis.summary.total_events, b.analysis.summary.total_events, "lower"],
    ["Desyncs", a.analysis.summary.desyncs, b.analysis.summary.desyncs, "lower"],
    ["Gyro Spikes", a.analysis.summary.gyro_spikes, b.analysis.summary.gyro_spikes, "lower"],
    ["Motor Sats", a.analysis.summary.motor_saturations, b.analysis.summary.motor_saturations, "lower"],
    ["Overshoots", a.analysis.summary.overshoots, b.analysis.summary.overshoots, "lower"],
  ];

  let html = `<table class="compare-table">
    <thead><tr><th>Metric</th><th>Flight A</th><th>Flight B</th><th>Delta</th></tr></thead><tbody>`;

  for (const [label, va, vb] of rows) {
    html += `<tr><td>${label}</td><td>${va}</td><td>${vb}</td><td></td></tr>`;
  }

  for (const [label, va, vb, better] of eventRows) {
    const diff = (vb as number) - (va as number);
    const cls = classifyDelta(diff, better as "lower" | "higher");
    const sign = diff > 0 ? "+" : "";
    html += `<tr><td>${label}</td><td>${va}</td><td>${vb}</td><td class="delta ${cls}">${sign}${diff}</td></tr>`;
  }

  html += "</tbody></table>";
  $("#compare-summary-table").innerHTML = html;
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

  const axes = ["roll", "pitch", "yaw"];
  for (const axisName of axes) {
    const specA = vibA.spectra.find((s: any) => s.axis === axisName);
    const specB = vibB.spectra.find((s: any) => s.axis === axisName);
    if (!specA || !specB) continue;

    const row = document.createElement("div");
    row.className = "spectrum-row";
    const title = document.createElement("h3");
    title.textContent = axisName.charAt(0).toUpperCase() + axisName.slice(1);
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
        { label: "Flight A", stroke: color, width: 1.5 },
        { label: "Flight B", stroke: color, width: 1.5, dash: [4, 4] },
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

  zone.addEventListener("click", () => input.click());
  input.addEventListener("change", () => {
    if (input.files && input.files.length > 0) handleFile(input.files[0]);
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
    if (dt && dt.files.length > 0) handleFile(dt.files[0]);
  });
}

async function handleFile(file: File): Promise<void> {
  if (!wasmReady) return;

  $("#drop-zone").classList.add("hidden");
  $("#loading").classList.remove("hidden");
  $("#results").classList.add("hidden");

  await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));

  const buffer = await file.arrayBuffer();
  const data = new Uint8Array(buffer);

  const json = analyze(data);
  result = JSON.parse(json);

  $("#loading").classList.add("hidden");

  if (result!.sessions.length === 0) {
    $("#drop-zone").classList.remove("hidden");
    alert("No sessions found in this file. " + (result!.warnings[0] || ""));
    return;
  }

  renderSessionTabs();
  $("#results").classList.remove("hidden");
  showSession(0);
}

function renderSessionTabs() {
  const nav = $("#session-tabs");
  nav.innerHTML = "";

  if (result!.sessions.length === 1) return;

  for (let i = 0; i < result!.sessions.length; i++) {
    const s = result!.sessions[i];
    const btn = document.createElement("button");
    btn.className = "session-tab" + (i === 0 ? " active" : "");
    btn.textContent = `Session ${s.index}`;
    btn.addEventListener("click", () => {
      $$(".session-tab").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      showSession(i);
    });
    nav.appendChild(btn);
  }
}

function showSession(idx: number): void {
  activeSessionIdx = idx;
  filterConfig = JSON.parse(get_filter_config(idx));
  renderedViews.clear();
  const activeView = $(".view-tab.active")?.dataset.view || "overview";
  renderViewIfNeeded(activeView);
}

function renderViewIfNeeded(view: string): void {
  const key = `${view}-${activeSessionIdx}`;
  if (renderedViews.has(key)) return;
  renderedViews.add(key);

  const s = result!.sessions[activeSessionIdx];
  switch (view) {
    case "overview":
      renderSummary(s);
      renderDiagnostics(s.analysis.diagnostics);
      renderEvents(s.analysis.events);
      break;
    case "timeline":
      renderTimeseries(activeSessionIdx, $(".ts-tab.active")?.dataset.group || "gyro");
      break;
    case "spectrum":
      renderSpectraEcharts(s.analysis.vibration);
      renderSpectrogram(activeSessionIdx);
      renderThrottleBandsEcharts(s.analysis.vibration);
      renderAccel(s.analysis.vibration);
      renderPropwash(s.analysis.vibration);
      break;
    case "raw":
      renderRawData(activeSessionIdx);
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
  const window = 1.5;
  const min = Math.max(0, seconds - window);
  const max = seconds + window;
  for (const p of tsPlots) {
    p.setScale("x", { min, max });
  }
}

function setupTimeseriesControls() {
  const tabs = $$("#ts-tabs .ts-tab");
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      renderedViews.delete(`timeline-${activeSessionIdx}`);
      renderTimeseries(activeSessionIdx, tab.dataset.group!);
    });
  });
  $("#ts-reset-zoom").addEventListener("click", () => {
    const activeGroup = $(".ts-tab.active")?.dataset.group || "gyro";
    renderedViews.delete(`timeline-${activeSessionIdx}`);
    renderTimeseries(activeSessionIdx, activeGroup);
  });
}

function renderTimeseries(sessionIdx: number, group: string): void {
  const container = $("#ts-charts");
  container.innerHTML = "";
  tsPlots = [];

  const g = FIELD_GROUPS[group as keyof typeof FIELD_GROUPS];
  if (!g) return;

  const allFields = g.fields.join(",");
  const json = get_timeseries(sessionIdx, TS_MAX_POINTS, allFields);
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

  const events = result!.sessions[sessionIdx]?.analysis?.events || [];

  const width = chartWidth();

  const datasets = [time];
  const series = [{}];
  let hasData = false;

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
    plugins: [eventMarkersPlugin(events)],
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

  grid.innerHTML = cards
    .map(([label, value]) =>
      `<div class="summary-card">
        <div class="label">${label}</div>
        <div class="value" title="${value}">${value}</div>
      </div>`
    )
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

function disposeEcharts() {
  for (const inst of echartsInstances) {
    inst.dispose();
  }
  echartsInstances.length = 0;
}

function echartsTheme() {
  return {
    backgroundColor: "transparent",
    textStyle: { fontFamily: "JetBrains Mono, Fira Code, monospace", color: "#8888a0" },
    axisLine: { lineStyle: { color: "#2a2d3a" } },
    splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } },
  };
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
    title.textContent = spectrum.axis.charAt(0).toUpperCase() + spectrum.axis.slice(1);
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
    meta.textContent = `${band.frame_count.toLocaleString()} frames`;
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

      const series = [];
      for (const s of band.spectra) {
        const color = AXIS_COLORS[s.axis] || "#5b8def";
        series.push({
          type: "line",
          name: s.axis,
          data: s.frequencies_hz.slice(0, n).map((f: number, i: number) => [f, s.magnitudes_db[i]]),
          smooth: false,
          symbol: "none",
          lineStyle: { color, width: 1.5 },
        });
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

function renderSpectra(vibration: VibrationAnalysis | null): void {
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
    title.textContent = spectrum.axis.charAt(0).toUpperCase() + spectrum.axis.slice(1);
    row.appendChild(title);

    const plotDiv = document.createElement("div");
    row.appendChild(plotDiv);
    container.appendChild(row);

    createSpectrumPlot(plotDiv, spectrum);

    if (spectrum.peaks && spectrum.peaks.length > 0) {
      const peaks = document.createElement("div");
      peaks.className = "peaks-list";
      for (const p of spectrum.peaks) {
        const badge = document.createElement("span");
        badge.className = "peak-badge";
        let inner = `<span class="freq">${p.frequency_hz.toFixed(0)} Hz</span> (${p.magnitude_db.toFixed(1)} dB)`;
        if (p.classification) {
          const cls = p.classification === "MotorNoise" ? "Motor" :
                      p.classification === "FrameResonance" ? "Frame" : "";
          if (cls) inner += `<span class="class">${cls}</span>`;
        }
        badge.innerHTML = inner;
        peaks.appendChild(badge);
      }
      row.appendChild(peaks);
    }
  }
}

function peakMarkersPlugin(peaks: any[], maxFreq: number): any {
  const filtered = (peaks || [])
    .filter((p) => p.frequency_hz <= maxFreq)
    .slice(0, 3);
  return {
    hooks: {
      draw: [
        (u: any) => {
          const ctx = u.ctx;
          const placed: number[] = [];
          const MIN_GAP = 40;

          for (const peak of filtered) {
            const x = u.valToPos(peak.frequency_hz, "x", true);
            const yTop = u.bbox.top;
            const yBot = u.bbox.top + u.bbox.height;

            const color = peak.classification === "MotorNoise" ? "#e8944a" :
                          peak.classification === "FrameResonance" ? "#e85454" : "#ffffff44";

            ctx.save();
            ctx.strokeStyle = color;
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.moveTo(x, yTop);
            ctx.lineTo(x, yBot);
            ctx.stroke();
            ctx.restore();

            const tooClose = placed.some((px) => Math.abs(px - x) < MIN_GAP);
            if (!tooClose) {
              ctx.save();
              ctx.fillStyle = "#e0e0e6";
              ctx.font = "10px JetBrains Mono, monospace";
              ctx.textAlign = "center";
              ctx.fillText(`${peak.frequency_hz.toFixed(0)} Hz`, x, yTop - 4);
              ctx.restore();
              placed.push(x);
            }
          }
        },
      ],
    },
  };
}

function createSpectrumPlot(container: HTMLElement, spectrum: Spectrum): void {
  const freqs = spectrum.frequencies_hz;
  const mags = spectrum.magnitudes_db;

  const maxFreq = Math.min(spectrum.sample_rate_hz / 2, 1000);
  const endIdx = freqs.findIndex((f: number) => f > maxFreq);
  const n = endIdx > 0 ? endIdx : freqs.length;

  const xData = freqs.slice(0, n);
  const yData = mags.slice(0, n);

  const color = AXIS_COLORS[spectrum.axis] || "#5b8def";
  const width = chartWidth();

  const plugins = [peakMarkersPlugin(spectrum.peaks, maxFreq)];
  if (filterConfig && !filterConfig.error) {
    plugins.push(filterOverlayPlugin(maxFreq));
  }

  const opts = {
    width,
    height: 200,
    cursor: { show: true },
    scales: {
      x: { time: false },
      y: {},
    },
    axes: [
      { label: "Frequency (Hz)", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
      { label: "dB", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
    ],
    series: [
      {},
      { stroke: color, width: 1.5, fill: color + "18" },
    ],
    plugins,
  };

  new uPlot(opts, [xData, yData], container);
}

function filterOverlayPlugin(maxFreq: number): any {
  return {
    hooks: {
      draw: [
        (u: any) => {
          try {
          if (!filterConfig || filterConfig.error) return;
          const ctx = u.ctx;

          const lines = [
            { freq: filterConfig.gyro_lpf_hz, label: "LPF1", color: "#e8944a" },
            { freq: filterConfig.gyro_lpf2_hz, label: "LPF2", color: "#e8944a" },
            { freq: filterConfig.dterm_lpf_hz, label: "D-LPF", color: "#9b59b6" },
            { freq: filterConfig.gyro_notch1_hz, label: "Notch1", color: "#e85454" },
            { freq: filterConfig.gyro_notch2_hz, label: "Notch2", color: "#e85454" },
          ];

          for (const line of lines) {
            if (!line.freq || line.freq <= 0 || line.freq > maxFreq) continue;
            const x = u.valToPos(line.freq, "x", true);
            ctx.save();
            ctx.strokeStyle = line.color;
            ctx.lineWidth = 1.5;
            ctx.setLineDash([2, 3]);
            ctx.globalAlpha = 0.6;
            ctx.beginPath();
            ctx.moveTo(x, u.bbox.top);
            ctx.lineTo(x, u.bbox.top + u.bbox.height);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.globalAlpha = 0.8;
            ctx.fillStyle = line.color;
            ctx.font = "9px JetBrains Mono, monospace";
            ctx.textAlign = "center";
            ctx.fillText(line.label, x, u.bbox.top + u.bbox.height + 12);
            ctx.restore();
          }

          if (filterConfig.dyn_notch_min_hz && filterConfig.dyn_notch_max_hz) {
            const xMin = u.valToPos(filterConfig.dyn_notch_min_hz, "x", true);
            const xMax = u.valToPos(filterConfig.dyn_notch_max_hz, "x", true);
            ctx.save();
            ctx.fillStyle = "rgba(232, 84, 84, 0.06)";
            ctx.fillRect(xMin, u.bbox.top, xMax - xMin, u.bbox.height);
            ctx.strokeStyle = "#e85454";
            ctx.lineWidth = 1;
            ctx.setLineDash([2, 3]);
            ctx.globalAlpha = 0.4;
            ctx.strokeRect(xMin, u.bbox.top, xMax - xMin, u.bbox.height);
            ctx.setLineDash([]);
            ctx.globalAlpha = 0.7;
            ctx.fillStyle = "#e85454";
            ctx.font = "9px JetBrains Mono, monospace";
            ctx.textAlign = "center";
            ctx.fillText("Dyn Notch", (xMin + xMax) / 2, u.bbox.top + 12);
            ctx.restore();
          }
          } catch (e) { /* filter overlay failed, chart still renders */ }
        },
      ],
    },
  };
}

function renderSpectrogram(sessionIdx: number): void {
  const container = $("#spectrogram-plots");
  container.innerHTML = "";

  const json = get_spectrogram(sessionIdx, "roll,pitch,yaw");
  const data = JSON.parse(json);
  if (data.error || !data.axes || data.axes.length === 0) {
    container.innerHTML = '<p class="hint">No spectrogram data available.</p>';
    return;
  }

  for (const axis of data.axes) {
    const row = document.createElement("div");
    row.className = "spectrogram-row";

    const title = document.createElement("h3");
    title.textContent = axis.axis.charAt(0).toUpperCase() + axis.axis.slice(1);
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


function renderThrottleBands(vibration: VibrationAnalysis | null): void {
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
    meta.textContent = `${band.frame_count.toLocaleString()} frames`;
    section.appendChild(meta);

    if (band.spectra && band.spectra.length > 0) {
      const plotDiv = document.createElement("div");
      section.appendChild(plotDiv);
      createMultiAxisPlot(plotDiv, band.spectra);
    }

    container.appendChild(section);
  }
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

const RAW_PAGE_SIZE = 200;
const RAW_DEFAULT_FIELDS = ["time", "gyro[roll]", "gyro[pitch]", "gyro[yaw]", "motor[0]", "motor[1]", "motor[2]", "motor[3]", "rc[throttle]"];

function renderRawData(sessionIdx: number): void {
  const s = result!.sessions[sessionIdx];
  const allFields = s.analysis.summary ? result!.sessions[sessionIdx] : null;

  const select = $("#raw-field-select") as HTMLSelectElement;
  select.innerHTML = "";

  const fieldNames = get_timeseries(sessionIdx, 1, "time");
  const ts = JSON.parse(fieldNames);
  const unified = result!.sessions[sessionIdx];

  const defaultFields = RAW_DEFAULT_FIELDS;
  const available = defaultFields;

  for (const name of available) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    opt.selected = true;
    select.appendChild(opt);
  }

  select.onchange = () => loadRawPage(sessionIdx, 0);
  loadRawPage(sessionIdx, 0);
}

function loadRawPage(sessionIdx: number, start: number): void {
  const select = $("#raw-field-select") as HTMLSelectElement;
  const selected = Array.from(select.selectedOptions).map((o) => o.value);
  if (selected.length === 0) return;

  const json = get_raw_frames(sessionIdx, start, RAW_PAGE_SIZE, selected.join(","));
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
      html += `<button class="ts-tab" onclick="window._rawPage(${sessionIdx},${Math.max(0, start - RAW_PAGE_SIZE)})">Prev</button>`;
    }
    if (start + RAW_PAGE_SIZE < data.total) {
      html += `<button class="ts-tab" onclick="window._rawPage(${sessionIdx},${start + RAW_PAGE_SIZE})">Next</button>`;
    }
    html += "</div>";
  }

  wrap.innerHTML = html;
}

window._rawPage = loadRawPage;

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

let activeSessionIdx = 0;
let resizeTimer: ReturnType<typeof setTimeout> | null = null;
window.addEventListener("resize", () => {
  if (resizeTimer) clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => {
    for (const inst of echartsInstances) {
      inst.resize();
    }
    if (result && !$("#results").classList.contains("hidden")) {
      const view = $(".view-tab.active")?.dataset.view;
      if (view === "timeline") {
        renderedViews.delete(`timeline-${activeSessionIdx}`);
        renderViewIfNeeded("timeline");
      }
    }
  }, 250);
});

boot().catch((err) => {
  console.error("Failed to initialize WASM:", err);
  $(".drop-zone-content p").textContent = "Failed to load analyzer. Check console.";
});
