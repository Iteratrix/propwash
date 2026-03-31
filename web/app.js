import init, { analyze, get_timeseries, get_spectrogram, get_filter_config } from "./pkg/propwash_web.js";

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

function chartWidth() {
  return Math.min($("main").clientWidth - 64, 880);
}

let wasmReady = false;
let result = null;

const AXIS_COLORS = {
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
    fields: ["gyroADC[0]", "gyroADC[1]", "gyroADC[2]"],
    names:  ["Roll", "Pitch", "Yaw"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
  },
  motors: {
    label: "Motors",
    fields: ["motor[0]", "motor[1]", "motor[2]", "motor[3]"],
    names:  ["Motor 1", "Motor 2", "Motor 3", "Motor 4"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e85454"],
  },
  rc: {
    label: "RC Commands",
    fields: ["rcCommand[0]", "rcCommand[1]", "rcCommand[2]", "rcCommand[3]"],
    names:  ["Roll", "Pitch", "Yaw", "Throttle"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e8944a"],
  },
  pids: {
    label: "PID Sum",
    fields: ["axisP[0]", "axisP[1]", "axisP[2]"],
    names:  ["P Roll", "P Pitch", "P Yaw"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
  },
};

const TS_MAX_POINTS = 4000;
let tsPlots = [];
let tsSync = null;
let filterConfig = null;

async function boot() {
  await init();
  wasmReady = true;
  setupDropZone();
  setupTimeseriesControls();
  $("#reset-btn").addEventListener("click", reset);
}

function reset() {
  result = null;
  $("#results").classList.add("hidden");
  $("#drop-zone").classList.remove("hidden");
  $("#file-input").value = "";
}

function setupDropZone() {
  const zone = $("#drop-zone");
  const input = $("#file-input");

  zone.addEventListener("click", () => input.click());
  input.addEventListener("change", (e) => {
    if (e.target.files.length > 0) handleFile(e.target.files[0]);
  });

  zone.addEventListener("dragover", (e) => {
    e.preventDefault();
    zone.classList.add("drag-over");
  });
  zone.addEventListener("dragleave", () => zone.classList.remove("drag-over"));
  zone.addEventListener("drop", (e) => {
    e.preventDefault();
    zone.classList.remove("drag-over");
    if (e.dataTransfer.files.length > 0) handleFile(e.dataTransfer.files[0]);
  });
}

async function handleFile(file) {
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

  if (result.sessions.length === 0) {
    $("#drop-zone").classList.remove("hidden");
    alert("No sessions found in this file. " + (result.warnings[0] || ""));
    return;
  }

  renderSessionTabs();
  $("#results").classList.remove("hidden");
  showSession(0);
}

function renderSessionTabs() {
  const nav = $("#session-tabs");
  nav.innerHTML = "";

  if (result.sessions.length === 1) return;

  for (let i = 0; i < result.sessions.length; i++) {
    const s = result.sessions[i];
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

function showSession(idx) {
  activeSessionIdx = idx;
  const s = result.sessions[idx];
  renderSummary(s);
  const activeGroup = $(".ts-tab.active")?.dataset.group || "gyro";
  renderTimeseries(idx, activeGroup);
  filterConfig = JSON.parse(get_filter_config(idx));
  renderDiagnostics(s.analysis.diagnostics);
  renderSpectra(s.analysis.vibration);
  renderSpectrogram(idx);
  renderThrottleBands(s.analysis.vibration);
  renderAccel(s.analysis.vibration);
  renderEvents(s.analysis.events);
}

function setupTimeseriesControls() {
  const tabs = $$("#ts-tabs .ts-tab");
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      renderTimeseries(activeSessionIdx, tab.dataset.group);
    });
  });
  $("#ts-reset-zoom").addEventListener("click", () => {
    const activeGroup = $(".ts-tab.active")?.dataset.group || "gyro";
    renderTimeseries(activeSessionIdx, activeGroup);
  });
}

function renderTimeseries(sessionIdx, group) {
  const container = $("#ts-charts");
  container.innerHTML = "";
  tsPlots = [];

  const g = FIELD_GROUPS[group];
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

  const events = result.sessions[sessionIdx]?.analysis?.events || [];

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
        (u) => {
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

function eventMarkersPlugin(events) {
  return {
    hooks: {
      draw: [
        (u) => {
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

function eventColor(type) {
  switch (type) {
    case "ThrottleChop": return "#e85454";
    case "GyroSpike": return "#e8944a";
    case "MotorSaturation": return "#e8b84a";
    case "Desync": return "#e85454";
    case "Overshoot": return "#5b8def";
    case "ThrottlePunch": return "#4ec88c";
    default: return "#ffffff";
  }
}

function renderSummary(session) {
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
        <div class="value">${value}</div>
      </div>`
    )
    .join("");
}

function renderDiagnostics(diagnostics) {
  const list = $("#diagnostics-list");
  if (!diagnostics || diagnostics.length === 0) {
    list.innerHTML = '<p class="hint">No issues detected.</p>';
    return;
  }

  list.innerHTML = diagnostics
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

function renderSpectra(vibration) {
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

function peakMarkersPlugin(peaks, maxFreq) {
  const filtered = (peaks || [])
    .filter((p) => p.frequency_hz <= maxFreq)
    .slice(0, 3);
  return {
    hooks: {
      draw: [
        (u) => {
          const ctx = u.ctx;
          const placed = [];
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

function createSpectrumPlot(container, spectrum) {
  const freqs = spectrum.frequencies_hz;
  const mags = spectrum.magnitudes_db;

  const maxFreq = Math.min(spectrum.sample_rate_hz / 2, 1000);
  const endIdx = freqs.findIndex((f) => f > maxFreq);
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

function filterOverlayPlugin(maxFreq) {
  return {
    hooks: {
      draw: [
        (u) => {
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

function renderSpectrogram(sessionIdx) {
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

function drawSpectrogram(canvas, axis) {
  const ctx = canvas.getContext("2d");
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
  tmpCanvas.getContext("2d").putImageData(img, 0, 0);

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tmpCanvas, 0, 0, width, height);

  ctx.fillStyle = "#e0e0e6";
  ctx.font = "10px JetBrains Mono, monospace";
  ctx.textAlign = "right";
  ctx.fillText(`${axis.frequencies_hz[nFreq - 1]?.toFixed(0) || ""} Hz`, width - 4, 12);
  ctx.fillText("0 Hz", width - 4, height - 4);
}

function heatColor(t) {
  if (t < 0.25) {
    const s = t / 0.25;
    return [0, 0, Math.round(80 + 175 * s)];
  }
  if (t < 0.5) {
    const s = (t - 0.25) / 0.25;
    return [0, Math.round(255 * s), 255];
  }
  if (t < 0.75) {
    const s = (t - 0.5) / 0.25;
    return [Math.round(255 * s), 255, Math.round(255 * (1 - s))];
  }
  const s = (t - 0.75) / 0.25;
  return [255, Math.round(255 * (1 - s)), 0];
}

function renderThrottleBands(vibration) {
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

function createMultiAxisPlot(container, spectra) {
  if (spectra.length === 0) return;

  const maxFreq = Math.min(spectra[0].sample_rate_hz / 2, 1000);
  const endIdx = spectra[0].frequencies_hz.findIndex((f) => f > maxFreq);
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

function renderAccel(vibration) {
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
    accel.rms.map((v, i) =>
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

function renderEvents(events) {
  const list = $("#events-list");
  if (!events || events.length === 0) {
    list.innerHTML = '<p class="hint">No events detected.</p>';
    return;
  }

  const MAX_DISPLAY = 200;
  const shown = events.slice(0, MAX_DISPLAY);

  let html = `<table class="event-table">
    <thead><tr><th>Time</th><th>Type</th><th>Details</th></tr></thead>
    <tbody>`;

  for (const e of shown) {
    html += `<tr>
      <td>${e.time_seconds.toFixed(2)}s</td>
      <td><span class="event-type ${e.kind.type}">${formatEventType(e.kind.type)}</span></td>
      <td>${formatEventDetails(e.kind)}</td>
    </tr>`;
  }

  html += "</tbody></table>";

  if (events.length > MAX_DISPLAY) {
    html += `<p class="events-truncated">Showing ${MAX_DISPLAY} of ${events.length} events</p>`;
  }

  list.innerHTML = html;
}

function formatEventType(type) {
  return type.replace(/([A-Z])/g, " $1").trim();
}

function formatEventDetails(kind) {
  switch (kind.type) {
    case "ThrottleChop":
      return `${kind.from_percent.toFixed(0)}% &rarr; ${kind.to_percent.toFixed(0)}% in ${kind.duration_ms.toFixed(0)}ms`;
    case "ThrottlePunch":
      return `${kind.from_percent.toFixed(0)}% &rarr; ${kind.to_percent.toFixed(0)}% in ${kind.duration_ms.toFixed(0)}ms`;
    case "MotorSaturation":
      return `Motor ${kind.motor_index} for ${kind.duration_frames} frames`;
    case "GyroSpike":
      return `${kind.axis} axis: ${kind.magnitude.toFixed(0)} deg/s`;
    case "Overshoot":
      return `${kind.axis}: ${kind.overshoot_percent.toFixed(0)}% over setpoint`;
    case "Desync":
      return `Motor ${kind.motor_index} (${kind.motor_value}) vs avg ${kind.average_others.toFixed(0)}`;
    default:
      return "";
  }
}

function escHtml(str) {
  const div = document.createElement("div");
  div.textContent = str;
  return div.innerHTML;
}

let activeSessionIdx = 0;
let resizeTimer = null;
window.addEventListener("resize", () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => {
    if (result && !$("#results").classList.contains("hidden")) {
      showSession(activeSessionIdx);
    }
  }, 250);
});

boot().catch((err) => {
  console.error("Failed to initialize WASM:", err);
  $(".drop-zone-content p").textContent = "Failed to load analyzer. Check console.";
});
