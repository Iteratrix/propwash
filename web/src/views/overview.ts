import { get_step_overlay } from "../../pkg/propwash_web.js";
import { formatEventType, formatEventDetails, eventColor } from "../format.js";
import type { SessionRef, SessionResult, FlightEvent, Diagnostic, PidAnalysis, StepOverlay } from "../types.js";
import { $, $$, chartWidth, escHtml, tsPlots, navigateTarget, setNavigateTarget, renderedViews, activeSession } from "../state.js";

function navigateToView(view: string, category: string): void {
  // Switch to target view tab
  const tabs = $$("#view-tabs .view-tab");
  tabs.forEach((t) => t.classList.remove("active"));
  const targetTab = $(`.view-tab[data-view="${view}"]`);
  if (targetTab) targetTab.classList.add("active");
  $$(".view-content").forEach((v) => v.classList.remove("active"));
  $(`.view-content[data-view="${view}"]`)?.classList.add("active");

  // If navigating to timeline, select the appropriate sub-tab
  if (view === "timeline") {
    const groupMap: Record<string, string> = {
      tuning: "pids",
      pid: "pids",
      motors: "motors",
      esc: "motors",
    };
    const group = groupMap[category];
    if (group) {
      const tsTab = $(`.ts-tab[data-group="${group}"]`);
      if (tsTab) tsTab.click();
    }
  }

  // Render the view
  import("../app.js").then(({ renderViewIfNeeded }) => {
    renderViewIfNeeded(view);
  });
}

export function navigateToTime(seconds: number): void {
  // Switch to timeline tab
  const tabs = $$("#view-tabs .view-tab");
  tabs.forEach((t) => t.classList.remove("active"));
  const timelineTab = $(".view-tab[data-view='timeline']");
  timelineTab.classList.add("active");
  $$(".view-content").forEach((v) => v.classList.remove("active"));
  $(`.view-content[data-view="timeline"]`)!.classList.add("active");

  // Ensure timeline is rendered — import lazily to avoid circular dep
  import("../app.js").then(({ renderViewIfNeeded }) => {
    renderViewIfNeeded("timeline");

    // Zoom to +/-1.5s window around the event
    const win = 1.5;
    const min = Math.max(0, seconds - win);
    const max = seconds + win;
    for (const p of tsPlots) {
      p.setScale("x", { min, max });
    }

    // Flash the navigation target marker
    setNavigateTarget(seconds);
    setTimeout(() => { setNavigateTarget(null); for (const p of tsPlots) p.redraw(); }, 2000);
    for (const p of tsPlots) p.redraw();
  });
}

export function navigateMarkerPlugin(): any {
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

export function renderVerdict(diagnostics: Diagnostic[]): void {
  const container = $("#health-verdict");
  if (!diagnostics || diagnostics.length === 0) {
    container.innerHTML = `<div class="health-verdict verdict-green">No issues detected</div>`;
    return;
  }

  const problems = diagnostics.filter(d => d.severity === "Problem").length;
  const warnings = diagnostics.filter(d => d.severity === "Warning").length;
  const infos = diagnostics.filter(d => d.severity === "Info").length;

  const parts: string[] = [];
  if (problems > 0) parts.push(`${problems} problem${problems > 1 ? "s" : ""}`);
  if (warnings > 0) parts.push(`${warnings} warning${warnings > 1 ? "s" : ""}`);
  if (infos > 0) parts.push(`${infos} info`);

  const severity = problems > 0 ? "red" : warnings > 0 ? "yellow" : "green";
  container.innerHTML = `<div class="health-verdict verdict-${severity}">${parts.join(", ")}</div>`;
}

export function renderSummary(session: SessionResult): void {
  const grid = $("#summary-grid");
  const meta = $("#summary-meta");

  // Compact metadata line for less important fields
  const metaItems: [string, string | number][] = [
    ["Firmware", session.firmware || "Unknown"],
    ["Craft", session.craft || "Unknown"],
    ["Sample Rate", `${session.sample_rate_hz.toFixed(0)} Hz`],
    ["Frames", session.frame_count.toLocaleString()],
    ["Motors", session.analysis.summary.motor_count],
  ];

  meta.innerHTML = metaItems
    .map(([label, value]) =>
      `<span><span class="meta-label">${label}</span>${value}</span>`
    )
    .join("");

  // Prominent cards for key operational fields
  const cards: [string, string | number][] = [
    ["Duration", `${session.duration_seconds.toFixed(1)}s`],
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

export function renderTuning(pid: PidAnalysis | null): void {
  const panel = $("#tuning-panel");
  const content = $("#tuning-content");

  if (!pid || pid.tuning.length === 0) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");

  // If all axes are Good or Tight, show a single summary line
  const allGood = pid.tuning.every(t => t.rating === "Good" || t.rating === "Tight");
  if (allGood) {
    content.innerHTML = `<p class="tuning-good">PID tuning looks good on all axes</p>`;
    return;
  }

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

export function renderStepOverlay(ref: SessionRef): void {
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

    // Individual gyro traces -- thin, semi-transparent
    for (let i = 0; i < axis.gyro_steps.length; i++) {
      datasets.push(axis.gyro_steps[i]);
      series.push({
        label: i === 0 ? "Steps" : "",
        stroke: color + "30",
        width: 0.5,
      });
    }

    // Average gyro -- thick, solid
    datasets.push(axis.gyro_average);
    series.push({
      label: "Average",
      stroke: color,
      width: 2.5,
    });

    // Average setpoint -- dashed gray
    if (axis.setpoint_steps.length > 0) {
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

export function renderDiagnostics(diagnostics: Diagnostic[]): void {
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

  const categoryView: Record<string, string> = {
    vibration: "spectrum",
    filters: "spectrum",
    mounting: "spectrum",
    tuning: "timeline",
    pid: "timeline",
    motors: "timeline",
    esc: "timeline",
  };

  list.innerHTML = sorted
    .map((d, i) => {
      const target = categoryView[d.category];
      const showBtn = target
        ? `<button class="diag-show-btn" data-idx="${i}">Show</button>`
        : "";
      return `<div class="diagnostic ${d.severity}">
        <div class="diag-header">
          <span class="severity">${d.severity}</span>
          <span class="category">${d.category}</span>
          ${showBtn}
        </div>
        <div class="message">${escHtml(d.message)}</div>
        <div class="detail">${escHtml(d.detail)}</div>
      </div>`;
    })
    .join("");

  // Wire up "Show" buttons
  list.querySelectorAll(".diag-show-btn").forEach((btn) => {
    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      const idx = parseInt((btn as HTMLElement).dataset.idx!, 10);
      const d = sorted[idx];
      const view = categoryView[d.category];
      if (view) {
        navigateToView(view, d.category);
      }
    });
  });
}

export function renderEvents(events: FlightEvent[]): void {
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
