import { get_timeseries } from "../../pkg/propwash_web.js";
import { eventColor } from "../format.js";
import type { SessionRef, FlightEvent } from "../types.js";
import { $, $$, chartWidth, activeSession, activeSessionResult, renderedViews, tsPlots, setTsPlots, setTsSync } from "../state.js";
import { FIELD_GROUPS, TS_MAX_POINTS, pidFieldGroup } from "../chart-config.js";
import type { PidAxis, FieldGroup } from "../chart-config.js";
import { navigateMarkerPlugin } from "./overview.js";

let selectedPidAxis: PidAxis = "roll";

function updatePidAxisToggleVisibility(group: string): void {
  const toggle = $("#ts-pid-axis-toggle");
  if (group === "pids") {
    toggle.classList.remove("hidden");
  } else {
    toggle.classList.add("hidden");
  }
}

function resolveFieldGroup(group: string): FieldGroup | undefined {
  if (group === "pids") {
    return pidFieldGroup(selectedPidAxis);
  }
  return FIELD_GROUPS[group];
}

export function setupTimeseriesControls(): void {
  const tabs = $$("#ts-tabs .ts-tab");
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      if (!activeSession) return;
      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      const group = tab.dataset.group!;
      updatePidAxisToggleVisibility(group);
      renderedViews.delete(`timeline-${activeSession.fileId}-${activeSession.sessionIdx}`);
      renderTimeseries(activeSession, group);
    });
  });

  // PID axis toggle
  const axisButtons = $$("#ts-pid-axis-toggle .pid-axis-btn");
  axisButtons.forEach((btn) => {
    btn.addEventListener("click", () => {
      if (!activeSession) return;
      axisButtons.forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      selectedPidAxis = btn.dataset.axis as PidAxis;
      renderedViews.delete(`timeline-${activeSession.fileId}-${activeSession.sessionIdx}`);
      renderTimeseries(activeSession, "pids");
    });
  });

  $("#ts-reset-zoom").addEventListener("click", () => {
    if (!activeSession) return;
    const activeGroup = $(".ts-tab.active")?.dataset.group || "gyro";
    renderedViews.delete(`timeline-${activeSession.fileId}-${activeSession.sessionIdx}`);
    renderTimeseries(activeSession, activeGroup);
  });
}

export function eventMarkersPlugin(events: FlightEvent[]): any {
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

export function renderTimeseries(ref: SessionRef, group: string): void {
  const container = $("#ts-charts");
  container.innerHTML = "";
  setTsPlots([]);

  updatePidAxisToggleVisibility(group);

  const g = resolveFieldGroup(group);
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
  setTsSync(uPlot.sync(syncKey));

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
