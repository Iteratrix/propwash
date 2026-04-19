import { get_trend } from "../../pkg/propwash_web.js";
import type { TrendPoint } from "../types.js";
import { $, escHtml, workspace } from "../state.js";

export function renderTrend(): void {
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
