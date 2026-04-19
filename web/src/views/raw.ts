import { get_raw_frames } from "../../pkg/propwash_web.js";
import type { SessionRef } from "../types.js";
import { $ } from "../state.js";

declare global {
  interface Window {
    _rawPage: (fileId: number, sessionIdx: number, start: number) => void;
  }
}

const RAW_PAGE_SIZE = 200;
const RAW_DEFAULT_FIELDS = ["time", "gyro[roll]", "gyro[pitch]", "gyro[yaw]", "motor[0]", "motor[1]", "motor[2]", "motor[3]", "rc[throttle]"];

export function renderRawData(ref: SessionRef): void {
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

export function loadRawPage(ref: SessionRef, start: number): void {
  const select = $("#raw-field-select") as HTMLSelectElement;
  const selected = Array.from(select.selectedOptions).map((o) => o.value);
  if (selected.length === 0) return;

  const json = get_raw_frames(ref.fileId, ref.sessionIdx, start, RAW_PAGE_SIZE, selected.join(","));
  const data = JSON.parse(json);
  if (data.error) return;

  $("#raw-frame-info").textContent = `Frames ${data.start}\u2013${data.start + data.frames.length} of ${data.total.toLocaleString()}`;

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

// Global bridge for inline onclick handlers in raw data pagination
window._rawPage = (fileId: number, sessionIdx: number, start: number) => {
  loadRawPage({ fileId, sessionIdx, label: "" }, start);
};
