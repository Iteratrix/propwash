import init, { add_file, clear_workspace, get_filter_config } from "../pkg/propwash_web.js";
import type { WorkspaceFile, SessionRef } from "./types.js";
import {
  $, $$, workspace, activeSession, renderedViews, echartsInstances, tsPlots,
  setWasmReady, setWorkspace, setActiveSession, setFilterConfig, setRenderedViews,
  activeSessionResult, allSessionRefs, disposeEcharts,
} from "./state.js";
import { renderVerdict, renderSummary, renderTuning, renderStepOverlay, renderDiagnostics, renderEvents } from "./views/overview.js";
import { renderTimeseries, setupTimeseriesControls } from "./views/timeline.js";
import { renderSpectraEcharts, renderSpectrogram, renderThrottleBandsEcharts, renderAccel, renderPropwash } from "./views/spectrum.js";
import { populateComparePickers, renderCompare } from "./views/compare.js";
import { renderTrend } from "./views/trend.js";
import { renderRawData } from "./views/raw.js";

async function boot() {
  await init({ module_or_path: __WASM_URL__ });
  setWasmReady(true);
  setupDropZone();
  setupTimeseriesControls();
  setupViewTabs();
  $("#reset-btn").addEventListener("click", reset);
  $("#add-files-btn")?.addEventListener("click", () => {
    ($("#file-input") as HTMLInputElement).click();
  });
  registerServiceWorker();
}

function registerServiceWorker() {
  if (!("serviceWorker" in navigator)) return;
  if (location.hostname === "localhost" || location.hostname === "127.0.0.1") return;
  navigator.serviceWorker.register("./sw.js").catch((err) => {
    console.warn("Service worker registration failed:", err);
  });
  if (navigator.storage?.persist) {
    navigator.storage.persist().catch(() => {});
  }
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
  setWorkspace([]);
  setActiveSession(null);
  clear_workspace();
  disposeEcharts();
  renderedViews.clear();
  $("#results").classList.add("hidden");
  $("#drop-zone").classList.remove("hidden");
  ($("#file-input") as HTMLInputElement).value = "";
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
  $("#drop-zone").classList.add("hidden");
  $("#loading").classList.remove("hidden");

  await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));

  const ws = [...workspace];
  for (let i = 0; i < files.length; i++) {
    const file = files[i];
    const buffer = await file.arrayBuffer();
    const data = new Uint8Array(buffer);
    const json = add_file(data, file.name);
    const result: WorkspaceFile = JSON.parse(json);
    if (result.sessions.length > 0) {
      ws.push(result);
    }
  }
  setWorkspace(ws);

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

  // Auto-compare: if exactly 2 sessions loaded, switch to compare view
  if (refs.length === 2) {
    const tabs = $$("#view-tabs .view-tab");
    tabs.forEach((t) => t.classList.remove("active"));
    const compareTab = $(".view-tab[data-view='compare']");
    if (compareTab) {
      compareTab.classList.add("active");
      $$(".view-content").forEach((v) => v.classList.remove("active"));
      $(`.view-content[data-view="compare"]`)!.classList.add("active");
      renderViewIfNeeded("compare");
    }
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
  setActiveSession(ref);
  setFilterConfig(JSON.parse(get_filter_config(ref.fileId, ref.sessionIdx)));
  renderedViews.clear();
  const activeView = $(".view-tab.active")?.dataset.view || "overview";
  renderViewIfNeeded(activeView);
}

export function renderViewIfNeeded(view: string): void {
  if (!activeSession) return;
  const key = `${view}-${activeSession.fileId}-${activeSession.sessionIdx}`;
  if (renderedViews.has(key)) return;
  renderedViews.add(key);

  const s = activeSessionResult();
  if (!s) return;

  switch (view) {
    case "overview":
      renderVerdict(s.analysis.diagnostics);
      renderDiagnostics(s.analysis.diagnostics);
      renderTuning(s.analysis.pid);
      renderStepOverlay(activeSession);
      renderSummary(s);
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

// Resize handler
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
