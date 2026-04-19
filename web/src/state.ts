import type { WorkspaceFile, SessionRef, SessionResult, FilterConfig } from "./types.js";

export const $ = (sel: string): HTMLElement => document.querySelector(sel) as HTMLElement;
export const $$ = (sel: string): NodeListOf<HTMLElement> => document.querySelectorAll(sel) as NodeListOf<HTMLElement>;

export function chartWidth(): number {
  const padding = window.innerWidth <= 640 ? 24 : 64;
  return Math.min($("main").clientWidth - padding, 880);
}

export function escHtml(str: string): string {
  const div = document.createElement("div");
  div.textContent = str;
  return div.innerHTML;
}

// --- Mutable state ---

export let wasmReady = false;
export let workspace: WorkspaceFile[] = [];
export let activeSession: SessionRef | null = null;
export let filterConfig: FilterConfig | null = null;
export let tsPlots: any[] = [];
export let tsSync: any = null;
export let navigateTarget: number | null = null;
export let renderedViews = new Set<string>();
export const echartsInstances: any[] = [];

// State setters (module-scoped lets cannot be reassigned from outside)
export function setWasmReady(v: boolean): void { wasmReady = v; }
export function setWorkspace(v: WorkspaceFile[]): void { workspace = v; }
export function setActiveSession(v: SessionRef | null): void { activeSession = v; }
export function setFilterConfig(v: FilterConfig | null): void { filterConfig = v; }
export function setTsPlots(v: any[]): void { tsPlots = v; }
export function setTsSync(v: any): void { tsSync = v; }
export function setNavigateTarget(v: number | null): void { navigateTarget = v; }
export function setRenderedViews(v: Set<string>): void { renderedViews = v; }

export function disposeEcharts(): void {
  for (const inst of echartsInstances) {
    inst.dispose();
  }
  echartsInstances.length = 0;
}

/** Look up the active session's analysis result. */
export function activeSessionResult(): SessionResult | null {
  if (!activeSession) return null;
  const file = workspace.find(f => f.file_id === activeSession!.fileId);
  return file?.sessions[activeSession.sessionIdx] ?? null;
}

/** Build a flat list of all sessions across workspace files. */
export function allSessionRefs(): SessionRef[] {
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

export function lookupSession(val: string): SessionResult | null {
  const [fileId, sessionIdx] = val.split(":").map(Number);
  const file = workspace.find(f => f.file_id === fileId);
  return file?.sessions[sessionIdx] ?? null;
}
