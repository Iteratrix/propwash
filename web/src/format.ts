import type { EventKind, Diagnostic } from "./types.js";

export function formatEventType(type: string): string {
  return type.replace(/([A-Z])/g, " $1").trim();
}

export function formatEventDetails(kind: EventKind): string {
  switch (kind.type) {
    case "ThrottleChop":
      return `${kind.from_percent.toFixed(0)}% \u2192 ${kind.to_percent.toFixed(0)}% in ${kind.duration_ms.toFixed(0)}ms`;
    case "ThrottlePunch":
      return `${kind.from_percent.toFixed(0)}% \u2192 ${kind.to_percent.toFixed(0)}% in ${kind.duration_ms.toFixed(0)}ms`;
    case "MotorSaturation":
      return `Motor ${kind.motor_index} for ${kind.duration_frames} frames`;
    case "GyroSpike":
      return `${kind.axis} axis: ${kind.magnitude.toFixed(0)} deg/s`;
    case "Overshoot":
      return `${kind.axis}: ${kind.overshoot_percent.toFixed(0)}% over setpoint`;
    case "Desync":
      return `Motor ${kind.motor_index} (${kind.motor_value}) vs avg ${kind.average_others.toFixed(0)}`;
    case "FirmwareMessage":
      return `[${kind.level}] ${kind.message}`;
    default:
      return "";
  }
}

export function eventColor(type: string): string {
  switch (type) {
    case "ThrottleChop": return "#e85454";
    case "GyroSpike": return "#e8944a";
    case "MotorSaturation": return "#e8b84a";
    case "Desync": return "#e85454";
    case "Overshoot": return "#5b8def";
    case "ThrottlePunch": return "#4ec88c";
    case "FirmwareMessage": return "#e8b84a";
    default: return "#ffffff";
  }
}

export function heatColor(t: number): [number, number, number] {
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

export interface DiagnosticBuckets {
  fixed: Diagnostic[];
  newIssues: Diagnostic[];
  unchanged: Diagnostic[];
}

export function bucketDiagnostics(a: Diagnostic[], b: Diagnostic[]): DiagnosticBuckets {
  const msgsA = new Set(a.map((d) => d.message));
  const msgsB = new Set(b.map((d) => d.message));
  return {
    fixed: a.filter((d) => !msgsB.has(d.message)),
    newIssues: b.filter((d) => !msgsA.has(d.message)),
    unchanged: b.filter((d) => msgsA.has(d.message)),
  };
}

export function classifyDelta(diff: number, better: "lower" | "higher"): "better" | "worse" | "neutral" {
  if (diff === 0) return "neutral";
  if (better === "lower") return diff < 0 ? "better" : "worse";
  return diff > 0 ? "better" : "worse";
}
