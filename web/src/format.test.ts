import { describe, it, expect } from "vitest";
import {
  formatEventType,
  formatEventDetails,
  eventColor,
  heatColor,
  bucketDiagnostics,
  classifyDelta,
} from "./format.js";
import type { EventKind, Diagnostic } from "./types.js";

// --- formatEventType ---

describe("formatEventType", () => {
  it("splits CamelCase into words", () => {
    expect(formatEventType("ThrottleChop")).toBe("Throttle Chop");
    expect(formatEventType("GyroSpike")).toBe("Gyro Spike");
    expect(formatEventType("MotorSaturation")).toBe("Motor Saturation");
    expect(formatEventType("ThrottlePunch")).toBe("Throttle Punch");
    expect(formatEventType("FirmwareMessage")).toBe("Firmware Message");
  });

  it("handles single-word types", () => {
    expect(formatEventType("Desync")).toBe("Desync");
    expect(formatEventType("Overshoot")).toBe("Overshoot");
  });

  it("handles empty string", () => {
    expect(formatEventType("")).toBe("");
  });
});

// --- formatEventDetails ---

describe("formatEventDetails", () => {
  it("formats ThrottleChop with percentages and duration", () => {
    const kind: EventKind = {
      type: "ThrottleChop",
      from_percent: 82.3,
      to_percent: 10.7,
      duration_ms: 153.4,
    };
    const result = formatEventDetails(kind);
    expect(result).toContain("82%");
    expect(result).toContain("11%");
    expect(result).toContain("153ms");
    expect(result).toContain("\u2192");
  });

  it("formats ThrottlePunch", () => {
    const kind: EventKind = {
      type: "ThrottlePunch",
      from_percent: 20,
      to_percent: 95,
      duration_ms: 80,
    };
    const result = formatEventDetails(kind);
    expect(result).toContain("20%");
    expect(result).toContain("95%");
    expect(result).toContain("80ms");
  });

  it("formats MotorSaturation", () => {
    const kind: EventKind = {
      type: "MotorSaturation",
      motor_index: 2,
      duration_frames: 150,
    };
    expect(formatEventDetails(kind)).toBe("Motor 2 for 150 frames");
  });

  it("formats GyroSpike with axis and magnitude", () => {
    const kind: EventKind = {
      type: "GyroSpike",
      axis: "roll",
      magnitude: 1234.5,
    };
    expect(formatEventDetails(kind)).toBe("roll axis: 1235 deg/s");
  });

  it("formats Overshoot", () => {
    const kind: EventKind = {
      type: "Overshoot",
      axis: "pitch",
      overshoot_percent: 42.7,
    };
    expect(formatEventDetails(kind)).toBe("pitch: 43% over setpoint");
  });

  it("formats Desync", () => {
    const kind: EventKind = {
      type: "Desync",
      motor_index: 3,
      motor_value: 1200,
      average_others: 1850.5,
    };
    const result = formatEventDetails(kind);
    expect(result).toContain("Motor 3");
    expect(result).toContain("1200");
    expect(result).toContain("1851"); // rounded
  });

  it("formats FirmwareMessage", () => {
    const kind: EventKind = {
      type: "FirmwareMessage",
      level: "Warning",
      message: "Compass variance",
    };
    expect(formatEventDetails(kind)).toBe("[Warning] Compass variance");
  });
});

// --- eventColor ---

describe("eventColor", () => {
  it("maps known event types to colors", () => {
    expect(eventColor("ThrottleChop")).toBe("#e85454");
    expect(eventColor("GyroSpike")).toBe("#e8944a");
    expect(eventColor("MotorSaturation")).toBe("#e8b84a");
    expect(eventColor("Desync")).toBe("#e85454");
    expect(eventColor("Overshoot")).toBe("#5b8def");
    expect(eventColor("ThrottlePunch")).toBe("#4ec88c");
    expect(eventColor("FirmwareMessage")).toBe("#e8b84a");
  });

  it("returns white for unknown types", () => {
    expect(eventColor("SomethingNew")).toBe("#ffffff");
    expect(eventColor("")).toBe("#ffffff");
  });

  it("shares colors for related severity (ThrottleChop = Desync = red)", () => {
    expect(eventColor("ThrottleChop")).toBe(eventColor("Desync"));
  });
});

// --- heatColor ---

describe("heatColor", () => {
  it("starts dark blue at t=0", () => {
    const [r, g, b] = heatColor(0);
    expect(r).toBe(0);
    expect(g).toBe(0);
    expect(b).toBe(80);
  });

  it("reaches bright blue at t=0.25", () => {
    const [r, g, b] = heatColor(0.25);
    expect(r).toBe(0);
    expect(g).toBe(0);
    expect(b).toBe(255);
  });

  it("reaches cyan at t=0.5", () => {
    const [r, g, b] = heatColor(0.5);
    expect(r).toBe(0);
    expect(g).toBe(255);
    expect(b).toBe(255);
  });

  it("reaches yellow at t=0.75", () => {
    const [r, g, b] = heatColor(0.75);
    expect(r).toBe(255);
    expect(g).toBe(255);
    expect(b).toBe(0);
  });

  it("reaches red at t=1.0", () => {
    const [r, g, b] = heatColor(1.0);
    expect(r).toBe(255);
    expect(g).toBe(0);
    expect(b).toBe(0);
  });

  it("interpolates smoothly at midpoints", () => {
    // t=0.125 — halfway through first segment
    const [r, g, b] = heatColor(0.125);
    expect(r).toBe(0);
    expect(g).toBe(0);
    expect(b).toBeGreaterThan(80);
    expect(b).toBeLessThan(255);
  });

  it("returns valid RGB values across full range", () => {
    for (let i = 0; i <= 100; i++) {
      const t = i / 100;
      const [r, g, b] = heatColor(t);
      expect(r).toBeGreaterThanOrEqual(0);
      expect(r).toBeLessThanOrEqual(255);
      expect(g).toBeGreaterThanOrEqual(0);
      expect(g).toBeLessThanOrEqual(255);
      expect(b).toBeGreaterThanOrEqual(0);
      expect(b).toBeLessThanOrEqual(255);
    }
  });

  it("transitions from cold (blue-dominant) to hot (red-dominant)", () => {
    const [r0, , b0] = heatColor(0);
    expect(b0).toBeGreaterThan(r0); // cold end: blue > red

    const [r1, , b1] = heatColor(1);
    expect(r1).toBeGreaterThan(b1); // hot end: red > blue
  });
});

// --- bucketDiagnostics ---

function diag(msg: string, severity = "Warning"): Diagnostic {
  return { severity, category: "Test", message: msg, detail: "" };
}

describe("bucketDiagnostics", () => {
  it("identifies fixed diagnostics (in A but not B)", () => {
    const a = [diag("noisy gyro"), diag("motor desync")];
    const b = [diag("motor desync")];
    const { fixed, newIssues, unchanged } = bucketDiagnostics(a, b);
    expect(fixed).toHaveLength(1);
    expect(fixed[0].message).toBe("noisy gyro");
    expect(newIssues).toHaveLength(0);
    expect(unchanged).toHaveLength(1);
  });

  it("identifies new issues (in B but not A)", () => {
    const a = [diag("motor desync")];
    const b = [diag("motor desync"), diag("high vibration")];
    const { fixed, newIssues, unchanged } = bucketDiagnostics(a, b);
    expect(fixed).toHaveLength(0);
    expect(newIssues).toHaveLength(1);
    expect(newIssues[0].message).toBe("high vibration");
    expect(unchanged).toHaveLength(1);
  });

  it("handles completely different diagnostics", () => {
    const a = [diag("issue A")];
    const b = [diag("issue B")];
    const { fixed, newIssues, unchanged } = bucketDiagnostics(a, b);
    expect(fixed).toHaveLength(1);
    expect(newIssues).toHaveLength(1);
    expect(unchanged).toHaveLength(0);
  });

  it("handles identical diagnostics", () => {
    const a = [diag("same"), diag("also same")];
    const b = [diag("same"), diag("also same")];
    const { fixed, newIssues, unchanged } = bucketDiagnostics(a, b);
    expect(fixed).toHaveLength(0);
    expect(newIssues).toHaveLength(0);
    expect(unchanged).toHaveLength(2);
  });

  it("handles empty inputs", () => {
    expect(bucketDiagnostics([], []).fixed).toHaveLength(0);
    expect(bucketDiagnostics([diag("a")], []).fixed).toHaveLength(1);
    expect(bucketDiagnostics([], [diag("b")]).newIssues).toHaveLength(1);
  });

  it("preserves severity from original diagnostic", () => {
    const a = [diag("high vibe", "Problem")];
    const b = [diag("high vibe", "Warning")];
    const { unchanged } = bucketDiagnostics(a, b);
    // unchanged comes from b's perspective
    expect(unchanged[0].severity).toBe("Warning");
  });
});

// --- classifyDelta ---

describe("classifyDelta", () => {
  it("returns neutral when diff is 0", () => {
    expect(classifyDelta(0, "lower")).toBe("neutral");
    expect(classifyDelta(0, "higher")).toBe("neutral");
  });

  it("classifies lower-is-better correctly", () => {
    expect(classifyDelta(-5, "lower")).toBe("better");
    expect(classifyDelta(5, "lower")).toBe("worse");
  });

  it("classifies higher-is-better correctly", () => {
    expect(classifyDelta(5, "higher")).toBe("better");
    expect(classifyDelta(-5, "higher")).toBe("worse");
  });
});
