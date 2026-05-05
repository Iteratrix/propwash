import type { SensorField, Axis, RcChannel } from "../pkg/propwash_web.js";

const AXIS_MAP: Record<string, Axis> = {
  roll: "Roll",
  pitch: "Pitch",
  yaw: "Yaw",
};

const RC_CHANNEL_MAP: Record<string, RcChannel> = {
  roll: "Roll",
  pitch: "Pitch",
  yaw: "Yaw",
  throttle: "Throttle",
};

const UNIT_FIELDS: Record<string, SensorField> = {
  time: "Time",
  vbat: "Vbat",
  altitude: "Altitude",
  gps_speed: "GpsSpeed",
  gps_lat: "GpsLat",
  gps_lng: "GpsLng",
  heading: "Heading",
  rssi: "Rssi",
};

function parseAxisIndex(name: string): Axis | undefined {
  const m = name.match(/\[(\w+)\]$/);
  if (!m) return undefined;
  return AXIS_MAP[m[1]];
}

function parseNumericIndex(name: string): number | undefined {
  const m = name.match(/\[(\d+)\]$/);
  if (!m) return undefined;
  return Number.parseInt(m[1], 10);
}

function parseRcChannel(name: string): RcChannel | undefined {
  const m = name.match(/\[(\w+)\]$/);
  if (!m) return undefined;
  return RC_CHANNEL_MAP[m[1]];
}

/**
 * Parse a canonical field name (e.g. "gyro[roll]", "motor[0]", "vbat")
 * into a typed SensorField. Mirrors the Rust SensorField::parse logic.
 */
export function parseSensorField(name: string): SensorField {
  const unit = UNIT_FIELDS[name];
  if (unit) return unit;

  if (name.startsWith("gyro[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { Gyro: axis };
  }
  if (name.startsWith("gyro_unfilt[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { GyroUnfilt: axis };
  }
  if (name.startsWith("setpoint[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { Setpoint: axis };
  }
  if (name.startsWith("accel[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { Accel: axis };
  }
  if (name.startsWith("pid_p[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { PidP: axis };
  }
  if (name.startsWith("pid_i[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { PidI: axis };
  }
  if (name.startsWith("pid_d[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { PidD: axis };
  }
  if (name.startsWith("feedforward[")) {
    const axis = parseAxisIndex(name);
    if (axis) return { Feedforward: axis };
  }
  if (name.startsWith("motor[")) {
    const idx = parseNumericIndex(name);
    if (idx !== undefined) return { Motor: idx };
  }
  if (name.startsWith("erpm[")) {
    const idx = parseNumericIndex(name);
    if (idx !== undefined) return { ERpm: idx };
  }
  if (name.startsWith("rc[")) {
    const ch = parseRcChannel(name);
    if (ch) return { Rc: ch };
  }

  return { Unknown: name };
}

/**
 * Parse a comma-separated list of canonical field names into an array
 * of typed SensorField values, ready to hand to a WASM bridge call.
 */
export function parseSensorFields(csv: string): SensorField[] {
  return csv.split(",").map((s) => parseSensorField(s.trim()));
}
