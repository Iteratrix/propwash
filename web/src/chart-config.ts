export const AXIS_COLORS: Record<string, string> = {
  Roll:  "#5b8def",
  Pitch: "#4ec88c",
  Yaw:   "#e8b84a",
  X: "#5b8def",
  Y: "#4ec88c",
  Z: "#e8b84a",
};

export type PidAxis = "roll" | "pitch" | "yaw";

export interface FieldGroup {
  label: string;
  fields: string[];
  names: string[];
  colors: string[];
  computed?: string;
}

export function pidFieldGroup(axis: PidAxis): FieldGroup {
  const label = axis.charAt(0).toUpperCase() + axis.slice(1);
  return {
    label: `PIDs (${label})`,
    fields: [`pid_p[${axis}]`, `pid_i[${axis}]`, `pid_d[${axis}]`, `feedforward[${axis}]`],
    names: ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  };
}

export const FIELD_GROUPS: Record<string, FieldGroup> = {
  gyro: {
    label: "Gyro (deg/s)",
    fields: ["gyro[roll]", "gyro[pitch]", "gyro[yaw]"],
    names:  ["Roll", "Pitch", "Yaw"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
  },
  spVsGyro: {
    label: "Setpoint vs Gyro (deg/s)",
    fields: ["setpoint[roll]", "gyro[roll]", "setpoint[pitch]", "gyro[pitch]", "setpoint[yaw]", "gyro[yaw]"],
    names:  ["SP Roll", "Gyro Roll", "SP Pitch", "Gyro Pitch", "SP Yaw", "Gyro Yaw"],
    colors: ["#5b8def66", "#5b8def", "#4ec88c66", "#4ec88c", "#e8b84a66", "#e8b84a"],
  },
  pidError: {
    label: "PID Error — Setpoint minus Gyro (deg/s)",
    fields: ["setpoint[roll]", "gyro[roll]", "setpoint[pitch]", "gyro[pitch]", "setpoint[yaw]", "gyro[yaw]"],
    names:  ["Roll", "Pitch", "Yaw"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
    computed: "error",
  },
  pids: {
    label: "PIDs (Roll)",
    fields: ["pid_p[roll]", "pid_i[roll]", "pid_d[roll]", "feedforward[roll]"],
    names:  ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  },
  motors: {
    label: "Motors",
    fields: ["motor[0]", "motor[1]", "motor[2]", "motor[3]"],
    names:  ["Motor 1", "Motor 2", "Motor 3", "Motor 4"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e85454"],
  },
  power: {
    label: "Power (Battery + RPM)",
    fields: ["vbat", "rc[throttle]", "erpm[0]", "erpm[1]", "erpm[2]", "erpm[3]"],
    names:  ["Battery (V)", "Throttle", "RPM 1", "RPM 2", "RPM 3", "RPM 4"],
    colors: ["#e85454", "#e8944a", "#5b8def88", "#4ec88c88", "#e8b84a88", "#e8545488"],
  },
  rc: {
    label: "RC Commands",
    fields: ["rc[roll]", "rc[pitch]", "rc[yaw]", "rc[throttle]"],
    names:  ["Roll", "Pitch", "Yaw", "Throttle"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e8944a"],
  },
  telemetry: {
    label: "Telemetry (Altitude / GPS / RSSI)",
    fields: ["altitude", "gps_speed", "gps_lat", "gps_lng", "heading", "rssi"],
    names:  ["Altitude (m)", "GPS Speed (m/s)", "Latitude", "Longitude", "Heading", "RSSI"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e8944a", "#9b59b6", "#4ec88c"],
  },
};

export const TS_MAX_POINTS = 4000;
