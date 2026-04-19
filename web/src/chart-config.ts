export const AXIS_COLORS: Record<string, string> = {
  Roll:  "#5b8def",
  Pitch: "#4ec88c",
  Yaw:   "#e8b84a",
  X: "#5b8def",
  Y: "#4ec88c",
  Z: "#e8b84a",
};

export const FIELD_GROUPS = {
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
  gyroFilt: {
    label: "Filtered vs Unfiltered Gyro (deg/s)",
    fields: ["gyro[roll]", "gyro_unfilt[roll]", "gyro[pitch]", "gyro_unfilt[pitch]", "gyro[yaw]", "gyro_unfilt[yaw]"],
    names:  ["Filt Roll", "Raw Roll", "Filt Pitch", "Raw Pitch", "Filt Yaw", "Raw Yaw"],
    colors: ["#5b8def", "#5b8def44", "#4ec88c", "#4ec88c44", "#e8b84a", "#e8b84a44"],
  },
  motors: {
    label: "Motors",
    fields: ["motor[0]", "motor[1]", "motor[2]", "motor[3]"],
    names:  ["Motor 1", "Motor 2", "Motor 3", "Motor 4"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e85454"],
  },
  rc: {
    label: "RC Commands",
    fields: ["rc[roll]", "rc[pitch]", "rc[yaw]", "rc[throttle]"],
    names:  ["Roll", "Pitch", "Yaw", "Throttle"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e8944a"],
  },
  voltage: {
    label: "Battery / Throttle",
    fields: ["vbat", "rc[throttle]"],
    names:  ["Battery (V)", "Throttle"],
    colors: ["#e85454", "#e8944a"],
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
  pidsPitch: {
    label: "PIDs (Pitch)",
    fields: ["pid_p[pitch]", "pid_i[pitch]", "pid_d[pitch]", "feedforward[pitch]"],
    names:  ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  },
  pidsYaw: {
    label: "PIDs (Yaw)",
    fields: ["pid_p[yaw]", "pid_i[yaw]", "pid_d[yaw]", "feedforward[yaw]"],
    names:  ["P", "I", "D", "FF"],
    colors: ["#5b8def", "#4ec88c", "#e85454", "#9b59b6"],
  },
  erpm: {
    label: "Motor RPM",
    fields: ["erpm[0]", "erpm[1]", "erpm[2]", "erpm[3]"],
    names:  ["Motor 1", "Motor 2", "Motor 3", "Motor 4"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a", "#e85454"],
  },
  altitude: {
    label: "Altitude / GPS Speed",
    fields: ["altitude", "gps_speed"],
    names:  ["Altitude (m)", "GPS Speed (m/s)"],
    colors: ["#5b8def", "#4ec88c"],
  },
  gps: {
    label: "GPS Position",
    fields: ["gps_lat", "gps_lng", "heading"],
    names:  ["Latitude", "Longitude", "Heading"],
    colors: ["#5b8def", "#4ec88c", "#e8b84a"],
  },
  rssi: {
    label: "RSSI / Link Quality (%)",
    fields: ["rssi"],
    names:  ["RSSI"],
    colors: ["#4ec88c"],
  },
};

export const TS_MAX_POINTS = 4000;
