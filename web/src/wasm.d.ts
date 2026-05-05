declare module "../pkg/propwash_web.js" {
  export type Axis = "Roll" | "Pitch" | "Yaw";
  export type RcChannel = "Roll" | "Pitch" | "Yaw" | "Throttle";
  export type MotorIndex = number;

  export type SensorField =
    | "Time"
    | "Vbat"
    | "Altitude"
    | "GpsSpeed"
    | "GpsLat"
    | "GpsLng"
    | "Heading"
    | "Rssi"
    | { Gyro: Axis }
    | { GyroUnfilt: Axis }
    | { Setpoint: Axis }
    | { Accel: Axis }
    | { PidP: Axis }
    | { PidI: Axis }
    | { PidD: Axis }
    | { Feedforward: Axis }
    | { Motor: MotorIndex }
    | { ERpm: MotorIndex }
    | { Rc: RcChannel }
    | { Unknown: string };

  export type SensorFields = SensorField[];

  export default function init(options?: { module_or_path: string }): Promise<void>;
  export function add_file(data: Uint8Array, filename: string): string;
  export function clear_workspace(): void;
  export function get_timeseries(file_id: number, session_idx: number, max_points: number, field_list: SensorFields): string;
  export function get_spectrogram(file_id: number, session_idx: number, axis_list: SensorFields): string;
  export function get_filter_config(file_id: number, session_idx: number): string;
  export function get_raw_frames(file_id: number, session_idx: number, start: number, count: number, field_list: SensorFields): string;
  export function get_trend(): string;
  export function get_step_overlay(file_id: number, session_idx: number): string;
}
