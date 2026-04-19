declare module "../pkg/propwash_web.js" {
  export default function init(options?: { module_or_path: string }): Promise<void>;
  export function add_file(data: Uint8Array, filename: string): string;
  export function clear_workspace(): void;
  export function get_timeseries(file_id: number, session_idx: number, max_points: number, field_list: string): string;
  export function get_spectrogram(file_id: number, session_idx: number, axis_list: string): string;
  export function get_filter_config(file_id: number, session_idx: number): string;
  export function get_raw_frames(file_id: number, session_idx: number, start: number, count: number, field_list: string): string;
  export function get_trend(): string;
  export function get_step_overlay(file_id: number, session_idx: number): string;
}
