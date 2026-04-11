declare module "../pkg/propwash_web.js" {
  export default function init(): Promise<void>;
  export function analyze(data: Uint8Array): string;
  export function get_timeseries(session_idx: number, max_points: number, field_list: string): string;
  export function get_spectrogram(session_idx: number, axis_list: string): string;
  export function get_filter_config(session_idx: number): string;
  export function get_raw_frames(session_idx: number, start: number, count: number, field_list: string): string;
}
