// Response interfaces for the WASM bridge JSON outputs

export interface AnalysisResult {
  sessions: SessionResult[];
  warnings: string[];
}

export interface SessionResult {
  index: number;
  firmware: string;
  craft: string;
  duration_seconds: number;
  sample_rate_hz: number;
  frame_count: number;
  analysis: FlightAnalysis;
}

export interface FlightAnalysis {
  summary: FlightSummary;
  events: FlightEvent[];
  vibration: VibrationAnalysis | null;
  diagnostics: Diagnostic[];
}

export interface FlightSummary {
  total_events: number;
  desyncs: number;
  gyro_spikes: number;
  motor_saturations: number;
  overshoots: number;
  motor_count: number;
}

export interface FlightEvent {
  time_seconds: number;
  kind: EventKind;
}

export type EventKind =
  | { type: "ThrottleChop"; from_percent: number; to_percent: number; duration_ms: number }
  | { type: "ThrottlePunch"; from_percent: number; to_percent: number; duration_ms: number }
  | { type: "MotorSaturation"; motor_index: number; duration_frames: number }
  | { type: "GyroSpike"; axis: string; magnitude: number }
  | { type: "Overshoot"; axis: string; overshoot_percent: number }
  | { type: "Desync"; motor_index: number; motor_value: number; average_others: number }
  | { type: "FirmwareMessage"; level: string; message: string };

export interface Diagnostic {
  severity: string;
  category: string;
  message: string;
  detail: string;
}

export interface VibrationAnalysis {
  spectra: Spectrum[];
  throttle_bands: ThrottleBand[];
  accel: AccelData | null;
}

export interface Spectrum {
  axis: string;
  frequencies_hz: number[];
  magnitudes_db: number[];
  sample_rate_hz: number;
  peaks: SpectrumPeak[];
}

export interface SpectrumPeak {
  frequency_hz: number;
  magnitude_db: number;
  classification: string | null;
}

export interface ThrottleBand {
  label: string;
  frame_count: number;
  spectra: Spectrum[];
}

export interface AccelData {
  rms: number[];
  spectra: Spectrum[];
}

export interface FilterConfig {
  gyro_lpf_hz: number | null;
  gyro_lpf2_hz: number | null;
  dterm_lpf_hz: number | null;
  gyro_notch1_hz: number | null;
  gyro_notch2_hz: number | null;
  dyn_notch_min_hz: number | null;
  dyn_notch_max_hz: number | null;
  error?: string;
}

export interface TimeseriesResponse {
  time_s: number[];
  fields: Record<string, number[]>;
  error?: string;
}

export interface SpectrogramAxis {
  axis: string;
  time_s: number[];
  frequencies_hz: number[];
  magnitudes_db: number[][];
}

export interface SpectrogramResponse {
  axes: SpectrogramAxis[];
  error?: string;
}

export interface RawFramesResponse {
  start: number;
  total: number;
  field_names: string[];
  frames: number[][];
  error?: string;
}
