// Response interfaces for the WASM bridge JSON outputs

// -- Workspace types --

export interface WorkspaceFile {
  file_id: number;
  filename: string;
  sessions: SessionResult[];
  warnings: string[];
}

export interface SessionRef {
  fileId: number;
  sessionIdx: number;
  label: string;
}

// -- Legacy single-file result (returned by analyze(), same shape as WorkspaceFile) --

export interface AnalysisResult {
  file_id: number;
  filename: string;
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
  is_truncated: boolean;
  corrupt_bytes: number;
  analysis: FlightAnalysis;
}

export interface FlightAnalysis {
  summary: FlightSummary;
  events: FlightEvent[];
  vibration: VibrationAnalysis | null;
  step_response: StepResponseAnalysis | null;
  pid: PidAnalysis | null;
  diagnostics: Diagnostic[];
}

export interface StepResponseAnalysis {
  axes: AxisStepResponse[];
}

export interface AxisStepResponse {
  axis: string;
  step_count: number;
  rise_time_ms: number;
  overshoot_percent: number;
  settling_time_ms: number;
}

export interface PidAnalysis {
  windup: AxisWindup[];
  oscillation: AxisOscillation[];
  tuning: TuningSuggestion[];
}

export interface AxisWindup {
  axis: string;
  i_dominant_fraction: number;
  peak_ratio: number;
}

export interface AxisOscillation {
  axis: string;
  frequency_hz: number | null;
  magnitude_db: number | null;
  overshoot_percent: number;
}

export interface TuningSuggestion {
  axis: string;
  rating: string;
  current: AxisGains;
  suggested: AxisGains;
  overshoot_percent: number;
  rise_time_ms: number;
  settling_time_ms: number;
  step_count: number;
}

export interface AxisGains {
  p: number | null;
  i: number | null;
  d: number | null;
}

export interface StepOverlay {
  axes: StepOverlayAxis[];
}

export interface StepOverlayAxis {
  axis: string;
  time_ms: number[];
  setpoint_steps: number[][];
  gyro_steps: number[][];
  gyro_average: number[];
}

export interface FlightSummary {
  total_events: number;
  desyncs: number;
  gyro_spikes: number;
  motor_saturations: number;
  overshoots: number;
  motor_count: number;
  motor_balance: MotorStats[];
}

export interface MotorStats {
  index: number;
  mean: number;
  deviation_percent: number;
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
  avg_motor_hz: number | null;
  accel: AccelData | null;
  propwash: PropwashAnalysis | null;
}

export interface PropwashAnalysis {
  spectra: Spectrum[];
  chop_count: number;
  dominant_frequency_hz: number | null;
  dominant_magnitude_db: number | null;
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
  avg_motor_hz: number | null;
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

export interface TrendPoint {
  label: string;
  duration_seconds: number;
  sample_rate_hz: number;
  frame_count: number;
  noise_floor_db: [number, number, number] | null;
  motor_balance_max_deviation: number | null;
  step_response_rise_ms: number | null;
  step_response_overshoot: number | null;
  total_events: number;
  desyncs: number;
  diagnostic_count: number;
}

export interface RawFramesResponse {
  start: number;
  total: number;
  field_names: string[];
  frames: number[][];
  error?: string;
}
