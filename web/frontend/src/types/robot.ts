export interface RobotStatus {
  mode: "simulation" | "hardware";
  connected: boolean;
  estop_active: boolean;
  safety_stop: boolean;
  uptime_sec: number;
  cpu_percent: number;
  ram_percent: number;
  active_streams: number;
  timestamp_sec: number;
}

export interface WheelTelemetry {
  names: string[];
  target_rad_s: number[];
  measured_rad_s: number[];
  encoder_ticks: number[];
  pwm_commands: number[];
}

export interface ImuTelemetry {
  roll_deg: number;
  pitch_deg: number;
  yaw_deg: number;
  accel_x: number;
  accel_y: number;
  accel_z: number;
  gyro_x: number;
  gyro_y: number;
  gyro_z: number;
  qx: number;
  qy: number;
  qz: number;
  qw: number;
}

export interface LidarTelemetry {
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  points: [number, number][];
}

export interface OdometryTelemetry {
  x: number;
  y: number;
  theta_rad: number;
  vx: number;
  vy: number;
  wz: number;
}

export interface StreamInfo {
  id: string;
  name: string;
  source: string;
  destination: string;
  topic: string;
  message_type: string;
  target_frequency_hz: number;
  actual_frequency_hz: number;
  packet_count: number;
  last_timestamp_sec: number;
  latency_ms: number;
  status: "active" | "degraded" | "stale" | "offline";
  payload_preview?: string | null;
}

export interface RobotConfig {
  wheel_radius_m: number;
  wheelbase_m: number;
  track_width_m: number;
  max_wheel_speed_rad_s: number;
  max_linear_speed_mps: number;
  max_angular_speed_rad_s: number;
  command_timeout_sec: number;
  control_frequency_hz: number;
  telemetry_frequency_hz: number;
  motor_kp: number;
  motor_ki: number;
  motor_kd: number;
  debug_telemetry: boolean;
}

export interface PathTelemetry {
  global_path: [number, number][];
  local_path: [number, number][];
  obstacles?: [number, number][];
}

export interface DebugTelemetry {
  raw_wheel_speed_rad_s: number[];
  filtered_wheel_speed_rad_s: number[];
  target_wheel_speed_rad_s: number[];
  motor_output: number[];
  imu_quaternion_xyzw: number[];
  body_vx_mps: number;
  body_vy_mps: number;
  body_wz_rad_s: number;
  cmd_vx_mps: number;
  cmd_vy_mps: number;
  cmd_wz_rad_s: number;
}

export interface FullTelemetryMessage {
  type: "telemetry";
  status: RobotStatus;
  wheels: WheelTelemetry;
  imu: ImuTelemetry;
  odom: OdometryTelemetry;
  lidar: LidarTelemetry;
  streams: StreamInfo[];
  paths?: PathTelemetry;
  debug?: DebugTelemetry;
}


