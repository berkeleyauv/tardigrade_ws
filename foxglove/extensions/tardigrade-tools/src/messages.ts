export type RosTime = { sec: number; nanosec?: number; nsec?: number };

export type PidDebug = {
  stamp: RosTime;
  axis: string;
  setpoint: number;
  measurement: number;
  error: number;
  kp: number;
  ki: number;
  kd: number;
  p_term: number;
  i_term: number;
  d_term: number;
  output: number;
  integral_limit: number;
  output_limit: number;
  saturated: boolean;
};

export type BoolMsg = { data: boolean };

export type Quaternion = { x: number; y: number; z: number; w: number };

export type Odometry = {
  pose: { pose: { position: { x: number; y: number; z: number }; orientation: Quaternion } };
  twist: {
    twist: {
      linear: { x: number; y: number; z: number };
      angular: { x: number; y: number; z: number };
    };
  };
};

export type ThrusterCommands = { names: string[]; setpoints: number[] };

export type AllocationStatus = { feasible: boolean; residual: number[] };

export type RobotStatus = {
  control_connected: boolean;
  armed: boolean;
  external_control_enabled: boolean;
  detail: string;
};

export type EspState = {
  armed: boolean;
  state_valid: boolean;
  altitude_valid: boolean;
  link_ok: boolean;
  pose_ok: boolean;
  depth: number;
};

export function timeToSec(time: RosTime | undefined): number {
  return time == undefined ? 0 : Number(time.sec) + Number(time.nanosec ?? time.nsec ?? 0) / 1e9;
}

export function finite(value: unknown, fallback = 0): number {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function quaternionToRpy(q: Quaternion): [number, number, number] {
  const norm = Math.hypot(q.x, q.y, q.z, q.w);
  if (!Number.isFinite(norm) || norm < 1e-9) {
    return [0, 0, 0];
  }
  const x = q.x / norm;
  const y = q.y / norm;
  const z = q.z / norm;
  const w = q.w / norm;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinPitch = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sinPitch);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}
