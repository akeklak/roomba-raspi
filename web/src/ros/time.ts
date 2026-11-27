export type RosTime = { sec: number; nanosec: number };

export function toSec(t: RosTime | undefined): number {
  return t ? t.sec + t.nanosec * 1e-9 : 0;
}
