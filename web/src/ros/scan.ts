import type { Matrix4 } from 'three';

export type LaserScan = {
  frame: string;
  /** Seconds. */
  stamp: number;
  angle_min: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  /** null where the beam hit nothing. */
  ranges: ArrayLike<number | null>;
};

/**
 * Turns a 2D scan into 3D points. `toFixed` maps points from the scan's frame into the frame the
 * scene is drawn in. Returns xyz positions and the range of each point (for coloring).
 */
export function scanToPoints(scan: LaserScan, toFixed: Matrix4): { positions: Float32Array; ranges: Float32Array } {
  const n = scan.ranges.length;
  const positions = new Float32Array(n * 3);
  const ranges = new Float32Array(n);
  const e = toFixed.elements;
  let count = 0;
  for (let i = 0; i < n; i++) {
    const r = scan.ranges[i];
    if (r === null || !Number.isFinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const a = scan.angle_min + i * scan.angle_increment;
    const x = r * Math.cos(a);
    const y = r * Math.sin(a);
    // Column-major 4x4 times (x, y, 0, 1).
    positions[count * 3] = e[0] * x + e[4] * y + e[12];
    positions[count * 3 + 1] = e[1] * x + e[5] * y + e[13];
    positions[count * 3 + 2] = e[2] * x + e[6] * y + e[14];
    ranges[count] = r;
    count++;
  }
  return { positions: positions.subarray(0, count * 3), ranges: ranges.subarray(0, count) };
}
