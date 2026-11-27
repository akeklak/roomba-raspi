import { Matrix4, Quaternion, Vector3 } from 'three';

import { Store } from '../store';
import type { RobotConnection } from './connection';
import { imageToRgba } from './image';
import type { RosImage } from './image';
import { scanToPoints } from './scan';
import type { LaserScan } from './scan';
import { TfTree, cleanFrame } from './tf';
import { toSec } from './time';
import { VoxelMemory } from './voxels';

/** Topic and frame names. The sim and the real robot use the same ones. */
export const TOPICS = {
  tf: '/tf',
  tfStatic: '/tf_static',
  scan: '/scan',
  odom: '/odom',
  cmdVel: '/cmd_vel',
  image: '/camera/image_raw',
  cameraInfo: '/camera/camera_info',
  world: '/sim/world',
  battery: '/battery/charge_ratio',
  charging: '/battery/charging_state',
} as const;

export const BASE_FRAME = 'base_footprint';
/** Preferred frames to draw everything in, best first. "world" only exists in the sim. */
const FIXED_FRAME_PREFERENCE = ['world', 'map', 'odom'];

export type WorldShape = {
  key: string;
  frame: string;
  kind: 'box' | 'cylinder';
  position: [number, number, number];
  quaternion: [number, number, number, number];
  size: [number, number, number];
  color: [number, number, number, number];
};

export type RobotStatus = {
  fixedFrame: string;
  /** Forward speed and turn rate from odometry. */
  speed: number | null;
  turnRate: number | null;
  battery: number | null;
  chargingState: number | null;
  scanHz: number | null;
};

export type CameraFrame = {
  rgba: Uint8ClampedArray;
  width: number;
  height: number;
  frame: string;
  version: number;
};

export type CameraModel = { fx: number; fy: number; cx: number; cy: number; width: number; height: number };

const TRAIL_CAPACITY = 20_000;
const TRAIL_STEP = 0.02;

/**
 * Everything the UI knows about the robot. High-rate data (transforms, scans, images) lives in
 * plain fields with version counters that the 3D scene polls every frame; low-rate data that
 * React renders lives in Stores.
 */
export class RobotData {
  readonly tf = new TfTree();
  readonly status = new Store<RobotStatus>({
    fixedFrame: 'odom',
    speed: null,
    turnRate: null,
    battery: null,
    chargingState: null,
    scanHz: null,
  });
  readonly world = new Store<WorldShape[]>([]);
  readonly memory = new VoxelMemory();

  /** Latest scan, already placed in the fixed frame. */
  scan: { positions: Float32Array; ranges: Float32Array; version: number } = {
    positions: new Float32Array(0),
    ranges: new Float32Array(0),
    version: 0,
  };
  trail = { positions: new Float32Array(TRAIL_CAPACITY * 3), count: 0, version: 0 };
  camera: CameraFrame | null = null;
  cameraModel: CameraModel | null = null;

  private unsubscribe: Array<() => void> = [];
  private cameraUnsubscribe: Array<() => void> = [];
  private scanTimes: number[] = [];
  private scratch = new Matrix4();

  constructor(private readonly connection: RobotConnection) {}

  start(): void {
    const c = this.connection;
    this.unsubscribe.push(
      c.on(TOPICS.tf, (m) => this.onTf(m, false)),
      c.on(TOPICS.tfStatic, (m) => this.onTf(m, true)),
      c.on(TOPICS.scan, (m) => this.onScan(m)),
      c.on(TOPICS.odom, (m) => this.onOdom(m)),
      c.on(TOPICS.world, (m) => this.onWorld(m)),
      c.on(TOPICS.battery, (m) => this.status.update({ battery: m.data })),
      c.on(TOPICS.charging, (m) => this.status.update({ chargingState: m.state })),
      c.info.subscribe(() => {
        if (c.info.get().status === 'disconnected') this.tf.clear();
      }),
    );
  }

  stop(): void {
    this.unsubscribe.forEach((u) => u());
    this.unsubscribe = [];
    this.setCameraEnabled(false);
  }

  /** The camera is the heaviest stream, so only subscribe while something shows it. */
  setCameraEnabled(enabled: boolean): void {
    if (enabled && this.cameraUnsubscribe.length === 0) {
      this.cameraUnsubscribe.push(
        this.connection.on(TOPICS.image, (m) => this.onImage(m)),
        this.connection.on(TOPICS.cameraInfo, (m) => this.onCameraInfo(m)),
      );
    } else if (!enabled) {
      this.cameraUnsubscribe.forEach((u) => u());
      this.cameraUnsubscribe = [];
    }
  }

  fixedFrame(): string {
    return this.status.get().fixedFrame;
  }

  /** Pose of `frame` in the fixed frame, newest data. */
  poseInFixed(frame: string, out?: Matrix4): Matrix4 | null {
    return this.tf.lookup(this.fixedFrame(), frame, undefined, out);
  }

  clearMemory(): void {
    this.memory.clear();
    this.trail.count = 0;
    this.trail.version++;
  }

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private onTf(msg: any, isStatic: boolean): void {
    const versionBefore = this.tf.version;
    let movedBase = false;
    for (const tr of msg.transforms) {
      const { translation: t, rotation: q } = tr.transform;
      this.tf.set(
        tr.header.frame_id,
        tr.child_frame_id,
        toSec(tr.header.stamp),
        new Vector3(t.x, t.y, t.z),
        new Quaternion(q.x, q.y, q.z, q.w),
        isStatic,
      );
      if (cleanFrame(tr.child_frame_id) === BASE_FRAME) movedBase = true;
    }
    if (this.tf.version !== versionBefore) this.pickFixedFrame();
    if (movedBase) this.extendTrail();
  }

  private pickFixedFrame(): void {
    const best = FIXED_FRAME_PREFERENCE.find((f) => this.tf.has(f)) ?? 'odom';
    if (best !== this.fixedFrame()) {
      this.status.update({ fixedFrame: best });
      this.clearMemory();
    }
  }

  private extendTrail(): void {
    const pose = this.poseInFixed(BASE_FRAME, this.scratch);
    if (!pose) return;
    const e = pose.elements;
    const t = this.trail;
    const p = t.positions;
    if (t.count > 0) {
      const i = (t.count - 1) * 3;
      if (Math.hypot(e[12] - p[i], e[13] - p[i + 1]) < TRAIL_STEP) return;
    }
    if (t.count === TRAIL_CAPACITY) {
      p.copyWithin(0, (TRAIL_CAPACITY / 2) * 3);
      t.count = TRAIL_CAPACITY / 2;
    }
    p.set([e[12], e[13], 0.005], t.count * 3);
    t.count++;
    t.version++;
  }

  private onScan(scan: LaserScan): void {
    const now = performance.now();
    this.scanTimes.push(now);
    while (this.scanTimes.length > 1 && this.scanTimes[0] < now - 2000) this.scanTimes.shift();
    // Place the scan where the robot was when it was taken, so it lines up with the walls.
    const toFixed = this.tf.lookup(this.fixedFrame(), scan.header.frame_id, toSec(scan.header.stamp), this.scratch);
    if (!toFixed) return;
    const { positions, ranges } = scanToPoints(scan, toFixed);
    this.scan = { positions, ranges, version: this.scan.version + 1 };
    this.memory.add(positions);
    if (this.scanTimes.length > 2) {
      const span = (this.scanTimes[this.scanTimes.length - 1] - this.scanTimes[0]) / 1000;
      const scanHz = Math.round(((this.scanTimes.length - 1) / span) * 10) / 10;
      if (scanHz !== this.status.get().scanHz) this.status.update({ scanHz });
    }
  }

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private onOdom(msg: any): void {
    const speed = Math.round(msg.twist.twist.linear.x * 100) / 100;
    const turnRate = Math.round(msg.twist.twist.angular.z * 100) / 100;
    const s = this.status.get();
    if (speed !== s.speed || turnRate !== s.turnRate) this.status.update({ speed, turnRate });
  }

  private onImage(msg: RosImage): void {
    const prev = this.camera;
    const rgba = imageToRgba(msg, prev?.rgba);
    if (!rgba) return;
    this.camera = {
      rgba,
      width: msg.width,
      height: msg.height,
      frame: cleanFrame(msg.header.frame_id),
      version: (prev?.version ?? 0) + 1,
    };
  }

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private onCameraInfo(msg: any): void {
    const k = msg.k as ArrayLike<number>;
    if (!k || !k[0]) return;
    const m = this.cameraModel;
    if (m && m.fx === k[0] && m.width === msg.width) return;
    this.cameraModel = { fx: k[0], fy: k[4], cx: k[2], cy: k[5], width: msg.width, height: msg.height };
  }

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private onWorld(msg: any): void {
    const shapes: WorldShape[] = [];
    for (const m of msg.markers) {
      if (m.action !== 0) continue; // ADD only
      if (m.type !== 1 && m.type !== 3) continue; // CUBE, CYLINDER
      const { position: p, orientation: q } = m.pose;
      shapes.push({
        key: `${m.ns}/${m.id}`,
        frame: cleanFrame(m.header.frame_id),
        kind: m.type === 1 ? 'box' : 'cylinder',
        position: [p.x, p.y, p.z],
        quaternion: [q.x, q.y, q.z, q.w],
        size: [m.scale.x, m.scale.y, m.scale.z],
        color: [m.color.r, m.color.g, m.color.b, m.color.a],
      });
    }
    this.world.set(shapes);
  }
}
