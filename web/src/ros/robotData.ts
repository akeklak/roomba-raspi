import { Matrix4, Quaternion, Vector3 } from 'three';

import { Store } from '../store';
import type { CameraMessage, Message, RobotConnection } from './connection';
import { scanToPoints } from './scan';
import type { LaserScan } from './scan';
import { TfTree, cleanFrame } from './tf';
import { VoxelMemory } from './voxels';

/** Frame names. The sim and the real robot use the same ones. */
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
  image: ImageBitmap;
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
  private cameraWanted = false;
  private decoding = false;
  private pendingCamera: CameraMessage | null = null;
  private retiredImages: ImageBitmap[] = [];
  private scanTimes: number[] = [];
  private scratch = new Matrix4();

  constructor(private readonly connection: RobotConnection) {}

  start(): void {
    const c = this.connection;
    this.unsubscribe.push(
      c.on('hello', () => c.send({ op: 'camera', enabled: this.cameraWanted })),
      c.on('tf', (m) => this.onTf(m)),
      c.on('scan', (m) => this.onScan(m as unknown as LaserScan)),
      c.on('odom', (m) => this.onOdom(m)),
      c.on('world', (m) => this.world.set(m.shapes as WorldShape[])),
      c.on('battery', (m) => this.status.update({ battery: m.charge })),
      c.on('charging', (m) => this.status.update({ chargingState: m.state })),
      c.on('camera', (m) => this.onCamera(m as CameraMessage)),
      c.on('disconnected', () => {
        this.tf.clear();
        this.status.update({ scanHz: null, speed: null, turnRate: null });
      }),
    );
  }

  stop(): void {
    this.unsubscribe.forEach((u) => u());
    this.unsubscribe = [];
  }

  /** The camera is the heaviest stream, so the robot only sends it while something shows it. */
  setCameraEnabled(enabled: boolean): void {
    this.cameraWanted = enabled;
    this.connection.send({ op: 'camera', enabled });
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

  private onTf(msg: Message): void {
    const versionBefore = this.tf.version;
    let movedBase = false;
    for (const tr of msg.transforms as TransformJson[]) {
      this.tf.set(
        tr.parent,
        tr.child,
        tr.stamp,
        new Vector3(tr.t[0], tr.t[1], tr.t[2]),
        new Quaternion(tr.q[0], tr.q[1], tr.q[2], tr.q[3]),
        tr.static,
      );
      if (cleanFrame(tr.child) === BASE_FRAME) movedBase = true;
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
    const toFixed = this.tf.lookup(this.fixedFrame(), scan.frame, scan.stamp, this.scratch);
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

  private onOdom(msg: Message): void {
    const s = this.status.get();
    if (msg.speed !== s.speed || msg.turn !== s.turnRate) this.status.update({ speed: msg.speed, turnRate: msg.turn });
  }

  /** Decodes JPEG pictures one at a time; if they arrive faster, only the newest is kept. */
  private onCamera(msg: CameraMessage): void {
    if (msg.header.model) this.cameraModel = msg.header.model;
    if (this.decoding) {
      this.pendingCamera = msg;
      return;
    }
    this.decoding = true;
    createImageBitmap(msg.jpeg)
      .then((image) => {
        const prev = this.camera;
        if (prev) this.retiredImages.push(prev.image);
        // Keep the last couple around: the 3D view may not have uploaded them to the GPU yet.
        while (this.retiredImages.length > 2) this.retiredImages.shift()?.close();
        this.camera = {
          image,
          width: msg.header.width,
          height: msg.header.height,
          frame: cleanFrame(msg.header.frame),
          version: (prev?.version ?? 0) + 1,
        };
      })
      .catch((err) => console.warn('Could not decode a camera picture', err))
      .finally(() => {
        this.decoding = false;
        const next = this.pendingCamera;
        this.pendingCamera = null;
        if (next) this.onCamera(next);
      });
  }
}

type TransformJson = {
  parent: string;
  child: string;
  t: [number, number, number];
  q: [number, number, number, number];
  stamp: number;
  static: boolean;
};
