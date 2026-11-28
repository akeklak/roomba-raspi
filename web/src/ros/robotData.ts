import { Matrix4, Quaternion, Vector3 } from 'three';

import { Store } from '../store';
import type { CameraMessage, Message, RobotConnection } from './connection';
import { decodeGrid } from './grid';
import type { Grid, GridKind } from './grid';
import { PlaybackClock } from './playback';
import { scanToPoints } from './scan';
import type { LaserScan } from './scan';
import { TfTree, cleanFrame } from './tf';
import { VoxelMemory } from './voxels';

/** Frame names. The sim and the real robot use the same ones. */
export const BASE_FRAME = 'base_footprint';
/** Navigation goals and the robot's position on the map are given in this frame. */
export const MAP_FRAME = 'map';
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

export type NavStateName = 'idle' | 'starting' | 'active' | 'succeeded' | 'canceled' | 'failed';

/** What Nav2 is doing, as the web bridge reports it. */
export type NavState = {
  state: NavStateName;
  goal: { x: number; y: number; yaw: number; frame: string } | null;
  /** Meters left along the path, and seconds Nav2 expects it to take. */
  distance: number | null;
  eta: number | null;
  recoveries: number;
  message: string;
};

/** map_manager's state: mapping, or navigating on the saved map. */
export type MappingState = {
  mode: 'mapping' | 'navigation' | 'off';
  /** What it is doing right now (saving, switching...), or null. */
  busy: string | null;
  message: string;
  error: string | null;
  /** Mapping on top of the saved map rather than from scratch. */
  extending: boolean;
  /** Nav2 is up, so the robot can be sent places. */
  nav_ready: boolean;
  map_name: string;
  saved: { saved_at: number; width_m: number; height_m: number; can_extend: boolean } | null;
};

export type MapCommand = 'save_map' | 'use_saved_map' | 'new_map' | 'extend_map';

export type Notice = { id: number; level: 'info' | 'error'; text: string; at: number };

export type Pose2D = { x: number; y: number; yaw: number };

const IDLE_NAV: NavState = { state: 'idle', goal: null, distance: null, eta: null, recoveries: 0, message: '' };

const TRAIL_CAPACITY = 20_000;
const TRAIL_STEP = 0.02;
/** Two samples of a moving transform further apart than this mean data was missing in between. */
const MAX_TF_GAP_SEC = 0.5;
/** How long a scan may wait for the transforms that place it before it's left out of the memory. */
const MAX_MEMORY_WAIT_MS = 1000;
const MEMORY_QUEUE_MAX = 30;

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
  readonly nav = new Store<NavState>(IDLE_NAV);
  readonly mapping = new Store<MappingState | null>(null);
  readonly notice = new Store<Notice | null>(null);
  /** Size of the current map, for the UI (the cells themselves are in `grids`). */
  readonly mapSize = new Store<{ width: number; height: number } | null>(null);

  /** Newest map and costmap. The 3D scene polls the versions. */
  grids: Record<GridKind, { grid: Grid | null; version: number }> = {
    map: { grid: null, version: 0 },
    costmap: { grid: null, version: 0 },
  };
  /** The path Nav2 is following, in `plan.frame`. */
  plan = { frame: MAP_FRAME, positions: new Float32Array(0), version: 0 };

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
  private costmapWanted = false;
  private topSpeed: number | null = null;
  private decodingGrid = false;
  private pendingGrids: ArrayBuffer[] = [];
  private noticeId = 0;
  private decoding = false;
  private pendingCamera: CameraMessage | null = null;
  private retiredImages: ImageBitmap[] = [];
  private scanTimes: number[] = [];
  /** Scans waiting for the transforms that place them, oldest first (see rememberScans). */
  private memoryQueue: Array<{ scan: LaserScan; at: number }> = [];
  private scratch = new Matrix4();
  /** The robot's clock, played back smoothly for drawing (see drawPose). */
  private playback = new PlaybackClock();

  constructor(private readonly connection: RobotConnection) {}

  start(): void {
    const c = this.connection;
    this.unsubscribe.push(
      c.on('hello', () => {
        c.send({ op: 'camera', enabled: this.cameraWanted });
        c.send({ op: 'costmap', enabled: this.costmapWanted });
        if (this.topSpeed !== null) c.send({ op: 'top_speed', speed: this.topSpeed });
      }),
      c.on('tf', (m) => this.onTf(m)),
      c.on('scan', (m) => this.onScan(m as unknown as LaserScan)),
      c.on('odom', (m) => this.onOdom(m)),
      c.on('world', (m) => this.world.set(m.shapes as WorldShape[])),
      c.on('battery', (m) => this.status.update({ battery: m.charge })),
      c.on('charging', (m) => this.status.update({ chargingState: m.state })),
      c.on('camera', (m) => this.onCamera(m as CameraMessage)),
      c.on('grid', (m) => this.onGrid(m.data as ArrayBuffer)),
      c.on('nav', (m) => this.nav.set({ ...IDLE_NAV, ...(m as Partial<NavState>) })),
      c.on('mapping', (m) => this.mapping.set(m as unknown as MappingState)),
      c.on('plan', (m) => this.onPlan(m)),
      c.on('notice', (m) => this.notice.set({ id: ++this.noticeId, level: m.level, text: m.text, at: Date.now() })),
      c.on('disconnected', () => {
        this.tf.clear();
        this.playback.reset();
        this.memoryQueue = [];
        this.status.update({ scanHz: null, speed: null, turnRate: null });
        this.nav.set(IDLE_NAV);
        this.mapping.set(null);
        this.onPlan({ op: 'plan', frame: MAP_FRAME, points: [] });
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

  /** Nav2's costmap is only sent while something shows it. */
  setCostmapEnabled(enabled: boolean): void {
    this.costmapWanted = enabled;
    this.connection.send({ op: 'costmap', enabled });
    if (!enabled) {
      this.grids.costmap = { grid: null, version: this.grids.costmap.version + 1 };
    }
  }

  /** The speed slider: the robot's top speed in m/s, for Go to as well as driving by hand. */
  setTopSpeed(speed: number): void {
    this.topSpeed = speed;
    this.connection.send({ op: 'top_speed', speed });
  }

  /** Drive to this pose on the map. */
  sendGoal(goal: Pose2D): boolean {
    return this.connection.send({ op: 'goal', x: goal.x, y: goal.y, yaw: goal.yaw });
  }

  cancelNav(): void {
    this.connection.send({ op: 'cancel' });
  }

  /** Tells localization where the robot really is on the map. */
  setInitialPose(pose: Pose2D): boolean {
    return this.connection.send({ op: 'initial_pose', x: pose.x, y: pose.y, yaw: pose.yaw });
  }

  mapCommand(command: MapCommand): boolean {
    return this.connection.send({ op: 'map_command', command });
  }

  /** Converts a pose in the frame the scene is drawn in to the map frame. Null without a map. */
  toMap(pose: Pose2D): Pose2D | null {
    return this.convert(pose, MAP_FRAME, this.fixedFrame());
  }

  /** Converts a pose on the map to the frame the scene is drawn in. */
  fromMap(pose: Pose2D): Pose2D | null {
    return this.convert(pose, this.fixedFrame(), MAP_FRAME);
  }

  /** The robot's pose on the map, newest data. */
  robotOnMap(): Pose2D | null {
    const m = this.tf.lookup(MAP_FRAME, BASE_FRAME, undefined, this.scratch);
    if (!m) return null;
    const e = m.elements;
    return { x: e[12], y: e[13], yaw: Math.atan2(e[1], e[0]) };
  }

  private convert(pose: Pose2D, target: string, source: string): Pose2D | null {
    const m = this.tf.lookup(target, source, undefined, this.scratch);
    if (!m) return null;
    const p = new Vector3(pose.x, pose.y, 0).applyMatrix4(m);
    const e = m.elements;
    return { x: p.x, y: p.y, yaw: pose.yaw + Math.atan2(e[1], e[0]) };
  }

  fixedFrame(): string {
    return this.status.get().fixedFrame;
  }

  /** Pose of `frame` in the fixed frame, newest data. */
  poseInFixed(frame: string, out?: Matrix4): Matrix4 | null {
    return this.tf.lookup(this.fixedFrame(), frame, undefined, out);
  }

  /**
   * Pose of `frame` in the fixed frame for drawing this frame of the 3D view: a moment behind
   * the newest data, interpolated, so the robot glides instead of jumping with each update.
   */
  drawPose(frame: string, out?: Matrix4): Matrix4 | null {
    return this.tf.lookup(this.fixedFrame(), frame, this.playback.now(performance.now() / 1000), out);
  }

  clearMemory(): void {
    this.memory.clear();
    this.memoryQueue = [];
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
      if (cleanFrame(tr.child) === BASE_FRAME) {
        movedBase = true;
        if (!tr.static) this.playback.observe(tr.stamp, performance.now() / 1000);
      }
    }
    if (this.tf.version !== versionBefore) this.pickFixedFrame();
    if (movedBase) this.extendTrail();
    this.rememberScans();
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
    p.set([e[12], e[13], 0.016], t.count * 3);
    t.count++;
    t.version++;
  }

  private onScan(scan: LaserScan): void {
    const now = performance.now();
    this.scanTimes.push(now);
    while (this.scanTimes.length > 1 && this.scanTimes[0] < now - 2000) this.scanTimes.shift();
    // Show it right away, placed with the newest data, so it moves with the robot model.
    const toFixed = this.tf.lookup(this.fixedFrame(), scan.frame, scan.stamp, this.scratch);
    if (toFixed) {
      const { positions, ranges } = scanToPoints(scan, toFixed);
      this.scan = { positions, ranges, version: this.scan.version + 1 };
    }
    this.memoryQueue.push({ scan, at: now });
    if (this.memoryQueue.length > MEMORY_QUEUE_MAX) this.memoryQueue.shift();
    this.rememberScans();
    if (this.scanTimes.length > 2) {
      const span = (this.scanTimes[this.scanTimes.length - 1] - this.scanTimes[0]) / 1000;
      const scanHz = Math.round(((this.scanTimes.length - 1) / span) * 10) / 10;
      if (scanHz !== this.status.get().scanHz) this.status.update({ scanHz });
    }
  }

  /**
   * Adds waiting scans to the memory once the robot's position at the moment each was taken is
   * known. A scan often arrives a little before the odometry that goes with it; placing it with
   * the position from just before (as the live scan is) puts it a few degrees off while the robot
   * turns, and the memory would keep that forever. Scans whose position can't be known (the
   * robot was switching maps, or the data stopped) are left out of the memory.
   */
  private rememberScans(): void {
    const q = this.memoryQueue;
    const fixed = this.fixedFrame();
    const now = performance.now();
    while (q.length) {
      const { scan, at } = q[0];
      const coverage = this.tf.coverage(fixed, scan.frame, scan.stamp, MAX_TF_GAP_SEC);
      if (coverage === 'wait' && now - at < MAX_MEMORY_WAIT_MS) break;
      q.shift();
      if (coverage !== 'ok') continue;
      const toFixed = this.tf.lookup(fixed, scan.frame, scan.stamp, this.scratch);
      if (toFixed) this.memory.add(scanToPoints(scan, toFixed).positions);
    }
  }

  private onOdom(msg: Message): void {
    const s = this.status.get();
    if (msg.speed !== s.speed || msg.turn !== s.turnRate) this.status.update({ speed: msg.speed, turnRate: msg.turn });
  }

  private onPlan(msg: Message): void {
    const points = (msg.points ?? []) as Array<[number, number]>;
    const positions = new Float32Array(points.length * 3);
    points.forEach(([x, y], i) => positions.set([x, y, 0.022], i * 3));
    this.plan = { frame: msg.frame ?? MAP_FRAME, positions, version: this.plan.version + 1 };
  }

  /** Decodes grids one at a time, keeping only the newest of each kind that is waiting. */
  private onGrid(data: ArrayBuffer): void {
    if (this.decodingGrid) {
      this.pendingGrids.push(data);
      if (this.pendingGrids.length > 4) this.pendingGrids.shift();
      return;
    }
    this.decodingGrid = true;
    decodeGrid(data)
      .then((grid) => {
        if (!grid) return;
        if (grid.kind === 'costmap' && !this.costmapWanted) return;
        const slot = this.grids[grid.kind];
        this.grids[grid.kind] = { grid, version: slot.version + 1 };
        if (grid.kind === 'map') this.mapSize.set({ width: grid.width * grid.resolution, height: grid.height * grid.resolution });
      })
      .catch((err) => console.warn('Could not decode a map', err))
      .finally(() => {
        this.decodingGrid = false;
        const next = this.pendingGrids.shift();
        if (next) this.onGrid(next);
      });
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
