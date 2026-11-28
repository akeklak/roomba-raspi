import { Matrix4, Quaternion, Vector3 } from 'three';

/** One stamped parent -> child transform. */
type Sample = { stamp: number; t: Vector3; q: Quaternion };

type Edge = { parent: string; isStatic: boolean; samples: Sample[] };

/** How much dynamic-transform history to keep, in seconds. */
const HISTORY_SEC = 2;
/** A stamp this far behind the newest one means the clock restarted (e.g. the sim was relaunched). */
const CLOCK_JUMP_SEC = 1;
const MAX_DEPTH = 64;

export function cleanFrame(frame: string): string {
  return frame.startsWith('/') ? frame.slice(1) : frame;
}

/**
 * The robot's transform tree, built from /tf and /tf_static. Keeps a short history of each moving
 * transform so a sensor reading can be placed where the robot was when it was taken.
 */
export class TfTree {
  private edges = new Map<string, Edge>();
  private known = new Set<string>();
  /** Bumped whenever the set of frames changes. */
  version = 0;

  set(parent: string, child: string, stamp: number, t: Vector3, q: Quaternion, isStatic: boolean): void {
    parent = cleanFrame(parent);
    child = cleanFrame(child);
    let edge = this.edges.get(child);
    if (!edge || edge.parent !== parent || edge.isStatic !== isStatic) {
      edge = { parent, isStatic, samples: [] };
      this.edges.set(child, edge);
      this.version++;
    }
    for (const f of [parent, child]) {
      if (!this.known.has(f)) {
        this.known.add(f);
        this.version++;
      }
    }
    const sample = { stamp, t: t.clone(), q: q.clone().normalize() };
    if (isStatic) {
      edge.samples = [sample];
      return;
    }
    const s = edge.samples;
    const last = s[s.length - 1];
    if (last && stamp < last.stamp - CLOCK_JUMP_SEC) s.length = 0;
    if (last && stamp < last.stamp) {
      // Out of order but recent: insert in place.
      let i = s.length - 1;
      while (i > 0 && s[i - 1].stamp > stamp) i--;
      s.splice(i, 0, sample);
    } else {
      s.push(sample);
    }
    const newest = s[s.length - 1].stamp;
    let drop = 0;
    while (drop < s.length - 2 && s[drop].stamp < newest - HISTORY_SEC) drop++;
    if (drop) s.splice(0, drop);
  }

  has(frame: string): boolean {
    return this.known.has(cleanFrame(frame));
  }

  frames(): string[] {
    return [...this.known].sort();
  }

  clear(): void {
    this.edges.clear();
    this.known.clear();
    this.version++;
  }

  /**
   * The pose of `source` expressed in `target` (the matrix that maps points in `source` into
   * `target`), at `time` in seconds or the newest data when `time` is omitted. Null when the two
   * frames are not connected.
   */
  lookup(target: string, source: string, time?: number, out = new Matrix4()): Matrix4 | null {
    target = cleanFrame(target);
    source = cleanFrame(source);
    const a = this.toRoot(source, time);
    const b = this.toRoot(target, time);
    if (!a || !b || a.root !== b.root) return null;
    return out.copy(b.matrix).invert().multiply(a.matrix);
  }

  /**
   * Whether `lookup(target, source, time)` would be exact rather than a guess:
   *   'ok'   every moving transform between the two has samples on both sides of `time`, at most
   *          `maxGap` seconds apart;
   *   'wait' one of them has nothing at or after `time` yet (its next sample is still on the way);
   *   'no'   never: the frames aren't connected, `time` is older than the history kept, or a
   *          transform has a hole around `time` (it stopped and started again, as when the robot
   *          switches from mapping to the saved map).
   */
  coverage(target: string, source: string, time: number, maxGap: number): 'ok' | 'wait' | 'no' {
    const a = this.chain(cleanFrame(source));
    const b = this.chain(cleanFrame(target));
    if (!a || !b || a[a.length - 1] !== b[b.length - 1]) return 'no';
    // Only the transforms below the frames' lowest common ancestor matter.
    while (a.length > 1 && b.length > 1 && a[a.length - 2] === b[b.length - 2]) {
      a.pop();
      b.pop();
    }
    let result: 'ok' | 'wait' = 'ok';
    for (const frame of [...a.slice(0, -1), ...b.slice(0, -1)]) {
      const edge = this.edges.get(frame)!;
      if (edge.isStatic) continue;
      const s = edge.samples;
      const newest = s[s.length - 1].stamp;
      if (time > newest) {
        // The next sample will be at or after `time`, so too far from this one already means a hole.
        if (time - newest > maxGap) return 'no';
        result = 'wait';
        continue;
      }
      if (time < s[0].stamp) return 'no';
      let hi = 0;
      while (s[hi].stamp < time) hi++;
      if (hi > 0 && s[hi].stamp - s[hi - 1].stamp > maxGap) return 'no';
    }
    return result;
  }

  /** The frames from `frame` up to its root, or null if unknown or in a cycle. */
  private chain(frame: string): string[] | null {
    if (!this.known.has(frame)) return null;
    const out = [frame];
    for (let depth = 0; depth < MAX_DEPTH; depth++) {
      const edge = this.edges.get(out[out.length - 1]);
      if (!edge) return out;
      out.push(edge.parent);
    }
    return null;
  }

  private toRoot(frame: string, time?: number): { root: string; matrix: Matrix4 } | null {
    if (!this.known.has(frame)) return null;
    const matrix = new Matrix4();
    const step = new Matrix4();
    let f = frame;
    for (let depth = 0; depth < MAX_DEPTH; depth++) {
      const edge = this.edges.get(f);
      if (!edge) return { root: f, matrix };
      const { t, q } = sampleAt(edge.samples, time);
      step.compose(t, q, ONE);
      matrix.premultiply(step);
      f = edge.parent;
    }
    return null; // a cycle
  }
}

const ONE = new Vector3(1, 1, 1);

function sampleAt(samples: Sample[], time?: number): { t: Vector3; q: Quaternion } {
  const last = samples[samples.length - 1];
  if (time === undefined || samples.length === 1 || time >= last.stamp) return last;
  const first = samples[0];
  if (time <= first.stamp) return first;
  let hi = 1;
  while (samples[hi].stamp < time) hi++;
  const a = samples[hi - 1];
  const b = samples[hi];
  const k = b.stamp > a.stamp ? (time - a.stamp) / (b.stamp - a.stamp) : 1;
  return { t: a.t.clone().lerp(b.t, k), q: a.q.clone().slerp(b.q, k) };
}
