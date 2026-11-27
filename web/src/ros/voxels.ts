/**
 * Remembers every spot the lidar has hit, snapped to a grid so repeated hits don't pile up.
 * This is what makes the walls "build up" as the robot drives around. A spot is only shown once
 * `minScans` different scans have hit it, so a scan that was placed a little wrong (or a stray
 * reflection) doesn't leave dots behind; real walls get hit again and again. Oldest points are
 * dropped once `capacity` is reached.
 */
export class VoxelMemory {
  readonly positions: Float32Array;
  count = 0;
  version = 0;
  private next = 0;
  private keys: Float64Array;
  private index = new Map<number, number>();
  /** Spots hit by fewer than `minScans` scans so far: key -> [hits, last scan number]. */
  private candidates = new Map<number, [number, number]>();
  private scans = 0;

  constructor(
    readonly cellSize = 0.03,
    readonly capacity = 200_000,
    readonly minScans = 2,
  ) {
    this.positions = new Float32Array(capacity * 3);
    this.keys = new Float64Array(capacity);
  }

  /** Adds the xyz triples of one scan. Returns how many spots appeared. */
  add(points: ArrayLike<number>): number {
    let added = 0;
    const s = this.cellSize;
    const scan = ++this.scans;
    for (let i = 0; i + 2 < points.length; i += 3) {
      const ix = Math.floor(points[i] / s);
      const iy = Math.floor(points[i + 1] / s);
      const iz = Math.floor(points[i + 2] / s);
      const key = ((ix + 32768) * 65536 + (iy + 32768)) * 1024 + (iz + 512);
      if (this.index.has(key)) continue;
      if (this.minScans > 1) {
        const c = this.candidates.get(key);
        if (!c) {
          this.candidates.set(key, [1, scan]);
          continue;
        }
        if (c[1] === scan) continue;
        c[0]++;
        c[1] = scan;
        if (c[0] < this.minScans) continue;
        this.candidates.delete(key);
      }
      const slot = this.next;
      if (this.count === this.capacity) this.index.delete(this.keys[slot]);
      this.keys[slot] = key;
      this.index.set(key, slot);
      this.positions[slot * 3] = (ix + 0.5) * s;
      this.positions[slot * 3 + 1] = (iy + 0.5) * s;
      this.positions[slot * 3 + 2] = (iz + 0.5) * s;
      this.next = (slot + 1) % this.capacity;
      this.count = Math.min(this.count + 1, this.capacity);
      added++;
    }
    // One-off hits pile up as candidates; forget them now and then so they don't grow forever.
    if (this.candidates.size > this.capacity) this.candidates.clear();
    if (added) this.version++;
    return added;
  }

  clear(): void {
    this.index.clear();
    this.candidates.clear();
    this.count = 0;
    this.next = 0;
    this.version++;
  }
}
