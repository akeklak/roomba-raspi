/**
 * Occupancy grids from the robot: the map (from SLAM while mapping, or the saved map) and Nav2's
 * costmap. They arrive as binary frames (see roomba_web/protocol.py):
 *   [2][uint32 LE header length][JSON header][zlib-compressed cells]
 * One byte per cell, row by row starting at the origin corner: 0..100 = free..occupied,
 * 255 = unknown.
 */

export const GRID_FRAME = 2;
export const UNKNOWN = 255;

export type GridKind = 'map' | 'costmap';

export type GridHeader = {
  kind: GridKind;
  frame: string;
  stamp: number;
  width: number;
  height: number;
  /** Meters per cell. */
  resolution: number;
  /** Pose of the corner of cell (0, 0) in `frame`. */
  origin: { x: number; y: number; z: number; q: [number, number, number, number] };
};

export type Grid = GridHeader & { cells: Uint8Array };

/** Splits a grid frame into its header and still-compressed cells. Null if it isn't one. */
export function parseGridFrame(data: ArrayBuffer): { header: GridHeader; compressed: Uint8Array } | null {
  const bytes = new Uint8Array(data);
  if (bytes.length < 5 || bytes[0] !== GRID_FRAME) return null;
  const length = new DataView(data).getUint32(1, true);
  if (5 + length > bytes.length) return null;
  const header = JSON.parse(new TextDecoder().decode(bytes.subarray(5, 5 + length))) as GridHeader;
  return { header, compressed: bytes.subarray(5 + length) };
}

/** Inflates zlib data (Python's zlib.compress) with the browser's own decompressor. */
export async function inflate(compressed: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([compressed as BlobPart]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

export async function decodeGrid(data: ArrayBuffer): Promise<Grid | null> {
  const parsed = parseGridFrame(data);
  if (!parsed) return null;
  const cells = await inflate(parsed.compressed);
  if (cells.length !== parsed.header.width * parsed.header.height) return null;
  return { ...parsed.header, cells };
}

type RGBA = [number, number, number, number];

// Map colors on the dark floor: free space a little lighter than the floor, walls bright.
const MAP_FREE: RGBA = [150, 170, 195, 70];
const MAP_OCCUPIED: RGBA = [236, 232, 224, 245];

/** RGBA pixels (same layout as the cells) for drawing the map. Unknown is transparent. */
export function mapPixels(cells: Uint8Array, out: Uint8Array = new Uint8Array(cells.length * 4)): Uint8Array {
  for (let i = 0; i < cells.length; i++) {
    const v = cells[i];
    const o = i * 4;
    if (v === UNKNOWN) {
      out[o + 3] = 0;
      continue;
    }
    // Mix from free to occupied (slam_toolbox only sends 0 or 100; a loaded map can have more).
    const k = Math.min(v, 100) / 100;
    out[o] = MAP_FREE[0] + (MAP_OCCUPIED[0] - MAP_FREE[0]) * k;
    out[o + 1] = MAP_FREE[1] + (MAP_OCCUPIED[1] - MAP_FREE[1]) * k;
    out[o + 2] = MAP_FREE[2] + (MAP_OCCUPIED[2] - MAP_FREE[2]) * k;
    out[o + 3] = MAP_FREE[3] + (MAP_OCCUPIED[3] - MAP_FREE[3]) * k;
  }
  return out;
}

/**
 * RGBA pixels for Nav2's costmap: nothing where it's free, then a violet haze that thickens
 * toward obstacles, orange where the robot's middle can't go (99) and red at obstacles (100).
 */
export function costmapPixels(cells: Uint8Array, out: Uint8Array = new Uint8Array(cells.length * 4)): Uint8Array {
  for (let i = 0; i < cells.length; i++) {
    const v = cells[i];
    const o = i * 4;
    if (v === 0 || v === UNKNOWN) {
      out[o + 3] = 0;
    } else if (v >= 100) {
      out.set([245, 80, 90, 200], o);
    } else if (v >= 99) {
      out.set([246, 160, 70, 150], o);
    } else {
      const k = v / 98;
      out.set([150 + 70 * k, 90, 230 - 60 * k, 25 + 95 * k], o);
    }
  }
  return out;
}

/** Cell value at a point given in the grid's own frame, or null outside the grid. */
export function cellAt(grid: Grid, x: number, y: number): number | null {
  const [qx, qy, qz, qw] = grid.origin.q;
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  const dx = x - grid.origin.x;
  const dy = y - grid.origin.y;
  const c = Math.cos(-yaw);
  const s = Math.sin(-yaw);
  const col = Math.floor((c * dx - s * dy) / grid.resolution);
  const row = Math.floor((s * dx + c * dy) / grid.resolution);
  if (col < 0 || row < 0 || col >= grid.width || row >= grid.height) return null;
  return grid.cells[row * grid.width + col];
}
