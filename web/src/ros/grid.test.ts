import { describe, expect, it } from 'vitest';

import { COSTMAP_RED, UNKNOWN, cellAt, costmapPixels, decodeGrid, mapPixels, parseGridFrame } from './grid';
import type { Grid, GridHeader } from './grid';

const HEADER: GridHeader = {
  kind: 'map',
  frame: 'map',
  stamp: 1,
  width: 3,
  height: 2,
  resolution: 0.5,
  origin: { x: -1, y: 2, z: 0, q: [0, 0, 0, 1] },
};

async function deflate(data: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([data as BlobPart]).stream().pipeThrough(new CompressionStream('deflate'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

/** Builds a frame the way roomba_web/protocol.py grid_frame() does. */
function frame(header: GridHeader, compressed: Uint8Array): ArrayBuffer {
  const json = new TextEncoder().encode(JSON.stringify(header));
  const buf = new Uint8Array(5 + json.length + compressed.length);
  buf[0] = 2;
  new DataView(buf.buffer).setUint32(1, json.length, true);
  buf.set(json, 5);
  buf.set(compressed, 5 + json.length);
  return buf.buffer;
}

describe('grid frames', () => {
  it('decode to the header and the cells', async () => {
    const cells = new Uint8Array([0, 100, UNKNOWN, 50, 0, 0]);
    const grid = (await decodeGrid(frame(HEADER, await deflate(cells))))!;
    expect(grid.width).toBe(3);
    expect(grid.origin.x).toBe(-1);
    expect(Array.from(grid.cells)).toEqual(Array.from(cells));
  });

  it('are rejected when they are something else or the size is wrong', async () => {
    expect(parseGridFrame(new Uint8Array([1, 0, 0, 0, 0]).buffer)).toBeNull();
    expect(parseGridFrame(new Uint8Array([2, 99, 0, 0, 0]).buffer)).toBeNull();
    expect(await decodeGrid(frame(HEADER, await deflate(new Uint8Array(5))))).toBeNull();
  });
});

describe('grid colors', () => {
  it('leave unknown map cells clear and draw walls brighter than free space', () => {
    const px = mapPixels(new Uint8Array([UNKNOWN, 0, 100]));
    expect(px[3]).toBe(0);
    expect(px[7]).toBeGreaterThan(0);
    expect(px[11]).toBeGreaterThan(px[7]);
    expect(px[8]).toBeGreaterThan(px[4]);
  });

  it('leave free costmap cells clear and fade bright red in toward obstacles', () => {
    const px = costmapPixels(new Uint8Array([0, UNKNOWN, 1, 30, 98, 99, 100]));
    expect(px[3]).toBe(0);
    expect(px[7]).toBe(0);
    const alpha = [2, 3, 4, 5, 6].map((i) => px[i * 4 + 3]);
    expect(alpha[0]).toBeGreaterThan(0);
    for (let i = 1; i < alpha.length; i++) expect(alpha[i]).toBeGreaterThan(alpha[i - 1]);
    expect(alpha[4]).toBe(255); // solid at obstacles
    expect(alpha[2]).toBeLessThan(128); // where it can still drive stays see-through
    expect(alpha[3]).toBeGreaterThan(200); // where its middle can't go is nearly solid
    for (const i of [2, 3, 4, 5, 6]) expect([...px.subarray(i * 4, i * 4 + 3)]).toEqual([...COSTMAP_RED]);
  });
});

describe('cellAt', () => {
  const grid: Grid = { ...HEADER, cells: new Uint8Array([0, 1, 2, 3, 4, 5]) };

  it('finds the cell under a point, rows starting at the origin', () => {
    expect(cellAt(grid, -0.9, 2.1)).toBe(0);
    expect(cellAt(grid, 0.2, 2.1)).toBe(2);
    expect(cellAt(grid, -0.4, 2.6)).toBe(4);
    expect(cellAt(grid, -1.1, 2.1)).toBeNull();
    expect(cellAt(grid, 0.6, 2.1)).toBeNull();
  });

  it('follows a turned grid', () => {
    // Turned a quarter turn left: columns run along +y, rows along -x.
    const s = Math.SQRT1_2;
    const turned: Grid = { ...grid, origin: { x: 0, y: 0, z: 0, q: [0, 0, s, s] } };
    expect(cellAt(turned, -0.1, 1.2)).toBe(2);
    expect(cellAt(turned, -0.6, 0.1)).toBe(3);
  });
});
