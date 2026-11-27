import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import { BufferAttribute, BufferGeometry, CanvasTexture, Color, DynamicDrawUsage, LinearFilter } from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';

import { useRobot } from '../context';

const MAX_SCAN_POINTS = 4096;
/** Ranges at or past this distance get the "far" color. */
const FAR_RANGE = 6;

let dotTexture: CanvasTexture | null = null;
/** A soft round dot, so points draw as circles instead of squares. */
function getDot(): CanvasTexture {
  if (dotTexture) return dotTexture;
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d')!;
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.6, 'rgba(255,255,255,1)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  dotTexture = new CanvasTexture(c);
  // No mipmaps: from far away (the top view) a dot is only a pixel or two wide, and the blurred
  // smaller copies are so faint that alphaTest throws every dot away.
  dotTexture.generateMipmaps = false;
  dotTexture.minFilter = LinearFilter;
  return dotTexture;
}

/** Color for a lidar hit: warm when close, cool when far. */
export function rangeColor(range: number, out: Color): Color {
  const k = Math.min(Math.max(range / FAR_RANGE, 0), 1);
  return out.setHSL(0.02 + k * 0.55, 0.95, 0.55);
}

/** The newest lidar scan, placed in 3D where the robot was when it took it. */
export function LidarScan() {
  const { data } = useRobot();
  const geometry = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array(MAX_SCAN_POINTS * 3), 3).setUsage(DynamicDrawUsage));
    g.setAttribute('color', new BufferAttribute(new Float32Array(MAX_SCAN_POINTS * 3), 3).setUsage(DynamicDrawUsage));
    g.setDrawRange(0, 0);
    return g;
  }, []);
  const seen = useRef(-1);
  const color = useMemo(() => new Color(), []);
  useFrame(() => {
    const scan = data.scan;
    if (scan.version === seen.current) return;
    seen.current = scan.version;
    const n = Math.min(scan.ranges.length, MAX_SCAN_POINTS);
    const pos = geometry.getAttribute('position') as BufferAttribute;
    const col = geometry.getAttribute('color') as BufferAttribute;
    (pos.array as Float32Array).set(scan.positions.subarray(0, n * 3));
    const c = col.array as Float32Array;
    for (let i = 0; i < n; i++) {
      rangeColor(scan.ranges[i], color);
      c[i * 3] = color.r;
      c[i * 3 + 1] = color.g;
      c[i * 3 + 2] = color.b;
    }
    pos.needsUpdate = true;
    col.needsUpdate = true;
    geometry.setDrawRange(0, n);
    geometry.computeBoundingSphere();
  });
  return (
    <points geometry={geometry} frustumCulled={false} renderOrder={2}>
      <pointsMaterial size={0.06} vertexColors map={getDot()} alphaTest={0.4} transparent depthWrite={false} />
    </points>
  );
}

/** Every spot the lidar has hit so far: the walls "build up" as the robot explores. */
export function LidarMemory() {
  const { data } = useRobot();
  const memory = data.memory;
  const geometry = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(memory.positions, 3).setUsage(DynamicDrawUsage));
    g.setDrawRange(0, 0);
    return g;
  }, [memory]);
  const uploaded = useRef({ version: -1, count: 0 });
  useFrame(() => {
    const u = uploaded.current;
    if (memory.version === u.version) return;
    const pos = geometry.getAttribute('position') as BufferAttribute;
    pos.clearUpdateRanges();
    if (memory.count > u.count && memory.count < memory.capacity) {
      pos.addUpdateRange(u.count * 3, (memory.count - u.count) * 3);
    }
    pos.needsUpdate = true;
    geometry.setDrawRange(0, memory.count);
    u.version = memory.version;
    u.count = memory.count;
  });
  return (
    <points geometry={geometry} frustumCulled={false} renderOrder={1}>
      <pointsMaterial size={0.035} color="#4fd1c5" map={getDot()} alphaTest={0.4} transparent opacity={0.75} depthWrite={false} />
    </points>
  );
}

/** Where the robot has driven, on the floor. */
export function Trail() {
  const { data } = useRobot();
  const line = useMemo(() => {
    const material = new LineMaterial({ color: 0xf6ad55, linewidth: 3, transparent: true, opacity: 0.85 });
    const l = new Line2(new LineGeometry(), material);
    l.frustumCulled = false;
    l.visible = false;
    return l;
  }, []);
  const seen = useRef({ version: -1, at: 0 });
  useFrame(({ clock }) => {
    const t = data.trail;
    const s = seen.current;
    // Rebuilding the line is not free, so refresh it a few times a second at most.
    if (t.version === s.version || clock.elapsedTime - s.at < 0.25) return;
    s.version = t.version;
    s.at = clock.elapsedTime;
    line.visible = t.count >= 2;
    if (!line.visible) return;
    const g = new LineGeometry();
    g.setPositions(t.positions.subarray(0, t.count * 3));
    line.geometry.dispose();
    line.geometry = g;
  });
  return <primitive object={line} />;
}
