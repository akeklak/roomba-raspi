import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef, useState } from 'react';
import {
  DataTexture,
  DoubleSide,
  NearestFilter,
  Plane,
  Raycaster,
  RGBAFormat,
  SRGBColorSpace,
  Vector2,
  Vector3,
} from 'three';
import type { Group, Mesh, MeshBasicMaterial } from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';

import { useRobot } from '../context';
import { costmapPixels, mapPixels } from '../ros/grid';
import type { GridKind } from '../ros/grid';
import { BASE_FRAME, MAP_FRAME } from '../ros/robotData';
import { useStore } from '../store';
import { MIN_DRAG, tool, toolDrag } from '../ui/tools';
import type { Tool } from '../ui/tools';
import { FrameGroup } from './FrameGroup';

const ACCENT = '#4fd1c5';
const PLAN_COLOR = '#63b3ed';
const POSE_COLOR = '#f6ad55';

/** A grid (map or costmap) drawn as a picture lying on the floor, in the grid's own frame. */
function GridLayer({ kind, height, renderOrder }: { kind: GridKind; height: number; renderOrder: number }) {
  const { data } = useRobot();
  const [frame, setFrame] = useState(MAP_FRAME);
  const group = useRef<Group>(null);
  const mesh = useRef<Mesh>(null);
  const material = useRef<MeshBasicMaterial>(null);
  const texture = useRef<DataTexture | null>(null);
  const seen = useRef(-1);

  useEffect(() => () => texture.current?.dispose(), []);

  useFrame(() => {
    const slot = data.grids[kind];
    const g = group.current;
    const m = mesh.current;
    const mat = material.current;
    if (!g || !m || !mat || slot.version === seen.current) return;
    seen.current = slot.version;
    const grid = slot.grid;
    m.visible = grid !== null;
    if (!grid) return;
    if (grid.frame !== frame) setFrame(grid.frame);
    let t = texture.current;
    if (!t || t.image.width !== grid.width || t.image.height !== grid.height) {
      t?.dispose();
      t = new DataTexture(new Uint8Array(grid.width * grid.height * 4), grid.width, grid.height, RGBAFormat);
      t.magFilter = NearestFilter;
      t.minFilter = NearestFilter;
      t.generateMipmaps = false;
      t.colorSpace = SRGBColorSpace;
      texture.current = t;
      mat.map = t;
      mat.needsUpdate = true;
    }
    const pixels = t.image.data as Uint8Array;
    if (kind === 'map') mapPixels(grid.cells, pixels);
    else costmapPixels(grid.cells, pixels);
    t.needsUpdate = true;
    // Cell (0, 0) starts at the origin; the picture's first row is the grid's first row.
    const w = grid.width * grid.resolution;
    const h = grid.height * grid.resolution;
    m.scale.set(w, h, 1);
    m.position.set(w / 2, h / 2, 0);
    g.position.set(grid.origin.x, grid.origin.y, grid.origin.z + height);
    g.quaternion.set(...grid.origin.q);
  });

  return (
    <FrameGroup frame={frame}>
      <group ref={group}>
        <mesh ref={mesh} visible={false} renderOrder={renderOrder}>
          <planeGeometry args={[1, 1]} />
          <meshBasicMaterial ref={material} transparent depthWrite={false} toneMapped={false} />
        </mesh>
      </group>
    </FrameGroup>
  );
}

/** The map: free space slightly lighter than the floor, walls bright, unknown left clear. */
export function MapLayer() {
  return <GridLayer kind="map" height={0.003} renderOrder={-2} />;
}

/** Nav2's costmap: obstacles grown by the robot's size, where it plans not to go. */
export function CostmapLayer() {
  return <GridLayer kind="costmap" height={0.006} renderOrder={-1} />;
}

/** The path Nav2 is following. */
export function PlanLine() {
  const { data } = useRobot();
  const [frame, setFrame] = useState(MAP_FRAME);
  const line = useMemo(() => {
    const material = new LineMaterial({ color: PLAN_COLOR, linewidth: 4, transparent: true, opacity: 0.95 });
    const l = new Line2(new LineGeometry(), material);
    l.frustumCulled = false;
    l.visible = false;
    l.renderOrder = 3;
    return l;
  }, []);
  const seen = useRef(-1);
  useFrame(() => {
    const plan = data.plan;
    if (plan.version === seen.current) return;
    seen.current = plan.version;
    if (plan.frame !== frame) setFrame(plan.frame);
    line.visible = plan.positions.length >= 6;
    if (!line.visible) return;
    const g = new LineGeometry();
    g.setPositions(plan.positions);
    line.geometry.dispose();
    line.geometry = g;
  });
  return (
    <FrameGroup frame={frame}>
      <primitive object={line} />
    </FrameGroup>
  );
}

/** A robot-sized ring with an arrow, lying on the floor at a pose. */
function PoseMarker({ color, opacity = 1 }: { color: string; opacity?: number }) {
  return (
    <group>
      <mesh position={[0, 0, 0.008]} renderOrder={4}>
        <ringGeometry args={[0.15, 0.19, 48]} />
        <meshBasicMaterial color={color} transparent opacity={opacity} depthWrite={false} toneMapped={false} side={DoubleSide} />
      </mesh>
      <mesh position={[0.27, 0, 0.008]} rotation={[0, 0, -Math.PI / 2]} renderOrder={4}>
        <circleGeometry args={[0.07, 3]} />
        <meshBasicMaterial color={color} transparent opacity={opacity} depthWrite={false} toneMapped={false} side={DoubleSide} />
      </mesh>
      {/* A thin beam so the spot is easy to find from far away. */}
      <mesh position={[0, 0, 0.3]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.012, 0.012, 0.6, 8]} />
        <meshBasicMaterial color={color} transparent opacity={0.45 * opacity} depthWrite={false} toneMapped={false} />
      </mesh>
    </group>
  );
}

/** Where the robot is heading, while it is navigating. */
export function GoalMarker() {
  const { data } = useRobot();
  const nav = useStore(data.nav);
  const show = nav.goal && (nav.state === 'starting' || nav.state === 'active');
  if (!show || !nav.goal) return null;
  return (
    <FrameGroup frame={nav.goal.frame || MAP_FRAME}>
      <group position={[nav.goal.x, nav.goal.y, 0]} rotation={[0, 0, nav.goal.yaw]}>
        <PoseMarker color={ACCENT} />
      </group>
    </FrameGroup>
  );
}

/** The spot being picked with a tool, in the scene's own frame. */
export function ToolPreview() {
  const t = useStore(tool);
  const ref = useRef<Group>(null);
  useFrame(() => {
    const g = ref.current;
    if (!g) return;
    g.visible = toolDrag.active;
    g.position.set(toolDrag.x, toolDrag.y, 0);
    g.rotation.set(0, 0, toolDrag.yaw);
  });
  if (t === 'none') return null;
  return (
    <group ref={ref} visible={false}>
      <PoseMarker color={t === 'pose' ? POSE_COLOR : ACCENT} opacity={0.8} />
    </group>
  );
}

/**
 * While a tool is picked: press on the floor to choose a spot, drag to choose a direction, let go
 * to send it. Without a drag, a goal faces away from the robot and a position keeps its heading.
 */
export function FloorTool() {
  const { data } = useRobot();
  const current = useStore(tool);
  const camera = useThree((s) => s.camera);
  const canvas = useThree((s) => s.gl.domElement);

  useEffect(() => {
    if (current === 'none') return;
    const raycaster = new Raycaster();
    const floor = new Plane(new Vector3(0, 0, 1), 0);
    const ndc = new Vector2();
    const hit = new Vector3();
    let start: Vector3 | null = null;
    let pointer = -1;

    const floorPoint = (e: PointerEvent): Vector3 | null => {
      const r = canvas.getBoundingClientRect();
      ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      raycaster.setFromCamera(ndc, camera);
      return raycaster.ray.intersectPlane(floor, hit) ? hit.clone() : null;
    };
    const defaultYaw = (p: Vector3, which: Tool): number => {
      const robot = data.poseInFixed(BASE_FRAME);
      if (!robot) return 0;
      const e = robot.elements;
      if (which === 'pose') return Math.atan2(e[1], e[0]);
      return Math.atan2(p.y - e[13], p.x - e[12]);
    };
    const down = (e: PointerEvent) => {
      if (e.button !== 0) return;
      const p = floorPoint(e);
      if (!p) return;
      start = p;
      pointer = e.pointerId;
      Object.assign(toolDrag, { active: true, x: p.x, y: p.y, yaw: defaultYaw(p, current), hasHeading: false });
    };
    const move = (e: PointerEvent) => {
      if (!start || e.pointerId !== pointer) return;
      const p = floorPoint(e);
      if (!p || p.distanceTo(start) < MIN_DRAG) return;
      toolDrag.yaw = Math.atan2(p.y - start.y, p.x - start.x);
      toolDrag.hasHeading = true;
    };
    const up = (e: PointerEvent) => {
      if (!start || e.pointerId !== pointer) return;
      const onMap = data.toMap({ x: toolDrag.x, y: toolDrag.y, yaw: toolDrag.yaw });
      if (onMap) {
        if (current === 'goal') data.sendGoal(onMap);
        else data.setInitialPose(onMap);
      }
      start = null;
      toolDrag.active = false;
      tool.set('none');
    };
    const cancel = () => {
      start = null;
      toolDrag.active = false;
    };
    const key = (e: KeyboardEvent) => {
      if (e.code === 'Escape') {
        cancel();
        tool.set('none');
      }
    };
    canvas.addEventListener('pointerdown', down);
    canvas.addEventListener('pointermove', move);
    canvas.addEventListener('pointerup', up);
    canvas.addEventListener('pointercancel', cancel);
    window.addEventListener('keydown', key);
    canvas.style.cursor = 'crosshair';
    return () => {
      canvas.removeEventListener('pointerdown', down);
      canvas.removeEventListener('pointermove', move);
      canvas.removeEventListener('pointerup', up);
      canvas.removeEventListener('pointercancel', cancel);
      window.removeEventListener('keydown', key);
      canvas.style.cursor = '';
      toolDrag.active = false;
    };
  }, [current, camera, canvas, data]);

  return null;
}
