import { OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import { Matrix4, Vector3 } from 'three';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';

import { useRobot } from '../context';
import { BASE_FRAME } from '../ros/robotData';
import type { WorldShape } from '../ros/robotData';
import { settings } from '../settings';
import { useStore } from '../store';

const FOLLOW_OFFSET = new Vector3(-1.1, -1.5, 2.3);
const TOP_HEIGHT = 9;

function worldArea(shapes: WorldShape[], fixedFrame: string): { x: number; y: number; size: number } | null {
  const inFrame = shapes.filter((s) => s.frame === fixedFrame);
  if (inFrame.length === 0) return null;
  let [minX, minY, maxX, maxY] = [Infinity, Infinity, -Infinity, -Infinity];
  for (const s of inFrame) {
    const r = Math.hypot(s.size[0], s.size[1]) / 2;
    minX = Math.min(minX, s.position[0] - r);
    maxX = Math.max(maxX, s.position[0] + r);
    minY = Math.min(minY, s.position[1] - r);
    maxY = Math.max(maxY, s.position[1] + r);
  }
  return { x: (minX + maxX) / 2, y: (minY + maxY) / 2, size: Math.max(maxX - minX, maxY - minY) };
}

/**
 * Camera behavior for each view: Follow keeps your orbit angle but moves with the robot, Free
 * stays put, Top looks straight down, Chase rides behind the robot.
 */
export function CameraRig() {
  const { data } = useRobot();
  const { view } = useStore(settings);
  const controls = useRef<OrbitControlsImpl>(null);
  const camera = useThree((s) => s.camera);
  const pose = useMemo(() => new Matrix4(), []);
  const robot = useMemo(() => new Vector3(), []);
  const heading = useMemo(() => new Vector3(), []);
  const last = useRef<Vector3 | null>(null);
  const placed = useRef(false);

  // Re-aim the camera whenever the view changes.
  useEffect(() => {
    placed.current = false;
  }, [view]);

  useFrame(() => {
    const c = controls.current;
    if (!c) return;
    const p = data.poseInFixed(BASE_FRAME, pose);
    if (p) {
      robot.setFromMatrixPosition(p);
      heading.set(p.elements[0], p.elements[1], 0).normalize();
    }

    if (!placed.current) {
      const center = p ? robot : new Vector3();
      if (view === 'top') {
        // Frame the whole apartment when we know it, otherwise center on the robot.
        const area = worldArea(data.world.get(), data.fixedFrame());
        const height = area ? Math.max(area.size * 0.9, 4) : TOP_HEIGHT;
        const cx = area ? area.x : center.x;
        const cy = area ? area.y : center.y;
        c.target.set(cx, cy, 0);
        camera.position.set(cx, cy - 0.01, height);
      } else if (view !== 'chase') {
        c.target.copy(center);
        camera.position.copy(center).add(FOLLOW_OFFSET);
      }
      last.current = p ? robot.clone() : null;
      placed.current = p !== null || view === 'top' || view === 'free';
      c.update();
    }

    if (!p) return;
    if (view === 'follow') {
      // Move the camera with the robot so your chosen angle and zoom stay the same.
      if (last.current) {
        const delta = robot.clone().sub(last.current);
        camera.position.add(delta);
        c.target.add(delta);
      }
      last.current = robot.clone();
      c.update();
    } else if (view === 'chase') {
      const eye = robot.clone().addScaledVector(heading, -1.2).add(new Vector3(0, 0, 0.75));
      camera.position.lerp(eye, 0.15);
      c.target.lerp(robot.clone().addScaledVector(heading, 0.8).setZ(0.1), 0.15);
      c.update();
    }
  });

  return (
    <OrbitControls
      ref={controls}
      makeDefault
      enableDamping
      dampingFactor={0.12}
      enabled={view !== 'chase'}
      enableRotate={view !== 'top'}
      maxDistance={30}
      minDistance={0.3}
    />
  );
}
