import { useMemo } from 'react';
import { Color } from 'three';

import { useRobot } from '../context';
import type { WorldShape } from '../ros/robotData';
import { CUT_HEIGHT, settings } from '../settings';
import type { WallMode } from '../settings';
import { useStore } from '../store';
import { FrameGroup } from './FrameGroup';

/** The simulated apartment (walls, furniture, dock), from the /sim/world markers. */
export function World() {
  const { data } = useRobot();
  const shapes = useStore(data.world);
  const { walls } = useStore(settings);
  const byFrame = useMemo(() => {
    const groups = new Map<string, WorldShape[]>();
    for (const s of shapes) groups.set(s.frame, [...(groups.get(s.frame) ?? []), s]);
    return [...groups];
  }, [shapes]);
  return (
    <>
      {byFrame.map(([frame, list]) => (
        <FrameGroup key={frame} frame={frame}>
          {list.map((s) => (
            <Shape key={s.key} shape={s} walls={walls} />
          ))}
        </FrameGroup>
      ))}
    </>
  );
}

function isWall(s: WorldShape): boolean {
  return s.key.toLowerCase().includes('wall');
}

function Shape({ shape, walls }: { shape: WorldShape; walls: WallMode }) {
  const wall = isWall(shape);
  let [x, y, z] = shape.position;
  let [sx, sy, sz] = shape.size;
  const level = Math.abs(shape.quaternion[0]) < 1e-3 && Math.abs(shape.quaternion[1]) < 1e-3;
  if (wall && walls === 'off') return null;
  if (wall && walls === 'cut' && level) {
    const bottom = z - sz / 2;
    const top = Math.min(z + sz / 2, CUT_HEIGHT);
    if (top <= bottom) return null;
    sz = top - bottom;
    z = (top + bottom) / 2;
  }
  const [r, g, b, a] = shape.color;
  return (
    <group position={[x, y, z]} quaternion={shape.quaternion}>
      {/* three.js cylinders stand along Y; ROS cylinders stand along Z. */}
      <mesh rotation={shape.kind === 'cylinder' ? [Math.PI / 2, 0, 0] : [0, 0, 0]} castShadow receiveShadow>
        {shape.kind === 'box' ? (
          <boxGeometry args={[sx, sy, sz]} />
        ) : (
          <cylinderGeometry args={[sx / 2, sx / 2, sz, 24]} />
        )}
        <meshStandardMaterial color={new Color(r, g, b)} transparent={a < 1} opacity={a} roughness={0.85} />
      </mesh>
      {wall && walls === 'cut' ? <WallTop sx={sx} sy={sy} sz={sz} /> : null}
    </group>
  );
}

/** A light cap on cut walls so the cut edge reads clearly from above. */
function WallTop({ sx, sy, sz }: { sx: number; sy: number; sz: number }) {
  return (
    <mesh position={[0, 0, sz / 2 + 0.001]}>
      <planeGeometry args={[sx, sy]} />
      <meshBasicMaterial color="#e8e4dc" />
    </mesh>
  );
}
