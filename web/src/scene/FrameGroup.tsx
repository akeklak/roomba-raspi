import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import type { ReactNode } from 'react';
import { Matrix4 } from 'three';
import type { Group } from 'three';

import { useRobot } from '../context';

/** Places its children at a TF frame (newest pose), and hides them while that frame is unknown. */
export function FrameGroup({ frame, children }: { frame: string; children: ReactNode }) {
  const ref = useRef<Group>(null);
  const { data } = useRobot();
  const m = useMemo(() => new Matrix4(), []);
  useFrame(() => {
    const g = ref.current;
    if (!g) return;
    const pose = data.poseInFixed(frame, m);
    g.visible = pose !== null;
    if (pose) pose.decompose(g.position, g.quaternion, g.scale);
  });
  return (
    <group ref={ref} visible={false}>
      {children}
    </group>
  );
}
