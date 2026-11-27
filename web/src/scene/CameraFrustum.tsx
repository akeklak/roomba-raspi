import { useFrame } from '@react-three/fiber';
import { useMemo, useRef, useState } from 'react';
import { BufferAttribute, BufferGeometry, DataTexture, LineBasicMaterial, LineSegments, RGBAFormat, SRGBColorSpace } from 'three';
import type { Mesh, MeshBasicMaterial } from 'three';

import { useRobot } from '../context';
import type { CameraModel } from '../ros/robotData';
import { FrameGroup } from './FrameGroup';

/** How far in front of the camera the picture is drawn, in meters. */
const DEPTH = 0.3;

/**
 * The camera's field of view as a wireframe pyramid, with the live picture on its far end. Drawn
 * in the camera's optical frame (z forward, x right, y down), so it shows what the camera sees
 * from where it sits.
 */
export function CameraFrustum() {
  const { data } = useRobot();
  const [frame, setFrame] = useState<string | null>(null);
  const [model, setModel] = useState<CameraModel | null>(null);
  useFrame(() => {
    const cam = data.camera;
    if (cam && cam.frame !== frame) setFrame(cam.frame);
    if (data.cameraModel && data.cameraModel !== model) setModel(data.cameraModel);
  });
  if (!frame || !model) return null;
  return (
    <FrameGroup frame={frame}>
      <Picture model={model} />
    </FrameGroup>
  );
}

function Picture({ model }: { model: CameraModel }) {
  const { data } = useRobot();
  const w = (DEPTH * model.width) / model.fx;
  const h = (DEPTH * model.height) / model.fy;
  // Image center offset from the optical axis.
  const ox = (DEPTH * (model.width / 2 - model.cx)) / model.fx;
  const oy = (DEPTH * (model.height / 2 - model.cy)) / model.fy;

  const lines = useMemo(() => {
    const corners = [
      [ox - w / 2, oy - h / 2],
      [ox + w / 2, oy - h / 2],
      [ox + w / 2, oy + h / 2],
      [ox - w / 2, oy + h / 2],
    ];
    const pts: number[] = [];
    corners.forEach(([x, y], i) => {
      const [nx, ny] = corners[(i + 1) % 4];
      pts.push(0, 0, 0, x, y, DEPTH, x, y, DEPTH, nx, ny, DEPTH);
    });
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array(pts), 3));
    return new LineSegments(g, new LineBasicMaterial({ color: '#63b3ed', transparent: true, opacity: 0.8 }));
  }, [w, h, ox, oy]);

  const texture = useRef<DataTexture | null>(null);
  const plane = useRef<Mesh>(null);
  const seen = useRef(-1);
  useFrame(() => {
    const cam = data.camera;
    if (!cam || cam.version === seen.current || !plane.current) return;
    seen.current = cam.version;
    let tex = texture.current;
    if (!tex || tex.image.width !== cam.width || tex.image.height !== cam.height) {
      tex?.dispose();
      tex = new DataTexture(cam.rgba, cam.width, cam.height, RGBAFormat);
      // Row 0 is the top of the picture, which is -y in the optical frame: no flip needed.
      tex.flipY = false;
      tex.colorSpace = SRGBColorSpace;
      texture.current = tex;
      const mat = plane.current.material as MeshBasicMaterial;
      mat.map = tex;
      mat.needsUpdate = true;
    } else {
      tex.image.data = cam.rgba;
    }
    tex.needsUpdate = true;
  });

  return (
    <group>
      <primitive object={lines} />
      <mesh ref={plane} position={[ox, oy, DEPTH]}>
        <planeGeometry args={[w, h]} />
        <meshBasicMaterial side={2} transparent opacity={0.85} toneMapped={false} />
      </mesh>
    </group>
  );
}
