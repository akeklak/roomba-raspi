import { useFrame, useLoader } from '@react-three/fiber';
import { Component, Suspense, useMemo, useRef } from 'react';
import type { ReactNode } from 'react';
import type { Mesh, Object3D } from 'three';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';

import bodyUrl from '../../../ros2_ws/src/roomba_bringup/models/roomba_675/meshes/body.dae?url';
import casterUrl from '../../../ros2_ws/src/roomba_bringup/models/roomba_675/meshes/caster.dae?url';
import wheelUrl from '../../../ros2_ws/src/roomba_bringup/models/roomba_675/meshes/drive_wheel.dae?url';
import { BASE_FRAME } from '../ros/robotData';
import { FrameGroup } from './FrameGroup';

/**
 * The Roomba 675 with the lidar and camera, each drawn at its own TF frame. The Roomba is the
 * model made from photos of the real robot (ros2_ws/src/roomba_bringup/models/roomba_675, see
 * description/README.md); a plain stand-in shows while it loads, or if it can't.
 */
export function RobotModel() {
  return (
    <>
      <FrameGroup frame={BASE_FRAME}>
        <MeshOr fallback={<RoombaBody />}>
          <Roomba675 />
        </MeshOr>
        <HeadingMarker />
        <FloorRing />
      </FrameGroup>
      <FrameGroup frame="ldlidar_link">
        <Lidar />
      </FrameGroup>
      <FrameGroup frame="camera_link">
        <CameraHousing />
      </FrameGroup>
    </>
  );
}

const R = 0.17;
const BODY_BOTTOM = 0.01;
const BODY_H = 0.07;
const TOP = BODY_BOTTOM + BODY_H;
/** How far the lidar's optical plane is above the top of the Roomba (0.13 m above the floor). */
const LIDAR_UP = 0.13 - TOP;

// Where the parts sit, from roomba_675.urdf.xacro. The meshes' origin is the floor under the middle
// of the drive axle (base_footprint), +X forward, +Z up.
const WHEEL_Y = 0.1195;
const WHEEL_RADIUS = 0.036;
const CASTER: [number, number, number] = [0.13, 0, 0.012];

/** Its own copy of a loaded COLLADA scene, kept Z-up like ROS (the loader turns it Y-up). */
function zUp(scene: Object3D): Object3D {
  const copy = scene.clone();
  copy.rotation.set(0, 0, 0);
  copy.traverse((o) => {
    o.castShadow = true;
  });
  return copy;
}

function Roomba675() {
  const [body, wheel, caster] = useLoader(ColladaLoader, [bodyUrl, wheelUrl, casterUrl]);
  const parts = useMemo(() => {
    if (!body || !wheel || !caster) throw new Error('a Roomba 675 mesh is not valid COLLADA');
    return {
      body: zUp(body.scene),
      left: zUp(wheel.scene),
      right: zUp(wheel.scene),
      caster: zUp(caster.scene),
    };
  }, [body, wheel, caster]);
  return (
    <group>
      <primitive object={parts.body} />
      <primitive object={parts.left} position={[0, WHEEL_Y, WHEEL_RADIUS]} />
      <primitive object={parts.right} position={[0, -WHEEL_Y, WHEEL_RADIUS]} />
      <primitive object={parts.caster} position={CASTER} />
    </group>
  );
}

/** Shows `fallback` while the children load, and instead of them if they fail to. */
function MeshOr({ fallback, children }: { fallback: ReactNode; children: ReactNode }) {
  return (
    <LoadFailed fallback={fallback}>
      <Suspense fallback={fallback}>{children}</Suspense>
    </LoadFailed>
  );
}

class LoadFailed extends Component<{ fallback: ReactNode; children: ReactNode }, { failed: boolean }> {
  state = { failed: false };

  static getDerivedStateFromError() {
    return { failed: true };
  }

  componentDidCatch(error: unknown) {
    console.warn('Could not load the Roomba 675 model, showing a plain one instead.', error);
  }

  render() {
    return this.state.failed ? this.props.fallback : this.props.children;
  }
}

/** The plain stand-in: a Roomba-sized puck with a bumper. */
function RoombaBody() {
  // Cylinders below are rotated to stand along +Z (three.js builds them along Y).
  const up: [number, number, number] = [Math.PI / 2, 0, 0];
  return (
    <group>
      <mesh position={[0, 0, BODY_BOTTOM + BODY_H / 2]} rotation={up} castShadow>
        <cylinderGeometry args={[R, R, BODY_H, 64]} />
        <meshStandardMaterial color="#3a3e46" roughness={0.6} metalness={0.1} />
      </mesh>
      {/* Top plate */}
      <mesh position={[0, 0, BODY_BOTTOM + BODY_H + 0.002]} rotation={up}>
        <cylinderGeometry args={[R * 0.9, R * 0.9, 0.004, 64]} />
        <meshStandardMaterial color="#6b7380" roughness={0.35} metalness={0.3} />
      </mesh>
      {/* CLEAN button */}
      <mesh position={[0.055, 0, BODY_BOTTOM + BODY_H + 0.005]} rotation={up}>
        <cylinderGeometry args={[0.026, 0.026, 0.004, 32]} />
        <meshStandardMaterial color="#9aa4b2" emissive="#4fd1c5" emissiveIntensity={0.25} />
      </mesh>
      {/* Front bumper: the front half of a slightly larger ring */}
      <mesh position={[0, 0, BODY_BOTTOM + BODY_H / 2 - 0.004]} rotation={up}>
        <cylinderGeometry args={[R + 0.006, R + 0.006, BODY_H - 0.012, 48, 1, true, 0, Math.PI]} />
        <meshStandardMaterial color="#15171a" roughness={0.9} side={2} />
      </mesh>
    </group>
  );
}

/** A small arrow on top pointing forward, so the heading reads from above. */
function HeadingMarker() {
  return (
    <mesh position={[0.125, 0, TOP + 0.003]}>
      <circleGeometry args={[0.018, 3]} />
      <meshBasicMaterial color="#4fd1c5" />
    </mesh>
  );
}

/** A glowing ring on the floor so the robot is easy to spot from far away. */
function FloorRing() {
  return (
    <mesh position={[0, 0, 0.003]} renderOrder={3}>
      <ringGeometry args={[R + 0.03, R + 0.06, 64]} />
      <meshBasicMaterial color="#4fd1c5" transparent opacity={0.85} depthWrite={false} />
    </mesh>
  );
}

function Lidar() {
  const spinner = useRef<Mesh>(null);
  useFrame((_, dt) => {
    if (spinner.current) spinner.current.rotation.y += dt * 2 * Math.PI * 2;
  });
  // The lidar's frame is at its optical plane, LIDAR_UP above the top of the Roomba.
  const base = 0.022;
  const stand = LIDAR_UP - base;
  return (
    <group>
      {/* Standoff from the top of the Roomba up to the lidar (placeholder, like the mount itself) */}
      <mesh position={[0, 0, -base - stand / 2]} castShadow>
        <boxGeometry args={[0.03, 0.03, stand]} />
        <meshStandardMaterial color="#2a2d33" roughness={0.8} />
      </mesh>
      <mesh position={[0, 0, -0.012]} castShadow>
        <boxGeometry args={[0.039, 0.039, 0.02]} />
        <meshStandardMaterial color="#1b1d21" />
      </mesh>
      <mesh ref={spinner} position={[0, 0, 0.004]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.018, 0.018, 0.014, 6]} />
        <meshStandardMaterial color="#3a3f47" metalness={0.5} roughness={0.3} />
      </mesh>
    </group>
  );
}

function CameraHousing() {
  return (
    <group>
      <mesh castShadow>
        <boxGeometry args={[0.012, 0.03, 0.025]} />
        <meshStandardMaterial color="#1f6f8b" />
      </mesh>
      <mesh position={[0.008, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.007, 0.007, 0.006, 16]} />
        <meshStandardMaterial color="#0b0c0e" metalness={0.6} roughness={0.2} />
      </mesh>
    </group>
  );
}
