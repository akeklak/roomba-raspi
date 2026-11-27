import { Canvas } from '@react-three/fiber';
import { Suspense } from 'react';

import { settings } from '../settings';
import { useStore } from '../store';
import { CameraFrustum } from './CameraFrustum';
import { CameraRig } from './CameraRig';
import { RobotModel } from './RobotModel';
import { LidarMemory, LidarScan, Trail } from './SensorLayers';
import { World } from './World';

/** The 3D view. Everything is in ROS coordinates: meters, Z up, X forward. */
export function Scene() {
  const s = useStore(settings);
  return (
    <Canvas
      shadows
      dpr={[1, 2]}
      camera={{ position: [-5.2, -2.4, 2.4], up: [0, 0, 1], fov: 50, near: 0.02, far: 200 }}
      gl={{ antialias: true, preserveDrawingBuffer: true }}
    >
      <color attach="background" args={['#14161a']} />
      <fog attach="fog" args={['#14161a', 18, 40]} />
      <hemisphereLight args={['#dfe7f2', '#2a2622', 0.9]} />
      <directionalLight
        position={[2.5, -1.5, 10]}
        intensity={1.6}
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-8}
        shadow-camera-right={8}
        shadow-camera-top={8}
        shadow-camera-bottom={-8}
        shadow-bias={-0.0005}
      />
      <Floor />
      <Suspense fallback={null}>
        <World />
        <RobotModel />
        {s.trail ? <Trail /> : null}
        {s.memory ? <LidarMemory /> : null}
        {s.scan ? <LidarScan /> : null}
        {s.camera3d ? <CameraFrustum /> : null}
      </Suspense>
      <CameraRig />
    </Canvas>
  );
}

function Floor() {
  return (
    <group>
      <mesh receiveShadow position={[0, 0, -0.001]}>
        <planeGeometry args={[60, 60]} />
        <meshStandardMaterial color="#2a2d33" roughness={1} />
      </mesh>
      {/* gridHelper lies in the XZ plane; turn it to lie on the XY floor. */}
      <gridHelper args={[60, 120, '#4a515c', '#353a42']} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, 0.0005]} />
    </group>
  );
}
