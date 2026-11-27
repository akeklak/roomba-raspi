import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import type { Mesh } from 'three';

import { BASE_FRAME } from '../ros/robotData';
import { FrameGroup } from './FrameGroup';

/** A Roomba 600-series with the lidar and camera, each drawn at its own TF frame. */
export function RobotModel() {
  return (
    <>
      <FrameGroup frame={BASE_FRAME}>
        <RoombaBody />
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
const BODY_BOTTOM = 0.012;
const BODY_H = 0.068;

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
      {/* Heading marker */}
      <mesh position={[0.125, 0, BODY_BOTTOM + BODY_H + 0.005]}>
        <circleGeometry args={[0.018, 3]} />
        <meshBasicMaterial color="#4fd1c5" />
      </mesh>
    </group>
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
  return (
    <group>
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
