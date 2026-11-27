import { parse } from '@foxglove/rosmsg';
import { MessageReader } from '@foxglove/rosmsg2-serialization';
import { Matrix4, Quaternion, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';

import { driveCommand } from '../ui/drive';
import { encodeTwist } from './cdr';
import { imageToRgba } from './image';
import { scanToPoints } from './scan';
import { TfTree } from './tf';
import { VoxelMemory } from './voxels';

const TWIST = `geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z`;

const yaw = (a: number) => new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), a);
const at = (m: Matrix4) => new Vector3().setFromMatrixPosition(m);

describe('encodeTwist', () => {
  it('round-trips through the ROS 2 CDR reader', () => {
    const bytes = encodeTwist({ x: 0.3, y: 0, z: 0 }, { x: 0, y: 0, z: -1.25 });
    expect(bytes.length).toBe(52);
    const msg = new MessageReader(parse(TWIST, { ros2: true })).readMessage<{
      linear: { x: number };
      angular: { z: number };
    }>(bytes);
    expect(msg.linear.x).toBeCloseTo(0.3);
    expect(msg.angular.z).toBeCloseTo(-1.25);
  });
});

describe('TfTree', () => {
  it('chains transforms and looks up between any two connected frames', () => {
    const tf = new TfTree();
    tf.set('world', 'odom', 0, new Vector3(-3.4, 0, 0), yaw(0), true);
    tf.set('odom', 'base_footprint', 1, new Vector3(1, 0, 0), yaw(Math.PI / 2), false);
    tf.set('/base_footprint', 'ldlidar_link', 0, new Vector3(0.1, 0, 0.13), yaw(0), true);
    const lidar = at(tf.lookup('world', 'ldlidar_link')!);
    // Robot at world (-2.4, 0) facing +y, lidar 0.1 m ahead of it.
    expect(lidar.x).toBeCloseTo(-2.4);
    expect(lidar.y).toBeCloseTo(0.1);
    expect(lidar.z).toBeCloseTo(0.13);
    const back = at(tf.lookup('ldlidar_link', 'world')!);
    expect(back.length()).toBeGreaterThan(0);
    expect(tf.lookup('world', 'nowhere')).toBeNull();
  });

  it('interpolates moving transforms by time', () => {
    const tf = new TfTree();
    tf.set('odom', 'base', 10, new Vector3(0, 0, 0), yaw(0), false);
    tf.set('odom', 'base', 11, new Vector3(1, 0, 0), yaw(0), false);
    expect(at(tf.lookup('odom', 'base', 10.25)!).x).toBeCloseTo(0.25);
    expect(at(tf.lookup('odom', 'base')!).x).toBeCloseTo(1);
    expect(at(tf.lookup('odom', 'base', 99)!).x).toBeCloseTo(1);
  });

  it('forgets history when the clock jumps back (sim restarted)', () => {
    const tf = new TfTree();
    tf.set('odom', 'base', 100, new Vector3(5, 0, 0), yaw(0), false);
    tf.set('odom', 'base', 1, new Vector3(0, 0, 0), yaw(0), false);
    expect(at(tf.lookup('odom', 'base')!).x).toBeCloseTo(0);
  });
});

describe('scanToPoints', () => {
  it('drops invalid ranges and places hits in the fixed frame', () => {
    const toFixed = new Matrix4().makeTranslation(1, 2, 0.1);
    const { positions, ranges } = scanToPoints(
      {
        header: { frame_id: 'l', stamp: { sec: 0, nanosec: 0 } },
        angle_min: 0,
        angle_increment: Math.PI / 2,
        range_min: 0.05,
        range_max: 10,
        ranges: [1, Infinity, 2, 0.01],
      },
      toFixed,
    );
    expect(ranges.length).toBe(2);
    expect([...positions].map((v) => +v.toFixed(3))).toEqual([2, 2, 0.1, -1, 2, 0.1]);
  });
});

describe('VoxelMemory', () => {
  it('stores each grid cell once and drops the oldest when full', () => {
    const mem = new VoxelMemory(0.1, 2);
    expect(mem.add([0.01, 0.01, 0, 0.02, 0.03, 0])).toBe(1);
    expect(mem.add([1, 1, 0])).toBe(1);
    expect(mem.count).toBe(2);
    expect(mem.add([2, 2, 0])).toBe(1);
    expect(mem.count).toBe(2);
    // The first cell was overwritten, so it counts as new again.
    expect(mem.add([0.01, 0.01, 0])).toBe(1);
  });
});

describe('imageToRgba', () => {
  it('converts bgr8 with row padding', () => {
    const data = new Uint8Array([3, 2, 1, 9, /* pad */ 6, 5, 4, 9]);
    const out = imageToRgba({ header: { frame_id: '' }, width: 1, height: 2, encoding: 'bgr8', step: 4, data });
    expect([...out!]).toEqual([1, 2, 3, 255, 4, 5, 6, 255]);
    expect(imageToRgba({ header: { frame_id: '' }, width: 1, height: 1, encoding: '16UC1', step: 2, data })).toBeNull();
  });
});

describe('driveCommand', () => {
  it('scales forward and turn by the chosen speed', () => {
    expect(driveCommand([1, 0], 0.3)).toEqual({ linear: 0.3, angular: 0 });
    expect(driveCommand([0, -1], 0.3).angular).toBeCloseTo(-1.5);
    expect(driveCommand([0, 1], 0.5).angular).toBe(2);
  });
});
