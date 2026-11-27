import { Matrix4, Quaternion, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';

import { driveCommand } from '../ui/drive';
import { parseCameraFrame } from './connection';
import { scanToPoints } from './scan';
import { TfTree } from './tf';
import { VoxelMemory } from './voxels';

const yaw = (a: number) => new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), a);
const at = (m: Matrix4) => new Vector3().setFromMatrixPosition(m);

describe('parseCameraFrame', () => {
  it('splits the header from the JPEG', () => {
    const header = new TextEncoder().encode(JSON.stringify({ frame: 'cam', stamp: 1, width: 2, height: 1, model: null }));
    const jpeg = new Uint8Array([0xff, 0xd8, 0xff]);
    const buf = new Uint8Array(5 + header.length + jpeg.length);
    buf[0] = 1;
    new DataView(buf.buffer).setUint32(1, header.length, true);
    buf.set(header, 5);
    buf.set(jpeg, 5 + header.length);
    const msg = parseCameraFrame(buf.buffer)!;
    expect(msg.header.frame).toBe('cam');
    expect(msg.jpeg.size).toBe(3);
    expect(parseCameraFrame(new Uint8Array([2, 0, 0, 0, 0]).buffer)).toBeNull();
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
        frame: 'l',
        stamp: 0,
        angle_min: 0,
        angle_increment: Math.PI / 2,
        range_min: 0.05,
        range_max: 10,
        ranges: [1, null, 2, 0.01],
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

describe('driveCommand', () => {
  it('scales forward and turn by the chosen speed', () => {
    expect(driveCommand([1, 0], 0.3)).toEqual({ linear: 0.3, angular: 0 });
    expect(driveCommand([0, -1], 0.3).angular).toBeCloseTo(-1.5);
    expect(driveCommand([0, 1], 0.5).angular).toBe(2);
  });
});
