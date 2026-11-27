/** Hand-rolled CDR (little-endian) encoders for the few messages the UI publishes. */

const CDR_LE_HEADER = [0x00, 0x01, 0x00, 0x00];

export type Vec3 = { x: number; y: number; z: number };

/** geometry_msgs/msg/Twist: two Vector3s of float64, 52 bytes including the 4-byte header. */
export function encodeTwist(linear: Vec3, angular: Vec3): Uint8Array {
  const out = new Uint8Array(4 + 6 * 8);
  out.set(CDR_LE_HEADER, 0);
  const view = new DataView(out.buffer);
  [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z].forEach((v, i) => {
    view.setFloat64(4 + i * 8, v, true);
  });
  return out;
}
