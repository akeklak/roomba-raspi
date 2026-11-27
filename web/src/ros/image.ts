export type RosImage = {
  header: { frame_id: string };
  width: number;
  height: number;
  encoding: string;
  step: number;
  data: Uint8Array;
};

/** Converts a sensor_msgs/Image to RGBA pixels (for a canvas or a texture). Null if unsupported. */
export function imageToRgba(img: RosImage, out?: Uint8ClampedArray): Uint8ClampedArray | null {
  const { width, height, step, data } = img;
  const size = width * height * 4;
  const rgba = out && out.length === size ? out : new Uint8ClampedArray(size);
  let order: [number, number, number, number] | null;
  let channels: number;
  switch (img.encoding.toLowerCase()) {
    case 'rgb8':
      order = [0, 1, 2, -1];
      channels = 3;
      break;
    case 'bgr8':
      order = [2, 1, 0, -1];
      channels = 3;
      break;
    case 'rgba8':
      order = [0, 1, 2, 3];
      channels = 4;
      break;
    case 'bgra8':
      order = [2, 1, 0, 3];
      channels = 4;
      break;
    case 'mono8':
    case '8uc1':
      order = [0, 0, 0, -1];
      channels = 1;
      break;
    default:
      return null;
  }
  const [r, g, b, a] = order;
  for (let y = 0; y < height; y++) {
    let src = y * step;
    let dst = y * width * 4;
    for (let x = 0; x < width; x++, src += channels, dst += 4) {
      rgba[dst] = data[src + r];
      rgba[dst + 1] = data[src + g];
      rgba[dst + 2] = data[src + b];
      rgba[dst + 3] = a < 0 ? 255 : data[src + a];
    }
  }
  return rgba;
}
