import { useEffect, useRef, useState } from 'react';

import { useRobot } from '../context';

/** The live camera picture, bottom left. Click to make it bigger. */
export function CameraWindow() {
  const { data } = useRobot();
  const canvas = useRef<HTMLCanvasElement>(null);
  const [big, setBig] = useState(false);
  const [hasImage, setHasImage] = useState(false);

  useEffect(() => {
    let raf = 0;
    let seen = -1;
    const draw = () => {
      raf = requestAnimationFrame(draw);
      const cam = data.camera;
      const c = canvas.current;
      if (!cam || !c || cam.version === seen) return;
      seen = cam.version;
      if (c.width !== cam.width || c.height !== cam.height) {
        c.width = cam.width;
        c.height = cam.height;
      }
      c.getContext('2d')?.drawImage(cam.image, 0, 0);
      setHasImage(true);
    };
    draw();
    return () => cancelAnimationFrame(raf);
  }, [data]);

  return (
    <div className={`panel camera ${big ? 'big' : ''}`} onClick={() => setBig(!big)} title="Click to resize">
      <canvas ref={canvas} />
      {!hasImage ? <div className="placeholder">Waiting for the camera…</div> : null}
      <div className="caption">Front camera</div>
    </div>
  );
}
