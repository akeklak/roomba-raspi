import { useEffect, useMemo, useState } from 'react';

import { useRobot } from '../context';
import { settings } from '../settings';
import { useStore } from '../store';
import { DriveController } from './drive';
import { tool } from './tools';
import type { DriveInput } from './drive';

const PAD: Array<{ input: DriveInput; label: string; aria: string }> = [
  { input: [1, 1], label: '↖', aria: 'Forward left' },
  { input: [1, 0], label: '↑', aria: 'Forward' },
  { input: [1, -1], label: '↗', aria: 'Forward right' },
  { input: [0, 1], label: '↺', aria: 'Spin left' },
  { input: [0, 0], label: '■', aria: 'Stop' },
  { input: [0, -1], label: '↻', aria: 'Spin right' },
  { input: [-1, 1], label: '↙', aria: 'Back left' },
  { input: [-1, 0], label: '↓', aria: 'Back' },
  { input: [-1, -1], label: '↘', aria: 'Back right' },
];

/**
 * Hold-to-drive buttons and a speed slider, bottom right. WASD and the arrow keys work too. The
 * slider sets the top speed for Go to as well (App passes it to the robot).
 */
export function DrivePad() {
  const { connection, data } = useRobot();
  const { speed } = useStore(settings);
  const info = useStore(connection.info);
  const drive = useMemo(() => new DriveController(connection), [connection]);
  const [input, setInput] = useState<DriveInput>([0, 0]);
  drive.speed = speed;

  useEffect(() => {
    drive.onChange = setInput;
    const isTyping = (e: KeyboardEvent) => (e.target as HTMLElement | null)?.closest('input, textarea, select') !== null;
    const down = (e: KeyboardEvent) => {
      if (e.repeat || e.metaKey || e.ctrlKey || isTyping(e)) return;
      if (e.code === 'Space') {
        drive.stopAll();
        data.cancelNav();
        e.preventDefault();
      } else if (e.code === 'KeyG') {
        const m = data.mapping.get();
        if (m?.nav_ready && m.mode !== 'off') tool.set(tool.get() === 'goal' ? 'none' : 'goal');
        e.preventDefault();
      } else if (drive.setKey(e.code, true)) e.preventDefault();
    };
    const up = (e: KeyboardEvent) => {
      if (drive.setKey(e.code, false)) e.preventDefault();
    };
    const blur = () => drive.stopAll();
    const hidden = () => document.hidden && drive.stopAll();
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', blur);
    document.addEventListener('visibilitychange', hidden);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', blur);
      document.removeEventListener('visibilitychange', hidden);
      drive.stopAll();
      drive.dispose();
    };
  }, [drive, data]);

  const active = (p: DriveInput) => p[0] === input[0] && p[1] === input[1] && (p[0] !== 0 || p[1] !== 0);
  const disabled = info.status !== 'connected';

  return (
    <div className={`panel drive ${disabled ? 'disabled' : ''}`}>
      <div className="pad">
        {PAD.map((b) => (
          <button
            key={b.aria}
            aria-label={b.aria}
            className={active(b.input) ? 'on' : ''}
            onPointerDown={(e) => {
              e.currentTarget.setPointerCapture(e.pointerId);
              if (b.input[0] === 0 && b.input[1] === 0) {
                drive.stopAll();
                data.cancelNav();
              } else drive.setPad(b.input);
            }}
            onPointerUp={() => drive.setPad([0, 0])}
            onPointerCancel={() => drive.setPad([0, 0])}
            onContextMenu={(e) => e.preventDefault()}
          >
            {b.label}
          </button>
        ))}
      </div>
      <label className="speed" title="Top speed when driving by hand and for Go to">
        <span>Top speed {speed.toFixed(2)} m/s</span>
        <input
          type="range"
          min={0.05}
          max={0.5}
          step={0.05}
          value={speed}
          onChange={(e) => settings.update({ speed: Number(e.target.value) })}
          onPointerUp={(e) => e.currentTarget.blur()}
        />
      </label>
      <div className="keys">Hold W A S D or the arrow keys · Space stops · G picks a spot to go to</div>
    </div>
  );
}
