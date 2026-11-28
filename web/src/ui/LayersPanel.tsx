import { useState } from 'react';

import { useRobot } from '../context';
import { settings } from '../settings';
import type { Settings, ViewMode, WallMode } from '../settings';
import { useStore } from '../store';

const VIEWS: Array<[ViewMode, string]> = [
  ['follow', 'Follow'],
  ['chase', 'Chase'],
  ['top', 'Top'],
  ['free', 'Free'],
];

const WALLS: Array<[WallMode, string]> = [
  ['full', 'Full'],
  ['cut', 'Cut'],
  ['off', 'Off'],
];

type Toggle = keyof Pick<Settings, 'map' | 'plan' | 'costmap' | 'scan' | 'memory' | 'trail' | 'camera3d' | 'cameraPip'>;
const TOGGLES: Array<[Toggle, string]> = [
  ['map', 'Map'],
  ['plan', 'Planned route'],
  ['costmap', 'Costmap (where it avoids)'],
  ['scan', 'Lidar scan'],
  ['memory', 'Lidar memory'],
  ['trail', 'Trail'],
  ['camera3d', 'Camera in 3D'],
  ['cameraPip', 'Camera window'],
];

/** View and layer controls, top right. */
export function LayersPanel() {
  const { data } = useRobot();
  const hasWorld = useStore(data.world).length > 0;
  const [open, setOpen] = useState(() => window.matchMedia('(min-width: 700px)').matches);
  return (
    <div className={`panel layers ${open ? 'open' : ''}`}>
      <button className="collapse" aria-expanded={open} onClick={() => setOpen(!open)}>
        Layers {open ? '▴' : '▾'}
      </button>
      {open ? <LayerControls hasWorld={hasWorld} /> : null}
    </div>
  );
}

function LayerControls({ hasWorld }: { hasWorld: boolean }) {
  const s = useStore(settings);
  const { data } = useRobot();
  return (
    <>
      <div className="label">View</div>
      <Segmented options={VIEWS} value={s.view} onChange={(view) => settings.update({ view })} />
      {hasWorld ? (
        <>
          <div className="label">Walls</div>
          <Segmented options={WALLS} value={s.walls} onChange={(walls) => settings.update({ walls })} />
        </>
      ) : null}
      <div className="label">Show</div>
      {TOGGLES.map(([key, label]) => (
        <label key={key} className="toggle">
          <input type="checkbox" checked={s[key]} onChange={(e) => settings.update({ [key]: e.target.checked })} />
          <span>{label}</span>
        </label>
      ))}
      <button className="ghost" onClick={() => data.clearMemory()}>
        Clear memory and trail
      </button>
    </>
  );
}

function Segmented<T extends string>({
  options,
  value,
  onChange,
}: {
  options: Array<[T, string]>;
  value: T;
  onChange: (v: T) => void;
}) {
  return (
    <div className="segmented" role="radiogroup">
      {options.map(([v, label]) => (
        <button key={v} role="radio" aria-checked={v === value} className={v === value ? 'on' : ''} onClick={() => onChange(v)}>
          {label}
        </button>
      ))}
    </div>
  );
}
