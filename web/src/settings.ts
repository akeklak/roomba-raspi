import { Store } from './store';

export type WallMode = 'full' | 'cut' | 'off';
export type ViewMode = 'follow' | 'free' | 'top' | 'chase';

export type Settings = {
  walls: WallMode;
  /** The map (from SLAM, or the saved one). */
  map: boolean;
  /** Nav2's costmap: obstacles grown by the robot's size. */
  costmap: boolean;
  /** The path Nav2 plans to the goal. */
  plan: boolean;
  scan: boolean;
  memory: boolean;
  trail: boolean;
  camera3d: boolean;
  cameraPip: boolean;
  view: ViewMode;
  /** Top driving speed in m/s. */
  speed: number;
};

const DEFAULTS: Settings = {
  walls: 'cut',
  map: true,
  costmap: false,
  plan: true,
  scan: true,
  memory: true,
  trail: true,
  camera3d: false,
  cameraPip: true,
  view: 'follow',
  speed: 0.3,
};

const KEY = 'roomba-ui-settings';

function load(): Settings {
  try {
    const saved = JSON.parse(localStorage.getItem(KEY) ?? '{}');
    return { ...DEFAULTS, ...saved };
  } catch {
    return DEFAULTS;
  }
}

export const settings = new Store<Settings>(load());

settings.subscribe(() => {
  try {
    localStorage.setItem(KEY, JSON.stringify(settings.get()));
  } catch {
    /* private window or storage blocked: settings just won't stick */
  }
});

/** Height walls are cut at in "cut" mode, so you can see into the rooms from above. */
export const CUT_HEIGHT = 0.5;
