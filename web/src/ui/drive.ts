import type { RobotConnection } from '../ros/connection';

/** [forward, turn], each -1..1. Turn is positive to the left (counterclockwise), like ROS. */
export type DriveInput = [number, number];

const RATE_HZ = 10;
/** Zero commands sent after letting go, in case one is dropped. */
const STOP_REPEATS = 3;

export function driveCommand([forward, turn]: DriveInput, speed: number): { linear: number; angular: number } {
  const turnRate = Math.min(speed * 5, 2);
  return { linear: forward * speed, angular: turn * turnRate };
}

/**
 * Sends drive commands while a key or on-screen button is held, ten times a second, and stops the
 * robot as soon as everything is released (or the page loses focus). The bridge also stops the
 * robot by itself if these commands stop arriving, e.g. when the tab is closed.
 */
export class DriveController {
  private keys = new Set<string>();
  private pad: DriveInput = [0, 0];
  private timer: ReturnType<typeof setInterval> | null = null;
  private stopsLeft = 0;
  speed = 0.3;
  onChange: ((input: DriveInput) => void) | null = null;

  constructor(private readonly connection: RobotConnection) {}

  setKey(code: string, down: boolean): boolean {
    if (!KEY_CODES.has(code)) return false;
    if (down) this.keys.add(code);
    else this.keys.delete(code);
    this.changed();
    return true;
  }

  setPad(input: DriveInput): void {
    this.pad = input;
    this.changed();
  }

  stopAll(): void {
    this.keys.clear();
    this.pad = [0, 0];
    this.changed();
  }

  input(): DriveInput {
    const k = this.keys;
    const fwd = (k.has('KeyW') || k.has('ArrowUp') ? 1 : 0) - (k.has('KeyS') || k.has('ArrowDown') ? 1 : 0);
    const turn = (k.has('KeyA') || k.has('ArrowLeft') ? 1 : 0) - (k.has('KeyD') || k.has('ArrowRight') ? 1 : 0);
    return [clamp(fwd + this.pad[0]), clamp(turn + this.pad[1])];
  }

  dispose(): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }

  private changed(): void {
    const input = this.input();
    this.onChange?.(input);
    const moving = input[0] !== 0 || input[1] !== 0;
    if (!moving) this.stopsLeft = STOP_REPEATS;
    this.send();
    if (!this.timer) this.timer = setInterval(() => this.send(), 1000 / RATE_HZ);
  }

  private send(): void {
    const input = this.input();
    const moving = input[0] !== 0 || input[1] !== 0;
    if (!moving) {
      if (this.stopsLeft <= 0) {
        this.dispose();
        return;
      }
      this.stopsLeft--;
    }
    const { linear, angular } = driveCommand(input, this.speed);
    this.connection.send({ op: 'drive', linear, angular });
  }
}

const KEY_CODES = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);

function clamp(v: number): number {
  return Math.max(-1, Math.min(1, v));
}
