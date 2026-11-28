/**
 * The robot's clock, played back smoothly for drawing.
 *
 * The robot's position arrives 20 times a second (odom -> base_footprint), and not evenly: the
 * sim pauses while it renders the camera, and wifi bunches messages up. Drawing the robot at
 * the newest position makes it jump forward with each one, which looks jerky. Instead the 3D view
 * draws everything at this clock's time, which runs a little behind the newest position at the
 * robot's own rate, so poses are interpolated between samples and the motion is smooth.
 *
 * Times are the robot's (sim time in the sim), in seconds; wall times are performance.now() / 1000.
 */
export class PlaybackClock {
  /** The clock's time at `wall0`, and how fast it runs from there (robot seconds per wall second). */
  private t0 = 0;
  private wall0 = 0;
  private slope = 1;
  /** Newest stamp seen, or null before the first. */
  private newest: number | null = null;
  /** The robot's clock rate (1 on a real robot; the real-time factor in the sim). */
  private rate = 1;
  private rateFrom: { stamp: number; wall: number } | null = null;
  /** Average time between samples, by their stamps. */
  private gap = 0.05;

  constructor(
    /** The least it draws behind the newest stamp; more when samples come less often. */
    private readonly minDelay = 0.12,
    /** How long to take to make up a difference from where the clock should be, in wall seconds. */
    private readonly catchUp = 0.8,
  ) {}

  /** How far behind the newest stamp it draws: enough to have a sample on each side. */
  get delay(): number {
    return Math.max(this.minDelay, 2 * this.gap);
  }

  /** Call with the stamp of each new robot position, and the wall time it arrived. */
  observe(stamp: number, wall: number): void {
    if (this.newest === null || stamp < this.newest - 1 || stamp > this.newest + 5) {
      // First sample, or the robot's clock restarted or jumped (sim relaunched): start over.
      this.newest = stamp;
      this.gap = 0.05;
      this.t0 = stamp - this.delay;
      this.wall0 = wall;
      this.slope = this.rate = 1;
      this.rateFrom = { stamp, wall };
      return;
    }
    if (stamp <= this.newest) return;
    this.gap += 0.1 * (Math.min(stamp - this.newest, 0.5) - this.gap);
    this.newest = stamp;
    // Measure the robot's clock rate over a second or more, so one late sample doesn't count much.
    const from = this.rateFrom!;
    if (wall - from.wall >= 1) {
      const measured = (stamp - from.stamp) / (wall - from.wall);
      this.rate = clamp(0.5 * this.rate + 0.5 * measured, 0.1, 10);
      this.rateFrom = { stamp, wall };
    }
    // Carry on from where the clock is now, heading to `delay` behind the newest stamp.
    const now = this.raw(wall);
    const error = stamp - this.delay - now;
    if (Math.abs(error) > 1) {
      this.t0 = stamp - this.delay; // far off (e.g. the tab was asleep): jump there
    } else {
      this.t0 = now;
    }
    this.wall0 = wall;
    this.slope = Math.abs(error) > 1 ? this.rate : this.rate * clamp(1 + error / this.catchUp, 0.5, 1.5);
  }

  /** The time to draw at, or undefined before any position has arrived. Never past the newest. */
  now(wall: number): number | undefined {
    if (this.newest === null) return undefined;
    return Math.min(this.raw(wall), this.newest);
  }

  reset(): void {
    this.newest = null;
    this.rateFrom = null;
  }

  private raw(wall: number): number {
    return this.t0 + this.slope * (wall - this.wall0);
  }
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}
