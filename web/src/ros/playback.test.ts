import { describe, expect, it } from 'vitest';

import { PlaybackClock } from './playback';

/** Feeds samples 50 ms apart in robot time, arriving at the given wall-clock gaps (repeating). */
function feed(clock: PlaybackClock, wallGaps: number[], seconds: number, rate = 1) {
  const arrivals: Array<[number, number]> = [];
  let wall = 100;
  let stamp = 5;
  for (let i = 0; wall < 100 + seconds; i++) {
    clock.observe(stamp, wall);
    arrivals.push([stamp, wall]);
    stamp += 0.05;
    wall += wallGaps[i % wallGaps.length] / rate;
  }
  return { arrivals, end: wall };
}

/** How far the clock moves in each 1/60 s frame over the last second. */
function frameSteps(clock: PlaybackClock, end: number): number[] {
  const steps: number[] = [];
  let prev = clock.now(end - 1)!;
  for (let w = end - 1 + 1 / 60; w < end; w += 1 / 60) {
    const t = clock.now(w)!;
    steps.push(t - prev);
    prev = t;
  }
  return steps;
}

describe('PlaybackClock', () => {
  it('has no time before the first sample', () => {
    expect(new PlaybackClock().now(1)).toBeUndefined();
  });

  it('runs smoothly behind samples that arrive unevenly (50 ms, then 100 ms, ...)', () => {
    const clock = new PlaybackClock();
    // 50 ms of robot time per sample, arriving 52 then 98 ms apart: the sim at 2/3 real time.
    const { arrivals, end } = feed(clock, [0.052, 0.098], 6);
    const steps = frameSteps(clock, end);
    const mean = steps.reduce((a, b) => a + b, 0) / steps.length;
    expect(mean).toBeCloseTo((0.05 / 0.075) / 60, 3);
    for (const s of steps) expect(Math.abs(s - mean) / mean).toBeLessThan(0.1); // no jumps or stalls
    // Always drawing between two samples it has already received.
    const [lastStamp] = arrivals[arrivals.length - 1];
    expect(clock.now(end)!).toBeLessThan(lastStamp);
    expect(clock.now(end)!).toBeGreaterThan(lastStamp - 0.3);
  });

  it('never runs past the newest sample when they stop coming', () => {
    const clock = new PlaybackClock();
    const { arrivals, end } = feed(clock, [0.05], 2);
    const [lastStamp] = arrivals[arrivals.length - 1];
    expect(clock.now(end + 10)).toBe(lastStamp);
  });

  it('starts over when the robot clock restarts', () => {
    const clock = new PlaybackClock();
    feed(clock, [0.05], 2);
    clock.observe(0.5, 200);
    expect(clock.now(200)).toBeCloseTo(0.5 - clock.delay, 6);
  });

  it('draws further behind when samples come less often', () => {
    const clock = new PlaybackClock();
    let wall = 0;
    for (let stamp = 1; stamp < 5; stamp += 0.1, wall += 0.1) clock.observe(stamp, wall);
    expect(clock.delay).toBeGreaterThan(0.15);
  });
});
