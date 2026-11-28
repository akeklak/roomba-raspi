import { useEffect, useState } from 'react';

import { useRobot } from '../context';
import type { MapCommand, MappingState, NavState, Notice } from '../ros/robotData';
import { useStore } from '../store';
import { tool } from './tools';
import type { Tool } from './tools';

/** How long a notice or a finished goal stays on screen. */
const SHOW_MS = 8000;

function modeLabel(m: MappingState): string {
  if (m.mode === 'mapping') return m.extending ? 'Adding to the map' : 'Making a map';
  if (m.mode === 'navigation') return 'Using the saved map';
  return 'Mapping is off';
}

function ago(seconds: number): string {
  const s = Date.now() / 1000 - seconds;
  if (s < 90) return 'just now';
  if (s < 90 * 60) return `${Math.round(s / 60)} min ago`;
  if (s < 36 * 3600) return `${Math.round(s / 3600)} h ago`;
  return new Date(seconds * 1000).toLocaleDateString();
}

function navLine(nav: NavState): string {
  if (nav.state === 'starting') return 'Planning a route…';
  if (nav.state === 'active') {
    const parts = ['Driving there'];
    if (nav.distance !== null) parts.push(`${nav.distance.toFixed(1)} m left`);
    if (nav.eta !== null && nav.eta > 0) parts.push(`about ${Math.ceil(nav.eta)} s`);
    if (nav.recoveries > 0) parts.push(nav.recoveries === 1 ? 'retried once' : `retried ${nav.recoveries} times`);
    return parts.join(' · ');
  }
  return nav.message;
}

/** True for `ms` after `key` changes (and while `key` is null, false). */
function useRecent(key: unknown, ms: number): boolean {
  const [recent, setRecent] = useState(false);
  useEffect(() => {
    if (key === null) {
      setRecent(false);
      return;
    }
    setRecent(true);
    const t = setTimeout(() => setRecent(false), ms);
    return () => clearTimeout(t);
  }, [key, ms]);
  return recent;
}

/**
 * Mapping and navigation, under the status panel: which map is in use, the buttons to save it or
 * start over, and "Go to" for sending the robot somewhere.
 */
export function MapPanel() {
  const { connection, data } = useRobot();
  const info = useStore(connection.info);
  const mapping = useStore(data.mapping);
  const nav = useStore(data.nav);
  const notice = useStore(data.notice);
  const current = useStore(tool);
  const [confirmNew, setConfirmNew] = useState(false);
  const navDone = nav.state === 'succeeded' || nav.state === 'canceled' || nav.state === 'failed';
  const showNavResult = useRecent(navDone ? `${nav.state}${nav.message}${nav.goal?.x}${nav.goal?.y}` : null, SHOW_MS);
  const showNotice = useRecent(notice?.id ?? null, SHOW_MS);

  useEffect(() => {
    if (!confirmNew) return;
    const t = setTimeout(() => setConfirmNew(false), SHOW_MS);
    return () => clearTimeout(t);
  }, [confirmNew]);

  if (info.status !== 'connected') return null;

  const busy = mapping?.busy ?? null;
  const command = (c: MapCommand) => {
    setConfirmNew(false);
    tool.set('none');
    // A new map starts from nothing (and on the robot it has its own origin), so the lidar
    // memory starts over with it.
    if (data.mapCommand(c) && c === 'new_map') data.clearMemory();
  };
  const pick = (t: Tool) => tool.set(current === t ? 'none' : t);
  const navigating = nav.state === 'starting' || nav.state === 'active';
  const canGo = mapping !== null && mapping.mode !== 'off' && mapping.nav_ready && !busy;

  return (
    <div className="panel map-panel">
      <div className="map-head">
        <span className="map-title">Map</span>
        {mapping ? <span className={`chip ${mapping.mode}`}>{modeLabel(mapping)}</span> : null}
      </div>

      {mapping === null ? (
        <p className="muted">Waiting for mapping to start on the robot…</p>
      ) : (
        <>
          {busy ? (
            <p className="busy">
              <span className="spinner" aria-hidden /> {busy}…
            </p>
          ) : null}
          {mapping.error ? <p className="error">{mapping.error}</p> : null}
          {!mapping.nav_ready && mapping.mode !== 'off' && !busy ? (
            <p className="muted small">Navigation starts once the robot is placed on the map…</p>
          ) : null}

          <div className="go-row">
            <button
              className={`primary ${current === 'goal' ? 'on' : ''}`}
              disabled={!canGo}
              aria-pressed={current === 'goal'}
              onClick={() => pick('goal')}
              title={
                mapping.nav_ready
                  ? 'Pick a spot on the floor and the robot drives there by itself (G)'
                  : 'Navigation starts once the robot is on a map'
              }
            >
              {current === 'goal' ? 'Pick a spot…' : 'Go to'}
            </button>
            {navigating ? (
              <button className="stop" onClick={() => data.cancelNav()} title="Stop driving there (Space)">
                Stop
              </button>
            ) : null}
          </div>
          {navigating || showNavResult ? <p className={`nav-line ${nav.state}`}>{navLine(nav)}</p> : null}

          <MapActions mapping={mapping} disabled={busy !== null} confirmNew={confirmNew} setConfirmNew={setConfirmNew}
            command={command} current={current} pick={pick} />

          {mapping.saved ? (
            <p className="muted small">
              Saved map {mapping.saved.width_m.toFixed(1)} × {mapping.saved.height_m.toFixed(1)} m ·{' '}
              {ago(mapping.saved.saved_at)}
            </p>
          ) : (
            <p className="muted small">No saved map yet.</p>
          )}
        </>
      )}
      {notice && showNotice ? <NoticeLine notice={notice} /> : null}
    </div>
  );
}

function MapActions({
  mapping,
  disabled,
  confirmNew,
  setConfirmNew,
  command,
  current,
  pick,
}: {
  mapping: MappingState;
  disabled: boolean;
  confirmNew: boolean;
  setConfirmNew: (v: boolean) => void;
  command: (c: MapCommand) => void;
  current: Tool;
  pick: (t: Tool) => void;
}) {
  if (confirmNew) {
    return (
      <div className="confirm">
        <p>Start a new map from scratch? The saved map is kept until you save the new one.</p>
        <div className="actions">
          <button onClick={() => command('new_map')} disabled={disabled}>
            Start new map
          </button>
          <button className="quiet" onClick={() => setConfirmNew(false)}>
            Cancel
          </button>
        </div>
      </div>
    );
  }
  if (mapping.mode === 'mapping') {
    return (
      <>
        <p className="muted small">Drive around (or use Go to) until the rooms are filled in, then save.</p>
        <div className="actions">
          <button onClick={() => command('save_map')} disabled={disabled} title="Save the map so far and keep mapping">
            Save map
          </button>
          <button onClick={() => command('use_saved_map')} disabled={disabled}
            title="Save the map, stop mapping and navigate on it">
            Done, use this map
          </button>
        </div>
      </>
    );
  }
  const saved = mapping.saved;
  return (
    <div className="actions">
      {mapping.mode === 'navigation' ? (
        <button className={current === 'pose' ? 'on' : ''} aria-pressed={current === 'pose'} disabled={disabled}
          onClick={() => pick('pose')} title="Show the robot where it really is on the map">
          {current === 'pose' ? 'Pick a spot…' : 'Set position'}
        </button>
      ) : saved ? (
        <button onClick={() => command('use_saved_map')} disabled={disabled}>
          Use saved map
        </button>
      ) : null}
      {saved?.can_extend ? (
        <button onClick={() => command('extend_map')} disabled={disabled} title="Map more rooms on top of the saved map">
          Add to map
        </button>
      ) : null}
      <button onClick={() => (saved ? setConfirmNew(true) : command('new_map'))} disabled={disabled}>
        New map
      </button>
    </div>
  );
}

function NoticeLine({ notice }: { notice: Notice }) {
  return (
    <p className={`notice ${notice.level}`} role="status">
      {notice.text}
    </p>
  );
}

/** What to do while a floor tool is picked, top center. */
export function ToolHint() {
  const current = useStore(tool);
  if (current === 'none') return null;
  return (
    <div className="panel tool-hint" role="status">
      {current === 'goal' ? 'Click where the robot should go' : 'Click where the robot really is'}
      <span> · drag to aim it · Esc cancels</span>
    </div>
  );
}
