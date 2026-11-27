import { useRobot } from '../context';
import { TOPICS } from '../ros/robotData';
import { useStore } from '../store';

const CHARGING = ['Not charging', 'Reconditioning', 'Full charge', 'Trickle charge', 'Waiting', 'Charge fault'];

function fmt(v: number | null, digits: number, unit: string): string {
  return v === null ? '—' : `${v.toFixed(digits)} ${unit}`;
}

function clock(seconds: number): string {
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m}:${s.toString().padStart(2, '0')}`;
}

/** Connection, speed, lidar rate and battery, top left. */
export function StatusPanel() {
  const { connection, data } = useRobot();
  const info = useStore(connection.info);
  const status = useStore(data.status);
  const isSim = TOPICS.world in info.topics || '/clock' in info.topics;
  const label =
    info.status === 'connected' ? (isSim ? 'Connected to the sim' : 'Connected to the robot') : info.status === 'connecting' ? 'Connecting…' : 'Not connected';
  return (
    <div className="panel status">
      <div className="title">
        <span className={`dot ${info.status}`} />
        <div>
          <div className="name">Roomba</div>
          <div className="sub">{label}</div>
        </div>
      </div>
      <dl>
        <dt>Speed</dt>
        <dd>{fmt(status.speed, 2, 'm/s')}</dd>
        <dt>Turning</dt>
        <dd>{fmt(status.turnRate, 2, 'rad/s')}</dd>
        <dt>Lidar</dt>
        <dd>{fmt(status.scanHz, 1, 'Hz')}</dd>
        {status.battery !== null ? (
          <>
            <dt>Battery</dt>
            <dd>
              {Math.round(status.battery * 100)}%
              {status.chargingState !== null && status.chargingState > 0 ? ` · ${CHARGING[status.chargingState] ?? ''}` : ''}
            </dd>
          </>
        ) : null}
        {isSim && info.serverTime !== null ? (
          <>
            <dt>Sim time</dt>
            <dd>{clock(info.serverTime)}</dd>
          </>
        ) : null}
        <dt>Frame</dt>
        <dd>{status.fixedFrame}</dd>
      </dl>
    </div>
  );
}

/** Shown when the bridge can't be reached, with the command that fixes it. */
export function ConnectionHelp() {
  const { connection } = useRobot();
  const info = useStore(connection.info);
  if (info.status === 'connected') return null;
  return (
    <div className="panel help">
      <strong>Waiting for the robot at {info.url}</strong>
      <p>
        Start the sim with <code>./scripts/sim.sh up</code>, or start the robot. This page keeps retrying, so it
        connects as soon as one is up.
      </p>
      <p className="hint">
        To use another address, open this page with <code>?ws=ws://roomba.local:8765</code>
      </p>
    </div>
  );
}
