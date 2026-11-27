import { Store } from '../store';

/**
 * Talks to the robot (or the sim) through our web bridge (ros2_ws/src/roomba_web), a websocket
 * server on port 8765. Text frames are JSON objects with an "op" field; binary frames are camera
 * pictures. The format is described in roomba_web/protocol.py.
 */

export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected';

export type ConnectionInfo = {
  url: string;
  status: ConnectionStatus;
  /** True when connected to the simulation. */
  sim: boolean;
  /** The robot's clock (sim time in the sim), in seconds. */
  serverTime: number | null;
};

export type CameraHeader = {
  frame: string;
  stamp: number;
  width: number;
  height: number;
  model: { fx: number; fy: number; cx: number; cy: number; width: number; height: number } | null;
};

export type CameraMessage = { op: 'camera'; header: CameraHeader; jpeg: Blob };

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type Message = { op: string; [key: string]: any };
export type MessageHandler = (msg: Message) => void;

const RECONNECT_MS = 2000;
const CAMERA_FRAME = 1;
const GRID_FRAME = 2;

/** Splits a binary camera frame: [1][uint32 LE header length][JSON header][JPEG]. */
export function parseCameraFrame(data: ArrayBuffer): CameraMessage | null {
  const bytes = new Uint8Array(data);
  if (bytes.length < 5 || bytes[0] !== CAMERA_FRAME) return null;
  const length = new DataView(data).getUint32(1, true);
  if (5 + length > bytes.length) return null;
  const header = JSON.parse(new TextDecoder().decode(bytes.subarray(5, 5 + length))) as CameraHeader;
  return { op: 'camera', header, jpeg: new Blob([bytes.subarray(5 + length)], { type: 'image/jpeg' }) };
}

/** Binary frames: camera pictures, or grids (the map, the costmap) that RobotData decodes. */
function parseBinary(data: ArrayBuffer): Message | null {
  const kind = new Uint8Array(data, 0, Math.min(1, data.byteLength))[0];
  if (kind === CAMERA_FRAME) return parseCameraFrame(data);
  if (kind === GRID_FRAME) return { op: 'grid', data };
  return null;
}

export class RobotConnection {
  readonly info: Store<ConnectionInfo>;

  private ws: WebSocket | null = null;
  private handlers = new Map<string, Set<MessageHandler>>();
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private closed = false;

  constructor(readonly url: string) {
    this.info = new Store<ConnectionInfo>({ url, status: 'disconnected', sim: false, serverTime: null });
  }

  connect(): void {
    this.closed = false;
    this.open();
  }

  close(): void {
    this.closed = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.ws?.close();
    this.ws = null;
  }

  /** Calls `handler` for every message with this op. Returns a function that stops it. */
  on(op: string, handler: MessageHandler): () => void {
    let set = this.handlers.get(op);
    if (!set) {
      set = new Set();
      this.handlers.set(op, set);
    }
    set.add(handler);
    return () => set.delete(handler);
  }

  /** Sends a JSON message to the bridge. Returns false when not connected. */
  send(msg: Message): boolean {
    const ws = this.ws;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    ws.send(JSON.stringify(msg));
    return true;
  }

  private emit(msg: Message): void {
    this.handlers.get(msg.op)?.forEach((h) => h(msg));
  }

  private open(): void {
    this.info.update({ status: 'connecting' });
    const ws = new WebSocket(this.url);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onmessage = (event) => {
      let msg: Message | null = null;
      try {
        msg = typeof event.data === 'string' ? (JSON.parse(event.data) as Message) : parseBinary(event.data);
      } catch (err) {
        console.warn('Bad message from the robot', err);
      }
      if (!msg) return;
      if (msg.op === 'hello') this.info.update({ status: 'connected', sim: Boolean(msg.sim) });
      else if (msg.op === 'status') this.info.update({ serverTime: msg.time });
      this.emit(msg);
    };
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.info.update({ status: 'disconnected', serverTime: null });
      this.emit({ op: 'disconnected' });
      if (!this.closed) this.reconnectTimer = setTimeout(() => this.open(), RECONNECT_MS);
    };
  }
}

/** ws://<page host>:8765 unless the page URL says ?ws=ws://somewhere:port */
export function defaultBridgeUrl(): string {
  const fromQuery = new URLSearchParams(window.location.search).get('ws');
  if (fromQuery) return fromQuery;
  return `ws://${window.location.hostname || 'localhost'}:8765`;
}
