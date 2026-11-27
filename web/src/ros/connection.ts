import { parse } from '@foxglove/rosmsg';
import { MessageReader } from '@foxglove/rosmsg2-serialization';
import { FoxgloveClient } from '@foxglove/ws-protocol';
import type { Channel } from '@foxglove/ws-protocol';

import { Store } from '../store';

/**
 * Talks to the robot (or the sim) through foxglove_bridge, the websocket server that runs next to
 * ROS on port 8765. The bridge only moves bytes; this class decodes the ROS 2 (CDR) messages.
 */

export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected';

export type ConnectionInfo = {
  url: string;
  status: ConnectionStatus;
  /** Topic name -> ROS type, for everything the robot currently publishes. */
  topics: Record<string, string>;
  /** Latest server clock (sim time when the robot uses it), in seconds. */
  serverTime: number | null;
  canPublish: boolean;
};

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type MessageHandler = (msg: any) => void;

type Subscription = { subscriptionId: number; channel: Channel; reader: MessageReader };

const RECONNECT_MS = 2000;

export class RobotConnection {
  readonly info: Store<ConnectionInfo>;

  private client: FoxgloveClient | null = null;
  private channels = new Map<string, Channel>();
  private subscriptions = new Map<string, Subscription>();
  private bySubscriptionId = new Map<number, Subscription>();
  private handlers = new Map<string, Set<MessageHandler>>();
  private publishers = new Map<string, number>();
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private closed = false;

  constructor(readonly url: string) {
    this.info = new Store<ConnectionInfo>({
      url,
      status: 'disconnected',
      topics: {},
      serverTime: null,
      canPublish: false,
    });
  }

  connect(): void {
    this.closed = false;
    this.open();
  }

  close(): void {
    this.closed = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.client?.close();
    this.client = null;
  }

  /** Calls `handler` for every message on `topic`. Returns a function that stops it. */
  on(topic: string, handler: MessageHandler): () => void {
    let set = this.handlers.get(topic);
    if (!set) {
      set = new Set();
      this.handlers.set(topic, set);
    }
    set.add(handler);
    this.syncSubscription(topic);
    return () => {
      set.delete(handler);
      this.syncSubscription(topic);
    };
  }

  /** Publishes already-encoded CDR bytes. Returns false when not connected. */
  publish(topic: string, schemaName: string, cdr: Uint8Array): boolean {
    const client = this.client;
    if (!client || this.info.get().status !== 'connected') return false;
    let id = this.publishers.get(topic);
    if (id === undefined) {
      id = client.advertise({ topic, encoding: 'cdr', schemaName });
      this.publishers.set(topic, id);
    }
    client.sendMessage(id, cdr);
    return true;
  }

  private open(): void {
    this.info.update({ status: 'connecting' });
    const ws = new WebSocket(this.url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]);
    const client = new FoxgloveClient({ ws });
    this.client = client;

    client.on('open', () => this.info.update({ status: 'connected' }));
    client.on('serverInfo', (info) => {
      this.info.update({ canPublish: info.capabilities.includes('clientPublish') });
    });
    client.on('time', (t) => this.info.update({ serverTime: Number(t.timestamp) * 1e-9 }));
    client.on('advertise', (channels) => {
      for (const ch of channels) {
        if (!this.channels.has(ch.topic)) this.channels.set(ch.topic, ch);
      }
      this.publishTopicList();
      for (const ch of channels) this.syncSubscription(ch.topic);
    });
    client.on('unadvertise', (ids) => {
      for (const [topic, ch] of this.channels) {
        if (!ids.includes(ch.id)) continue;
        this.channels.delete(topic);
        const sub = this.subscriptions.get(topic);
        if (sub) {
          this.bySubscriptionId.delete(sub.subscriptionId);
          this.subscriptions.delete(topic);
        }
      }
      this.publishTopicList();
    });
    client.on('message', ({ subscriptionId, data }) => {
      const sub = this.bySubscriptionId.get(subscriptionId);
      if (!sub) return;
      let msg: unknown;
      try {
        msg = sub.reader.readMessage(data);
      } catch (err) {
        console.warn(`Could not decode ${sub.channel.topic}`, err);
        return;
      }
      this.handlers.get(sub.channel.topic)?.forEach((h) => h(msg));
    });
    client.on('error', () => {
      /* followed by close */
    });
    client.on('close', () => {
      if (this.client !== client) return;
      this.client = null;
      this.channels.clear();
      this.subscriptions.clear();
      this.bySubscriptionId.clear();
      this.publishers.clear();
      this.info.update({ status: 'disconnected', topics: {}, canPublish: false });
      if (!this.closed) this.reconnectTimer = setTimeout(() => this.open(), RECONNECT_MS);
    });
  }

  private publishTopicList(): void {
    const topics: Record<string, string> = {};
    for (const [topic, ch] of this.channels) topics[topic] = ch.schemaName;
    this.info.update({ topics });
  }

  /** Subscribes while someone is listening and the topic exists; unsubscribes otherwise. */
  private syncSubscription(topic: string): void {
    const client = this.client;
    const wanted = (this.handlers.get(topic)?.size ?? 0) > 0;
    const sub = this.subscriptions.get(topic);
    if (!client) return;
    if (!wanted && sub) {
      client.unsubscribe(sub.subscriptionId);
      this.subscriptions.delete(topic);
      this.bySubscriptionId.delete(sub.subscriptionId);
      return;
    }
    const channel = this.channels.get(topic);
    if (!wanted || sub || !channel) return;
    const reader = makeReader(channel);
    if (!reader) return;
    const subscriptionId = client.subscribe(channel.id);
    const created = { subscriptionId, channel, reader };
    this.subscriptions.set(topic, created);
    this.bySubscriptionId.set(subscriptionId, created);
  }
}

function makeReader(channel: Channel): MessageReader | null {
  if (channel.encoding !== 'cdr' || (channel.schemaEncoding && channel.schemaEncoding !== 'ros2msg')) {
    console.warn(`Skipping ${channel.topic}: unsupported encoding ${channel.encoding}/${channel.schemaEncoding}`);
    return null;
  }
  try {
    return new MessageReader(parse(channel.schema, { ros2: true }));
  } catch (err) {
    console.warn(`Could not parse the schema for ${channel.topic}`, err);
    return null;
  }
}

/** ws://<page host>:8765 unless the page URL says ?ws=ws://somewhere:port */
export function defaultBridgeUrl(): string {
  const fromQuery = new URLSearchParams(window.location.search).get('ws');
  if (fromQuery) return fromQuery;
  return `ws://${window.location.hostname || 'localhost'}:8765`;
}
