// Gearotons request/reply transport. Derived from Matthias Wandel's Oct 2026
// ServoTune attachment; original files are retained in original/.
export const COMMANDS = Object.freeze({
  DISABLE: 0, ENABLE: 1, ESTOP: 12, ZERO: 13, POSITION: 15, STATUS: 16,
  CLOSED_LOOP: 17, SPECS: 18, INFO: 22, VERSION: 25, RESET: 27,
  CURRENT_LIMIT: 28, PING: 31, POSITIONS: 37, SET_PID: 43, GET_PID: 48,
  TARGET: 49, TARGET_POSITION: 50,
});

// Development .1 had excessive ISR overhead; .2 uses the legacy reply.
// Stable .17.0.0 promotes the validated compact implementation from .3+.
export function firmwareCapabilities(version) {
  const atLeast = wanted => {
    for (let i = 0; i < wanted.length; i++) {
      if (version[i] !== wanted[i]) return version[i] > wanted[i];
    }
    return true;
  };
  const stableImmediate = version.every((value, i) => value === [0, 17, 0, 0][i]);
  return {
    immediate: stableImmediate || atLeast([0, 17, 0, 2]),
    compactTargetPosition: stableImmediate || atLeast([0, 17, 0, 3]),
    pid: atLeast([0, 16, 0, 0]),
  };
}

export function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc ^= byte;
    for (let i = 0; i < 8; i++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
  }
  return (~crc) >>> 0;
}

export function pack(size, fill) {
  const bytes = new Uint8Array(size);
  fill(new DataView(bytes.buffer));
  return bytes;
}

export function frame(body, crc = true) {
  let size = body.length + 1 + (crc ? 4 : 0);
  const extended = size >= 127;
  if (extended) size += 2;
  if (size > 65535) throw new Error('Packet too large');
  const bytes = new Uint8Array(size);
  const view = new DataView(bytes.buffer);
  if (extended) { bytes[0] = 255; view.setUint16(1, size, true); }
  else bytes[0] = (size << 1) | 1;
  bytes.set(body, extended ? 3 : 1);
  if (crc) view.setUint32(size - 4, crc32(bytes.subarray(0, size - 4)), true);
  return bytes;
}

export function parseAddress(text) {
  text = text.trim();
  if (/^[0-9a-f]{16}$/i.test(text)) return BigInt('0x' + text);
  const value = /^\d+$/.test(text) ? Number(text) : text.length === 1 ? text.charCodeAt(0) : NaN;
  if (!Number.isInteger(value) || value < 0 || value > 251) {
    throw new Error('Use a single alias character, alias 0–251, or a 16-digit unique ID');
  }
  return BigInt(value);
}

export function requestPacket(address, command, payload = new Uint8Array(), crc = true) {
  address = BigInt(address);
  if (address < 0n || address > 0xffffffffffffffffn ||
      (address >= 252n && address <= 255n)) throw new Error('A unicast address is required');
  const prefix = address <= 251n ? Uint8Array.of(Number(address), command) :
    pack(10, view => { view.setUint8(0, 254); view.setBigUint64(1, address, true); view.setUint8(9, command); });
  return frame(Uint8Array.from([...prefix, ...payload]), crc);
}

export class PacketParser {
  constructor(onPacket, onError) { this.onPacket = onPacket; this.onError = onError; this.buffer = []; }
  push(chunk) {
    this.buffer.push(...chunk);
    try {
      while (this.buffer.length) {
        const first = this.buffer[0];
        if (!(first & 1)) throw new Error('Invalid packet size byte');
        let size = first >>> 1, offset = 1;
        if (size === 127) {
          if (this.buffer.length < 3) return;
          size = this.buffer[1] | (this.buffer[2] << 8); offset = 3;
        }
        if (size < offset + 1 || size > 65535) throw new Error('Invalid packet size');
        if (this.buffer.length < size) return;
        const packet = Uint8Array.from(this.buffer.splice(0, size));
        const marker = packet[offset];
        if (marker !== 253 && marker !== 252) throw new Error('Invalid response marker');
        const crc = marker === 253;
        if (size < offset + 1 + (crc ? 4 : 0)) throw new Error('Truncated response');
        if (crc && new DataView(packet.buffer).getUint32(size - 4, true) !== crc32(packet.subarray(0, size - 4))) {
          throw new Error('Response CRC mismatch');
        }
        // The existing compact success ACK is only size + marker (+ CRC).
        // Data/error replies also carry an error byte after the marker.
        const end = size - (crc ? 4 : 0), ack = end === offset + 1;
        this.onPacket({ error: ack ? 0 : packet[offset + 1], crc,
          payload: ack ? new Uint8Array() : packet.slice(offset + 2, end), packet });
      }
    } catch (error) { this.buffer = []; this.onError(error); }
  }
}

export class MotorError extends Error {
  constructor(code) {
    const reasons = { 8: 'trajectory queue is not empty', 13: 'not in closed-loop position mode',
      19: 'motor is busy', 27: 'target outside safety limits', 45: 'position deviation too large',
      51: 'wrong command payload size', 55: 'output MOSFETs are disabled' };
    super(`Fatal motor error ${code}: ${reasons[code] || 'reset required'}`);
    this.code = code;
  }
}

export class ServoClient {
  constructor(transport, address, { timeoutMs = 1200, compactTargetPosition = false } = {}) {
    this.transport = transport; this.address = BigInt(address); this.timeoutMs = timeoutMs;
    this.compactTargetPosition = compactTargetPosition;
    this.tail = Promise.resolve(); this.pending = null; this.fault = null;
    this.resetting = false; this.resetAckSeen = false;
    this.generation = 0; this.completed = 0; this.maxOutstanding = 0; this.trace = [];
    this.parser = new PacketParser(packet => this.receive(packet), error => this.fail(error));
    transport.onData = data => { if (!this.fault) this.parser.push(data); };
    transport.onError = error => this.fail(error);
  }
  fail(error) {
    this.fault = error; this.generation++;
    if (this.pending) { const pending = this.pending; this.pending = null; clearTimeout(pending.timer); pending.reject(error); }
  }
  receive(packet) {
    if (this.resetting && !this.pending) {
      // The normal firmware reset is silent. Its fatal-error handler sends
      // one compact ACK before resetting. Neither ends the quiet interval.
      if (packet.crc && !packet.error && !packet.payload.length && !this.resetAckSeen) {
        this.resetAckSeen = true;
        this.trace.push({ kind: 'reset_ack', command: COMMANDS.RESET, bytes: Array.from(packet.packet) });
        return;
      }
      this.fail(new Error('Unexpected packet during reset; reconnect to resynchronize'));
      return;
    }
    if (!this.pending) { this.fail(new Error('Unexpected reply; reconnect to resynchronize')); return; }
    const pending = this.pending;
    this.pending = null; clearTimeout(pending.timer);
    this.trace.push({ kind: 'reply', command: pending.command, bytes: Array.from(packet.packet) });
    if (!packet.crc) { this.fail(new Error('CRC-disabled reply on a CRC-enabled connection')); pending.reject(this.fault); return; }
    if (packet.error) { this.generation++; pending.reject(new MotorError(packet.error)); return; }
    if (packet.payload.length !== pending.expectedBytes) {
      this.fail(new Error(`Unexpected payload length for command ${pending.command}`)); pending.reject(this.fault); return;
    }
    this.completed++; pending.resolve(packet.payload);
  }
  request(command, payload = new Uint8Array(), expectedBytes = 0) {
    const generation = this.generation;
    const job = this.tail.then(async () => {
      if (this.fault) throw this.fault;
      if (generation !== this.generation) throw new Error('Command cancelled after an earlier failure');
      const packet = requestPacket(this.address, command, payload);
      if (expectedBytes === null) {
        this.trace.push({ kind: 'request', command, bytes: Array.from(packet) });
        this.resetting = true; this.resetAckSeen = false;
        try {
          await this.transport.write(packet);
          // Reserve the entire bootloader quiet interval, including when the
          // fatal-error handler acknowledges the reset immediately.
          await new Promise(resolve => setTimeout(resolve, 1500));
          if (this.fault) throw this.fault;
          return new Uint8Array();
        } catch (error) {
          if (!this.fault) this.fail(error);
          throw error;
        } finally { this.resetting = false; }
      }
      return new Promise((resolve, reject) => {
        if (this.pending) { reject(new Error('Internal transaction overlap')); return; }
        const pending = { command, expectedBytes, resolve, reject };
        this.pending = pending; this.maxOutstanding = Math.max(this.maxOutstanding, 1);
        pending.timer = setTimeout(() => this.fail(new Error('Reply timeout; reconnect before sending another command')), this.timeoutMs);
        this.trace.push({ kind: 'request', command, bytes: Array.from(packet) });
        if (this.trace.length > 200) this.trace.splice(0, 100);
        Promise.resolve().then(() => this.transport.write(packet)).catch(error => this.fail(error));
      });
    });
    this.tail = job.catch(() => {});
    return job;
  }
  cancelQueued() { this.generation++; }
  async status() { const p = await this.request(COMMANDS.STATUS, undefined, 3); return { flags: new DataView(p.buffer).getUint16(0, true), error: p[2] }; }
  async reset() { await this.request(COMMANDS.RESET, undefined, null); }
  async positions() {
    // Existing Get comprehensive position: commanded position, Hall position,
    // external encoder. Read both chart traces from the same transaction.
    const p = await this.request(COMMANDS.POSITIONS, undefined, 20);
    const v = new DataView(p.buffer);
    return { target: v.getBigInt64(0, true), measured: v.getBigInt64(8, true),
      externalEncoder: v.getInt32(16, true) };
  }
  async pid() { const p = await this.request(COMMANDS.GET_PID, undefined, 12); const v = new DataView(p.buffer); return [0, 4, 8].map(i => v.getUint32(i, true)); }
  async setPid(p, i, d) {
    for (const gain of [p, i, d]) if (!Number.isInteger(gain) || gain < 0 || gain > 0xffffffff) throw new Error('PID constants must be unsigned 32-bit integers');
    await this.request(COMMANDS.SET_PID, pack(12, v => [p, i, d].forEach((x, n) => v.setUint32(n * 4, x, true))));
  }
  async target(position, withPosition = true) {
    const target = BigInt(position);
    if (target < -(1n << 63n) || target >= (1n << 63n)) throw new Error('Target exceeds signed 64-bit range');
    const compact = this.compactTargetPosition;
    const p = await this.request(withPosition ? COMMANDS.TARGET_POSITION : COMMANDS.TARGET,
      pack(8, v => v.setBigInt64(0, target, true)), withPosition ? (compact ? 8 : 24) : 0);
    if (!withPosition) return null;
    const v = new DataView(p.buffer);
    // Compact firmware returns only Hall position. A successful command accepts
    // the exact requested reference; retain that local value for the chart.
    if (compact) return { target, measured: v.getBigInt64(0, true) };
    return { target: v.getBigInt64(0, true), measured: v.getBigInt64(8, true),
      sampleCounter: v.getUint32(16, true), flags: v.getUint16(20, true), mode: p[22], queue: p[23] };
  }
  async close() {
    this.fail(new Error('Connection closed'));
    await this.transport.close();
  }
}

export class WebSerialTransport {
  static async open() {
    if (!globalThis.navigator?.serial) throw new Error('Web Serial is unavailable. Open localhost in desktop Chrome or Edge. Demo mode works without Web Serial.');
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 230400 });
    const transport = new WebSerialTransport();
    transport.port = port; transport.writer = port.writable.getWriter();
    transport.reader = port.readable.getReader(); transport.closing = false;
    transport.readTask = transport.readLoop();
    // Drain old/late bytes before the first request; no commands are sent here.
    await new Promise(resolve => setTimeout(resolve, 150));
    return transport;
  }
  async readLoop() {
    try {
      while (!this.closing) {
        const { value, done } = await this.reader.read();
        if (done) { if (!this.closing) this.onError?.(new Error('Serial connection ended')); break; }
        if (value) this.onData?.(value);
      }
    } catch (error) { if (!this.closing) this.onError?.(error); }
    finally { this.reader.releaseLock(); }
  }
  async write(bytes) { await this.writer.write(bytes); }
  async close() {
    this.closing = true;
    await this.reader.cancel(); await this.readTask;
    this.writer.releaseLock(); await this.port.close();
  }
}
