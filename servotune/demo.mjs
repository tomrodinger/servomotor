// Browser demonstration only: this is not a physical motor or PID simulation.
// It consumes real request packets and returns CRC-checked, fragmented replies.
import { COMMANDS as C, crc32, frame, pack } from './protocol.mjs';

export class DemoTransport {
  constructor({ latencyMs = 10, fragment = true, compactTargetPosition = true } = {}) {
    this.latencyMs = latencyMs; this.fragment = fragment; this.closed = false;
    this.compactTargetPosition = compactTargetPosition;
    this.inFlight = 0; this.maxInFlight = 0; this.received = []; this.reset();
  }
  reset() {
    this.flags = 0; this.error = 0; this.gains = [2000, 5, 175000];
    this.targetPosition = 0n; this.measuredPosition = 0; this.queue = 0;
    this.started = performance.now(); this.lastUpdate = this.started;
  }
  update() {
    const now = performance.now(), elapsed = now - this.lastUpdate; this.lastUpdate = now;
    if ((this.flags & 6) === 6) this.measuredPosition += (Number(this.targetPosition) - this.measuredPosition) * (1 - Math.exp(-elapsed / 70));
  }
  reply(command, p) {
    this.update();
    if (command === C.RESET) {
      const acknowledge = this.error !== 0;
      this.reset();
      return acknowledge ? { error: 0, payload: new Uint8Array() } : null;
    }
    if (this.error) return { error: this.error, payload: new Uint8Array() };
    let payload = new Uint8Array();
    const sizes = { [C.SET_PID]: 12, [C.TARGET]: 8, [C.TARGET_POSITION]: 8, [C.CURRENT_LIMIT]: 4, [C.PING]: 10 };
    if (p.length !== (sizes[command] || 0)) this.error = 51;
    if (!this.error) switch (command) {
      case C.INFO: payload = pack(28, v => {
        [...'M17     '].forEach((c, i) => v.setUint8(i, c.charCodeAt(0)));
        v.setUint8(8, 3); v.setUint8(10, 5); v.setUint8(11, 1);
        v.setUint32(12, 1, true); v.setBigUint64(16, 0x04245c407d26ccd0n, true);
      }); break;
      case C.VERSION: payload = Uint8Array.of(this.compactTargetPosition ? 0 : 2, 0, 17, 0, 0); break;
      case C.SPECS: payload = pack(8, v => { v.setUint32(0, 31250, true); v.setUint32(4, 3276800, true); }); break;
      case C.STATUS: payload = pack(3, v => { v.setUint16(0, this.flags, true); v.setUint8(2, 0); }); break;
      case C.CLOSED_LOOP: this.flags = 6; break;
      case C.ENABLE: this.flags |= 2; break;
      case C.DISABLE: this.flags &= ~2; break;
      case C.ESTOP: this.flags &= ~2; this.queue = 0; break;
      case C.ZERO: this.targetPosition = 0n; this.measuredPosition = 0; break;
      case C.CURRENT_LIMIT: break;
      case C.PING: payload = p; break;
      case C.POSITION: payload = pack(8, v => v.setBigInt64(0, BigInt(Math.round(this.measuredPosition)), true)); break;
      case C.POSITIONS: payload = pack(20, v => {
        v.setBigInt64(0, this.targetPosition, true);
        v.setBigInt64(8, BigInt(Math.round(this.measuredPosition)), true);
        v.setInt32(16, 0, true);
      }); break;
      case C.SET_PID: { const v = new DataView(p.buffer); this.gains = [0, 4, 8].map(i => v.getUint32(i, true)); break; }
      case C.GET_PID: payload = pack(12, v => this.gains.forEach((g, i) => v.setUint32(i * 4, g, true))); break;
      case C.TARGET:
      case C.TARGET_POSITION: {
        const target = new DataView(p.buffer).getBigInt64(0, true);
        if (this.queue) this.error = 8;
        else if (!(this.flags & 4)) this.error = 13;
        else if (!(this.flags & 2)) this.error = 55;
        else if (Math.abs(Number(target) - this.measuredPosition) > 6553600) this.error = 45;
        if (this.error) break;
        this.targetPosition = target;
        if (command === C.TARGET_POSITION) {
          payload = this.compactTargetPosition ?
            pack(8, v => v.setBigInt64(0, BigInt(Math.round(this.measuredPosition)), true)) :
            pack(24, v => {
              v.setBigInt64(0, target, true); v.setBigInt64(8, BigInt(Math.round(this.measuredPosition)), true);
              v.setUint32(16, Math.floor((performance.now() - this.started) * 31.25) >>> 0, true);
              v.setUint16(20, this.flags, true); v.setUint8(22, 1); v.setUint8(23, this.queue);
            });
        }
        break;
      }
      default: throw new Error('Demo does not implement command ' + command);
    }
    if (this.error) this.flags = 0;
    return { error: this.error, payload: this.error ? new Uint8Array() : payload };
  }
  async write(bytes) {
    if (this.closed) throw new Error('Demo closed');
    const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (crc32(bytes.subarray(0, bytes.length - 4)) !== v.getUint32(bytes.length - 4, true)) return;
    const extendedSize = bytes[0] === 255, start = extendedSize ? 3 : 1;
    const extendedAddress = bytes[start] === 254;
    const address = extendedAddress ? v.getBigUint64(start + 1, true) : BigInt(bytes[start]);
    if (address !== 88n && address !== 0x04245c407d26ccd0n) return;
    const offset = start + (extendedAddress ? 9 : 1), command = bytes[offset];
    this.received.push(command);
    this.inFlight++; this.maxInFlight = Math.max(this.maxInFlight, this.inFlight);
    const result = this.reply(command, bytes.slice(offset + 1, bytes.length - 4));
    if (result === null) { this.inFlight--; return; }
    const response = frame(!result.error && !result.payload.length ? Uint8Array.of(253) :
      Uint8Array.from([253, result.error, ...result.payload]));
    setTimeout(() => {
      if (this.closed) { this.inFlight--; return; }
      if (this.fragment) {
        this.onData?.(response.subarray(0, 2));
        setTimeout(() => { this.inFlight--; if (!this.closed) this.onData?.(response.subarray(2)); }, 1);
      } else { this.inFlight--; this.onData?.(response); }
    }, this.latencyMs);
  }
  async close() { this.closed = true; }
}
