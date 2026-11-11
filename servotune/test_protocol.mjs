// No browser, motor or npm dependencies required: node servotune/test_protocol.mjs
import assert from 'node:assert/strict';
import { COMMANDS as C, ServoClient, PacketParser, MotorError, frame, pack,
  requestPacket, parseAddress, firmwareCapabilities } from './protocol.mjs';
import { DemoTransport } from './demo.mjs';

// Stable versions sort below their prerelease byte values in this protocol.
// Preserve the legacy .2 reply and the deliberately unsupported .1 build.
for (const [version, immediate, compact, pid] of [
  [[0,16,0,0],false,false,true], [[0,17,0,0],true,true,true],
  [[0,17,0,1],false,false,true], [[0,17,0,2],true,false,true],
  [[0,17,0,3],true,true,true], [[0,17,0,6],true,true,true],
  [[0,17,1,0],true,true,true], [[0,18,0,0],true,true,true],
  [[0,15,12,0],false,false,false],
]) {
  assert.deepEqual(firmwareCapabilities(version),
    { immediate, compactTargetPosition: compact, pid }, version.join('.'));
}

const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
const hex = bytes => Buffer.from(bytes).toString('hex');
// Actual compact success ACK from common_source_files/RS485.h (CRC enabled).
assert.equal(hex(frame(Uint8Array.of(253))), '0dfd13e27b37');
assert.equal(hex(requestPacket(88n, 48)), '0f5830df79a6a0');
// Cross-language wire examples in PROTOCOL_SPEC.md, also checked against the
// compiled C handler by test_host_immediate_target.py.
assert.equal(hex(requestPacket(88n, C.TARGET, pack(8, v => v.setBigInt64(0, 1000n, true)))), '1f5831e80300000000000085b7dda4');
assert.equal(hex(requestPacket(88n, C.TARGET_POSITION, pack(8, v => v.setBigInt64(0, 1000n, true)))), '1f5832e803000000000000408b509d');
assert.equal(parseAddress('X'), 88n);
assert.equal(parseAddress('88'), 88n);
assert.equal(parseAddress('04245C407D26CCD0'), 0x04245c407d26ccd0n);
for (const invalid of ['255', '254', '-1', 'hello', '']) assert.throws(() => parseAddress(invalid));
const received = [], faults = [];
const parser = new PacketParser(p => received.push(p), e => faults.push(e));
const ack = frame(Uint8Array.of(253));
const state = frame(Uint8Array.from([253, 0, ...pack(24, v => {
  v.setBigInt64(0, 9007199254740993n, true); v.setBigInt64(8, -9007199254740993n, true);
  v.setUint32(16, 0xffffffff, true); v.setUint16(20, 6, true); v.setUint8(22, 1);
})]));
// Several replies in one USB chunk, and a complete reply delivered one byte at a time.
parser.push(Uint8Array.from([...ack, ...state]));
for (const byte of state) parser.push(Uint8Array.of(byte));
assert.equal(received.length, 3); assert.equal(faults.length, 0);
assert.equal(received[0].payload.length, 0); assert.equal(received[0].error, 0);
assert.equal(new DataView(received[1].payload.buffer).getBigInt64(0, true), 9007199254740993n);
const corrupt = state.slice(); corrupt[corrupt.length - 1] ^= 1; parser.push(corrupt);
assert.equal(faults.length, 1); assert.match(faults[0].message, /CRC/);
const longReply = frame(Uint8Array.from([253, 0, ...new Uint8Array(200)]));
parser.push(longReply); assert.equal(received.at(-1).payload.length, 200);
parser.push(Uint8Array.of(5, 252)); assert.equal(received.at(-1).crc, false);
parser.push(Buffer.from('3ffd00e80300000000000084030000000000002b00000006000100e9490ef3', 'hex'));
assert.equal(new DataView(received.at(-1).payload.buffer).getBigInt64(8, true), 900n);

// Shared with the actual-C handler/framing test: target 1000, measured 900.
// Consume the firmware's literal compact packet through the browser client,
// rather than generating the reply with the JavaScript demo/encoder.
const compactFirmwareReply = Buffer.from('1ffd0084030000000000007517a255', 'hex');
const firmwareVectorTransport = {
  async write(packet) {
    assert.equal(hex(packet), '1f5832e803000000000000408b509d');
    for (const byte of compactFirmwareReply) this.onData(Uint8Array.of(byte));
  },
  async close() {},
};
const firmwareVectorClient = new ServoClient(firmwareVectorTransport, 88n, { compactTargetPosition: true });
assert.deepEqual(await firmwareVectorClient.target(1000n), { target: 1000n, measured: 900n });
assert.equal(firmwareVectorClient.completed, 1);
await firmwareVectorClient.close();

for (const compactTargetPosition of [false, true]) for (const address of [88n, 0x04245c407d26ccd0n]) {
  const transport = new DemoTransport({ latencyMs: 12, compactTargetPosition });
  const client = new ServoClient(transport, address, { compactTargetPosition });
  assert.deepEqual(await client.pid(), [2000, 5, 175000]);
  assert.deepEqual(transport.received, [C.GET_PID]); // Reads must not write defaults.
  await assert.rejects(client.target(100n), error => error instanceof MotorError && error.code === 13);
  await client.reset();
  assert.equal(client.resetAckSeen, true); // Fatal-error reset emits a compact ACK.
  await client.request(C.CLOSED_LOOP);
  await client.setPid(0xffffffff, 0, 175001);
  assert.deepEqual(await client.pid(), [0xffffffff, 0, 175001]);
  // Submit many callers concurrently. Only one request may reach the transport
  // until its full fragmented reply has arrived, including its trailing CRC.
  const jobs = Array.from({ length: 50 }, (_, i) => client.target(BigInt(i * 10)));
  const replies = await Promise.all(jobs);
  replies.forEach((s, i) => {
    assert.equal(s.target, BigInt(i * 10)); assert.equal(typeof s.measured, 'bigint');
    if (compactTargetPosition) assert.deepEqual(Object.keys(s).sort(), ['measured', 'target']);
    else { assert.equal(s.queue, 0); assert.equal(s.mode, 1); }
  });
  const targetReplies = client.trace.filter(item => item.kind === 'reply' && item.command === C.TARGET_POSITION && item.bytes[2] === 0);
  assert.ok(targetReplies.length > 0);
  assert.ok(targetReplies.every(item => item.bytes.length === (compactTargetPosition ? 15 : 31)));
  assert.equal(transport.maxInFlight, 1); assert.equal(client.maxOutstanding, 1);
  await client.target(-25n, false);
  const positions = await client.positions();
  assert.equal(positions.target, -25n);
  assert.equal(typeof positions.measured, 'bigint');
  assert.equal(positions.externalEncoder, 0);
  await client.request(C.DISABLE);
  await assert.rejects(client.target(0n), error => error.code === 55);
  await client.reset();
  assert.deepEqual(await client.pid(), [2000, 5, 175000]);
  await client.request(C.CLOSED_LOOP);
  transport.queue = 1;
  const oldTarget = transport.targetPosition;
  await assert.rejects(client.target(100n), error => error.code === 8);
  assert.equal(transport.queue, 1); assert.equal(transport.targetPosition, oldTarget);
  await client.close();
}

// Fatal errors invalidate already queued work; a later explicit reset is allowed.
const demo = new DemoTransport();
const link = new ServoClient(demo, 88n);
const bad = link.target(10n), stale = link.request(C.CLOSED_LOOP);
const badResults = await Promise.allSettled([bad, stale]);
assert.equal(badResults[0].reason.code, 13);
assert.match(badResults[1].reason.message, /cancelled/);
assert.deepEqual(demo.received, [C.TARGET_POSITION]);
await link.close();

// A delayed reply after timeout must not become the next command's reply.
const slow = new DemoTransport({ latencyMs: 60 });
const timed = new ServoClient(slow, 88n, { timeoutMs: 10 });
await assert.rejects(timed.pid(), /timeout/);
await delay(80);
await assert.rejects(timed.status(), /timeout/);
assert.deepEqual(slow.received, [C.GET_PID]);
await timed.close();

// A failed write is a connection fault, not a reason to advance the queue.
let writes = 0;
const broken = new ServoClient({ write() { writes++; throw new Error('write failed'); }, async close() {} }, 88n);
await assert.rejects(broken.pid(), /write failed/);
await assert.rejects(broken.status(), /write failed/);
assert.equal(writes, 1);
await broken.close();
// Ordinary resets are silent. Both paths reserve the quiet interval even if
// a second caller has already queued a request.
const normalResetTransport = new DemoTransport();
const normalReset = new ServoClient(normalResetTransport, 88n);
const resetStarted = performance.now();
await Promise.all([normalReset.reset(), normalReset.status()]);
assert.equal(normalReset.resetAckSeen, false);
assert.ok(performance.now() - resetStarted >= 1450);
assert.deepEqual(normalResetTransport.received, [C.RESET, C.STATUS]);
await normalReset.close();
// Decode the existing comprehensive-position schema independently of the demo:
// signed, distinct reference and Hall position, including values above 2^53.
const positionTransport = {
  async write(packet) {
    assert.equal(packet[2], 37);
    this.onData(frame(Uint8Array.from([253, 0, ...pack(20, v => {
      v.setBigInt64(0, 9007199254740993n, true);
      v.setBigInt64(8, -9007199254740995n, true);
      v.setInt32(16, -12345, true);
    })])));
  },
  async close() {},
};
const positionClient = new ServoClient(positionTransport, 88n);
assert.deepEqual(await positionClient.positions(), {
  target: 9007199254740993n, measured: -9007199254740995n, externalEncoder: -12345,
});
await positionClient.close();

// Both schemas preserve exact signed-i64 values. The compact schema derives
// only the accepted reference from the request and never fabricates metadata.
for (const compactTargetPosition of [false, true]) {
  const measurements = [-(1n << 63n), -9007199254740993n, 0n, 9007199254740993n, (1n << 63n) - 1n];
  let measured;
  const transport = {
    async write(packet) {
      assert.equal(packet[2], C.TARGET_POSITION);
      const target = new DataView(packet.buffer).getBigInt64(3, true);
      const payload = compactTargetPosition ? pack(8, v => v.setBigInt64(0, measured, true)) :
        pack(24, v => {
          v.setBigInt64(0, target, true); v.setBigInt64(8, measured, true);
          v.setUint32(16, 123, true); v.setUint16(20, 6, true); v.setUint8(22, 1);
        });
      for (const byte of frame(Uint8Array.from([253, 0, ...payload]))) this.onData(Uint8Array.of(byte));
    },
    async close() {},
  };
  const client = new ServoClient(transport, 88n, { compactTargetPosition });
  for (const target of measurements) {
    measured = measurements[measurements.length - 1 - measurements.indexOf(target)];
    const state = await client.target(target);
    assert.equal(state.target, target); assert.equal(state.measured, measured);
    if (compactTargetPosition) assert.deepEqual(state, { target, measured });
  }
  await client.close();
}
// A version/schema mismatch is a connection fault, never grounds to retry a
// motion command or accept an ambiguous reply. Exercise both mismatch directions.
for (const compactTargetPosition of [false, true]) {
  const transport = new DemoTransport({ compactTargetPosition: !compactTargetPosition });
  const client = new ServoClient(transport, 88n, { compactTargetPosition });
  await client.request(C.CLOSED_LOOP);
  await assert.rejects(client.target(100n), /Unexpected payload length for command 50/);
  await assert.rejects(client.target(200n), /Unexpected payload length for command 50/);
  assert.equal(transport.received.filter(command => command === C.TARGET_POSITION).length, 1);
  await client.close();
}
console.log('PASSED: legacy/compact command 50, exact signed i64, schema mismatch without retries, framing/CRC, compact ACK, fragments, coalesced and extended replies, both addresses, serialized concurrent calls, fatal cancellation, timeout/late reply and write failure');
