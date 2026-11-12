# Release notes

## 0.11.0

- Add PID gain readback (command 48, firmware 0.16.0.0+).
- Add immediate position reference commands 49/50 (firmware 0.17.0.0), including raw integer, converted-unit and explicit unique-ID methods. Command 50 returns the latest stored measured Hall position.
- Reject nonfinite and out-of-range converted scalar inputs before transmitting; `getError()` returns -10. Raw position methods preserve signed 64-bit precision; converted methods use float precision.
- Validate the complete reply and CRC before reporting a remote motor fault. Corrupted or truncated error replies produce transport errors.
- Add offline transport coverage for framing, CRCs, addressing, malformed/truncated replies, timeouts and device fatal errors, plus a focused physical motor acceptance module.

Targets require enabled closed-loop position control, an empty queue and zero commanded/planned velocity. They are immediate reference steps without a trajectory speed/acceleration profile. Fatal rejections require motor reset. Read returned values only after checking `getError()`. Broadcasts are silent and therefore produce the documented timeout (-1).
