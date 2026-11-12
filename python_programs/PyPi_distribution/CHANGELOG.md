# Release notes

## 0.13.0

- Add PID gain readback (command 48, firmware 0.16.0.0+).
- Add immediate position commands 49/50 for firmware 0.17.0.0. The combined method returns the latest stored measured Hall position, without waiting for arrival.
- Preserve full signed 64-bit precision in encoder-count inputs and outputs.
- Correct velocity/acceleration fixed-point scaling when internal units are selected.
- Reject unknown, duplicate or missing method arguments, nonfinite values and out-of-range integer fields before transmission.
- Reject malformed fatal replies with extra payload instead of reporting an unverified motor fault.
- Add offline API, protocol, package and C++ transport regression coverage.

Immediate references require enabled closed-loop position control, an empty queue and zero commanded/planned velocity. They do not create a velocity/acceleration profile. Device fatal errors require a reset. Broadcasts return an empty list.
