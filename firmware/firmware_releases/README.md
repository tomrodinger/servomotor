# Firmware releases

Each firmware image has adjacent `.release_notes.txt`, `.firmware.sha256` and
`.release_manifest.json` files. Notes describe changes, compatibility,
upgrade/rollback and validation. The manifest records hashes and evidence.

| Version | Target | Release notes |
|---|---|---|
| [0.17.0.0](servomotor_M17_fw0.17.0.0_scc3_hw1.5.firmware) | M17 hardware 1.5, compatibility code 3 | [Immediate position targets](servomotor_M17_fw0.17.0.0_scc3_hw1.5.release_notes.txt) |
| [0.16.0.0](servomotor_M17_fw0.16.0.0_scc3_hw1.5.firmware) | M17 hardware 1.5, compatibility code 3 | [PID constants readback](servomotor_M17_fw0.16.0.0_scc3_hw1.5.release_notes.txt) |

Versions are `major.minor.bugfix.development`. The development field is zero
for a stable release. Version 0.17.0.0 adds commands to stable 0.16.0.0 and
promotes the tested 0.17.0.6 implementation. Historical images without sidecars
predate this release-bundle convention.

Use the product, hardware and compatibility code in the filename to select an
image. Firmware release publication does not publish Python or Arduino packages;
use updated bindings containing the documented commands.
