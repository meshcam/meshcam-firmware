# Surveyor: handheld RNS probe (LILYGO T-Beam V1.2, SX1276)

Walk into a meshcam mesh, power on, press the button: a dummy probe rides the real
stack (microReticulum `Resource` → gateway) and both ends log a GPS-stamped
proof-of-connection. Needs a gateway running **gateway-0.7.8+probe** or later for
the telemetry beat + `probe_ack`; older gateways still prove the link (the probe
completes), they just don't log it server-side.

```bash
pio run -e tbeam -t upload      # or flash the bins with esptool (see below)
pio device monitor -b 115200
```

## UX

| Input | Action |
|---|---|
| button short press | probe (~5 KB thumb-equivalent Resource) |
| button long press (≥1.2 s) | big probe (2× 16 KB, the full-res chunk path) |
| button double press | toggle auto mode (probe every ~20 s idle) |
| serial `p` / `b` / `a` | same three |
| serial `s` | status (seq, ok/try, profile, GPS fix/sats, `nmea=<ok>/<chars>`, clock, heap) |
| serial `d` / `wipe` | dump / clear `/probes.csv` |

CSV columns: `ts,seq,part,kind,lat,lon,alt,hdop,sats,profile,bytes,ok,ms,rssi,snr`.
`probe`/`big` rows log surveyor-side RF numbers (failures included; those exist
ONLY here, they never reach the gateway); `ack` rows carry the gateway-side numbers
downlinked as `probe_ack`. `seq` is NVS-persistent for CSV↔telemetry correlation.

## Notes

- **Mesh auto-selection: one image walks every mesh.** The gateway destination is
  discovered at runtime: candidates are the NVS-cached last winner, the
  `SURVEYOR_GATEWAY_CANDIDATES` build flag (comma-separated hashes: home bench
  heltec, parents' tower T-Internet-POE), and any `trailcam_gatea.resource`
  announce overheard on the air (a brand-new mesh needs zero config). Path
  requests go out round-robin; only the mesh you're standing in answers for its
  hash, and the winner persists to NVS. OLED/status show the active gateway's
  first 8 hex chars (`?` prefix until a mesh has answered this boot).
- **Announces as `trailcam.leaf`** so the gateway's ADR/time-sync/command machinery
  applies unchanged; a spot's rows record the best granted profile that closes
  there. Don't walk a mesh with a live leaf on the air (single-leaf gateway
  assumption; fine for the range walk).
- Clock: GPS sets the system clock on first fix; gateway `time_sync` is the
  fallback. Until either lands, CSV `ts` is seconds-since-boot.
- ADR revert: off-base and deaf for 90 s → back to base + `checkin` announce
  (the gateway's quiet-leaf scan converges from its end).
- TX is 20 dBm (SX1276 PA_BOOST ceiling) vs the leaf's 22 dBm Core1262;
  readings run ~2 dB conservative.
- Remote flash (board on another machine): `pio run -e tbeam`, ship
  `.pio/build/tbeam/{bootloader,partitions,firmware}.bin` + the framework's
  `boot_app0.bin`, then
  `esptool --chip esp32 --port /dev/ttyACM0 write_flash 0x1000 bootloader.bin
  0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin`
  (app-only iterations: just `0x10000 firmware.bin`).

## Bench E2E (2026-07-10, T-Beam ↔ bench Heltec gateway, all remote)

Probe #6 (5 KB): COMPLETE in 23.8 s at sf8/bw125, RSSI −73 / SNR 12.5.
Big probe #7 (2×16 KB): both parts COMPLETE (81 s + 60 s; the reused link
skips the second handshake), no OOM. ADR grant → SX127x live retune → confirm
announce verified; gateway `time_sync` accepted; GPS time-fix set the clock
indoors. A persistent serial bridge on the host (`/tmp/surveyor/serial_bridge.py`
pattern: hold the port open, log to a file, take input on localhost TCP) is the
way to interact: every plain reopen of the CH9102 tty hard-resets the board.

## Bring-up log (2026-07-10, remote via the laptop's USB)

AXP2101 detected → ALDO2/ALDO3 rails on; SX1276 init OK first try (the
speculative `BOARD_TBEAM` pin block was correct); hello announce TX (173 B,
521 ms air at sf8/bw125); GPS NMEA decoding (no fix indoors); gateway-less probe
walked the path-request ladder, timed out at 20 s, logged the `ok=0` CSV row;
30 s checkin heartbeats. Destination hash (NVS-stable):
`ae56ffb5a66c4d04f5c6c1612ec3bd73`. Free heap after boot: ~315 KB.
Gotcha found: XPowersLib constructors take `(Wire, sda, scl, addr)`; passing
the address third wedges I2C for everything after it.
