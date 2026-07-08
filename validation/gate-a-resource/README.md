# Gate A — microReticulum `Resource` transfer over LoRa (bench test)

**Status:** **GATE A CONCLUDED (2026-07-03): Resource-on-MCU WORKS, ceiling measured.**
5/15/25 KB all reassembled byte-complete over real 915 MHz air (server logged
`RESOURCE COMPLETE` + hash for each); **35 KB+ is not possible** on a no-PSRAM
Heltec V3 — `std::bad_alloc` in the link encrypt even from a clean ~331 KB heap
(all-or-nothing in RAM; upstream multi-segment send is an empty TODO stub). Outcome =
the anticipated middle row of the decision matrix below: **thumbnail-on-mesh holds
with huge margin; full-res rides app-layer chunking at ≤ ~20 KB per chunk.**
Effective goodput at SF8/125 kHz desk range, 10 dBm: **~160-180 B/s** (15 KB in 86 s,
25 KB in 158 s). Four fatal runtime bugs found + fixed on the first-ever hardware run —
see "Hardware-run findings" below. Parent design:
[`docs/trailcam/design.md`](../../../docs/trailcam/design.md) → "Gates & validation".

> Verified builds (microReticulum pinned @ `c02b6e3`): `heltec-v3-server` and
> `heltec-v3-client` both link, ~40% of a 2 MB app partition, 7.5% RAM. The
> `LoRaInterface` is vendored + committed in `lib/lora_interface/`, so a clean checkout
> builds with no extra steps.

## What this proves (and why it goes first)

The whole trail-cam payload path rests on one unverified assumption: that
**microReticulum's `Resource` primitive actually segments, retries, and reassembles
a multi-KB blob on an MCU.** `Resource` exists in the source
(`src/microReticulum/Resource.{h,cpp}`) and the v0.5.0 roadmap marks it landed — but
**no upstream example exercises it** (the examples stop at `Link` + `Packet`, ≤ Link
MDU). So we prove it ourselves before any money goes into cameras/solar/enclosures.

Gate A = push **5 KB (thumbnail) → 50 KB → 250 KB (full-res JPEG scale)** as a
`Resource` over a real 915 MHz LoRa link between two boards, and confirm byte-exact
reassembly + retry-under-loss.

## The rig: two Heltec V3s, no Pi needed

Both boards run the **same firmware** in this project, one as server (receiver), one
as client (sender). These are the **two Heltec V3s from the phase-1 range kit** — Gate
A costs **zero extra hardware**. A laptop is only a serial monitor.

```
[Heltec V3 #1: CLIENT/leaf] --915MHz LoRa Resource--> [Heltec V3 #2: SERVER/gateway]
   pushes 5/50/250KB blobs                               logs COMPLETE + bytes + hash
```

### ⚠️ Why NOT "ESP32 ↔ Pi running Python RNS" (a correction)

An earlier draft framed Gate A as device ↔ a Pi running full Python RNS via an RNode.
**That's the wrong rig for this gate.** microReticulum's `LoRaInterface` uses its
**own split-packet L2 framing** (254-byte frames, custom 1-byte header — see
`examples/common/lora_interface/LoRaInterface.h`), which is **not** the air format
RNode firmware speaks. Pointing a microReticulum-LoRaInterface node at a Python-RNS
RNode would test *L2 interoperability* and *Resource-on-MCU* at the same time — if it
failed you wouldn't know which broke.

So: **Gate A = two microReticulum nodes, same L2** (isolates "does Resource work").
The microReticulum ↔ Python-RNS/RNode interop question is **real and separate** — call
it **Gate A2**, and it decides the *production gateway* design (does the leaf's L2
reach a Pi gateway directly, or does the gateway MCU bridge LoRaInterface→serial→Pi?).
Don't assume it; verify it on its own. Tracked as an open item in the design doc.

## Build setup

1. Install [PlatformIO](https://platformio.org/) (`pip install platformio`, or the
   VS Code ext).
2. `pio run -e heltec-v3-server` (and `-e heltec-v3-client`). First build pulls the
   esp32 platform/toolchain + microReticulum (pinned) + RadioLib + microStore +
   attermann/Crypto, ~3 min cold. **No manual vendoring** — `lib/lora_interface/` is
   committed. The radio defaults there are **915 MHz / SF8 / BW125 / CR5 / 17 dBm**
   (US ISM); both boards must match (they do, same firmware).

## Run procedure

```bash
# 1. flash + monitor the SERVER (receiver)
pio run -e heltec-v3-server -t upload --upload-port /dev/ttyACM0
pio device monitor -p /dev/ttyACM0 -b 115200
#    -> note the line: "SERVER destination hash: <32 hex chars>"

# 2. paste that hash into src/gate_a_config.h  ->  GATE_A_SERVER_DEST_HEX
# 3. flash the CLIENT (sender)
pio run -e heltec-v3-client -t upload --upload-port /dev/ttyACM1
pio device monitor -p /dev/ttyACM1 -b 115200
```

Antennas on **both** boards before powering — a bare LoRa PA into no load can fry.

## Pass / fail

**PASS** = the server logs `RESOURCE COMPLETE: 256000 bytes ...` for the 250 KB blob
(and the 5 KB + 50 KB before it), i.e. all three sizes reassemble byte-exact. Note the
airtime per size from the timestamps → feeds the battery/airtime budget in the design
doc's durability section.

**Retry sub-test (do this once it passes clean):** during the 250 KB transfer, briefly
pull the server's antenna (or walk it out of range and back). PASS = the transfer
recovers and still concludes COMPLETE, proving `Resource`'s segment-retry works. This
is the property that lets store-and-forward tolerate a flaky woods link.

## Outcomes → what each means for the build

- ✅ **All sizes + retry pass** → single-MCU leaf with on-mesh full-res is viable;
  proceed to Gate B (camera+RNS+TFLite RAM/pin coexistence) and Gate A2 (gateway
  interop).
- ⚠️ **Small (≤5–10 KB) works, large flakes** → **thumbnail-on-mesh holds**, full-res
  moves to on-visit BLE/WiFi pull (already the plan for the deepest nodes). Leaf
  design survives; note the practical max on-mesh blob size.
- ❌ **`Resource` unusable on-device** → the architecture bends: 2-MCU leaf (cam board
  + separate `microReticulum_Firmware` RNode over serial), or reconsider Meshtastic's
  bolted-on image transfer for the payload path. This is the finding that most changes
  the build — which is exactly why Gate A runs first.

## Files

| File | What |
|---|---|
| `platformio.ini` | two envs (`heltec-v3-server` / `heltec-v3-client`), lifted from upstream `lora_transport` |
| `src/main.cpp` | the firmware; role split by `-DGATE_A_SERVER` / `-DGATE_A_CLIENT` |
| `src/gate_a_config.h` | blob sizes, server dest hash, radio tick |
| `lib/lora_interface/` | **vendored** from microReticulum examples (step 2 above; not committed until pinned) |

## Closed at compile time

- [x] `RNS::Bytes` single-byte append → `append(uint8_t)` (Bytes.h:367). Compiles.
- [x] `no_ota.csv` partition fits — the app is ~40% of 2 MB. Fine.
- [x] All RNS API calls (Identity / Destination / Link / Resource / Transport / the
      `ACCEPT_ALL` strategy + started/concluded callbacks) compile against
      microReticulum `c02b6e3`.

## Open — runtime only (needs the two boards)

- [x] `ACCEPT_ALL` auto-delivery **CONFIRMED** on hardware: once the server-side link
      actually activates (see finding 2 below — that's the hard part), the advertise is
      accepted with no explicit `Resource::accept()` and `set_resource_started_callback`
      fires. No code change needed.
- [x] `advertise=true` self-drives the transfer **CONFIRMED** — sender advertise →
      receiver part-requests → parts → proof, all from the `reticulum.loop()` pump.
      (After fixing the deadlock that pump triggers — finding 3.)

## Hardware-run findings (2026-07-02, first run ever on boards)

Three bugs, each independently fatal to the transfer. The debugging ladder that found
them: `RNS_LOG_LEVEL=7` build flag (DEBUG logs are silently compiled OUT at the
default VERBOSE cap — `RNS::loglevel(LOG_DEBUG)` alone does nothing).

1. **Client TX-flooded itself deaf.** `role_loop` ran every 10 ms and called
   `Transport::request_path()` unconditionally while pathless → the half-duplex radio
   was permanently in TX and never heard the server's announce; the server heard a
   packet every ~1.3 s at −28 dBm forever. Fix: rate-limit path requests to
   `GATE_A_TICK_SECS`.
2. **Half-open link: the server's announce ticker shot down the RTT packet.** The
   initiator's one-shot LRRTT packet after proof validation is the ONLY thing that
   activates the responder-side link and fires the destination's
   `link_established` callback (`Link.cpp rtt_packet()` — and a lost one is never
   retried). The server's blind 5 s announce TX overlapped it (the handshake
   phase-locks the timing, so it recurred every run). Client then "established",
   advertise received by the server, silently dropped at `ACCEPT_NONE` (the callback
   that would set `ACCEPT_ALL` never ran), advertise retries forever. Fixes: server
   announces every `GATE_A_ANNOUNCE_SECS` (30 s) and stops once linked (Transport
   auto-answers path requests with an announce anyway); client tears down + fully
   re-handshakes when a send concludes FAILED instead of retrying into a dead link.
3. **Upstream single-thread deadlock on ANY Resource retry** (microReticulum
   @ `c02b6e3`): `Transport::jobs()` sets `_jobs_running=true` and pumps
   `link.tick_resources()`; a watchdog retry (advertise re-send, part-request re-send)
   re-enters `Transport::outbound()`, whose literal port of Python's thread-lock wait
   — `while (_jobs_running) OS::sleep(0.0005);` — spins forever on a single thread.
   Both boards wedged solid (no serial, no RX, no WDT reset) at the first retry, every
   run. Fixed by `patch_microreticulum.py` (idempotent `extra_scripts` pre-build patch
   of libdeps; same latent bug guarded in `Transport::inbound`). Worth upstreaming.
4. **Concluded Resources leaked ~5× their blob size** (our bug, and the reason 50 KB
   looked even less possible than it is): `role_loop` did `new RNS::Resource(...)` and
   dropped the pointer; the wrapper's shared_ptr kept blob + encrypted token + parts
   alive forever. Measured via the free-heap send log: 310 KB before the 15 KB send →
   230 KB before the 25 KB send. Fixed with a deferred `delete` on the loop tick after
   conclusion (deleting inside the concluded callback would be use-after-free). With
   the leak fixed, pre-send heap is flat (~331-339 KB every stage).

## Measured single-Resource ceiling (no-PSRAM Heltec V3)

| size | result | notes |
|---|---|---|
| 5120 B | ✅ COMPLETE | repeatedly, across many runs/reboots |
| 15360 B | ✅ COMPLETE | 86 s (~179 B/s) |
| 25600 B | ✅ COMPLETE | 158 s (~162 B/s) |
| 35840 B | ❌ `std::bad_alloc` | in link encrypt, clean 331 KB heap — fragmentation + several full-size copies + contiguous-block needs |
| 51200 B | ❌ never reachable | |

Design consequences: alert thumbnails (~4-8 KB) have huge margin. The leaf's
"standard" 800×600 tier (~26 KB) sits AT the edge — send it chunked too. Full-res
QXGA (~110-140 KB) = app-layer chunking at ≤ ~20 KB per chunk (~6-8 sequential
Resources, reassembled at the gateway; at ~170 B/s that's ~12-15 min of airtime, an
on-demand operation, not a per-event one). The receiving gateway Heltec has the same
no-PSRAM constraint, so the ceiling binds both directions.

Operational gotchas for reruns:

- **The server identity is random per boot** (`RNS::Identity()` + `filesystem.format()`
  at startup) — every server reflash/reset mints a new destination hash; re-capture it
  from the boot banner and rebake `GATE_A_SERVER_DEST_HEX` before reflashing the client.
- The esp_littlefs `Failed to unlink ... Has open FD` spam from the microStore
  known/path/hashlist stores is noisy but has not (yet) been load-bearing.
- Effective goodput at SF8/125 kHz desk range measured ~115 B/s for the 5 KB stage
  (44 s) — protocol turnarounds dominate at small sizes.
