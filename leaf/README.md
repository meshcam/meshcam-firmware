# Leaf power skeleton — deep-sleep + PIR(ext0) + timer wake

The battery/solar leaf's power core. The ESP32-S3 spends ~all its life in deep sleep and
wakes on either the **AM312 PIR (GPIO21, ext0)** — an always-instant capture event — or an
**adaptive timer** (proactive check-in; season/time-of-day controls the interval, never a
capture). Deep sleep resets the MCU, so every wake is a fresh boot; state that must survive
lives in `RTC_DATA_ATTR`. Parent: [`docs/trailcam/design.md`](../../../docs/trailcam/design.md).

## Build / run
```
pio run -e freenove-s3       -t upload   # bench: 20 s test timer, watchable wakes
pio run -e freenove-s3-field -t upload   # real adaptive schedule (needs gateway time sync)
pio run -e pin-monitor       -t upload   # diagnostic: characterize GPIO21 (see below)
pio device monitor -b 460800
```
Wave at the PIR → `wake ... cause=PIR (ext0)`. Leave it idle → `cause=timer` every ~20 s
(bench env). No camera/radio/antenna needed.



## Custom leaf PCB envs (added 2026-07-08, boards in transit)

`leaf-pcb` / `leaf-pcb-field` carry every PCB delta and COMPILE-VERIFIED
(hardware validation = pilot bring-up, see docs/trailcam/leaf-pcb-bringup.md):
native-USB CDC console (no UART bridge on the board), RADIO_MISO=38,
switched rails (leaf_rails.h: IO39 camera, IO44 radio+SD — LOW=on, board
pull-ups hold OFF through deep sleep), microSD in SPI mode CS=43 on the
shared radio bus (leaf_store.cpp PCB branch), battery divider log (IO3),
BME280 on I2C 40/41 @0x76 (leaf_env.cpp, Adafruit lib, PCB envs only).
Bench envs unchanged (MISO=41 now explicit per-env). Rails/vbat/env calls
are flag-gated no-ops on the Freenove — its GPIO39/40 belong to the SD slot.

## Status
**FULLY VALIDATED on hardware 2026-07-02.** Deep-sleeps (`rst:0x5 DSLEEP`), and both wake
sources fire cleanly: **PIR ext0** (hand-waves → `cause=PIR`) and the **adaptive timer**
(idle → `cause=timer` every 20 s), with **zero spurious wakes** once the GPIO21↔20 bridge
was cleared (see Troubleshooting).

**2026-07-03 additions, both hardware-validated:**
- **Alert thumbnail over the MESH**: `leaf_radio_send_alert` pushes the JPEG to the
  gateway (gate-a server Heltec) as an `RNS::Resource` on an established Link — first
  image ever carried by the mesh (3809 B, hash-verified, ~89 s). Push happens FIRST on
  quiet air, the alert announce LAST (announce-first loses the handshake to the
  gateway's own rebroadcast — phase-locked, 3-for-3), plus one in-wake link retry.
- **Command-driven WiFi OTA** (`leaf_ota.{h,cpp}`): queue
  `{"kind":"update_firmware","payload":{ssid,psk,url,sha256}}` on the command bus and
  the next awake window joins WiFi, streams the image into the idle OTA slot, verifies
  sha256 BEFORE committing, reboots. Validated 0.3.0→0.4.0: 1.78 MB in ~8 s, ~15 s
  command-to-new-firmware, identity intact. The partition migration to dual OTA slots
  was the LAST required USB flash; USB is disaster-recovery only now.

**`on_pir_event` filled in + validated (2026-07-02):** a PIR wake now does a **real camera
capture** —
```
[leaf] wake #5  cause=PIR (ext0)  | pir=1 timer=2  | slept~20s
[leaf] radio -> (stub) would TX alert thumbnail, 3910 bytes
[leaf] PIR event -> captured 320x240 JPEG, 3910 bytes in 652ms, send=1
```
Lazy camera init (only on a PIR wake, never a timer check-in) → 2-frame AEC/AGC warmup →
QVGA JPEG (~3.9 KB at q14) → detector stub → radio handoff → `esp_camera_deinit()` back to
the sleep floor. Capture ~650 ms. Files: `leaf_camera.{h,cpp}` + `leaf_board.h` (camera pin
map, ESP32S3_EYE, same as gate-b).

**Radio layer filled in + validated (2026-07-02):** `leaf_radio.{h,cpp}` brings up the
proven gate-b SX1262 + microReticulum stack with a **persistent NVS-backed identity**, and
`radio_send_alert` / `on_scheduled_checkin` / `on_cold_boot` now do real work —
```
[radio] identity loaded from NVS (64 key bytes)
[radio] leaf destination trailcam.leaf  hash=9e0d2930af11d35e297b4a4d783c531c
[INF] LoRa init succeeded.
[radio] SX1262 init OK
[radio] announced (status="alert:3879")
```
Validated on hardware: **SX1262 inits every wake**, and the **destination hash is
byte-identical across deep-sleep wakes** (loaded from NVS, not regenerated) — so the gateway
sees one stable node, not a new one every wake. The announce is a real on-air TX carrying the
live thumbnail size. The RNS filesystem is `init(reformatOnFail=true)` — mounts existing,
formats only on the first-ever boot (no per-wake flash wear).

**Identity persistence:** the RNS private key lives in NVS (`leaf`/`rns_prv`, 64 bytes),
minted once on cold boot, reloaded every wake — survives deep sleep AND power loss (dead
battery / solar dropout), so the node's mesh address is permanent.

**Detector layer filled in + validated (2026-07-02):** `lib/leaf_detector/` runs a real
INT8 TFLite-micro model on the captured frame to reject false PIR triggers (wind, a passing
car's heat, sun on foliage) before spending radio energy. Pipeline: decode the JPEG →
center-crop + box-average to 96×96 grayscale → INT8 inference → dequantized person score →
gate. Validated on hardware:
```
[det] ready: in=96x96x1 int8, arena used=80/160 KB (internal SRAM)
[leaf] detector: person=0.28 (no subject) in 4141ms   -> send=0   (hand-wave / empty)
[leaf] detector: person=0.45 (SUBJECT)    in 4141ms   -> send=1   (person in frame)
```
The model **discriminates cleanly** (empty/motion ~0.25–0.31, a framed person ~0.43–0.46)
and the gate flips the send accordingly. Arena is 80 KB in internal SRAM.

- **Model:** the canonical `person_detection` (MobileNet-ish, 96×96×1 grayscale, INT8,
  ~300 KB), vendored in `lib/leaf_detector/person_detect_model_data.cpp` (Apache-2.0, TF
  Authors). Runtime is `eloquentarduino/tflm_esp32`. 5 ops: AveragePool2D, Conv2D,
  DepthwiseConv2D, Reshape, Softmax.
- **person-only ≠ animal gate:** a person model reads a deer as "no person", so it is NOT a
  safe sole gate for a wildlife cam. `LEAF_DETECT_GATE` (bench env only) makes a no-subject
  verdict actually skip the send so you can *see* the model work; the **field env leaves it
  off** — the detector logs its score but never suppresses, safe until a multi-class
  deer/person/empty model replaces person-only. Swapping models = replace the array + input
  dims + threshold; the plumbing stays.
- **~4.1 s inference** is compute-bound (reference CONV kernels, not ESP-NN-accelerated) and
  unchanged by arena location. Net-positive when it suppresses a multi-second high-power LoRa
  TX of a false trigger; flagged as a future optimization (ESP-NN dispatch / lighter model).
- **Threshold** `LEAF_DETECT_THRESHOLD` (default 0.40) is model-specific; retune on real
  footage.

**Serial ingest transport filled in + E2E-verified (2026-07-02):** `leaf_serial_proto.{h,cpp}`
emits the host-bridge framing on the normal USB console (spec:
[`../host-bridge/README.md`](../host-bridge/README.md), contract:
[`docs/trailcam/ingest-api.md`](../../../docs/trailcam/ingest-api.md)) — `!TC TLM` heartbeat on
**every** wake (beacon-at-least-daily by construction) and `!TC EVT` + base64 + `!TC END`
(crc32) per sent capture. Protocol lines coexist with normal log output; no console changes.
Verified end-to-end against **trailcam-dev** via the bridge (run on the dev host with the
board's serial piped over ssh: `ssh <laptop> 'cat /dev/ttyACM0' | uv run serial_bridge.py
--port -`):
```
15:05:21 INFO uploaded c3-1783018946-1 thumb (3500 bytes)   <- wave -> photo in the gallery
15:05:23 INFO telemetry c3-back-of-lake ok                  <- every wake
```
- **event_id is stable across retries by construction**: minted once per capture from RTC
  time + an RTC-memory counter (`tc_new_event`), persisted in RTC memory — a retry or the
  later full-res send reuses the same id (`tc_current_event_id`), matching the contract's
  (event_id, kind) idempotency.
- **captured_at is required** (the bridge rejects frames without it — learned the hard way),
  so the leaf **seeds its clock from the firmware build timestamp** on cold boot
  (`build_epoch()`, `LEAF_BUILD_UTC_OFFSET_S` corrects the build host's EDT to UTC). Timer
  wakes advance it by the slept interval. Bench-grade: a burst of PIR wakes freezes the
  clock (ids stay unique via the counter); real gateway time sync replaces this.
- Node slug / fw version: `LEAF_NODE_SLUG` (default `c3-back-of-lake`) / `LEAF_FW_VERSION`.

**Downlink (`!TC CMD`) + full-res store filled in + E2E-verified (2026-07-02):** the leaf now
handles gateway commands relayed by the bridge, and every sent capture also grabs the
sensor-max full-res (same camera session, `set_framesize` switch) stored under
`/tcfull/<event_id>.jpg` on the **microSD card** (SD_MMC 1-bit on the Freenove slot pins
39/38/40 — the reason radio MISO lives on 41; no count-eviction, GBs of depth). Cardless
fallback: the 12.9 MB LittleFS partition as a newest-`LEAF_STORE_MAX_FULL` ring (mtime
eviction; shares the partition with RNS, both mount idempotently). Each wake ends with a
`tc_poll_commands` window (`LEAF_CMD_WINDOW_MS`=3 s, extended per handled command so a
queued burst drains in one wake). On `fetch_full`: send the stored full-res as an EVT
`kind=full` under the same event_id — the ingest auto-completes the command server-side.
Verified against a real pending command in trailcam-dev:
```
16:32:07 INFO -> leaf: fetch_full c3-1783019566-33            <- bridge delivers
[tc] CMD received: kind=fetch_full event_id=c3-1783019566-33
16:32:14 INFO uploaded c3-1783019566-33 full (25760 bytes)    <- command completed
[store] saved /tcfull/c3-1783023907-1.jpg (26233 bytes)       <- new captures store full-res
```
- **Delivery timing:** the bridge writes CMDs on its 30 s poll blind to our sleep, so a
  command lands when a write collides with an awake window — the server re-delivers until
  the full ingests, so misses self-heal (observed: one command completed per collision
  cycle, ~30-90 s apart). The real gateway will instead hold commands for the leaf's
  check-in, eliminating the lottery.
- **fetch_full serves the STORED ORIGINAL or nothing — never a re-capture.** The first
  implementation had a bench "live fallback" that answered an unstored event_id with a
  fresh capture under the requested id; verified bug 2026-07-02 (dev photo 93487506): the
  "full-res" showed a different moment than its thumbnail, corrupting the photo record.
  Removed entirely — a missing event is ignored and the command expires server-side (14 d).

### Quality-tiered fetch_full (feature request 2026-07-02)
The store now holds the **sensor-max original** (QXGA 2048×1536, `LEAF_FULL_FRAMESIZE`,
~200-400 KB at q12) and `fetch_full`'s `payload.quality` picks the tier — both are
transforms of the SAME stored frame, never a re-capture:
- `"max"` (or absent payload, the legacy form) → the stored original, byte-for-byte.
- `"standard"` and any unknown value → downscaled on-device to ~800×600
  (`leaf_jpeg_downscale`: decode at 1/2 scale to RGB565 — a full-scale RGB888 decode of
  QXGA is 9.4 MB and doesn't fit PSRAM — box-average to `LEAF_STD_W×H`, re-encode q80).
  If the transform fails (odd/legacy original) it degrades gracefully to sending as-is.

Two infrastructure changes ride along:
- **Partition table** `partitions_leaf.csv`: huge_app.csv only mapped 4 MB of the 16 MB
  flash; the new table keeps the head layout (NVS offset unchanged → the RNS identity
  survives) and gives ~12.9 MB to the LittleFS store — ~40 QXGA originals. The resized
  FS auto-formats once on first boot.
- **Console baud 115200 → 460800** (`LEAF_BAUD`): the bridge drops a frame that doesn't
  END within 30 s of its EVT; a QXGA frame is ~410 KB of base64 = ~36 s at 115200 but
  ~9 s at 460800. Match it everywhere (`monitor_speed`, the bridge's `--baud`).

**Tiers E2E-validated 2026-07-02** (photos verified visually in the dev gallery):
```
max:      sending stored ORIGINAL (117264 bytes) -> uploaded 117264   (byte-exact from SD)
standard: sending standard downscale (117982 -> 26905 bytes)          (4.4x airtime saving)
```
Three bugs found on hardware en route (details in commit 0e4fe28): `fmt2jpg` silently
truncates at a fixed 128 KB buffer (q80 blew it -> corrupt JPEG; now q60 + guards);
`jpg2rgb565` is low-byte-first and `fmt2jpg`'s RGB888 is really BGR (psychedelic colors —
which was also the size bug: scrambled bit-fields don't compress); and the bridge's SSE
delivery is one-shot per connect, unreachable for a leaf that sleeps 20 s of every ~24 —
the bridge now polls + delivers the instant it sees a `!TC TLM` (= the leaf is awake and
its command window opens next), draining a 4-command backlog in one 11 s wake window.

**Flash recipe (identity-preserving):** iterate with `esptool write_flash 0x10000
firmware.bin` ONLY. The full merged image 0xFF-pads the 0x9000-0xDFFF gap and wipes NVS —
the RNS identity rotated on every reflash until this was caught (4 different destination
hashes in one day). Full merged flash only for first-time bring-up / partition changes.
- **Thumb and full are captured back-to-back** (~1.5 s apart, one framesize switch) into
  RAM copies BEFORE the slow stages (detector ~4 s, serial emit ~3 s), for the same reason:
  grabbing the full after detector+emit put it ~8 s behind the thumb — meters of movement
  for a walking animal. Bonus: the sensor powers down ~7 s earlier per event. True
  same-frame thumb (decode the full, downscale + re-encode) is a possible follow-up.
- Unknown CMD kinds are logged + ignored (forward compat). CMD JSON is parsed with a tiny
  flat-string scanner — no ArduinoJson dependency outside radio builds.
- Serial RX buffer is raised to 2 KB (a CMD burst overflows the 256 B default while a slow
  handler runs).
- Bench transport note: the bridge now runs **on the laptop that owns the serial port**
  (`/tmp/bridge-venv/bin/python /tmp/serial_bridge.py --port /dev/ttyACM0 --baud 460800
  --env-file /tmp/trailcam-ingest-dev.env`), exactly like the future gateway Pi. The
  earlier ssh-cat pipe (uplink-only) and the socat TCP/PTY tunnel are both retired — the
  tunnel silently split/dropped bytes whenever two readers or stale respawn loops raced
  (shredded frames, hours of debugging; see git history 2026-07-02).

**Still stubbed / next:** the full image transfer over an established gateway **Link** as an
`RNS::Resource` (today the alert path just announces "alert pending" with the size — needs a
peer + the 915 antenna, Friday); RNS-side command delivery (the mesh equivalent of !TC CMD,
held for check-ins); a **multi-class detector** trained on deer/person/empty to replace the
person-only stand-in; battery ADC + BME280 reads to fill the telemetry/meta fields.

### Builds
`freenove-s3` / `freenove-s3-field` include radio (`-DLEAF_RADIO`) + detector
(`-DLEAF_DETECTOR`); the bench `freenove-s3` also sets `-DLEAF_DETECT_GATE`. `pin-monitor` is
lean (`lib_ignore = lora_interface, leaf_detector`). First full build pulls
microReticulum/RadioLib/Crypto + compiles the TFLite-micro runtime (~5 min); later builds are
cached (~25 s).

### Alert-image sizing note
Over LoRa (~a few kbps effective) the alert IS a small JPEG, so we capture directly at a
thumbnail framesize (`LEAF_ALERT_FRAMESIZE`, default QVGA) rather than grab a full frame and
downscale in software. Even 3.9 KB is minutes over SF8/125 kHz, so the field size may drop to
QQVGA — measured against real link throughput once the radio's up. A rare "send hi-res of
image N" gateway command would take a separate full-res path (backlog).

## Troubleshooting — spurious continuous PIR wakes (solved 2026-07-02)
Symptom: with nobody present, the leaf woke on `ext0` (PIR) continuously and never reached
the timer. Root cause: a **solder bridge between GPIO21 and GPIO20** on the perfboard.

The trap that cost an evening: **GPIO20 is the ESP32-S3 USB D+ line and carries an internal
~1.1 kΩ pull-up.** A 21↔20 bridge holds GPIO21 high through *that* pull-up — strong enough
to look like a hard short, but it's **inside the chip, not on the 3.3 V rail**, so a
continuity test from `21 ↔ 3.3V` reads **open** and looks innocent. The tell was continuity
`21 ↔ 20` (beeps). **Lesson: when a pin is stuck high with no short to 3V3, meter it against
its *neighbor* pins, not just the power rail** — especially GPIO19/20 (USB D-/D+), which
self-pull.

Diagnostic aid: the `pin-monitor` env reads a pin plain/pulldown/pullup to call
driven-vs-floating. **Caveat: its "floating" detection is unreliable on this board** — known
open pins (39/40) also read `pulldown=1`, so trust it only for *driven-low* (a real PIR
idle) and cross-check "high" verdicts with an unpowered continuity meter.

## Troubleshooting — "walked by it, no photos" (solved 2026-07-03, leaf-0.8.0)
Symptom: 24 min of walk-bys, zero photos, while the gateway heard perfectly healthy
check-ins every ~27 s. Three stacked faults, none visible from the mesh side:

1. **`LEAF_DETECT_GATE` was still on** (device ran the bench env). A real walk-by
   captured fine and then logged `send=0` — the person-only INT8 model said "no
   subject" and the alert was silently dropped. Fixed by removing the gate from the
   bench env too: the score is metadata until a model is proven against real captures.
   The gate's own comment predicted exactly this; believe it.
2. **OTA slot roulette**: the bootloader alternated slots across ordinary deep-sleep
   wakes — `leaf-0.5.0` for wakes #708–711, `0.7.0` from #712, with the RTC counters
   continuous (so no reset in between). otadata was in an unstable state after the
   5-OTA chain. Two fixes: `esp_ota_mark_app_valid_cancel_rollback()` on every wake
   (rewrites otadata to a clean record), and the recovery-flash recipe below writes
   BOTH slots so whatever the bootloader picks is current.
3. **PIR-capture crashes on dock USB power**: two PIR wakes died mid-capture with RTC
   RAM wiped (counter #712→#7, #10→#2 — full power-domain reset, i.e. brownout-class,
   during camera bring-up). The wall-wart period accumulated 711 wakes with zero
   resets, so it's the dock port's current limit, not firmware. The wake banner now
   prints `reset=` (PANIC/BROWNOUT/…) so the next one is attributable from the log
   alone. Lesson: **capture-path testing needs a real power supply, not a hub port.**

Recovery flash (USB, keeps the NVS identity at 0x9000):
```
esptool erase_region 0xe000 0x2000                      # otadata -> factory default
esptool write_flash 0x10000 fw.bin 0x310000 fw.bin      # BOTH slots = same image
```
Kill anything holding the serial port first (sercap/bridge), or esptool fails with
"No more data to read from the serial port".

### Maintenance mode (leaf-0.8.0)
`{"kind":"maintenance","payload":{"ssid":"…","psk":"…","minutes":15}}` — serial
`!TC CMD` line or mesh command packet. The leaf finishes its wake work, then stays
awake with BOTH command channels open (serial poll + 30 s mesh announces with the
RX window) instead of deep-sleeping, so an operator can service it without racing
the 3 s wake window. With WiFi creds it also joins and NTP-syncs `g_unixtime`
(real timestamps until the next full power loss). `{"kind":"sleep"}` exits early;
the deadline (1–120 min) exits otherwise. Queue remotely via
`POST /api/v1/nodes/{id}/commands` on the trailcam app.

### Adaptive radio profile / ADR (leaf-0.10.1 + gateway-0.4.1)
Both radios must run the identical SF/BW/CR or they're mutually deaf, so profiles are
a shared indexed ladder in `lib/lora_interface/lora_profiles.h` (vendored byte-identical
in BOTH firmware trees; append-only): 0 = sf10/bw125 (~122 B/s raw, far), 1 = sf8/bw125
(~488 B/s, **base/rendezvous**), 2 = sf7/bw250 (~1367 B/s, near). The gateway decides:
it averages announce SNR (5-sample ring) and moves the leaf one step at a time —
upgrade at ≥ 12 dB headroom over the faster profile's demod floor, downgrade at ≤ 6 dB
over the current one — via `{"kind":"radio_profile","payload":{"idx":N,"ttl_s":S}}`
into the post-announce RX window, then retunes itself immediately.

The safety story is that every failure mode independently converges both ends to base:
- Leaf applies a grant live, confirms with a `"profile"` announce ON the new profile,
  and persists the index in RTC memory (`g_rf_profile`) so later wakes — PIR thumb
  pushes included — come up already tuned.
- No confirm within 20 s → the gateway reverts to the previous profile (grant packet
  lost; the leaf never moved).
- Leaf reverts to base after 3 consecutive contactless wakes (`leaf_radio_had_contact`:
  no link, no packet) or on grant TTL expiry (24 h). Because an IDLE leaf normally gets
  zero downlink traffic, the gateway feeds that counter with a grant keepalive — it
  re-sends the standing grant every 2nd announce it hears while the leaf is off-base
  (self-adapting to bench 28 s or field 30 min cadence alike; leaf-0.10.1 re-arms
  TTL + counter on a same-idx refresh without a confirm announce). Observed live
  before the keepalive existed (2026-07-03): grant → confirm → 3 idle wakes → leaf
  reverted → gateway scanned → adopted base → re-granted, on a ~3 min loop — i.e.
  every fallback path works, they just churned on an idle bench.
- Power loss wipes RTC → leaf cold-boots on base and says `hello` → re-granted.
- Gateway persists its grant in NVS (`gw-rf/idx`); if the leaf goes quiet > 90 s while
  off-base it alternates camping between granted and base every 30 s until the leaf
  reappears, then adopts whatever profile it found it on.

Bench note: gateway TX power is still 10 dBm and the two radios are meters apart, so
SNR ~13 dB at base grants sf7/bw250 (~2.8× raw) almost immediately. Watch it in the
Mesh feed (`radio profile → … granted/confirmed/reverted` rows) and on the gateway
status page (`radio sf7/bw250 (confirming)`).

**Measured result (2026-07-03, same 35,353-byte full each run):** sf8/bw125 =
229 s (154 B/s); sf7/bw250 = 211 s (167 B/s). The PHY got ~2.8× faster but goodput
moved only ~9% — chunk transfers were **turnaround-bound in the RNS resource layer**,
not airtime-bound. ADR still matters for FIELD range (auto-downshifting to sf10 when
SNR is thin is its real job), but throughput needed the phase-2 fixes below.

**Phase 2 turnaround fixes (gateway-0.5.0, 2026-07-03): 211 s → 92.9 s (381 B/s,
2.5× the original baseline).** Frame-timeline instrumentation (`[rf] rx/tx` lines,
millis-stamped per frame) attributed the dead air; three gateway fixes landed it:
RX re-arm immediately after `readData` (inbound processing left the radio deaf up to
~2.5 s per window-final frame while the sender streamed into the silence), blocking
TLS work (beats/polls/uploads) deferred while frames are arriving (`radio_busy()`,
any frame in the last 2 s), and loop sleep 10 ms → 1 ms while busy. What remains per
the timeline: ~117 ms/frame of RNS packet processing and a fixed ~2.5-3 s CPU spike
at each window boundary inside microReticulum — doubling the resource window ladder
(gateway-0.5.1, Type.h patch via patch_microreticulum.py: WINDOW 4→8, MAX_SLOW
10→20, MAX_VERY_SLOW 4→8) measured NEUTRAL (99.3 s), so the boundary cost isn't
strictly per-window; next lever would be profiling that spike or leaf-side pacing
(the leaf still runs the pre-fix lib — its RX is only small spaced-out requests).

**Thumb vs full show different moments (stopwatch-verified 0.50 s apart).** The PIR
pipeline takes TWO exposures back-to-back: the alert thumbnail first, then a sensor
framesize switch, then the full-res grab — both filed under one event_id/captured_at.
Photo 8116c56e (stopwatch in frame): thumb reads 00:22.68, full reads 00:23.18.
Inherent to the two-exposure design; the single-moment alternative is to capture ONLY
the full-res frame and software-downscale it for the alert thumb (the downscale path
already exists for the standard tier) at the cost of a few seconds of leaf CPU per
alert before the thumb can send.

### LittleFS spool: quality=max unblocked gateway-side (gw-0.6.2, 2026-07-04)
The gateway now spools full-res chunks to LittleFS and streams the upload from flash
(no whole-file heap use), removing the ~48 KB RAM ceiling; the leaf's clamp is raised
to 512 KB (leaf-0.11.x). Two hardware lessons from the bring-up: (1) holding one
FILE* open across the minutes between chunks corrupted spools (random write failures,
silently-short files — fd clobbered under store churn); each chunk now opens r+ /
seeks / writes / closes / stat-verifies, and completion verifies size == total before
the rename. Verified end-to-end (35 KB: 3 chunks verified, upload HTTP 200).
(2) **leaf-0.11.x would not stick via mesh OTA**: three attempts each downloaded,
flashed, and rebooted — and every boot came back on the OLD slot still running
0.10.1 (rollback/otadata roulette, the 2026-07-03 slot-instability again; the NVS
sha guard then skips the re-delivery, which is why attempt #2 looked like a no-op).
True max over the mesh therefore needed a DOCK flash of the leaf (erase otadata +
write BOTH slots with 0.11.1) — done 2026-07-04, version confirmed on the leaf's
own console. **End-to-end result: the 160,399-byte QXGA original (2048×1536) of the
stopwatch event transferred as 10 chunks in 12 min at base profile, every chunk
spool-verified, streamed from flash, upload HTTP 200.** quality=max now means max.

### Mesh full-res ceiling (leaf-0.8.1)
quality=max (~148 KB QXGA original) over the mesh OOM'd the gateway: it reassembles
chunks into plain RAM (Heltec, no PSRAM; ~56 KB free after an hour up) — every chunk
was received then "OOM for assembly -> dropped", the command never auto-completed, and
the 18-minute transfer re-delivered in a loop. The leaf now clamps mesh-served fulls:
anything over `LEAF_MESH_FULL_MAX` (default 48 KB) is served as the standard tier
instead. True max-quality over mesh needs the gateway to spool chunks to LittleFS and
stream the upload from flash (backlog). Note the gateway boots with ~220 KB free heap
and settles to ~56 KB after ~1 h — worth watching for a slow leak.
