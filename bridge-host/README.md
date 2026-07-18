# host-bridge (bench uploader: leaf serial → trailcam)

Until the mesh has a relay + gateway, the workstation plays the gateway:
this script reads framed captures off the prototype leaf's **USB serial**
console and uploads them to **trailcam.example.com** with the standard
spool-until-2xx rule (see the [ingest API spec](https://github.com/meshcam/meshcam-api)).
It doubles as the reference implementation for the real gateway daemon.

```sh
uv run serial_bridge.py --port /dev/ttyACM0          # live bench
uv run serial_bridge.py --dir /media/sd --camera c3-back-of-lake   # SD pull
```

Credentials: `~/.config/meshcam/ingest.env` (create it: INGEST_URL + DEVICE_TOKEN).
Spool: `~/.cache/trailcam-bridge/spool`; images wait there across restarts
and network blips until the app acks.

## Serial framing spec (firmware side: emit exactly this)

Line-oriented UTF-8 on the normal console UART: `!TC `-prefixed lines are
protocol, everything else stays ordinary log output (the bridge passes it
through to stderr, so `idf.py monitor`-style logging keeps working).

**Telemetry heartbeat** (send on any wake; one line, fire-and-forget):

```
!TC TLM {"node":"c3-north-stand","kind":"camera","battery_v":3.29,"temp_c":24.5,"pressure_hpa":991.9,"rssi":-101,"snr":6.4,"boot_reason":"DSLEEP","fw_version":"leaf-0.3.1"}
```

**Capture event** (thumb right after PIR capture; full-res later works too,
same `event_id`, `"kind":"full"`):

```
!TC EVT {"camera":"c3-north-stand","event_id":"c3-<unixts>-<n>","captured_at":"2026-07-02T14:11:02Z","kind":"thumb","len":5123,"meta":{"battery_v":3.29,"rssi":-101}}
<base64 of the JPEG — any line width, multiple lines fine>
!TC END c3-<unixts>-<n> 1a2b3c4d
```

- `len` = decoded byte count (optional but recommended).
- `END` trailer = event_id + **crc32 of the decoded JPEG bytes**, lower-hex
  (`printf("%08x", crc32)`; optional: omit the second token to skip).
  On len/crc mismatch the bridge drops the frame and logs; the leaf's normal
  store-and-forward retry re-sends it.
- One frame at a time; a new `EVT` before `END` drops the unfinished one;
  30 s without `END` times the frame out.

The bridge maps frames straight onto the ingest contract: `event_id` gives
idempotency (safe to re-emit after resets), thumb/full merge onto one photo,
`meta.battery_v` feeds the Nodes health page.

## Downlink: commands to the leaf (added with trailcam v0.3.0)

In serial mode the bridge holds an SSE connection to
`GET /api/v1/commands/stream` (same token); commands reach the leaf the
instant they're queued (the "Gateway picked it up" step in the app's
diagnostics panel is now effectively immediate). The server re-emits all
outstanding commands on every (re)connect, and the bridge suppresses
duplicate serial sends of the same command for 5 minutes. `GET
/api/v1/commands` (plain poll) remains available as a fallback transport.
Each command goes to the leaf as a single line:

```
!TC CMD {"id":7,"kind":"fetch_full","site":"north40","node":"c3-north-stand","event_id":"c3-1783018946-5","payload":null,"status":"delivered","created_at":"..."}
```

**Receipt ack (leaf-0.16.0):** on reading a `CMD` line that carries an `id`,
the firmware immediately emits one line — before executing the command, since
receipt is not completion and a `fetch_full` handler can run for minutes:

```
!TC ACK 7
```

The bridge relays it to `POST /api/v1/commands/{id}/ack {"status":"received"}`,
which stops server-side redelivery (the mesh path does the same via the leaf's
`a=<id>` announce tail, relayed by the gateway). Lost acks self-heal: the
server keeps re-delivering, the leaf re-acks on its next window, and the
endpoint never regresses a command that already finished.

**Firmware handling:** on `kind:"fetch_full"`, re-send the stored full-res JPEG
for that `event_id` as a normal `EVT` frame with `"kind":"full"`. ⚠️ **It must be
the STORED ORIGINAL frame saved to SD at capture time; do NOT re-capture.**
Verified bug 2026-07-02: the first downlink implementation captured a fresh
800×600 at request time, so the "full-res" was a different moment than the
thumbnail (trailcam-dev photo 93487506 shows the mismatch). Save the full
frame alongside the thumbnail during on_pir_event and serve that file.

⚠️ **Resolution: fulls currently arrive at SVGA 800×600.** The app stores and
serves whatever bytes the leaf sends, no server-side limit. The OV3660-class
sensor does up to QXGA 2048×1536, so "full-res" quality is a leaf capture
setting (FRAMESIZE_* + jpeg_quality). Pick it consciously: on the USB bench
bigger is free, but over LoRa a QXGA JPEG (~200-400 KB) is many minutes of
airtime per photo; size the deployed full-res for what the mesh can afford,
not the sensor max. That ingest
automatically completes the command server-side (the receipt ack above only
marks it `received`), and repeats are harmless (the server re-delivers until
the full-res lands, the upload is idempotent). Unknown `kind`s should be ignored (forward compat). Commands the
mesh never satisfies expire server-side after 14 days.

In the UI this is the "📡 Request full-res" button on any photo that only has
its thumbnail.

## FEATURE REQUEST for the leaf firmware: quality-tiered fetch_full (2026-07-02)

The app now sends `"payload":{"quality":"standard"|"max"}` on `fetch_full`
commands (the UI gates "max" behind an airtime/battery disclaimer). Wanted
leaf behavior:

1. **At capture time, store the sensor-max original** (QXGA 2048×1536 or the
   best the OV3660 + PSRAM budget allows) on SD alongside the thumbnail. SD
   is a consumable anyway; storage is not the constraint, airtime is.
2. **On `fetch_full`:** `quality:"max"` (or absent-payload legacy) → send the
   stored original as-is; `quality:"standard"` → re-encode/downscale the
   stored original to the mesh-affordable size (~800×600) before sending.
   Same `EVT kind=full` framing either way; the server overwrites the stored
   full in place and the UI cache-busts on byte size.
3. Never re-capture (see the bug note above); unknown quality values →
   treat as "standard".

Until implemented, "max" degrades gracefully (leaf sends whatever it has).
