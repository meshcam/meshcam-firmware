# Gate B — camera + RNS + TFLite coexistence on one ESP32-S3 (bench test)

**Status: FULL run PASSED with the radio live, 2026-07-02.** Camera + RNS + the 512 KB
tensor arena + the wired SX1262 all coexist on one ESP32-S3. `chip->begin()` returned OK
(**TCXO = 1.8 V confirmed correct** for this Waveshare "SX1262 LoRa Node (HF)" module) and
there was **no pin collision** with the camera bus. The single-MCU leaf is hardware-validated
end to end. Remaining: an actual over-the-air link test (needs a 2nd node/peer + the real
915 antenna) and restoring TX power to 17 dBm (dropped to 10 for the mismatched-antenna
bench). (Earlier no-radio run: 2026-07-01, below.)

### Result — `freenove-s3` WITH radio, 2026-07-02
- **`LoRa init succeeded`** on serial — SX1262 up on the wired SPI (SCK 47 / MOSI 48 /
  MISO 38 / NSS 14 / RST 1 / BUSY 2 / DIO1 42). begin() only succeeds if BUSY + RESET +
  all four SPI lines are truly connected, so this validates the (labels-down) perfboard
  wiring too. **TCXO 1.8 V is correct** — no XTAL/0.0 change needed.
- **Internal-SRAM worst-case free: 316,128 bytes** (~309 KB) vs the 32 KB gate — the radio
  stack cost ~5 KB vs the no-radio 321 KB, as predicted. PSRAM ~6.76 MB free. Ran all 10
  events to VERDICT PASS, no crash → no pin collision, radio + camera + arena coexist.
- Camera now emits ~107 KB frames (OV3660 at higher detail), still comfortably in PSRAM.
- **TX power temporarily 10 dBm** (`LoRaInterface.h`) for a mismatched 2.5 GHz antenna;
  the real quarter-wave 915 antenna + a 2nd node are needed to prove actual RF TX/RX.

### Result — `freenove-s3-noradio`, 2026-07-01
- **Camera on real silicon: PASS.** Every capture produced a ~42 KB UXGA JPEG
  (`full=42266 bytes` …) → OV2640 + PSRAM + the ESP32S3_EYE pin map are all correct on
  the actual Freenove. No pin remap needed.
- **Internal SRAM worst-case free: 321,308 bytes (~314 KB)** vs the 32 KB gate →
  **PASS, ~10× headroom.** Internal SRAM was the feared bottleneck for a single-MCU
  leaf; it isn't close.
- **PSRAM worst-case free: 7,088,283 bytes (~6.76 MB)** of 8 MB, with the arena +
  double-buffered framebuffer live.
- RNS came up alongside the camera (`Transport instance … started`); detector stub
  alternated send 0/1 as designed.
- **Verdict: single-MCU leaf is RAM-validated.** Caveats (real number will be a bit
  tighter): the arena is a 512 KB *stub in PSRAM* (a real `MicroInterpreter` adds some
  internal-SRAM structs); the thumbnail is a 5 KB slice, not a real `frame2jpg` (which
  grabs a transient work buffer); and the LoRa segmentation buffers aren't allocated in
  this build. All easily absorbed by 314 KB of headroom — confirm on the radio run.

Flashed from the dev host toolchain to the laptop (`debian-t14`, board on `/dev/ttyACM0`
via its CH343 UART bridge, `1a86:55d3`): merged the pio build images into one binary
(`esptool merge_bin`), scp'd it over, flashed at 0x0 with a minimal `esptool` venv on the
laptop. Reproducible from `.pio/build/freenove-s3-noradio/` (bootloader/partitions/
firmware + framework `boot_app0.bin` at 0x0/0x8000/0xe000/0x10000).
Parent: [`docs/trailcam/design.md`](../../../docs/trailcam/design.md) → "Gates &
validation". Runs after / alongside [Gate A](../gate-a-resource/).

> Camera (`esp_camera`) + RNS (microReticulum `c02b6e3`) + the TFLite arena stub +
> the bare-SX1262 `LoRaInterface` (new `GATE_B_WIRED_SX1262` board-case) all link.
> Static footprint ~29 KB internal SRAM, ~28% of a 3 MB app — but the real coexistence
> numbers are runtime (framebuffer + arena are PSRAM allocs), which is the test itself.

## What this proves

The single-MCU leaf (one ESP32-S3 doing camera + PIR + tiny-ML + RNS) is **RAM- and
pin-plausible**, not proven. Gate B proves it. Two failure axes:

1. **RAM** — does the camera framebuffer + a TFLite tensor arena + the RNS stack fit?
   The thesis is they coexist because the pipeline is **sequential** (capture → free
   the framebuffer → infer → build thumbnail → hand to RNS), so peak *simultaneous*
   use stays bounded. **Internal SRAM (~512 KB) is the scarce axis** — PSRAM (8 MB)
   easily holds the big buffers; internal SRAM is where camera DMA, RNS, and Wi-Fi-less
   crypto contend. The firmware logs internal + PSRAM **high-water (minimum-free-ever)**
   at every stage and prints a PASS/FAIL on internal SRAM.
2. **GPIO pins** — a camera board burns a lot of GPIOs on the DVP bus; the SX1262 needs
   7 (SCK/MOSI/MISO/NSS/RST/BUSY/DIO1) on *free, non-strapping* pins. If they collide
   the radio won't init (or boot straps wrong). The pin maps below are picked to avoid
   that; confirming it on real silicon is half the point of this gate.

## Board + pin map (the actual deliverable)

Two candidate cam boards, both ≥8 MB PSRAM. Camera GPIOs are verbatim from
arduino-esp32's `camera_pins.h`; SX1262 pins are chosen from what the camera leaves free.

### `freenove-s3` — ESP32-S3-WROOM CAM (default; roomy GPIO)
Camera uses GPIO **{4,5,6,7,8,9,10,11,12,13,15,16,17,18}** (ESP32S3_EYE map). SX1262:

| SX1262 | SCK | MOSI | MISO | NSS | RST | BUSY | DIO1 |
|---|---|---|---|---|---|---|---|
| GPIO | 47 | 48 | **38** | 14 | 1 | 2 | 42 |

Avoids 0/3/45/46 (strapping), 19/20 (USB-JTAG), 26-37 (flash + octal PSRAM). **MISO is 38,
not 21:** GPIO21 is reserved for the deployed leaf's PIR ext0 deep-sleep wake (RTC domain
is 0-21); SPI routes to any pin via the S3 matrix so MISO on 38 is free. GPIO3 (battery
ADC) + 40/41 (BME280 I2C) are also reserved for the leaf — none are wired for this bench
test. See the leaf GPIO budget in `docs/trailcam/hardware.md`.

### `xiao-s3-sense` — Seeed XIAO ESP32S3 Sense (compact)
The Sense camera pins are **all internal to the B2B** {10-18,38,39,40,47,48}, so the
castellated pads stay free. SX1262 on D-pads (GPIO 1-9), keeping 43/44=UART for serial:

| SX1262 | SCK | MOSI | MISO | NSS | RST | BUSY | DIO1 |
|---|---|---|---|---|---|---|---|
| GPIO | 7 | 9 | 8 | 1 | 2 | 3 | 4 |

> **XIAO caveat:** the Wio-SX1262 **kit** mounts on the same B2B the camera uses, so you
> can't stack both — wire a **bare** SX1262 (Ebyte E22-900M / RA-01SH) to the pads above.
> (This corrects an earlier worry that the Sense was *pin-starved* — it isn't; the camera
> is internal. The constraint is the B2B mechanical conflict, not GPIO count.)

### Bench wiring rig — Core1262 ↔ Freenove (chosen, all owned gear)

**Wiring diagram:** [`leaf-wiring.svg`](leaf-wiring.svg) — top panel is the 9 bench-test
wires (MISO on 38); bottom panel is the deployed-leaf additions (PIR 21 / battery ADC 3 /
BME280 40,41) that are NOT wired for the bench test.

**Perfboard grid map:** [`leaf-perfboard-7x9.svg`](leaf-perfboard-7x9.svg) — exact
hole-by-hole placement on a Chanzon 7x9 (cols A..X right→left, rows 1..27). Freenove
socket strips at rows 4 & 14 (10 apart), SX1262 at rows 19 & 26, with the 9 wire runs by
(col,row). Confirmed against the physical boards 2026-07-01.

**Hardware confirmed on the actual boards (2026-07-01, photo):**
- **Freenove ships with male headers soldered on both edges** → it plugs into female
  sockets; never solder the Freenove itself. Silk: top row `5V 14 13 12 11 10 9 46 3 8 18
  17 16 15 7 6 5 4 EN 3V3`, bottom row `GND 19 20 21 47 48 45 0 35 36 37 38 39 40 41 42 2
  1 RX TX`. **GPIO 38/39/40/41 are broken out** (MISO-on-38 + BME280 40/41 + spare 39 all
  valid). 35/36/37 are printed but consumed by the octal PSRAM — avoid.
- **The SX1262 module (bare, 2x8 pads) breaks out `TXEN` and `RXEN`** in addition to
  DIO1/DIO2 — top row `BUSY RESET MISO MOSI CLK CS GND ANT`, bottom `3V3 GND DIO1 DIO2
  TXEN RXEN GND GND`. **Implication:** it has an external RF antenna switch, which the
  plain `setDio2AsRfSwitch(true)` path may not drive on its own — if `begin()` succeeds
  but TX is weak/dead, suspect the RF-switch control (wire DIO2→RXEN, or drive TXEN/RXEN).
  **Mitigation baked into the build: socket the WHOLE module (all 16 pads)** so DIO2/TXEN/
  RXEN stay reachable for a one-wire fix without desoldering. Open until the radio run.


9 connections: SCK 47, MOSI 48, **MISO 38**, NSS 14, RST 1, BUSY 2, DIO1 42, + 3V3 + GND
(Freenove side; Core1262 side is its labeled pads). *(MISO is 38, not 21 — GPIO21 is held
for the leaf's PIR wake; see the pin note above.)* **Skip flexible dupont leads** —
they're the fragile/annoying part. The setup, using gear already in the AliExpress order
history (see [`docs/soldering.md`](../../../docs/soldering.md)):

1. **Solder the owned right-angle 2.54mm male header onto the Core1262** so it's
   breadboard-pluggable (and a friendly first chisel-tip job).
2. **Butt the 700pt + 400pt breadboards long-edge to long-edge** — the Freenove is a wide
   cam board that won't straddle one board's center trench; with the trench gone it sits
   across the seam, each pin row in its own board.
3. **Wire with the owned 840pc preformed (solid-core) jumper kit**, not duponts — they
   seat flush and don't wiggle out. Keep **SCK/BUSY/DIO1 runs short** (<~15 cm); those are
   the first suspects for SPI flakiness.
4. **Antenna on the Core1262's u.FL before power** — use an owned u.FL→SMA cable (5pc from
   2023; the incoming DIYmall antennas aren't even needed for this). Bare PA = fried PA.

*Sturdier + still removable (owned gear):* crimp a single **9-way locking Dupont housing**
with the owned SWANAMB ratchet crimper + 1550 terminals — one connector that plugs on once,
none of the loose-jumper fragility. *Bombproof:* direct-solder a short 9-wire umbilical
(zero connectors), permanent. *Best-of-both (small buy ~$4):* perfboard + **female socket
strips** so both boards plug in over short soldered traces.

## Build setup

1. PlatformIO. `esp_camera.h` ships with the arduino-esp32 core — no extra lib.
2. `LoRaInterface` is already vendored in `lib/lora_interface/` with a `GATE_B_WIRED_SX1262`
   board-case (pins from the `-DRADIO_*_PIN` build_flags). Nothing to patch — a clean
   checkout builds. (LDF auto-discovers it from `lib/`.)
3. The TFLite arena is a raw `heap_caps_malloc` stub (the arena is TFLite-micro's
   dominant RAM cost, so allocating it is the honest proxy). Swap in a real INT8 model +
   `MicroInterpreter::AllocateTensors()` later — `esp-tflite-micro` dep is commented in
   `platformio.ini` and the call site is a TODO in `main.cpp`.

## Run

**Arrival day (Core1262 not here yet — lands Jul 8): flash `freenove-s3-noradio`.**
It's the full Gate B minus the SX1262 bring-up, so it runs the day the board arrives:
the first captured event prints the JPEG byte size (proves OV2640 + PSRAM + the pin map
on real silicon), and the 10-event run gives the internal-SRAM coexistence verdict. The
LoRa interface's own RAM is just the small segmentation buffers, so its absence barely
moves the number — the dominant costs (framebuffer, 512 KB arena, thumbnail) all run.
Confirmed to compile + link clean (RAM 8.6%, flash 23.5%).

```bash
pio run -e freenove-s3-noradio -t upload --upload-port /dev/ttyACM0   # arrival day, no radio
pio device monitor -p /dev/ttyACM0 -b 115200
```

**Jul 8, radio wired:** switch to `-e freenove-s3` for the full run *with* the SX1262 on
the pins in the map above (proves no pin collision + the RAM number including the radio).

```bash
pio run -e freenove-s3 -t upload --upload-port /dev/ttyACM0
pio device monitor -p /dev/ttyACM0 -b 115200
```

You'll see a heap line per stage, then a RESULT block:

```
[gateB] boot                 | INT free=... min=... | PSRAM free=... min=...
[gateB] after camera_init    | ...
[gateB] after rns_init       | ...
[gateB] after tflite_init    | ...
[gateB] after capture        | ...        <- per event (x10)
[gateB] after fb return      | ...
[gateB] ===== RESULT =====
[gateB] internal-SRAM worst-case free: NNNNN bytes (gate: > 32768)
[gateB] VERDICT: PASS (RAM fits)
```

## Pass / fail

- **PASS** = internal-SRAM worst-case free stays above `GATE_B_MIN_INTERNAL_FREE`
  (32 KB default) across all events, the radio inits without a pin-collision crash, and
  a deep-sleep/wake cycle still rejoins the mesh (stateless boot). Record the actual
  worst-case number — it's the headroom you have for the real model + bigger frames.
- **Tighten it real:** swap the stub for the actual detector model (its real
  `AllocateTensors()` arena), and replace the thumbnail slice with a real downscale +
  re-encode (`frame2jpg`). Those are the two places the stub understates RAM.

## Outcomes → build impact

- ✅ **Fits with margin** → single-MCU leaf confirmed. Lock the cam board; the
  `Where AI/Claude earns its keep` list in the design doc becomes the firmware backlog.
- ⚠️ **Fits only with the arena in PSRAM / small frames** → workable but constrains the
  model and capture resolution; note the ceiling.
- ❌ **Internal SRAM too tight, or unavoidable pin collision** → go **2-MCU leaf**: a
  cam board for capture+ML + a separate `microReticulum_Firmware` RNode over serial.
  More parts/power, but decouples the budgets.

## Closed at compile time

- [x] `camera_config_t` field names — `pin_sccb_sda/scl` compiles on the current core.
- [x] `LoRaInterface` bare-SX1262 support — added a `GATE_B_WIRED_SX1262` board-case
      (pins from `-DRADIO_*_PIN` build_flags); both envs link.
- [x] `RNS::Bytes::append` — resolved in Gate A (`append(uint8_t)`).
- [x] `seeed_xiao_esp32s3` board id + `qio_opi` memory_type — valid; the env links.

## Open — runtime only (needs the camera board + a wired Core1262)

- [ ] Real `AllocateTensors()` arena size for the chosen detector vs the 512 KB stub.
- [ ] Camera actually inits + PSRAM detected at runtime (static link can't show it).
- [ ] Core1262 TCXO voltage — the board-case assumes 1.8V; set `begin(...,0.0,...)` if
      your module is XTAL-based (`LoRaInterface.cpp`, the `GATE_B_WIRED_SX1262` block).
- [ ] The 7-wire SPI hookup matches the pin map (`gate_b_board.h` ⇄ `platformio.ini`).
