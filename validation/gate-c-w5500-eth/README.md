# Gate C — W5500 SPI ethernet + SX1262 coexistence (gateway-PCB de-risk)

**Status: STAGE 1 PASSED 2026-08-19.** Results at the bottom. Stage 2 is unrun and
is no longer a fab gate (see the v0.2 section). Original note below.

**Hardware in hand.** The HiLetgo W5500 2-pc landed 2026-07-21
(Amazon #112-9401697-7953825). Firmware is flashable as-is; this is a bench session,
not a research task. Stage 1 needs no camera and no radio.

The gateway PCB (homelab `docs/trailcam/gateway-pcb.md`) moves from the
T-Internet-POE's classic-ESP32 + LAN8720 (RMII) to **ESP32-S3 + W5500 (SPI)**. The
W5500-through-`ETH` path on arduino-esp32 **core 3.x** is the one firmware unknown on
that board. This gate proves it on owned hardware before the schematic is committed —
the same role gate-B played for camera+RNS coexistence.

## Hardware

- **Freenove ESP32-S3-WROOM CAM board** (owned, the gate-B board) — no camera attached.
- **W5500 breakout** (HiLetgo 2-pc, ~$13:
  https://www.amazon.com/HiLetgo-Ethernet-Network-Support-Microcontroller/dp/B0CDWX9VQ5)
- For the `-radio` env: **Waveshare Core1262-HF** jumpered per the gate-B/tower recipe
  (TCXO on DIO3 @1.8 V, DIO2 = RF switch). **Antenna on before power.**

## Wiring (= the gateway PCB GPIO draft, validated verbatim)

| W5500 breakout | Freenove GPIO |   | Core1262 | Freenove GPIO |
|---|---|---|---|---|
| SCLK | 12 |   | SCK  | 47 |
| MISO | 13 |   | MISO | **41** |
| MOSI | 11 |   | MOSI | 48 |
| SCS  | 10 |   | NSS  | 14 |
| INT  | 9  |   | RST  | 1  |
| RST  | 8  |   | BUSY | 2  |
| 3.3V + GND | — |   | DIO1 | 42 |

**Radio MISO is 41 on this bench rig, not the leaf PCB's 38.** GPIO38 on the Freenove
is hard-wired to the onboard microSD slot (SD_MMC CMD), so the bench leaf already
relocated radio MISO 21 → 38 → **41** (homelab `docs/trailcam/hardware.md`, "GPIO
budget"). The leaf/gateway PCBs keep 38; only this board needs the move. Corrected
2026-08-19 — the env had shipped with 38 and would have fought the SD slot.

**Board pin locations** (transcribed off the boards 2026-08-19, kept here so nobody
re-derives them from photos): on the Freenove, GPIO **9, 10, 11, 12, 13 are five
contiguous pins** near the bottom of the LEFT header (13 is 3rd up from the bottom,
9 is 7th up), and GPIO **8** is two further up past 46 and 3. The board's **only GND
pin is the bottom-RIGHT corner**; the bottom-LEFT corner is 5V. Full transcription of
both 20-pin columns plus the W5500 header: homelab `docs/trailcam/hardware.md`,
"Bench board pinouts". A printable colour wiring sheet for stage 1 lives at
homelab `docs/trailcam/wiring/gate-c-w5500.pdf`.

W5500 breakout note: the HiLetgo module wants 3.3 V logic (it is 3.3 V native; some
boards are sold as "5 V tolerant" — power it from 3V3 regardless) and can draw ~130 mA
active: power from the Freenove's 3V3 pin, common ground, keep jumpers short (SPI runs
at tens of MHz; 10 cm dupont is usually fine at the default clock, drop
`ETH_PHY_SPI_FREQ_MHZ` if flaky).

## Run it

```bash
pio run -e freenove-s3-eth       -t upload   # day the breakout arrives: link/DHCP/HTTP
pio run -e freenove-s3-eth-radio -t upload   # after jumpering the Core1262
pio device monitor
```

## Pass criteria

**freenove-s3-eth** (proves the driver + our pin map):
1. `ETH.begin(W5500) -> ok`, link UP, DHCP lease printed.
2. HTTP GETs succeed repeatedly (`[score] ... http ok/fail=N/0`).

**freenove-s3-eth-radio** (proves the two SPI buses don't fight):
3. `radio.begin()` returns 0 with the W5500 already active (and vice-versa — power
   cycle a few times; init order must not matter).
4. 10 clean minutes: HTTP ok count climbing, beacon `tx ok` climbing, zero WDT
   resets, heap flat. With a second radio (leaf/surveyor) nearby TXing, `rx_irq`
   climbs while the HTTP loop runs.
5. Note the scoreboard heap numbers in this README when done — the gateway RAM story
   (PSRAM present, no LAN8720 RMII buffers) is a selling point of the S3+W5500 design.

Then check the box in `gateway-pcb.md` → VERIFY before fab, and record any deviations
(SPI clock, INT behavior, init-order gotchas) HERE — this file is the bring-up
reference the real gateway env gets written from.

## What v0.2 of the gateway PCB changed (read before running stage 2)

This gate was written against `gateway-pcb.md` **"GPIO map draft (v0.1)"**, which that
doc now marks **SUPERSEDED by "GPIO map v0.2" (as built)**. Two consequences:

- **Stage 1 is untouched and still the gate that matters.** v0.2 keeps the W5500 on
  SCK 12, MISO 13, MOSI 11, CS 10, INT 9, RST 8. The one addition is a **10 k pull-up
  on CS** — the bench rig has no such resistor, so CS floats until the S3 drives it. If
  stage 1 is marginal or the first transaction after reset misbehaves, add a 10 k from
  CS (GPIO10) to 3V3 before blaming the driver.
- **Stage 2 now validates a topology the board does not ship.** v0.2 dropped the SPI
  SX1262 for a **RAK3172 on UART2** (TX IO14 → RAK 1 via 1k, RX IO2 ← RAK 2, NRST IO1,
  BOOT0 IO42). So `freenove-s3-eth-radio` proves SPI-vs-SPI coexistence that the v0.2
  board never has to survive. It is still a fine general result, but it is **no longer
  a fab pre-condition** — the ethernet-vs-radio question for v0.2 is UART DMA against
  W5500 SPI, which this env does not test. Decide whether stage 2 is worth the bench
  time before jumpering the Core1262.

## Port note for the production firmware

`gateway/` today pins `espressif32@6.9.0` (core 2.0.x) because the T-Internet-POE env
uses the old `ETH.begin(addr,power,mdc,mdio,type,clk)` signature. The gateway-PCB env
will need the core-3.x API (this gate's `Network.onEvent` + `ETH.begin(ETH_PHY_W5500,
…)` shape) via the pioarduino platform — plan that port as part of the PCB firmware
env, and leave the T-Internet-POE env pinned where it is.

## Results — stage 1, 2026-08-19: PASS

Freenove ESP32-S3-WROOM-1 N16R8, HiLetgo W5500, wired exactly per the table above,
`freenove-s3-eth` at the stock 20 MHz SPI (no de-rating needed):

```
[eth] ETH.begin(W5500) -> ok
[eth] link UP
[eth] got IP 10.7.1.109  gw 10.7.1.1  100 Mbps FD
[score] up=30s  http ok/fail=5/0    heap=312156 psram=8383916
[score] up=150s http ok/fail=29/0   heap=309404 psram=8383916
[score] up=780s http ok/fail=155/0  heap=309372 psram=8383916
```

| Pass criterion | Result |
|---|---|
| `ETH.begin(W5500) -> ok`, link UP, DHCP | ✅ 10.7.1.109, gw 10.7.1.1, 100 Mbps full duplex |
| HTTP GETs succeed repeatedly | ✅ **155 GETs, 0 failures, 13 min** |
| Heap flat | ✅ 314,156 pre-link → **309,372–309,404 from t=150 s, flat 10.5 min** (32 B oscillation = noise) |
| No WDT resets / link drops | ✅ none |

PSRAM constant at 8,383,916. **Heap number for the gateway RAM story: ~309.4 KB free
with the netif, LwIP, and an HTTP client live**, on a board whose PSRAM is untouched.

### The first breakout was dead on arrival — read this before debugging the next one

The unit that came out of the 2-pack first never answered. It cost most of an evening,
and **the wiring was correct from the very first attempt**. What it looked like:
`w5500_reset(290): reset timeout` from the driver, and `0xff` for VERSIONR from a raw
probe under *every* condition — both SPI modes, 20 MHz and 4 MHz, RST driven and RST
released, and finally under a brute-force scan of all **720** assignments of
SCK/MISO/MOSI/CS across the six wired GPIOs. Zero hits. The spare answered `04`
immediately and the scan's single hit was the intended map.

Every wiring theory chased was wrong, so do not repeat them: the W5500 header's row
ambiguity, an overlap with the Freenove camera DVP bus (real overlap — GPIO 8/9/10/11/
12/13 are camera D2/D1/D3/D0/D4/PCLK — but the camera was not attached), SPI clock,
the RST line holding the chip in reset, and the ground return. 3.3 V, GND continuity
and every jumper measured good the whole time.

Two readings that look like faults and are not:

- **All six GPIOs reading high under both an internal pull-up and pull-down.** That
  module has a bank of 10 kΩ pull-ups (the `103` parts on it), so an idle bus floats
  to the rail. Normal, and it also means a pull-up masks an open signal wire.
- **2.866 V on the 5V pin while 3.3V reads 3.276 V.** That is the AMS1117 back-feeding
  its input through the pass transistor's body diode, ~0.4 V below the output, with
  nothing connected to 5V. Normal.

**Next time: swap the spare in before theorising, and write the register-read probe as
the first instrument, not the fifth.** A probe that prints raw VERSIONR bytes
distinguishes "not connected" (`0xff`/`0x00`) from "wired wrong" (garbage) from "dead"
in one flash, and the permutation scan settles wiring outright. Both are ~60 lines.
This is also the argument for always buying the 2-pack on a part that gates a fab
order, which `gateway-pcb.md` had already made in July and which paid for itself here.

