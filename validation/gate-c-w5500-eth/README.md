# Gate C — W5500 SPI ethernet + SX1262 coexistence (gateway-PCB de-risk)

**Status: awaiting the W5500 breakout.** Firmware is ready to flash.

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
| MISO | 13 |   | MISO | 38 |
| MOSI | 11 |   | MOSI | 48 |
| SCS  | 10 |   | NSS  | 14 |
| INT  | 9  |   | RST  | 1  |
| RST  | 8  |   | BUSY | 2  |
| 3.3V + GND | — |   | DIO1 | 42 |

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

## Port note for the production firmware

`gateway/` today pins `espressif32@6.9.0` (core 2.0.x) because the T-Internet-POE env
uses the old `ETH.begin(addr,power,mdc,mdio,type,clk)` signature. The gateway-PCB env
will need the core-3.x API (this gate's `Network.onEvent` + `ETH.begin(ETH_PHY_W5500,
…)` shape) via the pioarduino platform — plan that port as part of the PCB firmware
env, and leave the T-Internet-POE env pinned where it is.
