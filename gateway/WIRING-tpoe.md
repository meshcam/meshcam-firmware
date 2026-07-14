# Wiring the SX1262 onto the T-Internet-POE (v1.0)

Nine connections put a **Waveshare Core1262-HF** on the LILYGO **T-Internet-POE
v1.0**'s 8×2 header. The `poe-gateway*` envs already build for these exact pins
(`platformio.ini` → `[poe_common]`); solder, reboot, done. Pin choice verified
against the v1.0 schematic (RMII ownership) and the no-extra-wires RF-switch
recipe is the one Gate A passed live transfers with, on this same module.

Wire by **silkscreen label** on both boards. All 3.3 V; no 5 V near the module.

```
  T-Internet-POE v1.0                          Core1262-HF (pad-label side up)
  8×2 header                                   ┌──────────────────────────┐
                                               │ ◎ u.FL                   │
   3V3  ──────────────────────────────┐        │ ANT               GND    │
   GND  ──────────────────────────┐   │        │ GND ◄────────┐    GND    │
   IO4  ────────────────────────┐ │   │        │ CS  ◄──────┐ │    RXEN   │ ─┐
   IO33 ──────────────────────┐ │ │   │        │ CLK ◄────┐ │ │    TXEN   │  ├─ leave
   IO32 ────────────────────┐ │ │ │   │        │ MOSI ◄─┐ │ │ │    DIO2   │ ─┘  unwired
   IO35 ◄─────────────────┐ │ │ │ │   │        │ MISO ──┼─┼─┼─┼──► DIO1 ──┼──► IO36
   IO16 ────────────────┐ │ │ │ │ │   │        │ RESET◄─┼─┼─┼─┼─┐  GND    │
   IO34 ◄─────────────┐ │ │ │ │ │ │   └──────► │ BUSY ──┼─┼─┼─┼─┼─► 3V3   │
   IO36 ◄── DIO1      │ │ │ │ │ │ │            └────────┼─┼─┼─┼─┼─────────┘
                      │ │ │ │ │ │ └─ GND ───────────────┘ │ │ │ │
                      │ │ │ │ │ └─── NSS/CS ──────────────┘ │ │ │
                      │ │ │ │ └───── SCK ───────────────────┘ │ │
                      │ │ │ └─────── MOSI ────────────────────┘ │
                      │ │ └───────── MISO (radio→ESP32)         │
                      │ └─────────── NRST ──────────────────────┘
                      └───────────── BUSY (radio→ESP32)
```

(The ASCII sketch is directional shorthand; the table below is the truth.)

| Core1262 pad | GPIO | Dir | Note |
|---|---|---|---|
| 3V3 (VCC) | 3V3 | power | 100 nF + ~10 µF ceramic **at the module**; TX bursts ~120 mA |
| GND | GND | power | any of the module's GND pads |
| CLK (SCK) | **33** | ESP32→radio | keep SCK/BUSY/DIO1 leads < 15 cm |
| MOSI | **32** | ESP32→radio | |
| MISO | **35** | radio→ESP32 | input-only pin: fine, MISO is an MCU input |
| CS (NSS) | **4** | ESP32→radio | **10 K pull-up to 3V3 at the module** (radio deselected during boot) |
| RESET (NRST) | **16** | ESP32→radio | |
| BUSY | **34** | radio→ESP32 | input-only OK |
| DIO1 (IRQ) | **36** | radio→ESP32 | input-only OK |

## Leave unwired

- **RXEN / TXEN / DIO2**: firmware sets `setDio2AsRfSwitch(true)`; this exact
  wiring passed Gate A (live QXGA transfers) on the same module.
- **DIO3** (no pad): feeds the module's 1.8 V TCXO; the build passes `1.8` to
  `begin()` explicitly (RadioLib defaults 1.6; already handled in
  `LoRaInterface.cpp`'s `GATE_B_WIRED_SX1262` case).
- The extra GND pads.

## Pins you must not borrow

- **IO5 / 17 / 18 / 19 / 21 / 22 / 23 / 25 / 26 / 27**: LAN8720 PHY
  (RMII + MDC/MDIO + GPIO17 clock-out + IO5 PHY reset).
- **IO12**: MTDI flash-voltage strapping; **IO0**: boot strapping.
- Spare after this wiring: IO39 (input-only) + the SD set (IO2/13/14/15);
  the TF slot stays usable.

## Power rules

- **Never PoE and the USB downloader board at the same time**: one or the other.
- **Antenna on before power**: TX into an open u.FL can kill the PA.
  Chain: u.FL pigtail → N bulkhead → 915 MHz omni (any 915 load on the bench).

## Verify after soldering

1. Power (PoE *or* downloader), serial 115200.
2. Expect `LoRa init succeeded`. `code -2` = SPI/wiring; `-707/-706` = TCXO/
   oscillator misconfig.
3. `http://<board-ip>/`: radio field shows the profile name, not *ABSENT*.
4. Wake a leaf → announce logs with RSSI/SNR. (Leaves must bake THIS board's
   destination hash as `LEAF_GATEWAY_DEST_HEX`, printed on every boot.)
