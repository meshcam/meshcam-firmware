# radio-test-mode — SX1262 cert-lab bench firmware

The RF test-mode firmware the FCC plan calls for (homelab
`docs/trailcam/fcc-certification.md`, **option C** — certify our own SX1262/E22 radio
module once, every SKU inherits via "Contains FCC ID"). It puts the radio section into
the emission modes a **§15.247 DTS** test lab drives, over a plain USB-serial console.
No RNS, no packet stack — just the radio.

Built now, **while the radio layer is warm**, on owned hardware (Core1262-HF, same
SX1262 die as the production E22-900M22S). Status: **compile-clean draft, not yet
bench-validated on a spectrum analyzer** — see "What's left" below.

## Modes → what the lab measures with each

| Command | SX126x primitive | Lab uses it for |
|---|---|---|
| `cw`  | `SetTxContinuousWave` (0xD1, RadioLib `transmitDirect`) | frequency accuracy; unmodulated carrier / occupied-bw reference; spurious sweeps |
| `mod` | `SetTxInfinitePreamble` (0xD2, raw cmd) | **the DTS emission**: continuous LoRa chirp at the set BW → 6 dB bandwidth (must be ≥500 kHz, §15.247(a)(2)), PSD (8 dBm/3 kHz), peak conducted power, band-edge, and the **KDB 996369 D04** spurious/harmonic sweep |
| `pkt` | back-to-back 255 B `transmit()` | alternative 100%-duty *framed* modulated source (some labs prefer real packets over infinite preamble) |

Hopping is **off by construction** — one fixed channel until you send `freq`. That is
the DTS operating shape (fixed-channel digital modulation), not FHSS.

## Serial console (115200)

```
help | ?          command list
status            current config + state
freq <MHz>        carrier (e.g. freq 902.0)
bw <kHz>          LoRa bandwidth (500 250 125 …; DTS wants ≥500)
sf <7..12>        spreading factor
cr <5..8>         coding rate 4/x
pwr <dBm>         TX power (−9..22)
cw | mod | pkt    start a mode
stop | idle       standby, stop TX
```
Changing `freq/bw/sf/cr/pwr` while transmitting drops to IDLE; re-send `cw`/`mod`/`pkt`
to re-arm with the new setting. Radio config applied every mode entry, so the sequence
the lab wants is just: set params → `mod` → measure → `stop` → next channel.

## Build / flash

```
pio run -e freenove-s3-radiotest              # compile
pio run -e freenove-s3-radiotest -t upload    # flash
pio device monitor -b 115200                  # then: help
```

Radio bus defaults = the leaf PCB / gate-c pins (Core1262-HF jumpered on a Freenove
ESP32-S3): SCK 47 / MOSI 48 / MISO 38 / CS 14 / RST 1 / BUSY 2 / DIO1 42. TCXO 1.8 V on
DIO3, DIO2 → RF switch, OCP raised to 140 mA so +22 dBm isn't silently clamped (all per
`docs/trailcam/e22-dossier.md`). Override the `-DRADIO_*` flags for a different board.

## ⚠️ SAFETY

`cw`/`mod`/`pkt` radiate **continuously at up to +22 dBm**. Bench into a spectrum
analyzer / tinySA through attenuation or a shielded fixture — never a bare antenna on
the desk, and never on a channel that puts the emission outside 902–928 MHz.

## Suggested band-edge channels (US 902–928, bw500)

At BW 500 kHz the 6 dB bandwidth is ~500 kHz, so keep the carrier ≥ ~0.28 MHz inside
each edge: **low ≈ 902.5**, **mid ≈ 915.0**, **high ≈ 927.5**. The lab confirms the
exact worst-case channels; these are the starting set.

## What's left (the bench-validation gate — needs hardware + a spectrum analyzer)

1. Flash a Core1262-HF board, confirm `[radio] SX1262 up`.
2. `cw` at 915.0 → tinySA shows a clean carrier at 915.000; check frequency error.
3. `mod` at bw500 → confirm a ~500 kHz-wide modulated skirt (the DTS 6 dB bw).
4. `pkt` → confirm ~100% duty framed output.
5. Sweep harmonics (2nd/3rd) under `mod` — the 996369 D04 pre-scan the dossier flags.
6. Re-run all of the above on the **E22-900M22S** board once it exists (the module
   actually being certified) — same firmware, same SX1262 die, expected parity.
