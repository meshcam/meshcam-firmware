# PoE bridge: wired gateway on a LilyGo T-Internet-POE

**Status: Step-1 PROVEN on hardware 2026-07-01.** microReticulum runs over the wired
LAN8720 ethernet and **interoperates with reference Python RNS**: a real `Link` formed
and 5 KB + 10 KB `Resource`s completed with verified SHA256, board↔Python-RNS over
ethernet. **OTA over ethernet also validated** (pushed firmware to a PoE-only board, no
USB). Runs on an ESP32-WROOM (espressif32@6.9.0). Parent:
[`docs/trailcam/hardware.md`](../../../docs/trailcam/hardware.md) → "Gateway / tower node".

> **WROOM RAM ceiling (found in test):** *originating* a ≥20 KB blob (5/50/250 KB was the
> Gate A ladder) resets this ESP32-WROOM: RAM + RNS segmentation buffers exceed the ~270 KB
> heap. Irrelevant to production: real thumbnails are ~5 KB, and the deployed board is a
> **BRIDGE** (forwards the S3+PSRAM leaf's Resources at L3, never originates big blobs). Test
> sizes capped at 5/10 KB.

The production gateway can be a **thin wired bridge at the remote site**: a LilyGo
**T-Internet-POE** (ESP32-WROOM + LAN8720 ethernet + **onboard PoE**) running
microReticulum, plugged straight into their PoE switch: no Pi, no WiFi, no SD card.
Bench-validated on the home Brocade ICX6450 (PoE Class 3, 100M link, DHCP, ping) 2026-06-30.

## Why two steps

The bridge firmware has two halves with very different risk, so we prove them separately:

- **Step 1, ethernet (this, now):** does microReticulum's UDP interface run over **wired
  ethernet** (LAN8720) and link to reference **Python RNS**? This is the *unproven* part,
  and it needs **no radio**. Doing it while the Core1262 is in transit de-risks the whole
  design early.
- **Step 2, +LoRa (when the Core1262 lands, Jul 8):** bolt the (Gate-A-proven) `LoRaInterface`
  onto the already-working ethernet node → the Gate A2 **BRIDGE** role (LoRa↔UDP transport).

## What Step 1 proves / how it's built

- `src/poe_config.h`: the **LAN8720 pin map** (MDC 23, MDIO 18, clock `GPIO17_OUT`, addr 0,
  no power pin). Verified vs ESPHome/Tasmota configs *and* the board pulled DHCP with it.
- `src/main.cpp`: `ETH.begin()` (classic core-2.x API; the platform is **pinned to
  espressif32@6.9.0** for exactly this signature) brings up ethernet + DHCP, then
  microReticulum starts a UDP interface over it, opens a Link to the Python peer, and pushes
  5/50/250 KB Resources. Mirrors the proven Gate A Resource/Link API.
- `lib/udp_interface/UDPInterface.{h,cpp}`: vendored from microReticulum `c02b6e3`, **patched:
  the upstream Arduino `start()` hardcodes `WiFi.begin("wifi_ssid","wifi_password")` and blocks
  forever on `WL_CONNECTED`.** That's stripped; the sketch brings up ETH instead, and
  `WiFiUDP` on ESP32 is just an lwIP socket so it routes over ethernet unchanged. **This patch
  is the crux of the whole step.**

## Run it (the test)

**1. Python peer** on any machine on the same LAN segment as the board:
```bash
python3 -m venv ~/.rns-venv && ~/.rns-venv/bin/pip install rns
# append the [[poe UDP]] stanza from pi/reticulum-config.example to ~/.reticulum/config
~/.rns-venv/bin/python pi/rns_peer.py server      # prints its destination hash, then announces
```

**2. Point the board at it:** paste the 32-hex destination hash into `POE_PI_DEST_HEX`
in `src/poe_config.h`.

**3. Flash** over the LilyGo **downloader board** (USB → laptop; the board has no onboard USB):
```bash
~/.pio-venv/bin/pio run -e poe-endpoint -t upload --upload-port /dev/ttyUSB0
~/.pio-venv/bin/pio device monitor -b 115200
```
Keep the downloader attached for power + serial, and plug the board's RJ45 into a switch
port for ethernet (PoE not needed while USB-powered).

## Pass / fail

- **PASS** = board serial shows `ETH got IP` → `LINK UP to Pi over wired ethernet`, and the
  **Python peer logs `RESOURCE COMPLETE`** for each of 5/50/250 KB. That proves
  microReticulum-UDP ↔ Python-RNS interop **over wired ethernet**, the open question.
- **If the link never forms:** the board + peer must share an L2 broadcast domain (both are
  in `your LAN segment`; confirm same VLAN). If broadcast discovery is flaky, switch to
  directed: set the peer's `forward_ip` to the board's IP, and rebuild the firmware with
  `-DDEFAULT_UDP_REMOTE_HOST=\"<peer-ip>\"`.
- **Isolation test (optional):** flash a *second* T-Internet-POE and run board↔board to
  separate "wired UDP transport works" from "interops with Python RNS."

> **Superseded for deployment (2026-07-09):** the production wired gateway now lives in
> [`../gateway/`](../gateway/) as the `poe-gateway*` PlatformIO envs: the full gateway
> role (ingest POST, ADR, command downlink, time sync, telemetry, OTA) on this same
> board, radio-optional at boot. This subproject remains as the ethernet/RNS-over-UDP
> interop validation record.

## Step 2: when the Core1262 arrives (Jul 8)

Wire the SX1262 to free GPIO (the LAN8720 RMII uses 17/18/19/21-23/25-27 + 0; pick SPI pins
around it), add a board-case to `LoRaInterface` (same as Gate B's `GATE_B_WIRED_SX1262`), add
`-DA2_ROLE_BRIDGE`-style LoRa interface registration, and the endpoint becomes a LoRa↔UDP
transport node. The home k8s `rnsd` connects to it over the WireGuard tunnel.
