# MeshCam firmware

Open-source firmware for a **LoRa mesh trail camera network**: solar leaf
cameras deep-sleep in the woods, wake on motion, filter captures with
on-device tiny-ML, and relay images over a [Reticulum](https://reticulum.network)
LoRa mesh to a gateway on the property's internet. No cellular modem per
camera, no per-camera subscription, no cloud required.

Part of the [MeshCam](https://getmeshcam.com) project. The self-hostable web
app, hardware design files, and device API live in sibling repos; see
[docs.getmeshcam.com](https://docs.getmeshcam.com). Live demo with real
captures: [demo.getmeshcam.com](https://demo.getmeshcam.com).

## Why it looks this way

"900 MHz" is a band, not a capability. Physics gives you an iron triangle:

> **Long range, low power, high bandwidth: pick two.**

The only corner that survives "heavy woods, all solar, months unattended" is
LoRa: miles through foliage at ~1–5 kbps. So MeshCam does what every serious
system in this corner does: **tiny thumbnail now, full-resolution deferred**,
with the mesh moving images opportunistically and never losing one
(store-and-forward, per-shot telemetry).

Transport is **Reticulum** (via the Apache-2.0
[microReticulum](https://github.com/attermann/microReticulum) C++ library,
pinned): encryption, routing, addressing, and a first-class arbitrary-size
`Resource` transfer primitive. Syncing an arbitrary-size blob over a LoRa
mesh comes built in. The leaf is a **single ESP32-S3** running camera + PIR + TFLite
person/animal gating + the RNS stack together: no second MCU, no radio
daughter-board protocol.

## Repo layout

| Dir | What it is | Target |
|---|---|---|
| `leaf/` | The camera node: PIR wake → capture → tiny-ML gate → RNS `Resource` over LoRa → deep sleep. OTA, serial command protocol, adaptive radio profiles (ADR), store-and-forward spool. | ESP32-S3 + SX1262 |
| `gateway/` | LoRa/RNS in, network + HTTPS out; pushes to any [ingest-API](https://github.com/meshcam/meshcam-api) server. No PC in the data path. Two board variants: Heltec V3 (WiFi uplink, OLED status) and T-Internet-POE (wired ethernet + PoE, hand-wired SX1262, RNS-over-UDP on the LAN). | Heltec V3 / LILYGO T-Internet-POE |
| `bridge-poe/` | Ethernet bring-up validation for the T-Internet-POE (RNS-over-UDP interop with Python RNS; superseded by `gateway/`'s `poe-*` envs for deployment). | LILYGO T-Internet-POE |
| `bridge-host/` | Dev bridge: leaf serial console → ingest API from a laptop. The "dev kit" gateway. | any host w/ Python |
| `validation/` | The bench-validation gates that proved the architecture (kept as engineering history): `gate-a-resource` (RNS Resource on-device), `gate-b-coexist` (camera + RNS + TFLite on one MCU), `gate-a2-interop` (interop with reference Python RNS). | — |

## Status (July 2026)

Field testing is underway. The whole chain runs today: PIR wake → capture →
on-device TFLite gate → LoRa RNS `Resource` transfer → gateway → gallery,
including full-resolution QXGA images over the mesh, adaptive radio
profiles, and interop with the reference Python RNS. Custom leaf PCBs are
in pilot production.

To hear when hardware kits are available: [getmeshcam.com](https://getmeshcam.com).

## Build & flash (leaf)

```
pip install platformio
cd leaf
pio run -e freenove-s3 -t upload    # Freenove ESP32-S3 WROOM CAM dev board
pio device monitor -b 115200
```

Each subproject's README covers its board variants, build flags (site name,
gateway destination, ingest URL), and the serial command protocol. Radios are
915 MHz (US) SX1262 modules; check your regional band plan.

## License

GPL-3.0, see [LICENSE](LICENSE). Commercial licensing available; open an
issue or email hello@getmeshcam.com.
