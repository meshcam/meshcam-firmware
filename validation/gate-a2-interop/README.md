# Gate A2: microReticulum ↔ Python RNS interop + the gateway bridge

**Status:** skeleton, not yet built/flashed (2026-06-30). No hardware in hand.
Parent: [`docs/trailcam/design.md`](../../../docs/trailcam/design.md) → "Gates &
validation". Depends on [Gate A](../gate-a-resource/) (Resource-on-device).

## The decision this gate makes

How does a leaf's image get from the LoRa mesh onto the Pi gateway (and thence the
NAS)? The leaf runs **microReticulum (C++)**; the Pi runs **reference Python RNS**.
Two things must be true and neither is proven:

1. The two implementations **interoperate** at L3 (a Resource sent by C++ concludes
   COMPLETE in Python).
2. A microReticulum board can act as an **L3 router** bridging the LoRa mesh to the
   Pi over IP.

## The architecture (what we're validating, and why)

**Bridge at L3 over IP: do NOT mix two L2 framings on the radio, and no RNode.**

```
[leaf: microReticulum]                [gateway BOARD: microReticulum]        [Pi: Python RNS]
  LoRaInterface  ----LoRa(same L2)----  LoRaInterface + UDPInterface  --WiFi/UDP--  UDPInterface
                                        transport_enabled = true (routes L3)
```

- **LoRa hop** is microReticulum ↔ microReticulum, **same `LoRaInterface` framing**
  (the custom 254-byte split-packet L2 proven in Gate A). No air-format mismatch.
- **IP hop** is microReticulum `UDPInterface` ↔ Python RNS `UDPInterface`: a standard,
  raw-RNS-frame-in-UDP interface on both sides, the **most likely-compatible** surface
  between the two implementations.
- The gateway board is an **RNS transport node** (`transport_enabled=true`) that routes
  between its LoRa and UDP interfaces. RNS is L2-agnostic at L3, so a Resource flows
  leaf→board→Pi without anyone translating framings.

**Why not RNode?** microReticulum's `LoRaInterface` is not RNode air format, so a
leaf could not talk directly to a Pi+RNode without proof anyway. Bridging at L3 (UDP)
sidesteps the entire RNode-air-format question and keeps the only cross-implementation
surface as `UDPInterface`↔`UDPInterface`. (If interop *fails* even there, see Fallbacks.)

## Test ladder

Two rungs, smallest-blast-radius first. **A2a is the make-or-break**: if the C++ and
Python stacks don't interop over a trivial UDP LAN link, nothing downstream matters, so
prove that before adding LoRa.

### A2a: cross-implementation interop, IP only (no LoRa)
Hardware: **1 Heltec V3** (WiFi) + the **Pi**. Both on the same LAN/subnet.

```bash
# Pi: install RNS, add the Gate-A2 UDP interface to ~/.reticulum/config
pip install rns
#   (append the [[Gate A2 UDP]] block from pi/reticulum-config.example)
./pi/rns_peer.py server
#   -> logs "SERVER destination hash: <hex>"

# Board: set A2_WIFI_* and paste that hash into gateway-board/src/a2_config.h, then:
pio run -e endpoint -t upload && pio device monitor
```

**PASS** = the Pi logs `RESOURCE COMPLETE: 256000 bytes ...` for all three sizes the
board pushes. That proves microReticulum (C++) and Python RNS interoperate end-to-end
including the Resource primitive across the language boundary.

### A2b: the full gateway (LoRa + bridge)
Hardware: **2 Heltec V3** (leaf + gateway-board) + the **Pi**.

1. Pi: same `./pi/rns_peer.py server` (owns the destination).
2. Gateway-board: `pio run -e bridge -t upload` (LoRa + UDP, transport node).
3. Leaf: flash the **Gate A `heltec-v3-client` firmware**, but point its
   `GATE_A_SERVER_DEST_HEX` at the **Pi's** destination hash (not another board).

**PASS** = a Resource initiated on the leaf over LoRa is forwarded by the gateway-board
over UDP and concludes COMPLETE on the Pi. That's the real production backhaul path,
end to end, minus the NAS hand-off (which is out of scope; "image on the Pi" is the
finish line per the design doc).

## Hardware reuse

Everything is the **phase-1 kit** again: A2a = 1 Heltec + Pi; A2b = both Heltecs + Pi.
The leaf in A2b is literally the Gate A client firmware re-pointed. No new boards.

## Outcomes → build impact

- ✅ **A2a + A2b pass** → gateway architecture locked: microReticulum mesh + a
  Heltec-class L3 bridge + Python RNS on the Pi. The backhaul (Pi → NAS over the WG
  tunnel) is the already-solved, out-of-scope part.
- ⚠️ **A2a passes, A2b flakes** → interop is fine; the bridge/routing needs work (MTU,
  transport timing). Tractable: iterate on the bridge, or split into two RNS networks
  joined by a small app-level relay on the board.
- ❌ **A2a fails (no C++↔Python interop)** → don't fight it. Fallbacks:
  - **Pi runs microReticulum native** (it builds for Linux too) instead of Python RNS:
    one implementation end to end, no interop surface. Simplest fix.
  - **RNode-everywhere**: leaf speaks an RNode-compatible interface, gateway = Pi+RNode
    via `RNodeInterface`. Requires the leaf to do RNode air format (re-opens that
    question), so only if going native on the Pi is somehow unacceptable.

## Files

| Path | What |
|---|---|
| `pi/rns_peer.py` | Python RNS peer: `server` (owns dest, receives Resources) / `client` (reverse test). API grounded in RNS `Examples/Filetransfer.py`. |
| `pi/reticulum-config.example` | the `[[Gate A2 UDP]]` interface stanza for `~/.reticulum/config` |
| `gateway-board/src/main.cpp` | one firmware, `-DA2_ROLE_ENDPOINT` (A2a) / `-DA2_ROLE_BRIDGE` (A2b) |
| `gateway-board/src/a2_config.h` | WiFi creds, Pi dest hash, blob sizes |
| `gateway-board/platformio.ini` | `endpoint` / `bridge` envs (Heltec V3) |

## Open / verify-before-build

- [ ] **The core unknown:** microReticulum `UDPInterface` wire-compatibility with
      Python RNS `UDPInterface` (both should be raw-RNS-frame-in-UDP on port 4242, but
      confirm: broadcast vs forward_ip semantics, any framing header).
- [ ] How the vendored Arduino `UDPInterface` gets WiFi creds (its `.cpp` calls
      `WiFi.begin` with internal members; wire `A2_WIFI_*` into its ctor/setter, or
      rely on us bringing WiFi up first).
- [ ] `RNS::Bytes` single-byte append (shared TODO with Gate A).
- [ ] Python `resource.size` field name + `resource.data.read()` for the received bytes
      (used in `rns_peer.py`; from Filetransfer.py but confirm on your RNS version).
- [ ] For a fixed board IP, set `forward_ip` to the board instead of LAN broadcast.
