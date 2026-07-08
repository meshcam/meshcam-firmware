#pragma once
// LilyGo T-Internet-POE ethernet (LAN8720) pin map — confirmed against ESPHome/Tasmota
// configs and the board itself (it pulled DHCP with this layout on the bench 2026-06-30).
// Classic Arduino-ESP32 core 2.x ETH API (see platformio.ini pin note).
#include <ETH.h>

#define POE_ETH_ADDR        0
#define POE_ETH_POWER_PIN  -1                       // no PHY power-enable GPIO
#define POE_ETH_MDC_PIN    23
#define POE_ETH_MDIO_PIN   18
#define POE_ETH_TYPE       ETH_PHY_LAN8720
#define POE_ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT     // T-Internet-POE clocks the PHY on GPIO17

// ENDPOINT test: the Python RNS peer (pi/rns_peer.py) prints its destination hash on boot.
// Paste the 32 hex chars here before flashing.
// Set from rns_peer.py's printed hash before each test (a fresh peer = a fresh hash;
// persist the peer identity if you want it stable). 2026-07-01 interop run used
// 7f5ad945211591ccb7d8f23db28a0cc8 and completed 5 KB + 10 KB Resources.
#define POE_PI_DEST_HEX    "00000000000000000000000000000000"   // <-- set from rns_peer.py

// Blob sizes pushed to the peer as RNS Resources (C++ -> Python). Sized to realistic
// trail-cam thumbnails (~5-20 KB) AND to the ESP32-WROOM's RAM: a 50 KB+ in-RAM blob plus
// RNS's segmentation buffers overflow the ~270 KB free heap and reset the board (proven
// 2026-07-01 — the 5 KB completed, the 50 KB crash-looped). Gate A's 50/250 KB ladder is
// fine on the ESP32-S3 leaf (8 MB PSRAM); this WROOM bridge stays small.
static const size_t POE_SIZES[] = { 5 * 1024, 10 * 1024 };   // both complete cleanly on the
// WROOM; ≥20 KB (or a 3rd sequential blob) resets it — RAM/fragmentation, not an interop
// fault, and moot for the deployed BRIDGE role (forwards, never originates large blobs).

#define POE_APP_NAME "trailcam_poe"
#define POE_ASPECT   "resource"

// Bumped to prove an OTA push actually replaced the firmware (visible in the boot banner).
#define POE_FW_VERSION "ota-1"
