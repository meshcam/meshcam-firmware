#pragma once
// Gate A test knobs. Keep both boards on identical radio params or they won't hear
// each other — these mirror microReticulum's LoRaInterface defaults (915 MHz US ISM).

#define GATE_A_APP_NAME   "trailcam_gatea"
#define GATE_A_ASPECT     "resource"

// Blob sizes to push, in order. Original plan was 5/50/250 KB; hardware said no:
// 35 KB+ std::bad_alloc's the no-PSRAM Heltec V3 sender in the link encrypt EVEN from
// a clean ~331 KB heap (the send path holds several full-size copies and needs large
// CONTIGUOUS blocks; upstream multi-segment send is an empty TODO stub, so a Resource
// is all-or-nothing in RAM). Measured ceiling 2026-07-03: 25600 PASSES, 35840 crashes.
// This ladder is the passing set; full-res (~110-140 KB QXGA) rides app-layer chunking
// at <=~20 KB per chunk instead.
static const size_t GATE_A_SIZES[] = { 5 * 1024, 15 * 1024, 25 * 1024 };

// CLIENT ONLY: the server prints its destination hash over serial at boot, e.g.
//   "SERVER destination hash: a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6"
// Paste those 32 hex chars (16 bytes, TRUNCATED_HASHLENGTH/8) here, then reflash the
// client. This manual coupling is fine for a 2-board bench test.
// NOTE the server identity is RANDOM PER BOOT (Identity() + filesystem.format() in
// reticulum_setup) — every server reflash/reset mints a new hash; re-capture from its
// boot log and rebake this before reflashing the client.
#define GATE_A_SERVER_DEST_HEX  "4c1024be2ed0e689f970d6faac1f555c"  // server Heltec 2026-07-02

// Seconds between client send attempts / path-request retries.
#define GATE_A_TICK_SECS  5

// Seconds between server announces while no link is up. Deliberately long: the radio
// is half-duplex and an announce TX that lands mid-handshake can eat the client's
// one-shot RTT packet (seen on hardware 2026-07-02). Transport also answers path
// requests with an announce on its own, so cold discovery doesn't depend on this.
#define GATE_A_ANNOUNCE_SECS  30
