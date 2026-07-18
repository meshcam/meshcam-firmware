#pragma once
/*
 * Leaf radio layer — persistent-identity RNS (Reticulum) over a bare SX1262.
 *
 * Every deep-sleep wake is a fresh boot, so the leaf re-inits the whole stack each time
 * it needs to talk. The one thing that MUST stay constant across wakes (and across power
 * loss / dead-battery) is the leaf's mesh identity — otherwise the gateway sees a brand
 * new node hundreds of times a day and the mesh never stabilizes. We persist the RNS
 * identity's private key in NVS (generate once on cold boot, reload every wake) so the
 * leaf's destination hash is stable forever. The RNS filesystem is mounted, never
 * reformatted per-wake (init(reformatOnFail=true) formats only on the first-ever boot).
 *
 * Radio bring-up (SX1262 + TCXO 1.8V + external RF switch) is the same config proven in
 * gate-b. Real over-the-air validation needs the 915 antenna + a second node.
 */
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Bring up filesystem + persistent identity + SX1262 + Reticulum. Idempotent per boot.
// Returns true if the SX1262 initialized. Prints the STABLE leaf destination hash.
bool leaf_radio_begin();

// Announce the leaf's destination on the air (real TX). `status` is a short app-data tag.
// leaf-0.12.0 (bug 5): app_data is "<status> p=<pir> c=<captures> f=<push_fails>[ vb=<V>]"
// — the health counters make a leaf that captures fine but fails every push visible
// remotely (the 07-15 outage looked like "quiet cam" for 21 h). Older gateways read the
// whole string as the status; gateway-0.8.0+ splits on the first space.
void leaf_radio_announce(const char* status);

// Stage the health counters the next announce(s) will carry (bug 5). Call once per wake
// before radio use; vbat_v = NAN when the board has no divider (bench Freenove).
void leaf_radio_set_health(uint32_t pir_wakes, uint32_t captures, uint32_t push_fails,
                           float vbat_v);

// Alert path: push the alert thumbnail to the gateway as an RNS::Resource, wrapped in the
// metadata ENVELOPE the gateway parses to upload under the leaf's real event id:
//   {"event_id":"...","captured_at":123,"camera":"<slug>","kind":"thumb"}\n<JPEG bytes>
// Falls back to an announce-only alert when no gateway hash is baked / the link fails.
bool leaf_radio_send_alert(const uint8_t* buf, size_t len,
                           const char* event_id, int64_t captured_at);

// Service RNS for ~`ms` to receive any queued gateway packets/commands, then return.
void leaf_radio_rx_window(uint32_t ms);

// Mesh command downlink: hold an RNS RX window for ~`ms`; any packet arriving on the
// leaf's destination is treated as one command JSON object (same shape as a serial
// `!TC CMD` body) and QUEUED, then executed via `handler` AFTER the window closes —
// handlers can run for minutes (a chunked full-res send) and must not re-enter
// reticulum.loop() from inside a packet callback.
#include "leaf_serial_proto.h"   // tc_cmd_handler
void leaf_radio_poll_commands(uint32_t ms, tc_cmd_handler handler);

// Push a stored full-res toward the gateway as SEQUENTIAL enveloped Resource chunks
// (<= ~16 KB each — the no-PSRAM gateway caps a single Resource at ~25 KB). The
// gateway reassembles by (event_id, kind=full) and uploads. Blocking: a QXGA original
// is ~8 chunks ≈ 13-15 min of airtime at bench goodput. Returns true if every chunk
// completed.
bool leaf_radio_send_full(const uint8_t* buf, size_t len, const char* event_id,
                          int64_t captured_at, const char* quality);

// --- ADR (adaptive radio profile, leaf-0.10.0) ------------------------------------------
// The gateway measures our announce SNR and grants profile switches by INDEX into the
// shared table (lib/lora_interface/lora_profiles.h) via {"kind":"radio_profile"} commands.
// Callable before leaf_radio_begin() (stages the params for radio init — how an
// RTC-persisted grant is applied on wake) or after (retunes the live SX1262 — how a fresh
// grant is applied mid-wake). Returns false for an unknown index / retune failure.
bool leaf_radio_set_profile(uint8_t idx);
const char* leaf_radio_profile_name();

// TX power (tx-power calibration, 2026-07-18): override the compile-time
// LORA_TX_DBM at runtime. Stage-or-live like set_profile(). The value in use is
// reported in every announce tail as "tx=<dbm>" so gateway-side telemetry
// self-describes which power each checkin was sent at.
bool leaf_radio_set_tx_dbm(int dbm);
int  leaf_radio_tx_dbm();

// True once THIS wake saw any proof the gateway can hear us (a link came up, or a mesh
// command/packet arrived). The ADR revert logic counts wakes without contact: N silent
// wakes on a non-base profile means the profiles may have diverged -> fall back to base.
bool leaf_radio_had_contact();
