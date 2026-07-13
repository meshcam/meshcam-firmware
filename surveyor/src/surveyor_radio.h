#pragma once
/*
 * Surveyor RNS radio — adapted from leaf-sleep-wake/src/leaf_radio.cpp (the Gate A-proven
 * Link + Resource flow), minus the camera/sleep machinery. The surveyor deliberately
 * announces as "trailcam.leaf": the gateway's announce handler, ADR grant machinery,
 * time-sync and command downlink are all keyed to that aspect, so looking like a leaf
 * buys the whole per-spot ADR record (best profile that closes) for free. Fine while a
 * mesh has no real leaf on the air at the same time (the range-walk case); revisit if
 * a walk ever happens over a live camera mesh.
 */
#include <Arduino.h>
#include <lora_profiles.h>
#include <stddef.h>
#include <stdint.h>

// Bring the stack up (filesystem, NVS identity, SX1276, Reticulum). Returns radio-ok.
bool sv_radio_begin();
bool sv_radio_ok();

// Announce our (persistent) identity with a short status string ("hello", "probe:12",
// "profile", "checkin"). The gateway's command/ack/time-sync delivery is
// announce-triggered, so every announce is also a downlink solicitation.
void sv_radio_announce(const char* status);

// Send ONE enveloped Resource (JSON header line + '\n' + body) to the baked gateway
// destination, establishing (or reusing) the link. Blocks to conclusion or timeout.
// On return, sv_last_* hold the surveyor-side RF numbers of the most recent frame.
bool sv_probe_send(const char* hdr_json, const uint8_t* body, size_t blen,
                   uint32_t timeout_ms);

// Close the gateway link (be a good peer between probe batches).
void sv_link_teardown();

// Pump RNS for `ms`, queueing any inbound packets addressed to us; each queued JSON is
// handed to `fn` AFTER the window (handlers may transmit/retune — never re-enter loop()).
typedef void (*sv_cmd_fn)(const char* json);
void sv_radio_poll_commands(uint32_t ms, sv_cmd_fn fn);

// Plain pump with no command dispatch (idle servicing).
void sv_radio_pump(uint32_t ms);

// ADR: retune (live) / stage. Both ends must match — only call from grant handling
// or the revert-to-base path.
bool        sv_radio_set_profile(uint8_t idx);
uint8_t     sv_radio_profile();
const char* sv_radio_profile_name();

// True if any frame addressed to us arrived since the last call site's mark (proof the
// gateway hears us); ms since ANY frame was demodulated is exposed for revert logic.
uint32_t sv_ms_since_rx();

// Surveyor-side RF numbers of the most recently received frame (NAN until first RX) —
// after a COMPLETE probe these are effectively "the gateway's proof, as heard here".
float sv_last_rssi();
float sv_last_snr();

// Mesh discovery (which gateway answered / was overheard): first 8 hex chars of the
// current top candidate for the UI, and whether any mesh has answered this boot.
const char* sv_gateway_short();
bool sv_gateway_known();
