#pragma once
/*
 * Leaf serial framing — emit the host-bridge protocol on the normal USB console.
 *
 * Until the LoRa mesh has a relay + gateway, a workstation plays gateway: it reads framed
 * captures off this console and uploads them to trailcam-dev. Spec + bridge:
 * prototype/trail-cam/host-bridge/README.md ; wider contract: docs/trailcam/ingest-api.md.
 *
 * These `!TC `-prefixed lines coexist with ordinary ESP_LOG/Serial output — the bridge
 * passes everything else through. No console reconfiguration needed.
 *
 *   !TC TLM {json}                         telemetry heartbeat (any wake, fire-and-forget)
 *   !TC EVT {json} / base64(jpeg) / !TC END <event_id> <crc32>    one capture frame
 *
 * event_id is minted ONCE per capture from RTC time + an RTC-memory counter and persisted
 * in RTC memory, so a retry (or the later full-res send) reuses the exact same id — the
 * ingest contract is idempotent per (event_id, kind).
 */
#include <Arduino.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// Optional telemetry fields; leave a field at its sentinel to omit it from the JSON.
struct LeafTelemetry {
    float       battery_v    = NAN;
    float       temp_c       = NAN;
    float       pressure_hpa = NAN;
    int32_t     rssi         = INT32_MIN;
    float       snr          = NAN;
    const char* boot_reason  = nullptr;
};

// Firmware version string, reported in telemetry and (with LEAF_BLE) in the BLE status
// characteristic. Here rather than in the .cpp so both report the same value.
#ifndef LEAF_FW_VERSION
#define LEAF_FW_VERSION "leaf-0.18.1"
#endif

// Wire identity: "leaf-" + the first 8 hex of the RNS destination hash, pushed in by the
// radio layer once per power cycle (RTC-persisted); a non-empty LEAF_NODE_SLUG build flag
// pins a fixed slug instead and makes tc_set_node_slug a no-op. Until either happens,
// tc_node_slug() returns "leaf-unidentified" (only reachable pre-radio on a cold boot).
const char* tc_node_slug();
void tc_set_node_slug(const char* slug);

// Begin a new capture event: increments the RTC-memory counter and mints the id from
// `unixtime` (the leaf's clock at PIR trigger; 0 if unsynced) + that counter. Returns the
// id, also retrievable via tc_current_event_id() for a later full-res send of the same event.
const char* tc_new_event(int64_t unixtime);
const char* tc_current_event_id();

// One telemetry heartbeat line.
void tc_emit_telemetry(const LeafTelemetry& t);

// One capture frame: EVT header + base64 body + END(id, crc32-of-jpeg). `captured_unixtime`
// stamps captured_at, which the ingest contract REQUIRES — callers must pass a seeded
// clock (build-time epoch until gateway time sync; see build_epoch() in main.cpp).
void tc_emit_capture(const char* event_id, int64_t captured_unixtime,
                     const uint8_t* jpeg, size_t len, const char* kind,
                     const LeafTelemetry& meta);

// Downlink: read `!TC CMD {json}` lines from the console for up to `window_ms` and invoke
// `handler(kind, event_id, quality)` per command (event_id/quality may be "" when absent —
// quality comes from the payload object, e.g. {"quality":"standard"|"max"}). The bridge
// polls the app every 30 s and writes commands blind to our sleep state, so delivery only
// lands when a write coincides with an awake window — the server re-delivers until the
// command is satisfied, so missed windows self-heal. Unknown kinds are the HANDLER's job
// to ignore (forward compat). Non-CMD lines in the window are discarded (the console RX
// side carries no other traffic).
typedef void (*tc_cmd_handler)(const char* kind, const char* event_id, const char* quality);
void tc_poll_commands(uint32_t window_ms, tc_cmd_handler handler);

// Parse ONE command JSON object (the {...} the server queues — same shape whether it
// arrived as a serial `!TC CMD` line or as a mesh packet body) and invoke the handler.
// Also stashes update_firmware payloads for tc_take_ota_request(). Transport-agnostic
// so the mesh downlink reuses the exact serial semantics.
void tc_handle_cmd_json(const char* json, tc_cmd_handler handler);

// kind=update_firmware carries more payload than the generic handler signature exposes:
//   !TC CMD {"kind":"update_firmware","payload":{"ssid":"...","psk":"...",
//            "url":"http(s)://.../firmware.bin","sha256":"<64 hex>"}}
// The scanner stashes those fields as a side effect; when the handler sees the kind it
// collects them with tc_take_ota_request() (one-shot: cleared on take) and hands them to
// leaf_ota_run(). Fields are sized for worst-case home-network reality, not elegance.
struct TcOtaRequest {
    char ssid[64]   = "";
    char psk[64]    = "";
    char url[192]   = "";
    char sha256[65] = "";
};
bool tc_take_ota_request(TcOtaRequest* out);

// kind=maintenance — "stay awake and serviceable" (2026-07-03). The leaf skips the next
// deep-sleep and instead sits in a loop with BOTH command channels open (serial + mesh),
// optionally on WiFi, for `minutes`:
//   !TC CMD {"kind":"maintenance","payload":{"ssid":"...","psk":"...","minutes":15}}
// ssid/psk absent -> no WiFi, just awake (serial servicing). With WiFi the leaf also
// NTP-syncs its clock (real captured_at timestamps until the next full power loss).
// {"kind":"sleep"} during the window ends it early. Same one-shot stash pattern as OTA.
// `seconds` overrides `minutes` when non-zero. The walk-up service window
// wants TENS OF SECONDS on a wake that already happened, and minute granularity cannot
// express that: the shortest window `minutes` can ask for is 60 s, which is ~0.25 mAh of
// BLE advertising against the ~0.5-1 mAh a whole capture event costs. Operator-requested
// maintenance still speaks in minutes and is unaffected (seconds defaults to 0).
struct TcMaintRequest {
    char     ssid[64] = "";
    char     psk[64]  = "";
    uint32_t minutes  = 10;
    uint32_t seconds  = 0;
};

// Window length in milliseconds, seconds-first. One place so the auto walk-up window and
// an operator request cannot drift apart.
static inline uint32_t tc_maint_window_ms(const TcMaintRequest& r) {
    return r.seconds ? r.seconds * 1000UL : r.minutes * 60000UL;
}
bool tc_take_maint_request(TcMaintRequest* out);

// kind=time_sync — the gateway is the mesh's clock authority (it has WiFi + NTP) and
// pushes {"kind":"time_sync","payload":{"unix":<epoch seconds>}} into the RX window on
// every cold-boot "hello" announce (the moment the leaf's clock has just reseeded from
// its build epoch) and periodically for drift. Same one-shot stash pattern as OTA.
bool tc_take_time_sync(long long* out_unix);

// kind=radio_profile — ADR grant from the gateway (leaf-0.10.0):
//   {"kind":"radio_profile","payload":{"idx":2,"ttl_s":86400}}
// idx indexes the shared profile table (lib/lora_interface/lora_profiles.h); ttl_s is
// how long the grant stays valid without a refresh (0/absent = the firmware default).
// The handler retunes the live radio, persists the grant in RTC memory, and confirms
// with an announce ON THE NEW PROFILE — the gateway reverts if that never arrives.
// Same one-shot stash pattern as OTA.
bool tc_take_radio_profile(uint32_t* out_idx, uint32_t* out_ttl_s);

// kind=format_card — the operator says "yes, erase the card I can't read":
//   {"kind":"format_card","payload":{"confirm":"erase"}}
// The confirm token is required and is not decoration: this is the one command that
// destroys data, commands are re-delivered until acked, and a bare kind is too easy to
// send by accident from a shell. A command without it is parsed, logged and dropped.
// The real safety gate is in leaf_store_format_card(), which refuses any card it can
// actually read; this is the second lock, not the only one. Same one-shot stash pattern.
bool tc_take_format_card();
