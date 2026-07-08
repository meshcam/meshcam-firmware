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
struct TcMaintRequest {
    char     ssid[64] = "";
    char     psk[64]  = "";
    uint32_t minutes  = 10;
};
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
