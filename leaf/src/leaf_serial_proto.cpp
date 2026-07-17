#include "leaf_serial_proto.h"

#include <string.h>
#include <time.h>
#include "esp_attr.h"           // RTC_DATA_ATTR
#include "esp_heap_caps.h"
#include "esp_system.h"         // esp_random (per-boot event-id salt)
#include "mbedtls/base64.h"

// Wire identity. Default (empty) = derived at radio init: "leaf-" + the first 8 hex of
// the RNS destination hash (homelab docs/trailcam/node-identity.md — firmware transmits
// a machine id, human names live only in the app). A non-empty LEAF_NODE_SLUG build flag
// pins a fixed slug instead (bench debugging only).
#ifndef LEAF_NODE_SLUG
#define LEAF_NODE_SLUG ""
#endif
#ifndef LEAF_FW_VERSION
#define LEAF_FW_VERSION "leaf-0.15.1"
#endif

// RTC-persisted: PIR wakes mint event ids and emit the serial EVT BEFORE the radio comes
// up, so they use the slug derived on the preceding cold boot (a PIR wake can never be
// the first wake of a power cycle). Power loss wipes it; the cold-boot path re-derives
// at radio init, before anything transmits.
static RTC_DATA_ATTR char s_node_slug[24] = LEAF_NODE_SLUG;

const char* tc_node_slug() { return s_node_slug[0] ? s_node_slug : "leaf-unidentified"; }

void tc_set_node_slug(const char* slug) {
    if (sizeof(LEAF_NODE_SLUG) > 1) return;   // build-flag override pins the slug
    if (slug && slug[0]) snprintf(s_node_slug, sizeof(s_node_slug), "%s", slug);
}

// Capture counter + the current event's id, both in RTC slow memory so they survive deep
// sleep (the normal between-events state). A full power loss resets them.
static RTC_DATA_ATTR uint32_t s_event_seq = 0;
static RTC_DATA_ATTR char     s_event_id[64] = {0};
// Per-power-cycle ID salt (bug 3, 2026-07-16): the clock reseeds from the build epoch
// on every power loss, so <slug>-<unixtime>-<seq> repeated identical IDs across
// power-cycles (c3-1783141499-3 was minted 07-04 AND again 07-16) and server-side
// dedup silently ate the later captures. RTC memory survives deep sleep and dies with
// power — exactly the reuse boundary — so one random token per power cycle makes IDs
// unique even on a build-epoch clock. 0 = unseeded (mint on first use).
static RTC_DATA_ATTR uint16_t s_boot_salt = 0;

// zlib-compatible CRC-32 (matches Python zlib.crc32 / binascii.crc32 that the bridge uses).
static uint32_t crc32_zlib(const uint8_t* p, size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return ~crc;
}

const char* tc_new_event(int64_t unixtime) {
    if (s_boot_salt == 0) s_boot_salt = (uint16_t)(esp_random() | 1);   // never 0
    s_event_seq++;
    const long long ts = (unixtime > 0) ? (long long)unixtime : 0;
    // Full slug as the prefix: with derived machine ids ("leaf-9e0d2930") the event id
    // doubles as node attribution, and the old truncate-at-first-'-' rule would collapse
    // every leaf to "leaf".
    snprintf(s_event_id, sizeof(s_event_id), "%s-%lld-%04x-%lu",
             tc_node_slug(), ts, (unsigned)s_boot_salt, (unsigned long)s_event_seq);
    return s_event_id;
}

const char* tc_current_event_id() { return s_event_id; }

void tc_emit_telemetry(const LeafTelemetry& t) {
    char j[320];
    int o = snprintf(j, sizeof(j), "{\"node\":\"%s\",\"kind\":\"camera\"", tc_node_slug());
    if (!isnan(t.battery_v))    o += snprintf(j + o, sizeof(j) - o, ",\"battery_v\":%.2f", t.battery_v);
    if (!isnan(t.temp_c))       o += snprintf(j + o, sizeof(j) - o, ",\"temp_c\":%.1f", t.temp_c);
    if (!isnan(t.pressure_hpa)) o += snprintf(j + o, sizeof(j) - o, ",\"pressure_hpa\":%.1f", t.pressure_hpa);
    if (t.rssi != INT32_MIN)    o += snprintf(j + o, sizeof(j) - o, ",\"rssi\":%ld", (long)t.rssi);
    if (!isnan(t.snr))          o += snprintf(j + o, sizeof(j) - o, ",\"snr\":%.1f", t.snr);
    if (t.boot_reason)          o += snprintf(j + o, sizeof(j) - o, ",\"boot_reason\":\"%s\"", t.boot_reason);
    snprintf(j + o, sizeof(j) - o, ",\"fw_version\":\"%s\"}", LEAF_FW_VERSION);
    Serial.printf("!TC TLM %s\n", j);
}

// Build the optional meta object from whatever telemetry we have; returns true if non-empty.
static bool build_meta(const LeafTelemetry& m, char* out, size_t cap) {
    int o = snprintf(out, cap, "{");
    bool any = false;
    if (!isnan(m.battery_v)) { o += snprintf(out + o, cap - o, "%s\"battery_v\":%.2f", any ? "," : "", m.battery_v); any = true; }
    if (m.rssi != INT32_MIN) { o += snprintf(out + o, cap - o, "%s\"rssi\":%ld", any ? "," : "", (long)m.rssi); any = true; }
    if (!isnan(m.snr))       { o += snprintf(out + o, cap - o, "%s\"snr\":%.1f", any ? "," : "", m.snr); any = true; }
    if (!isnan(m.temp_c))    { o += snprintf(out + o, cap - o, "%s\"temp_c\":%.1f", any ? "," : "", m.temp_c); any = true; }
    snprintf(out + o, cap - o, "}");
    return any;
}

void tc_emit_capture(const char* event_id, int64_t captured_unixtime,
                     const uint8_t* jpeg, size_t len, const char* kind,
                     const LeafTelemetry& meta) {
    if (!jpeg || !len) return;

    // --- EVT header ---
    char hdr[288];
    int o = snprintf(hdr, sizeof(hdr), "{\"camera\":\"%s\",\"event_id\":\"%s\"",
                     tc_node_slug(), event_id);
    // captured_at is REQUIRED by the ingest contract (the bridge rejects frames without
    // it) — the caller guarantees a seeded clock (build-time epoch until gateway sync).
    {
        time_t tt = (time_t)(captured_unixtime > 0 ? captured_unixtime : 0);
        struct tm tmv; gmtime_r(&tt, &tmv);
        char iso[24]; strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &tmv);
        o += snprintf(hdr + o, sizeof(hdr) - o, ",\"captured_at\":\"%s\"", iso);
    }
    o += snprintf(hdr + o, sizeof(hdr) - o, ",\"kind\":\"%s\",\"len\":%u",
                  kind ? kind : "thumb", (unsigned)len);
    char m[160];
    if (build_meta(meta, m, sizeof(m))) o += snprintf(hdr + o, sizeof(hdr) - o, ",\"meta\":%s", m);
    snprintf(hdr + o, sizeof(hdr) - o, "}");
    Serial.printf("!TC EVT %s\n", hdr);

    // --- base64 body (wrapped; the bridge concatenates all lines up to END) ---
    const size_t b64cap = 4 * ((len + 2) / 3) + 1;
    uint8_t* b64 = (uint8_t*)heap_caps_malloc(b64cap, MALLOC_CAP_SPIRAM);
    if (!b64) { Serial.println("; [tc] base64 buffer alloc failed, dropping frame"); return; }
    size_t olen = 0;
    if (mbedtls_base64_encode(b64, b64cap, &olen, jpeg, len) != 0) {
        Serial.println("; [tc] base64 encode failed, dropping frame");
        heap_caps_free(b64);
        return;
    }
    const size_t W = 100;
    for (size_t i = 0; i < olen; i += W) {
        size_t n = (olen - i < W) ? (olen - i) : W;
        Serial.write(b64 + i, n);
        Serial.write('\n');
    }
    heap_caps_free(b64);

    // --- END trailer: event_id + crc32 of the decoded JPEG bytes ---
    Serial.printf("!TC END %s %08x\n", event_id, (unsigned)crc32_zlib(jpeg, len));
    Serial.flush();
}

// Extract a string value ("key":"value", optional whitespace after the colon) from flat
// server-generated JSON. Enough for the bridge's json.dumps output — no nesting, no
// escaped quotes in the fields we read (slugs and event ids).
static bool json_str(const char* json, const char* key, char* out, size_t cap) {
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) out[i++] = *p++;
    out[i] = '\0';
    return *p == '"';
}

// Extract a bare unsigned number ("key": 15) — companion to json_str for the few
// numeric payload fields (maintenance minutes).
static bool json_u32(const char* json, const char* key, uint32_t* out) {
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    if (*p < '0' || *p > '9') return false;
    *out = (uint32_t)strtoul(p, nullptr, 10);
    return true;
}

// One-shot stash for the extra update_firmware payload fields (see header).
static TcOtaRequest s_ota_req;
static bool         s_ota_pending = false;

// Same pattern for maintenance requests.
static TcMaintRequest s_maint_req;
static bool           s_maint_pending = false;

// ...and for time syncs.
static long long s_time_sync_unix    = 0;
static bool      s_time_sync_pending = false;

// ...and for ADR radio-profile grants.
static uint32_t s_rf_idx         = 0;
static uint32_t s_rf_ttl_s       = 0;
static bool     s_rf_pending     = false;

bool tc_take_radio_profile(uint32_t* out_idx, uint32_t* out_ttl_s) {
    if (!s_rf_pending) return false;
    if (out_idx)   *out_idx   = s_rf_idx;
    if (out_ttl_s) *out_ttl_s = s_rf_ttl_s;
    s_rf_pending = false;
    return true;
}

bool tc_take_time_sync(long long* out_unix) {
    if (!s_time_sync_pending) return false;
    if (out_unix) *out_unix = s_time_sync_unix;
    s_time_sync_pending = false;
    return true;
}

bool tc_take_maint_request(TcMaintRequest* out) {
    if (!s_maint_pending) return false;
    if (out) *out = s_maint_req;
    s_maint_pending = false;
    s_maint_req = TcMaintRequest();   // don't keep the psk around longer than needed
    return true;
}

bool tc_take_ota_request(TcOtaRequest* out) {
    if (!s_ota_pending) return false;
    if (out) *out = s_ota_req;
    s_ota_pending = false;
    s_ota_req = TcOtaRequest();   // don't keep the psk around longer than needed
    return true;
}

void tc_handle_cmd_json(const char* json, tc_cmd_handler handler) {
    char kind[32] = "", eid[48] = "", quality[16] = "";
    json_str(json, "kind", kind, sizeof(kind));
    json_str(json, "event_id", eid, sizeof(eid));
    json_str(json, "quality", quality, sizeof(quality));  // from payload{}
    if (strcmp(kind, "update_firmware") == 0) {
        s_ota_req = TcOtaRequest();
        json_str(json, "ssid",   s_ota_req.ssid,   sizeof(s_ota_req.ssid));
        json_str(json, "psk",    s_ota_req.psk,    sizeof(s_ota_req.psk));
        json_str(json, "url",    s_ota_req.url,    sizeof(s_ota_req.url));
        json_str(json, "sha256", s_ota_req.sha256, sizeof(s_ota_req.sha256));
        s_ota_pending = s_ota_req.ssid[0] && s_ota_req.url[0];
    }
    if (strcmp(kind, "maintenance") == 0) {
        s_maint_req = TcMaintRequest();
        json_str(json, "ssid", s_maint_req.ssid, sizeof(s_maint_req.ssid));
        json_str(json, "psk",  s_maint_req.psk,  sizeof(s_maint_req.psk));
        json_u32(json, "minutes", &s_maint_req.minutes);
        if (s_maint_req.minutes < 1)   s_maint_req.minutes = 1;
        if (s_maint_req.minutes > 120) s_maint_req.minutes = 120;
        s_maint_pending = true;   // WiFi-less maintenance (awake only) is valid
    }
    if (strcmp(kind, "time_sync") == 0) {
        uint32_t unix32 = 0;   // epoch seconds fit uint32 until 2106
        if (json_u32(json, "unix", &unix32) && unix32 > 1700000000u) {
            s_time_sync_unix    = (long long)unix32;
            s_time_sync_pending = true;
        }
    }
    if (strcmp(kind, "radio_profile") == 0) {
        uint32_t idx = 0, ttl = 0;
        if (json_u32(json, "idx", &idx)) {   // "idx":0 is a valid grant (base/far)
            json_u32(json, "ttl_s", &ttl);
            s_rf_idx     = idx;
            s_rf_ttl_s   = ttl;
            s_rf_pending = true;
        }
    }
    Serial.printf("[tc] CMD received: kind=%s event_id=%s quality=%s\n",
                  kind, eid, quality[0] ? quality : "(absent)");
    if (handler) handler(kind, eid, quality);
}

void tc_poll_commands(uint32_t window_ms, tc_cmd_handler handler) {
    // 768: an update_firmware command (ssid+psk+url+sha256) runs ~400 chars of JSON.
    static char line[768];
    size_t n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < window_ms) {
        int c = Serial.read();
        if (c < 0) { delay(5); continue; }
        if (c != '\n' && c != '\r') {
            if (n + 1 < sizeof(line)) line[n++] = (char)c;
            continue;
        }
        line[n] = '\0';
        if (n >= 8 && strncmp(line, "!TC CMD ", 8) == 0) {
            tc_handle_cmd_json(line + 8, handler);
            // The bridge delivers its whole queue as a burst, and a slow handler (live
            // capture ~5 s) would otherwise eat the window before the next line is read.
            // Grant a fresh window per handled command — bounded by the server's small
            // queue, and buffered lines drain in milliseconds.
            t0 = millis();
        }
        n = 0;
    }
}
