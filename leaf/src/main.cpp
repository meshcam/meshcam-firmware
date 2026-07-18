/*
 * Trail-cam LEAF — power skeleton: deep-sleep + dual wake (PIR ext0 + timer).
 *
 * This is the architectural heart of the battery/solar leaf. The MCU spends ~all
 * its life in deep sleep (~tens of uA) and wakes on one of two sources:
 *
 *   1. PIR (AM312 on GPIO21, ext0)  -> a real capture event. ALWAYS instant, hardware
 *      interrupt; the check-in schedule below NEVER delays a capture.
 *   2. Timer (adaptive interval)    -> a proactive check-in: brief RX window to pull
 *      any queued gateway commands (e.g. "send hi-res of image N"), heartbeat, and
 *      relay. The interval is what the adaptive season/time-of-day schedule controls.
 *
 * Deep sleep resets the MCU, so every wake is a fresh boot (stateless -> the real leaf
 * re-inits RNS and auto-rejoins the mesh; see docs/trailcam/design.md). State that must
 * survive a wake lives in RTC_DATA_ATTR (RTC slow memory, retained through deep sleep).
 *
 * This skeleton proves the WAKE MECHANICS on real hardware (the PIR is wired to GPIO21).
 * Capture / detect / thumbnail / radio-TX are stubbed hooks to fill in next.
 *
 * Pins/roles per the leaf GPIO budget (docs + gate-b): GPIO21 = PIR ext0 wake.
 */
#include <Arduino.h>
#include <WiFi.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"        // rtc_gpio_deinit: hand the PIR pad back to the GPIO matrix
#include "leaf_rails.h"
#include "leaf_env.h"
#include "esp_heap_caps.h"
#include "esp_system.h"          // esp_reset_reason
#include "esp_ota_ops.h"         // esp_ota_mark_app_valid_cancel_rollback
#include "leaf_camera.h"
#include "leaf_serial_proto.h"
#include "leaf_ota.h"
#include "leaf_store.h"
#ifdef LEAF_RADIO
#include "leaf_radio.h"
#endif
#ifdef LEAF_DETECTOR
#include "leaf_detector.h"
#endif

#ifndef LEAF_PIR_GPIO
#define LEAF_PIR_GPIO 21          // AM312 OUT, active-HIGH; RTC-capable (0-21) for ext0
#endif

// Console baud: a quality=max QXGA frame is ~300 KB -> ~410 KB base64, and the bridge
// drops any frame that doesn't END within 30 s of its EVT header. 115200 (~11.5 KB/s)
// would take ~36 s; 460800 streams it in ~9 s. The CH343 USB-UART handles this fine —
// match it on the reader side (socat FILE:...,b460800 / monitor_speed).
#ifndef LEAF_BAUD
#define LEAF_BAUD 460800
#endif
static const gpio_num_t PIR_GPIO = (gpio_num_t)LEAF_PIR_GPIO;

// --- state retained across deep sleep (RTC slow memory) ------------------------------
RTC_DATA_ATTR uint32_t g_boot_count   = 0;   // total wakes (incl. cold boot)
RTC_DATA_ATTR uint32_t g_pir_wakes    = 0;   // capture events
RTC_DATA_ATTR uint32_t g_timer_wakes  = 0;   // scheduled check-ins
RTC_DATA_ATTR uint32_t g_captures     = 0;   // successful camera captures (bug 5)
RTC_DATA_ATTR uint32_t g_push_fails   = 0;   // alert pushes that failed (bug 5): the
                                             // counters ride every announce's app_data
                                             // so "capturing but can't push" is visible
                                             // remotely (07-15: 21 h of failed pushes
                                             // looked like a quiet cam)
RTC_DATA_ATTR int64_t  g_unixtime     = 0;   // approx wall clock; 0 = unknown until the
                                             // gateway syncs it (piggybacked on an ACK).
RTC_DATA_ATTR uint32_t g_last_sleep_s = 0;   // how long we intended to sleep last time

// ADR (adaptive radio profile, leaf-0.10.0): the gateway grants profile switches by
// index into the shared table (lib/lora_interface/lora_profiles.h). The grant lives in
// RTC memory so every wake comes up already tuned to it — and a power loss wipes it,
// which IS the fallback: cold boot -> base profile -> "hello" -> the gateway re-grants.
// Kept as plain numbers here (not lora_profiles.h) so non-radio builds still compile;
// LEAF_RF_BASE_PROFILE must equal LORA_PROFILE_BASE.
#ifndef LEAF_RF_BASE_PROFILE
#define LEAF_RF_BASE_PROFILE 1
#endif
RTC_DATA_ATTR uint8_t g_rf_profile      = LEAF_RF_BASE_PROFILE;
RTC_DATA_ATTR int64_t g_rf_grant_until  = 0;   // unix deadline; 0 = no grant active
RTC_DATA_ATTR uint8_t g_rf_silent_wakes = 0;   // consecutive wakes w/o gateway contact

// --- adaptive check-in interval ------------------------------------------------------
// Governs ONLY the proactive timer check-ins (command latency), never PIR captures.
// Deer season (Oct-Nov) daytime = frequent for low alert latency; 2am = sparse.
static uint32_t checkin_interval_s(bool time_known, int month, int hour) {
#ifdef LEAF_TEST_INTERVAL_S
    return LEAF_TEST_INTERVAL_S;             // bench override: short, to watch timer wakes
#endif
    if (!time_known) return 900;             // no time sync yet -> safe 15 min middle
    const bool deer   = (month == 10 || month == 11);      // Oct-Nov
    const bool active = (hour >= 6 && hour < 20);           // daylight-ish
    if (deer)  return active ? 300  : 1800;   // 5 min day / 30 min night
    else       return active ? 1800 : 7200;   // 30 min day / 2 h night
}

// --- pipeline pieces the hooks call --------------------------------------------------
// Decide whether a captured frame is worth the radio energy. With LEAF_DETECTOR this runs
// the real INT8 person_detection model (leaf_detector.*); the person score is always
// logged. Send policy:
//   LEAF_DETECT_GATE set  -> gate on the verdict (skip frames with no subject). Use on the
//                            bench to SEE the model work; NOT safe as an animal gate yet
//                            (a person-only model reads a deer as "no person").
//   LEAF_DETECT_GATE unset-> always send, detector score is metadata only (safe default
//                            until a multi-class deer/person model exists).
static bool detector_says_send(const uint8_t* jpeg, size_t len, int w, int h) {
#ifdef LEAF_DETECTOR
    LeafDetection d = leaf_detector_run(jpeg, len, w, h);
    if (d.ok) {
        Serial.printf("[leaf] detector: person=%.2f (%s) in %ums\n",
                      d.score, d.subject ? "SUBJECT" : "no subject", (unsigned)d.infer_ms);
    } else {
        Serial.println("[leaf] detector: inference unavailable -> send anyway");
    }
#ifdef LEAF_DETECT_GATE
    if (d.ok) return d.subject;
#endif
    return true;
#else
    (void)jpeg; (void)len; (void)w; (void)h;
    return true;   // no detector compiled in -> send everything
#endif
}

// Radio TX of an alert thumbnail. With LEAF_RADIO the real SX1262 + microReticulum stack
// (leaf_radio.*) handles it; otherwise log what would go out (camera-only bench builds).
// Returns false when the push failed (feeds the g_push_fails health counter, bug 5).
static bool radio_send_alert(const uint8_t* buf, size_t len,
                             const char* event_id, int64_t captured_at) {
#ifdef LEAF_RADIO
    return leaf_radio_send_alert(buf, len, event_id, captured_at);
#else
    (void)buf; (void)event_id; (void)captured_at;
    Serial.printf("[leaf] radio -> (stub) would TX alert thumbnail, %u bytes\n",
                  (unsigned)len);
    return true;   // no radio compiled in: nothing to fail
#endif
}

// Gateway command handler (downlink, via the bridge's !TC CMD lines). fetch_full: re-send
// the stored full-res for that event_id as an EVT kind=full — the ingest auto-completes
// the command server-side, repeats are idempotent. Unknown kinds ignored (forward compat).
//
// Quality tiers (feature request 2026-07-02): the store holds the SENSOR-MAX original
// (QXGA). payload quality "max" — or absent, the legacy form — sends it as-is;
// "standard" (and any unknown value) downscales the stored original to ~800x600 first.
// Either way it's a transform of the SAME stored frame — never a re-capture.
// Resolve a fetch_full to the right tier's bytes from the STORED original — never a
// re-capture (bug 2026-07-02, dev photo 93487506). Returns a malloc-family buffer
// (free() it — esp-idf's free() handles both malloc and heap_caps_malloc) or nullptr
// if the event isn't stored. *out_capts is the best capture-time estimate.
static uint8_t* resolve_full_bytes(const char* event_id, const char* quality,
                                   size_t* out_len, int64_t* out_capts) {
    size_t len = 0;
    int64_t mtime = 0;
    uint8_t* buf = leaf_store_load(event_id, &len, &mtime);
    if (!buf) {
        Serial.printf("[leaf] fetch_full %s: not in store -> ignored (expires server-side)\n",
                      event_id);
        return nullptr;
    }
    if (out_capts) *out_capts = mtime > 0 ? mtime : g_unixtime;

    const bool want_max = (quality[0] == '\0') || (strcmp(quality, "max") == 0);
    if (!want_max) {
        size_t std_len = 0;
        uint8_t* std_jpg = leaf_jpeg_downscale(buf, len, &std_len);
        if (std_jpg && std_len < len) {
            Serial.printf("[leaf] fetch_full %s: standard downscale (%u -> %u bytes)\n",
                          event_id, (unsigned)len, (unsigned)std_len);
            heap_caps_free(buf);
            *out_len = std_len;
            return std_jpg;
        }
        if (std_jpg) free(std_jpg);   // downscale not smaller -> the original wins
        Serial.printf("[leaf] fetch_full %s: downscale unavailable -> original as-is\n", event_id);
    } else {
        Serial.printf("[leaf] fetch_full %s: stored ORIGINAL (%u bytes, quality=max)\n",
                      event_id, (unsigned)len);
    }
    *out_len = len;
    return buf;
}

static void handle_update_firmware() {
    // Operator-queued OTA: the parser stashed ssid/psk/url/sha256 from the payload.
    // Success reboots into the new image; failure logs and falls through to sleep.
    TcOtaRequest req;
    if (tc_take_ota_request(&req)) leaf_ota_run(req);
    else Serial.println("[leaf] update_firmware without usable payload -> ignored");
}

// Maintenance-mode flags (2026-07-03): "maintenance" arms a stay-awake service window
// after the normal wake work (see maintenance_loop); "sleep" ends that window early.
// Flags, not immediate action, because the command lands mid-poll and the loop decision
// belongs to setup()'s tail.
static bool g_maint_exit = false;

// Gateway time sync (2026-07-03): set the SYSTEM clock, not just g_unixtime — the
// ESP32-S3 RTC timer keeps time(nullptr) ticking through deep sleep, so one sync makes
// every later wake's clock exact (PIR wakes included, whose slept portion the interval
// arithmetic can't know) until the next full power loss.
static void handle_time_sync() {
    long long unix_now = 0;
    if (!tc_take_time_sync(&unix_now)) return;
    const long long before = (long long)g_unixtime;
    struct timeval tv = {};
    tv.tv_sec = (time_t)unix_now;
    settimeofday(&tv, nullptr);
    g_unixtime = unix_now;
    Serial.printf("[leaf] time sync: %lld -> %lld (drift %+lld s)\n",
                  before, unix_now, unix_now - before);
}

// ADR grant handler + wake-time revert logic. The safety story: every failure mode
// converges both ends back to the base profile — the leaf reverts on grant expiry or
// N contactless wakes (profiles diverged?), power loss wipes the RTC grant outright,
// and the gateway independently falls back when it stops hearing us. Neither side
// needs the other's cooperation to get home.
#ifdef LEAF_RADIO
#define LEAF_RF_GRANT_TTL_S      (24 * 3600)   // grant life without a gateway refresh
#define LEAF_RF_SILENT_WAKES_MAX 3

static void handle_radio_profile() {
    uint32_t idx = 0, ttl = 0;
    if (!tc_take_radio_profile(&idx, &ttl)) return;
    const bool changed = (uint8_t)idx != g_rf_profile;
    if (!leaf_radio_set_profile((uint8_t)idx)) {   // retunes the LIVE radio
        Serial.printf("[leaf] radio_profile idx=%u REJECTED (unknown idx / retune failed)\n",
                      (unsigned)idx);
        return;
    }
    g_rf_profile      = (uint8_t)idx;
    g_rf_silent_wakes = 0;
    g_rf_grant_until  = g_unixtime + (int64_t)(ttl ? ttl : LEAF_RF_GRANT_TTL_S);
    if (!changed) {
        // Keepalive refresh (the gateway re-sends the standing grant every couple of
        // announces): TTL + contact counter re-armed above, nothing to confirm.
        Serial.printf("[leaf] radio_profile: keepalive on %s\n", leaf_radio_profile_name());
        return;
    }
    Serial.printf("[leaf] radio_profile: now on %s (idx %u, ttl %lus) -> confirm announce\n",
                  leaf_radio_profile_name(), (unsigned)idx,
                  (unsigned long)(ttl ? ttl : LEAF_RF_GRANT_TTL_S));
    // Confirm ON THE NEW PROFILE — the gateway retuned right after sending the grant
    // and reverts unless it hears us there.
    leaf_radio_announce("profile");
}

// Before any radio use this wake: decide base-vs-granted, stage the choice for
// leaf_radio_begin(). Runs even on PIR wakes (the thumb push must already be tuned).
static void rf_profile_wake_begin() {
    if (g_rf_profile == LEAF_RF_BASE_PROFILE) return;
    const char* revert = nullptr;
    if (g_rf_silent_wakes >= LEAF_RF_SILENT_WAKES_MAX) revert = "no gateway contact";
    else if (g_rf_grant_until && g_unixtime > g_rf_grant_until) revert = "grant expired";
    else if (!leaf_radio_set_profile(g_rf_profile)) revert = "bad stored profile";
    if (revert) {
        Serial.printf("[leaf] ADR: reverting to base profile (%s)\n", revert);
        g_rf_profile      = LEAF_RF_BASE_PROFILE;
        g_rf_grant_until  = 0;
        g_rf_silent_wakes = 0;
        return;   // base is what leaf_radio_begin() uses by default
    }
    Serial.printf("[leaf] ADR: waking on granted profile %s (silent wakes %u/%u)\n",
                  leaf_radio_profile_name(), (unsigned)g_rf_silent_wakes,
                  (unsigned)LEAF_RF_SILENT_WAKES_MAX);
}

#ifdef LEAF_TX_SWEEP
// TX-power sweep (calibration, 2026-07-18): cycle output power across wakes so one
// soak maps gateway-side RSSI vs TX dBm with the levels interleaved — slow fading
// hits every level equally instead of biasing whichever ran last. The announce tail's
// "tx=<dbm>" makes each checkin self-describing in telemetry. Bench flag only; field
// builds pin one power via -DLORA_TX_DBM.
RTC_DATA_ATTR uint8_t g_tx_sweep_idx = 0;
static void tx_sweep_wake_begin() {
    static const int SWEEP[] = {10, 14, 17, 20, 22};
    const int dbm = SWEEP[g_tx_sweep_idx % (sizeof(SWEEP) / sizeof(SWEEP[0]))];
    g_tx_sweep_idx++;
    leaf_radio_set_tx_dbm(dbm);   // pre-begin: stages for radio init
    Serial.printf("[leaf] TX sweep: %d dBm this wake\n", dbm);
}
#endif
#endif

static void on_gateway_command(const char* kind, const char* event_id, const char* quality) {
    if (strcmp(kind, "update_firmware") == 0) { handle_update_firmware(); return; }
    if (strcmp(kind, "maintenance") == 0) return;   // stashed by the parser; setup() tail acts
    if (strcmp(kind, "time_sync") == 0) { handle_time_sync(); return; }
    if (strcmp(kind, "sleep") == 0) { g_maint_exit = true; return; }
    if (strcmp(kind, "radio_profile") == 0) {
#ifdef LEAF_RADIO
        handle_radio_profile();
#endif
        return;
    }
    if (strcmp(kind, "fetch_full") != 0) {
        Serial.printf("[leaf] CMD kind \"%s\" unknown -> ignored (forward compat)\n", kind);
        return;
    }
    size_t len = 0;
    int64_t capts = 0;
    uint8_t* buf = resolve_full_bytes(event_id, quality, &len, &capts);
    if (!buf) return;
    LeafTelemetry meta;
    tc_emit_capture(event_id, capts, buf, len, "full", meta);   // serial -> bridge uploads
    free(buf);
}

#ifdef LEAF_RADIO
// Same commands arriving over the MESH (gateway packet -> queued -> here). fetch_full
// answers over the mesh too: chunked Resources the gateway reassembles and uploads.
static void on_mesh_command(const char* kind, const char* event_id, const char* quality) {
    if (strcmp(kind, "update_firmware") == 0) { handle_update_firmware(); return; }
    if (strcmp(kind, "maintenance") == 0) return;   // stashed by the parser; setup() tail acts
    if (strcmp(kind, "time_sync") == 0) { handle_time_sync(); return; }
    if (strcmp(kind, "sleep") == 0) { g_maint_exit = true; return; }
    if (strcmp(kind, "radio_profile") == 0) { handle_radio_profile(); return; }
    if (strcmp(kind, "fetch_full") != 0) {
        Serial.printf("[leaf] mesh CMD kind \"%s\" unknown -> ignored\n", kind);
        return;
    }
    size_t len = 0;
    int64_t capts = 0;
    uint8_t* buf = resolve_full_bytes(event_id, quality, &len, &capts);
    if (!buf) return;
    // Mesh ceiling: the gateway spools chunks to LittleFS (gateway-0.6.0, 2026-07-04)
    // and streams the upload from flash, so the old ~48 KB heap ceiling is gone. The
    // remaining cap matches the gateway's chunk-geometry sanity check (512 KB / 32
    // chunks of 16 KB) — far above any QXGA original. Anything bigger still degrades
    // to the standard tier rather than failing.
#ifndef LEAF_MESH_FULL_MAX
#define LEAF_MESH_FULL_MAX (512 * 1024)
#endif
    const char* effective = quality && quality[0] ? quality : "max";
    if (len > LEAF_MESH_FULL_MAX) {
        Serial.printf("[leaf] mesh full %s: %u bytes > %u gateway cap -> standard tier\n",
                      event_id, (unsigned)len, (unsigned)LEAF_MESH_FULL_MAX);
        free(buf);
        buf = resolve_full_bytes(event_id, "standard", &len, &capts);
        if (!buf) return;
        effective = "standard";
    }
    leaf_radio_send_full(buf, len, event_id, capts, effective);
    free(buf);
}
#endif

// --- event hooks (stubs — fill in as the real leaf firmware grows) -------------------
// A capture event: power the camera, grab one alert-thumbnail JPEG, run the detector,
// hand the bytes to the radio, tear the camera back down. Camera bring-up is proven in
// gate-b; radio TX is the next layer (stubbed handoff below until the 915 antenna + a
// peer are on). Everything here runs on a fresh boot every PIR wake, then we deep-sleep.
static void on_pir_event() {
    // Capture thumb + full BACK-TO-BACK (one framesize switch apart, ~1.5 s) into copies,
    // then power the camera down before any slow work (detector ~4 s, serial emit ~3 s).
    // The stored full must show the SAME MOMENT as its thumbnail — grabbing it after the
    // detector/emit put it ~8 s late, meters of movement for a walking animal (same class
    // of bug as the fetch_full re-capture, dev photo 93487506).
    uint32_t t0 = millis();
    LeafFrame frame = leaf_camera_capture();
    if (!frame.ok()) {
        Serial.println("[leaf] PIR event -> CAPTURE FAILED (see [cam] line above)");
        leaf_camera_end(frame);
        return;
    }
    size_t len = frame.fb->len;
    int    w   = frame.fb->width;
    int    h   = frame.fb->height;
    uint8_t* thumb = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    if (!thumb) {   // can't copy -> keep the old in-fb path degenerately: bail, retry next PIR
        Serial.println("[leaf] thumb copy alloc FAILED");
        leaf_camera_end(frame);
        return;
    }
    memcpy(thumb, frame.fb->buf, len);
    leaf_camera_release(frame);

    uint8_t* fullbuf = nullptr;
    size_t   fulllen = 0;
    LeafFrame full = leaf_camera_capture_full();
    if (full.ok()) {
        fulllen = full.fb->len;
        fullbuf = (uint8_t*)heap_caps_malloc(fulllen, MALLOC_CAP_SPIRAM);
        if (fullbuf) memcpy(fullbuf, full.fb->buf, fulllen);
        else fulllen = 0;
    } else {
        Serial.println("[leaf] full-res grab failed (thumb still sends; no store)");
    }
    leaf_camera_end(full);              // sensor OFF before the slow stages
    uint32_t cap_ms = millis() - t0;
    g_captures++;                       // health counter (bug 5): capture succeeded

    bool send = detector_says_send(thumb, len, w, h);

    if (send) {
        // Mint the stable event id ONCE (RTC time + RTC counter); thumb now, the stored
        // full is served later under the same id by a fetch_full command.
        const char* eid = tc_new_event(g_unixtime);

        // Bench transport: emit the framed capture on the USB console for the host-bridge
        // to upload (prototype/trail-cam/host-bridge). Emitted BEFORE the radio so its logs
        // don't interleave the base64 body. TODO: fill meta.battery_v (ADC) / temp_c (BME280).
        LeafTelemetry meta;
        tc_emit_capture(eid, g_unixtime, thumb, len, "thumb", meta);

        // Mesh transport: same bytes as an RNS::Resource to the gateway, wrapped in the
        // metadata envelope so the gateway uploads under the REAL event id (and the
        // serial + mesh copies of the same event dedupe server-side).
#ifdef LEAF_RADIO
        leaf_radio_set_health(g_pir_wakes, g_captures, g_push_fails, leaf_read_vbat());
#endif
        if (!radio_send_alert(thumb, len, eid, g_unixtime))
            g_push_fails++;             // health counter (bug 5): push failed

        // Store the same-moment full-res, keyed by event_id — it only travels when a
        // fetch_full command asks for it.
        if (fullbuf && fulllen) leaf_store_save(eid, fullbuf, fulllen);
    }

    if (fullbuf) heap_caps_free(fullbuf);
    heap_caps_free(thumb);

    Serial.printf("[leaf] PIR event -> captured %dx%d JPEG, %u bytes (+full %u) in %ums, send=%d\n",
                  w, h, (unsigned)len, (unsigned)fulllen, (unsigned)cap_ms, (int)send);
}

// A proactive check-in: bring the radio up, open a brief RX window to pull any queued
// gateway commands (hi-res requests, schedule/time sync), heartbeat, then sleep.
static void on_scheduled_checkin() {
#ifdef LEAF_RADIO
    if (leaf_radio_begin()) {
        // Announce only — the mesh command window at the END of every wake does the
        // listening, because the gateway fires queued commands at us the moment it
        // HEARS this announce (announce-handler wake-triggered delivery).
        leaf_radio_set_health(g_pir_wakes, g_captures, g_push_fails, leaf_read_vbat());
        leaf_radio_announce("checkin");     // heartbeat so the gateway sees us alive
    }
#else
    Serial.println("[leaf] scheduled check-in -> (stub) brief RX window + heartbeat");
#endif
}

// First boot / reset: full init + announce our (persistent) identity to the mesh so the
// gateway learns our path.
static void on_cold_boot() {
#ifdef LEAF_RADIO
    if (leaf_radio_begin()) {
        leaf_radio_set_health(g_pir_wakes, g_captures, g_push_fails, leaf_read_vbat());
        leaf_radio_announce("hello");
    }
#else
    Serial.println("[leaf] cold boot -> (stub) full init + mesh announce");
#endif
}

// --- maintenance mode (2026-07-03) ----------------------------------------------------
// A "maintenance" command (serial or mesh) lands here after the normal wake work: skip
// the next deep-sleep and stay awake with both command channels open, so an operator can
// service the device (OTA, fetch_full, diagnostics) without racing the 3 s wake window.
// With WiFi creds in the payload we also join and NTP-sync the clock — real timestamps
// until the next full power loss. Exits on the deadline or a {"kind":"sleep"} command.

// PIR stays live during the window (2026-07-16 outage, open item 5): ext0 only fires
// from deep sleep, so a maintenance window used to be a capture blackout — cmd 58's
// 60 min window silently ate 52 min of walk-bys. A RISING interrupt on the PIR pin
// feeds the normal capture path, rate-limited to roughly the deep-sleep cycle cadence
// (capture+detector+send+PIR-drain is ~25 s end to end).
#ifndef LEAF_MAINT_PIR_REFRACTORY_MS
#define LEAF_MAINT_PIR_REFRACTORY_MS 30000
#endif
static volatile bool s_maint_pir_edge = false;
static void IRAM_ATTR maint_pir_isr() { s_maint_pir_edge = true; }

static void maintenance_loop(const TcMaintRequest& req) {
    Serial.printf("[maint] entering maintenance mode for %u min (wifi=%s)\n",
                  (unsigned)req.minutes, req.ssid[0] ? req.ssid : "no");
    bool ntp_ok = false;
    if (req.ssid[0]) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(req.ssid, req.psk);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(100);
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[maint] WiFi up, ip=%s rssi=%d\n",
                          WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
            configTime(0, 0, "pool.ntp.org");
            t0 = millis();
            while (millis() - t0 < 8000) {
                time_t now = time(nullptr);
                if (now > 1750000000) {   // sanity: past mid-2025 = a real NTP answer
                    g_unixtime = (int64_t)now;
                    ntp_ok = true;
                    Serial.printf("[maint] clock synced via NTP: %lld\n", (long long)now);
                    break;
                }
                delay(200);
            }
            if (!ntp_ok) Serial.println("[maint] NTP sync timed out (clock unchanged)");
        } else {
            Serial.println("[maint] WiFi join FAILED -> staying awake without it");
        }
    }

    g_maint_exit = false;
    const uint32_t start_ms    = millis();
    const uint32_t deadline_ms = req.minutes * 60000UL;
    uint32_t last_announce = 0;
    // Arm the awake-mode PIR path. The pin is often still HIGH here (this wake was
    // likely the PIR event itself); RISING means we only fire on the NEXT motion.
    // ext0 arming routed the pad to the RTC mux, where the digital GPIO matrix (and
    // attachInterrupt) can't see it — hand it back first. Verified live 2026-07-16:
    // without the deinit the ISR never fires no matter how much you wave.
    s_maint_pir_edge = false;
    uint32_t pir_last_cap = 0;
    rtc_gpio_deinit(PIR_GPIO);
    pinMode((int)PIR_GPIO, INPUT);
    attachInterrupt(digitalPinToInterrupt((int)PIR_GPIO), maint_pir_isr, RISING);
    // Fold wall time into g_unixtime as we go, not just at exit — a capture in minute 4
    // of a 5 min window must not carry a start-of-wake timestamp.
    uint32_t clock_credit_ms = start_ms;
#ifdef LEAF_RADIO
    leaf_radio_set_health(g_pir_wakes, g_captures, g_push_fails, leaf_read_vbat());
#endif
    while (!g_maint_exit && millis() - start_ms < deadline_ms) {
        if (time(nullptr) > 1700000000) {       // system clock valid (NTP or time_sync)
            g_unixtime = (int64_t)time(nullptr);
            clock_credit_ms = millis();
        } else if (g_unixtime > 0) {
            uint32_t el = millis() - clock_credit_ms;
            g_unixtime      += el / 1000;
            clock_credit_ms += (el / 1000) * 1000;   // keep the sub-second remainder
        }

        if (s_maint_pir_edge) {
            s_maint_pir_edge = false;           // edges inside the refractory are dropped
            if (pir_last_cap == 0 || millis() - pir_last_cap >= LEAF_MAINT_PIR_REFRACTORY_MS) {
                pir_last_cap = millis();
                g_pir_wakes++;                  // same health counter as an ext0 wake
                Serial.println("[maint] PIR edge -> capture (window stays open)");
                on_pir_event();
            }
        }

#ifdef LEAF_RADIO
        // Periodic announce so the gateway keeps seeing us and fires queued mesh
        // commands (its announce-triggered delivery suppresses repeats server-side).
        if (millis() - last_announce > 30000) {
            last_announce = millis();
            leaf_radio_announce("maint");
        }
        leaf_radio_poll_commands(1000, on_mesh_command);
#endif
        tc_poll_commands(1000, on_gateway_command);
    }
    detachInterrupt(digitalPinToInterrupt((int)PIR_GPIO));

    // Keep the RTC clock honest for whatever we do next: NTP made it exact; otherwise
    // credit the (remaining, uncredited) time we sat here.
    if (ntp_ok) g_unixtime = (int64_t)time(nullptr);
    else if (g_unixtime > 0) g_unixtime += (millis() - clock_credit_ms) / 1000;
    if (req.ssid[0]) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }
    Serial.printf("[maint] maintenance over (%s) -> normal sleep cycle\n",
                  g_maint_exit ? "sleep command" : "deadline");
}

// Bench-grade clock seed so captured_at and event_id are real-ish from the first boot
// (the ingest contract REQUIRES captured_at; the host-bridge rejects frames without it).
// Replaced by real gateway time sync later.
//
// The seed MUST NEVER RUN AHEAD of real time (bug 6, 2026-07-16): announce packets embed
// the leaf's clock as their emission timestamp, and the gateway's Transport replay guard
// rejects any announce "older" than the max it has recorded — so a fast seed followed by
// a corrective (backward) time sync wedges announce processing on the gateway for as
// long as the seed was fast. Preferred source: LEAF_BUILD_EPOCH, the build host's actual
// UTC epoch injected by platformio.ini — timezone-proof and always in the past. The
// __DATE__/__TIME__ parse below is the fallback; its LEAF_BUILD_UTC_OFFSET_S assumed an
// EDT build host, which made every UTC-built image seed 4 h FAST (measured drift -14027 s
// on the first gateway sync — the bench reproduction of the 07-15 all-night wedge).
#ifndef LEAF_BUILD_UTC_OFFSET_S
#define LEAF_BUILD_UTC_OFFSET_S 14400        // build host is EDT (UTC-4)
#endif
static int64_t build_epoch() {
#ifdef LEAF_BUILD_EPOCH
    return (int64_t)LEAF_BUILD_EPOCH;
#else
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char m[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
    struct tm tmv = {};
    const char* hit = strstr(months, m);
    tmv.tm_mon  = hit ? (int)(hit - months) / 3 : 0;
    tmv.tm_mday = atoi(__DATE__ + 4);
    tmv.tm_year = atoi(__DATE__ + 7) - 1900;
    tmv.tm_hour = atoi(__TIME__);
    tmv.tm_min  = atoi(__TIME__ + 3);
    tmv.tm_sec  = atoi(__TIME__ + 6);
    return (int64_t)mktime(&tmv) + LEAF_BUILD_UTC_OFFSET_S;   // TZ unset on ESP32 -> UTC
#endif
}

// Human tag for the telemetry beacon's boot_reason (matches the ingest contract vocab).
static const char* boot_reason_str() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_DEEPSLEEP: return "DSLEEP";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_INT_WDT:   return "WDT";
        default:                return "OTHER";
    }
}

static void print_status(esp_sleep_wakeup_cause_t cause) {
    const char* why =
        cause == ESP_SLEEP_WAKEUP_EXT0  ? "PIR (ext0)" :
        cause == ESP_SLEEP_WAKEUP_TIMER ? "timer"      : "cold boot / reset";
    // reset= tells crash boots apart from clean ones: a PIR-capture crash shows up as
    // cause="cold boot / reset" + reset=PANIC/BROWNOUT with the RTC counters wiped
    // (observed 2026-07-03 on dock USB power; the capture path died mid-wake twice).
    Serial.printf("\n[leaf] wake #%u  cause=%s  reset=%s  | pir=%u timer=%u  | slept~%us\n",
                  (unsigned)g_boot_count, why, boot_reason_str(),
                  (unsigned)g_pir_wakes, (unsigned)g_timer_wakes,
                  (unsigned)g_last_sleep_s);
}

// Don't re-arm ext0 while the AM312 output is still HIGH from the event we just handled
// (~2 s hold) or we'd immediately wake again. Wait for it to settle low (bounded).
static void wait_pir_low() {
    // Same RTC-mux gotcha as the maintenance PIR path: after any ext0-armed sleep the
    // pad is RTC-routed and digitalRead sees a constant 0 — this drain loop was a no-op
    // on every wake since ext0 was first armed (it only ever worked on cold boot).
    rtc_gpio_deinit(PIR_GPIO);
    pinMode((int)PIR_GPIO, INPUT);
    uint32_t t0 = millis();
    while (digitalRead((int)PIR_GPIO) == HIGH && millis() - t0 < 6000) delay(20);
}

// --- over-discharge lockout (leaf-0.16.0) --------------------------------------------
// The CN3058E guards the cell while CHARGING (3.6 V CV, C/10 term); nothing on the
// board guards it from US — protected 1S LiFePO4 26650s effectively don't exist
// (homelab docs/trailcam/leaf-pcb.md, 2026-07-03) and there is no protection PCB. So
// the discharge cutoff is firmware: below LOCKOUT every wake collapses to "read ADC,
// sleep an hour" — no radio, no camera, no ext0 (a busy game trail must not burn
// boots on a flat cell) — until solar lifts the cell past RESUME. The deep-sleep
// floor (~tens of uA incl. the 940 k divider + PIR) parks a 26650 for years, so
// refusing the SPENDING is as good as a hardware disconnect. What firmware can't
// stop is a brownout boot loop — hence the last-gasp announce is skipped after a
// BROWNOUT reset, and a real UVLO stays on the next-spin wishlist.
// Hysteresis is wide on purpose: the LiFePO4 curve sits flat near 3.2 V for most of
// capacity, so a narrow band would chatter on ADC noise; 3.30 V is only reachable
// by actual charging. Reads are unloaded by construction — the FET rails are still
// held off this early in boot.
#ifndef LEAF_VBAT_LOCKOUT_V
#define LEAF_VBAT_LOCKOUT_V 3.00f      // resting ~5-10% SoC on LiFePO4
#endif
#ifndef LEAF_VBAT_RESUME_V
#define LEAF_VBAT_RESUME_V  3.30f      // unreachable except by charging
#endif
#ifndef LEAF_VBAT_SENSE_FLOOR_V
#define LEAF_VBAT_SENSE_FLOOR_V 2.00f  // the LDO can't be running the MCU from less:
#endif                                 // a lower reading means the SENSE is broken
#ifndef LEAF_LOCKOUT_SLEEP_S
#define LEAF_LOCKOUT_SLEEP_S 3600
#endif
RTC_DATA_ATTR uint8_t g_lowbatt = 0;   // wiped by cold boot -> re-evaluated from the ADC

static float read_vbat_avg() {         // 8-sample mean (~1 ms): kills single-read jitter
    float sum = 0; int n = 0;
    for (int i = 0; i < 8; i++) {
        float v = leaf_read_vbat();
        if (!isnan(v)) { sum += v; n++; }
    }
    return n ? sum / (float)n : NAN;
}

[[noreturn]] static void lowbatt_sleep(float vb) {
    leaf_rails_all_off();
    Serial.printf("[leaf] LOW BATTERY %.2fV -> lockout: timer-only sleep %us (resume >= %.2fV)\n",
                  vb, (unsigned)LEAF_LOCKOUT_SLEEP_S, LEAF_VBAT_RESUME_V);
    Serial.flush();
    g_last_sleep_s = LEAF_LOCKOUT_SLEEP_S;
    esp_sleep_enable_timer_wakeup((uint64_t)LEAF_LOCKOUT_SLEEP_S * 1000000ULL);
    esp_deep_sleep_start();
}

static void arm_and_sleep() {
    // time-of-day for the schedule (approx; corrected by gateway sync later)
    bool time_known = (g_unixtime > 0);
    int month = 1, hour = 12;
    if (time_known) {
        time_t t = (time_t)g_unixtime; struct tm tmv; gmtime_r(&t, &tmv);
        month = tmv.tm_mon + 1; hour = tmv.tm_hour;
    }
    uint32_t interval = checkin_interval_s(time_known, month, hour);
    g_last_sleep_s = interval;

    wait_pir_low();
    // PIR wake: AM312 goes HIGH on motion -> wake on level 1
    esp_sleep_enable_ext0_wakeup(PIR_GPIO, 1);
    // Scheduled check-in
    esp_sleep_enable_timer_wakeup((uint64_t)interval * 1000000ULL);

    leaf_log_vbat();          // PCB: battery divider on IO3 (no-op on bench)
    leaf_env_log();           // PCB: BME280 T/RH/P (no-op unless LEAF_BME280)
    leaf_rails_all_off();     // PCB: camera + radio rails hard-off for the sleep floor
    Serial.printf("[leaf] arming: PIR on GPIO%d + timer %us  (time %s)\n",
                  (int)PIR_GPIO, (unsigned)interval, time_known ? "known" : "unknown");
    Serial.flush();
    // TODO(power): rtc_gpio_isolate() unused pins, power down RTC peripherals we don't
    //              need, to reach the ~tens-of-uA floor. Measure with a uCurrent/meter.
    esp_deep_sleep_start();   // never returns; MCU resets into setup() on next wake
}

#ifdef LEAF_SD_PROBE
// Diagnostic: isolate the microSD from the rest of the stack. Mount (SD_MMC 1-bit on the
// Freenove slot pins), print card info, write + read back a test file, loop. No sleep, no
// camera, no radio. Use to tell a hardware problem (card/slot/pins) from an interaction
// bug in the full firmware (SD after camera/radio init).
#include <SD_MMC.h>
void setup() {
    Serial.begin(LEAF_BAUD);
    delay(500);
    Serial.println("\n[sdp] SD probe: setPins(39,38,40) + begin, 1-bit");
}
void loop() {
    Serial.println("[sdp] setPins...");
    SD_MMC.setPins(39, 38, 40);
    Serial.println("[sdp] begin...");
    bool ok = SD_MMC.begin("/sdcard", /*mode1bit=*/true);
    Serial.printf("[sdp] begin -> %s\n", ok ? "OK" : "FAILED");
    if (ok) {
        Serial.printf("[sdp] card: type=%d size=%llu MB\n",
                      (int)SD_MMC.cardType(), SD_MMC.cardSize() / (1024 * 1024));
        File f = SD_MMC.open("/probe.txt", "w");
        if (f) { f.println("hello from leaf sd probe"); f.close(); }
        f = SD_MMC.open("/probe.txt", "r");
        if (f) {
            Serial.printf("[sdp] readback: %s", f.readStringUntil('\n').c_str());
            Serial.println();
            f.close();
        } else Serial.println("[sdp] readback FAILED");
        SD_MMC.end();
    }
    delay(4000);
}
#elif defined(LEAF_PIN_MONITOR)
// Diagnostic: characterize the PIR line on GPIO21 without sleeping. Reads it plain,
// then with internal pulldown, then pullup, to tell driven-vs-floating apart:
//   pulldown=1 & pullup=1 -> DRIVEN HIGH (PIR output high / motion / short to 3V3)
//   pulldown=0 & pullup=0 -> DRIVEN LOW  (PIR idle, correct)
//   pulldown=0 & pullup=1 -> FLOATING    (OUT not actually connected to GPIO21)
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n[mon] GPIO21 PIR-line characterizer (no sleep). Wave, then hold still.");
    Serial.println("[mon] HHH=driven-high  LLL=driven-low  (pd=0,pu=1)=FLOATING/not connected");
}
static void characterize(int pin) {
    pinMode(pin, INPUT);          delay(6); int a = digitalRead(pin);
    pinMode(pin, INPUT_PULLDOWN); delay(6); int b = digitalRead(pin);
    pinMode(pin, INPUT_PULLUP);   delay(6); int c = digitalRead(pin);
    const char* verdict = (b==1 && c==1) ? "DRIVEN HIGH"
                        : (b==0 && c==0) ? "DRIVEN LOW"
                        :                  "FLOATING (nothing driving it)";
    Serial.printf("[mon] GPIO%-2d plain=%d pulldown=%d pullup=%d  => %s\n", pin, a, b, c, verdict);
}
void loop() {
    characterize(LEAF_PIR_GPIO);   // pin 21 (PIR) — the suspect
    characterize(39);              // control: unused spare -> expect FLOATING
    characterize(40);              // control: BME280 SDA (nothing wired) -> expect FLOATING
    Serial.println();
    delay(500);
}
#else
void setup() {
    // A CMD burst from the bridge (~230 B/line) overflows the default 256 B UART RX
    // buffer while a slow handler runs — size it for a whole queue. Must precede begin().
    Serial.setRxBufferSize(2048);
    Serial.begin(LEAF_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 800) delay(10);

    g_boot_count++;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    print_status(cause);

    // Pin the bootloader to THIS slot. Observed 2026-07-03: the board alternated between
    // leaf-0.5.0 and leaf-0.7.0 across ordinary deep-sleep wakes (wake #708-711 vs #712,
    // RTC counters continuous), i.e. otadata was in a state where slot selection wasn't
    // stable. Marking the running app valid rewrites otadata to a clean "boot me" record;
    // a no-op error when there's nothing pending, so safe to call every wake.
    esp_ota_mark_app_valid_cancel_rollback();

    // Clock, best source first: the RTC-kept system clock (ticks through deep sleep;
    // valid once any time_sync/NTP ran this power cycle) > interval arithmetic on
    // g_unixtime > the build-timestamp seed. The gateway pushes a time_sync on our
    // cold-boot "hello", so the seed only covers the first minute after power loss.
    const bool rtc_clock_valid = time(nullptr) > 1700000000;
    if (rtc_clock_valid) {
        g_unixtime = (int64_t)time(nullptr);
    } else if (g_unixtime == 0) {
        g_unixtime = build_epoch();
        Serial.printf("[leaf] clock seeded from build time: %lld\n", (long long)g_unixtime);
    }

#ifdef LEAF_RADIO
    // ADR: stage the granted (or reverted) radio profile BEFORE any radio use — a PIR
    // wake's thumb push is the first TX and must already be on the negotiated profile.
    rf_profile_wake_begin();
#ifdef LEAF_TX_SWEEP
    tx_sweep_wake_begin();   // before any radio use, so the announce TXes at it too
#endif
#endif

    // Over-discharge lockout gate (see the block above arm_and_sleep). After the clock
    // block (the approx clock must keep ticking through a long lockout) and before the
    // dispatch — a lockout wake does NONE of the normal work.
    {
        const float vb    = read_vbat_avg();
        const bool  vb_ok = !isnan(vb) && vb >= LEAF_VBAT_SENSE_FLOOR_V;
        if (g_lowbatt) {
            if (vb_ok && vb < LEAF_VBAT_RESUME_V) {
                // Still low. Unsynced-clock fallback advance (mirrors the TIMER case
                // below, which we never reach), then straight back down.
                if (!rtc_clock_valid && g_unixtime > 0 && cause == ESP_SLEEP_WAKEUP_TIMER)
                    g_unixtime += g_last_sleep_s;
                lowbatt_sleep(vb);
            }
            // Recovered — or the sense went bogus, in which case run rather than brick:
            // a dark camera until a site visit costs more than the cell it protects.
            g_lowbatt = 0;
            Serial.printf("[leaf] battery lockout cleared (vb=%.2fV) -> normal duty\n", vb);
        } else if (vb_ok && vb < LEAF_VBAT_LOCKOUT_V) {
            g_lowbatt = 1;
            // Last gasp: one announce so the dashboard shows WHY this node goes dark
            // (vb rides the health tail; a node that vanished at vb=2.98 is a different
            // debug session than one that vanished at 3.5). Never after a BROWNOUT
            // reset — TX sag is the prime brownout suspect at this voltage and
            // retrying it every boot is the loop that kills cells.
#ifdef LEAF_RADIO
            if (esp_reset_reason() != ESP_RST_BROWNOUT && leaf_radio_begin()) {
                leaf_radio_set_health(g_pir_wakes, g_captures, g_push_fails, vb);
                leaf_radio_announce("lowbatt");
            }
#endif
            lowbatt_sleep(vb);
        }
    }

    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT0:  g_pir_wakes++;   on_pir_event();        break;
        case ESP_SLEEP_WAKEUP_TIMER: g_timer_wakes++; on_scheduled_checkin();
            // Unsynced fallback only: advance the approx clock by the slept interval
            // (the RTC clock path above already includes it exactly).
            if (!rtc_clock_valid && g_unixtime > 0) g_unixtime += g_last_sleep_s;
            break;
        default:                                       on_cold_boot();        break;
    }

    // Telemetry beacon on every wake (fire-and-forget) so the node reads OK on the Nodes
    // health page — at least daily by construction, since we wake at least each check-in
    // interval. TODO: fill battery_v (ADC divider, GPIO3) + temp_c/pressure_hpa (BME280).
    LeafTelemetry tlm;
    tlm.boot_reason = boot_reason_str();
    tc_emit_telemetry(tlm);

    // Downlink window: listen for !TC CMD lines from the bridge before sleeping. The
    // bridge writes on its own 30 s poll, blind to our sleep, so only writes that land in
    // this window arrive — the server re-delivers until satisfied, so misses self-heal.
#ifndef LEAF_CMD_WINDOW_MS
#define LEAF_CMD_WINDOW_MS 3000
#endif
#ifdef LEAF_RADIO
    // Mesh downlink: the gateway fires queued commands at us the moment it hears
    // whatever announce this wake made (checkin/alert/hello) — catch and execute them.
    // A fetch_full answer can hold the wake for many minutes (chunked full-res).
    leaf_radio_poll_commands(2500, on_mesh_command);
#endif
    tc_poll_commands(LEAF_CMD_WINDOW_MS, on_gateway_command);

    // A maintenance command from either channel above defers the sleep: stay awake and
    // serviceable, then fall through to the normal arm+sleep.
    TcMaintRequest maint;
    if (tc_take_maint_request(&maint)) maintenance_loop(maint);

#ifdef LEAF_RADIO
    // ADR revert bookkeeping: on a non-base profile, count wakes where the gateway gave
    // no sign of hearing us (no link, no packet). Hitting the cap reverts us to base on
    // the next wake — the recovery path when the two ends' profiles diverge.
    if (g_rf_profile != LEAF_RF_BASE_PROFILE) {
        if (leaf_radio_had_contact()) g_rf_silent_wakes = 0;
        else if (g_rf_silent_wakes < 255) g_rf_silent_wakes++;
    }
#endif

    arm_and_sleep();
}

void loop() { /* unreachable: we deep-sleep out of setup() every wake */ }
#endif // LEAF_PIN_MONITOR
