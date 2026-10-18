// BLE fit spike — the RUNTIME half of the kill-gate for the BLE + native-app branch
// The compile-only half (flash + static RAM against the real dual-OTA
// partition table) is answered by building `freenove-s3` vs `freenove-s3-ble` and reading
// the linker's numbers; nothing needs to run for that. This file answers the four
// questions that only hardware can:
//
//   2. Free INTERNAL heap with camera + RNS + TFLite arena all live, plus BLE advertising.
//   3. Free PSRAM in that same state.
//   4. That advertising and a GATT connection actually WORK there — not merely that the
//      image links. A stack that links and then fails ble_gap_adv_start() for want of a
//      few KB of internal DRAM is the exact failure this gate exists to catch, and it is
//      invisible to the linker.
//   5. BLE/WiFi coexistence on the single 2.4 GHz radio: that the intended SEQUENTIAL
//      handoff (drop BLE -> bring WiFi up -> transfer -> tear down -> BLE back) neither
//      wedges nor browns out. Running both at once is explicitly NOT the design, so this
//      does not test concurrent operation — it tests the transition, which is where a
//      single-radio part actually breaks.
//
// WHY A STANDALONE HARNESS AND NOT THE NORMAL WAKE PATH. The production leaf deep-sleeps
// out of setup() within seconds, which is the wrong shape for a measurement you have to
// stand in front of with a phone. This stays awake, brings the stack up one layer at a
// time, prints a heap/PSRAM line after EACH layer, and then waits for you. The per-layer
// lines are the point: a single "it fit" number tells you nothing about which layer to cut
// if it does not, and the answer this gate hands to the walk-up work has to survive being
// argued with.
//
// WHAT IT DELIBERATELY DOES NOT DO. No deep sleep, no PIR, no store eviction, no alert
// push. Those are all measured elsewhere and would only add noise to the memory picture.
//
// RUNNING IT
//   pio run -e freenove-s3-ble-fit -t upload && pio device monitor -b 460800
// Then, when the console says so: scan with a phone (nRF Connect / LightBlue), connect to
// "MeshCam-XXXXXX", read the status characteristic, and write "wifi" to it to trigger the
// handoff test. The harness also runs the handoff UNPROMPTED after LEAF_BLE_FIT_WAIT_S so
// an unattended overnight run still answers item 5.
//
// WIFI CREDENTIALS. The handoff test joins a real AP if LEAF_FIT_WIFI_SSID is defined —
// picked up from the gitignored wire_secrets.h below — because "WiFi came up after BLE
// went down" is only convincing if the radio actually associated. Without creds it
// falls back to a SoftAP, which still
// exercises the same driver bring-up on the same radio and is the fallback
// design anyway — so the run is valid either way, just note which path the log shows.

#include "leaf_camera.h"
#include "leaf_ble.h"
#include "leaf_radio.h"
#include "leaf_detector.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include <WiFi.h>
// Bench AP credentials for the STA half of the handoff. They come from the gitignored
// wire_secrets.h — the same header sdq_wire.inc and cam_bakeoff.inc use — NOT from -D
// flags on the build command line. As flags they sit on every compiler invocation's
// ARGV and got echoed verbatim by a compiler error on 2026-08-09. Absent the
// header the harness falls back to SoftAP, which is still a valid run.
#if !defined(LEAF_FIT_WIFI_SSID) && __has_include("wire_secrets.h")
#include "wire_secrets.h"
#if defined(GW_WIFI_SSID2) && defined(GW_WIFI_PSK2)
#define LEAF_FIT_WIFI_SSID GW_WIFI_SSID2   // net 2 is the bench SSID
#define LEAF_FIT_WIFI_PSK  GW_WIFI_PSK2
#endif
#endif

#ifndef LEAF_BLE_FIT_WAIT_S
#define LEAF_BLE_FIT_WAIT_S 180      // how long to hold the BLE window open for a phone
#endif
#ifndef LEAF_FIT_SOFTAP_SSID
#define LEAF_FIT_SOFTAP_SSID "MeshCam-Fit"
#endif
#ifndef LEAF_FIT_SOFTAP_PSK
#define LEAF_FIT_SOFTAP_PSK  "meshcamfit"
#endif

// --- measurement ------------------------------------------------------------------------
// Internal DRAM is the scarce resource, not PSRAM: the BT controller, the NimBLE host and
// the WiFi driver all allocate from it, and none of them can be moved to PSRAM. So the
// headline number is free INTERNAL 8-bit heap, and the largest free internal BLOCK is
// printed beside it — a heap that is 60 KB free in 2 KB fragments will not satisfy the
// WiFi driver's single large allocation, and total-free alone hides that completely.
struct FitMark {
    const char* stage;
    size_t int_free, int_largest, int_min_ever;
    size_t psram_free, psram_largest;
};
static FitMark s_marks[12];
static int     s_nmarks = 0;

static void fit_mark(const char* stage) {
    FitMark m;
    m.stage         = stage;
    m.int_free      = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    m.int_largest   = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    m.int_min_ever  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    m.psram_free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    m.psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    if (s_nmarks < (int)(sizeof(s_marks) / sizeof(s_marks[0]))) s_marks[s_nmarks++] = m;
    Serial.printf("[fit] %-22s int_free=%7u  int_largest=%7u  int_min=%7u  "
                  "psram_free=%8u  psram_largest=%8u\n",
                  stage, (unsigned)m.int_free, (unsigned)m.int_largest,
                  (unsigned)m.int_min_ever, (unsigned)m.psram_free,
                  (unsigned)m.psram_largest);
}

static void fit_summary() {
    Serial.println();
    Serial.println("======================== BLE FIT RUNTIME RESULTS ========================");
    Serial.printf("%-22s %12s %12s %12s %12s\n",
                  "stage", "int_free", "int_largest", "psram_free", "delta_int");
    for (int i = 0; i < s_nmarks; i++) {
        long d = (i == 0) ? 0 : (long)s_marks[i].int_free - (long)s_marks[i - 1].int_free;
        Serial.printf("%-22s %12u %12u %12u %+12ld\n",
                      s_marks[i].stage, (unsigned)s_marks[i].int_free,
                      (unsigned)s_marks[i].int_largest, (unsigned)s_marks[i].psram_free, d);
    }
    Serial.println("=========================================================================");
}

// --- the WiFi half of the handoff ---------------------------------------------------------
// Returns true if the radio came up on WiFi at all. Which path it took is logged; a SoftAP
// bring-up is a weaker but still valid answer to item 5 (same driver, same radio, same
// internal-DRAM allocations — it just does not prove association with a real AP).
static bool fit_wifi_up() {
#if defined(LEAF_FIT_WIFI_SSID)
    Serial.printf("[fit] WiFi STA -> joining \"%s\"\n", LEAF_FIT_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(LEAF_FIT_WIFI_SSID, LEAF_FIT_WIFI_PSK);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(100);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[fit] WiFi UP: ip=%s rssi=%d (join took %u ms)\n",
                      WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
                      (unsigned)(millis() - t0));
        return true;
    }
    Serial.println("[fit] WiFi STA join FAILED -> falling back to SoftAP");
#endif
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(LEAF_FIT_SOFTAP_SSID, LEAF_FIT_SOFTAP_PSK);
    Serial.printf("[fit] SoftAP \"%s\" %s (ip=%s)\n", LEAF_FIT_SOFTAP_SSID,
                  ok ? "UP" : "FAILED", WiFi.softAPIP().toString().c_str());
    return ok;
}

static void fit_wifi_down() {
    WiFi.disconnect(true, true);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
}

// Item 5 proper. The order here IS the design under test: BLE all the way down (which
// releases the controller's heap) BEFORE WiFi touches the radio, and symmetrically on the
// way back. Doing it in the other order is what wedges, and the whole point of running it
// is that "obviously it works" is not evidence.
static void fit_handoff_test() {
    Serial.println();
    Serial.println("[fit] --- item 5: sequential BLE -> WiFi -> BLE handoff ---");

    // Pass 1: the RESTARTABLE teardown (keep the controller's memory). This is the only
    // way to measure a full BLE -> WiFi -> BLE round trip, because releasing the
    // controller BSS is one-way for the boot (see leaf_ble.h).
    leaf_ble_end(false);
    fit_mark("ble down (kept)");

    const bool wifi_ok = fit_wifi_up();
    fit_mark(wifi_ok ? "wifi up" : "wifi FAILED");

    // Hold WiFi up for a beat: a brownout from the TX current step does not necessarily
    // land in the first millisecond of association, and a reset here is exactly the
    // finding item 5 asks for.
    delay(3000);
    fit_mark("wifi settled");

    fit_wifi_down();
    fit_mark("wifi down");

    const bool ble_again = leaf_ble_begin();
    fit_mark(ble_again ? "ble back up" : "ble RESTART FAILED");

    // Pass 2: the PRODUCT teardown. The leaf's maintenance window ends in deep sleep, so
    // it can afford the one-way release and wants the RAM back. The delta between this
    // mark and the one above is what that release is actually worth. Nothing may call
    // leaf_ble_begin() after this point in the boot.
    if (ble_again) {
        leaf_ble_end(true);
        fit_mark("ble down (released)");
    }

    Serial.printf("[fit] handoff verdict: wifi=%s  ble_round_trip=%s  reset_reason=%d\n",
                  wifi_ok ? "UP" : "FAILED", ble_again ? "OK" : "FAILED",
                  (int)esp_reset_reason());
    Serial.println("[fit] (a brownout or wedge shows up as a REBOOT — if you are reading");
    Serial.println("[fit]  this line, neither happened on this pass.)");
}

// --- harness -----------------------------------------------------------------------------
void setup() {
    Serial.setRxBufferSize(2048);
    Serial.begin(LEAF_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) delay(10);
    delay(300);

    Serial.println();
    Serial.println("=========================================================================");
    Serial.println("BLE FIT SPIKE — camera + RNS + TFLite + NimBLE on one S3");
    Serial.printf("reset_reason=%d  chip_rev=%d  psram_total=%u  heap_total=%u\n",
                  (int)esp_reset_reason(), (int)ESP.getChipRevision(),
                  (unsigned)ESP.getPsramSize(), (unsigned)ESP.getHeapSize());
    Serial.println("=========================================================================");

    fit_mark("boot (bare)");

    // Layer 1: camera. Init at QXGA with the framebuffer in PSRAM, and HOLD a frame — the
    // worst case is a live capture in flight, not an idle initialized driver.
    LeafFrame frame = leaf_camera_capture();
    Serial.printf("[fit] camera capture: %s (%u bytes)\n",
                  frame.ok() ? "ok" : "FAILED", frame.ok() ? (unsigned)frame.fb->len : 0u);
    fit_mark("camera up + frame");

    // Layer 2: TFLite. leaf_detector_begin() allocates the tensor arena (PSRAM) and runs
    // AllocateTensors, so the arena is really committed, not merely reserved.
    const bool det_ok = leaf_detector_begin();
    Serial.printf("[fit] detector begin: %s\n", det_ok ? "ok" : "FAILED");
    fit_mark("tflite arena");

    // Run one real inference on the frame we are holding: AllocateTensors alone does not
    // touch every scratch buffer the kernels use.
    if (frame.ok()) {
        LeafDetection d = leaf_detector_run(frame.fb->buf, frame.fb->len,
                                            frame.fb->width, frame.fb->height);
        Serial.printf("[fit] inference: ok=%d score=%.2f in %ums\n",
                      (int)d.ok, d.score, (unsigned)d.infer_ms);
    }
    fit_mark("after inference");

    // Layer 3: RNS. Filesystem + persistent identity + SX1262 + Reticulum — the full stack
    // a real wake brings up, not a subset.
    const bool radio_ok = leaf_radio_begin();
    Serial.printf("[fit] radio begin: %s\n", radio_ok ? "ok" : "FAILED (no SX1262 wired?)");
    if (radio_ok) {
        leaf_radio_set_health(0, 0, 0, NAN);
        leaf_radio_announce("blefit");
    }
    fit_mark("rns up + announce");

    // Layer 4: BLE. This is the allocation that either fits or does not.
    const bool ble_ok = leaf_ble_begin();
    Serial.printf("[fit] BLE begin: %s\n", ble_ok ? "ok" : "FAILED");
    fit_mark(ble_ok ? "ble advertising" : "ble FAILED");

    if (!ble_ok) {
        Serial.println();
        Serial.println("[fit] *** VERDICT: BLE DID NOT COME UP with the full stack live. ***");
        Serial.println("[fit] That is the kill-gate answer: the GATT walk-up paths are");
        Serial.println("[fit] invalidated and the SoftAP-only walk-up wins.");
        fit_summary();
        Serial.println("[fit] halting (nothing further to measure).");
        while (true) delay(1000);
    }

    leaf_ble_set_status("fit-bench", "ble-fit", 0, 0, 0, 0, NAN);

    Serial.println();
    Serial.printf("[fit] --- item 4: scan for \"%s\" with a phone now. ---\n", leaf_ble_name());
    Serial.printf("[fit] Read the status characteristic; write \"wifi\" to trigger the\n");
    Serial.printf("[fit] handoff early. Otherwise it runs on its own in %d s.\n",
                  LEAF_BLE_FIT_WAIT_S);

    // Hold the window open. Notifying once a second both keeps a subscribed phone honest
    // and proves the connection is still alive rather than merely established.
    const uint32_t deadline = millis() + (uint32_t)LEAF_BLE_FIT_WAIT_S * 1000UL;
    bool saw_connection = false;
    uint32_t last_tick = 0, ticks = 0;
    while (millis() < deadline) {
        if (leaf_ble_connected()) {
            if (!saw_connection) {
                saw_connection = true;
                fit_mark("gatt connected");
            }
            if (millis() - last_tick > 1000) {
                last_tick = millis();
                leaf_ble_set_status("fit-bench", "ble-fit", ++ticks, 0, 0, 0, NAN);
            }
        }
        LeafBleCmd c = leaf_ble_take_cmd();
        if (c == LEAF_BLE_CMD_WIFI)  {
            Serial.println("[fit] phone asked for the handoff");
            // Give the client a chance to drop the link itself before we tear the stack
            // down. A well-behaved app disconnects after asking for WiFi (it is about to
            // join an AP), and a teardown with no live link is the path that was already
            // proven safe. leaf_ble_end() force-terminates if it does not.
            for (int i = 0; i < 300 && leaf_ble_connected(); i++) delay(10);   // <= 3 s
            Serial.printf("[fit] client %s before teardown\n",
                          leaf_ble_connected() ? "STILL CONNECTED" : "disconnected");
            break;
        }
        if (c == LEAF_BLE_CMD_SLEEP) { Serial.println("[fit] phone asked to stop"); break; }
        delay(20);
    }

    Serial.printf("[fit] item 4 verdict: a phone %s connect over GATT.\n",
                  saw_connection ? "DID" : "did NOT (unattended run, or out of range)");

    fit_handoff_test();
    fit_summary();

    Serial.println();
    Serial.println("[fit] done. BLE is down for good this boot (memory released); reset to rerun.");
}

void loop() {
    // Stay alive and keep advertising, so the board can be re-probed with a phone after
    // the handoff without another flash cycle.
    delay(1000);
}
