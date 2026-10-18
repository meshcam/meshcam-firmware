// Upload WIRE FORMAT test — posts real multipart bodies at a probe server.
//
// The gate harness (sdq_gates.inc) injects upload OUTCOMES. That proves what the gateway
// DOES with a 2xx/4xx/5xx, but it cannot prove the bytes are right: a ConcatStream that
// miscounts, a boundary that disagrees with the Content-Type header, a body streamed from
// the wrong file offset, or a missing form field would all sail straight through it. Those
// only surface against a server that reads what actually arrived — which is what
// scratchpad/ingest_probe_server.py does (Content-Length vs bytes received, boundary
// present in the body, all five fields, and the file payload compared byte for byte).
//
// This compiles gateway/src/sd_queue_upload.inc — the REAL ConcatStream and upload_body —
// and drives it through the REAL drain, so the whole path from a queued file on the card
// to bytes on the wire is the shipping code.
//
// Runs on the leaf because the gateway board cannot mount a card (missing D0 pull-up).
// The leaf posts over WiFi; the gateway posts over ethernet. Both ride lwIP under
// HTTPClient, so the body construction — the thing under test — is identical.

#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "leaf_rails.h"
// Generated + gitignored (see leaf/.gitignore). Credentials live in a header rather than
// on the build command line: passed as -D flags they got echoed verbatim by a compiler
// error on 2026-08-09. Regenerate from the gateway's secrets.ini; never commit it.
#include "wire_secrets.h"

static void log_event(const char* fmt, ...) {
    char b[160];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    Serial.printf("    [ev] %s\n", b);
}
static char g_unnoted[48] = "";
static int  g_unnote_calls = 0;
static void have_unnote(const char* eid, const char* /*kind*/) {
    snprintf(g_unnoted, sizeof g_unnoted, "%s", eid ? eid : "");
    g_unnote_calls++;
}
static bool json_str(const char* json, const char* key, char* out, size_t cap) {
    char pat[32]; snprintf(pat, sizeof(pat), "\"%s\"", key);
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
static long long json_ll(const char* json, const char* key) {
    char pat[32]; snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    return atoll(p);
}
static struct { uint32_t uploads_ok = 0; } s_stats;   // upload_body bumps this
static const char* gw_fallback_camera() { return "leaf-wire-fallback"; }

// --- who this harness uploads as ------------------------------------------------------
// sd_queue_upload.inc asks its host for gw_site()/gw_url()/gw_token() and, unless
// GW_NO_CLAIM is defined, for claim_note_ingest_401()/claim_note_ingest_ok(). Those five
// arrived on the gateway with the claim flow (gateway-0.16.0) and live in
// gateway/src/main.cpp, which this harness does not compile — so from that commit until
// this one, leaf-pcb-sdq-wire did not build at all.
//
// GW_NO_CLAIM is the gateway's own name for "a build that must not carry the claim path",
// and that is exactly this build: there is no NVS claim block on a leaf and no claim
// client here to write one. Defining it compiles the accessors down to the baked macros,
// which is where this harness's credentials have always come from — GW_SITE from the env's
// build_flags, GW_INGEST_URL and GW_INGEST_TOKEN from the gitignored wire_secrets.h — and
// it drops the two claim_note_ingest_* calls with them.
//
// WHAT THAT COSTS THE TEST: nothing this harness exists to check. The bytes on the wire,
// Content-Length, the boundary, the five form fields, the file payload and the
// code->outcome mapping are all still the shipping code, posted at the probe server. The
// one gateway behaviour NOT exercised here is the 401 strike counter that triggers an
// unattended re-claim; that path is gateway-only and needs the claim client, so testing it
// belongs on a gateway, not on this board.
#define GW_NO_CLAIM 1
#if !defined(GW_INGEST_URL) || !defined(GW_INGEST_TOKEN)
#error "GW_INGEST_URL / GW_INGEST_TOKEN must come from leaf/src/wire_secrets.h — regenerate it from the gateway's secrets.ini"
#endif
static inline const char* gw_site()  { return GW_SITE; }
static inline const char* gw_token() { return GW_INGEST_TOKEN; }
static inline const char* gw_url()   { return GW_INGEST_URL; }

#define SDQ_FS SD
#define SDQ_FS_MOUNT() SD.begin(LEAF_SD_SPI_CS, SPI)
#define SDQ_NO_PROBE 1
#include "../../gateway/src/sd_queue.inc"
#include "../../gateway/src/sd_queue_upload.inc"   // the REAL ConcatStream + upload_body
#include "../../gateway/src/sd_queue_drain.inc"

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* what) {
    Serial.printf("    %s  %s\n", ok ? "PASS" : "FAIL", what);
    ok ? g_pass++ : g_fail++;
}

static uint8_t g_body[2048];

static void wipe_queue() {
    File d = SD.open(GW_SDQ_DIR);
    if (!d) return;
    char victims[64][40];
    int n = 0;
    for (File f = d.openNextFile(); f && n < 64; f = d.openNextFile()) {
        snprintf(victims[n++], sizeof victims[0], "%s", f.path());
        f.close();
    }
    d.close();
    for (int i = 0; i < n; i++) SD.remove(victims[i]);
    sdq_scan(nullptr, nullptr, nullptr, true);
}

void setup() {
    Serial.begin(LEAF_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(50);
    delay(300);
    // A known ramp, so the server can compare the payload byte for byte.
    for (size_t i = 0; i < sizeof g_body; i++) g_body[i] = (uint8_t)(i % 256);

    Serial.println();
    Serial.println("==================================================");
    Serial.println(" WIRE FORMAT test (real multipart -> probe server)");
    Serial.println(" posting gateway/src/sd_queue_upload.inc verbatim");
    Serial.println("==================================================");

    // Try every configured network, same as the gateway's multi-network fallback: net 1 is
    // the NORTH40 SSID and this board is on the home bench, so a single-network attempt
    // just times out. disconnect(true) between tries because a STA left "connecting" makes
    // the next begin() fail with ESP_ERR_WIFI_STATE (the 2026-07-21 gateway wedge).
    struct { const char* ssid; const char* psk; } nets[] = {
        { GW_WIFI_SSID,  GW_WIFI_PSK  },
#ifdef GW_WIFI_SSID2
        { GW_WIFI_SSID2, GW_WIFI_PSK2 },
#endif
    };
    WiFi.mode(WIFI_STA);
    for (auto& n : nets) {
        Serial.printf("  joining WiFi \"%s\" ...\n", n.ssid);
        WiFi.disconnect(true);
        delay(200);
        WiFi.begin(n.ssid, n.psk);
        t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
        if (WiFi.status() == WL_CONNECTED) break;
        Serial.println("    no join, trying the next network");
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("  WiFi FAILED on every network — cannot run the wire test");
        return;
    }
    Serial.printf("  ip %s, rssi %d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

    // upload_body refuses to post before the clock is NTP-synced (a real guard, not a
    // test artifact: captured_at would be nonsense). Satisfy it the same way the gateway
    // does rather than special-casing the code under test.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    t0 = millis();
    while (time(nullptr) < 1600000000 && millis() - t0 < 20000) delay(250);
    Serial.printf("  clock %s (epoch %ld)\n",
                  time(nullptr) > 1600000000 ? "synced" : "NOT SYNCED", (long)time(nullptr));

    leaf_radio_rail(true);
    delay(50);
    SPI.begin(47, RADIO_MISO_PIN, 48, LEAF_SD_SPI_CS);
    if (!SD.begin(LEAF_SD_SPI_CS, SPI)) {
        Serial.println("  SD mount FAILED — cannot run the wire test");
        return;
    }
    if (!SD.exists(GW_SDQ_DIR)) SD.mkdir(GW_SDQ_DIR);
    s_sdq.mounted = true;
    wipe_queue();

    Serial.println("\nWIRE 1 - a queued THUMB posts as a well-formed multipart body");
    sdq_enqueue("wire-thumb-01", "leaf-wire-cam", "thumb",
                (long long)time(nullptr), g_body, nullptr, 512);
    sdq_scan(nullptr, nullptr, nullptr, true);
    s_stats.uploads_ok = 0;
    bool did = sdq_drain_one();
    check(did, "drain posted the frame and acted on the reply");
    check(s_stats.uploads_ok == 1, "the server returned 2xx");
    sdq_scan(nullptr, nullptr, nullptr, true);
    check(s_sdq.depth == 0, "the frame was deleted only after that 2xx");

    Serial.println("\nWIRE 2 - a queued FULL streams from the card at the right offset");
    sdq_enqueue("wire-full-02", "leaf-wire-cam", "full",
                (long long)time(nullptr), g_body, nullptr, sizeof g_body);
    sdq_scan(nullptr, nullptr, nullptr, true);
    s_stats.uploads_ok = 0;
    did = sdq_drain_one();
    check(did, "drain posted the larger body");
    check(s_stats.uploads_ok == 1, "the server accepted it");
    sdq_scan(nullptr, nullptr, nullptr, true);
    check(s_sdq.depth == 0, "and it was removed");

    Serial.println("\nWIRE 3 - a 4xx from the SERVER is treated as permanent");
    Serial.println("    (stage it with: curl -sk 'https://<probe>/control?code=422')");
    sdq_enqueue("wire-4xx-03", "leaf-wire-cam", "thumb",
                (long long)time(nullptr), g_body, nullptr, 256);
    g_unnote_calls = 0;
    sdq_scan(nullptr, nullptr, nullptr, true);
    did = sdq_drain_one();
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    drain=%d depth=%lu unnote_calls=%d eid='%s'\n",
                  (int)did, (unsigned long)s_sdq.depth, g_unnote_calls, g_unnoted);
    Serial.println("    ^ if the server was staged to 422: expect depth 0 and 1 unnote.");
    Serial.println("      if it was left at 200: expect depth 0 and 0 unnote.");

    Serial.println();
    Serial.printf("RESULT: %d passed, %d FAILED\n", g_pass, g_fail);
    Serial.println("The BYTES are judged by the probe server's log, not here — this side");
    Serial.println("only sees the status code. Read ingest_probe_log.jsonl for the verdict.");
    Serial.println("==================================================");
}

void loop() { delay(1000); }
