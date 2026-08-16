// Upload-queue acceptance gates, run on the custom leaf PCB's microSD.
//
// WHY HERE AND NOT ON THE GATEWAY: the gateway board (T-Internet-POE) is permanently
// depth-1 — its TF slot is missing the D0 pull-up, measured 2026-08-09 (homelab
// docs/trailcam/gateway-pcb.md). It cannot mount a card at all, so the gates cannot run
// there. The leaf PCB's card works (bring-up gate 3), so it is the only hardware on the
// project that can exercise this code.
//
// It compiles gateway/src/sd_queue.inc — THE REAL QUEUE, not a copy. Only the
// filesystem object differs (SPI-mode SD here, SD_MMC on the gateway) via SDQ_FS.
//
// ⚠️ WHAT THIS DOES **NOT** COVER, so nobody reads a green run as "queue validated":
//   * gate 4 (card removed -> depth-1 + alarm) is an SD_MMC behaviour. The empty-slot
//     BUSY-SPIN that wedges the gateway does NOT reproduce over SPI, so the containment
//     fix (pinned probe task, idle-WDT, timeout) is NOT exercised here. It was verified
//     separately on the gateway board.
//   * gate 5's telemetry beat and web-ring rendering are gateway-only. This checks the
//     underlying numbers (depth, oldest age) are right, not that they are published.
//   * the upload half (sdq_drain_one -> HTTP, delete-only-after-2xx against a real
//     server, 4xx custody revoke) is gateway-only. Drain ORDER is checked here using the
//     same sdq_scan() the uploader picks with, which is the part that can be wrong.

#include <SPI.h>
#include <SD.h>
#include "leaf_rails.h"

// --- shims the queue expects from its host translation unit ---------------------------
static void log_event(const char* fmt, ...) {
    char b[160];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    Serial.printf("    [ev] %s\n", b);
}
// Custody revoke is a gateway/mesh concern (the dedup "have" ring). Here it is recorded,
// because "was custody revoked on a 4xx?" is exactly what the drain gates assert: a
// dropped frame whose custody is NOT revoked makes the leaf dedup-skip every future
// re-send, and the photo is lost with all three ends sure someone else had it.
static char g_unnoted[48] = "";
static int  g_unnote_calls = 0;
static void have_unnote(const char* eid, const char* /*kind*/) {
    snprintf(g_unnoted, sizeof g_unnoted, "%s", eid ? eid : "");
    g_unnote_calls++;
}

// Byte-for-byte the gateway's parsers — the queue writes and re-reads its own JSON header
// line, and gate 3's survive-a-reboot check depends on that round-trip being identical.
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
static long long json_ll(const char* json, const char* key) {
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    return atoll(p);
}

#define SDQ_FS SD
// SPI-mode mount instead of SD_MMC's 1-bit call. Everything else in the mount path —
// including the TMP.TMP purge gate 3 checks — stays shared.
#define SDQ_FS_MOUNT() SD.begin(LEAF_SD_SPI_CS, SPI)
// The async probe + idle-watchdog containment guards against SD_MMC busy-spinning on an
// empty slot. That does not happen over SPI, and it was verified on the gateway board.
#define SDQ_NO_PROBE 1
#include "../../gateway/src/sd_queue.inc"

// --- scripted uploader --------------------------------------------------------------
// The real drain state machine (sd_queue_drain.inc) is compiled below and driven with
// injected outcomes. That tests the DECISIONS — keep on an ambiguous failure, delete on
// 2xx, revoke-then-delete on 4xx, drop an unreadable entry instead of head-of-line
// blocking — which is where a bug loses photos. It does NOT test the multipart body
// upload_body() builds on the gateway; that needs a real server.
enum UploadOutcome { UP_RETRY = 0, UP_OK, UP_DROP };

static UploadOutcome g_next_outcome = UP_OK;
static int  g_upload_calls = 0;
static char g_last_eid[48] = "", g_last_kind[16] = "";

static UploadOutcome upload_body(const char* eid, const char* /*cam*/, const char* kind,
                                 long long /*capts*/, const uint8_t* /*buf*/,
                                 File* src, size_t len) {
    g_upload_calls++;
    snprintf(g_last_eid,  sizeof g_last_eid,  "%s", eid  ? eid  : "");
    snprintf(g_last_kind, sizeof g_last_kind, "%s", kind ? kind : "");
    // Read the body the way the real uploader streams it, so a wrong seek or a bad
    // declared length still shows up here rather than passing silently.
    if (src && len) {
        size_t got = 0; uint8_t tmp[128];
        while (got < len) {
            int n = src->read(tmp, sizeof tmp);
            if (n <= 0) break;
            got += (size_t)n;
        }
        if (got != len) {
            Serial.printf("    [up] BODY SHORT: declared %u, streamed %u\n",
                          (unsigned)len, (unsigned)got);
            return UP_RETRY;
        }
    }
    return g_next_outcome;
}

static const char* gw_fallback_camera() { return "leaf-gate-fallback"; }

#include "../../gateway/src/sd_queue_drain.inc"

// --- tiny test scaffolding ------------------------------------------------------------
static int g_pass = 0, g_fail = 0;

static void check(bool ok, const char* what) {
    Serial.printf("    %s  %s\n", ok ? "PASS" : "FAIL", what);
    ok ? g_pass++ : g_fail++;
}

static void wipe_queue() {
    File d = SDQ_FS.open(GW_SDQ_DIR);
    if (!d) return;
    char victims[64][40];
    int n = 0;
    for (File f = d.openNextFile(); f && n < 64; f = d.openNextFile()) {
        snprintf(victims[n++], sizeof victims[0], "%s", f.path());
        f.close();
    }
    d.close();
    for (int i = 0; i < n; i++) SDQ_FS.remove(victims[i]);
    sdq_scan(nullptr, nullptr, nullptr, true);
}

// A frame body big enough to be realistic but small enough that 100 of them are quick.
static uint8_t g_body[1024];

static bool put(uint32_t capts, const char* kind, const char* eid) {
    return sdq_enqueue(eid, "leaf-gate", kind, (long long)capts,
                       g_body, nullptr, sizeof g_body);
}

// ---------------------------------------------------------------------------------------
static void gate1_outage_keeps_every_frame() {
    Serial.println("\nGATE 1 - an outage must lose NOTHING (the actual defect)");
    wipe_queue();
    // Must stay UNDER GW_SDQ_MAX_FILES: this gate is about an outage losing nothing, and
    // the cap is deliberately tiny here so gate 6 can trip eviction quickly. Overrunning
    // it makes the queue evict correctly and this gate call it data loss. Twenty is
    // plenty — the defect being tested delivered exactly ONE.
    const int N = (GW_SDQ_MAX_FILES > 20) ? 20 : (GW_SDQ_MAX_FILES - 2);
    int enq = 0;
    for (int i = 0; i < N; i++) {
        char eid[24]; snprintf(eid, sizeof eid, "e%04d", i);
        // alternate thumb/full like a real capture pair
        if (put(1700000000u + i * 10, (i % 2) ? "full" : "thumb", eid)) enq++;
    }
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    enqueued %d/%d, queue depth now %lu\n",
                  enq, N, (unsigned long)s_sdq.depth);
    check(enq == N, "every frame written while the uplink is down");
    check((int)s_sdq.depth == N, "all of them are still on the card (none evicted)");
    check(s_sdq.thumbs + s_sdq.fulls == s_sdq.depth, "thumb/full accounting adds up");
}

static void gate2_drain_order() {
    Serial.println("\nGATE 2 - drain order via the REAL drain (every upload a 2xx)");
    g_next_outcome = UP_OK;
    uint32_t last = 0xFFFFFFFF;
    int steps = 0, order_ok = 1;
    while (steps < 200) {
        SdqPick n{0, false, false};
        sdq_scan(&n, nullptr, nullptr, false);
        if (!n.valid) break;
        if (n.key > last) order_ok = 0;               // must never jump back to newer
        last = n.key;
        if (!sdq_drain_one()) break;
        steps++;
    }
    Serial.printf("    drained %d entries in %d upload calls\n", steps, g_upload_calls);
    check(order_ok == 1, "never drained an older frame before a newer one");
    sdq_scan(nullptr, nullptr, nullptr, true);
    check(s_sdq.depth == 0, "card ends EMPTY after a full 2xx drain");
}

static void gate2b_ambiguous_keeps_the_frame() {
    Serial.println("\nGATE 2b - an AMBIGUOUS failure must KEEP the frame (the whole point)");
    wipe_queue();
    put(1700003000u, "thumb", "amb01");
    sdq_scan(nullptr, nullptr, nullptr, true);
    const uint32_t before = s_sdq.depth;

    g_next_outcome = UP_RETRY;                        // e.g. 5xx / dropped connection
    g_upload_calls = 0;
    const bool did = sdq_drain_one();
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    depth %lu -> %lu after a failed upload (drain_one returned %d)\n",
                  (unsigned long)before, (unsigned long)s_sdq.depth, (int)did);
    check(g_upload_calls == 1, "the upload was actually attempted");
    check(s_sdq.depth == before, "the frame is STILL on the card after a failure");
    check(did == false, "drain reports 'no progress' so the caller backs off");

    g_next_outcome = UP_OK;                           // pipe comes back
    check(sdq_drain_one() == true, "the retry succeeds once the uplink returns");
    sdq_scan(nullptr, nullptr, nullptr, true);
    check(s_sdq.depth == before - 1, "and only THEN is it deleted");
}

static void gate2c_permanent_4xx_revokes_custody() {
    Serial.println("\nGATE 2c - a permanent 4xx drops the frame AND revokes custody");
    wipe_queue();
    put(1700004000u, "full", "perm42");
    g_next_outcome  = UP_DROP;
    g_unnote_calls  = 0;
    g_unnoted[0]    = '\0';
    const bool did  = sdq_drain_one();
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    unnote calls=%d eid='%s', depth now %lu\n",
                  g_unnote_calls, g_unnoted, (unsigned long)s_sdq.depth);
    check(did == true, "drain made progress");
    check(s_sdq.depth == 0, "the 4xx frame was removed (not retried forever)");
    check(g_unnote_calls == 1 && strcmp(g_unnoted, "perm42") == 0,
          "custody was REVOKED so the leaf may re-send it");
    g_next_outcome = UP_OK;
}

static void gate2d_torn_entry_does_not_block() {
    Serial.println("\nGATE 2d - an unreadable entry is dropped, not head-of-line blocking");
    wipe_queue();
    // A file with a valid queue NAME but garbage inside: exactly what a torn rename or a
    // corrupted card yields. It can never become a valid POST.
    File bad = SDQ_FS.open(GW_SDQ_DIR "/7FFFFFFF.QF", FILE_WRITE);
    if (bad) { bad.write((const uint8_t*)"not json at all\n", 16); bad.close(); }
    put(1700005000u, "thumb", "good01");             // older, so it drains second
    sdq_scan(nullptr, nullptr, nullptr, true);
    const uint32_t before = s_sdq.depth;
    g_upload_calls = 0;
    g_next_outcome = UP_OK;

    const bool did = sdq_drain_one();                // newest = the bad one
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    depth %lu -> %lu, uploads attempted %d\n",
                  (unsigned long)before, (unsigned long)s_sdq.depth, g_upload_calls);
    check(did == true, "drain made progress on the bad entry");
    check(g_upload_calls == 0, "it was never uploaded (unreadable, so no POST)");
    check(s_sdq.depth == before - 1, "the bad entry was removed");
    check(sdq_drain_one() == true, "the good frame behind it still drains");
}

static void gate3_reboot_keeps_queue_drops_partial() {
    Serial.println("\nGATE 3 - a reboot KEEPS finished frames, DROPS a torn write");
    wipe_queue();
    for (int i = 0; i < 5; i++) {
        char eid[24]; snprintf(eid, sizeof eid, "r%04d", i);
        put(1700001000u + i * 10, (i % 2) ? "full" : "thumb", eid);
    }
    // Simulate a power cut mid-assembly: a TMP.TMP left behind.
    File t = SDQ_FS.open(GW_SDQ_TMP, FILE_WRITE);
    if (t) { t.write(g_body, 256); t.close(); }
    check(SDQ_FS.exists(GW_SDQ_TMP), "a torn partial exists before the reboot");

    // Remount is exactly what boot does.
    sdq_mount_blocking();
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    after remount: depth %lu\n", (unsigned long)s_sdq.depth);
    check(s_sdq.depth == 5, "all 5 completed frames SURVIVED the reboot");
    check(!SDQ_FS.exists(GW_SDQ_TMP), "the torn partial was purged");
}

static void gate5_numbers() {
    Serial.println("\nGATE 5 - depth + oldest-age are correct (publishing is gateway-only)");
    sdq_scan(nullptr, nullptr, nullptr, true);
    long age = sdq_oldest_age_s();
    Serial.printf("    depth=%lu thumbs=%lu fulls=%lu oldest_key=%lu age=%ld free=%lluMB\n",
                  (unsigned long)s_sdq.depth, (unsigned long)s_sdq.thumbs,
                  (unsigned long)s_sdq.fulls, (unsigned long)s_sdq.oldest_key,
                  age, (unsigned long long)(s_sdq.free_bytes >> 20));
    check(s_sdq.depth > 0 && s_sdq.oldest_key != 0, "depth and oldest-key are populated");
    check(s_sdq.newest_key >= s_sdq.oldest_key, "newest is not older than oldest");
    check(age == -1 || age >= 0, "oldest-age is -1 (no clock) or a sane number");
}

static void gate6_eviction_prefers_fulls() {
    Serial.println("\nGATE 6 - at the cap: oldest FULLS go first, thumbs are kept");
    Serial.printf("    (harness cap GW_SDQ_MAX_FILES=%d so this is quick)\n",
                  (int)GW_SDQ_MAX_FILES);
    wipe_queue();
    // Oldest entries are fulls, newest are thumbs; overflow must eat the fulls.
    for (int i = 0; i < GW_SDQ_MAX_FILES; i++) {
        char eid[24]; snprintf(eid, sizeof eid, "f%04d", i);
        put(1700002000u + i * 10, "full", eid);
    }
    sdq_scan(nullptr, nullptr, nullptr, true);
    uint32_t fulls_before = s_sdq.fulls;
    for (int i = 0; i < 4; i++) {
        char eid[24]; snprintf(eid, sizeof eid, "t%04d", i);
        put(1700009000u + i * 10, "thumb", eid);
    }
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("    fulls %lu -> %lu, thumbs %lu, depth %lu, evicted %lu\n",
                  (unsigned long)fulls_before, (unsigned long)s_sdq.fulls,
                  (unsigned long)s_sdq.thumbs, (unsigned long)s_sdq.depth,
                  (unsigned long)s_sdq.evicted);
    check(s_sdq.depth <= (uint32_t)GW_SDQ_MAX_FILES, "queue stayed within its cap");
    check(s_sdq.evicted > 0, "eviction actually fired");
    check(s_sdq.thumbs == 4, "every thumb was retained");
    check(s_sdq.fulls < fulls_before, "the fulls are what got evicted");
}

void setup() {
    Serial.begin(LEAF_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(50);
    delay(300);
    for (size_t i = 0; i < sizeof g_body; i++) g_body[i] = (uint8_t)i;

    Serial.println();
    Serial.println("==================================================");
    Serial.println(" QUEUE GATES on the leaf PCB (SPI-mode SD)");
    Serial.println(" running gateway/src/sd_queue.inc — the real code");
    Serial.println("==================================================");

    leaf_radio_rail(true);            // the card is unpowered until this goes LOW
    delay(50);
    SPI.begin(47, RADIO_MISO_PIN, 48, LEAF_SD_SPI_CS);
    if (!SD.begin(LEAF_SD_SPI_CS, SPI)) {
        Serial.println("  SD mount FAILED — cannot run the gates (rail on? card in?)");
        return;
    }
    if (!SD.exists(GW_SDQ_DIR)) SD.mkdir(GW_SDQ_DIR);
    s_sdq.mounted = true;
    sdq_scan(nullptr, nullptr, nullptr, true);
    Serial.printf("  card %llu MB, %llu MB free\n",
                  (unsigned long long)(SD.cardSize() >> 20),
                  (unsigned long long)(s_sdq.free_bytes >> 20));

    gate1_outage_keeps_every_frame();
    gate2_drain_order();
    gate2b_ambiguous_keeps_the_frame();
    gate2c_permanent_4xx_revokes_custody();
    gate2d_torn_entry_does_not_block();
    gate3_reboot_keeps_queue_drops_partial();
    gate5_numbers();
    gate6_eviction_prefers_fulls();

    Serial.println();
    Serial.printf("RESULT: %d passed, %d FAILED\n", g_pass, g_fail);
    Serial.println("NOT covered here: gate 4 (empty-slot containment) is SD_MMC-only and");
    Serial.println("was verified on the gateway board. The multipart body upload_body()");
    Serial.println("builds (ConcatStream/Content-Length/HTTPClient) needs a real server —");
    Serial.println("these gates inject the OUTCOME to test the drain decisions.");
    Serial.println("==================================================");
}

void loop() { delay(1000); }
