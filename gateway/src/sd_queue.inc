// SD-backed upload queue — the storage half of the store-and-forward path.
//
// Included by gateway/src/main.cpp (SD_MMC on the gateway) and by the leaf harness
// (leaf/src/sdq_gates.inc, SPI-mode SD on the custom leaf PCB) so BOTH COMPILE THE SAME
// CODE. The point of the leaf harness is to exercise this file on real hardware; a
// re-implementation would test nothing. The gateway board cannot host these tests — its
// TF slot is missing the D0 pull-up (homelab docs/trailcam/gateway-pcb.md).
//
// The filesystem object is the ONLY thing that varies. Define SDQ_FS (and include the
// matching FS header) before including this; otherwise it defaults to SD_MMC.
//
// Requires from the including translation unit:
//   log_event(fmt, ...)   — human-readable event ring
//   have_unnote(eid)      — revoke custody so a leaf may re-send (the 4xx path)

#ifndef SDQ_FS
#include <SD_MMC.h>
#define SDQ_FS SD_MMC
#endif

// The mount CALL differs between backends (SD_MMC takes a 1-bit flag, SPI-mode SD takes
// a CS pin and a bus) — everything around it, including the TMP.TMP purge that gate 3
// checks, is shared. Hosts on a different backend override this one macro.
#ifndef SDQ_FS_MOUNT
#define SDQ_FS_MOUNT() SDQ_FS.begin("/sdcard", /*mode1bit=*/true, /*format_if_empty=*/false)
#endif
#include <ctype.h>

#define GW_SDQ_DIR       "/q"
#define GW_SDQ_TMP       "/q/TMP.TMP"
// Overridable so the leaf gate harness can trip the eviction paths at a sane scale:
// the real guard is 32 MB against a 64 GB card, so honouring it verbatim would mean
// writing ~59 GB to see an eviction. The LOGIC under test is identical either way.
#ifndef GW_SDQ_MAX_FILES
#define GW_SDQ_MAX_FILES 512                     // dir-scan cost ceiling, not a space one
#endif
#ifndef GW_SDQ_MIN_FREE
#define GW_SDQ_MIN_FREE  (32ull * 1024 * 1024)   // never fill the card to the rim
#endif
#define GW_SDQ_HDR_MAX   256

static struct {
    bool     mounted   = false;
    bool     announced = false;   // "no card" alarm already logged (log once per state)
    uint32_t mount_ms  = 0;       // last mount attempt (0 = never)
    uint32_t stats_ms  = 0, free_ms = 0;
    uint32_t depth = 0, thumbs = 0, fulls = 0, strays = 0;
    uint32_t oldest_key = 0, newest_key = 0;
    uint32_t evicted = 0, write_errors = 0, read_errors = 0, mount_fails = 0;
    uint8_t  consec_write_errors = 0;
    uint64_t card_bytes = 0, free_bytes = 0;
} s_sdq;

struct SdqPick { uint32_t key; bool full; bool valid; };   // POD: brace-init on gnu++11

static void sdq_set(SdqPick* p, uint32_t key, bool full) {
    p->key = key; p->full = full; p->valid = true;
}

static void sdq_name(char* out, size_t n, uint32_t key, bool full) {
    snprintf(out, n, GW_SDQ_DIR "/%08lX.Q%c", (unsigned long)key, full ? 'F' : 'T');
}

// "0068C1A3.QT" -> key + kind. Anything else in /q is NOT a queue entry (see atomic
// rename above), so it is counted as a stray and otherwise ignored.
static bool sdq_parse(const char* path, uint32_t* key, bool* full) {
    if (!path) return false;
    const char* n = strrchr(path, '/');
    n = n ? n + 1 : path;
    if (strlen(n) != 11 || n[8] != '.' || n[9] != 'Q') return false;
    for (int i = 0; i < 8; i++) if (!isxdigit((unsigned char)n[i])) return false;
    if      (n[10] == 'T') *full = false;
    else if (n[10] == 'F') *full = true;
    else return false;
    *key = (uint32_t)strtoul(n, nullptr, 16);
    return true;
}

// f_getfree walks the FAT, so it is the one genuinely slow call here: throttle it.
static void sdq_refresh_free(bool force) {
    if (!force && s_sdq.free_ms && millis() - s_sdq.free_ms < 30 * 1000u) return;
    s_sdq.free_ms = millis() | 1u;
    const uint64_t total = SDQ_FS.totalBytes(), used = SDQ_FS.usedBytes();
    s_sdq.card_bytes = total;
    s_sdq.free_bytes = total > used ? total - used : 0;
}

// ONE pass over /q: the depth/oldest/newest counters plus the three entries callers need
// (newest to drain, oldest full and oldest thumb to evict). Index-free by design.
static void sdq_scan(SdqPick* newest, SdqPick* oldest_full, SdqPick* oldest_thumb,
                     bool with_free) {
    if (newest)       *newest       = SdqPick();
    if (oldest_full)  *oldest_full  = SdqPick();
    if (oldest_thumb) *oldest_thumb = SdqPick();
    s_sdq.depth = s_sdq.thumbs = s_sdq.fulls = s_sdq.strays = 0;
    s_sdq.oldest_key = s_sdq.newest_key = 0;
    if (!s_sdq.mounted) return;
    File d = SDQ_FS.open(GW_SDQ_DIR);
    if (!d || !d.isDirectory()) {
        if (d) d.close();
        Serial.println("[gw] sd queue: /q unreadable -> card marked bad");
        s_sdq.mounted = false;
        return;
    }
    for (File e = d.openNextFile(); e; e = d.openNextFile()) {
        uint32_t key = 0; bool full = false;
        const bool ok = !e.isDirectory() && sdq_parse(e.name(), &key, &full);
        e.close();
        if (!ok) { s_sdq.strays++; continue; }
        s_sdq.depth++;
        if (full) s_sdq.fulls++; else s_sdq.thumbs++;
        if (!s_sdq.oldest_key || key < s_sdq.oldest_key) s_sdq.oldest_key = key;
        if (key > s_sdq.newest_key) s_sdq.newest_key = key;
        // newest wins; at equal capture time the THUMB goes first (it is the frame the
        // customer actually sees, and it is 25x cheaper to push through a thin pipe).
        if (newest && (!newest->valid || key > newest->key ||
                       (key == newest->key && newest->full && !full)))
            sdq_set(newest, key, full);
        if (full && oldest_full && (!oldest_full->valid || key < oldest_full->key))
            sdq_set(oldest_full, key, full);
        if (!full && oldest_thumb && (!oldest_thumb->valid || key < oldest_thumb->key))
            sdq_set(oldest_thumb, key, full);
    }
    d.close();
    if (with_free) sdq_refresh_free(/*force=*/false);
}

// Read a queue entry's header line. Leaves the handle positioned anywhere; callers seek.
static bool sdq_read_header(File& f, char* eid, size_t neid, char* cam, size_t ncam,
                            char* kind, size_t nkind, long long* capts, size_t* len,
                            size_t* hdr_len) {
    char buf[GW_SDQ_HDR_MAX + 1];
    if (!f.seek(0)) return false;
    const size_t got = f.read((uint8_t*)buf, GW_SDQ_HDR_MAX);
    if (!got) return false;
    buf[got] = '\0';
    const char* nl = (const char*)memchr(buf, '\n', got);
    if (!nl || buf[0] != '{') return false;
    *hdr_len = (size_t)(nl - buf);
    buf[*hdr_len] = '\0';
    if (!json_str(buf, "event_id", eid, neid) || !eid[0]) return false;
    json_str(buf, "camera", cam, ncam);
    json_str(buf, "kind", kind, nkind);
    *capts = json_ll(buf, "captured_at");
    *len   = (size_t)json_ll(buf, "len");
    // The declared length must match what is actually on the card, or the multipart
    // Content-Length would lie and the POST would hang/422 (a torn write, caught here).
    return *len && f.size() == *hdr_len + 1 + *len;
}

// Delete one entry, revoking dedup custody first: a queued frame we drop never reached
// the server, and a "have" ring that still claims it would make the leaf dedup-skip
// every future re-send — the photo lost with all three ends sure someone else had it.
static void sdq_evict(const SdqPick& p, const char* why) {
    char path[32];
    sdq_name(path, sizeof(path), p.key, p.full);
    char eid[48] = "", cam[48] = "", kind[16] = "";
    long long capts = 0; size_t len = 0, hdr = 0;
    File f = SDQ_FS.open(path, FILE_READ);
    const size_t bytes = f ? f.size() : 0;
    if (f) {
        if (sdq_read_header(f, eid, sizeof(eid), cam, sizeof(cam), kind, sizeof(kind),
                            &capts, &len, &hdr))
            have_unnote(eid, kind);
        f.close();
    }
    SDQ_FS.remove(path);
    s_sdq.evicted++;
    s_sdq.free_bytes += bytes;
    if (s_sdq.depth) s_sdq.depth--;
    if (p.full) { if (s_sdq.fulls)  s_sdq.fulls--;  }
    else        { if (s_sdq.thumbs) s_sdq.thumbs--; }
    Serial.printf("[gw] sd queue EVICT %s (%s %s, %u B): %s\n", path,
                  eid[0] ? eid : "?", p.full ? "full" : "thumb", (unsigned)bytes, why);
    log_event("sd queue EVICT %s %s (%s)", eid[0] ? eid : path,
              p.full ? "full" : "thumb", why);
}

// Make room for `bytes`. Oldest FULLS go first; a thumb is only evicted once no full is
// left, and NEVER to make room for a full — the thumb is often the only frame a site
// will ever get out, so trading it for a 25x-larger original is a bad trade.
static bool sdq_make_room(size_t bytes, bool incoming_full) {
    sdq_refresh_free(/*force=*/true);
    for (int guard = 0; guard <= GW_SDQ_MAX_FILES + 2; guard++) {
        SdqPick newest, oldest_full, oldest_thumb;
        sdq_scan(&newest, &oldest_full, &oldest_thumb, /*with_free=*/false);
        if (!s_sdq.mounted) return false;
        if (s_sdq.depth + 1 <= GW_SDQ_MAX_FILES &&
            s_sdq.free_bytes >= (uint64_t)bytes + GW_SDQ_MIN_FREE)
            return true;
        if (oldest_full.valid) { sdq_evict(oldest_full, "queue full"); continue; }
        if (incoming_full) {
            Serial.println("[gw] sd queue holds only thumbs -> DROPPING the incoming "
                           "full (a thumb is never traded for a full)");
            log_event("sd queue full of thumbs -> incoming full dropped");
            return false;
        }
        if (oldest_thumb.valid) { sdq_evict(oldest_thumb, "queue full"); continue; }
        Serial.printf("[gw] sd queue: nothing to evict and only %llu B free -> card is "
                      "full of foreign data\n", (unsigned long long)s_sdq.free_bytes);
        log_event("sd card FULL with no queue entries to evict");
        return false;
    }
    return false;
}

// Take custody of a frame: header + body -> TMP.TMP, verify, then atomic rename in.
// Body comes from a RAM buffer (thumbs) or an open File (a completed LittleFS assembly);
// either way the heap never holds a full. Runs in RNS dispatch context like the existing
// chunk writes do — after the transfer has concluded, never between its packets.
static bool sdq_enqueue(const char* eid, const char* cam, const char* kind,
                        long long capts, const uint8_t* buf, File* src, size_t len) {
    if (!s_sdq.mounted || !len) return false;
    const bool full = strcmp(kind, "full") == 0;
    char hdr[GW_SDQ_HDR_MAX];
    const int hl = snprintf(hdr, sizeof(hdr),
                            "{\"event_id\":\"%s\",\"camera\":\"%s\",\"kind\":\"%s\","
                            "\"captured_at\":%lld,\"len\":%u}\n",
                            eid, cam, kind, capts, (unsigned)len);
    if (hl <= 0 || hl >= (int)sizeof(hdr)) return false;
    if (!sdq_make_room(len + (size_t)hl, full)) return false;

    if (SDQ_FS.exists(GW_SDQ_TMP)) SDQ_FS.remove(GW_SDQ_TMP);
    bool ok = false;
    {
        File t = SDQ_FS.open(GW_SDQ_TMP, FILE_WRITE);
        if (!t) {
            Serial.println("[gw] sd queue: TMP open FAILED");
        } else if (t.write((const uint8_t*)hdr, (size_t)hl) != (size_t)hl) {
            Serial.println("[gw] sd queue: header write FAILED");
            t.close();
        } else if (buf) {
            ok = t.write(buf, len) == len;
            t.close();
            if (!ok) Serial.println("[gw] sd queue: body write FAILED");
        } else if (src) {
            uint8_t* io = (uint8_t*)malloc(1024);
            if (!io) { t.close(); Serial.println("[gw] sd queue: OOM on copy buffer"); }
            else {
                size_t done = 0;
                ok = src->seek(0);
                while (ok && done < len) {
                    const size_t want = len - done < 1024 ? len - done : 1024;
                    const size_t got  = src->read(io, want);
                    if (!got || t.write(io, got) != got) { ok = false; break; }
                    done += got;
                }
                t.close();
                free(io);
                if (!ok)
                    Serial.printf("[gw] sd queue: copy FAILED at %u/%u\n",
                                  (unsigned)done, (unsigned)len);
            }
        } else {
            t.close();
        }
    }
    if (ok) {   // belt: the size on the card must match what we meant to write
        File v = SDQ_FS.open(GW_SDQ_TMP, FILE_READ);
        const size_t vs = v ? v.size() : 0;
        if (v) v.close();
        if (vs != (size_t)hl + len) {
            Serial.printf("[gw] sd queue: TMP is %u B, expected %u -> discarded\n",
                          (unsigned)vs, (unsigned)((size_t)hl + len));
            ok = false;
        }
    }
    if (!ok) {
        SDQ_FS.remove(GW_SDQ_TMP);
        s_sdq.write_errors++;
        // Three failures in a row is a sick card, not a hiccup: drop to the depth-1
        // fallback and re-mount, LOUDLY, rather than quietly writing into a void.
        if (++s_sdq.consec_write_errors >= 3) {
            Serial.println("[gw] *** sd queue: 3 consecutive write failures -> card "
                           "marked BAD, falling back to the depth-1 slot ***");
            log_event("sd card WRITE FAILURES -> depth-1 fallback");
            s_sdq.mounted = false;
            s_sdq.announced = false;
        }
        return false;
    }
    s_sdq.consec_write_errors = 0;

    // Name it by capture time so the directory order IS the drain order. An unsynced
    // clock has no ordering to offer, so such frames sort as oldest (they drain last and
    // are evicted first) — honest, and loud in the log.
    uint32_t key;
    if (capts >= 1600000000LL) {
        key = (uint32_t)capts;
    } else {
        const time_t now = time(nullptr);
        key = now > 1600000000 ? (uint32_t)now : (uint32_t)(1600000000u + millis() / 1000);
        Serial.printf("[gw] sd queue: %s has no capture time -> keyed %08lX\n",
                      eid, (unsigned long)key);
    }
    char path[32];
    for (int i = 0; i < 64; i++) {   // same-second collisions (thumb+full, or a burst)
        sdq_name(path, sizeof(path), key, full);
        if (!SDQ_FS.exists(path)) break;
        key++;
    }
    if (!SDQ_FS.rename(GW_SDQ_TMP, path)) {
        Serial.printf("[gw] sd queue: rename -> %s FAILED\n", path);
        SDQ_FS.remove(GW_SDQ_TMP);
        s_sdq.write_errors++;
        return false;
    }
    s_sdq.depth++;
    if (full) s_sdq.fulls++; else s_sdq.thumbs++;
    if (key > s_sdq.newest_key) s_sdq.newest_key = key;
    if (!s_sdq.oldest_key || key < s_sdq.oldest_key) s_sdq.oldest_key = key;
    if (s_sdq.free_bytes > (uint64_t)hl + len) s_sdq.free_bytes -= (uint64_t)hl + len;
    Serial.printf("[gw] sd queue + %s (%s %s, %u B) — depth %lu (%lu thumb / %lu full)\n",
                  path, eid, kind, (unsigned)len, (unsigned long)s_sdq.depth,
                  (unsigned long)s_sdq.thumbs, (unsigned long)s_sdq.fulls);
    return true;
}

// Mount (or re-mount) the card. Never fatal: a gateway with no card must still run, it
// just runs with the old depth-1 exposure — and says so.
//
// ⚠️ THIS CALL CAN BLOCK FOREVER — never call it directly. Use the async probe below.
// Bench-proven 2026-08-09 on a T-Internet-POE: with the TF slot physically EMPTY,
// SDQ_FS.begin() never returns, so the `if (!SDQ_FS.begin(...))` guard underneath is
// unreachable and the whole "never fatal" contract above is a lie. (With a card fitted
// it fails fast and correctly — even a card that never initialises returns 0x107 in
// ~45 ms.) The Arduino wrapper hardcodes SDMMC_SLOT_NO_CD, so the driver has no
// card-detect line to tell it the slot is empty; esp-idf's slot_config.cd exists but
// SDQ_FS.cpp never sets it. Until a CD line is wired (S3 board: GPIO 21, reserved in
// homelab docs/trailcam/gateway-pcb.md) the only safe way to call this is off the main
// thread, under a timeout.
static bool sdq_mount_blocking() {
    s_sdq.mount_ms = millis() | 1u;
    s_sdq.mounted  = false;
    SDQ_FS.end();
    // 1-bit mode: only CLK 14 / CMD 15 / D0 2 are used, which is what keeps the SD set
    // clear of the RMII PHY and the Core1262's SPI. Never format — a card with photos on
    // it from a previous box is data, not scratch space.
    // PORTING NOTE: those pins are fixed-function on the ESP32 classic. On the S3 gateway
    // PCB they are GPIO-matrix routed, so that port must call
    // SDQ_FS.setPins(clk, cmd, d0) BEFORE begin() (pins reserved in the board doc).
    if (!SDQ_FS_MOUNT()) {
        s_sdq.mount_fails++;
        if (!s_sdq.announced) {
            // NOT necessarily an absent card (2026-08-21). A mount failure is
            // ALSO what an unformatted-for-us card looks like, and the card this queue is
            // specced for — the SanDisk 128 GB High Endurance (gateway-sd-queue.md) — is
            // over 32 GB, so it is SDXC and arrives formatted exFAT, which ESP-IDF's
            // FATFS is not built to mount. Saying "NO CARD" here sent the reader hunting
            // a slot/pull-up fault for a card that only needed reformatting.
            Serial.println("[gw] *** sd queue: MOUNT FAILED — no card, OR a card whose "
                           "filesystem won't mount (>32 GB cards ship exFAT; reformat "
                           "FAT32). Upload queue is DEPTH 1, an outage WILL lose "
                           "frames ***");
            log_event("SD queue mount FAILED (absent, or not FAT32) — depth 1, "
                      "outages lose frames");
            s_sdq.announced = true;
        }
        return false;
    }
    if (SDQ_FS.cardType() == CARD_NONE) {
        SDQ_FS.end();
        s_sdq.mount_fails++;
        if (!s_sdq.announced) {
            Serial.println("[gw] *** sd queue: slot reports NO CARD — depth 1 ***");
            log_event("SD CARD ABSENT — upload depth 1, outages lose frames");
            s_sdq.announced = true;
        }
        return false;
    }
    if (!SDQ_FS.exists(GW_SDQ_DIR) && !SDQ_FS.mkdir(GW_SDQ_DIR)) {
        Serial.println("[gw] *** sd queue: cannot create /q -> unusable card ***");
        log_event("SD CARD unusable (no /q) — upload depth 1");
        SDQ_FS.end();
        s_sdq.mount_fails++;
        s_sdq.announced = true;
        return false;
    }
    // The ONLY thing a reboot invalidates: a half-written assembly. Completed entries
    // are exactly what this queue exists to keep.
    if (SDQ_FS.exists(GW_SDQ_TMP)) {
        SDQ_FS.remove(GW_SDQ_TMP);
        Serial.println("[gw] sd queue: purged a torn TMP.TMP");
    }
    s_sdq.mounted   = true;
    s_sdq.announced = false;
    s_sdq.consec_write_errors = 0;
    sdq_scan(nullptr, nullptr, nullptr, /*with_free=*/true);
    if (!s_sdq.mounted) return false;
    Serial.printf("[gw] sd queue: card %llu MB, %llu MB free, %lu queued "
                  "(%lu thumb / %lu full)%s\n",
                  (unsigned long long)(s_sdq.card_bytes >> 20),
                  (unsigned long long)(s_sdq.free_bytes >> 20),
                  (unsigned long)s_sdq.depth, (unsigned long)s_sdq.thumbs,
                  (unsigned long)s_sdq.fulls,
                  s_sdq.strays ? " (+strays ignored)" : "");
    log_event("sd queue ready: %lu queued, %llu MB free", (unsigned long)s_sdq.depth,
              (unsigned long long)(s_sdq.free_bytes >> 20));
    return true;
}

// --- async mount probe -----------------------------------------------------------------
// sdq_mount_blocking() can hang forever on an empty slot (see its comment), and it used
// to be called from setup() BEFORE net_setup()/ota_setup() — so a hang meant no ethernet
// and no OTA, i.e. an unrecoverable board with no rollback in this firmware. On a tower
// that is a climb. So: run it in its own task, bounded by a timeout, and never block
// loop() waiting for it.
#ifndef SDQ_NO_PROBE
// The probe runs PINNED TO CPU 0 and the idle task of that core is temporarily
// unsubscribed from the task watchdog while it is in flight. Both are required, and
// both were learned the hard way (bench, 2026-08-09): on an empty slot SDQ_FS.begin()
// does not merely block, it BUSY-SPINS without yielding, so IDLE0 never runs and the
// TWDT panics at ~5 s — a reboot loop — long before any userland timeout could fire.
// Unsubscribing IDLE0 lets our own timeout win; pinning keeps the spin off CPU 1 where
// loopTask/Arduino lives, so the gateway keeps servicing ethernet, OTA and the radio.
#define GW_SDQ_PROBE_TIMEOUT_MS 4000u
#define GW_SDQ_PROBE_STACK      4096

static volatile bool s_sdq_probe_pending = false;   // a probe task is in flight
static volatile bool s_sdq_probe_done    = false;   // ...and it finished its work
static bool          s_sdq_probe_stuck   = false;   // a probe never returned: give up
static uint32_t      s_sdq_probe_start   = 0;
static TaskHandle_t  s_sdq_probe_handle  = nullptr;

// Idle-task TWDT subscription, toggled around a probe. Guarded so we only ever re-add
// what we actually removed.
static bool s_sdq_idle_wdt_off = false;

static void sdq_idle_wdt(bool watched) {
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    if (!idle0) return;
    if (!watched && !s_sdq_idle_wdt_off) {
        if (esp_task_wdt_delete(idle0) == ESP_OK) s_sdq_idle_wdt_off = true;
    } else if (watched && s_sdq_idle_wdt_off) {
        esp_task_wdt_add(idle0);
        s_sdq_idle_wdt_off = false;
    }
}

// Ends SUSPENDED rather than self-deleting, so sdq_loop() is the single owner of the
// handle and can vTaskDelete() it on either path without racing a self-delete.
static void sdq_probe_task(void*) {
    sdq_mount_blocking();
    s_sdq_probe_done = true;
    vTaskSuspend(nullptr);
}

static void sdq_probe_spawn() {
    s_sdq_probe_done    = false;
    s_sdq_probe_pending = true;
    s_sdq_probe_start   = millis();
    s_sdq.mount_ms      = millis() | 1u;
    sdq_idle_wdt(false);
    if (xTaskCreatePinnedToCore(sdq_probe_task, "sdq_probe", GW_SDQ_PROBE_STACK,
                                nullptr, 1, &s_sdq_probe_handle, 0) != pdPASS) {
        s_sdq_probe_pending = false;
        s_sdq_probe_handle  = nullptr;
        sdq_idle_wdt(true);
        s_sdq.mount_fails++;
        Serial.println("[gw] sd queue: could not spawn mount probe — staying depth 1");
    }
}

static void sdq_probe_reap() {
    if (s_sdq_probe_handle) { vTaskDelete(s_sdq_probe_handle); s_sdq_probe_handle = nullptr; }
    s_sdq_probe_pending = false;
    sdq_idle_wdt(true);
}

// Periodic housekeeping from loop(): re-mount attempts (so installing a card in the
// field needs no reboot / no tree climb) and a stats refresh for the beat + web ring.
// NOTE: this never blocks. The first mount attempt of the boot also happens here, on
// purpose — by the time loop() runs, ethernet and OTA are already up, so even a wedged
// SD probe leaves the gateway reachable and re-flashable.
static void sdq_loop() {
    if (s_sdq_probe_pending) {
        if (s_sdq_probe_done) {
            sdq_probe_reap();
        } else if (millis() - s_sdq_probe_start > GW_SDQ_PROBE_TIMEOUT_MS) {
            // The driver is wedged on an empty/miswired slot. Kill the task — it is
            // busy-spinning in the sdmmc driver, not holding a lock any other code
            // takes, and we never touch SD again this boot — then stop probing. A fresh
            // probe every 60 s would leak a task and re-starve CPU 0 each time. Fitting
            // a card now needs a reboot, a fair price for staying reachable.
            sdq_probe_reap();
            s_sdq_probe_stuck   = true;
            s_sdq.mounted       = false;
            s_sdq.mount_fails++;
            if (!s_sdq.announced) {
                Serial.println("[gw] *** sd queue: mount probe WEDGED (no card, or card "
                               "lines miswired/unpulled) — upload queue is DEPTH 1, an "
                               "outage WILL lose frames. Reboot with a card fitted. ***");
                log_event("SD probe wedged — upload depth 1, outages lose frames");
                s_sdq.announced = true;
            }
        }
        return;
    }
    if (!s_sdq.mounted) {
        if (s_sdq_probe_stuck) return;   // one wedged probe is enough for this boot
        if (!s_sdq.mount_ms || millis() - s_sdq.mount_ms > 60 * 1000u) sdq_probe_spawn();
        return;
    }
    if (!s_sdq.stats_ms || millis() - s_sdq.stats_ms > 15 * 1000u) {
        s_sdq.stats_ms = millis() | 1u;
        sdq_scan(nullptr, nullptr, nullptr, /*with_free=*/true);
    }
}

#endif   // SDQ_NO_PROBE

// Age of the oldest queued frame, in seconds; -1 when unknown (empty, or no clock).
static long sdq_oldest_age_s() {
    const time_t now = time(nullptr);
    if (!s_sdq.depth || !s_sdq.oldest_key || now < 1600000000) return -1;
    return (long)now > (long)s_sdq.oldest_key ? (long)now - (long)s_sdq.oldest_key : 0;
}
