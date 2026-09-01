#include "leaf_store.h"

#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#ifdef LEAF_SD_SPI_CS
#include <SD.h>
#include <SPI.h>
// sd_diskio.h is the SD library's own header (SD.cpp includes it): it exposes the two
// halves SD.begin() fuses together — sdcard_init() talks to the card, sdcard_mount()
// mounts its filesystem. We need them apart to tell "no card" from "exFAT card".
#include <sd_diskio.h>
// FatFs directly, for the consent-gated format. sdcard_init() registers the
// diskio impl for the drive, so f_mount/f_mkfs on "<pdrv>:" work without the VFS layer.
#include <ff.h>
#endif
#include "leaf_rails.h"
#include "esp_heap_caps.h"

// Ring capacity for the LittleFS FALLBACK (12.9 MB partition, QXGA ~130-400 KB each).
// The SD card path does no count-eviction at all — a 32 GB card holds months of bench
// churn and the server-side command window is 14 days anyway; revisit if cards run full.
#ifndef LEAF_STORE_MAX_FULL
#define LEAF_STORE_MAX_FULL 4
#endif

// Freenove onboard microSD slot (1-bit SD_MMC; the slot doesn't break out DAT3, so SPI
// mode isn't possible on this board). These pins are the reason radio MISO moved to 41.
#ifndef LEAF_SD_CLK
#define LEAF_SD_CLK 39
#endif
#ifndef LEAF_SD_CMD
#define LEAF_SD_CMD 38
#endif
#ifndef LEAF_SD_D0
#define LEAF_SD_D0  40
#endif

// /tcfull2, not /tcfull (2026-07-18): the original directory's FAT chain is damaged
// on the office leaf's card — 1247 ancient files (July-2 era, led by a 0-byte scar
// from an interrupted write) list forever, while every NEW save verifies in-session
// and is GONE on remount (three fetch_fulls proved it). Entries appended past the
// break land in an orphaned cluster the directory cache shows but the on-card chain
// doesn't. A fresh directory = a fresh, intact chain. The old dir stays untouched
// for fsck/postmortem at next physical access; reformat the card when convenient.
static const char* DIR = "/tcfull2";

static fs::FS* s_fs      = nullptr;   // active backend (SD_MMC or LittleFS)
static bool    s_on_sd   = false;
static LeafStoreState s_state = LEAF_STORE_UNINIT;

LeafStoreState leaf_store_state() { return s_state; }

const char* leaf_store_state_code() {
    switch (s_state) {
        case LEAF_STORE_NO_CARD:          return "n";
        case LEAF_STORE_CARD_UNMOUNTABLE: return "u";
        case LEAF_STORE_FALLBACK_UNKNOWN: return "?";
        default:                          return nullptr;   // healthy (or not yet begun)
    }
}

#ifdef LEAF_SD_SPI_CS
// SD.begin() returns one bool for two unrelated faults. Re-run its two steps by hand so
// the log can name which one happened — and, when the card IS there, name the fix. Only
// runs on the failure path, so a healthy boot pays nothing for it.
// SD.h's default bus clock is 4 MHz; match it so the probe fails/succeeds for the same
// reasons the real mount did (raising it is another job, not this probe's).
static LeafStoreState classify_sd_spi_failure() {
    uint8_t pdrv = sdcard_init(LEAF_SD_SPI_CS, &SPI, 4000000);
    if (pdrv == 0xFF) {
        Serial.println("[store] no SD card (card never answered) -> LittleFS fallback");
        return LEAF_STORE_NO_CARD;
    }
    const uint64_t mb = (uint64_t)sdcard_num_sectors(pdrv) * sdcard_sector_size(pdrv)
                        / (1024ull * 1024ull);
    sdcard_uninit(pdrv);
    Serial.printf("[store] *** SD CARD IS PRESENT (%llu MB) BUT ITS FILESYSTEM WOULD NOT "
                  "MOUNT -> LittleFS fallback ***\n", (unsigned long long)mb);
    Serial.println("[store] *** this is NOT a wiring or board fault. Cards over 32 GB are "
                   "SDXC and ship formatted exFAT, which this firmware cannot mount. "
                   "Reformat the card FAT32 (MBR, one type-0x0c partition, 32 KiB "
                   "clusters) and it will mount. ***");
    return LEAF_STORE_CARD_UNMOUNTABLE;
}
#endif

bool leaf_store_begin() {
    if (s_fs) return true;

    // Primary: the microSD card — "SD is a consumable; storage is not the constraint."
#ifdef LEAF_SD_SPI_CS
    // Custom PCB: card in SPI mode on the shared radio bus (SCK47/MOSI48/MISO38,
    // CS=IO43 w/ 10k pull-up), riding the SWITCHED LORA rail — power it first.
    leaf_radio_rail(true);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);   // idempotent; same bus as SX1262
    pinMode(LEAF_SD_SPI_CS, OUTPUT);
    digitalWrite(LEAF_SD_SPI_CS, HIGH);
    if (SD.begin(LEAF_SD_SPI_CS, SPI)) {
        uint64_t sz = SD.cardSize() / (1024 * 1024);
        Serial.printf("[store] SD card mounted via SPI (%llu MB)\n", (unsigned long long)sz);
        s_fs = &SD;
        s_on_sd = true;
        s_state = LEAF_STORE_SD;
    } else {
        s_state = classify_sd_spi_failure();
#else
    SD_MMC.setPins(LEAF_SD_CLK, LEAF_SD_CMD, LEAF_SD_D0);
    if (SD_MMC.begin("/sdcard", /*mode1bit=*/true)) {
        uint64_t sz = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("[store] SD card mounted (%llu MB)\n", (unsigned long long)sz);
        s_fs = &SD_MMC;
        s_on_sd = true;
        s_state = LEAF_STORE_SD;
    } else {
        // SD_MMC (bench Freenove) hides which half failed: the Arduino wrapper collapses
        // esp_vfs_fat_sdmmc_mount()'s ESP_FAIL (card fine, filesystem unmountable) and its
        // card-init errors into one false. Splitting them needs a hand-rolled IDF mount,
        // which is not worth it on a bench-only path — so say both, and don't claim which.
        s_state = LEAF_STORE_FALLBACK_UNKNOWN;
        Serial.println("[store] SD_MMC mount failed: either NO CARD, or a card whose "
                       "filesystem won't mount (>32 GB cards ship exFAT — reformat FAT32)");
#endif
        // Fallback: the 12.9 MB LittleFS partition (same args as microStore's
        // PosixFileSystem on the RNS side — idempotent either order).
        Serial.println("[store] falling back to LittleFS");
        if (!LittleFS.begin(true, "")) {
            Serial.println("[store] LittleFS mount FAILED");
            return false;
        }
        s_fs = &LittleFS;
        s_on_sd = false;
    }
    if (!s_fs->exists(DIR)) s_fs->mkdir(DIR);
    return true;
}

static void store_path(const char* event_id, char* out, size_t cap) {
    snprintf(out, cap, "%s/%s.jpg", DIR, event_id);
}

// LittleFS fallback only: evict oldest-by-mtime until under capacity.
static void evict_for_space() {
    if (s_on_sd) return;   // SD: no count-eviction (see header note)
    while (true) {
        File d = s_fs->open(DIR);
        if (!d || !d.isDirectory()) return;
        int count = 0;
        time_t oldest_t = 0;
        String oldest;
        for (File f = d.openNextFile(); f; f = d.openNextFile()) {
            count++;
            time_t mt = f.getLastWrite();
            if (oldest.isEmpty() || mt < oldest_t) { oldest_t = mt; oldest = String(f.path()); }
        }
        d.close();
        if (count < LEAF_STORE_MAX_FULL || oldest.isEmpty()) return;
        Serial.printf("[store] evicting %s (at capacity %d)\n", oldest.c_str(), LEAF_STORE_MAX_FULL);
        s_fs->remove(oldest);
    }
}

bool leaf_store_save(const char* event_id, const uint8_t* buf, size_t len) {
    if (!leaf_store_begin() || !buf || !len) return false;
    evict_for_space();
    char path[96];
    store_path(event_id, path, sizeof(path));
    File f = s_fs->open(path, "w");
    if (!f) { Serial.printf("[store] open-for-write FAILED: %s\n", path); return false; }
    size_t n = f.write(buf, len);
    f.close();
    if (n != len) {
        Serial.printf("[store] short write %u/%u -> removing %s\n", (unsigned)n, (unsigned)len, path);
        s_fs->remove(path);
        return false;
    }
    // Post-save read-back (2026-07-18): three straight fetch_fulls hit "not in
    // store" for events whose save had logged success on SD — something between
    // f.close() and a later wake loses the file. Same philosophy as the gateway's
    // spool stat-verify: never trust a quiet write. This catches a card that acks
    // writes it didn't commit at the moment it lies, not 30 wakes later.
    File v = s_fs->open(path, "r");
    const size_t vsz = v ? v.size() : 0;
    if (v) v.close();
    if (vsz != len) {
        Serial.printf("[store] POST-SAVE VERIFY FAILED %s (read %u != wrote %u)\n",
                      path, (unsigned)vsz, (unsigned)len);
        return false;
    }
    Serial.printf("[store] saved %s (%u bytes, %s, verified)\n", path, (unsigned)len,
                  s_on_sd ? "SD" : "LittleFS");
    return true;
}

// Diagnostic listing for fetch-miss forensics: what IS in the store, on which
// backend. Turns every "not in store" into a diagnosis instead of a mystery.
void leaf_store_debug_list(const char* why) {
    if (!leaf_store_begin()) return;
    File d = s_fs->open(DIR);
    if (!d || !d.isDirectory()) {
        Serial.printf("[store] list(%s): %s won't open on %s\n", why, DIR,
                      s_on_sd ? "SD" : "LittleFS");
        return;
    }
    int n = 0;
    size_t bytes = 0;
    for (File f = d.openNextFile(); f; f = d.openNextFile()) {
        n++;
        bytes += f.size();
        if (n <= 8) Serial.printf("[store]   %s (%u B)\n", f.path(), (unsigned)f.size());
    }
    d.close();
    Serial.printf("[store] list(%s): %d file(s), %u B, backend=%s\n",
                  why, n, (unsigned)bytes, s_on_sd ? "SD" : "LittleFS");
}

uint8_t* leaf_store_load(const char* event_id, size_t* len_out, int64_t* mtime_out) {
    if (len_out) *len_out = 0;
    if (!leaf_store_begin()) return nullptr;
    char path[96];
    store_path(event_id, path, sizeof(path));
    File f = s_fs->open(path, "r");
    if (!f) return nullptr;
    size_t len = f.size();
    uint8_t* buf = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    if (!buf) { f.close(); Serial.println("[store] load alloc FAILED"); return nullptr; }
    size_t n = f.read(buf, len);
    if (mtime_out) *mtime_out = (int64_t)f.getLastWrite();
    f.close();
    if (n != len) { heap_caps_free(buf); return nullptr; }
    if (len_out) *len_out = len;
    return buf;
}


// --- consent-gated card format ---------------------------------------------------------
// Design rule, 2026-08-21: a leaf never formats a card on its own. The go arrives over
// the radio from the app ("the couch version"), because the alternative — the library's
// format_if_empty flag — is a standing promise to erase anything FATFS cannot parse, and
// FRESULT 13 does not mean "blank". See the header for the invariant this must preserve.
#ifdef LEAF_SD_SPI_CS
const char* leaf_store_format_card() {
    static char out[96];

    if (s_on_sd) {
        Serial.println("[store] format REFUSED: the store is live on this card");
        return "refused: card is mounted and readable";
    }

    leaf_radio_rail(true);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    pinMode(LEAF_SD_SPI_CS, OUTPUT);
    digitalWrite(LEAF_SD_SPI_CS, HIGH);

    const uint8_t pdrv = sdcard_init(LEAF_SD_SPI_CS, &SPI, 4000000);
    if (pdrv == 0xFF) {
        Serial.println("[store] format REFUSED: no card answered");
        return "refused: no card";
    }
    char drv[3] = {(char)('0' + pdrv), ':', 0};

    // THE GATE. Re-prove the filesystem is unreadable at this instant rather than trusting
    // s_state from boot: a card can be swapped during a maintenance window, and the command
    // may have been queued minutes ago. Anything that mounts is somebody's data.
    FATFS* fs = (FATFS*)calloc(1, sizeof(FATFS));
    if (!fs) { sdcard_uninit(pdrv); return "failed: out of memory"; }
    FRESULT res = f_mount(fs, drv, 1);
    f_mount(nullptr, drv, 0);
    if (res == FR_OK) {
        Serial.println("[store] format REFUSED: the card mounts fine. Nothing to fix, and "
                       "formatting it would destroy whatever is on it.");
        free(fs); sdcard_uninit(pdrv);
        return "refused: card is readable";
    }
    if (res != FR_NO_FILESYSTEM) {
        // Read errors, a dying card, a wiring fault: all real, none of them fixed by mkfs.
        Serial.printf("[store] format REFUSED: card unreadable for another reason "
                      "(FRESULT %d) — this is not a format problem\n", (int)res);
        free(fs); sdcard_uninit(pdrv);
        snprintf(out, sizeof(out), "refused: card fault (FRESULT %d), not a format problem",
                 (int)res);
        return out;
    }

    // 32 KiB clusters, stated rather than defaulted. The library's own format path passes
    // au_size 0 with a 512-byte work buffer, which is the same defaulting that makes
    // mkfs.vfat need an explicit -s 64 by hand on a card this size. FM_FAT32 without FM_SFD
    // writes an MBR plus one partition, matching the hand recipe in kitting-pipeline.md 4a.
    // The work buffer is 16 KB, not FF_MAX_SS: 512 bytes "works" and crawls, and the FAT on
    // a 128 GB card is tens of MB to clear.
    const UINT WORK = 16384;
    BYTE* work = (BYTE*)malloc(WORK);
    if (!work) { free(fs); sdcard_uninit(pdrv); return "failed: out of memory"; }
    const MKFS_PARM opt = { (BYTE)FM_FAT32, 0, 0, 0, 32u * 1024u };

    Serial.println("[store] *** FORMATTING THE CARD FAT32 — minutes, not seconds, on a "
                   "large card. DO NOT power-cycle the node until this reports done. ***");
    const uint32_t t0 = millis();
    res = f_mkfs(drv, &opt, work, WORK);
    const uint32_t secs = (millis() - t0) / 1000u;
    free(work);

    if (res != FR_OK) {
        Serial.printf("[store] format FAILED after %us (FRESULT %d)\n",
                      (unsigned)secs, (int)res);
        free(fs); sdcard_uninit(pdrv);
        snprintf(out, sizeof(out), "failed: mkfs FRESULT %d after %us", (int)res,
                 (unsigned)secs);
        return out;
    }

    // Never report success on a write we did not read back — same rule as the post-save
    // verify. A format that produces a filesystem we still cannot mount is a failure.
    res = f_mount(fs, drv, 1);
    f_mount(nullptr, drv, 0);
    free(fs);
    sdcard_uninit(pdrv);
    if (res != FR_OK) {
        Serial.printf("[store] format wrote a filesystem we STILL cannot mount "
                      "(FRESULT %d)\n", (int)res);
        snprintf(out, sizeof(out), "failed: formatted but won't mount (FRESULT %d)",
                 (int)res);
        return out;
    }

    // Re-open the store so the card is in use immediately, without waiting for a reboot.
    s_fs = nullptr;
    s_state = LEAF_STORE_UNINIT;
    leaf_store_begin();
    Serial.printf("[store] format OK in %us; store backend is now %s\n",
                  (unsigned)secs, s_on_sd ? "SD" : "LittleFS");
    snprintf(out, sizeof(out), "formatted FAT32 in %us, store now on %s", (unsigned)secs,
             s_on_sd ? "the card" : "LittleFS (unexpected)");
    return out;
}
#else
const char* leaf_store_format_card() {
    // SD_MMC bench path. Not implemented on purpose: this board cannot even tell "no card"
    // from "unmountable card" (see leaf_store_begin), so it cannot honour the safety gate,
    // and a format it cannot justify is exactly what this feature exists to avoid.
    Serial.println("[store] format not supported on the SD_MMC build");
    return "refused: not supported on this board";
}
#endif
