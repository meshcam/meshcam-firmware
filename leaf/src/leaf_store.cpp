#include "leaf_store.h"

#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#ifdef LEAF_SD_SPI_CS
#include <SD.h>
#include <SPI.h>
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

static const char* DIR = "/tcfull";

static fs::FS* s_fs      = nullptr;   // active backend (SD_MMC or LittleFS)
static bool    s_on_sd   = false;

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
    } else {
#else
    SD_MMC.setPins(LEAF_SD_CLK, LEAF_SD_CMD, LEAF_SD_D0);
    if (SD_MMC.begin("/sdcard", /*mode1bit=*/true)) {
        uint64_t sz = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("[store] SD card mounted (%llu MB)\n", (unsigned long long)sz);
        s_fs = &SD_MMC;
        s_on_sd = true;
    } else {
#endif
        // Fallback: the 12.9 MB LittleFS partition (same args as microStore's
        // PosixFileSystem on the RNS side — idempotent either order).
        Serial.println("[store] no SD card -> LittleFS fallback");
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
    Serial.printf("[store] saved %s (%u bytes, %s)\n", path, (unsigned)len,
                  s_on_sd ? "SD" : "LittleFS");
    return true;
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
