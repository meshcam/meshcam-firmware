#pragma once
/*
 * Leaf full-res store — recent full-res JPEG originals, keyed by event_id.
 *
 * The alert path only sends a small thumbnail; the sensor-max original stays HERE until a
 * gateway command (!TC CMD fetch_full / future RNS command) asks for it by event_id.
 *
 * Backend: the microSD card (SD_MMC 1-bit on the Freenove slot pins 39/38/40 — the reason
 * radio MISO moved to GPIO41) when a card is present, with NO count-eviction (GBs of
 * depth; "SD is a consumable, storage is not the constraint"). Cardless fallback: the
 * 12.9 MB LittleFS partition as a ring of the newest LEAF_STORE_MAX_FULL images, oldest
 * evicted by mtime (valid — the clock is seeded from build time). LittleFS mounting is
 * idempotent with RNS's PosixFileSystem, so init order doesn't matter.
 */
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Which backend the store actually landed on, and WHY when it isn't the card.
// "no card" and "card present but won't mount" are the same false from SD.begin(), and
// they have completely different fixes: the first is a cardless node working as designed,
// the second is almost always an SDXC card straight out of its package (>32 GB cards ship
// exFAT; ESP-IDF's FATFS is built without exFAT) and is cured by reformatting FAT32, NOT
// by replacing the board. Conflating them sends the next person hunting a hardware fault
// (homelab docs/trailcam/kitting-pipeline.md "microSD must be FAT32").
enum LeafStoreState : uint8_t {
    LEAF_STORE_UNINIT = 0,
    LEAF_STORE_SD,                 // card mounted; full-res originals live on the card
    LEAF_STORE_NO_CARD,            // no card answered -> LittleFS ring
    LEAF_STORE_CARD_UNMOUNTABLE,   // card answered, filesystem did not mount -> LittleFS ring
    LEAF_STORE_FALLBACK_UNKNOWN,   // on LittleFS, cause not separable (SD_MMC bench path)
};

// Mount + ensure the store dir exists. Idempotent. False if the FS won't mount.
bool leaf_store_begin();

// Store state after the first leaf_store_begin() (LEAF_STORE_UNINIT before it).
LeafStoreState leaf_store_state();

// One-char code for the announce tail: nullptr while the card is healthy (a good node
// pays zero bytes), "n" for no card, "u" for present-but-unmountable.
const char* leaf_store_state_code();

// Reformat the card to FAT32. DESTRUCTIVE, and deliberately consent-gated: it is never
// called on boot and never on a mount failure, only from an explicit operator command
//. The reason is in leaf_store.cpp: FATFS returns the same "no valid FAT
// volume" for a factory-fresh exFAT card and for a card whose boot sector was torn with
// the owner's photos still behind it, and the firmware cannot tell those apart. A human
// can, so a human decides.
//
// SAFETY INVARIANT, keep it if this is ever refactored: it refuses unless a card is
// present AND its filesystem genuinely will not mount. A card the leaf can read is never
// formatted, whatever the command says — so the worst a spurious or replayed command can
// do is finish the job on a card already reported unreadable (sd=u).
//
// Blocking and slow: clearing the FAT on a 128 GB card over a 4 MHz SPI bus is tens of MB
// of writes, so budget minutes. Safe to run inline on the Arduino loop task, which lives
// on core 1 whose idle task the WDT does not watch (sdkconfig
// CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 is unset) — the same reason OTA blocks inline.
//
// Returns a short outcome string for the log. Never null, always safe to print.
const char* leaf_store_format_card();

// Save a full-res JPEG under event_id, evicting the oldest if at capacity.
bool leaf_store_save(const char* event_id, const uint8_t* buf, size_t len);

// Load a stored full-res into a PSRAM buffer (caller frees with heap_caps_free).
// Returns nullptr if not stored. mtime_out (optional) = file mtime, ~the capture time.
uint8_t* leaf_store_load(const char* event_id, size_t* len_out, int64_t* mtime_out);

// Print the store directory (first few entries + count/bytes/backend) tagged with
// `why`. Fetch-miss forensics: three 2026-07-18 fetch_fulls found saves gone.
void leaf_store_debug_list(const char* why);
