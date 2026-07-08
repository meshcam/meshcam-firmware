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

// Mount + ensure the store dir exists. Idempotent. False if the FS won't mount.
bool leaf_store_begin();

// Save a full-res JPEG under event_id, evicting the oldest if at capacity.
bool leaf_store_save(const char* event_id, const uint8_t* buf, size_t len);

// Load a stored full-res into a PSRAM buffer (caller frees with heap_caps_free).
// Returns nullptr if not stored. mtime_out (optional) = file mtime, ~the capture time.
uint8_t* leaf_store_load(const char* event_id, size_t* len_out, int64_t* mtime_out);
