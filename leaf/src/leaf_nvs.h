#pragma once
/*
 * Durable NVS — the partition that survives what the ordinary one does not.
 *
 * The leaf keeps two things it must never lose: its RNS identity (the private key the
 * node's whole wire name derives from) and the portal AP key (printed on the label at
 * kitting). Both used to live in the default `nvs` partition at 0x9000, which turns out
 * to be the wrong place twice over:
 *
 *   - `write-flash 0x0 firmware.factory.bin` writes 20 KB of 0xff straight across it,
 *     because the combined image is contiguous from 0x0
 *   - arduino-esp32's initArduino() erases the entire partition and re-inits whenever
 *     nvs_flash_init() answers NO_FREE_PAGES, and says nothing about it. The WiFi stack
 *     shares that 20 KB for PHY calibration, so filling it is a matter of time rather
 *     than a mistake anyone makes
 *
 * So they moved to `id` at 0x610000 (partitions_leaf.csv). Past the end of any factory
 * image, and never the partition initArduino() resolves. See that file for why the
 * offset and the ordering are load-bearing.
 *
 * Everything else stays in the default partition, where being wiped is harmless: the OTA
 * "already applied" sha, and whatever the WiFi driver wants.
 */
#include <Preferences.h>

// Partition label, matching the `id` row of partitions_leaf.csv.
#define LEAF_NVS_DURABLE "id"

// Does this board's partition table actually have it? False on any board still running
// a pre-leaf-0.23.0 table, which is the case the migration path exists for. Cached after
// the first call, so it is free to ask on every wake.
bool leaf_nvs_durable_available();

// Move anything that must be durable out of the default partition and into `id`, once,
// EARLY IN BOOT AND UNCONDITIONALLY. Both halves of that matter.
//
// Early, because the migration has to happen before anything can destroy the old copy.
// Lazy migration is what a first draft does — move the AP key the first time the portal
// is raised — and it loses the key outright on a board that gets updated and then
// factory-flashed before anyone ever opens its portal. That is not a corner case, it is
// the normal bring-up order for a board that has been kitted and labelled but not yet
// deployed.
//
// Unconditional, because the cheap-looking guards are all wrong. An RTC flag does not
// survive what it needs to survive: an RTS hard reset leaves RTC memory intact, so the
// boot straight after a reflash would skip the migration entirely. The honest cost is
// two NVS opens per wake when there is nothing to move.
//
// ERASES the whole default partition once anything has been moved out of it. Not
// Preferences::remove(): NVS is log-structured, so remove() writes a tombstone and the
// key's bytes stay readable in flash until that page is compacted. Measured on board
// 0013 — after a remove()-based migration the 64-byte private key was still recoverable
// with a plain `esptool read-flash`. Two copies of the key defeats the point.
void leaf_nvs_migrate_durable();
