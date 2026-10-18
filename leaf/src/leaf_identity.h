#pragma once
/*
 * Identity witness — notice, and say out loud, that this board's RNS identity changed.
 *
 * The leaf's mesh identity is a private key in NVS at 0x9000 (leaf_radio.cpp). Losing it
 * mints a new one, and a new identity IS a new node to everything downstream: the
 * gateway's TOFU pin goes stale so every queued command silently stops being deliverable,
 * and the server auto-creates a second camera row so the node's history splits in two.
 * Nothing in the old firmware noticed. The console printed "identity CREATED" once, on
 * the first boot after the loss, and then years of "identity loaded from NVS" lines that
 * look exactly like a healthy node.
 *
 * Since leaf-0.23.0 the key lives in the durable `id` partition (leaf_nvs.h), which is
 * out of reach of both of the things that used to take it: the 20 KB of 0xFF that
 * firmware.factory.bin carries across the default nvs partition, and arduino-esp32's
 * silent erase of that partition when it fills. So the routine ways to lose an identity
 * are gone.
 *
 * This layer stays anyway, because "gone" is a claim about the paths we know:
 *   - `esptool erase-flash` still clears everything, and always will
 *   - a future partition-table change that moves `id` would take it, exactly as the
 *     0.23.0 change would have without its migration path
 *   - a board still running a pre-0.23.0 table keeps the key in the old place
 * A loss that cannot happen costs one file and one comparison to prove each boot. A loss
 * that does happen and is not reported costs a split camera history and a queue of
 * undeliverable commands, and nobody finds out for weeks.
 *
 * The witness lives on LittleFS, in the spiffs partition. It records the identity this
 * board last booted with. When the derived identity does not match it, that is a loss,
 * and the leaf says so: a boot banner, an "x=<previous-slug>" token on the next few
 * announces (the only channel a fielded node has), and a "prev_node" field in serial
 * telemetry.
 *
 * What it deliberately does NOT do: restore the key. Keeping a second copy of the private
 * key is a security decision and a product decision, and it stays an explicit choice, not a
 * default. This layer only makes the loss impossible to miss.
 */
#include <Arduino.h>
#include <stdint.h>

#ifdef LEAF_RADIO

// How many announces of each power cycle carry the "x=" alarm token. Bounded rather than
// forever: 16 bytes on a ~174-byte announce is 9 % more airtime, and the alarm re-arms on
// every power cycle, so a gateway that missed the whole burst gets another chance. The
// witness file keeps the record permanently either way.
#ifndef LEAF_IDENTITY_ALARM_ANNOUNCES
#define LEAF_IDENTITY_ALARM_ANNOUNCES 12
#endif

// Compare the identity this boot derived against the witness on LittleFS, log the verdict,
// and rewrite the witness when it changed. Call once per boot from leaf_radio_begin(),
// after the destination hash is known and the filesystem is mounted.
//   slug     "leaf-xxxxxxxx", derived from the destination hash
//   hash_hex full destination hash, lowercase hex
//   minted   true when load_or_create_identity() had to generate a key (NVS was empty)
void leaf_identity_witness(const char* slug, const char* hash_hex, bool minted);

// The slug this board announced under before the change, or nullptr when the identity is
// the one the witness expected (the healthy case, and the first-boot case).
const char* leaf_identity_prev_slug();

// How many identities this board has had, witness-file lifetime. 1 = never changed.
uint32_t leaf_identity_generation();

// True while this power cycle still owes the mesh an alarm announce. Announce code calls
// leaf_identity_alarm_sent() once the token actually made it into the app_data.
bool leaf_identity_alarm_pending();
void leaf_identity_alarm_sent();

#endif  // LEAF_RADIO
