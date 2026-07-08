#pragma once
/*
 * Leaf OTA — command-driven firmware update over WiFi.
 *
 * The leaf has no schedule and no baked-in credentials: an update happens because the
 * operator queued an `update_firmware` command on the message bus (the same !TC CMD
 * downlink that serves fetch_full), carrying everything needed for this one update:
 *
 *   {"kind":"update_firmware","payload":{"ssid":"...","psk":"...",
 *    "url":"http(s)://host/firmware.bin","sha256":"<64 hex of the .bin>"}}
 *
 * leaf_ota_run(): join WiFi -> stream the image into the idle app slot (dual-OTA
 * partition table) while hashing it -> verify sha256 BEFORE committing -> set boot
 * partition -> reboot into the new image. Any failure leaves the running firmware
 * untouched (the idle slot is scratch space). The esp-idf rollback machinery guards
 * the other half: a new image that crash-loops never marks itself valid and the
 * bootloader falls back to this one.
 *
 * Runs in the post-event command window: the camera is released, we're awake anyway,
 * and WiFi+TLS heap (~50 KB) is free at that point. Blocks for the duration (a ~1.9 MB
 * image on home WiFi is ~10-30 s). The sha256 comes over the command bus, which is the
 * trust anchor here — TLS certificate validation is deliberately skipped (no CA store
 * on the leaf; integrity is the hash, authenticity is "only the operator can queue
 * commands").
 */
#include <stddef.h>

struct TcOtaRequest;   // leaf_serial_proto.h

// Execute one update request. Returns only on failure (success reboots the board).
// `false` = firmware unchanged, WiFi torn down, safe to sleep.
bool leaf_ota_run(const TcOtaRequest& req);
