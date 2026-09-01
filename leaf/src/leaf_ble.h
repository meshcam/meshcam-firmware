#pragma once
/*
 * Leaf BLE layer — NimBLE advertising + a one-characteristic GATT service for walk-up
 * node access.
 *
 * WHAT THIS IS FOR: a person standing at the camera opens the phone app, sees the node
 * advertising, connects, reads its status, and writes a command that makes the node bring
 * up WiFi so the phone can pull photos at real bandwidth. BLE is the DISCOVERY and
 * CONTROL channel only — the bulk transfer never runs over it.
 *
 * NIMBLE, NOT BLUEDROID. arduino-esp32 3.3.9 ships its precompiled libbt.a built
 * CONFIG_BT_NIMBLE_ENABLED=y / CONFIG_BLUEDROID_ENABLED unset, and the bundled `BLE`
 * library (BLEDevice.h) is the NimBLE-backed rewrite derived from esp-nimble-cpp. So on
 * this toolchain there IS no Bluedroid path — including <BLEDevice.h> IS using NimBLE,
 * and no external NimBLE-Arduino dependency is needed. Verified in the SDK's sdkconfig.
 *
 * THE SINGLE RADIO. The S3 has ONE 2.4 GHz radio shared by BLE and WiFi. The design is a
 * SEQUENTIAL handoff, never concurrent operation: advertise -> phone connects -> phone
 * writes "wifi" -> leaf tears BLE all the way down (leaf_ble_end) -> WiFi comes up ->
 * transfer -> WiFi down -> BLE back up. leaf_ble_end() calls BLEDevice::deinit(true),
 * which also releases the BT controller's heap back to the system — that release is what
 * makes the WiFi driver's allocations fit, so the teardown is not optional politeness.
 *
 * DEEP SLEEP. Nothing here survives a wake; like the RNS stack, BLE is brought up fresh
 * whenever a wake wants it and torn down before arm_and_sleep(). The advertised name is
 * derived from the MAC so it is stable across boots without any stored state.
 */
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// 128-bit UUIDs for the MeshCam node service. Random base, no relation to any assigned
// number; the phone app filters its scan on the service UUID, which is what makes a
// walk-up "which of these is my camera" list possible at all.
#define LEAF_BLE_SERVICE_UUID "6d657368-6361-6d00-8000-00805f9b34fb"
#define LEAF_BLE_STATUS_UUID  "6d657368-6361-6d01-8000-00805f9b34fb"

// What a phone wrote to the status characteristic, drained by the wake loop. Kept as a
// tiny enum rather than a callback so nothing app-level runs inside a NimBLE host
// callback (same discipline as leaf_radio_poll_commands: queue in the callback, execute
// after it returns).
enum LeafBleCmd : uint8_t {
    LEAF_BLE_CMD_NONE = 0,
    LEAF_BLE_CMD_WIFI,        // "wifi"   -> tear BLE down, bring WiFi/SoftAP up
    LEAF_BLE_CMD_SLEEP,       // "sleep"  -> end the window, go back to deep sleep
    LEAF_BLE_CMD_UNKNOWN,     // anything else (forward compat: ignored, not an error)
};

// Bring up the NimBLE host + controller, register the service and its one characteristic,
// and start advertising. Idempotent per boot. Returns false if the stack failed to init.
// `name_suffix` is appended to "MeshCam-" for the advertised name; pass nullptr to derive
// it from the last 3 bytes of the base MAC.
bool leaf_ble_begin(const char* name_suffix = nullptr);

// The advertised device name actually in use ("MeshCam-XXXXXX"), or "" before begin().
const char* leaf_ble_name();

// Stage the status payload the characteristic serves on read and pushes on notify. Small
// JSON, same counters the RNS announce tail carries so both channels agree:
//   {"node":"...","fw":"...","boot":N,"pir":N,"cap":N,"fail":N,"vb":x.xx}
// Safe to call before begin() (staged) or with a phone connected (notifies subscribers).
void leaf_ble_set_status(const char* node, const char* fw, uint32_t boot_count,
                         uint32_t pir_wakes, uint32_t captures, uint32_t push_fails,
                         float vbat_v);

// True while a phone holds a GATT connection.
bool leaf_ble_connected();

// Take the last command a phone wrote, clearing it. LEAF_BLE_CMD_NONE if nothing pending.
LeafBleCmd leaf_ble_take_cmd();

// Stop advertising, terminate any live link, wait for the host to confirm it is down, then
// deinit. Call this BEFORE WiFi.begin() — see the single-radio note above. Safe to call
// when BLE was never started.
//
// release_memory=true additionally calls esp_bt_controller_mem_release(), handing the
// controller's BSS to the general heap. It is ONE-WAY FOR THE REST OF THE BOOT:
// leaf_ble_begin() after a released teardown panics (LoadProhibited on the heap poison
// canary).
//
// DEFAULT IS false, and that is a measured decision, not caution. Bench 2026-08-17: the
// release is worth about 2.2 KB (a kept teardown leaves 1,520 bytes unrecovered; a
// released one ends 732 bytes above the pre-BLE level). Two kilobytes is not worth giving
// up the ability to advertise again in the same wake, and the earlier guess that it was
// worth ~18 KB came from reading the linker's static IRAM delta — that is controller CODE
// in IRAM, which this call does not hand back. Pass true only where the wake is certain to
// end in deep sleep and the RAM is genuinely needed.
void leaf_ble_end(bool release_memory = false);

// True between a successful begin() and end().
bool leaf_ble_active();
