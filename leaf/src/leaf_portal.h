#pragma once
/*
 * Leaf SoftAP + captive portal — walk-up node access with no infrastructure.
 *
 * WHAT THIS IS FOR: a person standing at a camera in a woodlot, with no router, no cell
 * signal and no gateway in earshot, joins the camera's own WiFi network and gets a page
 * served by the camera. That page is the landing pad the rest of the epic hangs off:
 * bulk file pull needs an IP link to the node before it needs anything else,
 * and the live-view aiming stream is now served from here — see LIVE VIEW.
 *
 * PORTED FROM THE GATEWAY, NOT REINVENTED. The gateway built exactly this shape on the
 * gateway: softAP() + a wildcard DNSServer + handlers on a WebServer + onNotFound 302,
 * which is the combination that turns a phone's captive-portal probe into a popup. No
 * WiFiManager: DNSServer ships with the core. Three things differ, and each is the leaf
 * being a leaf rather than a preference:
 *
 *   PSK, NOT OPEN. The gateway's AP is open because it holds nothing and exists only to
 *       receive a credential. A leaf holds wildlife imagery and the ingest bearer token,
 *       so its AP is WPA2. The key is per-node, minted from hardware RNG on first use and
 *       kept in NVS, so no two nodes share one and a compromised node does not open the
 *       rest. It is printed to the console on every portal start for the kitting label
 *       (the label-printer tooling), and BLE will hand it to the app.
 *
 *   AP, NOT AP_STA. The gateway needs the station side to scan for the network it is
 *       being told to join. A leaf is not joining anything: its uplink is LoRa. Dropping
 *       the station interface drops its share of the radio's draw, which on a battery
 *       node is the whole argument.
 *
 *   BOUNDED BY A WINDOW, NOT BY A STATE. The gateway's AP follows a provisioning state
 *       that can legitimately last until someone gets round to it. A leaf's AP lives
 *       inside one maintenance/walk-up window and dies with it, because the node has to
 *       be back in deep sleep for the power budget to hold. There is no path that leaves
 *       an AP up across a sleep: deep sleep resets the MCU.
 *
 * WHAT RAISES IT. The PIR walk-up window, an operator's maintenance command
 * over serial or the mesh, or a BLE handoff once the GATT path lands. The walk-up trigger is
 * the important one and it is ON by default: the whole argument for doing SoftAP before
 * BLE is that a person walks up, opens their phone's WiFi list and finds the camera with
 * no app in the way, and an AP you have to ASK for over the mesh cannot be found by
 * someone standing in front of the camera. -DLEAF_PORTAL_ON_WALKUP=0 turns that off for a
 * build that wants the AP only on demand. See main.cpp for what bounds the cost.
 *
 * LIVE VIEW. /view serves a page whose <img> is an MJPEG multipart stream from
 * /stream, so a person can see what the camera sees while they aim it. It is a plain
 * browser page: no app, no store, nothing installed. The frames are pumped from the wake
 * loop one at a time rather than from a loop inside the HTTP handler, for the same reason
 * commands are queued rather than executed in their callbacks — a handler that does not
 * return owns the whole node, and here that would stall the window deadline, the mesh
 * downlink and the PIR path for as long as somebody left a tab open. -DLEAF_PORTAL_STREAM=0
 * compiles it out.
 *
 * THE SINGLE RADIO, same rule as leaf_ble.h. The S3 has one 2.4 GHz radio. This never
 * runs concurrently with a station join or with BLE: the caller brings up exactly one.
 */
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// What a phone asked for through the page, drained by the wake loop. Queued rather than
// acted on inside the HTTP handler for the same reason leaf_ble queues GATT writes: the
// handler must return its response before the thing it asked for tears its link down.
enum LeafPortalCmd : uint8_t {
    LEAF_PORTAL_CMD_NONE = 0,
    LEAF_PORTAL_CMD_SLEEP,     // "Done" -> end the window now, back to deep sleep
};

// Length of the generated key. 12 characters from a 32-symbol alphabet is ~60 bits, and
// it has to be typed off a label by someone standing in a field, so longer is not free.
#ifndef LEAF_PORTAL_PSK_LEN
#define LEAF_PORTAL_PSK_LEN 12
#endif

// Hard ceiling on how long an associated phone may hold the window open. leaf_ble has no
// equivalent and does not need one: a GATT link exists only while the app holds it, so a
// held BLE deadline implies a person. WiFi does not work that way — a phone rejoins a
// remembered network by itself, from a pocket, and would pin a battery node awake with
// nobody present. This is the bound on that.
#ifndef LEAF_PORTAL_MAX_S
#define LEAF_PORTAL_MAX_S 600
#endif

// Raise the AP: WPA2 SoftAP, wildcard DNS on :53, HTTP on :80. Idempotent per boot.
// Returns false if the AP failed to start, in which case nothing is left running.
bool leaf_portal_begin();

// The network name ("meshcam-leaf-XXXXXX"). The suffix is the last three bytes of the
// BASE MAC — the one hand-marked on the board and printed by the /dev/serial/by-id path —
// so the network on a phone's list matches the unit in hand. Valid BEFORE begin(): it is
// derived from the efuse MAC, not read out of the running WiFi driver, and the BLE
// credential handoff needs the name while the AP is still down.
const char* leaf_portal_ssid();

// The node's AP key. Valid before begin() too — the first call mints it into NVS.
// Print it, put it on the label, hand it over BLE — never render it into the page.
const char* leaf_portal_psk();

// Stage what the status page renders. Same counters the RNS announce tail and the BLE
// status characteristic carry, so all three channels agree. `card` is
// leaf_store_state_code() or nullptr. Safe to call before begin().
void leaf_portal_set_status(const char* node, const char* fw, uint32_t boot_count,
                            uint32_t pir_wakes, uint32_t captures, uint32_t push_fails,
                            float vbat_v, const char* card);

// Tell the page when the window currently expires, so it can say so. Absolute millis().
// Called every pass because the deadline is held off while a phone is associated.
void leaf_portal_set_deadline(uint32_t deadline_abs_ms);

// True while at least one station is associated. The caller holds its deadline off on
// this, exactly as it does on leaf_ble_connected() — subject to LEAF_PORTAL_MAX_S.
bool leaf_portal_connected();

// Take the last command the page queued, clearing it.
LeafPortalCmd leaf_portal_take_cmd();

// Pump the captive DNS, the HTTP server, and one live-view frame if a stream is open.
// Call it often: all three are polled, a phone that waits too long for its probe decides
// the network is broken and drops off, and the stream's frame rate is exactly how often
// this gets called.
void leaf_portal_loop();

// --- live view ----------------------------------------------------------------------
#ifndef LEAF_PORTAL_STREAM
#define LEAF_PORTAL_STREAM 1
#endif
// Hard ceiling on one live-view session. Aiming a camera takes under a minute, and a
// stream is the most expensive thing this node can do (the sensor clocking flat out AND
// the AP transmitting), so a tab left open in a pocket must not be what drains the
// battery. The page states the number and offers a restart link.
#ifndef LEAF_STREAM_MAX_S
#define LEAF_STREAM_MAX_S 180
#endif

// True while a live-view stream is open. The wake loop reads this to stop slicing whole
// seconds into the radio polls: every millisecond spent there is a frame not sent.
bool leaf_portal_streaming();

// Stop the DNS server, stop answering HTTP, drop every association and take the AP down.
// Safe to call when the portal was never started. After this the WiFi radio is off.
void leaf_portal_end();

// True between a successful begin() and end().
bool leaf_portal_active();
