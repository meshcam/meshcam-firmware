/*
 * PoE bridge — Step 1: ethernet-only endpoint (LilyGo T-Internet-POE, LAN8720).
 *
 * Purpose: prove the UNPROVEN half of the wired gateway BEFORE the radio arrives —
 * does microReticulum's UDP interface run over WIRED ethernet and link to reference
 * Python RNS?  No LoRa here (Core1262 lands Jul 8 -> Step 2 = the Gate A2 BRIDGE role).
 *
 * Flow: ETH.begin() brings up the LAN8720 + DHCP, THEN microReticulum starts a UDP
 * interface over it (the vendored UDPInterface was patched to skip WiFi — WiFiUDP on
 * ESP32 is just an lwIP socket and routes over ethernet). The board recalls the Pi's
 * destination, opens a Link, and pushes 5/50/250 KB Resources.
 * PASS = the Python peer (pi/rns_peer.py) logs RESOURCE COMPLETE for each.
 *
 * Mirrors the proven Gate A Resource/Link API and the Gate A2 endpoint role.
 */

#include <Arduino.h>
#include <WiFi.h>          // WiFiUDP lives here; we use it over ETH, never WiFi.begin()
#include <ArduinoOTA.h>    // network firmware updates (no downloader after the 1st USB flash)

#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <UDPInterface.h>  // vendored + patched (lib/udp_interface): ethernet, not WiFi
#include <microReticulum.h>

#include "poe_config.h"

static RNS::Reticulum  reticulum({RNS::Type::NONE});
static RNS::Interface  udp_interface({RNS::Type::NONE});
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};

static volatile bool eth_got_ip = false;

static void on_eth_event(WiFiEvent_t event) {
	switch (event) {
		case ARDUINO_EVENT_ETH_START:     Serial.println("[poe] ETH start"); ETH.setHostname("trailcam-poe-bridge"); break;
		case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("[poe] ETH link up"); break;
		case ARDUINO_EVENT_ETH_GOT_IP:
			Serial.printf("[poe] ETH got IP: %s  (%uMbps, %s)\n",
				ETH.localIP().toString().c_str(), ETH.linkSpeed(),
				ETH.fullDuplex() ? "full" : "half");
			eth_got_ip = true;
			break;
		case ARDUINO_EVENT_ETH_DISCONNECTED: Serial.println("[poe] ETH link DOWN"); eth_got_ip = false; break;
		case ARDUINO_EVENT_ETH_STOP:         Serial.println("[poe] ETH stop"); eth_got_ip = false; break;
		default: break;
	}
}

static void eth_up() {
	WiFi.onEvent(on_eth_event);                          // ETH events ride the WiFi event bus
	ETH.begin(POE_ETH_ADDR, POE_ETH_POWER_PIN, POE_ETH_MDC_PIN,
	          POE_ETH_MDIO_PIN, POE_ETH_TYPE, POE_ETH_CLK_MODE);
	Serial.print("[poe] waiting for ethernet DHCP");
	uint32_t t0 = millis();
	while (!eth_got_ip && millis() - t0 < 20000) { delay(250); Serial.print("."); }
	Serial.println();
	if (!eth_got_ip) Serial.println("[poe] WARNING: no ETH IP in 20s — check PHY config / cable / DHCP");
}

static void rns_setup() {
	filesystem.init();
	filesystem.format();
	RNS::Utilities::OS::register_filesystem(filesystem);

	udp_interface = new UDPInterface();
	udp_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
	RNS::Transport::register_interface(udp_interface);
	udp_interface.start();                               // patched: opens the lwIP UDP socket over ETH

	reticulum = RNS::Reticulum();
	reticulum.transport_enabled(true);
	reticulum.start();
}

// ---- endpoint role: open a Link to the Pi and push Resources (proven Gate A API) ----
static RNS::Link  pi_link({RNS::Type::NONE});
static RNS::Bytes pi_hash;
static int  next_idx = 0;
static bool linked = false, in_flight = false;

static RNS::Bytes make_blob(size_t n) {
	RNS::Bytes data; uint32_t s = 0xB0E5u ^ (uint32_t)n;   // deterministic filler
	for (size_t i = 0; i < n; ++i) { s = s * 1664525u + 1013904223u; uint8_t b = (uint8_t)(s >> 24); data.append(&b, 1); }
	return data;
}
static void on_sent(const RNS::Resource& r) {
	if (r.status() == RNS::Type::Resource::COMPLETE) { RNS::log("[poe] SEND COMPLETE"); ++next_idx; }
	else RNS::logf(RNS::LOG_ERROR, "[poe] SEND FAILED status=%d", (int)r.status());
	in_flight = false;
}
static void on_link(RNS::Link& l) { RNS::log("[poe] LINK UP to Pi over wired ethernet"); linked = true; }

static void role_setup() { pi_hash.assignHex(POE_PI_DEST_HEX); }
static void role_loop() {
	if (!linked && !pi_link) {
		if (!RNS::Transport::has_path(pi_hash)) { RNS::Transport::request_path(pi_hash); return; }
		RNS::Identity id = RNS::Identity::recall(pi_hash);
		RNS::Destination d(id, RNS::Type::Destination::OUT, RNS::Type::Destination::SINGLE, POE_APP_NAME, POE_ASPECT);
		pi_link = RNS::Link(d);
		pi_link.set_link_established_callback(on_link);
		return;
	}
	const int n = sizeof(POE_SIZES) / sizeof(POE_SIZES[0]);
	if (linked && !in_flight && next_idx < n) {
		size_t sz = POE_SIZES[next_idx];
		RNS::logf(RNS::LOG_NOTICE, "[poe] sending %u-byte Resource to Pi over ethernet", (unsigned)sz);
		in_flight = true;
		new RNS::Resource(make_blob(sz), pi_link, true, false, on_sent, nullptr);
	}
}

void setup() {
	Serial.begin(115200);
	uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(100);
	RNS::loglevel(RNS::LOG_NOTICE);
	Serial.println("[poe] T-Internet-POE ethernet endpoint — microReticulum over LAN8720 (" POE_FW_VERSION ")");
	eth_up();
	// OTA listener over ethernet — all future updates push to this board's IP:3232,
	// no downloader (works over the WG tunnel once deployed). Bootstrapped by this USB flash.
	ArduinoOTA.setHostname("trailcam-poe-bridge");
	ArduinoOTA.onStart([]() { Serial.println("[poe] OTA update starting"); });
	ArduinoOTA.onEnd([]()   { Serial.println("[poe] OTA done, rebooting"); });
	ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[poe] OTA error %u\n", e); });
	ArduinoOTA.begin();
	Serial.println("[poe] ArduinoOTA listening (port 3232)");
	rns_setup();
	role_setup();
}
void loop() {
	ArduinoOTA.handle();        // service OTA pushes
	reticulum.loop();
	role_loop();
	RNS::Utilities::OS::sleep(0.01);
}
