/*
 * Gate A2 — does microReticulum (C++) interoperate with reference Python RNS, and
 * can a microReticulum board bridge a LoRa mesh to a Pi over IP?  This decides the
 * production GATEWAY architecture.  (see ../README.md, docs/trailcam/design.md)
 *
 * Two roles (PlatformIO env / build flag), one firmware:
 *
 *   -DA2_ROLE_ENDPOINT  (A2a, IP-only interop):
 *       WiFi + UDPInterface only. The board creates a Link to the Pi's destination
 *       and pushes 5/50/250 KB Resources over UDP. PASS = the Pi (Python RNS) logs
 *       RESOURCE COMPLETE. This is the make-or-break cross-implementation test.
 *
 *   -DA2_ROLE_BRIDGE    (A2b, full gateway):
 *       LoRaInterface + UDPInterface, transport_enabled=true, NO app destination.
 *       Pure L3 router. The leaf (Gate A `heltec-v3-client` firmware) sends Resources
 *       over LoRa to the Pi's dest hash; this board forwards leaf --LoRa--> --UDP--> Pi.
 *
 * The clean part of the design: we never mix two L2 framings on one radio. The LoRa
 * hop is microReticulum<->microReticulum (same framing, proven in Gate A); the IP hop
 * is microReticulum-UDP<->Python-RNS-UDP. RNS transport routes between them at L3.
 */

#include <Arduino.h>
#include <WiFi.h>

#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <UDPInterface.h>           // vendored from microReticulum examples/common
#if defined(A2_ROLE_BRIDGE)
#include <LoRaInterface.h>          // vendored; patch pins per board if not a Heltec V3
#endif
#include <microReticulum.h>

#include "a2_config.h"

static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface udp_interface({RNS::Type::NONE});
#if defined(A2_ROLE_BRIDGE)
static RNS::Interface lora_interface({RNS::Type::NONE});
#endif
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};

static void wifi_up() {
	WiFi.mode(WIFI_STA);
	WiFi.begin(A2_WIFI_SSID, A2_WIFI_PASS);
	Serial.print("[gateA2] WiFi");
	while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
	Serial.printf(" connected, ip=%s\n", WiFi.localIP().toString().c_str());
}

static void rns_setup() {
	filesystem.init();
	filesystem.format();
	RNS::Utilities::OS::register_filesystem(filesystem);

	// NOTE: microReticulum's Arduino UDPInterface also calls WiFi.begin with its own
	// creds (see UDPInterface.cpp). We bring WiFi up first; if your vendored copy needs
	// creds passed in, wire A2_WIFI_* into its constructor/setter. (verify-before-build)
	udp_interface = new UDPInterface();
	udp_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
	RNS::Transport::register_interface(udp_interface);
	udp_interface.start();

#if defined(A2_ROLE_BRIDGE)
	lora_interface = new LoRaInterface();
	lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
	RNS::Transport::register_interface(lora_interface);
	lora_interface.start();
#endif

	reticulum = RNS::Reticulum();
	reticulum.transport_enabled(true);   // route between interfaces (essential for BRIDGE)
	reticulum.start();
}

// =========================================================================== ENDPOINT
#if defined(A2_ROLE_ENDPOINT)
static RNS::Link  pi_link({RNS::Type::NONE});
static RNS::Bytes pi_hash;
static int  next_idx = 0;
static bool linked = false, in_flight = false;

static RNS::Bytes make_blob(size_t n) {
	RNS::Bytes data; uint32_t s = 0xA2A2u ^ (uint32_t)n;
	for (size_t i = 0; i < n; ++i) { s = s * 1664525u + 1013904223u; uint8_t b = (uint8_t)(s >> 24); data.append(&b, 1); }
	return data;   // TODO(verify): RNS::Bytes single-byte append (shared with Gate A)
}
static void on_sent(const RNS::Resource& r) {
	if (r.status() == RNS::Type::Resource::COMPLETE) { RNS::log("[gateA2] SEND COMPLETE"); ++next_idx; }
	else RNS::logf(RNS::LOG_ERROR, "[gateA2] SEND FAILED status=%d", (int)r.status());
	in_flight = false;
}
static void on_link(RNS::Link& l) { RNS::log("[gateA2] link up to Pi (over UDP)"); linked = true; }

static void role_setup() { pi_hash.assignHex(A2_PI_DEST_HEX); }
static void role_loop() {
	if (!linked && !pi_link) {
		if (!RNS::Transport::has_path(pi_hash)) { RNS::Transport::request_path(pi_hash); return; }
		RNS::Identity id = RNS::Identity::recall(pi_hash);
		RNS::Destination d(id, RNS::Type::Destination::OUT, RNS::Type::Destination::SINGLE, A2_APP_NAME, A2_ASPECT);
		pi_link = RNS::Link(d);
		pi_link.set_link_established_callback(on_link);
		return;
	}
	const int n = sizeof(A2_SIZES) / sizeof(A2_SIZES[0]);
	if (linked && !in_flight && next_idx < n) {
		size_t sz = A2_SIZES[next_idx];
		RNS::logf(RNS::LOG_NOTICE, "[gateA2] sending %u-byte Resource to Pi over UDP", (unsigned)sz);
		in_flight = true;
		new RNS::Resource(make_blob(sz), pi_link, true, false, on_sent, nullptr);
	}
}

// ============================================================================= BRIDGE
#elif defined(A2_ROLE_BRIDGE)
static void role_setup() {
	RNS::log("[gateA2] BRIDGE: routing LoRa mesh <-> Pi over UDP. No destination of its own.");
	RNS::log("[gateA2] point the Gate-A leaf (heltec-v3-client) at the PI's dest hash.");
}
static void role_loop() { /* transport does the work; nothing to drive here */ }

#else
#error "Define one of A2_ROLE_ENDPOINT / A2_ROLE_BRIDGE"
#endif

// ------------------------------------------------------------------------------- main
void setup() {
	Serial.begin(115200);
	uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(100);
	RNS::loglevel(RNS::LOG_NOTICE);
	wifi_up();
	rns_setup();
	role_setup();
}
void loop() {
	reticulum.loop();
	role_loop();
	RNS::Utilities::OS::sleep(0.01);
}
