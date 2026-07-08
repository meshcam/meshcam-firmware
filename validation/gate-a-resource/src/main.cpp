/*
 * Gate A — does microReticulum's Resource primitive move a full-res-scale blob
 * over a real LoRa link, on an MCU, with retry?  (see ../README.md and
 * docs/trailcam/design.md "Phase-3 bench-test plan")
 *
 * Two Heltec V3 boards, both running THIS firmware:
 *   - GATE_A_SERVER : creates a destination, accepts links + incoming Resources,
 *                     logs COMPLETE/FAILED + byte count + hash.
 *   - GATE_A_CLIENT : links to the server and pushes 5 KB -> 50 KB -> 250 KB
 *                     blobs in sequence, one at a time, logging progress.
 *
 * API surface is taken verbatim from microReticulum @ master:
 *   examples/link_native/src/main.cpp        (Identity / Destination / Link)
 *   examples/lora_transport/src/main.cpp     (Reticulum + LoRaInterface setup)
 *   src/microReticulum/Resource.h            (Resource send/accept)
 *   src/microReticulum/Link.h                (resource strategy + callbacks)
 *   src/microReticulum/Type.h                (status / strategy enums)
 *
 * Items still marked TODO are the handful of calls not exercised by an upstream
 * example (chiefly the raw-bytes fill) — confirm against the headers before first
 * build rather than trusting this blind.
 */

#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <LoRaInterface.h>          // vendored from examples/common/lora_interface/
#include <microReticulum.h>
#include <Arduino.h>

#include "gate_a_config.h"

// --- globals (file scope so lifetimes outlive setup(); the upstream example keeps
//     the filesystem as a local, which looks lifetime-fragile — keep it global here) ---
static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface lora_interface({RNS::Type::NONE});
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};

// ---------------------------------------------------------------------------
// Shared bring-up (mirrors examples/lora_transport)
// ---------------------------------------------------------------------------
static void reticulum_setup() {
	filesystem.init();
	filesystem.format();
	RNS::Utilities::OS::register_filesystem(filesystem);

	lora_interface = new LoRaInterface();
	lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
	RNS::Transport::register_interface(lora_interface);
	lora_interface.start();

	reticulum = RNS::Reticulum();
	reticulum.transport_enabled(true);
	reticulum.start();
}

// Build a deterministic, INCOMPRESSIBLE blob (LCG pseudo-random) so auto_compress=false
// is honest about airtime — a real JPEG won't compress either. Same size => same bytes,
// so the receiver could re-derive and verify, but we rely on Resource's own hash check.
static RNS::Bytes make_blob(size_t n) {
	RNS::Bytes data;
	uint32_t s = 0x1234567u ^ (uint32_t)n;
	data.reserve(n);
	for (size_t i = 0; i < n; ++i) {
		s = s * 1664525u + 1013904223u;          // Numerical Recipes LCG
		data.append((uint8_t)(s >> 24));         // RNS::Bytes::append(uint8_t) — Bytes.h:367
	}
	return data;
}

// ===========================================================================
#if defined(GATE_A_SERVER)
// ===========================================================================
static RNS::Destination server_destination({RNS::Type::NONE});
static RNS::Link        latest_link({RNS::Type::NONE});

static void on_resource_started(const RNS::Resource& resource) {
	RNS::logf(RNS::LOG_NOTICE, "[gateA] incoming resource, transfer_size=%u",
	          (unsigned)resource.get_transfer_size());
}

static void on_resource_concluded(const RNS::Resource& resource) {
	if (resource.status() == RNS::Type::Resource::COMPLETE) {
		RNS::logf(RNS::LOG_NOTICE, "[gateA] RESOURCE COMPLETE: %u bytes  hash=%s  compressed=%d",
		          (unsigned)resource.get_data_size(),
		          resource.get_hash().toHex().c_str(),
		          (int)resource.is_compressed());
	} else {
		RNS::logf(RNS::LOG_ERROR, "[gateA] RESOURCE FAILED status=%d",
		          (int)resource.status());
	}
}

static void on_link_established(RNS::Link& link) {
	RNS::log("[gateA] client linked");
	latest_link = link;
	link.set_resource_strategy(RNS::Type::Link::ACCEPT_ALL);
	link.set_resource_started_callback(on_resource_started);
	link.set_resource_concluded_callback(on_resource_concluded);
}

static void role_setup() {
	RNS::Identity id = RNS::Identity();
	server_destination = RNS::Destination(
		id, RNS::Type::Destination::IN, RNS::Type::Destination::SINGLE,
		GATE_A_APP_NAME, GATE_A_ASPECT);
	server_destination.accepts_links(true);
	server_destination.set_link_established_callback(on_link_established);
	RNS::logf(RNS::LOG_NOTICE, "[gateA] SERVER destination hash: %s",
	          server_destination.hash().toHex().c_str());
	RNS::log("[gateA] ^ paste that into gate_a_config.h GATE_A_SERVER_DEST_HEX and reflash the client");
}

static void role_loop() {
	// Announce SPARINGLY. The radio is half-duplex, and a blind periodic announce
	// during link setup is lethal: on the first hardware run the server's 5 s announce
	// TX overlapped the client's one-shot RTT packet — every single time (the handshake
	// phase-locks the timing) — leaving the link half-open and the transfer dead.
	// Transport already auto-replies to the client's path request with an announce, so
	// this ticker only matters for cold discovery; once a link is up, stop entirely.
	if (latest_link) return;
	static uint32_t last = 0;
	if (millis() - last > GATE_A_ANNOUNCE_SECS * 1000u) {
		last = millis();
		server_destination.announce();   // so the client can find a path
	}
}

// ===========================================================================
#elif defined(GATE_A_CLIENT)
// ===========================================================================
static RNS::Link client_link({RNS::Type::NONE});
static RNS::Bytes server_hash;
static int  next_size_idx = 0;
static bool send_in_flight = false;
static bool linked         = false;
// The in-flight Resource, kept so it can be DELETED after it concludes. Dropping the
// `new` pointer (the original code) leaks the whole thing — blob + encrypted token +
// parts list, ~5x the blob size — because the wrapper holds a shared_ptr ref forever;
// measured ~80 KB retained per completed 15 KB stage, which is what starved the
// 50 KB send into std::bad_alloc (found via the free-heap log, 2026-07-03).
static RNS::Resource* in_flight_resource = nullptr;

static void on_send_progress(const RNS::Resource& resource) {
	RNS::logf(RNS::LOG_NOTICE, "[gateA] send progress %.1f%%", resource.get_progress() * 100.0f);
}

static void on_send_concluded(const RNS::Resource& resource) {
	if (resource.status() == RNS::Type::Resource::COMPLETE) {
		RNS::logf(RNS::LOG_NOTICE, "[gateA] SEND COMPLETE (%u bytes)",
		          (unsigned)resource.get_data_size());
		++next_size_idx;          // advance to the next size only on success
	} else {
		// Half-open-link recovery (found on hardware 2026-07-02): the initiator's
		// one-shot RTT packet after proof validation is the ONLY thing that activates
		// the responder side + fires its link_established callback (Link.cpp
		// rtt_packet). Lose it (half-duplex collision with the server's announce) and
		// the server keeps resource_strategy=ACCEPT_NONE forever — every advertise is
		// silently dropped and retrying on the same link can never succeed. Tear down
		// and re-handshake instead.
		RNS::logf(RNS::LOG_ERROR, "[gateA] SEND FAILED status=%d — tearing down link to re-handshake",
		          (int)resource.status());
		if (client_link) client_link.teardown();
		client_link = RNS::Link({RNS::Type::NONE});
		linked = false;
	}
	send_in_flight = false;
}

static void on_link_closed(RNS::Link& link) {
	RNS::log("[gateA] link closed — will re-establish");
	client_link = RNS::Link({RNS::Type::NONE});
	linked = false;
	send_in_flight = false;
}

static void on_link_established(RNS::Link& link) {
	RNS::log("[gateA] link up to server");
	linked = true;
}

static void role_setup() {
	server_hash.assignHex(GATE_A_SERVER_DEST_HEX);
}

static void role_loop() {
	// 1. learn a path to the server
	if (!linked && !client_link) {
		if (!RNS::Transport::has_path(server_hash)) {
			// Rate-limit! An unconditional request_path here (10 ms loop) keeps the
			// radio permanently in TX, so it never RXes the server's announce and the
			// path is never learned — found on first hardware run 2026-07-02 (server
			// heard a packet every ~1.3 s at -28 dBm; client heard nothing, forever).
			static uint32_t last_req = 0;
			if (last_req == 0 || millis() - last_req > GATE_A_TICK_SECS * 1000u) {
				last_req = millis();
				RNS::log("[gateA] no path to server yet — requesting (listening for announce)");
				RNS::Transport::request_path(server_hash);
			}
			return;   // wait for the announce to arrive
		}
		RNS::Identity server_id = RNS::Identity::recall(server_hash);
		if (!server_id) {
			// has_path can go true before the identity lands in known_destinations
			// (or the microStore persist failed — watch for esp_littlefs unlink/rename
			// errors). Don't build a Link on a NONE identity; wait for the next announce.
			static uint32_t last_warn = 0;
			if (millis() - last_warn > 5000) {
				last_warn = millis();
				RNS::log("[gateA] have path but Identity::recall returned NONE — waiting");
			}
			return;
		}
		RNS::log("[gateA] path + identity learned; sending link request");
		RNS::Destination server_destination = RNS::Destination(
			server_id, RNS::Type::Destination::OUT, RNS::Type::Destination::SINGLE,
			GATE_A_APP_NAME, GATE_A_ASPECT);
		client_link = RNS::Link(server_destination);
		client_link.set_link_established_callback(on_link_established);
		client_link.set_link_closed_callback(on_link_closed);
		return;
	}

	// Reap the previous Resource once it has concluded. Deferred to here (not done
	// inside on_send_concluded) because the callback receives a reference into the
	// object being deleted — freeing it there would be use-after-free on return.
	if (!send_in_flight && in_flight_resource) {
		delete in_flight_resource;
		in_flight_resource = nullptr;
	}

	// 2. once linked, push the next blob (one at a time)
	const int n_sizes = sizeof(GATE_A_SIZES) / sizeof(GATE_A_SIZES[0]);
	if (linked && !send_in_flight && next_size_idx < n_sizes) {
		size_t sz = GATE_A_SIZES[next_size_idx];
		RNS::logf(RNS::LOG_NOTICE, "[gateA] sending %u-byte Resource ... (free heap %u)",
		          (unsigned)sz, (unsigned)ESP.getFreeHeap());
		RNS::Bytes blob = make_blob(sz);
		send_in_flight = true;
		// Resource(data, link, advertise=true, auto_compress=false, concluded, progress)
		// per src/microReticulum/Resource.h. auto_compress=false = honest airtime.
		in_flight_resource = new RNS::Resource(blob, client_link, true, false,
		                                       on_send_concluded, on_send_progress);
	}

	if (linked && next_size_idx >= n_sizes) {
		static bool done = false;
		if (!done) { RNS::log("[gateA] ALL SIZES SENT — Gate A pass if server logged COMPLETE for each"); done = true; }
	}
}

#else
#error "Define exactly one of GATE_A_SERVER / GATE_A_CLIENT (set by the PlatformIO env)"
#endif

// ---------------------------------------------------------------------------
void setup() {
	Serial.begin(115200);
	uint32_t t0 = millis();
	while (!Serial && millis() - t0 < 3000) delay(100);
	RNS::loglevel(RNS::LOG_DEBUG);   // bump to LOG_TRACE when debugging the radio
#if defined(ESP32)
	Serial.printf("[gateA] boot. free heap=%u, free psram=%u\n",
	              (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
#endif
	reticulum_setup();
	role_setup();
}

void loop() {
	reticulum.loop();
	role_loop();
	RNS::Utilities::OS::sleep(0.01);
}
