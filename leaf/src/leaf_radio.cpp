#include "leaf_radio.h"
#include "leaf_rails.h"
// Compiled only in radio builds. Without LEAF_RADIO this file is empty, so the LDF never
// pulls in microReticulum / LoRaInterface (keeps the pin-monitor diagnostic tiny).
#ifdef LEAF_RADIO

#include <Preferences.h>
#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <LoRaInterface.h>          // vendored in lib/lora_interface (proven in gate-b)
#include <microReticulum.h>

#include "leaf_serial_proto.h"      // tc_node_slug / tc_set_node_slug (wire identity)

// --- persistent RNS objects (rebuilt each boot; identity is the stable part) ----------
static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface lora_interface({RNS::Type::NONE});
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};
static RNS::Identity    identity({RNS::Type::NONE});
static RNS::Destination destination({RNS::Type::NONE});

static bool s_up    = false;   // full stack (radio incl.) came up
static bool s_radio = false;   // SX1262 init succeeded

// --- ADR state ------------------------------------------------------------------------
// Profile staged before begin() (the wake path applies an RTC-persisted grant before
// radio init) or retuned live after it. s_contact: any proof this wake that the gateway
// hears us — feeds the revert-to-base counter in main.cpp.
static uint8_t s_profile = LORA_PROFILE_BASE;
static bool    s_contact = false;

bool leaf_radio_set_profile(uint8_t idx) {
    if (idx >= LORA_PROFILE_COUNT) return false;
    s_profile = idx;
    if (LoRaInterface::active) return LoRaInterface::active->set_profile(idx);
    return true;   // staged; leaf_radio_begin() applies it before radio init
}

const char* leaf_radio_profile_name() { return LORA_PROFILES[s_profile].name; }

// TX power (tx-power calibration, 2026-07-18). Same stage-or-live shape as
// set_profile(): callable before begin() (stages for radio init) or after (pokes
// the live SX1262). s_tx_dbm mirrors the interface so the announce tail can
// report the power actually in use.
static int s_tx_dbm = LORA_TX_DBM;

bool leaf_radio_set_tx_dbm(int dbm) {
    if (LoRaInterface::active && !LoRaInterface::active->set_tx_dbm(dbm))
        return false;
    s_tx_dbm = dbm;
    return true;
}

int leaf_radio_tx_dbm() { return s_tx_dbm; }

bool leaf_radio_had_contact() { return s_contact; }

// App namespace for the leaf's IN destination. The destination hash derives from this +
// the (persistent) identity, so it too is stable across wakes.
static const char* APP_NAME = "trailcam";
static const char* ASPECT   = "leaf";

// NVS-backed identity: the private key survives deep sleep AND power loss.
static const char* NVS_NS  = "leaf";
static const char* NVS_KEY = "rns_prv";

// --- inbound packets (gateway -> leaf commands) --------------------------------------
static void on_packet(const RNS::Bytes& data, const RNS::Packet& /*packet*/) {
    s_contact = true;   // any packet addressed to us = the gateway hears us
    Serial.printf("[radio] RX packet: %u bytes -> (stub) would parse gateway command\n",
                  (unsigned)data.size());
    // TODO(command): parse "send hi-res of image N" / schedule update / time sync.
}

static RNS::Identity load_or_create_identity() {
    Preferences p;
    p.begin(NVS_NS, /*readOnly=*/false);
    size_t n = p.getBytesLength(NVS_KEY);
    if (n > 0 && n <= 128) {
        uint8_t buf[128];
        p.getBytes(NVS_KEY, buf, n);
        p.end();
        RNS::Identity id(false);                    // empty, no keys yet
        if (id.load_private_key(RNS::Bytes(buf, n))) {
            Serial.printf("[radio] identity loaded from NVS (%u key bytes)\n", (unsigned)n);
            return id;
        }
        Serial.println("[radio] NVS key load FAILED -> regenerating");
    } else {
        p.end();
    }
    // Cold boot (or unusable stored key): mint once, persist forever.
    RNS::Identity id(true);
    RNS::Bytes prv = id.get_private_key();
    Preferences pw;
    pw.begin(NVS_NS, false);
    pw.putBytes(NVS_KEY, prv.data(), prv.size());
    pw.end();
    Serial.printf("[radio] identity CREATED + stored to NVS (%u key bytes)\n",
                  (unsigned)prv.size());
    return id;
}

bool leaf_radio_begin() {
    leaf_radio_rail(true);   // PCB: power LORA_3V3 (radio + microSD; no-op on bench)
    if (s_up) return s_radio;

    // 1. Filesystem: mount existing, format only on the first-ever boot (no per-wake wear).
    filesystem.init(/*reformatOnFail=*/true);
    RNS::Utilities::OS::register_filesystem(filesystem);

    // 2. Persistent identity + the leaf's stable IN destination.
    identity    = load_or_create_identity();
    destination = RNS::Destination(identity, RNS::Type::Destination::IN,
                                   RNS::Type::Destination::SINGLE, APP_NAME, ASPECT);
    destination.set_proof_strategy(RNS::Type::Destination::PROVE_ALL);
    destination.set_packet_callback(on_packet);
    Serial.printf("[radio] leaf destination %s.%s  hash=%s\n",
                  APP_NAME, ASPECT, destination.hash().toHex().c_str());

    // Derive the wire identity from the destination hash (node-identity.md): stable for
    // the life of the NVS identity, zero-config, never a human name. No-op if a
    // LEAF_NODE_SLUG build flag pinned a fixed slug.
    {
        char derived[24];
        snprintf(derived, sizeof(derived), "leaf-%.8s", destination.hash().toHex().c_str());
        tc_set_node_slug(derived);
        Serial.printf("[radio] wire identity: %s\n", tc_node_slug());
    }

    // 3. SX1262 bring-up (same config proven in gate-b). start() returns false if the
    //    radio doesn't answer (e.g. antenna/BUSY issue); we keep RNS up regardless so the
    //    identity/destination are still valid, but no packets go out.
    lora_interface = new LoRaInterface();
    // Apply any staged (RTC-persisted) ADR grant BEFORE start() so this whole wake —
    // announce included — runs on the granted profile. The ctor registered itself as
    // LoRaInterface::active.
    if (s_profile != LORA_PROFILE_BASE && LoRaInterface::active)
        LoRaInterface::active->set_profile(s_profile);
    // Same for a staged TX power override (tx-power sweep): begin() must already
    // transmit at it, announce included.
    if (s_tx_dbm != LORA_TX_DBM && LoRaInterface::active)
        LoRaInterface::active->set_tx_dbm(s_tx_dbm);
    lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);  // relays; power-mode tuning TBD
    RNS::Transport::register_interface(lora_interface);
    s_radio = lora_interface.start();
    Serial.printf("[radio] SX1262 init %s\n", s_radio ? "OK" : "FAILED (RNS up, no TX/RX)");

    // 4. Reticulum transport.
    reticulum = RNS::Reticulum();
    reticulum.transport_enabled(true);
    reticulum.start();

    s_up = true;
    return s_radio;
}

// Health counters staged by leaf_radio_set_health() each wake; ride every announce's
// app_data (bug 5) so push failures / battery are visible remotely, not just on serial.
static uint32_t s_h_pir = 0, s_h_cap = 0, s_h_pf = 0;
static float    s_h_vbat = NAN;
static bool     s_h_set  = false;

// Command-receipt acks (bug 8): server command ids received this wake, announced as
// "a=<id>,<id>" so the gateway can flip them delivered -> received server-side.
// One-shot: cleared after they ride an announce.
#define LEAF_ACK_SLOTS 4
static uint32_t s_ack_ids[LEAF_ACK_SLOTS];
static int      s_ack_n = 0;

void leaf_radio_set_health(uint32_t pir_wakes, uint32_t captures, uint32_t push_fails,
                           float vbat_v) {
    s_h_pir = pir_wakes; s_h_cap = captures; s_h_pf = push_fails;
    s_h_vbat = vbat_v;
    s_h_set  = true;
}

void leaf_radio_announce(const char* status) {
    if (!s_up || !s_radio) return;
    char ad[128];
    int n = snprintf(ad, sizeof(ad), "%s", status ? status : "");
    // Node slug (leaf-0.13.0, multi-leaf roster): the gateway routes commands and
    // attributes telemetry per camera, and the announce is the only channel that
    // reliably ties our identity to a slug. Right after status so truncation can
    // never eat it.
    if (n >= 0 && n < (int)sizeof(ad))
        n += snprintf(ad + n, sizeof(ad) - n, " n=%s", tc_node_slug());
    // TX power in use (tx-power calibration): lets gateway-side telemetry attribute
    // each checkin's RSSI to the power it was sent at. Cheap (6 bytes), always on.
    if (n >= 0 && n < (int)sizeof(ad))
        n += snprintf(ad + n, sizeof(ad) - n, " tx=%d", s_tx_dbm);
    if (s_h_set && n >= 0 && n < (int)sizeof(ad)) {
        n += snprintf(ad + n, sizeof(ad) - n, " p=%lu c=%lu f=%lu",
                      (unsigned long)s_h_pir, (unsigned long)s_h_cap,
                      (unsigned long)s_h_pf);
        if (!isnan(s_h_vbat) && n > 0 && n < (int)sizeof(ad))
            n += snprintf(ad + n, sizeof(ad) - n, " vb=%.2f", s_h_vbat);
    }
    if (s_ack_n && n > 0 && n < (int)sizeof(ad)) {
        n += snprintf(ad + n, sizeof(ad) - n, " a=");
        for (int i = 0; i < s_ack_n && n > 0 && n < (int)sizeof(ad); i++)
            n += snprintf(ad + n, sizeof(ad) - n, "%s%lu", i ? "," : "",
                          (unsigned long)s_ack_ids[i]);
        s_ack_n = 0;   // one announce carries them; the server ack is idempotent anyway
    }
    RNS::Bytes app_data(ad);
    destination.announce(app_data);
    reticulum.loop();
    Serial.printf("[radio] announced (status=\"%s\")\n", ad);
}

// --- gateway thumbnail push (Gate A-proven Link + Resource flow) -----------------------
// LEAF_GATEWAY_DEST_HEX is the gateway's destination hash (32 hex chars), baked at build
// time exactly like gate-a's client. Unset/empty -> announce-only alert (the old path).
// BENCH NOTE: the gate-a server mints a RANDOM identity per boot, so this needs rebaking
// after any server reflash; a production gateway will have a persistent identity.
#ifndef LEAF_GATEWAY_DEST_HEX
#define LEAF_GATEWAY_DEST_HEX ""
#endif

static volatile bool s_gw_link_up   = false;
static volatile int  s_gw_res_state = -1;    // -1 = in flight, else Type::Resource status
static RNS::Link s_gw_link({RNS::Type::NONE});

static void on_gw_link_established(RNS::Link& /*link*/) { s_gw_link_up = true; s_contact = true; }
static void on_gw_link_closed(RNS::Link& /*link*/)      { s_gw_link_up = false; }
static void on_gw_send_concluded(const RNS::Resource& r) { s_gw_res_state = (int)r.status(); }
static void on_gw_send_progress(const RNS::Resource& r) {
    static uint8_t last_bucket = 0xFF;
    uint8_t bucket = (uint8_t)(r.get_progress() * 5.0f);   // log at ~20% steps
    if (bucket != last_bucket) {
        last_bucket = bucket;
        Serial.printf("[radio] thumb push %.0f%%\n", r.get_progress() * 100.0f);
    }
}

// Pump RNS for up to `ms`, returning early when *flag becomes true.
static bool pump_until(volatile bool* flag, uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        reticulum.loop();
        if (flag && *flag) return true;
        delay(5);
    }
    return flag && *flag;
}

static void on_gw_link_packet(const RNS::Bytes& plaintext, const RNS::Packet& packet);

// Establish (or reuse) the link to the baked gateway destination. Gate A findings as
// policy: rate-limited path requests (an every-loop request keeps the half-duplex radio
// deaf), full teardown + ONE in-wake retry on failure (second handshakes land).
static bool establish_gateway_link() {
    if (s_gw_link && s_gw_link_up) return true;   // still up from earlier this wake
    const char* gw_hex = LEAF_GATEWAY_DEST_HEX;
    if (strlen(gw_hex) != 32) return false;
    RNS::Bytes gw_hash;
    gw_hash.assignHex(gw_hex);
    uint32_t t0 = millis(), last_req = 0;
    while (!RNS::Transport::has_path(gw_hash) && millis() - t0 < 20000) {
        if (last_req == 0 || millis() - last_req > 4000) {
            last_req = millis();
            Serial.println("[radio] requesting path to gateway...");
            RNS::Transport::request_path(gw_hash);
        }
        reticulum.loop();
        delay(5);
    }
    RNS::Identity gw_id = RNS::Identity::recall(gw_hash);
    if (!RNS::Transport::has_path(gw_hash) || !gw_id) {
        Serial.println("[radio] no path/identity for gateway");
        return false;
    }
    RNS::Destination gw_dest(gw_id, RNS::Type::Destination::OUT,
                             RNS::Type::Destination::SINGLE, "trailcam_gatea", "resource");
    for (int attempt = 1; ; attempt++) {
        s_gw_link_up = false;
        s_gw_link = RNS::Link(gw_dest);
        s_gw_link.set_link_established_callback(on_gw_link_established);
        s_gw_link.set_link_closed_callback(on_gw_link_closed);
        if (pump_until(&s_gw_link_up, 15000)) {
            s_gw_link.set_packet_callback(on_gw_link_packet);   // "have?" replies
            return true;
        }
        Serial.printf("[radio] gateway link FAILED to establish (attempt %d) -> teardown\n", attempt);
        s_gw_link.teardown();
        s_gw_link = RNS::Link({RNS::Type::NONE});
        if (attempt >= 2) return false;
        // Let the air clear (whatever ate the request is mid-TX) before the retry.
        pump_until(nullptr, 2000);
    }
}

static void teardown_gateway_link() {
    if (s_gw_link) s_gw_link.teardown();
    s_gw_link = RNS::Link({RNS::Type::NONE});
    s_gw_link_up = false;
}

// --- transfer dedup ("have?" protocol, leaf-0.17.0) ------------------------------------
// A lost RNS conclusion ack used to mean a full re-transfer of data the gateway already
// holds — the 07-17 walk's "battery-life bug wearing a range bug's clothing". Before
// committing a Resource, ask over the link what the gateway has: one tiny packet each
// way vs 30-160 s of re-sent chunks. No reply (old gateway firmware, lost packet) fails
// OPEN to sending — exactly the pre-0.17 behavior.
static volatile bool s_have_reply = false;
static char          s_have_json[224];

static void on_gw_link_packet(const RNS::Bytes& plaintext, const RNS::Packet& /*packet*/) {
    size_t n = plaintext.size();
    if (n >= sizeof(s_have_json)) n = sizeof(s_have_json) - 1;
    memcpy(s_have_json, plaintext.data(), n);
    s_have_json[n] = 0;
    s_have_reply = true;
}

static bool query_gateway_have(const char* eid, const char* kind,
                               bool* complete, uint32_t* got_mask) {
    *complete = false;
    *got_mask = 0;
    if (!eid || !eid[0] || !s_gw_link || !s_gw_link_up) return false;
    char q[128];
    int n = snprintf(q, sizeof(q),
                     "{\"q\":\"have\",\"event_id\":\"%s\",\"kind\":\"%s\"}", eid, kind);
    s_have_reply = false;
    RNS::Packet(s_gw_link, RNS::Bytes((const uint8_t*)q, (size_t)n)).send();
    // 8 s, not 4 (2026-07-18 air test): the gateway can be inside a blocking TLS
    // beat-POST when the query lands, and its staged reply waits that POST out.
    // Only a live-gateway path ever waits here (the link just established), so the
    // cap is the pathological ceiling, not the typical cost (~1 s).
    if (!pump_until(&s_have_reply, 8000)) {
        Serial.println("[radio] have? no reply -> fail-open send");
        return false;
    }
    // The echo must match — a stale reply for another event must never skip this one.
    if (!strstr(s_have_json, eid)) return false;
    const char* t;
    if ((t = strstr(s_have_json, "\"complete\":")))
        *complete = atoi(t + 11) != 0;
    if ((t = strstr(s_have_json, "\"got_mask\":")))
        *got_mask = (uint32_t)strtoul(t + 11, nullptr, 10);
    Serial.printf("[radio] have? %s %s -> complete=%d mask=0x%lx\n",
                  eid, kind, (int)*complete, (unsigned long)*got_mask);
    return true;
}

// One enveloped Resource over the (established) gateway link. Blocks to conclusion.
static bool send_one_resource(const char* hdr, size_t hlen,
                              const uint8_t* body, size_t blen, uint32_t timeout_ms) {
    RNS::Bytes payload(reinterpret_cast<const uint8_t*>(hdr), hlen);
    payload.append(body, blen);
    s_gw_res_state = -1;
    RNS::Resource* res = new RNS::Resource(payload, s_gw_link,
                                           /*advertise=*/true, /*auto_compress=*/false,
                                           on_gw_send_concluded, on_gw_send_progress);
    uint32_t t0 = millis();
    while (s_gw_res_state == -1 && millis() - t0 < timeout_ms) {
        reticulum.loop();
        delay(5);
    }
    const bool ok = (s_gw_res_state == (int)RNS::Type::Resource::COMPLETE);
    delete res;   // or ~5x the payload stays referenced forever (Gate A finding #4)
    return ok;
}

bool leaf_radio_send_alert(const uint8_t* buf, size_t len,
                           const char* event_id, int64_t captured_at) {
    if (!leaf_radio_begin()) {           // ensure stack up; false = no radio
        Serial.println("[radio] send_alert: radio down, alert NOT sent");
        return false;
    }
    // ORDER MATTERS: the push goes FIRST, the alert announce LAST. Announcing first
    // loses the handshake every time once the path is persisted: the gateway (a
    // transport node) rebroadcasts our announce for ~0.5 s, and with no path-request
    // round trip to space things out, the link request fires straight into that
    // rebroadcast — half-duplex, phase-locked, 3-for-3 reproducible (2026-07-03).
    char status[24];
    snprintf(status, sizeof(status), "alert:%u", (unsigned)len);

    const char* gw_hex = LEAF_GATEWAY_DEST_HEX;
    if (!buf || !len || strlen(gw_hex) != 32) {
        leaf_radio_announce(status);
        Serial.printf("[radio] alert announced only (%u-byte thumbnail held; no gateway baked)\n",
                      (unsigned)len);
        return true;
    }

    if (!establish_gateway_link()) {
        leaf_radio_announce(status);   // at least tell the world an event fired
        return false;
    }
    Serial.println("[radio] gateway link up, pushing thumbnail as Resource");

    // Dedup first: if the gateway already took custody of this thumb (we lost the
    // conclusion ack last attempt), the whole push is one packet round trip.
    {
        bool have = false;
        uint32_t mask = 0;
        query_gateway_have(event_id, "thumb", &have, &mask);
        if (have) {
            Serial.printf("[radio] gateway already has thumb %s -> push skipped (dedup)\n",
                          event_id);
            teardown_gateway_link();
            leaf_radio_announce(status);
            return true;
        }
    }

    // 3. The thumbnail as one enveloped Resource (JSON header + '\n' + JPEG) so the
    //    gateway uploads under the leaf's REAL event id — that's what lets a fetch_full
    //    for this photo find the stored full on OUR SD card, and what dedupes the
    //    serial + mesh copies of the same event server-side.
    char hdr[160];
    int hlen = snprintf(hdr, sizeof(hdr),
                        "{\"event_id\":\"%s\",\"captured_at\":%lld,\"camera\":\"%s\",\"kind\":\"thumb\"}\n",
                        event_id ? event_id : "", (long long)captured_at, tc_node_slug());
    uint32_t t0 = millis();
    const bool ok = send_one_resource(hdr, (size_t)hlen, buf, len, 120000);
    Serial.printf("[radio] thumb push %s (status=%d, %u bytes, %lu ms)\n",
                  ok ? "COMPLETE" : "FAILED", s_gw_res_state, (unsigned)len,
                  (unsigned long)(millis() - t0));

    // 4. Be a good peer: close the link before sleeping (the gateway would otherwise hold
    //    a stale link until its stale-time). Then announce the alert — AFTER the push so
    //    the gateway's rebroadcast of it can't collide with our own handshake.
    teardown_gateway_link();
    leaf_radio_announce(status);
    return ok;
}

// --- mesh full-res: sequential enveloped chunks under the gateway's 25 KB ceiling ------
#ifndef LEAF_MESH_CHUNK
#define LEAF_MESH_CHUNK (16 * 1024)
#endif

bool leaf_radio_send_full(const uint8_t* buf, size_t len, const char* event_id,
                          int64_t captured_at, const char* quality) {
    if (!buf || !len || !leaf_radio_begin()) return false;
    if (!establish_gateway_link()) {
        Serial.println("[radio] send_full: no gateway link -> aborted");
        return false;
    }
    const unsigned chunks = (len + LEAF_MESH_CHUNK - 1) / LEAF_MESH_CHUNK;
    Serial.printf("[radio] send_full %s: %u bytes as %u chunk(s), quality=%s\n",
                  event_id, (unsigned)len, chunks, quality);
    // Dedup / resume: a previous attempt's chunks may already sit in the gateway's
    // assembly (its got_mask survives between our wakes), or the whole full may be
    // in custody with only our conclusion ack lost. Mask is only trustworthy when
    // every chunk index fits in it.
    bool     have = false;
    uint32_t mask = 0;
    query_gateway_have(event_id, "full", &have, &mask);
    if (have) {
        Serial.printf("[radio] gateway already has full %s -> send skipped (dedup)\n",
                      event_id);
        teardown_gateway_link();
        return true;
    }
    if (chunks > 32) mask = 0;
    bool ok = true;
    for (unsigned i = 0; i < chunks && ok; i++) {
        if (i < 32 && (mask & (1UL << i))) {
            Serial.printf("[radio] send_full %s: chunk %u/%u already at gateway -> skipped\n",
                          event_id, i + 1, chunks);
            continue;
        }
        const size_t off  = (size_t)i * LEAF_MESH_CHUNK;
        const size_t blen = min((size_t)LEAF_MESH_CHUNK, len - off);
        char hdr[224];
        int hlen = snprintf(hdr, sizeof(hdr),
            "{\"event_id\":\"%s\",\"captured_at\":%lld,\"camera\":\"%s\","
            "\"kind\":\"full\",\"quality\":\"%s\",\"chunk\":%u,\"chunks\":%u,"
            "\"offset\":%u,\"total\":%u}\n",
            event_id, (long long)captured_at, tc_node_slug(),
            quality && quality[0] ? quality : "max",
            i, chunks, (unsigned)off, (unsigned)len);
        // Generous per-chunk budget: 16 KB ≈ 100-160 s at bench goodput, plus retries.
        ok = send_one_resource(hdr, (size_t)hlen, buf + off, blen, 300000);
        Serial.printf("[radio] send_full %s: chunk %u/%u %s\n",
                      event_id, i + 1, chunks, ok ? "OK" : "FAILED");
        if (!ok && establish_gateway_link()) {
            // Before re-sending, ask what actually landed: a "failed" chunk whose
            // conclusion ack got lost is already in the assembly (the exact waste
            // this protocol exists to kill).
            bool rhave = false;
            uint32_t rmask = 0;
            if (query_gateway_have(event_id, "full", &rhave, &rmask)) {
                if (rhave) { ok = true; break; }   // custody taken; nothing left to send
                if (i < 32 && (rmask & (1UL << i))) {
                    Serial.printf("[radio] send_full %s: chunk %u/%u landed despite "
                                  "lost ack -> not re-sent\n", event_id, i + 1, chunks);
                    mask = rmask;
                    ok = true;
                    continue;
                }
                if (chunks <= 32) mask = rmask;   // resume with fresh gateway state
            }
            ok = send_one_resource(hdr, (size_t)hlen, buf + off, blen, 300000);
            Serial.printf("[radio] send_full %s: chunk %u/%u retry %s\n",
                          event_id, i + 1, chunks, ok ? "OK" : "FAILED");
        }
    }
    teardown_gateway_link();
    Serial.printf("[radio] send_full %s: %s\n", event_id, ok ? "ALL CHUNKS SENT" : "ABORTED");
    return ok;
}

// --- mesh command downlink: RX window -> queue -> execute after the window -------------
#define MESH_CMD_SLOTS 4
static char s_mesh_cmds[MESH_CMD_SLOTS][768];
static int  s_mesh_cmd_count = 0;

static void on_mesh_cmd_packet(const RNS::Bytes& data, const RNS::Packet& /*packet*/) {
    s_contact = true;   // a packet addressed to us = the gateway hears our announces
    if (data.size() < 8 || data.size() >= 768) return;
    if (s_mesh_cmd_count >= MESH_CMD_SLOTS) return;
    memcpy(s_mesh_cmds[s_mesh_cmd_count], data.data(), data.size());
    s_mesh_cmds[s_mesh_cmd_count][data.size()] = 0;
    s_mesh_cmd_count++;
    Serial.printf("[radio] mesh command queued (%u bytes)\n", (unsigned)data.size());
}

// Tiny numeric extractor for the ack path ("id": 57 in the server's command JSON).
// Gateway-minted packets (time_sync, radio_profile grants) carry no id -> no ack.
static bool mesh_cmd_id(const char* json, uint32_t* out) {
    const char* p = strstr(json, "\"id\"");
    if (!p) return false;
    p += 4;
    while (*p == ' ' || *p == ':') p++;
    if (*p < '0' || *p > '9') return false;
    *out = (uint32_t)strtoul(p, nullptr, 10);
    return true;
}

void leaf_radio_poll_commands(uint32_t ms, tc_cmd_handler handler) {
    if (!leaf_radio_begin()) return;
    s_mesh_cmd_count = 0;
    destination.set_packet_callback(on_mesh_cmd_packet);
    leaf_radio_rx_window(ms);
    // Receipt acks FIRST (bug 8): stage the server ids we just received and announce
    // them before executing anything — receipt is not completion, and a fetch_full
    // handler can hold the wake for minutes. The gateway hears the announce and flips
    // the commands delivered -> received so the server stops redelivering.
    for (int i = 0; i < s_mesh_cmd_count; i++) {
        uint32_t id = 0;
        if (mesh_cmd_id(s_mesh_cmds[i], &id) && s_ack_n < LEAF_ACK_SLOTS)
            s_ack_ids[s_ack_n++] = id;
    }
    if (s_ack_n) leaf_radio_announce("ack");
    // Execute AFTER the window: a handler can run for many minutes (chunked full-res)
    // and must not be invoked from inside reticulum.loop()'s packet dispatch.
    for (int i = 0; i < s_mesh_cmd_count; i++) {
        tc_handle_cmd_json(s_mesh_cmds[i], handler);
    }
    s_mesh_cmd_count = 0;
}

void leaf_radio_rx_window(uint32_t ms) {
    if (!s_up) return;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        reticulum.loop();               // service RX; on_packet fires for inbound
        delay(5);
    }
}

#endif // LEAF_RADIO
