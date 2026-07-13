#include "surveyor_radio.h"

#include <Preferences.h>
#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <LoRaInterface.h>          // vendored in lib/lora_interface (byte-identical rule)
#include <microReticulum.h>

// --- persistent RNS objects (identity is the stable part) ------------------------------
static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface lora_interface({RNS::Type::NONE});
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};
static RNS::Identity    identity({RNS::Type::NONE});
static RNS::Destination destination({RNS::Type::NONE});

static bool s_up    = false;
static bool s_radio = false;

// Same aspect as the leaf — see the header for why.
static const char* APP_NAME = "trailcam";
static const char* ASPECT   = "leaf";

// NVS-backed identity: survives power loss, so the surveyor's destination hash (and the
// gateway's memory of it) is stable across field days.
static const char* NVS_NS  = "surveyor";
static const char* NVS_KEY = "rns_prv";

static RNS::Identity load_or_create_identity() {
    Preferences p;
    p.begin(NVS_NS, /*readOnly=*/false);
    size_t n = p.getBytesLength(NVS_KEY);
    if (n > 0 && n <= 128) {
        uint8_t buf[128];
        p.getBytes(NVS_KEY, buf, n);
        p.end();
        RNS::Identity id(false);
        if (id.load_private_key(RNS::Bytes(buf, n))) {
            Serial.printf("[radio] identity loaded from NVS (%u key bytes)\n", (unsigned)n);
            return id;
        }
        Serial.println("[radio] NVS key load FAILED -> regenerating");
    } else {
        p.end();
    }
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

// --- inbound packets (gateway -> surveyor: radio_profile / time_sync / probe_ack) ------
// Same queue-then-dispatch pattern as the leaf: handlers can transmit or retune, so they
// must never run from inside reticulum.loop()'s packet dispatch.
#define SV_CMD_SLOTS 4
static char s_cmds[SV_CMD_SLOTS][768];
static int  s_cmd_count = 0;

static void on_packet(const RNS::Bytes& data, const RNS::Packet& /*packet*/) {
    if (data.size() < 8 || data.size() >= 768) return;
    if (s_cmd_count >= SV_CMD_SLOTS) return;
    memcpy(s_cmds[s_cmd_count], data.data(), data.size());
    s_cmds[s_cmd_count][data.size()] = 0;
    s_cmd_count++;
    Serial.printf("[radio] gateway packet queued (%u bytes)\n", (unsigned)data.size());
}

// --- gateway discovery: which mesh are we standing in? ----------------------------------
// No single baked destination: candidates come from (1) the NVS-cached last winner,
// (2) the SURVEYOR_GATEWAY_CANDIDATES build flag (comma-separated 32-hex hashes, one per
// mesh), (3) any trailcam_gatea.resource announce overheard on the air. The link path
// below path-requests candidates round-robin — only the mesh we're actually standing in
// answers for its hash — and persists the winner. One image walks every mesh; site
// attribution happens gateway-side (GW_SITE rides the telemetry beat), so the surveyor
// never needs to know WHERE it is, only WHO answers.
#ifndef SURVEYOR_GATEWAY_CANDIDATES
#define SURVEYOR_GATEWAY_CANDIDATES ""
#endif

#define SV_GW_MAX 5
static char s_gw_cands[SV_GW_MAX][33];
static int  s_gw_n     = 0;
static bool s_gw_known = false;   // a mesh has answered / been adopted this boot

static bool gw_valid_hex(const char* h) { return h && strlen(h) == 32; }

// Move `hex` to the front of the candidate list (insert if new, evict the tail if full).
static void gw_promote(const char* hex_in) {
    char hex[33];
    snprintf(hex, sizeof(hex), "%s", hex_in);   // hex_in may point INTO the array
    if (!gw_valid_hex(hex)) return;
    int found = -1;
    for (int i = 0; i < s_gw_n; i++)
        if (strcmp(s_gw_cands[i], hex) == 0) { found = i; break; }
    if (found == 0) return;
    if (found < 0 && s_gw_n < SV_GW_MAX) s_gw_n++;
    const int top = (found > 0) ? found : s_gw_n - 1;
    for (int i = top; i > 0; i--) strcpy(s_gw_cands[i], s_gw_cands[i - 1]);
    strcpy(s_gw_cands[0], hex);
}

static void gw_persist(const char* hex) {
    Preferences p;
    p.begin(NVS_NS, false);
    p.putString("gw", hex);
    p.end();
}

static void gw_load_candidates() {
    s_gw_n = 0;
    static char flag[32 * SV_GW_MAX + SV_GW_MAX];
    snprintf(flag, sizeof(flag), "%s", SURVEYOR_GATEWAY_CANDIDATES);
    for (char* tok = strtok(flag, ", "); tok && s_gw_n < SV_GW_MAX;
         tok = strtok(nullptr, ", "))
        if (gw_valid_hex(tok)) strcpy(s_gw_cands[s_gw_n++], tok);
    Preferences p;
    p.begin(NVS_NS, true);
    String last = p.getString("gw", "");
    p.end();
    if (last.length() == 32) gw_promote(last.c_str());   // last winner probes first
    Serial.printf("[gw-disc] %d gateway candidate(s), first %.8s\n",
                  s_gw_n, s_gw_n ? s_gw_cands[0] : "-");
}

// Any gateway we overhear announcing joins the ladder at the front — this is what makes
// a brand-new mesh (hash not baked anywhere) work with zero config.
class GatewayAnnounceHandler : public RNS::AnnounceHandler {
public:
    GatewayAnnounceHandler() : AnnounceHandler("trailcam_gatea.resource") {}
    void received_announce(const RNS::Bytes& destination_hash,
                           const RNS::Identity& /*announced_identity*/,
                           const RNS::Bytes& /*app_data*/) override {
        std::string hex = destination_hash.toHex();
        if (hex.size() != 32) return;
        Serial.printf("[gw-disc] gateway announce heard: %.8s -> adopted\n", hex.c_str());
        gw_promote(hex.c_str());
        gw_persist(s_gw_cands[0]);
        s_gw_known = true;
    }
};

const char* sv_gateway_short() {
    static char s[10];
    if (!s_gw_n) return "--------";
    snprintf(s, sizeof(s), "%.8s", s_gw_cands[0]);
    return s;
}

bool sv_gateway_known() { return s_gw_known; }

bool sv_radio_ok() { return s_radio; }

bool sv_radio_begin() {
    if (s_up) return s_radio;

    filesystem.init(/*reformatOnFail=*/true);
    RNS::Utilities::OS::register_filesystem(filesystem);

    identity    = load_or_create_identity();
    destination = RNS::Destination(identity, RNS::Type::Destination::IN,
                                   RNS::Type::Destination::SINGLE, APP_NAME, ASPECT);
    destination.set_proof_strategy(RNS::Type::Destination::PROVE_ALL);
    destination.set_packet_callback(on_packet);
    Serial.printf("[radio] surveyor destination %s.%s  hash=%s\n",
                  APP_NAME, ASPECT, destination.hash().toHex().c_str());

    lora_interface = new LoRaInterface();
    lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
    RNS::Transport::register_interface(lora_interface);
    s_radio = lora_interface.start();
    Serial.printf("[radio] SX1276 init %s\n", s_radio ? "OK" : "FAILED (RNS up, no TX/RX)");

    reticulum = RNS::Reticulum();
    reticulum.transport_enabled(true);
    reticulum.start();

    gw_load_candidates();
    static RNS::HAnnounceHandler gw_ah(new GatewayAnnounceHandler());
    RNS::Transport::register_announce_handler(gw_ah);

    s_up = true;
    return s_radio;
}

void sv_radio_announce(const char* status) {
    if (!s_up || !s_radio) return;
    RNS::Bytes app_data(status ? status : "");
    destination.announce(app_data);
    reticulum.loop();
    Serial.printf("[radio] announced (status=\"%s\")\n", status ? status : "");
}

// --- gateway link + Resource send (leaf_radio.cpp flow, findings preserved) ------------

static volatile bool s_gw_link_up   = false;
static volatile int  s_gw_res_state = -1;
static RNS::Link s_gw_link({RNS::Type::NONE});

static void on_gw_link_established(RNS::Link& /*link*/) { s_gw_link_up = true; }
static void on_gw_link_closed(RNS::Link& /*link*/)      { s_gw_link_up = false; }
static void on_gw_send_concluded(const RNS::Resource& r) { s_gw_res_state = (int)r.status(); }
static void on_gw_send_progress(const RNS::Resource& r) {
    static uint8_t last_bucket = 0xFF;
    uint8_t bucket = (uint8_t)(r.get_progress() * 5.0f);
    if (bucket != last_bucket) {
        last_bucket = bucket;
        Serial.printf("[radio] probe push %.0f%%\n", r.get_progress() * 100.0f);
    }
}

static bool pump_until(volatile bool* flag, uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        reticulum.loop();
        if (flag && *flag) return true;
        delay(5);
    }
    return flag && *flag;
}

// One path-discovery + link sequence on the CURRENT radio profile. Path answers can
// come from the PERSISTED path table with zero RF exchange, so only a formed LINK
// proves this profile actually reaches the gateway.
static bool try_link_on_current_profile(RNS::Bytes* hashes, uint32_t path_ms,
                                        int link_attempts) {
    int have = -1;
    uint32_t t0 = millis(), last_req = 0;
    int rr = 0;
    while (have < 0 && millis() - t0 < path_ms) {
        for (int i = 0; i < s_gw_n; i++)
            if (RNS::Transport::has_path(hashes[i])) { have = i; break; }
        if (have >= 0) break;
        if (last_req == 0 || millis() - last_req > 4000) {
            last_req = millis();
            const int k = rr++ % s_gw_n;
            Serial.printf("[radio] requesting path to gateway %.8s (%s)...\n",
                          s_gw_cands[k], sv_radio_profile_name());
            RNS::Transport::request_path(hashes[k]);
        }
        reticulum.loop();
        delay(5);
    }
    if (have < 0) return false;
    RNS::Identity gw_id = RNS::Identity::recall(hashes[have]);
    if (!gw_id) return false;
    RNS::Destination gw_dest(gw_id, RNS::Type::Destination::OUT,
                             RNS::Type::Destination::SINGLE, "trailcam_gatea", "resource");
    for (int attempt = 1; ; attempt++) {
        s_gw_link_up = false;
        s_gw_link = RNS::Link(gw_dest);
        s_gw_link.set_link_established_callback(on_gw_link_established);
        s_gw_link.set_link_closed_callback(on_gw_link_closed);
        if (pump_until(&s_gw_link_up, 15000)) {
            // Link is the real proof: adopt this gateway as the mesh selection.
            if (have != 0) gw_promote(s_gw_cands[have]);
            if (!s_gw_known) gw_persist(s_gw_cands[0]);
            s_gw_known = true;
            Serial.printf("[radio] mesh selected: gateway %.8s on %s\n",
                          s_gw_cands[0], sv_radio_profile_name());
            return true;
        }
        Serial.printf("[radio] gateway link FAILED to establish (attempt %d, %s) -> teardown\n",
                      attempt, sv_radio_profile_name());
        s_gw_link.teardown();
        s_gw_link = RNS::Link({RNS::Type::NONE});
        if (attempt >= link_attempts) return false;
        pump_until(nullptr, 2000);   // let the air clear before the retry
    }
}

// Gate A findings as policy (rate-limited path requests, teardown + retry), PLUS a
// client-side PROFILE SCAN (2026-07-10): a mesh with a live leaf can hold the gateway
// on a granted profile indefinitely — the gateway only scans back to base while its
// leaf is quiet — so a surveyor arriving on base would NEVER be heard (observed on the
// home bench: leaf + gateway parked on sf7/bw250, probe #11 failed all attempts on
// sf8). On total failure at the current profile, retry the whole sequence on each
// other rung of the shared ladder; a formed link means we stay there (the gateway
// serves multiple identities on one profile; ADR keepalives take over). Total failure
// converges back to base, mirroring the rendezvous rule.
static bool establish_gateway_link() {
    if (s_gw_link && s_gw_link_up) return true;
    if (!s_gw_n) { Serial.println("[radio] no gateway candidates at all"); return false; }
    RNS::Bytes hashes[SV_GW_MAX];
    for (int i = 0; i < s_gw_n; i++) hashes[i].assignHex(s_gw_cands[i]);

    uint8_t order[LORA_PROFILE_COUNT];
    int n = 0;
    order[n++] = sv_radio_profile();
    for (uint8_t p = 0; p < LORA_PROFILE_COUNT; p++)
        if (p != order[0]) order[n++] = p;

    for (int oi = 0; oi < n; oi++) {
        if (order[oi] != sv_radio_profile()) {
            Serial.printf("[radio] profile scan -> %s\n", LORA_PROFILES[order[oi]].name);
            if (!sv_radio_set_profile(order[oi])) continue;
        }
        // First rung gets the long windows (45 s path — must overlap a grant-parked
        // gateway's 30 s base-scan phases — and 4 link attempts); scan rungs go quicker.
        if (try_link_on_current_profile(hashes, oi == 0 ? 45000 : 15000,
                                        oi == 0 ? 4 : 2))
            return true;
    }
    Serial.println("[radio] no link on ANY profile -> revert to base");
    if (sv_radio_profile() != LORA_PROFILE_BASE)
        sv_radio_set_profile(LORA_PROFILE_BASE);
    return false;
}

void sv_link_teardown() {
    if (s_gw_link) s_gw_link.teardown();
    s_gw_link = RNS::Link({RNS::Type::NONE});
    s_gw_link_up = false;
}

bool sv_probe_send(const char* hdr_json, const uint8_t* body, size_t blen,
                   uint32_t timeout_ms) {
    if (!s_up || !s_radio) return false;
    if (!establish_gateway_link()) return false;
    RNS::Bytes payload(reinterpret_cast<const uint8_t*>(hdr_json), strlen(hdr_json));
    payload.append((const uint8_t*)"\n", 1);
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

void sv_radio_pump(uint32_t ms) {
    if (!s_up) return;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        reticulum.loop();
        delay(5);
    }
}

void sv_radio_poll_commands(uint32_t ms, sv_cmd_fn fn) {
    if (!s_up) return;
    s_cmd_count = 0;
    sv_radio_pump(ms);
    for (int i = 0; i < s_cmd_count; i++) {
        if (fn) fn(s_cmds[i]);
    }
    s_cmd_count = 0;
}

bool sv_radio_set_profile(uint8_t idx) {
    if (idx >= LORA_PROFILE_COUNT) return false;
    if (LoRaInterface::active) return LoRaInterface::active->set_profile(idx);
    return false;
}

uint8_t sv_radio_profile() {
    return LoRaInterface::active ? LoRaInterface::active->profile() : LORA_PROFILE_BASE;
}

const char* sv_radio_profile_name() {
    return LORA_PROFILES[sv_radio_profile()].name;
}

uint32_t sv_ms_since_rx() {
    if (!LoRaInterface::last_rx_ms) return UINT32_MAX;
    return millis() - LoRaInterface::last_rx_ms;
}

float sv_last_rssi() { return LoRaInterface::last_rssi; }
float sv_last_snr()  { return LoRaInterface::last_snr;  }
