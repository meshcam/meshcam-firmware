/*
 * Trail-cam gateway — Heltec V3. RNS/LoRa in, WiFi/HTTPS out.
 *
 * The receive side is gate-a-resource's server role, hardware-proven 2026-07-03
 * (5/15/25 KB Resources + the leaf's live thumbnail push), with the bugs it found
 * baked in as policy:
 *   - announce SPARINGLY and only while unlinked (half-duplex: a blind announce TX
 *     eats the initiator's one-shot RTT packet and half-opens the link)
 *   - resource strategy/callbacks installed in the destination's link_established
 *     callback (fires on RTT receipt — the only hook there is)
 *   - microReticulum's Transport jobs deadlock patched pre-build (see
 *     patch_microreticulum.py) or the first Resource retry wedges the board
 *
 * What's new vs gate-a:
 *   - PERSISTENT identity (NVS): the destination hash baked into leaves as
 *     LEAF_GATEWAY_DEST_HEX survives reflashes/reboots of this board, forever.
 *     Print it at boot; bake it once.
 *   - WiFi uplink: every COMPLETE Resource is POSTed (multipart, Bearer token) to
 *     the ingest API per docs/trailcam/ingest-api.md. Payload today is the leaf's bare
 *     thumbnail JPEG; metadata is synthesized here (event_id is content-addressed —
 *     "mesh-<crc32>-<len>" — so a re-received identical frame dedupes server-side).
 *     When the leaf later wraps payloads in an envelope (JSON header + JPEG), parse
 *     it here and pass the real event_id/captured_at through.
 *   - NTP for captured_at (thumb arrives ~90 s after the trigger; close enough
 *     until the envelope carries the leaf's own timestamp).
 *
 * Uploads run from loop(), never from RNS callbacks (a blocking HTTPS POST inside
 * reticulum.loop() would stall the radio for seconds).
 */

#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>
#include <LoRaInterface.h>
#include <microReticulum.h>

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>           // WiFiClientSecure/WiFiUDP ride lwIP — they route over ETH too
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#ifdef GW_HAS_OLED
#include <U8g2lib.h>        // SSD1306 OLED (parent-facing status screen)
#endif
#ifdef GW_NET_ETH
#include <ETH.h>            // LAN8720 wired ethernet (LILYGO T-Internet-POE)
#include <UDPInterface.h>   // vendored (lib/udp_interface): RNS-over-UDP on the wire
#endif
#include <ArduinoOTA.h>     // network firmware updates (espota) over the WG tunnel

#ifndef GW_SITE
#define GW_SITE "home"
#endif
#ifndef GW_FW_VERSION
#define GW_FW_VERSION "gateway-0.11.2"
#endif
#ifndef GW_VIEW_HOST
#define GW_VIEW_HOST "trailcam.example.com"   // family gallery shown on the OLED
#endif
#ifndef GW_OTA_PASS
#define GW_OTA_PASS "trailcam-ota"     // real value injected from secrets.ini
#endif
#if !defined(GW_INGEST_URL) || !defined(GW_INGEST_TOKEN)
#error "GW_INGEST_URL / GW_INGEST_TOKEN required — see secrets.ini.example"
#endif
#if !defined(GW_NET_ETH) && (!defined(GW_WIFI_SSID) || !defined(GW_WIFI_PSK))
#error "GW_WIFI_SSID / GW_WIFI_PSK required for the WiFi build — see secrets.ini.example"
#endif

// --- network abstraction: the uplink is WiFi (Heltec) or wired ethernet (T-Internet-POE).
// Everything below the transport (TLS ingest, OTA, NTP, web server) is identical — lwIP
// doesn't care which interface carries the socket.
#ifdef GW_NET_ETH
static volatile bool s_eth_got_ip = false;
static bool net_up()  { return s_eth_got_ip; }
static IPAddress net_ip()   { return ETH.localIP(); }
static int    net_rssi()    { return 0; }              // wired: no meaningful RSSI
static String net_desc()    { return String("ethernet ") + (s_eth_got_ip ? ETH.linkSpeed() : 0) + "M"; }
#else
static bool net_up()  { return WiFi.status() == WL_CONNECTED; }
static IPAddress net_ip()   { return WiFi.localIP(); }
static int    net_rssi()    { return (int)WiFi.RSSI(); }
static String net_desc()    { return WiFi.SSID(); }
#endif

static const char* APP_NAME = "trailcam_gatea";   // leaves build their OUT destination
static const char* ASPECT   = "resource";         // from these exact strings — don't drift
#define GW_ANNOUNCE_SECS 30

// --- RNS plumbing ----------------------------------------------------------------------
static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface lora_interface({RNS::Type::NONE});
static bool s_radio_ok = false;   // SX1262 present + init'd; false = ethernet/WiFi-only boot
#ifdef GW_NET_ETH
static RNS::Interface udp_interface({RNS::Type::NONE});   // RNS-over-UDP on the wire
#endif
static microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};
static RNS::Destination gw_destination({RNS::Type::NONE});
static RNS::Link        latest_link({RNS::Type::NONE});

// NVS-persistent identity, same pattern the leaf validated: mint once, reload forever.
static RNS::Identity load_or_create_identity() {
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    size_t n = p.getBytesLength("rns_prv");
    if (n > 0 && n <= 128) {
        uint8_t buf[128];
        p.getBytes("rns_prv", buf, n);
        p.end();
        RNS::Identity id(false);
        if (id.load_private_key(RNS::Bytes(buf, n))) {
            Serial.printf("[gw] identity loaded from NVS (%u key bytes)\n", (unsigned)n);
            return id;
        }
        Serial.println("[gw] NVS key load FAILED -> regenerating");
    } else {
        p.end();
    }
    RNS::Identity id(true);
    RNS::Bytes prv = id.get_private_key();
    Preferences pw;
    pw.begin("gateway", false);
    pw.putBytes("rns_prv", prv.data(), prv.size());
    pw.end();
    Serial.printf("[gw] identity CREATED + stored to NVS (%u key bytes) — PERMANENT from now on\n",
                  (unsigned)prv.size());
    return id;
}

// --- upload queue (filled by RNS callback, drained by loop()) --------------------------
// Holds the BARE JPEG plus parsed metadata; envelopes are peeled at queue time so a
// thumb transfers pointer-ownership with zero extra copies. Reassembled FULLS are not
// held in RAM at all (2026-07-04): the chunks spool to LittleFS (see Assembly) and the
// POST streams from the flash file — that removes the old ~48 KB heap ceiling that
// forced quality=max down to the standard tier.
#include <LittleFS.h>   // already mounted by filesystem.init() (microStore PosixFS)
#include <errno.h>
#define GW_SPOOL_PART "/spool.part"   // in-flight chunk assembly
#define GW_SPOOL_JPG  "/spool.jpg"    // completed full awaiting upload
struct PendingUpload {
    uint8_t*  jpeg = nullptr;         // RAM-backed (thumbs) ...
    bool      spooled = false;        // ... or file-backed at GW_SPOOL_JPG (fulls)
    size_t    len  = 0;
    char      event_id[48] = "";
    char      camera[48]   = "";
    char      kind[16]     = "";
    long long captured_at  = 0;
};
static PendingUpload s_pending;    // depth 1: pushes are far apart

static bool json_str(const char* json, const char* key, char* out, size_t cap);
static long long json_ll(const char* json, const char* key);

// --- live mesh event ring: served at http://<gateway-ip>/ -------------------------------
// Every noteworthy mesh event lands here (as well as on serial + telemetry beats), so
// "what is the mesh doing right now" is one browser tab away, no laptop attached.
#include <WebServer.h>
static WebServer s_web(80);
#define GW_EVT_SLOTS 30
static char     s_evt[GW_EVT_SLOTS][104];
static uint32_t s_evt_ms[GW_EVT_SLOTS];
static int      s_evt_head = 0;   // next write slot; ring is chronological from head
static struct {
    uint32_t announces = 0, cmds_delivered = 0, resources_ok = 0, resources_fail = 0,
             chunks = 0, uploads_ok = 0, probes = 0, announces_relayed = 0,
             announces_foreign = 0;
} s_stats;

// --- parent-facing OLED (Heltec V3 SSD1306 on I2C 17/18, reset 21, Vext 36) ------------
// The board's built-in screen, unused until now. Shows a rotating two-page status the
// the property owner can glance at: "is it working / where to see photos" and "last capture".
// Pin constants (Vext/RST_OLED/SCL_OLED/SDA_OLED) come from the board variant.
// The T-Internet-POE has no screen — its glanceable status is the web ring at :80.
#ifdef GW_HAS_OLED
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C
    u8g2(U8G2_R0, /*reset=*/RST_OLED, /*clock=*/SCL_OLED, /*data=*/SDA_OLED);
#endif

// Site display name (the address), fetched from the backend by this gateway's slug
// and cached in NVS so it shows instantly at boot, even before WiFi/first fetch. A
// rename in the trailcam settings UI trickles here within a fetch cycle.
static char s_site_name[40] = "";
static void load_cached_site_name() {
    Preferences p;
    p.begin("gateway", /*readOnly=*/true);
    p.getString("site_name", s_site_name, sizeof(s_site_name));
    p.end();
}

// Last capture + a per-(UTC-)day counter, fed by the upload-queue chokepoints below.
static uint32_t s_last_cap_ms   = 0;
static bool     s_have_cap      = false;
static char     s_last_cap_cam[40] = "";
static char     s_last_counted_eid[48] = "";
static uint16_t s_caps_today    = 0;
static long     s_cap_day       = -1;   // UTC day index the counter belongs to

static void roll_day_if_needed() {
    time_t now = time(nullptr);
    if (now < 1600000000) return;       // NTP not synced yet -> don't roll
    long day = now / 86400;
    if (s_cap_day < 0)      s_cap_day = day;
    else if (day != s_cap_day) { s_cap_day = day; s_caps_today = 0; }
}

// Called when an image (thumb or full) is ready to upload — camera is known here.
// Counts one per distinct event_id so a thumb + its later full don't double-count.
static void note_capture(const char* cam, const char* eid) {
    s_last_cap_ms = millis();
    s_have_cap    = true;
    snprintf(s_last_cap_cam, sizeof(s_last_cap_cam), "%s", cam ? cam : "");
    roll_day_if_needed();
    if (strcmp(eid ? eid : "", s_last_counted_eid) != 0) {
        snprintf(s_last_counted_eid, sizeof(s_last_counted_eid), "%s", eid ? eid : "");
        s_caps_today++;
    }
}

static void human_ago(uint32_t ms_ago, char* out, size_t n) {
    uint32_t s = ms_ago / 1000;
    if      (s < 45)     snprintf(out, n, "just now");
    else if (s < 5400)   snprintf(out, n, "%lu min ago",   (unsigned long)((s + 30) / 60));
    else if (s < 172800) snprintf(out, n, "%lu hours ago", (unsigned long)((s + 1800) / 3600));
    else                 snprintf(out, n, "%lu days ago",  (unsigned long)(s / 86400));
}
static void human_up(uint32_t s, char* out, size_t n) {
    if      (s < 3600)   snprintf(out, n, "up %lum", (unsigned long)(s / 60));
    else if (s < 172800) snprintf(out, n, "up %luh", (unsigned long)(s / 3600));
    else                 snprintf(out, n, "up %lud", (unsigned long)(s / 86400));
}

// Leaf identity roster (bug 7, multi-leaf form, gateway-0.10.0) — the policy story
// lives at the announce handler; the state sits up here because web_root() renders it.
// Each entry is one TOFU-pinned trailcam.leaf identity this gateway manages, with the
// per-identity state that used to be single globals: latest announced identity (for
// addressing), camera slug (from the announce's "n=" token, leaf-0.13.0+ — commands
// route by it), latest status/health, and per-leaf time-sync bookkeeping.
#define GW_ROSTER_MAX 4
struct LeafEntry {
    uint8_t  pin[16];                    // trailcam.leaf destination hash (TOFU-pinned)
    char     hex[33]         = "";       // pin as hex, for logs/web/beats
    char     slug[24]        = "";       // camera slug ("n=" in announces); empty until heard
    RNS::Identity identity   = RNS::Identity({RNS::Type::NONE});   // latest announcer
    volatile bool announced  = false;    // set in dispatch, consumed in loop()
    bool     announce_direct = false;    // hops==0 for the announce above
    char     status[24]      = "";       // "hello"/"checkin"/... of the latest announce
    char     health[96]      = "";       // health tail "p=.. c=.. f=.." (bug 5)
    uint32_t last_direct_ms  = 0;        // 0 = not heard direct this boot
    uint32_t last_sync_ms    = 0;        // per-leaf time_sync cadence (bug 10)
    bool     synced_once     = false;
    bool     sync_urgent     = false;    // profile adopt/change -> resync (bug 10)
};
static LeafEntry s_roster[GW_ROSTER_MAX];
static int       s_roster_n        = 0;
static bool      s_pin_save_pending = false;   // NVS write deferred out of packet dispatch

static void entry_hex_refresh(LeafEntry& L) {
    for (size_t i = 0; i < sizeof(L.pin); i++)
        sprintf(L.hex + i * 2, "%02x", L.pin[i]);
    // Machine-id default until an announce's "n=" tail refreshes it (node-identity.md in
    // the homelab tree): attribution never falls back to a baked human name.
    if (!L.slug[0]) snprintf(L.slug, sizeof(L.slug), "leaf-%.8s", L.hex);
}

// Attribution of last resort for PHOTO uploads whose envelope carries no camera (legacy
// bare-JPEG / pre-0.13 chunks — an ingest must name a camera, unlike telemetry beats,
// which post as gateway self-beats when unattributable): the single roster leaf's wire id
// when unambiguous, else "leaf-unknown". GW_DEFAULT_CAMERA (a baked human name) is retired.
static const char* gw_fallback_camera() {
    return (s_roster_n == 1) ? s_roster[0].slug : "leaf-unknown";
}
// NVS: the roster is one blob of concatenated 16-byte pins under "gw-leaf"/"roster".
// A gateway-0.9.0 single pin ("pin") migrates to entry 0 on first boot.
static bool load_roster() {
    Preferences p;
    if (!p.begin("gw-leaf", /*readOnly=*/true)) return false;   // namespace not created yet
    uint8_t buf[GW_ROSTER_MAX * 16];
    size_t n = p.getBytesLength("roster");
    if (n >= 16 && n % 16 == 0 && n <= sizeof(buf) && p.getBytes("roster", buf, n) == n) {
        p.end();
        s_roster_n = (int)(n / 16);
        for (int i = 0; i < s_roster_n; i++) {
            memcpy(s_roster[i].pin, buf + i * 16, 16);
            entry_hex_refresh(s_roster[i]);
        }
        return true;
    }
    // legacy single pin -> migrate (rewritten as "roster" on the next deferred save)
    if (p.getBytesLength("pin") == 16 && p.getBytes("pin", s_roster[0].pin, 16) == 16) {
        p.end();
        entry_hex_refresh(s_roster[0]);
        s_roster_n = 1;
        s_pin_save_pending = true;
        Serial.println("[gw] legacy single leaf pin migrated to roster entry 0");
        return true;
    }
    p.end();
    return false;
}
static void save_roster() {
    uint8_t buf[GW_ROSTER_MAX * 16];
    for (int i = 0; i < s_roster_n; i++) memcpy(buf + i * 16, s_roster[i].pin, 16);
    Preferences p;
    p.begin("gw-leaf", false);
    p.putBytes("roster", buf, (size_t)s_roster_n * 16);
    p.end();
}

static void log_event(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_evt[s_evt_head], sizeof(s_evt[0]), fmt, ap);
    va_end(ap);
    s_evt_ms[s_evt_head] = millis();
    s_evt_head = (s_evt_head + 1) % GW_EVT_SLOTS;
}

// --- ADR (adaptive radio profile, gateway-0.4.0) ----------------------------------------
// The gateway is the ADR decision-maker: it measures the SNR of the leaf's announces and
// grants profile switches by index into the shared table (lib/lora_interface/
// lora_profiles.h) via {"kind":"radio_profile"} packets into the leaf's RX window. The
// full state machine (grant -> confirm-or-revert, TTL refresh, scan fallback) lives in
// the rf_* functions below the command plumbing; this block is the state + helpers the
// status page and transfer beats need.
struct RfState {
    uint8_t  current = LORA_PROFILE_BASE;  // what OUR radio is tuned to right now
    uint8_t  granted = LORA_PROFILE_BASE;  // what we believe the leaf runs
    uint8_t  pending = 0xFF;               // grant sent, awaiting leaf confirm (0xFF none)
    uint8_t  prev    = LORA_PROFILE_BASE;  // revert target on confirm timeout
    bool     scanning = false;             // leaf lost off-base -> alternating camp
    uint32_t pending_ms = 0, last_leaf_ms = 0, last_switch_ms = 0,
             last_grant_tx_ms = 0, last_scan_ms = 0;
    uint8_t  announces_since_tx = 0;       // announces heard since our last grant TX
    float    snr[5];                       // rolling announce SNR at the CURRENT profile
    uint8_t  snr_n = 0, snr_i = 0;
};
static RfState s_rf;

// DIRECT frames only (bug 1, 2026-07-16): a transport relay (the surveyor; the dam
// spine's R1/R2 later) can deliver the leaf's announces across a radio-profile split,
// and counting them here kept the quiet-leaf scan disarmed for 21 h while every reply
// went out on a profile the leaf never listens on (07-15 outage root cause). App
// callbacks run synchronously inside inbound dispatch, so last_hops belongs to the
// frame that fired the callback.
static void rf_note_leaf_heard() {
    if (LoRaInterface::last_hops == 0) s_rf.last_leaf_ms = millis();
}
static void rf_clear_snr() { s_rf.snr_n = 0; s_rf.snr_i = 0; }
static void rf_push_snr(float snr) {
    s_rf.snr[s_rf.snr_i] = snr;
    s_rf.snr_i = (s_rf.snr_i + 1) % 5;
    if (s_rf.snr_n < 5) s_rf.snr_n++;
}
static float rf_avg_snr() {
    float sum = 0;
    for (uint8_t i = 0; i < s_rf.snr_n; i++) sum += s_rf.snr[i];
    return s_rf.snr_n ? sum / s_rf.snr_n : NAN;
}
static const char* rf_name(uint8_t idx) { return LORA_PROFILES[idx].name; }
static void rf_retune(uint8_t idx) {
    if (LoRaInterface::active) LoRaInterface::active->set_profile(idx);
    s_rf.current = idx;
}
static void rf_persist(uint8_t idx) {
    Preferences p;
    p.begin("gw-rf", false);
    p.putUChar("idx", idx);
    p.end();
}

// True while frames are actively arriving (a transfer or command exchange in
// progress). Blocking work — TLS beats, command polls, uploads — is deferred while
// this holds: a single WiFiClientSecure handshake is 1-3 s during which the loop
// isn't servicing the radio, and a windowed Resource sender fills that silence with
// parts we then miss and re-request (part of the measured ~60 s/chunk overhead).
static bool radio_busy() {
    return LoRaInterface::last_rx_ms && millis() - LoRaInterface::last_rx_ms < 2000;
}

static void web_root() {
    String h;
    h.reserve(4600);
    h += F("<!doctype html><meta charset=utf-8><meta http-equiv=refresh content=5>"
           "<title>trailcam gateway</title><style>body{font:14px monospace;background:#111;"
           "color:#cdc;margin:2em}h1{font-size:16px;color:#8f8}table{border-collapse:collapse}"
           "td{padding:2px 10px;border-bottom:1px solid #333}.t{color:#888}</style>"
           "<h1>trailcam gateway — live mesh</h1><p>");
    char line[380];
    char clock[32] = "clock NOT SYNCED";
    {
        time_t now = time(nullptr);
        if (now > 1600000000) {
            struct tm tmv;
            gmtime_r(&now, &tmv);
            strftime(clock, sizeof(clock), "clock %H:%M:%SZ", &tmv);
        }
    }
    snprintf(line, sizeof(line),
             "uptime %lus | heap %u | fs free %uK | %s | radio %s%s | leaf rssi %.0f dBm snr %.1f<br>"
             "announces %lu (rf %lu, relayed %lu, foreign %lu) | cmds %lu | res ok/fail %lu/%lu | chunks %lu | uploads %lu<br>"
             "leaf roster %d/%d%s (POST /pin/clear to re-arm TOFU)",
             (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(),
             (unsigned)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024), clock,
             s_radio_ok ? LORA_PROFILES[s_rf.current].name : "ABSENT — wire the SX1262",
             s_rf.pending != 0xFF ? " (confirming)" : (s_rf.scanning ? " (scanning)" : ""),
             LoRaInterface::last_rssi, LoRaInterface::last_snr,
             (unsigned long)s_stats.announces, (unsigned long)LoRaInterface::announces_seen,
             (unsigned long)s_stats.announces_relayed, (unsigned long)s_stats.announces_foreign,
             (unsigned long)s_stats.cmds_delivered,
             (unsigned long)s_stats.resources_ok, (unsigned long)s_stats.resources_fail,
             (unsigned long)s_stats.chunks, (unsigned long)s_stats.uploads_ok,
             s_roster_n, GW_ROSTER_MAX,
             s_roster_n ? "" : " — TOFU armed, first direct health announce pins");
    h += line;
    for (int i = 0; i < s_roster_n; i++) {
        const LeafEntry& L = s_roster[i];
        char ago[24] = "never (this boot)";
        if (L.last_direct_ms) human_ago(millis() - L.last_direct_ms, ago, sizeof(ago));
        snprintf(line, sizeof(line), "<br>leaf %d: %.16s… %s — %s [%s], direct %s",
                 i, L.hex, L.slug[0] ? L.slug : "(no slug yet)",
                 L.status[0] ? L.status : "-", L.health[0] ? L.health : "-", ago);
        h += line;
    }
    h += F("</p><table>");
    for (int i = GW_EVT_SLOTS - 1; i >= 0; i--) {
        int idx = (s_evt_head + i) % GW_EVT_SLOTS;
        if (!s_evt[idx][0]) continue;
        uint32_t age = (millis() - s_evt_ms[idx]) / 1000;
        snprintf(line, sizeof(line), "<tr><td class=t>-%lus</td><td>%s</td></tr>",
                 (unsigned long)age, s_evt[idx]);
        h += line;
    }
    h += F("</table>");
    s_web.send(200, "text/html", h);
}

// --- telemetry beats: feed the Nodes/diagnostics page over WiFi -------------------------
// Queued from RNS callbacks (no blocking HTTP inside reticulum.loop), drained in loop().
// Fire-and-forget per the contract: a dropped beat is fine, a stalled radio isn't.
#define GW_BEAT_SLOTS 6
static char s_beats[GW_BEAT_SLOTS][960];   // sized for a hex-dumped announce frame
static int  s_beat_n = 0;

static void queue_beat(const char* fmt, ...) {
    if (s_beat_n >= GW_BEAT_SLOTS) return;   // lossy by design
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_beats[s_beat_n], sizeof(s_beats[0]), fmt, ap);
    va_end(ap);
    s_beat_n++;
}

static void post_beats() {
    if (!s_beat_n || !net_up()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(8000);
    String url = String(GW_INGEST_URL);
    url.replace("/ingest", "/telemetry");
    for (int i = 0; i < s_beat_n; i++) {
        if (!http.begin(tls, url)) break;
        http.addHeader("Authorization", "Bearer " GW_INGEST_TOKEN);
        http.addHeader("Content-Type", "application/json");
        int code = http.POST((uint8_t*)s_beats[i], strlen(s_beats[i]));
        if (code < 200 || code >= 300)
            Serial.printf("[gw] telemetry beat -> HTTP %d (dropped)\n", code);
        http.end();
    }
    s_beat_n = 0;
}


static uint32_t crc32_zlib(const uint8_t* p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (-(int32_t)(c & 1)));
    }
    return c ^ 0xFFFFFFFFu;
}

// One resource in flight at a time (half-duplex link), so a single start-stamp is
// enough to time each transfer for the bandwidth diagnostics in the beats.
static uint32_t s_res_start_ms = 0;

static void on_resource_started(const RNS::Resource& resource) {
    s_res_start_ms = millis();
    rf_note_leaf_heard();
    Serial.printf("[gw] incoming resource, transfer_size=%u\n",
                  (unsigned)resource.get_transfer_size());
}

// Chunked full-res reassembly. The leaf splits a full under the 25 KB single-Resource
// ceiling; each chunk's envelope carries chunk/chunks/offset/total. Chunks are written
// straight to a LittleFS spool file at their offset (2026-07-04) — the heap never
// holds the whole full, so quality=max originals (~100-200 KB QXGA) fit where the old
// in-RAM assembly OOM'd past ~48 KB. On completion the spool is renamed for the
// uploader, which streams the POST body from flash.
struct Assembly {
    char      event_id[48] = "";
    char      camera[48]   = "";
    char      quality[16]  = "";
    long long captured_at  = 0;
    size_t    total = 0;
    unsigned  chunks = 0;
    uint32_t  got_mask = 0;
    uint32_t  last_ms  = 0;
    uint32_t  first_ms = 0;   // start of the whole transfer (first chunk's arrival)
};
static Assembly s_asm;

static bool asm_active() { return s_asm.event_id[0] != '\0'; }

static void asm_reset() {
    if (LittleFS.exists(GW_SPOOL_PART)) LittleFS.remove(GW_SPOOL_PART);
    s_asm = Assembly();
}

// One chunk -> the spool file, in a SHORT-LIVED handle: open r+, seek, write, close,
// then stat-verify the size actually grew. Holding one FILE* open across the minutes
// between chunks corrupted spools in every shape (writes failing at random offsets,
// "successful" writes losing their tail, a completed file 2 KB short) — consistent
// with the stream's fd getting clobbered by the heavy store churn elsewhere in the
// stack (microStore's own "Failed to unlink: Has open FD" errors show its fd hygiene).
// A fresh open/close per chunk shrinks the exposure window from minutes to
// milliseconds and makes every write durable (close = full flush) and verifiable.
static bool spool_write_chunk(size_t offset, const uint8_t* body, size_t blen) {
    errno = 0;
    File f = LittleFS.open(GW_SPOOL_PART, "r+");
    if (!f) { Serial.printf("[gw] spool r+ open FAILED (errno=%d)\n", errno); return false; }
    const bool seek_ok = f.seek(offset);
    const size_t wrote = seek_ok ? f.write(body, blen) : 0;
    f.close();
    File v = LittleFS.open(GW_SPOOL_PART, "r");
    const size_t vsize = v ? v.size() : 0;
    if (v) v.close();
    if (!seek_ok || wrote != blen || vsize < offset + blen) {
        Serial.printf("[gw] spool write FAILED at %u (seek=%d wrote=%u/%u size=%u "
                      "errno=%d, fs free %u)\n",
                      (unsigned)offset, (int)seek_ok, (unsigned)wrote, (unsigned)blen,
                      (unsigned)vsize, errno,
                      (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()));
        return false;
    }
    return true;
}

static void pending_clear() {
    if (s_pending.jpeg) free(s_pending.jpeg);
    if (s_pending.spooled && LittleFS.exists(GW_SPOOL_JPG)) LittleFS.remove(GW_SPOOL_JPG);
    s_pending = PendingUpload();
}

// Take ownership of a bare-JPEG buffer + metadata (zero-copy for thumbs).
static void queue_upload_owned(uint8_t* jpeg, size_t len, const char* eid,
                               const char* cam, const char* kind, long long capts) {
    if (s_pending.jpeg || s_pending.spooled) {
        Serial.println("[gw] upload queue full -> DROPPING the older pending frame");
        pending_clear();
    }
    s_pending.jpeg = jpeg;
    s_pending.len  = len;
    snprintf(s_pending.event_id, sizeof(s_pending.event_id), "%s", eid);
    snprintf(s_pending.camera,   sizeof(s_pending.camera),   "%s", cam);
    snprintf(s_pending.kind,     sizeof(s_pending.kind),     "%s", kind);
    s_pending.captured_at = capts;
    note_capture(cam, eid);
}

// File-backed variant: the body already sits at GW_SPOOL_JPG (a completed assembly);
// the uploader streams it from flash.
static void queue_upload_spooled(size_t len, const char* eid,
                                 const char* cam, long long capts) {
    if (s_pending.jpeg || s_pending.spooled) {
        Serial.println("[gw] upload queue full -> DROPPING the older pending frame");
        pending_clear();
    }
    s_pending.spooled = true;
    s_pending.len     = len;
    snprintf(s_pending.event_id, sizeof(s_pending.event_id), "%s", eid);
    snprintf(s_pending.camera,   sizeof(s_pending.camera),   "%s", cam);
    snprintf(s_pending.kind,     sizeof(s_pending.kind),     "full");
    s_pending.captured_at = capts;
    note_capture(cam, eid);
}

// Single-Resource path: peel the envelope (or synthesize legacy metadata) and copy
// just the JPEG part (thumbs are ~4 KB).
static void queue_upload(const uint8_t* data, size_t len) {
    const uint8_t* jpeg = data;
    size_t jlen = len;
    char eid[48] = "", cam[48] = "", kind[16] = "";
    long long capts = 0;
    if (len > 2 && data[0] == '{') {
        const uint8_t* nl = (const uint8_t*)memchr(data, '\n', min(len, (size_t)256));
        if (nl) {
            char hdr[257];
            size_t hl = nl - data;
            memcpy(hdr, data, hl);
            hdr[hl] = '\0';
            json_str(hdr, "event_id", eid, sizeof(eid));
            json_str(hdr, "camera", cam, sizeof(cam));
            json_str(hdr, "kind", kind, sizeof(kind));
            capts = json_ll(hdr, "captured_at");
            jpeg = nl + 1;
            jlen = len - hl - 1;
        }
    }
    if (!eid[0]) {   // legacy bare JPEG: content-addressed id
        const uint32_t crc = crc32_zlib(jpeg, jlen);
        snprintf(eid, sizeof(eid), "mesh-%08x-%u", (unsigned)crc, (unsigned)jlen);
    }
    if (!cam[0])  snprintf(cam, sizeof(cam), "%s", gw_fallback_camera());
    if (!kind[0]) snprintf(kind, sizeof(kind), "thumb");
    uint8_t* copy = (uint8_t*)malloc(jlen);
    if (!copy) { Serial.println("[gw] OOM copying payload -> dropped"); return; }
    memcpy(copy, jpeg, jlen);
    queue_upload_owned(copy, jlen, eid, cam, kind, capts);
}

static void handle_chunk(const char* hdr, const uint8_t* body, size_t blen,
                         uint32_t res_ms) {
    char eid[48] = "", cam[48] = "", qual[16] = "";
    json_str(hdr, "event_id", eid, sizeof(eid));
    json_str(hdr, "camera", cam, sizeof(cam));
    json_str(hdr, "quality", qual, sizeof(qual));
    const unsigned  chunk  = (unsigned)json_ll(hdr, "chunk");
    const unsigned  chunks = (unsigned)json_ll(hdr, "chunks");
    const size_t    offset = (size_t)json_ll(hdr, "offset");
    const size_t    total  = (size_t)json_ll(hdr, "total");
    if (!eid[0] || !chunks || chunks > 32 || !total || total > 512 * 1024 ||
        offset + blen > total) {
        Serial.printf("[gw] chunk with bad geometry (%u/%u off=%u total=%u) -> dropped\n",
                      chunk, chunks, (unsigned)offset, (unsigned)total);
        return;
    }
    if (strcmp(s_asm.event_id, eid) != 0 || s_asm.total != total) {
        asm_reset();   // different event (or first chunk): fresh assembly
        // Spool-space guard (replaces the old heap-OOM failure mode): the partition
        // holds RNS stores too, so keep 64 KB of slack beyond the file itself.
        if (LittleFS.totalBytes() - LittleFS.usedBytes() < total + 64 * 1024) {
            Serial.printf("[gw] spool: only %u B free for a %u B full -> dropped\n",
                          (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()),
                          (unsigned)total);
            return;
        }
        strcpy(s_asm.event_id, eid);
        strcpy(s_asm.camera, cam);
        strcpy(s_asm.quality, qual);
        s_asm.captured_at = json_ll(hdr, "captured_at");
        s_asm.total  = total;
        s_asm.chunks = chunks;
        s_asm.first_ms = millis() - res_ms;   // include this first chunk's own airtime
        // Create the (empty) spool; every chunk write reopens it briefly in r+.
        File c = LittleFS.open(GW_SPOOL_PART, FILE_WRITE);
        if (!c) { Serial.println("[gw] spool create FAILED -> dropped"); asm_reset(); return; }
        c.close();
    }
    if (!spool_write_chunk(offset, body, blen)) {
        Serial.println("[gw] -> assembly dropped");
        asm_reset();
        return;
    }
    s_asm.got_mask |= (1u << chunk);
    s_asm.last_ms = millis();
    rf_note_leaf_heard();
    Serial.printf("[gw] full chunk %u/%u for %s (%u bytes at %u, %lu ms)\n",
                  chunk + 1, chunks, eid, (unsigned)blen, (unsigned)offset,
                  (unsigned long)res_ms);
    s_stats.chunks++;
    log_event("full chunk %u/%u for %s", chunk + 1, chunks, eid);
    // Bandwidth diagnostics ride in the beat: airtime, goodput vs the radio profile,
    // where this chunk sits in the file, and the signal it arrived at.
    {
        char tj[300];
        snprintf(tj, sizeof(tj),
                 "{\"event_id\":\"%s\",\"chunk\":%u,\"chunks\":%u,\"bytes\":%u,"
                 "\"ms\":%lu,\"bps\":%lu,\"offset\":%u,\"total\":%u,"
                 "\"quality\":\"%s\",\"profile\":\"%s\",\"raw_bps\":%u}",
                 eid, chunk + 1, chunks, (unsigned)blen,
                 (unsigned long)res_ms,
                 (unsigned long)(res_ms ? (uint64_t)blen * 1000 / res_ms : 0),
                 (unsigned)offset, (unsigned)total, qual[0] ? qual : "max",
                 rf_name(s_rf.current), (unsigned)LORA_PROFILES[s_rf.current].raw_bps);
        if (isnan(LoRaInterface::last_rssi))
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\","
                       "\"extra\":{\"transfer\":%s}}",
                       GW_SITE, cam[0] ? cam : gw_fallback_camera(), tj);
        else
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\","
                       "\"rssi\":%.0f,\"snr\":%.1f,\"extra\":{\"transfer\":%s}}",
                       GW_SITE, cam[0] ? cam : gw_fallback_camera(),
                       LoRaInterface::last_rssi, LoRaInterface::last_snr, tj);
    }

    if (__builtin_popcount(s_asm.got_mask) == (int)s_asm.chunks) {
        const uint32_t span_ms = millis() - s_asm.first_ms;
        Serial.printf("[gw] full REASSEMBLED: %s, %u bytes in %lu ms\n",
                      eid, (unsigned)s_asm.total, (unsigned long)span_ms);
        log_event("REASSEMBLED %s (%u bytes)", eid, (unsigned)s_asm.total);
        queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\","
                   "\"extra\":{\"transfer\":{\"event_id\":\"%s\",\"reassembled\":%u,"
                   "\"chunks\":%u,\"ms\":%lu,\"bps\":%lu,\"quality\":\"%s\","
                   "\"profile\":\"%s\",\"raw_bps\":%u}}}",
                   GW_SITE, s_asm.camera[0] ? s_asm.camera : gw_fallback_camera(),
                   eid, (unsigned)s_asm.total,
                   s_asm.chunks, (unsigned long)span_ms,
                   (unsigned long)(span_ms ? (uint64_t)s_asm.total * 1000 / span_ms : 0),
                   s_asm.quality[0] ? s_asm.quality : "max",
                   rf_name(s_rf.current), (unsigned)LORA_PROFILES[s_rf.current].raw_bps);
        // Promote the spool to the upload slot: verify the final size (belt — every
        // chunk was verified at write time), clear any older completed spool, then
        // rename PART -> JPG. asm_reset() must not run before the rename or it would
        // delete the file we're promoting.
        {
            File v = LittleFS.open(GW_SPOOL_PART, "r");
            const size_t vsize = v ? v.size() : 0;
            if (v) v.close();
            if (vsize != s_asm.total) {
                Serial.printf("[gw] spool final size %u != %u -> dropped\n",
                              (unsigned)vsize, (unsigned)s_asm.total);
                asm_reset();
                return;
            }
        }
        if (s_pending.spooled) pending_clear();   // frees GW_SPOOL_JPG for the rename
        if (LittleFS.exists(GW_SPOOL_JPG)) LittleFS.remove(GW_SPOOL_JPG);
        if (!LittleFS.rename(GW_SPOOL_PART, GW_SPOOL_JPG)) {
            Serial.println("[gw] spool rename FAILED -> full dropped");
            asm_reset();
            return;
        }
        queue_upload_spooled(s_asm.total, s_asm.event_id,
                             s_asm.camera[0] ? s_asm.camera : gw_fallback_camera(),
                             s_asm.captured_at);
        asm_reset();
    }
}

// --- surveyor probes: proof-of-connection records, never photos --------------------------
// The surveyor (docs/trailcam/surveyor.md in the homelab tree) pushes dummy Resources with
// kind=probe and its GPS fix in the envelope. Log the whole header verbatim into a
// telemetry beat (it's already JSON) with OUR side's RF numbers + airtime, and queue a
// one-shot probe_ack downlink so the walker's CSV gets the gateway-side numbers too.
// Never uploaded as a photo, never counted as a capture.
static char s_probe_ack[192] = "";   // delivered into the surveyor's post-announce RX window

static void handle_probe(const char* hdr, size_t total_len, uint32_t res_ms) {
    char slug[40] = "";
    json_str(hdr, "camera", slug, sizeof(slug));
    const long long seq  = json_ll(hdr, "seq");
    const long long part = json_ll(hdr, "part");
    const float rssi = LoRaInterface::last_rssi;
    const float snr  = LoRaInterface::last_snr;
    // No rf_note_leaf_heard() here (gateway-0.10.0): probes come from the surveyor —
    // FOREIGN by definition under the roster — and counting them as leaf liveness
    // kept the quiet-leaf scan disarmed while the real leaf sat unreachable (the
    // bug-7 pathology in miniature). Mid-transfer retune protection is radio_busy().
    Serial.printf("[gw] PROBE #%lld part %lld from %s: %u bytes in %lu ms "
                  "(rssi %.0f snr %.1f, %s)\n",
                  seq, part, slug[0] ? slug : "?", (unsigned)total_len,
                  (unsigned long)res_ms, rssi, snr, rf_name(s_rf.current));
    s_stats.probes++;
    log_event("probe #%lld from %s (%.0f dBm)", seq, slug[0] ? slug : "surveyor", rssi);
    // kind=surveyor so the app doesn't auto-register the walker as a camera
    // (needs app >= the schemas.py Literal that accepts it, else the beat 422s).
    if (isnan(rssi))
        queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"surveyor\","
                   "\"extra\":{\"probe\":%s,\"ms\":%lu,\"bytes\":%u,\"profile\":\"%s\"}}",
                   GW_SITE, slug[0] ? slug : "surveyor", hdr,
                   (unsigned long)res_ms, (unsigned)total_len, rf_name(s_rf.current));
    else
        queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"surveyor\","
                   "\"rssi\":%.0f,\"snr\":%.1f,"
                   "\"extra\":{\"probe\":%s,\"ms\":%lu,\"bytes\":%u,\"profile\":\"%s\"}}",
                   GW_SITE, slug[0] ? slug : "surveyor", rssi, snr, hdr,
                   (unsigned long)res_ms, (unsigned)total_len, rf_name(s_rf.current));
    if (isnan(rssi))
        snprintf(s_probe_ack, sizeof(s_probe_ack),
                 "{\"kind\":\"probe_ack\",\"payload\":{\"seq\":%lld,\"part\":%lld,"
                 "\"ms\":%lu}}", seq, part, (unsigned long)res_ms);
    else
        snprintf(s_probe_ack, sizeof(s_probe_ack),
                 "{\"kind\":\"probe_ack\",\"payload\":{\"seq\":%lld,\"part\":%lld,"
                 "\"rssi\":%.0f,\"snr\":%.1f,\"ms\":%lu}}",
                 seq, part, rssi, snr, (unsigned long)res_ms);
}

static void on_resource_concluded(const RNS::Resource& resource) {
    if (resource.status() != RNS::Type::Resource::COMPLETE) {
        Serial.printf("[gw] resource FAILED status=%d\n", (int)resource.status());
        s_stats.resources_fail++;
        log_event("resource FAILED (status %d)", (int)resource.status());
        // Gateway self-beat: a failed Resource has no envelope, so there is no leaf to
        // honestly attribute it to — it is gateway-observed state (node-identity.md).
        queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
                   "\"extra\":{\"transfer_error\":\"resource_failed_%d\"}}",
                   GW_SITE, GW_SITE, (int)resource.status());
        return;
    }
    const RNS::Bytes& d = resource.data();
    Serial.printf("[gw] RESOURCE COMPLETE: %u bytes hash=%s\n",
                  (unsigned)d.size(), resource.get_hash().toHex().c_str());
    s_stats.resources_ok++;

    // Enveloped? Peek the header once: probes are logged (never uploaded), multi-chunk
    // fulls go to reassembly, everything else uploads as-is.
    if (d.size() > 2 && d.data()[0] == '{') {
        const uint8_t* nl = (const uint8_t*)memchr(d.data(), '\n',
                                                   min(d.size(), (size_t)256));
        if (nl) {
            char hdr[257];
            size_t hl = nl - d.data();
            memcpy(hdr, d.data(), hl);
            hdr[hl] = '\0';
            char kind[16] = "";
            json_str(hdr, "kind", kind, sizeof(kind));
            if (strcmp(kind, "probe") == 0) {
                handle_probe(hdr, d.size(), millis() - s_res_start_ms);
                return;
            }
            if (json_ll(hdr, "chunks") > 1) {
                handle_chunk(hdr, nl + 1, d.size() - hl - 1,
                             millis() - s_res_start_ms);
                return;
            }
        }
    }
    queue_upload(d.data(), d.size());
}

static void on_link_established(RNS::Link& link) {
    Serial.println("[gw] leaf linked");
    rf_note_leaf_heard();
    log_event("leaf linked");
    // Gateway self-beat: the link handshake is anonymous (identity arrives with the
    // Resource/announce that follows), so "a leaf linked" is gateway state, not any
    // particular camera's — this is what used to mint the leaf-unknown pseudo-node.
    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"extra\":{\"link\":\"up\",\"via\":\"mesh\"}}",
               GW_SITE, GW_SITE);
    latest_link = link;
    link.set_resource_strategy(RNS::Type::Link::ACCEPT_ALL);
    link.set_resource_started_callback(on_resource_started);
    link.set_resource_concluded_callback(on_resource_concluded);
}

// --- command downlink: poll the API, deliver to the leaf the moment it announces -------
// The leaf sleeps ~always; its awake windows follow its own announces (it opens a mesh
// RX window right after announcing). So: keep pending commands fresh via WiFi polling,
// and an AnnounceHandler on the leaf's aspect fires them as single RNS packets exactly
// when the leaf can hear them. Suppression prevents hammering a leaf mid-transfer.
#define GW_CMD_POLL_SECS      20
#define GW_CMD_SUPPRESS_SECS  120
#define GW_CMD_SLOTS          6

struct PendingCmd {
    char     json[512] = "";
    uint32_t last_sent = 0;      // millis of last delivery attempt (0 = never)
    bool     used      = false;
};
static PendingCmd s_cmds[GW_CMD_SLOTS];

// --- leaf identity roster (bug 7; single pin 2026-07-16, roster gateway-0.10.0) ---------
// "s_leaf_identity = last announcer" let ANY trailcam.leaf identity win grants, command
// delivery, time sync and liveness. On the 07-16 surveyor re-test the surveyor won the
// ADR grant lottery and held the real leaf off the mesh for ~100 min — keepalive grants
// self-sustain the impostor lock, and hop counting can't help because the impostor's
// announces are direct. So pin the identities this gateway manages (the roster,
// GW_ROSTER_MAX slots — one entry per camera at multi-cam sites):
//   - TOFU per identity: a DIRECT announce carrying a health tail ("p=.. c=.. f=..")
//     enrolls its trailcam.leaf destination hash into a free slot (NVS). Only real
//     leaves (0.12.0+) send the tail; the surveyor never does, so it can't win
//     commissioning either. Relayed announces never enroll — a relay can forward
//     another site's leaf across a profile split (the 07-15 pathology), and a gateway
//     that has never heard a leaf direct has no business managing it. Enrollment is
//     open for the roster's lifetime (that's how camera N+1 self-commissions: mount
//     it, power it, done) — the loud log/beat below is the audit trail. Caveat for a
//     future two-sites-in-earshot layout: announces don't carry a site id, so a
//     NEIGHBOR site's real leaf heard direct would enroll here too; today's sites are
//     miles apart.
//   - Commands route per entry by camera slug: leaf-0.13.0 announces carry "n=<slug>",
//     and a command's "node" field must match the entry's slug (a sole slug-less
//     entry keeps the legacy deliver-everything behavior).
//   - Any non-roster trailcam.leaf identity is FOREIGN: counted, logged, beaten to
//     telemetry under the GATEWAY node (so it can't pollute the camera's rows), still
//     owed a probe_ack (the surveyor's actual need) and a one-shot time_sync on
//     "hello" — but it can't touch roster identities, ADR, liveness, commands or
//     command acks.
//   - Site hardware swap: POST /pin/clear on the web UI empties the roster.
static RNS::Identity s_foreign_identity({RNS::Type::NONE});   // last foreign announcer
static volatile bool s_foreign_announced = false;
static bool     s_foreign_hello = false;        // foreign cold boot -> one-shot time_sync

static void clear_roster() {
    Preferences p;
    p.begin("gw-leaf", false);
    p.remove("roster");
    p.remove("pin");    // the pre-roster key, in case migration never re-saved
    p.end();
    for (int i = 0; i < s_roster_n; i++)
        s_roster[i] = LeafEntry();      // stop addressing the old leaves
    s_roster_n = 0;
    Serial.println("[gw] leaf roster CLEARED — TOFU re-armed");
    log_event("leaf roster cleared (TOFU re-armed)");
}

// Command-receipt acks (bug 8): the leaf announces "a=<id>,<id>" for server command
// ids it received; we relay each to POST /commands/<id>/ack {"status":"received"} so
// the server stops redelivering. Queued here (announce handler runs inside packet
// dispatch — no blocking HTTP there), drained in loop() when the radio is quiet.
#define GW_ACK_SLOTS 8
static uint32_t s_cmd_acks[GW_ACK_SLOTS];
static int      s_cmd_ack_n = 0;

static void queue_cmd_acks(const char* health_tail) {
    const char* a = strstr(health_tail, "a=");
    if (!a) return;
    a += 2;
    while (*a && s_cmd_ack_n < GW_ACK_SLOTS) {
        if (*a < '0' || *a > '9') break;
        s_cmd_acks[s_cmd_ack_n++] = (uint32_t)strtoul(a, (char**)&a, 10);
        if (*a == ',') a++;
    }
}

static void post_cmd_acks() {
    if (!s_cmd_ack_n || !net_up()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(8000);
    for (int i = 0; i < s_cmd_ack_n; i++) {
        String url = String(GW_INGEST_URL);
        char path[40];
        snprintf(path, sizeof(path), "/commands/%lu/ack", (unsigned long)s_cmd_acks[i]);
        url.replace("/ingest", path);
        if (!http.begin(tls, url)) break;
        http.addHeader("Authorization", "Bearer " GW_INGEST_TOKEN);
        http.addHeader("Content-Type", "application/json");
        static const char* body = "{\"status\":\"received\",\"detail\":\"leaf ack via announce\"}";
        int code = http.POST((uint8_t*)body, strlen(body));
        Serial.printf("[gw] cmd %lu ack received -> HTTP %d\n",
                      (unsigned long)s_cmd_acks[i], code);
        log_event("cmd %lu leaf-acked (HTTP %d)", (unsigned long)s_cmd_acks[i], code);
        http.end();
    }
    s_cmd_ack_n = 0;   // fire-and-forget: redelivery re-acks if one was lost
}

// Pull the "n=<slug>" token out of an announce tail (leaf-0.13.0). Token-boundary
// aware: matches only at the tail start or right after a space, so a slug can never
// be faked by a substring of another token. Returns false if absent (older leaves).
static bool htail_slug(const char* htail, char* out, size_t cap) {
    for (const char* p = htail; p; p = strchr(p, ' '), p = p ? p + 1 : nullptr) {
        if (strncmp(p, "n=", 2) != 0) continue;
        p += 2;
        size_t i = 0;
        while (p[i] && p[i] != ' ' && i + 1 < cap) { out[i] = p[i]; i++; }
        out[i] = '\0';
        return i > 0;
    }
    return false;
}

class LeafAnnounceHandler : public RNS::AnnounceHandler {
public:
    LeafAnnounceHandler() : AnnounceHandler("trailcam.leaf") {}
    void received_announce(const RNS::Bytes& destination_hash,
                           const RNS::Identity& announced_identity,
                           const RNS::Bytes& app_data) override {
        // This handler runs synchronously inside inbound dispatch of the announce
        // packet, so the LoRaInterface last_* statics describe THIS announce.
        const bool direct = (LoRaInterface::last_hops == 0);
        // app_data = "<status>[ <tail>]" — leaf-0.13.0 tails carry the node slug
        // ("n=<slug>") plus the health counters ("p=<pir> c=<captures> f=<push_fails>
        // [vb=<V>]", bug 5); older leaves and the surveyor send the bare status.
        // Split on the first space, into LOCAL buffers — a foreign announce must not
        // overwrite any roster entry's state.
        char status[24], htail[96];
        {
            std::string ad((const char*)app_data.data(),
                           std::min(app_data.size(), (size_t)120));
            size_t sp = ad.find(' ');
            snprintf(status, sizeof(status), "%s",
                     ((sp == std::string::npos) ? ad : ad.substr(0, sp)).c_str());
            snprintf(htail, sizeof(htail), "%s",
                     (sp == std::string::npos) ? "" : ad.substr(sp + 1).c_str());
        }
        s_stats.announces++;               // "processed by Transport" — feeds the bug-6
        if (!direct) s_stats.announces_relayed++;   // wedge watchdog, so count ALL
        // --- roster membership (bug 7): is this announce from a leaf WE manage? ----
        const bool has_health = strstr(htail, "p=") && strstr(htail, "c=");
        int idx = -1;
        if (destination_hash.size() == sizeof(s_roster[0].pin)) {
            for (int i = 0; i < s_roster_n; i++)
                if (memcmp(destination_hash.data(), s_roster[i].pin,
                           sizeof(s_roster[i].pin)) == 0) { idx = i; break; }
            // TOFU enrollment: direct + health tail = a real leaf physically at this
            // site -> claim a free slot. Relayed or tail-less never enrolls (see the
            // roster policy block above). A full roster demotes newcomers to FOREIGN.
            if (idx < 0 && direct && has_health && s_roster_n < GW_ROSTER_MAX) {
                idx = s_roster_n;
                memcpy(s_roster[idx].pin, destination_hash.data(),
                       sizeof(s_roster[idx].pin));
                entry_hex_refresh(s_roster[idx]);
                s_roster_n++;
                s_pin_save_pending = true;   // NVS write happens in loop(), not dispatch
                Serial.printf("[gw] leaf identity PINNED (TOFU, roster %d/%d): %s\n",
                              s_roster_n, GW_ROSTER_MAX, s_roster[idx].hex);
                log_event("leaf %d pinned %.8s", idx, s_roster[idx].hex);
                queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
                           "\"extra\":{\"leaf_pinned\":{\"hash\":\"%s\",\"slot\":%d}}}",
                           GW_SITE, GW_SITE, s_roster[idx].hex, idx);
            }
        }
        if (idx < 0) {
            s_stats.announces_foreign++;
            s_foreign_identity  = announced_identity;
            s_foreign_hello     = direct && strcmp(status, "hello") == 0;
            s_foreign_announced = true;   // loop() owes it a probe_ack / hello sync
            char fh[9] = "????????";
            if (destination_hash.size() >= 4)
                for (int i = 0; i < 4; i++)
                    sprintf(fh + i * 2, "%02x", destination_hash.data()[i]);
            Serial.printf("[gw] FOREIGN trailcam.leaf announce %s (%s%s) — no leaf "
                          "state touched (bug 7)\n", fh, status,
                          direct ? "" : ", relayed");
            log_event("FOREIGN announce %s (%s)", fh, status);
            char opt[48] = "";
            if (!isnan(LoRaInterface::last_rssi))
                snprintf(opt, sizeof(opt), ",\"rssi\":%.0f,\"snr\":%.1f",
                         LoRaInterface::last_rssi, LoRaInterface::last_snr);
            queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
                       "\"extra\":{\"foreign_announce\":{\"hash\":\"%s\","
                       "\"status\":\"%s\",\"hops\":%u%s}}}",
                       GW_SITE, GW_SITE, fh, status,
                       (unsigned)LoRaInterface::last_hops, opt);
            return;
        }
        LeafEntry& L = s_roster[idx];
        L.identity        = announced_identity;
        L.announce_direct = direct;
        // Empty-app_data announces are real protocol frames — path responses, link
        // handshakes, and relayed copies whose app_data doesn't survive the hop
        // (microReticulum quirk, see the roster bench notes). The leaf IS awake, so
        // delivery below still runs, but don't let them wipe the entry's last-known
        // status/health.
        if (status[0]) {
            snprintf(L.status, sizeof(L.status), "%s", status);
            snprintf(L.health, sizeof(L.health), "%s", htail);
        }
        // Camera slug (leaf-0.13.0): learn/refresh the identity->camera mapping that
        // command routing and beat attribution key on.
        {
            char slug[sizeof(L.slug)];
            if (htail_slug(htail, slug, sizeof(slug)) && strcmp(slug, L.slug) != 0) {
                Serial.printf("[gw] leaf %d (%.8s) is camera \"%s\"%s\n", idx, L.hex,
                              slug, L.slug[0] ? " (CHANGED)" : "");
                log_event("leaf %d slug: %s", idx, slug);
                snprintf(L.slug, sizeof(L.slug), "%s", slug);
            }
        }
        L.announced = true;   // loop() delivers — not from inside packet dispatch
        if (direct) L.last_direct_ms = millis();
        Serial.printf("[gw] leaf %d announce heard (%s)%s%s%s\n", idx, status,
                      htail[0] ? " [" : "", htail,
                      htail[0] ? "]" : "");
        queue_cmd_acks(htail);   // bug 8: relay leaf receipt acks
        if (!direct)
            Serial.printf("[gw] ^ RELAYED (hops=%u) — not counted for liveness/ADR\n",
                          (unsigned)LoRaInterface::last_hops);
        rf_note_leaf_heard();   // no-op for relayed frames (bug 1)
        // ADR SNR samples must measure the DIRECT leaf<->gateway link — a relayed
        // announce's SNR is the relay's link and poisons grant decisions (bug 1).
        if (direct && !isnan(LoRaInterface::last_snr)) rf_push_snr(LoRaInterface::last_snr);
        log_event("announce %s %s (rssi %.0f%s)", L.slug[0] ? L.slug : "leaf", status,
                  LoRaInterface::last_rssi, direct ? "" : ", RELAYED");
        // Attach the raw frame we are processing RIGHT NOW (the announce handler
        // runs synchronously inside inbound handling of this very packet) so the
        // UI's packet inspector has the actual bytes, not just the summary.
        static char hexbuf[513];   // 256 frame bytes max in a beat (announces are ~174)
        size_t hn = std::min(LoRaInterface::last_frame_len, (size_t)256);
        for (size_t i = 0; i < hn; i++)
            sprintf(hexbuf + i * 2, "%02x", LoRaInterface::last_frame[i]);
        hexbuf[hn * 2] = 0;
        // Health tail (bug 5): "p=12 c=11 f=3 vb=3.87" -> structured JSON so the app
        // can alarm on push_fails climbing without string-parsing telemetry rows.
        char health[112] = "";
        // Top-level battery_v mirrors health.vb — it's the field the app actually
        // stores in the telemetry column / last_battery_v (extra.health is opaque
        // JSON to the ingest path), so without it the UI shows "external power".
        char batt[24] = "";
        if (htail[0]) {
            long p = -1, c = -1, f = -1;
            float vb = NAN;
            const char* t;
            if ((t = strstr(htail, "p=")))  p  = atol(t + 2);
            if ((t = strstr(htail, "c=")))  c  = atol(t + 2);
            if ((t = strstr(htail, "f=")))  f  = atol(t + 2);
            if ((t = strstr(htail, "vb=")))              vb = atof(t + 3);
            int n = snprintf(health, sizeof(health),
                             ",\"health\":{\"pir_wakes\":%ld,\"captures\":%ld,\"push_fails\":%ld",
                             p, c, f);
            if (!isnan(vb) && n > 0 && n < (int)sizeof(health))
                n += snprintf(health + n, sizeof(health) - n, ",\"battery_v\":%.2f", vb);
            if (!isnan(vb))
                snprintf(batt, sizeof(batt), ",\"battery_v\":%.2f", vb);
            if (n > 0 && n < (int)sizeof(health)) snprintf(health + n, sizeof(health) - n, "}");
        }
        if (isnan(LoRaInterface::last_rssi))
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\"%s,"
                       "\"extra\":{\"status\":\"%s\",\"via\":\"mesh\",\"hops\":%u%s,"
                       "\"packet\":{\"len\":%u,\"hex\":\"%s\"}}}",
                       GW_SITE, L.slug[0] ? L.slug : gw_fallback_camera(), batt, status,
                       (unsigned)LoRaInterface::last_hops, health,
                       (unsigned)LoRaInterface::last_frame_len, hexbuf);
        else
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\"%s,"
                       "\"rssi\":%.0f,\"snr\":%.1f,"
                       "\"extra\":{\"status\":\"%s\",\"via\":\"mesh\",\"hops\":%u%s,"
                       "\"packet\":{\"len\":%u,\"hex\":\"%s\"}}}",
                       GW_SITE, L.slug[0] ? L.slug : gw_fallback_camera(), batt,
                       LoRaInterface::last_rssi, LoRaInterface::last_snr, status,
                       (unsigned)LoRaInterface::last_hops, health,
                       (unsigned)LoRaInterface::last_frame_len, hexbuf);
    }
};

static void poll_commands() {
    if (!net_up()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(10000);
    String url = String(GW_INGEST_URL);
    url.replace("/ingest", "/commands");
    if (!http.begin(tls, url)) return;
    http.addHeader("Authorization", "Bearer " GW_INGEST_TOKEN);
    int code = http.GET();
    if (code != 200) { http.end(); return; }
    String body = http.getString();
    http.end();

    // The endpoint returns a JSON LIST of command objects. Split on top-level '},{'
    // boundaries — the objects are flat except the payload{} nesting, so tracking
    // brace depth is enough (no strings contain braces in this contract).
    for (auto& c : s_cmds) c.used = false;   // rebuild from the authoritative server list
    int depth = 0, slot = 0, start = -1;
    for (int i = 0; i < (int)body.length() && slot < GW_CMD_SLOTS; i++) {
        char ch = body[i];
        if (ch == '{') { if (depth == 0) start = i; depth++; }
        else if (ch == '}') {
            depth--;
            if (depth == 0 && start >= 0) {
                int n = i - start + 1;
                if (n < (int)sizeof(s_cmds[slot].json)) {
                    // Preserve last_sent for commands we already delivered recently.
                    char tmp[512];
                    memcpy(tmp, body.c_str() + start, n);
                    tmp[n] = 0;
                    uint32_t prev_sent = 0;
                    for (auto& c : s_cmds)
                        if (!c.used && strcmp(c.json, tmp) == 0) prev_sent = c.last_sent;
                    strcpy(s_cmds[slot].json, tmp);
                    s_cmds[slot].last_sent = prev_sent;
                    s_cmds[slot].used = true;
                    slot++;
                }
                start = -1;
            }
        }
    }
}

static int deliver_pending_commands(LeafEntry& L) {
    if (!L.identity) return 0;
    RNS::Destination leaf_dest(L.identity, RNS::Type::Destination::OUT,
                               RNS::Type::Destination::SINGLE, "trailcam", "leaf");
    int sent = 0;
    for (auto& c : s_cmds) {
        if (!c.used || !c.json[0]) continue;
        // Route by camera slug: the server stamps every command with its "node"
        // (camera) slug, and leaf-0.13.0 announces theirs ("n="). A leaf that hasn't
        // told us its slug only gets commands while it's the ONLY roster member (the
        // legacy single-cam behavior); delivering blind at a multi-cam site would let
        // the wrong leaf receipt-ack a command it can never execute (bug 8 would then
        // stop the server redelivering it to the right one).
        char node[48] = "";
        json_str(c.json, "node", node, sizeof(node));
        if (L.slug[0]) {
            if (node[0] && strcmp(node, L.slug) != 0) continue;
        } else if (s_roster_n > 1) {
            continue;
        }
        if (c.last_sent && millis() - c.last_sent < GW_CMD_SUPPRESS_SECS * 1000u) continue;
        RNS::Packet pkt(leaf_dest, RNS::Bytes((const uint8_t*)c.json, strlen(c.json)));
        pkt.send();
        c.last_sent = millis();
        sent++;
    }
    if (sent) {
        Serial.printf("[gw] delivered %d command(s) into %s's RX window\n", sent,
                      L.slug[0] ? L.slug : "the leaf");
        s_stats.cmds_delivered += sent;
        log_event("delivered %d command(s) to %s", sent, L.slug[0] ? L.slug : "leaf");
    }
    return sent;
}

// Time-sync downlink: we're the mesh's clock authority (WiFi + NTP; the leaf has
// neither in normal operation). Its RTC clock reseeds from the firmware build epoch on
// every power loss and can't count the slept portion of PIR wakes — so push the real
// time into the same post-announce RX window commands use: on every "hello" (cold boot
// = its clock JUST reseeded) and every few hours as a drift refresh. The leaf sets its
// system clock from this (leaf-0.9.0+); older leaves ignore the kind (forward compat).
#define GW_TIME_SYNC_REFRESH_S (4 * 3600)
// Bug 10 (2026-07-16): the one-shot "hello" sync structurally misses leaf cold boots
// under a standing grant — the leaf boots onto base while we sit on the granted
// profile, and by the time the quiet-scan adopts down, the hello is long past (leaf
// then runs its build-epoch clock for up to 4 h; photos backdate 12 days, bug 4).
// So scan-adopt and every confirmed profile change arm an urgent sync ON EVERY roster
// entry (a profile event moves the whole site's radio, so every leaf's next announce
// may be its first heard in a while), sent on that entry's first processed announce
// after (stays armed until our own NTP can serve it).
static void arm_urgent_sync_all() {
    for (int i = 0; i < s_roster_n; i++) s_roster[i].sync_urgent = true;
}
static void maybe_time_sync(LeafEntry& L) {
    if (!L.identity) return;
    time_t now = time(nullptr);
    if (now < 1600000000) return;   // our own NTP isn't up -> nothing worth sharing
    const bool cold_boot = strcmp(L.status, "hello") == 0;
    if (!cold_boot && !L.sync_urgent && L.synced_once &&
        millis() - L.last_sync_ms < GW_TIME_SYNC_REFRESH_S * 1000u) return;
    RNS::Destination leaf_dest(L.identity, RNS::Type::Destination::OUT,
                               RNS::Type::Destination::SINGLE, "trailcam", "leaf");
    char json[80];
    snprintf(json, sizeof(json), "{\"kind\":\"time_sync\",\"payload\":{\"unix\":%lld}}",
             (long long)time(nullptr));
    RNS::Packet pkt(leaf_dest, RNS::Bytes((const uint8_t*)json, strlen(json)));
    pkt.send();
    L.last_sync_ms = millis();
    L.synced_once  = true;
    Serial.printf("[gw] time sync sent to %s (unix %lld%s%s)\n",
                  L.slug[0] ? L.slug : "leaf", (long long)now,
                  cold_boot ? ", leaf cold boot" : "",
                  L.sync_urgent ? ", profile adopt/change" : "");
    log_event("time sync sent (unix %lld)", (long long)now);
    L.sync_urgent = false;
}

// --- ADR state machine -------------------------------------------------------------------
// Grant policy: rolling announce SNR vs the profile table's demod floors. Upgrade one
// step only with >= 12 dB headroom above the FASTER profile's floor; downgrade one step
// at <= 6 dB above the CURRENT floor (asymmetric on purpose: slow to speed up, fast to
// slow down). The lockstep dance: send the grant into the RX window, retune ourselves
// immediately, and expect the leaf's confirm announce ON THE NEW PROFILE — no confirm in
// 20 s means the grant packet was lost, so revert (the leaf never heard it and is still
// on the old profile). Every failure path converges both ends back to base: the leaf
// reverts on TTL expiry / contactless wakes / power loss, we revert on confirm timeout
// and scan between granted-and-base when the leaf goes quiet off-base.
#define GW_RF_SNR_SAMPLES_MIN     3
#define GW_RF_UP_HEADROOM_DB      12.0f
#define GW_RF_DOWN_HEADROOM_DB    6.0f
#define GW_RF_CONFIRM_TIMEOUT_MS  20000u
#define GW_RF_SWITCH_COOLDOWN_MS  60000u
#define GW_RF_GRANT_TTL_S         (24 * 3600)      // matches the leaf's default
#define GW_RF_SCAN_SILENT_MS      (90 * 1000u)     // leaf quiet this long off-base -> scan
#define GW_RF_SCAN_TOGGLE_MS      (30 * 1000u)

static void rf_beat(const char* event, uint8_t idx, float snr, float headroom) {
    // snr/headroom are optional (NAN = omit). %.1f of NAN prints "nan" — invalid JSON
    // that the telemetry endpoint rejects — so the fields are appended conditionally.
    char opt[48] = "";
    if (!isnan(snr)) {
        int n = snprintf(opt, sizeof(opt), ",\"snr\":%.1f", snr);
        if (!isnan(headroom) && n > 0 && n < (int)sizeof(opt))
            snprintf(opt + n, sizeof(opt) - n, ",\"headroom\":%.1f", headroom);
    }
    // Gateway self-beat: ADR decisions (scan/grant/confirm/revert) are made BY the
    // gateway; they were only ever filed under a camera because a node string was needed.
    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"extra\":{\"rf\":{\"event\":\"%s\",\"profile\":\"%s\",\"idx\":%u%s}}}",
               GW_SITE, GW_SITE, event, rf_name(idx), (unsigned)idx, opt);
}

static void rf_send_grant(const RNS::Identity& leaf_id, uint8_t idx) {
    RNS::Destination leaf_dest(leaf_id, RNS::Type::Destination::OUT,
                               RNS::Type::Destination::SINGLE, "trailcam", "leaf");
    char json[96];
    snprintf(json, sizeof(json),
             "{\"kind\":\"radio_profile\",\"payload\":{\"idx\":%u,\"ttl_s\":%u}}",
             (unsigned)idx, (unsigned)GW_RF_GRANT_TTL_S);
    RNS::Packet pkt(leaf_dest, RNS::Bytes((const uint8_t*)json, strlen(json)));
    pkt.send();
    s_rf.last_grant_tx_ms   = millis();
    s_rf.announces_since_tx = 0;
}

// On every DIRECT leaf announce, BEFORE command delivery: settle any in-flight state.
// Relayed announces (hops>0) must not touch ADR state (bug 1): a relay can hear the
// leaf on a profile we can't, so "an announce arrived" proves nothing about OUR link —
// confirming/adopting on one latches the split instead of healing it.
static void rf_on_leaf_announce(bool direct) {
    if (!direct) return;
    if (s_rf.announces_since_tx < 255) s_rf.announces_since_tx++;
    if (s_rf.pending != 0xFF && s_rf.current == s_rf.pending) {
        // Heard the leaf on the profile we just granted -> lock it in.
        s_rf.granted = s_rf.pending;
        s_rf.pending = 0xFF;
        rf_persist(s_rf.granted);
        Serial.printf("[gw] ADR: leaf CONFIRMED on %s\n", rf_name(s_rf.granted));
        log_event("ADR confirmed %s", rf_name(s_rf.granted));
        rf_beat("confirmed", s_rf.granted, LoRaInterface::last_snr, NAN);
        arm_urgent_sync_all();   // bug 10: fresh profile -> resync the leaf clocks
    }
    if (s_rf.scanning) {
        // Scan found the leaf on whatever we're parked on — adopt that as the truth
        // (usually base: the leaf power-cycled and its RTC grant wiped).
        s_rf.scanning = false;
        s_rf.granted  = s_rf.current;
        rf_persist(s_rf.granted);
        rf_clear_snr();
        Serial.printf("[gw] ADR: scan found the leaf on %s -> adopted\n",
                      rf_name(s_rf.granted));
        log_event("ADR scan: leaf found on %s", rf_name(s_rf.granted));
        rf_beat("adopted", s_rf.granted, LoRaInterface::last_snr, NAN);
        // Bug 10: scan-adopt IS the cold-boot-under-standing-grant path — the "hello"
        // announce came and went on a profile we weren't parked on, so the one-shot
        // hello sync never fired and the leaf is running its build-epoch clock.
        arm_urgent_sync_all();
    }
}

// After command delivery on an announce: decide whether to move the leaf. cmds_sent
// gates the grant — a leaf busy executing a fetch_full must not have the PHY yanked
// from under the transfer it's about to start.
static void maybe_grant_profile(LeafEntry& L, int cmds_sent) {
    if (!L.identity || s_rf.pending != 0xFF) return;
    // Multi-leaf site (gateway-0.10.0): everyone lives on BASE. One radio can't serve
    // two leaves on different profiles, and a fast site profile makes a newly added
    // leaf invisible (it boots on base while we camp off-base — its hello is never
    // heard). Any standing grant gets actively walked back: tell whichever member is
    // announcing right now "base" and adopt base ourselves immediately; members still
    // holding the old grant self-revert within 3 contactless wakes (leaf ADR safety).
    if (s_roster_n > 1) {
        if (s_rf.granted != LORA_PROFILE_BASE || s_rf.current != LORA_PROFILE_BASE) {
            rf_send_grant(L.identity, LORA_PROFILE_BASE);
            rf_retune(LORA_PROFILE_BASE);
            s_rf.granted  = LORA_PROFILE_BASE;
            s_rf.scanning = false;
            rf_persist(LORA_PROFILE_BASE);
            rf_clear_snr();
            Serial.printf("[gw] ADR: %d leaves on the roster -> walking the site back "
                          "to %s (grants suspended)\n", s_roster_n,
                          rf_name(LORA_PROFILE_BASE));
            log_event("ADR: multi-leaf -> base, grants suspended");
            rf_beat("multi_leaf_base", LORA_PROFILE_BASE, NAN, NAN);
        }
        return;
    }
    if (s_rf.scanning) return;
    if (cmds_sent || asm_active()) return;   // transfer imminent or in progress
    // Keepalive refresh of a standing off-base grant. The leaf treats ANY packet from
    // us as proof we still hear it and reverts to base after 3 contactless wakes — but
    // an idle leaf gets no traffic at all (commands/time_sync are occasional), so a
    // granted profile would silently expire in 3 wakes. Re-sending the grant every 2nd
    // announce feeds that counter at whatever cadence the leaf actually wakes on
    // (bench 28 s or field 30 min alike); a same-idx refresh just re-arms the leaf's
    // TTL + counter without a confirm announce (leaf-0.10.1).
    if (s_rf.granted != LORA_PROFILE_BASE && s_rf.announces_since_tx >= 2) {
        rf_send_grant(L.identity, s_rf.granted);
        Serial.printf("[gw] ADR: grant keepalive (%s)\n", rf_name(s_rf.granted));
        return;
    }
    if (s_rf.last_switch_ms && millis() - s_rf.last_switch_ms < GW_RF_SWITCH_COOLDOWN_MS)
        return;
    if (s_rf.snr_n < GW_RF_SNR_SAMPLES_MIN) return;
    const float   snr  = rf_avg_snr();
    const uint8_t cur  = s_rf.current;
    uint8_t       want = cur;
    float         headroom = NAN;
    if (cur + 1 < LORA_PROFILE_COUNT) {
        headroom = snr - LORA_PROFILES[cur + 1].snr_floor;
        if (headroom >= GW_RF_UP_HEADROOM_DB) want = cur + 1;
    }
    if (want == cur && cur > 0 &&
        snr - LORA_PROFILES[cur].snr_floor <= GW_RF_DOWN_HEADROOM_DB) {
        want = cur - 1;
        headroom = snr - LORA_PROFILES[cur].snr_floor;
    }
    if (want == cur) return;
    Serial.printf("[gw] ADR: granting %s (snr %.1f, headroom %.1f dB) -> retuning\n",
                  rf_name(want), snr, headroom);
    log_event("ADR grant %s (snr %.1f)", rf_name(want), snr);
    rf_beat("grant", want, snr, headroom);
    rf_send_grant(L.identity, want);
    s_rf.prev           = cur;
    s_rf.pending        = want;
    s_rf.pending_ms     = millis();
    s_rf.last_switch_ms = millis();
    rf_retune(want);   // follow our own grant; the leaf confirms here or we revert
    rf_clear_snr();    // SNR history doesn't transfer across profiles (BW noise floor)
}

// --- announce-starvation watchdog (bug 6, root-caused 2026-07-16) -----------------------
// Transport's announce replay guard keys on the EMISSION TIMESTAMP embedded in each
// announce (random_hash bytes 5..9): an announce is only processed if its timestamp
// exceeds the max ever recorded in the path-table entry. If the leaf's clock is ever
// corrected BACKWARD — e.g. our own time_sync landing on a leaf that booted 4 h fast
// off a UTC-built build-epoch seed — every subsequent announce reads "older" and is
// silently dropped: frames demodulate (announces_seen grows) but no handler ever fires,
// so no scan-adopt, no commands, no keepalives, until wall clock crosses the poisoned
// timebase (the 07-15 all-night wedge; reproduced deliberately on the bench 07-16).
// The path-table entry PERSISTS in LittleFS, so reboots don't clear it. Self-heal:
// when announces keep arriving but none process for a while, drop the leaf's path
// entry — the next announce then arrives as "unknown destination" and is accepted
// unconditionally, rebuilding the path with a fresh timebase.
#define GW_ANN_WEDGE_MIN_SEEN   4
#define GW_ANN_WEDGE_SILENT_MS  (4 * 60 * 1000u)
#define GW_ANN_WEDGE_REBOOT_AT  3   // consecutive fires with zero processed -> reboot
static void announce_wedge_watchdog() {
    static uint32_t processed_baseline = 0, seen_baseline = 0, baseline_ms = 0;
    static uint8_t  consecutive_fires = 0;
    if (s_stats.announces != processed_baseline || baseline_ms == 0) {
        processed_baseline = s_stats.announces;          // announces are flowing (or
        seen_baseline      = LoRaInterface::announces_seen;   // first call) -> re-arm
        baseline_ms        = millis();
        consecutive_fires  = 0;
        return;
    }
    if (LoRaInterface::announces_seen - seen_baseline < GW_ANN_WEDGE_MIN_SEEN) return;
    if (millis() - baseline_ms < GW_ANN_WEDGE_SILENT_MS) return;
    if (!s_roster_n) return;
    // Drop every roster member's path entry. The pins ARE the trailcam.leaf
    // destination hashes (that's what the announce handler matched on), so the NVS
    // roster serves here even before any announce has processed this boot — the old
    // s_leaf_identity gate meant a reboot-into-wedge couldn't self-heal at all.
    bool had_path = false;
    for (int i = 0; i < s_roster_n; i++) {
        RNS::Bytes h(s_roster[i].pin, sizeof(s_roster[i].pin));
        if (RNS::Transport::has_path(h)) {
            RNS::Transport::remove_path(h);
            had_path = true;
        }
    }
    consecutive_fires++;
    Serial.printf("[gw] announce WEDGE (%u/%u): %lu announces demodulated, 0 processed "
                  "in %lus -> dropped leaf path entry%s (replay-guard reset)\n",
                  (unsigned)consecutive_fires, (unsigned)GW_ANN_WEDGE_REBOOT_AT,
                  (unsigned long)(LoRaInterface::announces_seen - seen_baseline),
                  (unsigned long)((millis() - baseline_ms) / 1000),
                  had_path ? "" : " (none existed)");
    log_event("announce wedge %u/%u: dropped leaf path", (unsigned)consecutive_fires,
              (unsigned)GW_ANN_WEDGE_REBOOT_AT);
    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"extra\":{\"announce_wedge\":{\"seen\":%lu,\"silent_s\":%lu,\"fires\":%u}}}",
               GW_SITE, GW_SITE,
               (unsigned long)(LoRaInterface::announces_seen - seen_baseline),
               (unsigned long)((millis() - baseline_ms) / 1000),
               (unsigned)consecutive_fires);
    // Escalation (2026-07-16 surveyor re-test): a wedge survived repeated path drops —
    // some RAM-side Transport state (not the persisted path table) kept rejecting
    // announces at Identity::validate_announce / the announce region, and only a
    // reboot has ever cleared that class. An unattended field gateway must not sit
    // announce-starved for hours (no commands, no ADR, no scan-adopt), so after
    // GW_ANN_WEDGE_REBOOT_AT fruitless drops, restart: NVS grant + identity persist,
    // boot-time scan-first (bug 2) re-converges within ~2 min. A transfer aborted by
    // the reboot retries server-side; announce starvation does not.
    if (consecutive_fires >= GW_ANN_WEDGE_REBOOT_AT) {
        Serial.println("[gw] announce WEDGE persists through path drops -> REBOOTING "
                       "to clear Transport RAM state");
        log_event("announce wedge unrecoverable -> reboot");
        post_beats();     // best-effort: get the wedge beats out before restarting
        delay(500);
        ESP.restart();
    }
    seen_baseline = LoRaInterface::announces_seen;   // re-arm; don't spin-remove
    baseline_ms   = millis();
}

// Housekeeping outside the announce path: confirm timeouts + the quiet-leaf scan.
static void rf_loop() {
    // Scan liveness counts only DIRECT PINNED-LEAF announces (s_rf.last_leaf_ms via
    // rf_note_leaf_heard). It used to count every direct frame so a >90 s in-flight
    // resource chunk wouldn't get the radio retuned from under it (2026-07-03) — but
    // that let ANY transmitter suppress the scan: the surveyor's frames kept it
    // disarmed while the real leaf sat unreachable at base (bug 7, 07-16 re-test).
    // The mid-transfer protection moved to the radio_busy()/asm_active() gates below.
    if (s_rf.pending != 0xFF && millis() - s_rf.pending_ms > GW_RF_CONFIRM_TIMEOUT_MS) {
        Serial.printf("[gw] ADR: no confirm on %s -> reverting to %s\n",
                      rf_name(s_rf.pending), rf_name(s_rf.prev));
        log_event("ADR revert to %s (no confirm)", rf_name(s_rf.prev));
        rf_beat("revert", s_rf.prev, NAN, NAN);
        rf_retune(s_rf.prev);
        s_rf.pending = 0xFF;
        rf_clear_snr();
    }
    // Camped off-base and the leaf has gone quiet: it may have reverted to base
    // (power loss / TTL) where we can't hear it. Alternate between the granted
    // profile and base until it reappears; announce-time adoption settles it.
    // radio_busy()/asm_active() keep the scan off a live transfer (frames every
    // <2 s while a resource is in flight; the assembly holds across leaf wakes).
    if (s_rf.pending == 0xFF && s_rf.granted != LORA_PROFILE_BASE &&
        !radio_busy() && !asm_active() &&
        millis() - s_rf.last_leaf_ms > GW_RF_SCAN_SILENT_MS &&
        millis() - s_rf.last_scan_ms > GW_RF_SCAN_TOGGLE_MS) {
        s_rf.last_scan_ms = millis();
        s_rf.scanning     = true;
        const uint8_t park = (s_rf.current == s_rf.granted) ? LORA_PROFILE_BASE
                                                            : s_rf.granted;
        rf_retune(park);
        Serial.printf("[gw] ADR: leaf quiet %lus -> scanning on %s\n",
                      (unsigned long)((millis() - s_rf.last_leaf_ms) / 1000),
                      rf_name(park));
    }
}

#ifdef GW_NET_ETH
// --- wired ethernet (LILYGO T-Internet-POE, LAN8720) ------------------------------------
// Pin map proven on the bench 2026-06-30 (bridge-poe Step 1): MDC 23, MDIO 18, PHY addr 0,
// clock OUT on GPIO17, no power-enable pin. Classic Arduino-ESP32 core 2.x ETH API — the
// env is pinned to espressif32@6.x for exactly this signature.
#define GW_ETH_ADDR        0
#define GW_ETH_POWER_PIN  -1
#define GW_ETH_MDC_PIN    23
#define GW_ETH_MDIO_PIN   18
#define GW_ETH_TYPE       ETH_PHY_LAN8720
#define GW_ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT

static void on_eth_event(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:     Serial.println("[gw] ETH start"); ETH.setHostname("trailcam-gw"); break;
        case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("[gw] ETH link up"); break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.printf("[gw] ETH got IP: %s  (%uMbps, %s)\n",
                          ETH.localIP().toString().c_str(), ETH.linkSpeed(),
                          ETH.fullDuplex() ? "full" : "half");
            s_eth_got_ip = true;
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED: Serial.println("[gw] ETH link DOWN"); s_eth_got_ip = false; break;
        case ARDUINO_EVENT_ETH_STOP:         Serial.println("[gw] ETH stop"); s_eth_got_ip = false; break;
        default: break;
    }
}

// Bring-up is event-driven and non-blocking; DHCP renewal/link flaps are the PHY's
// problem, so the periodic ensure is a no-op (kept for loop() symmetry with WiFi).
static void net_setup()  { WiFi.onEvent(on_eth_event);   // ETH events ride the WiFi event bus
                           ETH.begin(GW_ETH_ADDR, GW_ETH_POWER_PIN, GW_ETH_MDC_PIN,
                                     GW_ETH_MDIO_PIN, GW_ETH_TYPE, GW_ETH_CLK_MODE); }
static void net_ensure() {}
#else
// --- WiFi + ingest ---------------------------------------------------------------------
// Baked network list, tried in order: the site's primary network first, then any fallback
// (the bench SSID) so the SAME firmware works on-site and on the bench. Add more pairs
// by defining GW_WIFI_SSID3/PSK3, etc.
struct WifiCred { const char* ssid; const char* psk; };
static const WifiCred WIFI_NETS[] = {
    { GW_WIFI_SSID,  GW_WIFI_PSK  },
#if defined(GW_WIFI_SSID2) && defined(GW_WIFI_PSK2)
    { GW_WIFI_SSID2, GW_WIFI_PSK2 },
#endif
#if defined(GW_WIFI_SSID3) && defined(GW_WIFI_PSK3)
    { GW_WIFI_SSID3, GW_WIFI_PSK3 },
#endif
};
static const size_t WIFI_NET_COUNT = sizeof(WIFI_NETS) / sizeof(WIFI_NETS[0]);
static size_t s_wifi_idx = 0;   // which network we're currently attempting

#define GW_WIFI_TRY_MS 15000    // per-network association window before rotating

static void wifi_ensure() {
    if (WiFi.status() == WL_CONNECTED) return;
    static uint32_t last_try = 0;
    if (last_try && millis() - last_try < GW_WIFI_TRY_MS) return;
    // The current network didn't associate in its window -> rotate to the next one.
    if (last_try && WIFI_NET_COUNT > 1) s_wifi_idx = (s_wifi_idx + 1) % WIFI_NET_COUNT;
    last_try = millis();
    const WifiCred& n = WIFI_NETS[s_wifi_idx];
    Serial.printf("[gw] WiFi joining \"%s\" (net %u/%u) ...\n",
                  n.ssid, (unsigned)(s_wifi_idx + 1), (unsigned)WIFI_NET_COUNT);
    WiFi.mode(WIFI_STA);
    WiFi.begin(n.ssid, n.psk);
}
static void net_setup()  { wifi_ensure(); }
static void net_ensure() { wifi_ensure(); }
#endif

static bool s_ntp_started = false;
static void ntp_ensure() {
    if (s_ntp_started || !net_up()) return;
    configTime(0, 0, "pool.ntp.org");   // UTC
    s_ntp_started = true;
}

// Tiny flat-JSON string extractor (same approach as the leaf's command scanner).
static bool json_str(const char* json, const char* key, char* out, size_t cap) {
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) out[i++] = *p++;
    out[i] = '\0';
    return *p == '"';
}
static long long json_ll(const char* json, const char* key) {
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    return atoll(p);
}

// Multipart POST per the ingest contract (mirrors host-bridge _post_photo), streamed
// from three segments (head / body / tail). The body segment is either a RAM buffer
// (thumbs) or an open LittleFS File (spooled fulls) — either way a 135 KB full never
// touches the heap.
class ConcatStream : public Stream {
public:
    ConcatStream(const String& head, const uint8_t* body, size_t blen, const String& tail)
        : _h(head), _b(body), _bl(blen), _t(tail) {}
    ConcatStream(const String& head, File* body, size_t blen, const String& tail)
        : _h(head), _f(body), _bl(blen), _t(tail) {}
    int available() override {
        size_t total = _h.length() + _bl + _t.length();
        return (int)(total - _pos);
    }
    int read() override {
        uint8_t c;
        return readBytes(&c, 1) == 1 ? c : -1;
    }
    size_t readBytes(char* out, size_t want) { return readBytes((uint8_t*)out, want); }
    size_t readBytes(uint8_t* out, size_t want) {
        size_t done = 0;
        while (done < want) {
            size_t hl = _h.length();
            if (_pos < hl) {
                out[done++] = _h[_pos++];
            } else if (_pos < hl + _bl) {
                size_t off = _pos - hl;
                size_t n = min(want - done, _bl - off);
                if (_f) {
                    size_t got = _f->read(out + done, n);
                    if (got == 0) break;   // short file (shouldn't happen): end stream
                    n = got;
                } else {
                    memcpy(out + done, _b + off, n);
                }
                done += n;
                _pos += n;
            } else {
                size_t off = _pos - hl - _bl;
                if (off >= (size_t)_t.length()) break;
                out[done++] = _t[off];
                _pos++;
            }
        }
        return done;
    }
    int peek() override { return -1; }
    size_t write(uint8_t) override { return 0; }
private:
    const String&  _h;
    const uint8_t* _b = nullptr;
    File*          _f = nullptr;
    size_t         _bl;
    const String&  _t;
    size_t         _pos = 0;
};

static bool upload_pending() {
    time_t now = time(nullptr);
    if (now < 1600000000) {
        Serial.println("[gw] clock not NTP-synced yet -> deferring upload");
        return false;
    }
    long long capts = s_pending.captured_at;
    if (capts < 1600000000) capts = (long long)now;

    static const char* B = "----tcgw7f3a";
    String head;
    head.reserve(768);
    auto field = [&](const char* name, const String& val) {
        head += "--"; head += B; head += "\r\n";
        head += "Content-Disposition: form-data; name=\""; head += name; head += "\"\r\n\r\n";
        head += val; head += "\r\n";
    };
    field("site", GW_SITE);
    field("camera", s_pending.camera);
    field("event_id", s_pending.event_id);
    field("captured_at", String(capts));
    field("kind", s_pending.kind);
    head += "--"; head += B; head += "\r\n";
    head += "Content-Disposition: form-data; name=\"file\"; filename=\"";
    head += s_pending.event_id; head += ".jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = String("\r\n--") + B + "--\r\n";

    // Spooled fulls stream from flash; reopen per attempt so a retry restarts at 0.
    File spool;
    if (s_pending.spooled) {
        spool = LittleFS.open(GW_SPOOL_JPG, FILE_READ);
        if (!spool || spool.size() != s_pending.len) {
            Serial.printf("[gw] spool file bad (%u vs %u expected) -> dropping upload\n",
                          spool ? (unsigned)spool.size() : 0, (unsigned)s_pending.len);
            if (spool) spool.close();
            return true;   // treat as handled; the server re-delivers the command
        }
    }

    WiFiClientSecure tls;
    tls.setInsecure();   // no CA store; the token is the auth, the LAN path is ours
    HTTPClient http;
    http.setConnectTimeout(10000);
    http.setTimeout(30000);
    bool ok = false;
    if (http.begin(tls, GW_INGEST_URL)) {
        http.addHeader("Authorization", "Bearer " GW_INGEST_TOKEN);
        http.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
        ConcatStream mem(head, s_pending.jpeg, s_pending.len, tail);
        ConcatStream fil(head, &spool,         s_pending.len, tail);
        ConcatStream* body = s_pending.spooled ? &fil : &mem;
        size_t total = head.length() + s_pending.len + tail.length();
        int code = http.sendRequest("POST", body, total);
        ok = code >= 200 && code < 300;
        Serial.printf("[gw] upload %s %s (%u bytes) -> HTTP %d%s\n",
                      s_pending.event_id, s_pending.kind, (unsigned)s_pending.len,
                      code, ok ? "" : " (will retry)");
        if (ok) s_stats.uploads_ok++;
        log_event("upload %s %s -> HTTP %d", s_pending.event_id, s_pending.kind, code);
        if (!ok && code >= 400 && code < 500) {
            Serial.printf("[gw] 4xx is permanent -> dropping: %s\n",
                          http.getString().substring(0, 200).c_str());
            ok = true;   // treat as handled; retrying a malformed frame forever helps nobody
        }
        http.end();
    } else {
        Serial.println("[gw] http.begin failed");
    }
    if (spool) spool.close();
    return ok;
}

// --- site name (address) fetch ----------------------------------------------------------
// GET /api/v1/site?slug=<GW_SITE> with the device token; cache the display name in NVS.
// Blocking TLS GET, so callers gate it on !radio_busy() like the other HTTP work.
static void fetch_site_name() {
    if (!net_up()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(10000);
    String url = String(GW_INGEST_URL);
    url.replace("/ingest", "/site?slug=" GW_SITE);
    if (!http.begin(tls, url)) return;
    http.addHeader("Authorization", "Bearer " GW_INGEST_TOKEN);
    if (http.GET() == 200) {
        String body = http.getString();
        char name[sizeof(s_site_name)];
        if (json_str(body.c_str(), "name", name, sizeof(name)) && name[0] &&
            strcmp(name, s_site_name) != 0) {
            snprintf(s_site_name, sizeof(s_site_name), "%s", name);
            Preferences p;
            p.begin("gateway", false);
            p.putString("site_name", s_site_name);
            p.end();
            Serial.printf("[gw] site name: \"%s\"\n", s_site_name);
        }
    }
    http.end();
}

// --- OLED render ------------------------------------------------------------------------
#ifdef GW_HAS_OLED
static void oled_init() {
    // Heltec V3 powers the OLED through Vext — must be pulled LOW or the panel stays dark.
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(60);                       // let the rail settle before I2C
    u8g2.setBusClock(400000);
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.drawStr(14, 28, "Trail Cam");
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(14, 46, GW_FW_VERSION);
    u8g2.sendBuffer();
}

// WiFi signal bars at the top-right of page 1. RSSI -> 0..4 filled bars; a small
// empty box when the link is down. Bars span x 106..124, bottom aligned to y=11.
static void draw_wifi_icon(bool wifi, int rssi) {
    const int x0 = 106, base = 11;
    if (!wifi) { u8g2.drawFrame(x0 + 6, base - 4, 5, 4); return; }
    int bars = 0;
    if      (rssi >= -55) bars = 4;
    else if (rssi >= -65) bars = 3;
    else if (rssi >= -72) bars = 2;
    else if (rssi >= -80) bars = 1;
    for (int i = 0; i < 4; i++) {
        const int h  = 2 + i * 2;           // 2,4,6,8 px tall
        const int bx = x0 + i * 5;          // 3px bar + 2px gap
        if (i < bars) u8g2.drawBox(bx, base - h, 3, h);
        else          u8g2.drawBox(bx, base - 1, 3, 1);   // baseline stub
    }
}

// Two pages, auto-rotating every 4 s. 6x12 font = ~21 cols.
static void oled_render() {
    const bool page2 = ((millis() / 4000) % 2) == 1;
    const bool wifi  = WiFi.status() == WL_CONNECTED;
    char line[28];
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    if (!page2) {
        // Page 1 — which site (address) + how to see photos. Title is the backend
        // site name so the two gateways are distinguishable at a glance (falls back
        // to the slug until the first fetch populates it).
        // Never show the internal slug (e.g. "north40") on a screen in someone's
        // home — fall back to a neutral label until the backend name is fetched.
        const char* title = s_site_name[0] ? s_site_name : "Trail Cam";
        snprintf(line, sizeof(line), "%.17s", title);
        u8g2.drawStr(0, 10, line);
        draw_wifi_icon(wifi, wifi ? WiFi.RSSI() : 0);
        u8g2.drawHLine(0, 13, 128);
        if (wifi) {
            u8g2.drawStr(0, 28, "See photos at:");
            u8g2.drawStr(0, 42, GW_VIEW_HOST);   // trailcam.example.com (family gallery)
        } else {
            u8g2.drawStr(0, 28, "trying network:");
            snprintf(line, sizeof(line), "%.21s", WIFI_NETS[s_wifi_idx].ssid);
            u8g2.drawStr(0, 42, line);
        }
        snprintf(line, sizeof(line), "%u photos today", (unsigned)s_caps_today);
        u8g2.drawStr(0, 56, line);
    } else {
        // Page 2 — last capture
        u8g2.drawStr(0, 10, "Last capture");
        u8g2.drawHLine(0, 13, 128);
        if (s_have_cap) {
            snprintf(line, sizeof(line), "%.20s", s_last_cap_cam);
            u8g2.drawStr(4, 30, line);
            char ago[24];
            human_ago(millis() - s_last_cap_ms, ago, sizeof(ago));
            u8g2.drawStr(4, 44, ago);
        } else {
            u8g2.drawStr(4, 34, "no captures yet");
        }
        char up[16];
        human_up(millis() / 1000, up, sizeof(up));
        snprintf(line, sizeof(line), "today %u  %s", (unsigned)s_caps_today, up);
        u8g2.drawStr(0, 56, line);
    }
    u8g2.sendBuffer();
}
#endif  // GW_HAS_OLED

// --- network firmware updates (ArduinoOTA / espota) -------------------------------------
// Enabled once, the first time WiFi comes up. Reachable at <ip>:3232, password-gated.
// Push from a machine that can route to the box (over the WG site-to-site tunnel):
//   pio run -e heltec-v3-gateway-ota -t upload --upload-port <gateway-ip>
static void ota_setup() {
    ArduinoOTA.setHostname("trailcam-gw");
    ArduinoOTA.setPassword(GW_OTA_PASS);
    ArduinoOTA.onStart([]() {
        Serial.println("[gw] OTA start — pausing mesh");
#ifdef GW_HAS_OLED
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.drawStr(10, 38, "Updating...");
        u8g2.sendBuffer();
#endif
    });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        static int last = -1;
        int pct = t ? (int)((uint64_t)p * 100 / t) : 0;
        if (pct == last || pct % 10) return;   // redraw every 10% only
        last = pct;
#ifdef GW_HAS_OLED
        char b[16];
        snprintf(b, sizeof(b), "OTA  %d%%", pct);
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.drawStr(24, 38, b);
        u8g2.sendBuffer();
#else
        Serial.printf("[gw] OTA %d%%\n", pct);
#endif
    });
    ArduinoOTA.onEnd([]()  { Serial.println("[gw] OTA complete — rebooting"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[gw] OTA error %u\n", (unsigned)e); });
    ArduinoOTA.begin();
    Serial.println("[gw] ArduinoOTA ready (host trailcam-gw, port 3232)");
}

// --- gateway self-telemetry -------------------------------------------------------------
static const char* reset_reason_str() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "power_on";
        case ESP_RST_SW:        return "sw_reset";    // esp_restart() — e.g. after OTA
        case ESP_RST_PANIC:     return "panic";       // crash
        case ESP_RST_INT_WDT:   return "int_wdt";
        case ESP_RST_TASK_WDT:  return "task_wdt";
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_EXT:       return "ext";
        default:                return "other";
    }
}

// The gateway's own health beat. rssi/boot_reason are first-class columns; everything
// gateway-specific (mesh reception, forwarding counts, IP, SoC temp) rides in `extra`.
static void emit_self_beat() {
    // seconds since we last heard ANY leaf packet ("is the mesh alive?"); null if never.
    long mesh_age = s_rf.last_leaf_ms ? (long)((millis() - s_rf.last_leaf_ms) / 1000) : -1;
    char meshc[40];
    if (mesh_age < 0) snprintf(meshc, sizeof(meshc), "\"last_mesh_contact_s\":null");
    else              snprintf(meshc, sizeof(meshc), "\"last_mesh_contact_s\":%ld", mesh_age);
    // LoRa reception of the last leaf packet (distinct from WiFi rssi); null if none yet.
    float lr = LoRaInterface::last_rssi, ls = LoRaInterface::last_snr;
    char lora[56];
    if (isnan(lr)) snprintf(lora, sizeof(lora), "\"lora_rssi\":null,\"lora_snr\":null");
    else           snprintf(lora, sizeof(lora), "\"lora_rssi\":%.0f,\"lora_snr\":%.1f", lr, ls);
    // Roster (bug 7 multi-leaf): every pinned identity + its learned camera slug, so
    // "which cameras does this gateway manage" is visible remotely. "leaf_pin"
    // (entry 0) stays for continuity with gateway-0.9.0 dashboards.
    char roster[GW_ROSTER_MAX * 80 + 4] = "[";
    {
        int n = 1;
        for (int i = 0; i < s_roster_n && n > 0 && n < (int)sizeof(roster) - 2; i++)
            n += snprintf(roster + n, sizeof(roster) - n,
                          "%s{\"hash\":\"%s\",\"slug\":%s%s%s}", i ? "," : "",
                          s_roster[i].hex,
                          s_roster[i].slug[0] ? "\"" : "",
                          s_roster[i].slug[0] ? s_roster[i].slug : "null",
                          s_roster[i].slug[0] ? "\"" : "");
        strlcat(roster, "]", sizeof(roster));
    }

    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"uptime_s\":%lu,\"fw_version\":\"" GW_FW_VERSION "\","
               "\"rssi\":%d,\"boot_reason\":\"%s\","
               "\"extra\":{\"free_heap\":%u,\"wifi_ssid\":\"%s\",\"ip\":\"%s\","
               "\"soc_temp_c\":%.1f,\"lora_profile\":\"%s\",%s,%s,"
               "\"uploads\":%lu,\"res_ok\":%lu,\"res_fail\":%lu,"
               "\"announces\":%lu,\"announces_rf\":%lu,\"announces_relayed\":%lu,"
               "\"announces_foreign\":%lu,\"leaf_pin\":%s%s%s,\"roster\":%s,"
               "\"chunks\":%lu}}",
               GW_SITE, GW_SITE, (unsigned long)(millis() / 1000),
               net_rssi(), reset_reason_str(),
               (unsigned)ESP.getFreeHeap(), net_desc().c_str(),
               net_ip().toString().c_str(),
               temperatureRead(), rf_name(s_rf.current), meshc, lora,
               (unsigned long)s_stats.uploads_ok, (unsigned long)s_stats.resources_ok,
               (unsigned long)s_stats.resources_fail, (unsigned long)s_stats.announces,
               // announces_rf counts ANNOUNCE frames demodulated at the radio;
               // a growing rf-vs-processed gap is Transport dropping announces
               // (replay guard / dedup) — the bug-6 pathology, now visible remotely.
               (unsigned long)LoRaInterface::announces_seen,
               (unsigned long)s_stats.announces_relayed,
               // foreign announce count + pin: a nonzero foreign count on a field
               // gateway = an impostor/surveyor is in earshot (bug 7, visible remotely)
               (unsigned long)s_stats.announces_foreign,
               s_roster_n ? "\"" : "", s_roster_n ? s_roster[0].hex : "null",
               s_roster_n ? "\"" : "", roster,
               (unsigned long)s_stats.chunks);
}

// --- bring-up ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(100);
    Serial.printf("[gw] firmware %s booting\n", GW_FW_VERSION);
    load_cached_site_name();          // so the screen shows the address at once
    // Bug 7: restore the leaf identity roster. Empty -> TOFU arms; direct announces
    // with a health tail (real leaves, never the surveyor) enroll themselves.
    if (load_roster())
        for (int i = 0; i < s_roster_n; i++)
            Serial.printf("[gw] leaf pin %d/%d restored: %s\n", i + 1, s_roster_n,
                          s_roster[i].hex);
    else
        Serial.println("[gw] empty leaf roster — TOFU armed (direct health announces enroll)");
#ifdef GW_HAS_OLED
    oled_init();                      // light the parent-facing screen ASAP
#endif
    RNS::loglevel(RNS::LOG_NOTICE);   // bump to LOG_DEBUG when chasing radio issues
    Serial.printf("[gw] boot. free heap=%u\n", (unsigned)ESP.getFreeHeap());

    // Filesystem: mount existing, format only on the very first boot (identity's stores
    // and Transport's path tables persist — unlike gate-a, which formatted every boot).
    filesystem.init(/*reformatOnFail=*/true);
    RNS::Utilities::OS::register_filesystem(filesystem);
    // Crash leftovers: a spool interrupted by a reboot is worthless (the server
    // re-delivers the fetch command), and stale files would confuse the next rename.
    if (LittleFS.exists(GW_SPOOL_PART)) LittleFS.remove(GW_SPOOL_PART);
    if (LittleFS.exists(GW_SPOOL_JPG))  LittleFS.remove(GW_SPOOL_JPG);
    // Census + cleanup: microReticulum persists per-transfer resource hashlists under
    // hashlist_store and its failed unlinks ("Has open FD") leak segments, slowly
    // eating the partition the spool needs. Transfer state is worthless across a
    // reboot, so purge it, and print what every top-level dir costs while we're here.
    {
        File root = LittleFS.open("/");
        for (File e = root.openNextFile(); e; e = root.openNextFile()) {
            if (!e.isDirectory()) {
                Serial.printf("[gw] fs: /%s %u B\n", e.name(), (unsigned)e.size());
                continue;
            }
            size_t sum = 0; unsigned n = 0;
            String dir = String("/") + e.name();
            File d = LittleFS.open(dir);
            for (File c = d.openNextFile(); c; c = d.openNextFile()) { sum += c.size(); n++; }
            d.close();
            Serial.printf("[gw] fs: %s/ %u files, %u B\n", dir.c_str(), n, (unsigned)sum);
            if (dir == "/hashlist_store") {
                File d2 = LittleFS.open(dir);
                for (File c = d2.openNextFile(); c; c = d2.openNextFile()) {
                    String path = dir + "/" + c.name();
                    c.close();
                    LittleFS.remove(path);
                }
                d2.close();
                Serial.printf("[gw] fs: purged %u stale hashlist segment(s)\n", n);
            }
        }
        root.close();
    }
    Serial.printf("[gw] littlefs: %u / %u bytes used\n",
                  (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());

    lora_interface = new LoRaInterface();
    // ADR: resume listening on the last profile we granted the leaf (NVS). If the leaf
    // actually lost its grant while we were down, the quiet-leaf scan in rf_loop()
    // finds it back at base within a couple of announce intervals.
    {
        Preferences p;
        p.begin("gw-rf", false);
        uint8_t idx = p.getUChar("idx", LORA_PROFILE_BASE);
        p.end();
        if (idx >= LORA_PROFILE_COUNT) idx = LORA_PROFILE_BASE;
        s_rf.current = s_rf.granted = s_rf.prev = idx;
        if (idx != LORA_PROFILE_BASE && LoRaInterface::active) {
            LoRaInterface::active->set_profile(idx);   // staged; start() applies it
            // Bug 2 (2026-07-16): the NVS grant is a HINT, not the truth — the leaf
            // may have reverted (TTL / power loss) while we were down, and trusting
            // it blindly violated "every failure path converges to base". Boot camped
            // on it but in SCANNING state: a direct announce adopts whatever profile
            // it arrives on; 90 s of direct silence starts the granted<->base toggle.
            // No ADR grants fire until the leaf is confirmed (scanning gates them).
            s_rf.scanning = true;
            Serial.printf("[gw] ADR: resuming on persisted profile %s — UNCONFIRMED, "
                          "scan-first until a direct announce adopts it\n", rf_name(idx));
        }
    }
    lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
    RNS::Transport::register_interface(lora_interface);
    // Radio-optional boot: on the T-Internet-POE the SX1262 is hand-wired and may not
    // be attached yet — the gateway must still come up on the wire (ethernet + OTA +
    // identity), so a failed radio init is loud but not fatal. The interface stays
    // offline (send/loop no-op) until a reboot with the radio present.
    s_radio_ok = lora_interface.start();
    if (!s_radio_ok)
        Serial.println("[gw] *** LoRa radio NOT initialized (absent or miswired) — "
                       "running network-only; wire the SX1262 and reboot ***");

#ifdef GW_NET_ETH
    // RNS-over-UDP on the wired LAN: lets Python RNS peers (rnsd, the bench host)
    // reach the mesh through this box — the tower-bridge role. Registered now,
    // started on first link-up in loop() (the lwIP socket wants an interface).
    udp_interface = new UDPInterface();
    udp_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
    RNS::Transport::register_interface(udp_interface);
#endif

    reticulum = RNS::Reticulum();
    reticulum.transport_enabled(true);
    reticulum.start();

    RNS::Identity id = load_or_create_identity();
    gw_destination = RNS::Destination(id, RNS::Type::Destination::IN,
                                      RNS::Type::Destination::SINGLE, APP_NAME, ASPECT);
    gw_destination.accepts_links(true);
    gw_destination.set_link_established_callback(on_link_established);
    Serial.printf("[gw] GATEWAY destination hash: %s\n",
                  gw_destination.hash().toHex().c_str());
    Serial.println("[gw] ^ bake into leaves as LEAF_GATEWAY_DEST_HEX — persistent, never rotates");

    // Watch for the leaf's announces: each one marks an awake window we can push
    // pending commands into (the mesh downlink's wake-triggered delivery).
    static RNS::HAnnounceHandler h(new LeafAnnounceHandler());
    RNS::Transport::register_announce_handler(h);

    s_web.on("/", web_root);
    // Site hardware swap (bug 7): empty the roster so subsequent direct health
    // announces re-enroll. POST-only — a crawler's GET must not unpin a field gateway.
    s_web.on("/pin/clear", HTTP_POST, []() {
        clear_roster();
        s_web.send(200, "text/plain", "leaf roster cleared — TOFU re-armed\n");
    });
    // Commissioning aid (gateway-0.10.0): park the site radio on BASE. Adding camera
    // N+1 to a site holding a fast standing grant is otherwise a deadlock — the new
    // leaf announces on base where we never listen (the multi-leaf walk-back only
    // engages once it's ENROLLED, and enrollment needs it heard). POST here when
    // mounting a new camera; the granted leaf self-reverts within 3 contactless
    // wakes and the site re-converges (multi-leaf: stays on base; single-leaf: ADR
    // simply re-grants once announces resume).
    s_web.on("/rf/base", HTTP_POST, []() {
        rf_retune(LORA_PROFILE_BASE);
        s_rf.granted  = LORA_PROFILE_BASE;
        s_rf.pending  = 0xFF;
        s_rf.scanning = false;
        rf_persist(LORA_PROFILE_BASE);
        rf_clear_snr();
        Serial.println("[gw] rf/base: parked on base (operator commissioning)");
        log_event("operator parked radio on base");
        s_web.send(200, "text/plain", "radio parked on base\n");
    });
    // s_web.begin() happens on first network-up in loop(): binding before the
    // interface exists leaves the listener dead (connection refused).

    net_setup();
}

void loop() {
    reticulum.loop();
    s_web.handleClient();
    ArduinoOTA.handle();

    net_ensure();
    ntp_ensure();

#ifdef GW_HAS_OLED
    // Refresh the parent-facing screen ~2x/s when idle (slow down during a transfer so
    // the ~25 ms I2C blast never steals from the tight radio loop).
    static uint32_t last_oled = 0;
    if (millis() - last_oled > (radio_busy() ? 3000u : 500u)) {
        last_oled = millis();
        roll_day_if_needed();
        oled_render();
    }
#endif

    // One-shot confirmation that NTP actually answered (uploads defer on an unsynced
    // clock, so a silent NTP failure used to only show up as mysterious spooling).
    static bool ntp_logged = false;
    if (!ntp_logged && time(nullptr) > 1600000000) {
        ntp_logged = true;
        Serial.printf("[gw] NTP time acquired: %lld\n", (long long)time(nullptr));
        log_event("NTP time acquired");
    }

    static bool net_was_up = false;
    if (net_up() != net_was_up) {
        net_was_up = net_up();
        if (net_was_up) {
            Serial.printf("[gw] network up (%s), ip=%s rssi=%d\n",
                          net_desc().c_str(), net_ip().toString().c_str(), net_rssi());
            static bool web_started = false;
            if (!web_started) { s_web.begin(); web_started = true; }
            static bool ota_started = false;
            if (!ota_started) { ota_setup(); ota_started = true; }
#ifdef GW_NET_ETH
            static bool udp_started = false;
            if (!udp_started) { udp_interface.start(); udp_started = true;
                                Serial.println("[gw] RNS UDP interface up on the wire (:4242)"); }
#endif
        }
        else {
            Serial.println("[gw] network DOWN");
        }
    }

    // Announce while unlinked so cold leaves can find us (path requests are answered by
    // Transport regardless); quiet once linked — see the half-duplex note up top.
    if (!latest_link) {
        static uint32_t last = 0;
        if (millis() - last > GW_ANNOUNCE_SECS * 1000u) {
            last = millis();
            gw_destination.announce();
        }
    }

    // Keep the pending-command list fresh (the gallery's fetch_full / OTA requests).
    // Deferred while the radio is busy — this is a blocking TLS GET.
    static uint32_t last_poll = 0;
    if (!radio_busy() &&
        net_up() && millis() - last_poll > GW_CMD_POLL_SECS * 1000u) {
        last_poll = millis();
        poll_commands();
    }

    // Refresh the site display name (address) every ~5 min so a rename in the
    // settings UI reaches the OLED. Blocking TLS GET -> same radio-quiet gate.
    static uint32_t last_sitefetch = 0;
    if (!radio_busy() && net_up() &&
        (!last_sitefetch || millis() - last_sitefetch > 300000u)) {
        last_sitefetch = millis();
        fetch_site_name();
    }

    // TOFU enrollment landed inside announce dispatch -> persist it here (no NVS
    // writes from inside packet handling).
    if (s_pin_save_pending) {
        s_pin_save_pending = false;
        save_roster();
    }

    // A roster leaf just announced -> it's awake and listening; fire ITS pending
    // commands NOW, plus a time_sync when due (cold-boot hello, or the periodic
    // drift refresh). Per entry: each leaf wakes on its own schedule.
    for (int i = 0; i < s_roster_n; i++) {
        LeafEntry& L = s_roster[i];
        if (!L.announced) continue;
        L.announced = false;
        const bool direct = L.announce_direct;
        rf_on_leaf_announce(direct);                 // confirm/adopt profile state first
        // Commands + time sync still deliver on relayed announces (RNS routes them
        // through the relay — legitimate in a relay topology); only ADR is direct-only.
        const int cmds_sent = deliver_pending_commands(L);
        maybe_time_sync(L);
        // Grant last: it retunes the radio. Direct announces only (bug 1) — a grant
        // decision keyed off a relayed announce acts on a link we can't even hear.
        if (direct) maybe_grant_profile(L, cmds_sent);
    }

    // A FOREIGN trailcam.leaf announced (bug 7): it gets its probe_ack — probes are
    // the surveyor's whole job, and the walker's post-announce RX window is the same
    // delivery slot commands use — and a one-shot time_sync on "hello" (the surveyor
    // stamps its CSV from us until GPS gets a fix). Deliberately WITHOUT touching
    // the global sync state: the pinned leaf's refresh cadence is not the walker's.
    if (s_foreign_announced) {
        s_foreign_announced = false;
        if (s_foreign_identity && (s_probe_ack[0] || s_foreign_hello)) {
            RNS::Destination f_dest(s_foreign_identity, RNS::Type::Destination::OUT,
                                    RNS::Type::Destination::SINGLE, "trailcam", "leaf");
            if (s_probe_ack[0]) {
                RNS::Packet pkt(f_dest, RNS::Bytes((const uint8_t*)s_probe_ack,
                                                   strlen(s_probe_ack)));
                pkt.send();
                Serial.printf("[gw] probe_ack delivered: %s\n", s_probe_ack);
                s_probe_ack[0] = 0;
            }
            time_t now = time(nullptr);
            if (s_foreign_hello && now > 1600000000) {
                char json[80];
                snprintf(json, sizeof(json),
                         "{\"kind\":\"time_sync\",\"payload\":{\"unix\":%lld}}",
                         (long long)now);
                RNS::Packet pkt(f_dest, RNS::Bytes((const uint8_t*)json, strlen(json)));
                pkt.send();
                Serial.println("[gw] one-shot time sync to foreign hello");
            }
        }
        s_foreign_hello = false;
    }
    rf_loop();   // ADR confirm timeouts + quiet-leaf profile scan
    announce_wedge_watchdog();   // bug 6: replay-guard starvation self-heal

    // A stalled chunked reassembly (leaf died mid-send) shouldn't hold the spool.
    if (asm_active() && millis() - s_asm.last_ms > 10 * 60 * 1000u) {
        Serial.printf("[gw] assembly of %s stalled -> dropped\n", s_asm.event_id);
        asm_reset();
    }

    // Gateway self-beat: the Nodes page should show THIS box's health too.
    static uint32_t last_selfbeat = 0;
    if (net_up() &&
        (millis() - last_selfbeat > 5 * 60 * 1000u || !last_selfbeat)) {
        last_selfbeat = millis();
        emit_self_beat();
    }
    // Beats + uploads are blocking HTTPS — defer both until the channel goes quiet
    // (the beat queue is lossy by design; the upload slot holds until it succeeds).
    if (!radio_busy()) {
        post_cmd_acks();   // bug 8: relay leaf receipt acks (blocking TLS, radio-quiet)
        post_beats();
        if ((s_pending.jpeg || s_pending.spooled) && net_up()) {
            if (upload_pending()) pending_clear();
        }
    }

    // Poll tight while frames are arriving (a 10 ms nap per loop adds up across a
    // windowed transfer's hundreds of frames); relax when the mesh is idle.
    RNS::Utilities::OS::sleep(radio_busy() ? 0.001 : 0.01);
}
