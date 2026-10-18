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

// GENERATED + gitignored (gen_gw_secrets.py, from secrets.ini). WiFi creds, ingest
// URL/token, OTA password. A header rather than -D flags on the build command line:
// as flags they were readable by any user via ps and got echoed verbatim by a
// compiler error on the leaf side on 2026-08-09. Must come before the
// #error guards below, which test these macros.
#include "gw_secrets.h"

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
// Wired-ethernet uplink comes in two flavours, one firmware:
//   GW_NET_ETH        — LAN8720 RMII (LILYGO T-Internet-POE), classic core-2.x ETH API.
//   GW_NET_ETH_W5500  — WIZnet W5500 over SPI (production gateway PCB), core-3.x ETH API.
// Everything above the netif is identical, so GW_NET_HAS_ETH gates the shared plumbing.
#if defined(GW_NET_ETH) || defined(GW_NET_ETH_W5500)
#define GW_NET_HAS_ETH
#endif
// WiFi is compiled in either because it is the ONLY uplink (Heltec) or because the build
// asked for the runtime dual stack (GW_NET_DUALSTACK, the production S3 gateway PCB).
#if !defined(GW_NET_HAS_ETH) || defined(GW_NET_DUALSTACK)
#define GW_NET_HAS_WIFI
#endif
// SoftAP + captive portal provisioning: the consumer front-end onto the
// credential store AND the claim flow. Compiled into every build that HAS WiFi:
// the production dual-stack gateway PCB, which is the SKU this exists for, and the Heltec
// WiFi boards, where it is what makes the thing testable on hardware that exists today.
// An ethernet-only PoE build has no WiFi at all, so it is excluded automatically and
// never pays for this. -DGW_NO_PROVISION_PORTAL opts a build out explicitly, for a
// sealed tower unit that should not be ABLE to raise an AP.
#if defined(GW_NET_HAS_WIFI) && !defined(GW_NO_PROVISION_PORTAL)
#define GW_HAS_PORTAL
#include <DNSServer.h>       // bundled with arduino-esp32; answers "*" with the AP IP
// Declared up here because things far above the portal itself have to know about it:
// wifi_ensure() must not drop the AP when it rotates the STA, and web_root() hands "/"
// to the setup form while the box is unprovisioned.
static bool s_portal_up = false;
static char s_portal_ssid[24] = "";   // "meshcam-gw-XXXX"; console + OLED print it
static void portal_page();
#endif
#ifdef GW_NET_HAS_ETH
#include <ETH.h>            // wired ethernet (LAN8720 RMII or W5500 SPI)
#include <UDPInterface.h>   // vendored (lib/udp_interface): RNS-over-UDP on the wire
#endif
#include <ArduinoOTA.h>     // network firmware updates (espota) over the WG tunnel

#ifndef GW_SITE
#define GW_SITE "home"
#endif
#ifndef GW_FW_VERSION
#define GW_FW_VERSION "gateway-0.22.0"
#endif
#ifndef GW_VIEW_HOST
#define GW_VIEW_HOST "trailcam.example.com"   // family gallery shown on the OLED
#endif
#ifndef GW_OTA_PASS
#define GW_OTA_PASS "trailcam-ota"     // real value injected from secrets.ini
#endif
// The ingest URL and token used to be mandatory at compile time. With the claim flow
// (homelab docs/trailcam/gateway-provisioning.md) both can instead arrive at
// RUNTIME and live in NVS, so the baked macros are now the FALLBACK and may be absent —
// the same relaxation the WiFi guard below went through for field provisioning.
// -DGW_NO_CLAIM restores the old contract for a build that must not carry the claim
// path (a flash-squeezed env): there the baked values are the only source again.
#ifdef GW_NO_CLAIM
#if !defined(GW_INGEST_URL) || !defined(GW_INGEST_TOKEN)
#error "GW_INGEST_URL / GW_INGEST_TOKEN required with -DGW_NO_CLAIM — see secrets.ini.example"
#endif
#else
#ifndef GW_INGEST_URL
#define GW_INGEST_URL ""
#endif
#ifndef GW_INGEST_TOKEN
#define GW_INGEST_TOKEN ""
#endif
#endif
#if !defined(GW_NET_HAS_ETH) && !defined(GW_NET_HAS_WIFI)
#error "no uplink compiled in — define GW_NET_ETH, GW_NET_ETH_W5500 and/or GW_NET_DUALSTACK"
#endif
// A build with NO way to acquire credentials in the field must bake them. Two things
// count as a way: ethernet (needs no credential at all; the WiFi half gets provisioned
// later, see net_creds_* below), and the provisioning portal, which is precisely a
// credential source. A WiFi-only build with neither would boot with an empty list and
// sit in PROVISION forever with nothing listening, so that one is still an error.
#if defined(GW_NET_HAS_WIFI) && !defined(GW_NET_HAS_ETH) && !defined(GW_HAS_PORTAL) && \
    (!defined(GW_WIFI_SSID) || !defined(GW_WIFI_PSK))
#error "GW_WIFI_SSID / GW_WIFI_PSK required for the WiFi build — see secrets.ini.example"
#endif

// --- claim block ------------------------------------------------------------------------
// WHO THIS GATEWAY UPLOADS AS. A claimed unit carries its site slug, ingest URL and
// per-unit device token in NVS ("gateway" namespace, beside the WiFi credential and the
// RNS identity); every use below goes through the gw_site()/gw_url()/gw_token()
// accessors, which fall back to the baked macros when no claim block exists. So an
// unclaimed unit behaves byte-identically to every build before gateway-0.16.0, and a
// claimed one ignores the macros entirely. Keys:
//   claim_code  the 10 normalized code symbols (kept: RECLAIMING re-authenticates
//               with it unattended after a token revocation)
//   claim_site  site slug                     claim_url  full ingest URL
//   claim_tok   per-unit device token         claim_tid  token id, for revocation
// Written by the claim client (step 7: portal / serial console / LAN /setup), cleared
// by factory reset — which must NEVER clear rns_prv, the identity is the second claim
// factor and the device's serial number to the whole mesh. NVS is not secure storage
// and the design doc does not pretend otherwise: the token is scoped to one tenant's
// ingest and revocable from the app in one click.
#ifndef GW_NO_CLAIM
static char s_claim_code[11] = "";   // 10 Crockford symbols + NUL
static char s_claim_site[65] = "";   // control-plane slug limit is 64
static char s_claim_url[129] = "";   // https://<slug>.getmeshcam.com/api/v1/ingest
static char s_claim_tok[81]  = "";   // "tc_" + token_urlsafe(32) = 46 today, room to grow
static char s_claim_tid[41]  = "";   // UUID, 36

static void claim_load() {
    Preferences p;
    if (!p.begin("gateway", /*readOnly=*/true)) return;   // namespace not created yet
    p.getString("claim_code", s_claim_code, sizeof(s_claim_code));
    p.getString("claim_site", s_claim_site, sizeof(s_claim_site));
    p.getString("claim_url",  s_claim_url,  sizeof(s_claim_url));
    p.getString("claim_tok",  s_claim_tok,  sizeof(s_claim_tok));
    p.getString("claim_tid",  s_claim_tid,  sizeof(s_claim_tid));
    p.end();
    if (s_claim_site[0])
        Serial.printf("[gw] claim block: site \"%s\", token %s\n", s_claim_site,
                      s_claim_tok[0] ? "present" : "MISSING");
}

static void claim_clear() {
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    p.remove("claim_code"); p.remove("claim_site"); p.remove("claim_url");
    p.remove("claim_tok");  p.remove("claim_tid");
    p.end();
    s_claim_code[0] = s_claim_site[0] = s_claim_url[0] = 0;
    s_claim_tok[0] = s_claim_tid[0] = 0;
}

static const char* gw_site()  { return s_claim_site[0] ? s_claim_site : GW_SITE; }
static const char* gw_token() { return s_claim_tok[0]  ? s_claim_tok  : GW_INGEST_TOKEN; }
static const char* gw_url()   { return s_claim_url[0]  ? s_claim_url  : GW_INGEST_URL; }
#else
static inline const char* gw_site()  { return GW_SITE; }
static inline const char* gw_token() { return GW_INGEST_TOKEN; }
static inline const char* gw_url()   { return GW_INGEST_URL; }
#endif

// An unprovisioned unit (no claim block AND no baked secrets) has nowhere to upload.
// Every HTTP path early-returns on this rather than hammering timeouts against an
// empty URL; frames keep queueing to the SD until a claim lands.
static inline bool gw_provisioned() { return gw_url()[0] != '\0' && gw_token()[0] != '\0'; }

// Filled at boot from the RNS identity: the two values kitting reads off this console
// in one step (meshcam-unit register --pubkey <key> --expect <id>). The key is
// X25519(32) then Ed25519(32), 64 bytes; the device id is the first 8 hex of the
// identity hash, exactly what the claim control plane derives from the key.
static char s_pubkey_hex[129] = "";
static char s_device_id[12]   = "";
static char s_identity_hash[33] = "";

#ifndef GW_NO_CLAIM
// The claim client itself lives in claim_client.inc (included after the JSON helpers
// it parses with); these are the seams the console and the upload paths call through.
static RNS::Identity s_identity({RNS::Type::NONE});   // set in setup(); signs the claim
static bool claim_start(const char* entered);
static void claim_tick();
static void claim_note_ingest_401();
static void claim_note_ingest_ok();
static void claim_console_status();
static void claim_forget();
#endif

// --- network abstraction ----------------------------------------------------------------
// The uplink is wired ethernet (T-Internet-POE tower), WiFi (Heltec), or — on the
// production S3 gateway PCB — BOTH, with the box choosing at RUNTIME. One SKU has to cover
// "window unit on the household WiFi" and "sealed mast unit on PoE", so transport cannot be
// a compile-time fact any more.
//
// What IS still compile-time is which interfaces physically exist (a Heltec has no PHY; the
// T-Internet-POE's WiFi is unused). What is runtime is WHICH ONE IS CARRYING TRAFFIC. On a
// single-transport build the state machine in net_ensure() collapses to exactly the old
// behaviour — the only interface either comes up or doesn't.
//
// Nothing above the netif changes: TLS ingest, OTA, NTP, RNS-over-UDP and the web server
// all ride lwIP, which does not care which interface owns the route. All any of them needs
// is these four accessors answering for whichever interface is active right now.
enum NetIface : uint8_t { NET_IFACE_NONE = 0, NET_IFACE_ETH, NET_IFACE_WIFI };
static volatile NetIface s_net_active = NET_IFACE_NONE;
#ifdef GW_NET_HAS_ETH
static volatile bool s_eth_got_ip = false;   // set from the link-event callback
#endif

static bool net_up() {
    switch (s_net_active) {
#ifdef GW_NET_HAS_ETH
        case NET_IFACE_ETH:  return s_eth_got_ip;
#endif
#ifdef GW_NET_HAS_WIFI
        case NET_IFACE_WIFI: return WiFi.status() == WL_CONNECTED;
#endif
        default: return false;
    }
}
static IPAddress net_ip() {
    switch (s_net_active) {
#ifdef GW_NET_HAS_ETH
        case NET_IFACE_ETH:  return ETH.localIP();
#endif
#ifdef GW_NET_HAS_WIFI
        case NET_IFACE_WIFI: return WiFi.localIP();
#endif
        default: return IPAddress((uint32_t)0);
    }
}
// Wired has no meaningful RSSI. 0 is deliberate rather than a sentinel: it reads as
// "full signal" to draw_wifi_icon()'s thresholds, which is the truthful rendering.
static int net_rssi() {
#ifdef GW_NET_HAS_WIFI
    if (s_net_active == NET_IFACE_WIFI) return (int)WiFi.RSSI();
#endif
    return 0;
}
// Goes out in the health beat's extra.wifi_ssid, so it must say WHICH uplink is live —
// that field is how a remote gateway's transport is diagnosed without a serial cable.
static String net_desc() {
    switch (s_net_active) {
#ifdef GW_NET_HAS_ETH
        case NET_IFACE_ETH:  return String("ethernet ") + (s_eth_got_ip ? ETH.linkSpeed() : 0) + "M";
#endif
#ifdef GW_NET_HAS_WIFI
        case NET_IFACE_WIFI: return WiFi.SSID();
#endif
        default: return String("down");
    }
}

static const char* APP_NAME = "trailcam_gatea";   // leaves build their OUT destination
static const char* ASPECT   = "resource";         // from these exact strings — don't drift
#define GW_ANNOUNCE_SECS 30

// --- RNS plumbing ----------------------------------------------------------------------
static RNS::Reticulum reticulum({RNS::Type::NONE});
static RNS::Interface lora_interface({RNS::Type::NONE});
static bool s_radio_ok = false;   // SX1262 present + init'd; false = ethernet/WiFi-only boot
#ifdef GW_NET_HAS_ETH
static RNS::Interface udp_interface({RNS::Type::NONE});   // RNS-over-UDP on the wire
static bool s_udp_started = false;   // has udp_interface.start() ever run?
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
// begin() binds the listener, so it must not run before an interface exists (binding
// early leaves it dead, connection refused) and must not run twice. Two callers: the
// first network-up in loop(), and the provisioning portal, which serves its form over
// the SoftAP while net_up() is still false.
static bool s_web_started = false;
static void web_begin_once() { if (!s_web_started) { s_web.begin(); s_web_started = true; } }
// One stylesheet for every setup-facing page (the SoftAP portal and the LAN /setup claim
// form) so they read as the same product whichever path the customer lands on.
static const char GW_SETUP_CSS[] PROGMEM =
    "<!doctype html><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>MeshCam gateway setup</title><style>"
    "body{font:16px system-ui,-apple-system,sans-serif;background:#111;color:#ddd;"
    "margin:0;padding:1.5em;max-width:26em}h1{font-size:19px;color:#8f8;margin:0}"
    "label{display:block;margin:1.1em 0 .3em;color:#bbb;font-size:14px}"
    "input,select,button{font:inherit;width:100%;box-sizing:border-box;padding:.6em;"
    "border-radius:6px;border:1px solid #444;background:#1b1b1b;color:#eee}"
    "button{margin-top:1.5em;background:#2a7;border:0;color:#04120a;font-weight:600}"
    "p{color:#999;font-size:13px;line-height:1.5}.e{color:#f88}a{color:#8bf}"
    "b{color:#ddd}</style>";
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
    uint32_t base_heard_ms   = 0;        // last direct announce heard while WE sat on base
    bool     shep_wait       = false;    // walk-back shepherd still owes this leaf
    volatile uint32_t announce_ms = 0;   // millis() when dispatch set `announced` (send-delay probe)
    uint32_t last_sync_ms    = 0;        // per-leaf time_sync cadence (bug 10)
    bool     synced_once     = false;
    bool     sync_urgent     = false;    // profile adopt/change -> resync (bug 10)
    // Witness evidence for the TOFU auto re-arm: how many DIRECT announces
    // carrying a health tail this entry has produced since boot, and when the first of
    // them landed. Per entry on purpose — a single global counter was reset to 1 by
    // every announce from a different leaf, so two live cameras at one site knocked
    // each other's evidence down and the rule could never fire where it matters most.
    uint8_t  dh_n            = 0;
    uint32_t dh_first_ms     = 0;
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

// --- TOFU auto re-arm knobs + state — the policy and the argument for these
// bounds are at tofu_rearm_scan() further down; they sit here because web_root()
// renders them and it is defined long before that.
#ifndef GW_TOFU_REARM_GRACE_MS
#define GW_TOFU_REARM_GRACE_MS      (30UL * 60UL * 1000UL)   // bench: -DGW_TOFU_REARM_GRACE_MS=120000
#endif
#ifndef GW_TOFU_REARM_MIN_SPAN_MS
#define GW_TOFU_REARM_MIN_SPAN_MS   (10UL * 60UL * 1000UL)   // bench: -DGW_TOFU_REARM_MIN_SPAN_MS=60000
#endif
#ifndef GW_TOFU_REARM_MIN_HEARD
#define GW_TOFU_REARM_MIN_HEARD     3
#endif
#ifndef GW_TOFU_REARM_COOLDOWN_MS
#define GW_TOFU_REARM_COOLDOWN_MS   (6UL * 3600UL * 1000UL)
#endif
#ifndef GW_TOFU_REARM_MAX_PER_BOOT
#define GW_TOFU_REARM_MAX_PER_BOOT  2
#endif

// Witness evidence from a NON-roster leaf. Roster members carry their own counters
// (LeafEntry::dh_n above); this is the one extra slot a stranger gets, because on a
// single-camera site the replacement leaf is foreign at first and is the only witness
// that will ever exist there. One slot only: two strangers trading announces displace
// each other, which is exactly the case where neither should qualify.
static uint8_t  s_live_hash[16]  = {0};
static uint8_t  s_live_heard     = 0;      // direct+health announces from s_live_hash
static uint32_t s_live_first_ms  = 0;      // millis() of the first of them
static bool     s_live_valid     = false;
static uint8_t  s_rearms_done    = 0;      // re-arms performed this boot
static uint32_t s_last_rearm_ms  = 0;

static void log_event(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_evt[s_evt_head], sizeof(s_evt[0]), fmt, ap);
    va_end(ap);
    s_evt_ms[s_evt_head] = millis();
    s_evt_head = (s_evt_head + 1) % GW_EVT_SLOTS;
}

// Custody revocation, defined with the dedup ring further down; the SD queue below needs
// it when it evicts or permanently drops a frame.
static void have_unnote(const char* eid, const char* kind);

// --- microSD spool queue (gateway-0.15.0) -----------------------------------------------
// Why this exists: the upload slot above is DEPTH 1, and until 0.15.0 that meant any
// uplink outage longer than one capture interval DESTROYED every frame but the newest —
// the queue-full path evicted the older pending frame, and thumbs and fulls shared the
// single slot, so a 5 KB thumb could evict a 135 KB full mid-retry. However long the
// outage, the server got ONE frame instead of a backlog. That is data loss, not latency,
// and it contradicted the store-and-forward promise the product is sold on.
//
// The flash spool cannot be grown to fix it: partitions_ota_4mb.csv allocates all
// 4.000 MB (nvs + otadata + app0 + app1 + spiffs) with ZERO bytes free, and the only
// slack is an OTA slot — i.e. the rollback that keeps a bad flash from bricking a node
// 20 ft up a tree. So depth lives on the T-Internet-POE's TF slot instead: SD_MMC in
// 1-BIT mode (CLK 14 / CMD 15 / D0 2) contends with nothing here — ethernet is LAN8720
// RMII, the Core1262 has its own SPI set (33/32/35/4).
//
// Shape:
//   * one file per (event, kind) under /q, named <captured_at as 8 hex>.QT (thumb) or
//     .QF (full). 8.3-safe, and the NAME alone carries both the drain order and the
//     kind — so there is no index file to desync or corrupt, and a scan never has to
//     open a file to decide what to do next.
//   * each file is a JSON header line + '\n' + the bare JPEG. The uploader seeks past
//     the header and streams the body straight off the card; the heap never holds a full.
//   * writes land in /q/TMP.TMP and are ATOMICALLY RENAMED in. A name that isn't
//     8-hex + .QT/.QF is never treated as queued, so a torn write is inert, not a
//     half-frame that uploads as garbage.
//   * drain NEWEST-FIRST, thumb before full at equal capture time (the ingest API's own
//     thumb-first/full-later philosophy), older frames backfilled behind. Delete ONLY
//     after a 2xx; only a rejected FRAME (400/413/415/422) drops. Ingest is idempotent per
//     (event_id, kind), so re-sending after an ambiguous failure is safe.
//   * reboots KEEP the queue. This deliberately REVERSES the old wipe-on-boot: only the
//     partial assembly (TMP.TMP) is worthless across a reboot, completed frames are not.
//   * no card / mount fail / write fail -> fall back to the depth-1 slot above, never
//     crash, and RAISE A VISIBLE ALARM (self-beat field + web ring). Silent degradation
//     is exactly how the depth-1 defect stayed invisible for a month.
#ifdef GW_SD_QUEUE
#include "sd_queue.inc"
#endif   // GW_SD_QUEUE

// --- /debuglog ring (bug 6): RNS NOTICE+ lines and microStore events, cable-free -------
// The 07-15/16 announce wedge dropped announces on branches that logged at DEBUG or not
// at all, and the gateway rarely has a serial cable attached in the field. Patch 5/6
// (patch_microreticulum.py) elevate those branches to NOTICE / USTORE_LOG; this ring
// captures both so `curl /debuglog` reads the story remotely. Sized for ~30 min of
// NOTICE-rate traffic; the interesting lines (wedge gates, store compaction) are rare.
static constexpr int DBGLOG_N = 96, DBGLOG_W = 128;
static char     s_dbglog[DBGLOG_N][DBGLOG_W];
static uint32_t s_dbglog_ms[DBGLOG_N];
static uint32_t s_dbglog_seq = 0;   // total lines ever pushed; slot = seq % DBGLOG_N
static void dbglog_push(const char* line) {
    const uint32_t slot = s_dbglog_seq % DBGLOG_N;
    snprintf(s_dbglog[slot], DBGLOG_W, "%s", line);
    size_t n = strlen(s_dbglog[slot]);
    if (n && s_dbglog[slot][n - 1] == '\n') s_dbglog[slot][n - 1] = 0;
    s_dbglog_ms[slot] = millis();
    s_dbglog_seq++;
}
static void rns_log_cb(const char* msg, RNS::LogLevel level) {
    if (level <= RNS::LOG_NOTICE) dbglog_push(msg);   // CRITICAL..NOTICE only
}
extern "C" void ustore_log_hook(const char* fmt, ...) {   // microStore Log.h patch calls this
    char buf[DBGLOG_W];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    dbglog_push(buf);
}

// --- ADR (adaptive radio profile, gateway-0.4.0; RSSI-driven since 0.12.0) --------------
// The gateway is the ADR decision-maker: it measures the RSSI + SNR of the leaf's direct
// announces and grants profile switches by index into the shared table (lib/lora_interface/
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
    uint8_t  shep_rung = 0xFF;             // walk-back shepherd: vacated rung to
                                           // sweep for the incumbent (0xFF = idle)
    uint32_t shep_start_ms = 0, shep_toggle_ms = 0;
    uint32_t pending_ms = 0, last_leaf_ms = 0, last_switch_ms = 0,
             last_grant_tx_ms = 0, last_scan_ms = 0;
    uint8_t  announces_since_tx = 0;       // announces heard since our last grant TX
    float    snr[5];                       // rolling announce SNR at the CURRENT profile
    float    rssi[5];                      // rolling announce RSSI (same samples)
    uint8_t  snr_n = 0, snr_i = 0;
    uint8_t  up_reverts = 0;               // consecutive up-shift grants never confirmed
    uint32_t up_revert_ms = 0;             // when the last one timed out
};
static RfState s_rf;

// Survey/commissioning hold (gateway-0.13.0): POST /rf/base parks the radio on base
// AND suspends ADR for a window, so the site stays audible on base long enough to
// enroll a new leaf or run a surveyor walk — without it, a single-leaf site re-grants
// itself up within a few announces and goes deaf on base again. 0 = no hold. The
// signed-diff compare survives millis() wrap; rf_loop() clears it on expiry.
#ifndef GW_RF_BASE_HOLD_DEFAULT_MIN
#define GW_RF_BASE_HOLD_DEFAULT_MIN 15
#endif
#define GW_RF_BASE_HOLD_MAX_MIN     240
static uint32_t s_rf_hold_until = 0;
static bool rf_hold_active() {
    return s_rf_hold_until && (int32_t)(s_rf_hold_until - millis()) > 0;
}

// --- Radio test mode -----------------------------------------------------------------
// Reversible cert-bench emission, driven over the local web server (POST /rf/test).
// Transient RAM state layered on the NORMAL firmware: a hard deadline, POST /rf/test/stop,
// or ANY reboot returns the radio to normal mesh operation. WiFi + OTA stay up the whole
// time, so the node is always reachable and OTA-recoverable — this is what makes flashing
// it to a live gateway safe. While active the mesh radio is dedicated to the emission and
// all mesh TX/RX is suspended (loop() guard). See docs/trailcam/fcc-certification.md.
#define GW_RFTEST_MAX_SECS 300          // hard ceiling on a single test window
static uint32_t s_rftest_until = 0;     // deadline millis(); 0 = not testing
static char     s_rftest_desc[80] = "";
static bool rftest_active() { return s_rftest_until != 0; }

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
static void rf_push_link(float snr, float rssi) {
    s_rf.snr[s_rf.snr_i]  = snr;
    s_rf.rssi[s_rf.snr_i] = rssi;
    s_rf.snr_i = (s_rf.snr_i + 1) % 5;
    if (s_rf.snr_n < 5) s_rf.snr_n++;
}
static float rf_avg_snr() {
    float sum = 0;
    for (uint8_t i = 0; i < s_rf.snr_n; i++) sum += s_rf.snr[i];
    return s_rf.snr_n ? sum / s_rf.snr_n : NAN;
}
static float rf_avg_rssi() {
    float sum = 0;
    for (uint8_t i = 0; i < s_rf.snr_n; i++) sum += s_rf.rssi[i];
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
#ifdef GW_HAS_PORTAL
    // An unprovisioned box has no mesh worth showing, and whoever just joined its AP is
    // here to set it up. Phones probe "/" as often as their vendor probe URL, so this is
    // a real entry point into the portal, not just a nicety.
    if (s_portal_up) { portal_page(); return; }
#endif
    String h;
    h.reserve(4600);
    h += F("<!doctype html><meta charset=utf-8><meta http-equiv=refresh content=5>"
           "<title>trailcam gateway</title><style>body{font:14px monospace;background:#111;"
           "color:#cdc;margin:2em}h1{font-size:16px;color:#8f8}table{border-collapse:collapse}"
           "td{padding:2px 10px;border-bottom:1px solid #333}.t{color:#888}</style>"
           "<h1>trailcam gateway — live mesh</h1><p>");
    char line[380];
    char rftag[32] = "";
    if (s_rf.pending != 0xFF) strcpy(rftag, " (confirming)");
    else if (s_rf.scanning)   strcpy(rftag, " (scanning)");
    else if (s_rf.shep_rung != 0xFF)
        snprintf(rftag, sizeof(rftag), " (shepherding %s)", rf_name(s_rf.shep_rung));
    else if (rf_hold_active())
        snprintf(rftag, sizeof(rftag), " (base hold %lum left)",
                 (unsigned long)((s_rf_hold_until - millis()) / 60000u + 1));
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
             rftag,
             LoRaInterface::last_rssi, LoRaInterface::last_snr,
             (unsigned long)s_stats.announces, (unsigned long)LoRaInterface::announces_seen,
             (unsigned long)s_stats.announces_relayed, (unsigned long)s_stats.announces_foreign,
             (unsigned long)s_stats.cmds_delivered,
             (unsigned long)s_stats.resources_ok, (unsigned long)s_stats.resources_fail,
             (unsigned long)s_stats.chunks, (unsigned long)s_stats.uploads_ok,
             s_roster_n, GW_ROSTER_MAX,
             s_roster_n ? "" : " — TOFU armed, first direct health announce pins");
    h += line;
    // TOFU auto re-arm: the operator-visible half. A re-arm removes a pin
    // without anyone asking, so the page says so, and says what the gateway is currently
    // willing to act on.
    {
        char rearm[220];
        if (s_rearms_done) {
            char ago[24];
            human_ago(millis() - s_last_rearm_ms, ago, sizeof(ago));
            snprintf(rearm, sizeof(rearm),
                     "<br><b style=color:#fc8>TOFU auto re-armed %u time(s) this boot, "
                     "last %s</b> — a pin was dropped without a human asking; see the "
                     "event log below", (unsigned)s_rearms_done, ago);
        } else {
            // Best witness evidence on hand, so "why hasn't it fired?" is answerable
            // from the page instead of from the source.
            unsigned best = s_live_valid ? s_live_heard : 0;
            for (int i = 0; i < s_roster_n; i++)
                if (s_roster[i].dh_n > best) best = s_roster[i].dh_n;
            snprintf(rearm, sizeof(rearm),
                     "<br><span class=t>TOFU auto re-arm: armed from %lu min uptime "
                     "(now %lu); best witness %u/%u direct health announces</span>",
                     (unsigned long)(GW_TOFU_REARM_GRACE_MS / 60000UL),
                     (unsigned long)(millis() / 60000UL),
                     best, (unsigned)GW_TOFU_REARM_MIN_HEARD);
        }
        h += rearm;
    }
#ifdef GW_SD_QUEUE
    // The glanceable half of the alarm: no card here means outages lose frames, and this
    // page is what someone standing at the tower with a phone actually looks at.
    if (!s_sdq.mounted) {
        snprintf(line, sizeof(line),
                 "<br><b style=color:#f88>upload queue: NO CARD — DEPTH 1, an outage "
                 "loses every frame but the newest</b> (fit a microSD in the TF slot; "
                 "%lu mount attempt(s) failed)", (unsigned long)s_sdq.mount_fails);
    } else {
        char ago[24] = "-";
        const long age = sdq_oldest_age_s();
        if (age >= 0) human_ago((uint32_t)age * 1000u, ago, sizeof(ago));
        snprintf(line, sizeof(line),
                 "<br>upload queue: %lu on card (%lu thumb / %lu full), oldest %s, "
                 "%lu/%lu MB free, evicted %lu, write/read errs %lu/%lu",
                 (unsigned long)s_sdq.depth, (unsigned long)s_sdq.thumbs,
                 (unsigned long)s_sdq.fulls, ago,
                 (unsigned long)(s_sdq.free_bytes >> 20),
                 (unsigned long)(s_sdq.card_bytes >> 20),
                 (unsigned long)s_sdq.evicted, (unsigned long)s_sdq.write_errors,
                 (unsigned long)s_sdq.read_errors);
    }
    h += line;
#endif
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
// Sized for the two big ones: a hex-dumped announce frame, and the self-beat with a FULL
// 4-leaf roster + the sd_queue block (0.15.0). vsnprintf TRUNCATES silently, and a
// truncated beat is invalid JSON the backend 422s — so keep the headroom honest.
static char s_beats[GW_BEAT_SLOTS][1120];
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
    if (!s_beat_n || !net_up() || !gw_provisioned()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(8000);
    String url = String(gw_url());
    url.replace("/ingest", "/telemetry");
    for (int i = 0; i < s_beat_n; i++) {
        if (!http.begin(tls, url)) break;
        http.addHeader("Authorization", String("Bearer ") + gw_token());
        http.addHeader("Content-Type", "application/json");
        int code = http.POST((uint8_t*)s_beats[i], strlen(s_beats[i]));
#ifndef GW_NO_CLAIM
        if (code == 401) claim_note_ingest_401();
        else if (code >= 200 && code < 300) claim_note_ingest_ok();
#endif
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

// --- transfer dedup ("have?" protocol, gateway-0.14.0) ---------------------------------
// The envelope header travels INSIDE the Resource payload, so the gateway can't refuse
// a duplicate until the whole thing has already burned its airtime. Instead the leaf
// asks first, with a tiny packet over the link: {"q":"have","event_id":..,"kind":..}.
// We answer from a ring of transfers we've taken custody of, plus the live assembly's
// chunk bitmask — so a leaf that lost the RNS conclusion ack (the classic edge-of-range
// shape, 07-17 walk) pays one packet round trip instead of a full re-transfer. RAM-only
// by design: a gateway reboot forgets, the leaf's query times out, and it fails open to
// sending. Custody = queued for upload here; the WiFi leg has its own retry loop.
#define GW_HAVE_RING 12
struct HaveEntry { char event_id[48]; char kind[8]; };
static HaveEntry s_have[GW_HAVE_RING];
static int       s_have_next = 0;

static void have_note(const char* eid, const char* kind) {
    if (!eid || !eid[0]) return;
    for (int i = 0; i < GW_HAVE_RING; i++)
        if (strcmp(s_have[i].event_id, eid) == 0 && strcmp(s_have[i].kind, kind) == 0)
            return;
    snprintf(s_have[s_have_next].event_id, sizeof(s_have[0].event_id), "%s", eid);
    snprintf(s_have[s_have_next].kind,     sizeof(s_have[0].kind),     "%s", kind);
    s_have_next = (s_have_next + 1) % GW_HAVE_RING;
}

static bool have_check(const char* eid, const char* kind) {
    for (int i = 0; i < GW_HAVE_RING; i++)
        if (strcmp(s_have[i].event_id, eid) == 0 && strcmp(s_have[i].kind, kind) == 0)
            return true;
    return false;
}

// Custody REVOKED: a queued upload we dropped never reached the server, and a ring
// that still claims it would make the leaf dedup-skip every server retry — the photo
// would be lost with all three ends convinced someone else had it (0.14.3).
static void have_unnote(const char* eid, const char* kind) {
    for (int i = 0; i < GW_HAVE_RING; i++)
        if (strcmp(s_have[i].event_id, eid) == 0 && strcmp(s_have[i].kind, kind) == 0)
            s_have[i].event_id[0] = 0;
}


// Take ownership of a bare-JPEG buffer + metadata (zero-copy for thumbs).
static void queue_upload_owned(uint8_t* jpeg, size_t len, const char* eid,
                               const char* cam, const char* kind, long long capts) {
#ifdef GW_SD_QUEUE
    // Deep queue first: the card holds hundreds of frames, so an outage costs nothing
    // but latency. Only a missing or failing card falls through to the depth-1 slot.
    if (s_sdq.mounted && sdq_enqueue(eid, cam, kind, capts, jpeg, nullptr, len)) {
        free(jpeg);
        note_capture(cam, eid);
        have_note(eid, kind);
        return;
    }
#endif
    if (s_pending.jpeg || s_pending.spooled) {
        Serial.println("[gw] upload queue full -> DROPPING the older pending frame");
        have_unnote(s_pending.event_id, s_pending.kind);
        pending_clear();
    }
    s_pending.jpeg = jpeg;
    s_pending.len  = len;
    snprintf(s_pending.event_id, sizeof(s_pending.event_id), "%s", eid);
    snprintf(s_pending.camera,   sizeof(s_pending.camera),   "%s", cam);
    snprintf(s_pending.kind,     sizeof(s_pending.kind),     "%s", kind);
    s_pending.captured_at = capts;
    note_capture(cam, eid);
    have_note(eid, kind);
}

// File-backed variant: the body already sits at GW_SPOOL_JPG (a completed assembly);
// the uploader streams it from flash.
static void queue_upload_spooled(size_t len, const char* eid,
                                 const char* cam, long long capts) {
#ifdef GW_SD_QUEUE
    // Hand the completed full over to the card and give the flash spool straight back —
    // assembly stays on LittleFS (that short-handle chunk-write dance is hard-won; see
    // spool_write_chunk) and only the finished frame is copied across, once.
    if (s_sdq.mounted) {
        File src = LittleFS.open(GW_SPOOL_JPG, FILE_READ);
        const bool ok = src && src.size() == len &&
                        sdq_enqueue(eid, cam, "full", capts, nullptr, &src, len);
        if (src) src.close();
        if (ok) {
            LittleFS.remove(GW_SPOOL_JPG);   // the card owns this frame now
            note_capture(cam, eid);
            have_note(eid, "full");
            return;
        }
    }
#endif
    if (s_pending.jpeg || s_pending.spooled) {
        Serial.println("[gw] upload queue full -> DROPPING the older pending frame");
        have_unnote(s_pending.event_id, s_pending.kind);
        pending_clear();
    }
    s_pending.spooled = true;
    s_pending.len     = len;
    snprintf(s_pending.event_id, sizeof(s_pending.event_id), "%s", eid);
    snprintf(s_pending.camera,   sizeof(s_pending.camera),   "%s", cam);
    snprintf(s_pending.kind,     sizeof(s_pending.kind),     "full");
    s_pending.captured_at = capts;
    note_capture(cam, eid);
    have_note(eid, "full");
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
                       gw_site(), cam[0] ? cam : gw_fallback_camera(), tj);
        else
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\","
                       "\"rssi\":%.0f,\"snr\":%.1f,\"extra\":{\"transfer\":%s}}",
                       gw_site(), cam[0] ? cam : gw_fallback_camera(),
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
                   gw_site(), s_asm.camera[0] ? s_asm.camera : gw_fallback_camera(),
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
                   gw_site(), slug[0] ? slug : "surveyor", hdr,
                   (unsigned long)res_ms, (unsigned)total_len, rf_name(s_rf.current));
    else
        queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"surveyor\","
                   "\"rssi\":%.0f,\"snr\":%.1f,"
                   "\"extra\":{\"probe\":%s,\"ms\":%lu,\"bytes\":%u,\"profile\":\"%s\"}}",
                   gw_site(), slug[0] ? slug : "surveyor", rssi, snr, hdr,
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
                   gw_site(), gw_site(), (int)resource.status());
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

// "have?" query arriving as a small packet over the link (see the dedup block above).
// The reply is STAGED for loop(), never sent from here: this callback runs inside
// inbound dispatch, and TX from dispatch context is the exact re-entrancy the
// microReticulum outbound/inbound patch exists for — the send gets eaten (verified
// on air 2026-07-18: leaf logged "have? no reply" twice while the ring had the
// event). Same defer-to-loop() pattern as the probe_ack.
static char s_have_reply_out[160] = "";
static void on_link_packet(const RNS::Bytes& plaintext, const RNS::Packet& /*packet*/) {
    char q[16] = "", eid[48] = "", kind[8] = "";
    std::string js((const char*)plaintext.data(),
                   std::min(plaintext.size(), (size_t)200));
    json_str(js.c_str(), "q", q, sizeof(q));
    if (strcmp(q, "have") != 0) return;   // not ours; other link packets stay possible
    json_str(js.c_str(), "event_id", eid, sizeof(eid));
    json_str(js.c_str(), "kind", kind, sizeof(kind));
    if (!eid[0] || !kind[0]) return;
    const bool complete = have_check(eid, kind);
    uint32_t mask = 0;
    unsigned chunks = 0;
    if (!complete && strcmp(kind, "full") == 0 && asm_active() &&
        strcmp(s_asm.event_id, eid) == 0) {
        mask   = s_asm.got_mask;
        chunks = s_asm.chunks;
    }
    snprintf(s_have_reply_out, sizeof(s_have_reply_out),
             "{\"a\":\"have\",\"event_id\":\"%s\",\"kind\":\"%s\","
             "\"complete\":%d,\"got_mask\":%lu,\"chunks\":%u}",
             eid, kind, complete ? 1 : 0, (unsigned long)mask, chunks);
    Serial.printf("[gw] have? %s %s -> %s%s (reply staged)\n", eid, kind,
                  complete ? "COMPLETE (dedup save)" : "miss",
                  mask ? " (partial mask)" : "");
    log_event("have? %.12s %s -> %s", eid, kind, complete ? "HIT (dedup)" : "miss");
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
               gw_site(), gw_site());
    latest_link = link;
    link.set_resource_strategy(RNS::Type::Link::ACCEPT_ALL);
    link.set_resource_started_callback(on_resource_started);
    link.set_resource_concluded_callback(on_resource_concluded);
    link.set_packet_callback(on_link_packet);
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

// --- automatic TOFU re-arm for a pin that has gone dead ---------------------------------
// A full `esptool write-flash 0x0` can wipe the leaf's NVS, and it comes back with a NEW
// RNS identity — so a new destination hash AND a new slug, because the slug is derived
// from the hash (leaf_radio.cpp: "leaf-%.8s"; no env defines LEAF_NODE_SLUG). The pin we
// hold then names a leaf that will never speak again, and until a human POSTs /pin/clear
// the entry sits in the roster forever: it holds a slot, it keeps advertising a phantom
// camera, and on a full roster it locks the replacement out as FOREIGN entirely. That case
// produced THREE identities from one physical board in an afternoon, so a 4-slot roster
// reaches that deadlock after three reflashes of a single camera.
//
// WHY NOT THE ORIGINALLY PROPOSED BOUND. The first draft suggested "pinned identity never heard
// direct this boot AND a new identity announces on the SAME slug". The same-slug conjunct
// cannot ever be true in the scenario it was written for: the slug IS the first 8 hex of
// the destination hash, so a regenerated identity always announces a different one. The
// rule would never fire. It becomes the right rule the day slugs are operator-assigned
// and survive a reflash; it is not the right rule today.
//
// The first conjunct is kept, but it is NECESSARY, not sufficient. "Never heard direct
// this boot" is true of EVERY roster entry for the first seconds after a gateway reboot —
// including the reboot this gateway's own announce-wedge watchdog performs — so on its own
// it is most permissive exactly when the gateway knows least. Hence the extra bounds:
//
//   1. GRACE     — no re-arm in the first GW_TOFU_REARM_GRACE_MS of uptime. A leaf wakes
//                  every few minutes; 30 min gives every live member many chances to prove
//                  itself direct before we are willing to forget anything.
//   2. SILENT    — the victim must be last_direct_ms == 0, i.e. not heard direct ONCE in
//                  that whole window. A relayed announce does not save it, deliberately:
//                  a relayed frame proves a radio somewhere, not this pin at this site.
//   3. CORROBORATION — some OTHER trailcam.leaf identity must be provably alive here:
//                  GW_TOFU_REARM_MIN_HEARD direct announces WITH a health tail, from one
//                  hash, spanning at least GW_TOFU_REARM_MIN_SPAN_MS. This is the load-
//                  bearing guard. It means we never forget a pin because WE went deaf —
//                  a wedged radio, a snapped antenna and a dead site all fail this test,
//                  and in every one of those the roster survives untouched.
//   4. RATE      — one entry per event, oldest-silent first, at most
//                  GW_TOFU_REARM_MAX_PER_BOOT per boot and GW_TOFU_REARM_COOLDOWN_MS
//                  apart, so nobody walks the whole roster out in an afternoon.
//
// BLAST RADIUS, stated plainly, because this weakens TOFU on purpose:
//   * When a slot is FREE this grants an attacker nothing new. Enrollment is already open
//     for the roster's lifetime (that is how camera N+1 self-commissions), so a direct
//     health announce already claims a slot today. Re-arm only garbage-collects the ghost.
//   * When the roster is FULL it does grant something new: today a full roster refuses
//     every newcomer, and after this a newcomer that holds the channel for
//     GW_TOFU_REARM_MIN_SPAN_MS can displace a pin that has been silent for the whole
//     grace window. The cost of that attack is: be within direct radio range of the site,
//     emit a well-formed trailcam.leaf announce with a health tail N times over 10 minutes,
//     and wait for one of the site's real cameras to already be off the air for 30 minutes.
//     The reward is one roster slot — which buys command delivery for a camera slug that
//     the SERVER must also already be addressing, and it is loudly logged, evented and
//     beaten to the cloud at the moment it happens.
//   * What it can NEVER do: evict an entry heard direct this boot, evict more than one
//     entry per cooldown, or fire at all while the gateway's own receive path is broken.
//
// NOT COVERED, and the named-exit patch is the only defence: a leaf whose hash is UNCHANGED but whose
// key material diverged (NVS identity survived, filesystem ratchet state did not). That
// entry keeps announcing, keeps being heard direct, and keeps failing to decrypt. No
// liveness rule can see it; only the leaf's own decrypt-failure line can.
//
// The knobs and state these functions read sit up beside the roster definition, because
// web_root() renders them and is defined long before this point.

// Called from the announce handler for every DIRECT announce carrying a health tail —
// the same test TOFU enrollment itself uses, so nothing weaker can ever corroborate.
static void tofu_note_live(const RNS::Bytes& destination_hash) {
    if (destination_hash.size() != sizeof(s_live_hash)) return;
    for (int i = 0; i < s_roster_n; i++) {
        if (memcmp(destination_hash.data(), s_roster[i].pin,
                   sizeof(s_roster[i].pin)) != 0) continue;
        if (!s_roster[i].dh_n) s_roster[i].dh_first_ms = millis();
        if (s_roster[i].dh_n < 255) s_roster[i].dh_n++;
        return;
    }
    if (s_live_valid && memcmp(destination_hash.data(), s_live_hash,
                               sizeof(s_live_hash)) == 0) {
        if (s_live_heard < 255) s_live_heard++;
        return;
    }
    memcpy(s_live_hash, destination_hash.data(), sizeof(s_live_hash));
    s_live_heard    = 1;
    s_live_first_ms = millis();
    s_live_valid    = true;
}

// Is some leaf OTHER than `victim_pin` provably alive and audible right now? This is the
// load-bearing guard: it is what stops the gateway forgetting a pin because IT went deaf.
// A wedged radio, a snapped antenna and a dead site all fail it, and in every one of those
// the roster survives untouched. Fills `out_hash` with the witness on success.
static bool tofu_witness_ok(const uint8_t* victim_pin, uint8_t* out_hash,
                            uint8_t* out_heard, uint32_t* out_span_ms) {
    for (int i = 0; i < s_roster_n; i++) {
        if (memcmp(s_roster[i].pin, victim_pin, sizeof(s_roster[i].pin)) == 0) continue;
        if (s_roster[i].dh_n < GW_TOFU_REARM_MIN_HEARD) continue;
        if (millis() - s_roster[i].dh_first_ms < GW_TOFU_REARM_MIN_SPAN_MS) continue;
        memcpy(out_hash, s_roster[i].pin, sizeof(s_roster[i].pin));
        *out_heard   = s_roster[i].dh_n;
        *out_span_ms = millis() - s_roster[i].dh_first_ms;
        return true;
    }
    if (s_live_valid && memcmp(s_live_hash, victim_pin, sizeof(s_live_hash)) != 0 &&
        s_live_heard >= GW_TOFU_REARM_MIN_HEARD &&
        millis() - s_live_first_ms >= GW_TOFU_REARM_MIN_SPAN_MS) {
        memcpy(out_hash, s_live_hash, sizeof(s_live_hash));
        *out_heard   = s_live_heard;
        *out_span_ms = millis() - s_live_first_ms;
        return true;
    }
    return false;
}

// Runs in loop(), never in packet dispatch: it rewrites the roster array and defers its
// NVS write the same way enrollment does. Returns true if an entry was dropped.
static bool tofu_rearm_scan() {
    if (!s_roster_n) return false;
    if (millis() < GW_TOFU_REARM_GRACE_MS) return false;
    if (s_rearms_done >= GW_TOFU_REARM_MAX_PER_BOOT) return false;
    if (s_last_rearm_ms && millis() - s_last_rearm_ms < GW_TOFU_REARM_COOLDOWN_MS)
        return false;

    int      victim = -1;
    uint8_t  whash[16], wheard = 0;
    uint32_t wspan = 0;
    for (int i = 0; i < s_roster_n; i++) {
        if (s_roster[i].last_direct_ms != 0) continue;   // heard direct -> alive, leave it
        if (!tofu_witness_ok(s_roster[i].pin, whash, &wheard, &wspan)) continue;
        victim = i;                                      // first silent entry wins
        break;
    }
    if (victim < 0) return false;

    char vhex[33], vslug[sizeof(s_roster[0].slug)], whex[9] = "????????";
    snprintf(vhex,  sizeof(vhex),  "%s", s_roster[victim].hex);
    snprintf(vslug, sizeof(vslug), "%s", s_roster[victim].slug);
    for (int i = 0; i < 4; i++) sprintf(whex + i * 2, "%02x", whash[i]);
    const bool was_full = (s_roster_n >= GW_ROSTER_MAX);

    // Forget the pin, and the RNS path that goes with it — the pin IS the trailcam.leaf
    // destination hash, so this is the same handle the announce-wedge watchdog drops.
    {
        RNS::Bytes h(s_roster[victim].pin, sizeof(s_roster[victim].pin));
        if (RNS::Transport::has_path(h)) RNS::Transport::remove_path(h);
    }
    for (int i = victim; i < s_roster_n - 1; i++) s_roster[i] = s_roster[i + 1];
    s_roster[s_roster_n - 1] = LeafEntry();
    s_roster_n--;
    s_rearms_done++;
    s_last_rearm_ms    = millis();
    s_pin_save_pending = true;      // NVS write happens in loop(), same as enrollment

    Serial.printf("[gw] TOFU AUTO RE-ARM: dropped pin %s (%s) — never heard direct in "
                  "%lus of uptime, while %s was heard direct+healthy %u times over %lus. "
                  "Roster now %d/%d%s. This WEAKENS TOFU deliberately; "
                  "POST /pin/clear is still the manual path.\n",
                  vhex, vslug, (unsigned long)(millis() / 1000), whex,
                  (unsigned)wheard, (unsigned long)(wspan / 1000),
                  s_roster_n, GW_ROSTER_MAX,
                  was_full ? " (roster had been FULL — this unblocked enrollment)" : "");
    log_event("TOFU auto re-arm: dropped %.8s (%s), witness %s", vhex, vslug, whex);
    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"extra\":{\"leaf_tofu_rearm\":{\"dropped\":\"%s\",\"slug\":\"%s\","
               "\"witness\":\"%s\",\"witness_heard\":%u,\"witness_span_s\":%lu,"
               "\"roster_was_full\":%s,\"roster_n\":%d,\"rearms_this_boot\":%u}}}",
               gw_site(), gw_site(), vhex, vslug, whex, (unsigned)wheard,
               (unsigned long)(wspan / 1000), was_full ? "true" : "false",
               s_roster_n, (unsigned)s_rearms_done);
    return true;
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
    if (!s_cmd_ack_n || !net_up() || !gw_provisioned()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(8000);
    for (int i = 0; i < s_cmd_ack_n; i++) {
        String url = String(gw_url());
        char path[40];
        snprintf(path, sizeof(path), "/commands/%lu/ack", (unsigned long)s_cmd_acks[i]);
        url.replace("/ingest", path);
        if (!http.begin(tls, url)) break;
        http.addHeader("Authorization", String("Bearer ") + gw_token());
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

// Pull a "<key>=<value>" token out of an announce tail. Token-boundary aware: matches
// only at the tail start or right after a space, so a value can never be faked by a
// substring of another token — "tx=20" must not answer a lookup for "x=". Returns false
// if the key is absent (older leaves send shorter tails).
static bool htail_token(const char* htail, const char* key, char* out, size_t cap) {
    const size_t klen = strlen(key);
    for (const char* p = htail; p; p = strchr(p, ' '), p = p ? p + 1 : nullptr) {
        if (strncmp(p, key, klen) != 0) continue;
        p += klen;
        size_t i = 0;
        while (p[i] && p[i] != ' ' && i + 1 < cap) { out[i] = p[i]; i++; }
        out[i] = '\0';
        return i > 0;
    }
    return false;
}

// The node slug ("n=<slug>", leaf-0.13.0).
static bool htail_slug(const char* htail, char* out, size_t cap) {
    return htail_token(htail, "n=", out, cap);
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
        // Corroboration for the TOFU auto re-arm: a direct announce with a
        // health tail is proof that our receive path and this site's radio both work.
        // Recorded for EVERY such announce, roster member or not, and before the roster
        // lookup below — the replacement leaf on a one-camera site is foreign at first
        // and is the only witness that will ever exist there.
        if (direct && has_health) tofu_note_live(destination_hash);
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
                           gw_site(), gw_site(), s_roster[idx].hex, idx);
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
                       gw_site(), gw_site(), fh, status,
                       (unsigned)LoRaInterface::last_hops, opt);
            return;
        }
        LeafEntry& L = s_roster[idx];
        L.identity        = announced_identity;
        L.announce_direct = direct;
        // Empty-app_data announces are real protocol frames — path responses and
        // link handshakes genuinely carry none. (Relayed copies used to lose theirs
        // too, but that was the recall-cache starvation fixed by microReticulum
        // patch 4 — app_data now parses from the frame and survives relay hops.)
        // The leaf IS awake, so delivery below still runs, but don't let them wipe
        // the entry's last-known status/health.
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
        L.announce_ms = millis();   // send-delay probe: dispatch instant
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
        if (direct && !isnan(LoRaInterface::last_snr))
            rf_push_link(LoRaInterface::last_snr, LoRaInterface::last_rssi);
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
        // TX power the leaf says it sent this announce at ("tx=<dbm>", leaf tx-power
        // calibration 2026-07-18) -> extra.tx_dbm, so RSSI rows are attributable to
        // the power that produced them.
        char txkv[24] = "";
        // Leaf full-res store degraded: "sd=n" no card, "sd=u" a card whose
        // filesystem would not mount (an out-of-the-package exFAT card — a FAT32 reformat
        // fixes it, an RMA does not), "sd=?" degraded, cause not separable. The field is
        // ABSENT on a healthy leaf, so its presence alone is the alarm.
        char sdkv[24] = "";
        // Identity-change alarm: "x=<the slug this leaf announced under before
        // its NVS key was lost>" -> extra.prev_node. Absent on a healthy node, so its
        // presence alone is the alarm; the server uses it to merge the split camera row.
        // Bounded burst per power cycle at the leaf, so this is not a steady-state field.
        char prevkv[48] = "";
        if (htail[0]) {
            long p = -1, c = -1, f = -1;
            float vb = NAN;
            const char* t;
            if ((t = strstr(htail, "p=")))  p  = atol(t + 2);
            if ((t = strstr(htail, "c=")))  c  = atol(t + 2);
            if ((t = strstr(htail, "f=")))  f  = atol(t + 2);
            if ((t = strstr(htail, "vb=")))              vb = atof(t + 3);
            if ((t = strstr(htail, "tx=")))
                snprintf(txkv, sizeof(txkv), ",\"tx_dbm\":%ld", atol(t + 3));
            if ((t = strstr(htail, "sd=")) && t[3] && t[3] != ' ')
                snprintf(sdkv, sizeof(sdkv), ",\"sd\":\"%c\"", t[3]);
            char prevslug[24], nowslug[24];
            if (htail_token(htail, "x=", prevslug, sizeof(prevslug))) {
                snprintf(prevkv, sizeof(prevkv), ",\"prev_node\":\"%s\"", prevslug);
                if (!htail_slug(htail, nowslug, sizeof(nowslug)))
                    snprintf(nowslug, sizeof(nowslug), "a leaf");
                Serial.printf("[gw] *** %s reports an IDENTITY CHANGE: it used to be %s "
                              "(its history and its queued commands are split) ***\n",
                              nowslug, prevslug);
            }
            // Only leaves that actually sent the health counters get a health object
            // (a tail can now be just "n=<slug> tx=<dbm>").
            if (p >= 0 || c >= 0 || f >= 0) {
                int n = snprintf(health, sizeof(health),
                                 ",\"health\":{\"pir_wakes\":%ld,\"captures\":%ld,\"push_fails\":%ld",
                                 p, c, f);
                if (!isnan(vb) && n > 0 && n < (int)sizeof(health))
                    n += snprintf(health + n, sizeof(health) - n, ",\"battery_v\":%.2f", vb);
                if (n > 0 && n < (int)sizeof(health)) snprintf(health + n, sizeof(health) - n, "}");
            }
            if (!isnan(vb))
                snprintf(batt, sizeof(batt), ",\"battery_v\":%.2f", vb);
        }
        if (isnan(LoRaInterface::last_rssi))
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\"%s,"
                       "\"extra\":{\"status\":\"%s\",\"via\":\"mesh\",\"hops\":%u%s%s%s%s,"
                       "\"packet\":{\"len\":%u,\"hex\":\"%s\"}}}",
                       gw_site(), L.slug[0] ? L.slug : gw_fallback_camera(), batt, status,
                       (unsigned)LoRaInterface::last_hops, health, txkv, sdkv, prevkv,
                       (unsigned)LoRaInterface::last_frame_len, hexbuf);
        else
            queue_beat("{\"site\":\"%s\",\"node\":\"%s\",\"kind\":\"camera\"%s,"
                       "\"rssi\":%.0f,\"snr\":%.1f,"
                       "\"extra\":{\"status\":\"%s\",\"via\":\"mesh\",\"hops\":%u%s%s%s%s,"
                       "\"packet\":{\"len\":%u,\"hex\":\"%s\"}}}",
                       gw_site(), L.slug[0] ? L.slug : gw_fallback_camera(), batt,
                       LoRaInterface::last_rssi, LoRaInterface::last_snr, status,
                       (unsigned)LoRaInterface::last_hops, health, txkv, sdkv, prevkv,
                       (unsigned)LoRaInterface::last_frame_len, hexbuf);
    }
};

static void poll_commands() {
    if (!net_up() || !gw_provisioned()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(10000);
    String url = String(gw_url());
    url.replace("/ingest", "/commands");
    if (!http.begin(tls, url)) return;
    http.addHeader("Authorization", String("Bearer ") + gw_token());
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

// Strip the server-only fields off a command before it goes on the air.
//
// The /commands contract is server-facing: it carries "site", "node" and "status" for the
// GATEWAY's benefit, and the leaf parses NONE of them (leaf_serial_proto.cpp reads
// kind/event_id/quality/ssid/psk/url/sha256/minutes/confirm, plus "id" for the ack).
// Those spectators cost real air. A maintenance command measured 167 B of JSON, which with
// the ~106 B of RNS overhead measured on board 0016 is ~273 B — past the 255 B PHY frame,
// so it split in two. Measured: the 255 B first part arrives and is buffered for
// reassembly, the second never lands inside the leaf's 2500 ms window, reassembly never
// completes, and the command is lost with no log anywhere. Single-frame traffic (time_sync,
// 58 B) arrived every time.
//
// Emitting only what the leaf reads takes that command to ~94 B, comfortably one frame.
// This does NOT make large commands safe: update_firmware carries a url plus a 64-hex
// sha256 and will still split, so the reassembly path needs its own fix before OTA works.
static void trim_cmd_for_air(const char* in, char* out, size_t cap) {
    out[0] = 0;
    char kind[32] = "";
    json_str(in, "kind", kind, sizeof(kind));
    if (!kind[0]) return;                       // unrecognisable -> caller sends it verbatim
    const long long id = json_ll(in, "id");
    char payload[320] = "";
    const char* pl = strstr(in, "\"payload\"");
    if (pl) {
        const char* b = strchr(pl, '{');
        if (b) {
            int depth = 0;
            const char* e = b;
            for (; *e; e++) {
                if (*e == '{') depth++;
                else if (*e == '}') { depth--; if (!depth) { e++; break; } }
            }
            size_t n = (size_t)(e - b);
            if (n && n < sizeof(payload)) { memcpy(payload, b, n); payload[n] = 0; }
        }
    }
    char eid[64] = "";
    json_str(in, "event_id", eid, sizeof(eid));   // null/absent -> empty -> omitted
    snprintf(out, cap, "{\"id\":%lld,\"kind\":\"%s\"%s%s%s%s%s}",
             id, kind,
             eid[0] ? ",\"event_id\":\"" : "", eid[0] ? eid : "", eid[0] ? "\"" : "",
             payload[0] ? ",\"payload\":" : "", payload[0] ? payload : "");
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
        // Trim server-only fields so an ordinary command fits one PHY frame.
        char air[512];
        trim_cmd_for_air(c.json, air, sizeof(air));
        const char* wire = air[0] ? air : c.json;
        RNS::Packet pkt(leaf_dest, RNS::Bytes((const uint8_t*)wire, strlen(wire)));
        const uint32_t t_send = millis();
        pkt.send();
        // Send-delay probe. The one number the passive sniffer could not give: how long
        // after dispatch heard the announce did this packet actually leave, and how
        // big was it. Pair with the leaf's "[cmd-probe] enter" line to tell
        // never-sent from outside-the-window from arrived-and-rejected.
        Serial.printf("[cmd-probe] TX t=%lu dispatch+%lums to=%s len=%u json=%.72s\n",
                      (unsigned long)t_send,
                      (unsigned long)(L.announce_ms ? t_send - L.announce_ms : 0),
                      L.slug[0] ? L.slug : "?", (unsigned)strlen(wire), wire);
        log_event("cmd-probe TX dispatch+%lums to %s len %u",
                  (unsigned long)(L.announce_ms ? t_send - L.announce_ms : 0),
                  L.slug[0] ? L.slug : "?", (unsigned)strlen(wire));
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
// Grant policy (reworked 0.12.0 — probe-analysis.md finding 1: the SNR-only loop was a
// constant function, 534/534 grants to sf7/bw250, down-shift unreachable): rolling
// announce RSSI vs the profile table's sensitivity floors is the primary input, because
// the SX126x SNR readout saturates ~+13 dB and carries no information at real distances.
// The uplink we measure is the STRONG half of the link — the grant governs both
// directions and the downlink measured ~13 dB weaker with TX power already matched
// (probes 30-34) — so uplink RSSI is derated by GW_RF_DOWNLINK_ASYM_DB into a downlink
// estimate before comparing against the floors. SNR stays as a second gate: an
// interference-limited link can read strong RSSI with no demod margin, and that is
// exactly the regime SNR still sees. Upgrade one step only when BOTH inputs have
// >= 12 dB headroom above the FASTER profile's floor; downgrade one step when EITHER
// has <= 6 dB above the CURRENT floor (asymmetric on purpose: slow to speed up, fast to
// slow down). The lockstep dance: send the grant into the RX window, retune ourselves
// immediately, and expect the leaf's confirm announce ON THE NEW PROFILE — no confirm in
// 20 s means the grant packet was lost, so revert (the leaf never heard it and is still
// on the old profile). Every failure path converges both ends back to base: the leaf
// reverts on TTL expiry / contactless wakes / power loss, we revert on confirm timeout
// and scan between granted-and-base when the leaf goes quiet off-base. Up-shift grants
// ride the weak downlink, and 33% of them historically never landed — after
// GW_RF_UP_REVERTS_MAX consecutive confirm-timeouts, up-shifts hold off for
// GW_RF_UP_BACKOFF_MS instead of churning (down-shifts are the safety direction and
// are never blocked).
#define GW_RF_SNR_SAMPLES_MIN     3
#define GW_RF_UP_HEADROOM_DB      12.0f
#define GW_RF_DOWN_HEADROOM_DB    6.0f
#ifndef GW_RF_DOWNLINK_ASYM_DB
#define GW_RF_DOWNLINK_ASYM_DB    13.0f            // FIELD-VALIDATED twice. DO NOT TRIM.
#endif                                             // 07-17 (office) measured 6-9 dB at the
                                                   // decision thresholds and this comment
                                                   // used to call 13 "~5 dB conservative".
                                                   // WITHDRAWN: 08-07 (north40) replicated
                                                   // the same protocol on an UNCONTENDED
                                                   // channel and measured -13.1 dB (n=9) in
                                                   // that band. The office's low figure was
                                                   // survivorship bias -- its probes were
                                                   // dying to contention, and a dead probe
                                                   // yields no asymmetry sample, so the
                                                   // surviving edge samples skewed
                                                   // small-asymmetry. 13 is ACCURATE, not
                                                   // padded; trimming it toward 8 would grant
                                                   // up-shifts the downlink cannot carry.
                                                   // Asym still compresses past ~-120 (both
                                                   // walks agree) but that is below every
                                                   // threshold here. Data: homelab
                                                   // docs/trailcam/adr-field-calibration.md
                                                   // (build-flag override: bench tests
                                                   // fake a huge asym to force down-shifts
                                                   // at desk RSSI)
#define GW_RF_UP_REVERTS_MAX      2
#define GW_RF_UP_BACKOFF_MS       (30 * 60 * 1000u)
#define GW_RF_CONFIRM_TIMEOUT_MS  20000u
#define GW_RF_SWITCH_COOLDOWN_MS  60000u
#define GW_RF_GRANT_TTL_S         (24 * 3600)      // matches the leaf's default
#define GW_RF_SCAN_SILENT_MS      (90 * 1000u)     // leaf quiet this long off-base -> scan
#define GW_RF_SCAN_TOGGLE_MS      (30 * 1000u)
// Walk-back shepherd: the multi-leaf walk-back grants base to whichever member
// is announcing right now — but a member asleep through that moment still holds the old
// rung, and a gateway parked on base can never hear it again (no commands, no PIR alert
// pushes) until the leaf's own contactless-wake safety reverts it: that check runs at
// wake-BEGIN after LEAF_RF_SILENT_WAKES_MAX=3 silent wakes, i.e. the 4th wake — up to
// ~8 h at the 2 h field night cadence. So after a walk-back, alternate between base and
// the vacated rung (same cadence and busy-gates as the quiet-leaf scan); the walk-back
// branch itself delivers the base grant into the RX window of anything caught there,
// the leaf live-retunes and confirms ON BASE, and that confirm ends the wait for it.
// Deadline reasoning: at the slowest cadence the sweep sees the incumbent's announce
// twice inside 4 h, each a ~50% catch (30 s alternation, random phase) plus an active
// delivery — past that, the leaf's own revert is at most one wake out, and alternating
// longer only costs the base-resident members announces. No per-leaf cadence
// prediction on purpose: PIR wakes randomize the schedule, a mispredicted park is
// strictly worse than the coin flip.
#define GW_RF_SHEPHERD_MAX_MS     (4 * 3600 * 1000u)
#define GW_RF_SHEPHERD_GRACE_MS   (60 * 1000u)     // base-heard this recently = not owed

// ADR stepping is confined to ONE profile family, because the table stopped being
// monotonically ordered when the certified ladder was appended (lora_profiles.h,
// 2026-08-07): idx 3 (sf12/bw500) is the FAR end of the cert family sitting directly
// after idx 2 (sf7/bw250), the NEAR end of the legacy one. Unclamped, `cur + 1` walks
// straight out of the family — and it always succeeds, because the headroom gate tests
// against the DESTINATION rung's floors and a far rung advertises very low ones
// (-130 dBm / -14 dB), which every working link clears by tens of dB. The result is an
// "up-shift" onto the slowest rung on the board, followed by 4 and 5 for the same
// reason. lora_profiles.h names this exact hazard and prescribes this exact fix
// ("clamp to one family"). Crossing families is a flag day (both ends go mutually deaf
// across a bandwidth change), never something ADR may do on its own.
#define GW_RF_FAMILY_MIN  ((uint8_t)(LORA_PROFILE_BASE <= 2 ? 0 : 3))
#define GW_RF_FAMILY_MAX  ((uint8_t)(LORA_PROFILE_BASE <= 2 ? 2 : LORA_PROFILE_COUNT - 1))

static void rf_beat(const char* event, uint8_t idx, float snr, float rssi,
                    float headroom) {
    // snr/rssi/headroom are optional (NAN = omit). %.1f of NAN prints "nan" — invalid
    // JSON that the telemetry endpoint rejects — so the fields are appended
    // conditionally. rssi is in the beat so the range re-walk can calibrate the 0.12.0
    // floors/asymmetry against what the decision loop actually saw.
    char opt[72] = "";
    int  n = 0;
    if (!isnan(snr))
        n += snprintf(opt + n, sizeof(opt) - n, ",\"snr\":%.1f", snr);
    if (!isnan(rssi) && n < (int)sizeof(opt))
        n += snprintf(opt + n, sizeof(opt) - n, ",\"rssi\":%.1f", rssi);
    if (!isnan(headroom) && n >= 0 && n < (int)sizeof(opt))
        snprintf(opt + n, sizeof(opt) - n, ",\"headroom\":%.1f", headroom);
    // Gateway self-beat: ADR decisions (scan/grant/confirm/revert) are made BY the
    // gateway; they were only ever filed under a camera because a node string was needed.
    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"extra\":{\"rf\":{\"event\":\"%s\",\"profile\":\"%s\",\"idx\":%u%s}}}",
               gw_site(), gw_site(), event, rf_name(idx), (unsigned)idx, opt);
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

// Start (or refresh) the walk-back shepherd for a vacated rung. Owed = every
// roster member not heard on base just now — the member whose announce triggered the
// walk-back was marked base-heard moments ago (when it announced on base) or is being
// handed the grant right now (when it announced on the stray rung), so the grace
// window sorts the two without special-casing the caller's L.
static void shepherd_arm(uint8_t stray_rung) {
    int owed = 0;
    for (int i = 0; i < s_roster_n; i++) {
        LeafEntry& e = s_roster[i];
        e.shep_wait = !(e.base_heard_ms &&
                        millis() - e.base_heard_ms < GW_RF_SHEPHERD_GRACE_MS);
        if (e.shep_wait) owed++;
    }
    if (!owed) return;
    s_rf.shep_rung      = stray_rung;
    s_rf.shep_start_ms  = millis();
    s_rf.shep_toggle_ms = millis();
    Serial.printf("[gw] ADR: SHEPHERD start — %d roster member(s) may still hold %s, "
                  "sweeping it against base for up to %lu min\n",
                  owed, rf_name(stray_rung),
                  (unsigned long)(GW_RF_SHEPHERD_MAX_MS / 60000u));
    log_event("ADR shepherd start: %d owed on %s", owed, rf_name(stray_rung));
    rf_beat("shepherd_start", stray_rung, NAN, NAN, NAN);
}

static void shepherd_end(const char* why) {
    Serial.printf("[gw] ADR: SHEPHERD end (%s) after %lu min\n", why,
                  (unsigned long)((millis() - s_rf.shep_start_ms) / 60000u));
    log_event("ADR shepherd end: %s", why);
    rf_beat("shepherd_end", s_rf.shep_rung, NAN, NAN, NAN);
    s_rf.shep_rung = 0xFF;
    for (int i = 0; i < s_roster_n; i++) s_roster[i].shep_wait = false;
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
        rf_beat("confirmed", s_rf.granted, LoRaInterface::last_snr,
                LoRaInterface::last_rssi, NAN);
        s_rf.up_reverts = 0;   // a landed grant resets the failed-up-shift backoff
        arm_urgent_sync_all();   // bug 10: fresh profile -> resync the leaf clocks
    }
    if (s_rf.scanning) {
        // Scan found the leaf on whatever we're parked on — adopt that as the truth
        // (usually base: the leaf power-cycled and its RTC grant wiped).
        const uint8_t abandoned = s_rf.granted;   // the rung the adopt walks away from
        s_rf.scanning = false;
        s_rf.granted  = s_rf.current;
        rf_persist(s_rf.granted);
        // A multi-leaf adopt-to-base is a walk-back by another door — the
        // reboot-mid-install path (boot scan-first, bug 2) converges HERE, not in the
        // walk-back branch, and the member still out on the old NVS rung would be
        // just as stranded. Same shepherd, same exit evidence.
        if (s_roster_n > 1 && abandoned != s_rf.granted &&
            abandoned != LORA_PROFILE_BASE && s_rf.shep_rung == 0xFF)
            shepherd_arm(abandoned);
        rf_clear_snr();
        Serial.printf("[gw] ADR: scan found the leaf on %s -> adopted\n",
                      rf_name(s_rf.granted));
        log_event("ADR scan: leaf found on %s", rf_name(s_rf.granted));
        rf_beat("adopted", s_rf.granted, LoRaInterface::last_snr,
                LoRaInterface::last_rssi, NAN);
        // Bug 10: scan-adopt IS the cold-boot-under-standing-grant path — the "hello"
        // announce came and went on a profile we weren't parked on, so the one-shot
        // hello sync never fired and the leaf is running its build-epoch clock.
        arm_urgent_sync_all();
    }
}

// After command delivery on an announce: decide whether to move the leaf. cmds_sent
// gates the grant — a leaf busy executing a fetch_full must not have the PHY yanked
// from under the transfer it's about to start.
// Returns TRUE if this call PUT A FRAME ON AIR (grant, walk-back or keepalive), so the
// caller can keep the rest of the downlink out of the way — see the note at the
// call site. Every early return means "sent nothing".
static bool maybe_grant_profile(LeafEntry& L, int cmds_sent) {
    if (!L.identity || s_rf.pending != 0xFF) return false;
    // Multi-leaf site (gateway-0.10.0): everyone lives on BASE. One radio can't serve
    // two leaves on different profiles, and a fast site profile makes a newly added
    // leaf invisible (it boots on base while we camp off-base — its hello is never
    // heard). Any standing grant gets actively walked back: tell whichever member is
    // announcing right now "base" and adopt base ourselves immediately. Members asleep
    // through this moment still hold the old rung — the shepherd (armed below)
    // sweeps that rung so THIS branch can catch their next announce there and hand
    // them the same grant, instead of leaving them deaf until the leaf's own
    // contactless-wake safety reverts them (the 4th wake, ~8 h at night cadence).
    if (s_roster_n > 1) {
        if (s_rf.granted != LORA_PROFILE_BASE || s_rf.current != LORA_PROFILE_BASE) {
            const uint8_t stray = s_rf.granted;   // the rung members may still hold
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
            rf_beat("multi_leaf_base", LORA_PROFILE_BASE, NAN, NAN, NAN);
            // stray == BASE here means granted was already walked back and we were
            // merely parked off-base (the shepherd's own sweep park) — the shepherd
            // that put us there stays armed; nothing new was vacated.
            if (stray != LORA_PROFILE_BASE && s_rf.shep_rung == 0xFF)
                shepherd_arm(stray);
            return true;
        }
        return false;
    }
    // Operator hold: parked on base for surveying/commissioning — no grants, no
    // keepalives, just listen. (Sits after the multi-leaf walk-back on purpose: a
    // member still holding an old grant gets actively told "base" even mid-hold.)
    if (rf_hold_active()) return false;
    if (s_rf.scanning) return false;
    if (cmds_sent || asm_active()) return false;   // transfer imminent or in progress
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
        return true;
    }
    if (s_rf.last_switch_ms && millis() - s_rf.last_switch_ms < GW_RF_SWITCH_COOLDOWN_MS)
        return false;
    if (s_rf.snr_n < GW_RF_SNR_SAMPLES_MIN) return false;
    const float   snr    = rf_avg_snr();
    const float   rssi   = rf_avg_rssi();
    // Derate the (strong) uplink reading into a downlink estimate — the grant governs
    // both directions but only the uplink is measurable from here.
    const float   dl_est = rssi - GW_RF_DOWNLINK_ASYM_DB;
    const uint8_t cur    = s_rf.current;
    uint8_t       want   = cur;
    float         headroom = NAN;
    const bool up_blocked = s_rf.up_reverts >= GW_RF_UP_REVERTS_MAX &&
                            millis() - s_rf.up_revert_ms < GW_RF_UP_BACKOFF_MS;
    if (cur < GW_RF_FAMILY_MAX && !up_blocked) {
        const float rssi_hr = dl_est - LORA_PROFILES[cur + 1].rssi_floor;
        const float snr_hr  = snr    - LORA_PROFILES[cur + 1].snr_floor;
        if (rssi_hr >= GW_RF_UP_HEADROOM_DB && snr_hr >= GW_RF_UP_HEADROOM_DB) {
            want = cur + 1;
            headroom = rssi_hr;
        }
    }
    if (want == cur && cur > GW_RF_FAMILY_MIN) {
        const float rssi_hr = dl_est - LORA_PROFILES[cur].rssi_floor;
        const float snr_hr  = snr    - LORA_PROFILES[cur].snr_floor;
        if (rssi_hr <= GW_RF_DOWN_HEADROOM_DB || snr_hr <= GW_RF_DOWN_HEADROOM_DB) {
            want = cur - 1;
            headroom = (rssi_hr < snr_hr) ? rssi_hr : snr_hr;   // the binding margin
        }
    }
    if (want == cur) return false;
    Serial.printf("[gw] ADR: granting %s (rssi %.0f dl_est %.0f snr %.1f, "
                  "headroom %.1f dB) -> retuning\n",
                  rf_name(want), rssi, dl_est, snr, headroom);
    log_event("ADR grant %s (rssi %.0f snr %.1f)", rf_name(want), rssi, snr);
    rf_beat("grant", want, snr, rssi, headroom);
    rf_send_grant(L.identity, want);
    s_rf.prev           = cur;
    s_rf.pending        = want;
    s_rf.pending_ms     = millis();
    s_rf.last_switch_ms = millis();
    rf_retune(want);   // follow our own grant; the leaf confirms here or we revert
    rf_clear_snr();    // SNR history doesn't transfer across profiles (BW noise floor)
    return true;
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
               gw_site(), gw_site(),
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
    if (s_rf_hold_until && !rf_hold_active()) {
        s_rf_hold_until = 0;
        Serial.println("[gw] rf/base: hold expired -> ADR resumed");
        log_event("rf/base hold expired, ADR resumed");
    }
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
        rf_beat("revert", s_rf.prev, NAN, NAN, NAN);
        // Up-shift grants ride the weak downlink; a lost one proves nothing changed,
        // so repeated losses back off further attempts instead of churning (33% of
        // all historical grants reverted). Down-shift losses are NOT counted — moving
        // toward robust is the safety direction and must stay eligible every announce.
        if (s_rf.pending > s_rf.prev) {
            if (s_rf.up_reverts < 255) s_rf.up_reverts++;
            s_rf.up_revert_ms = millis();
            if (s_rf.up_reverts == GW_RF_UP_REVERTS_MAX)
                log_event("ADR: %u up-shift grants lost -> backoff %lu min",
                          (unsigned)s_rf.up_reverts,
                          (unsigned long)(GW_RF_UP_BACKOFF_MS / 60000u));
        }
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
    // Walk-back shepherd: sweep the vacated rung for members that slept
    // through the walk-back. The multi-leaf branch in maybe_grant_profile() is the
    // delivery arm — anything caught announcing on the swept rung gets the base grant
    // in its RX window and confirms on base seconds later, which clears its debt.
    // Exit the moment nobody is owed, on the hard deadline (the leaf's own
    // contactless-wake revert is at most one wake out by then), or if the site stops
    // being multi-leaf (single-leaf ADR takes back over). Same busy-gates as the
    // scan: never retune off a live transfer.
    if (s_rf.shep_rung != 0xFF) {
        bool owed = false;
        for (int i = 0; i < s_roster_n; i++) owed |= s_roster[i].shep_wait;
        const bool busy = radio_busy() || asm_active();
        if (!owed || s_roster_n <= 1 ||
            millis() - s_rf.shep_start_ms > GW_RF_SHEPHERD_MAX_MS) {
            if (s_rf.current == LORA_PROFILE_BASE || !busy) {
                shepherd_end(!owed          ? "all members heard on base"
                             : s_roster_n <= 1 ? "roster no longer multi-leaf"
                                               : "deadline, leaf self-revert finishes");
                if (s_rf.current != LORA_PROFILE_BASE) rf_retune(LORA_PROFILE_BASE);
            }
        } else if (s_rf.pending == 0xFF && !busy && !rf_hold_active() &&
                   millis() - s_rf.shep_toggle_ms > GW_RF_SCAN_TOGGLE_MS) {
            s_rf.shep_toggle_ms = millis();
            const uint8_t park = (s_rf.current == LORA_PROFILE_BASE) ? s_rf.shep_rung
                                                                     : LORA_PROFILE_BASE;
            rf_retune(park);
            Serial.printf("[gw] ADR: shepherd sweep -> %s\n", rf_name(park));
        }
    }
}

#ifdef GW_NET_HAS_ETH
// --- wired ethernet ---------------------------------------------------------------------
// The link-event handling is identical for both PHYs (the ETH API abstracts localIP /
// linkSpeed / fullDuplex); only the ETH.begin bring-up and the event-bus name differ.
//
// LAN8720 (T-Internet-POE): classic core-2.x ETH.begin(addr,power,mdc,mdio,type,clk),
//   events on the WiFi bus, WiFiEvent_t handler. Env pinned to espressif32@6.x.
// W5500 (production gateway PCB): core-3.x ETH.begin(W5500,addr,cs,int,rst,spi_host,
//   sck,miso,mosi) over its own SPI3 bus, events on the Network bus, arduino_event_id_t
//   handler (pioarduino core 3.x). Pin map = gateway PCB GPIO draft, gate-c-proven.
#ifdef GW_NET_ETH
// Pin map proven on the bench 2026-06-30 (bridge-poe Step 1): MDC 23, MDIO 18, PHY addr 0,
// clock OUT on GPIO17, no power-enable pin.
#define GW_ETH_ADDR        0
#define GW_ETH_POWER_PIN  -1
#define GW_ETH_MDC_PIN    23
#define GW_ETH_MDIO_PIN   18
#define GW_ETH_TYPE       ETH_PHY_LAN8720
#define GW_ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT
#endif

// Which bus the ETH events arrive on is a CORE-VERSION fact, not a PHY fact: core 3.x moved
// them from the WiFi event bus to the Network one and renamed the id type. Key off the core
// version directly so a future core-3.x LAN8720 env (or a core-2.x anything) still compiles.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_eth_event(arduino_event_id_t event) {
#else
static void on_eth_event(WiFiEvent_t event) {
#endif
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

// Bring-up is event-driven and non-blocking; DHCP renewal/link flaps are the PHY's problem.
static void eth_begin() {
#ifdef GW_NET_ETH_W5500
    ETH.begin(ETH_PHY_W5500, /*phy_addr=*/1,
              ETH_CS_PIN, ETH_INT_PIN, ETH_RST_PIN,
              SPI3_HOST, ETH_SCK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
#else
    ETH.begin(GW_ETH_ADDR, GW_ETH_POWER_PIN, GW_ETH_MDC_PIN,
              GW_ETH_MDIO_PIN, GW_ETH_TYPE, GW_ETH_CLK_MODE);
#endif
}
#endif  // GW_NET_HAS_ETH

#ifdef GW_NET_HAS_WIFI
// --- WiFi credentials -------------------------------------------------------------------
// Two sources, tried NVS-first:
//
//   NVS (one network, key "wifi" in the "gateway" namespace) — what a provisioned box was
//     set up on. First because it is a deliberate act by whoever owns the unit, whereas the
//     baked list is a factory/bench convenience that must never outrank it.
//   BAKED (GW_WIFI_SSID/PSK, ...SSID2/3) — the site's primary then any fallback, so the
//     SAME firmware works on-site and on the bench. A shipped consumer build bakes NONE and
//     relies entirely on provisioning; a dev/bench build bakes the bench SSID and so never
//     reaches the provisioning path at all.
//
// net_creds_save() is the single seam every provisioning front-end writes through — captive
// portal, mesh config downlink, or serial/QR at kitting all end here. Which of those ships
// is still open; see homelab docs/trailcam/gateway-provisioning.md.
struct WifiCred { const char* ssid; const char* psk; };
static const WifiCred WIFI_BAKED[] = {
// The portal bench rig (heltec-v3-portal-bench) boots with an EMPTY baked list so it
// lands in PROVISION and raises the AP. A -U flag cannot do this any more — the
// credentials arrive via gw_secrets.h, and -U only undoes command-line -D.
#if !defined(GW_PORTAL_BENCH)
#if defined(GW_WIFI_SSID) && defined(GW_WIFI_PSK)
    { GW_WIFI_SSID,  GW_WIFI_PSK  },
#endif
#if defined(GW_WIFI_SSID2) && defined(GW_WIFI_PSK2)
    { GW_WIFI_SSID2, GW_WIFI_PSK2 },
#endif
#if defined(GW_WIFI_SSID3) && defined(GW_WIFI_PSK3)
    { GW_WIFI_SSID3, GW_WIFI_PSK3 },
#endif
#endif  // !GW_PORTAL_BENCH
    { nullptr, nullptr },   // terminator: a zero-length array is illegal, and a consumer
};                          // build legitimately bakes nothing at all
static const size_t WIFI_BAKED_COUNT = (sizeof(WIFI_BAKED) / sizeof(WIFI_BAKED[0])) - 1;

static char s_nvs_ssid[33] = "";    // 32 + NUL
static char s_nvs_psk[64]  = "";    // WPA2 PSK max 63 + NUL

// Has the provisioned credential EVER associated? Persisted, because it is the
// difference between the two failures that look identical from inside the box:
//
//   never worked   -> the passphrase is almost certainly a typo, and on a sealed
//                     consumer unit with no ethernet and no serial that is a brick.
//                     Re-open the portal (portal_rescue_due()) so it can be corrected.
//   worked before  -> the router is down, moved, or renamed. NOT a reason to raise an
//                     open AP in someone's house; the box just keeps retrying. That is
//                     the "don't resurrect on a later WiFi drop" rule in
//                     homelab docs/trailcam/gateway-provisioning.md.
static bool s_nvs_cred_ok = false;

static void net_creds_load() {
    Preferences p;
    p.begin("gateway", /*readOnly=*/true);
    p.getString("wifi_ssid", s_nvs_ssid, sizeof(s_nvs_ssid));
    p.getString("wifi_psk",  s_nvs_psk,  sizeof(s_nvs_psk));
    s_nvs_cred_ok = p.getBool("wifi_ok", false);
    p.end();
    if (s_nvs_ssid[0])
        Serial.printf("[gw] provisioned WiFi network: \"%s\" (%s)\n",
                      s_nvs_ssid, s_nvs_cred_ok ? "proven" : "never associated yet");
}
// One NVS write, the first time a provisioned credential actually associates. Idempotent
// so the reconnect path can call it freely.
static void net_creds_mark_ok() {
    if (s_nvs_cred_ok || !s_nvs_ssid[0]) return;
    s_nvs_cred_ok = true;
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    p.putBool("wifi_ok", true);
    p.end();
    Serial.printf("[gw] credential for \"%s\" proven on air\n", s_nvs_ssid);
}
// Returns false and changes nothing on a malformed SSID — a provisioning front-end must be
// able to reject bad input without having already clobbered a working credential.
//
// This is the sink EVERY provisioning option writes through — the serial console below is
// the first one; a SoftAP captive portal would land here too. Nothing about the
// selector changes when another front-end is added.
static bool net_creds_save(const char* ssid, const char* psk) {
    if (!ssid || !ssid[0] || strlen(ssid) >= sizeof(s_nvs_ssid)) return false;
    if (!psk  || strlen(psk) >= sizeof(s_nvs_psk)) return false;
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    p.putString("wifi_ssid", ssid);
    p.putString("wifi_psk",  psk);
    p.putBool("wifi_ok", false);   // a NEW credential is unproven until it associates
    p.end();
    s_nvs_cred_ok = false;
    strlcpy(s_nvs_ssid, ssid, sizeof(s_nvs_ssid));
    strlcpy(s_nvs_psk,  psk,  sizeof(s_nvs_psk));
    Serial.printf("[gw] WiFi credential stored for \"%s\"\n", ssid);
    return true;
}
// Forget the provisioned network and fall back to whatever is baked in. Deliberately does
// NOT touch a live association: if we are currently up on this credential, tearing the
// uplink down the instant it is erased would take the box off the air with no warning.
// The next re-selection (a link drop, or a reboot) is soon enough.
static void net_creds_clear() {
    Preferences p;
    p.begin("gateway", /*readOnly=*/false);
    p.remove("wifi_ssid");
    p.remove("wifi_psk");
    p.remove("wifi_ok");
    p.end();
    s_nvs_ssid[0] = 0;
    s_nvs_psk[0]  = 0;
    s_nvs_cred_ok = false;
    Serial.println("[gw] provisioned WiFi credential erased");
}
static size_t net_cred_count() { return WIFI_BAKED_COUNT + (s_nvs_ssid[0] ? 1 : 0); }
// Bounds-safe on purpose: s_wifi_idx is a rotation cursor that outlives the list it points
// into (a save or clear changes the count under it), and the OLED reads it on its own
// schedule, before wifi_ensure() next gets a chance to clamp. Out of range returns the
// array terminator — a {nullptr,nullptr} that callers already have to handle — rather than
// reading past the end.
static WifiCred net_cred_at(size_t i) {
    if (i >= net_cred_count()) return WifiCred{ nullptr, nullptr };
    if (s_nvs_ssid[0]) {
        if (i == 0) return WifiCred{ s_nvs_ssid, s_nvs_psk };
        i--;
    }
    return WIFI_BAKED[i];
}

static size_t s_wifi_idx = 0;   // which network we're currently attempting

#define GW_WIFI_TRY_MS 15000    // per-network association window before rotating

// File-scope rather than a local static inside wifi_ensure() so net_creds_changed() can
// zero it: a credential that just arrived should be tried NOW, not after the remainder of
// the previous network's 15 s window. Somebody typing a password at a serial console and
// watching nothing happen for 15 s will reasonably conclude it did not take.
static uint32_t s_wifi_last_try = 0;

// How many association attempts this credential set has burned without ever connecting.
// Zeroed on a successful association and by net_creds_changed(); read by the portal's
// rescue trigger, which is the only thing that cares.
static uint8_t s_wifi_fails = 0;

static void wifi_ensure() {
    if (WiFi.status() == WL_CONNECTED) return;
    const size_t n_nets = net_cred_count();
    if (n_nets == 0) return;
    if (s_wifi_last_try && millis() - s_wifi_last_try < GW_WIFI_TRY_MS) return;
    // The current network didn't associate in its window -> rotate to the next one.
    if (s_wifi_last_try) {
        if (s_wifi_fails < 255) s_wifi_fails++;
        if (n_nets > 1) s_wifi_idx = (s_wifi_idx + 1) % n_nets;
    }
    if (s_wifi_idx >= n_nets) s_wifi_idx = 0;   // a save/clear can shrink the list under us
    s_wifi_last_try = millis();
    const WifiCred n = net_cred_at(s_wifi_idx);
    if (!n.ssid) return;   // cannot happen: n_nets > 0 and s_wifi_idx was just clamped
    Serial.printf("[gw] WiFi joining \"%s\" (net %u/%u) ...\n",
                  n.ssid, (unsigned)(s_wifi_idx + 1), (unsigned)n_nets);
    // Clear any in-flight association before (re)configuring. Without this the
    // multi-network fallback wedges: when the current network is out of range the STA
    // sits in "connecting", and the next WiFi.begin() (this call, for the NEXT network)
    // fails with ESP_ERR_WIFI_STATE ("sta is connecting, cannot set config") — so the
    // gateway can never fail over to a reachable network. A cold boot with net-1 absent
    // then never gets online (2026-07-21: took GW-HOME down after an OTA reboot at a
    // site where only net-2 was in range). disconnect(true) drops the radio and clears
    // that state; mode()+begin() below bring it back up fresh on the chosen network.
    // On the very first call the WiFi driver has never been brought up (dual-stack
    // defers it to WAIT_WIFI, see above), so there is no in-flight association yet to
    // clear — disconnect(true) there only logs a spurious ESP_ERR_WIFI_NOT_INIT. Skip it
    // ONCE; every later call, including the very next rotation, still runs it.
    static bool s_wifi_driver_up = false;
    if (s_wifi_driver_up) WiFi.disconnect(true);
    // disconnect(true) only clears the STA bit, so the SoftAP survives it, but mode()
    // would not — hence AP_STA while the provisioning portal is live. That combination
    // is what lets the rescue portal keep retrying the stored credential underneath
    // itself: a router that merely came back late closes the portal with nobody
    // touching the box.
#ifdef GW_HAS_PORTAL
    WiFi.mode(s_portal_up ? WIFI_AP_STA : WIFI_STA);
#else
    WiFi.mode(WIFI_STA);
#endif
    WiFi.begin(n.ssid, n.psk);
    s_wifi_driver_up = true;
}
#endif  // GW_NET_HAS_WIFI

// --- uplink selector --------------------------------------------------------------------
// Ethernet is PREFERRED whenever it is available: no credentials, no support surface, no
// airtime, and it is the tower pattern. WiFi is the fallback that makes a window-mounted
// consumer unit possible on the same SKU. The full state machine and the reasoning behind
// it are in homelab docs/trailcam/gateway-pcb.md.
//
//   WAIT_ETH   bounded window (GW_ETH_PRIMARY_MS) for link+DHCP before spending anything
//              on WiFi. On an ethernet-only build this is simply where the box waits.
//   WAIT_WIFI  rotate the credential list; ethernet coming up late still preempts.
//   UP         active interface serves every socket; on loss, re-select from the top.
//   PROVISION  no ethernet and no usable credential — nothing to try. The provisioning
//              front-end is NOT chosen yet, so this state currently only reports itself.
//
// DEVIATION from the drafted state machine, deliberate: WiFi is not registered at boot,
// only when the box actually enters WAIT_WIFI. The draft brought the STA up early and left
// it idle. Deferring it keeps the WiFi PA and its ~40 kB of heap out of the common
// ethernet-only path entirely, which is also the cheap answer to the drafted coexistence
// question (a tower unit never has WiFi and the radio live at the same time). The cost is
// that the first fallback association starts ~100 ms later, on a path already budgeted 12 s.
enum NetState : uint8_t { NET_ST_WAIT_ETH, NET_ST_WAIT_WIFI, NET_ST_UP, NET_ST_PROVISION };
static NetState s_net_state = NET_ST_WAIT_ETH;
static uint32_t s_net_t0    = 0;    // when the current wait window started

#ifndef GW_ETH_PRIMARY_MS
#define GW_ETH_PRIMARY_MS 12000     // link + DHCP on a cold switch port fits well inside this
#endif

#if defined(GW_NET_HAS_ETH) && defined(GW_NET_HAS_WIFI)
static bool s_eth_ever_up = false;  // has this box EVER had a wired lease?
#endif

// Where re-selection starts after losing the active interface. Only pay the ethernet
// window again if ethernet has actually worked here before — on a WiFi-only box, or a
// dual-stack box with nothing plugged in, that window is 12 s of dead air on every blip.
// Skipping it is safe because the preempt check at the top of net_ensure() runs in EVERY
// state, so a late link is never missed regardless of where selection resumes.
static void net_select_restart() {
    s_net_t0 = millis();
#ifdef GW_NET_HAS_ETH
#ifdef GW_NET_HAS_WIFI
    s_net_state = s_eth_ever_up ? NET_ST_WAIT_ETH : NET_ST_WAIT_WIFI;
#else
    s_net_state = NET_ST_WAIT_ETH;
#endif
#else
    s_net_state = NET_ST_WAIT_WIFI;
#endif
}

#ifdef GW_HAS_PORTAL
static void portal_sync();   // start/stop the SoftAP to match the state we just landed in
static void portal_loop();   // pump the captive DNS while it is up
#else
static inline void portal_sync() {}
static inline void portal_loop() {}
#endif

static void net_select_tick();   // the selector proper; net_ensure() is the wrapper below

// One selector tick, then whatever the resulting state implies for the provisioning
// portal. Every caller goes through this (setup, loop, net_creds_changed), so no path
// can change state and leave a SoftAP running behind it. That matters most for the
// ethernet preempt, which returns early from the tick from any state, PROVISION included.
static void net_ensure() { net_select_tick(); portal_sync(); }

static void net_setup() {
#ifdef GW_NET_HAS_WIFI
    net_creds_load();
#endif
#ifdef GW_NET_HAS_ETH
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    Network.onEvent(on_eth_event);   // core-3.x: ETH events ride the Network event bus
#else
    WiFi.onEvent(on_eth_event);      // core-2.x: ETH events ride the WiFi event bus
#endif
    eth_begin();
    s_net_state = NET_ST_WAIT_ETH;
#else
    s_net_state = NET_ST_WAIT_WIFI;
#endif
    s_net_t0 = millis();
    net_ensure();   // a WiFi-only box associates immediately, exactly as it always has
}

static void net_select_tick() {
#ifdef GW_NET_HAS_ETH
    // Ethernet PREEMPTS in every state: it is strictly cheaper than WiFi, so a link that
    // shows up late (switch finished booting, someone plugged the cable in) takes over.
    if (s_eth_got_ip && s_net_active != NET_IFACE_ETH) {
#ifdef GW_NET_HAS_WIFI
        s_eth_ever_up = true;
        if (s_net_active == NET_IFACE_WIFI) {
            Serial.println("[gw] ethernet link up — preempting WiFi");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
        }
#endif
        s_net_active = NET_IFACE_ETH;
        s_net_state  = NET_ST_UP;
        // The preempt path is a seamless interface switch: net_up() was already true
        // (WiFi) and stays true (now ETH), so it never edge-triggers loop()'s net_was_up
        // check — which is where the mesh-bridge resume normally lives. Re-arm
        // directly here instead: reaching this line already means "just became ETH", so
        // it is always correct to resume when the bridge was running before the outage.
        // Gap found on the gate-C bench rig 2026-08-20 cable-pull retest: an
        // ETH -> WiFi -> ETH cycle left the bridge stopped forever after landing the ETH
        // gate on the WiFi-side resume (the bug that gate fixed had been masking this —
        // the ungated resume used to fire on the WiFi leg and leave the socket running).
        if (s_udp_started) {
            udp_interface.start();
            Serial.println("[gw] RNS UDP interface resumed");
        }
        return;
    }
#endif
    switch (s_net_state) {

    case NET_ST_WAIT_ETH: {
        if (millis() - s_net_t0 <= GW_ETH_PRIMARY_MS) break;
#ifdef GW_NET_HAS_WIFI
        s_net_t0 = millis();
        if (net_cred_count() > 0) {
            Serial.printf("[gw] no wired lease in %u ms — falling back to WiFi\n",
                          (unsigned)GW_ETH_PRIMARY_MS);
            s_net_state = NET_ST_WAIT_WIFI;
            wifi_ensure();
        } else {
            s_net_state = NET_ST_PROVISION;
        }
#else
        // Ethernet-only build: there is nothing to fall back to, so keep waiting for the
        // link event exactly as before. Say so ONCE — a dead PHY used to be silent.
        static bool warned = false;
        if (!warned) {
            warned = true;
            Serial.printf("[gw] still no wired lease after %u ms; no WiFi in this build, "
                          "waiting on the link\n", (unsigned)GW_ETH_PRIMARY_MS);
        }
#endif
        break;
    }

#ifdef GW_NET_HAS_WIFI
    case NET_ST_WAIT_WIFI:
        if (WiFi.status() == WL_CONNECTED) {
            s_net_active = NET_IFACE_WIFI;
            s_net_state  = NET_ST_UP;
            s_wifi_fails = 0;
            // Pointer equality, not a string compare: net_cred_at() hands back
            // s_nvs_ssid itself for the provisioned entry, so this asks "did the
            // credential somebody typed in just work", not "does some baked entry
            // happen to share the name".
            if (net_cred_at(s_wifi_idx).ssid == s_nvs_ssid) net_creds_mark_ok();
            break;
        }
        if (net_cred_count() == 0) { s_net_state = NET_ST_PROVISION; s_net_t0 = millis(); break; }
        wifi_ensure();   // owns its own GW_WIFI_TRY_MS rotation window
        break;

    case NET_ST_PROVISION: {
        // Credentials can appear while we sit here (that is the whole point), so re-check
        // and leave the moment there is something to try.
        if (net_cred_count() > 0) { s_net_state = NET_ST_WAIT_WIFI; s_net_t0 = millis(); break; }
        // The provisioning front-ends attach HERE: portal_sync() (called by net_ensure
        // right after this tick) raises the SoftAP + captive portal for as long as this
        // state holds, and the serial console is always live. Both write through
        // net_creds_save() / claim_start(). Say so on a slow beat regardless: a build
        // with the portal compiled out, or a unit whose owner never joins the AP,
        // should not look silently wedged on the monitor.
        static uint32_t last_nag = 0;
        if (!last_nag || millis() - last_nag > 30000) {
            last_nag = millis();
            Serial.println("[gw] UNPROVISIONED: no wired link and no WiFi credential — "
                           "no uplink is possible until one is supplied");
        }
        break;
    }
#endif

    case NET_ST_UP:
        if (!net_up()) {
            Serial.printf("[gw] uplink (%s) lost — re-selecting\n", net_desc().c_str());
#ifdef GW_NET_HAS_ETH
            // _online tracks the RNS interface's own state, not whether the netif still
            // has a route — left running here, it keeps firing into a dead netif for the
            // whole re-selection window ("endPacket(): could not send data: 118
            // ENETUNREACH"). stop() just clears _online on this build (see
            // UDPInterface::stop()), so this is cheap and reversible; the up-transition
            // below re-arms it once a route exists again, wired or WiFi.
            if (s_udp_started) udp_interface.stop();
#endif
            s_net_active = NET_IFACE_NONE;
            net_select_restart();
        }
        break;

    default: break;
    }
}

#ifdef GW_NET_HAS_WIFI
// Call after any change to the credential list. Rewinds the rotation cursor to the new
// network and clears the retry window so the next net_ensure() tick acts on it, instead of
// waiting out the remainder of the previous network's 15 s.
//
// Only nudges when we are NOT already up: a credential edited from the serial console while
// the box is happily online must not knock it off the air. The new credential takes effect
// on the next natural re-selection.
static void net_creds_changed() {
    s_wifi_idx      = 0;
    s_wifi_last_try = 0;
    s_wifi_fails    = 0;   // a fresh credential starts the rescue count over
    if (s_net_state == NET_ST_UP) return;
    if (net_cred_count() > 0 && s_net_state == NET_ST_PROVISION) {
        s_net_state = NET_ST_WAIT_WIFI;
        s_net_t0    = millis();
    }
    net_ensure();
}
#endif

// --- serial console ---------------------------------------------------------------------
// The gateway's USB port was output-only until now. This is the smallest line reader that
// lets a credential be written to a box IN HAND without rebuilding firmware for it — the
// bench/pilot half of the provisioning question (homelab docs/trailcam/gateway-provisioning.md).
// It is not the consumer answer; a buyer gets the SoftAP portal, which writes
// through the same net_creds_save() sink.
//
// Commands, one per line:
//   help                      this list
//   net                       which uplink is live, selector state, IP
//   wifi show                 the provisioned SSID (never the PSK)
//   wifi ssid <rest of line>  stage an SSID
//   wifi psk <rest of line>   stage a passphrase
//   wifi save                 commit the staged pair to NVS
//   wifi clear                forget the provisioned network
//   claim show                identity pubkey + device id + claim state (never the token)
//   claim clear               drop the claim block, revert to the baked fallback
//   claim <code>              claim this unit with a printed setup code
//
// SSID and PSK take the REST OF THE LINE verbatim, spaces included: both legitimately
// contain spaces, and a one-shot "wifi set <ssid> <psk>" form cannot express that
// unambiguously. Staging plus an explicit save is what makes it safe — a half-typed pair
// never reaches NVS, and net_creds_save() rejects a malformed SSID without clobbering a
// working credential.
//
// NOTHING IS ECHOED, deliberately: a passphrase typed here would otherwise land in every
// captured monitor log. Turn on local echo in the terminal instead (`pio device monitor
// --echo`) if you want to see what you are typing.
//
// Trust model: physical USB access. Anyone who can reach this port can already reflash the
// board, so the console grants nothing new — but it still never prints a stored PSK back.
#define GW_CONSOLE_LINE_MAX 160

static void serial_exec(char* line) {
    while (*line == ' ') line++;
    if (!*line) return;

    if (!strcmp(line, "help")) {
        Serial.println("[gw] commands: help | net"
#ifdef GW_NET_HAS_WIFI
                       " | wifi show | wifi ssid <v> | wifi psk <v> | wifi save | wifi clear"
#endif
#ifndef GW_NO_CLAIM
                       " | claim show | claim clear | claim <code>"
#endif
                       );
        Serial.println("[gw] nothing is echoed; use `pio device monitor --echo` to see typing");
        return;
    }

    if (!strcmp(line, "net")) {
        const char* st = s_net_state == NET_ST_WAIT_ETH  ? "WAIT_ETH"
                       : s_net_state == NET_ST_WAIT_WIFI ? "WAIT_WIFI"
                       : s_net_state == NET_ST_UP        ? "UP"
                       : s_net_state == NET_ST_PROVISION ? "PROVISION" : "?";
        Serial.printf("[gw] state=%s up=%d iface=%s ip=%s rssi=%d\n",
                      st, (int)net_up(), net_desc().c_str(),
                      net_ip().toString().c_str(), net_rssi());
#ifdef GW_HAS_PORTAL
        if (s_portal_up)
            Serial.printf("[gw] provisioning portal UP: \"%s\" at %s\n",
                          s_portal_ssid, WiFi.softAPIP().toString().c_str());
        else
            Serial.printf("[gw] provisioning portal down (%u failed association(s), "
                          "credential %s)\n", (unsigned)s_wifi_fails,
                          !s_nvs_ssid[0] ? "none" : s_nvs_cred_ok ? "proven" : "unproven");
#endif
        return;
    }

#ifndef GW_NO_CLAIM
    // The claim half of the console. "claim show" is what kitting and the
    // family-gateway migration read; it never prints the token, same rule as the PSK.
    // "claim clear" reverts to the baked fallback — the bench escape hatch that does
    // not need an NVS erase (which would also re-mint the identity).
    if (!strcmp(line, "claim show")) {
        Serial.printf("[gw] device id:  %s\n", s_device_id);
        Serial.printf("[gw] public key: %s\n", s_pubkey_hex);
        if (s_claim_site[0]) {
            Serial.printf("[gw] claimed: site \"%s\"  url %s\n", s_claim_site, s_claim_url);
            Serial.printf("[gw] token: %s  token id: %s\n",
                          s_claim_tok[0] ? "(set)" : "(none)",
                          s_claim_tid[0] ? s_claim_tid : "(none)");
        } else {
            Serial.printf("[gw] unclaimed — baked fallback (site \"%s\", token %s)\n",
                          gw_site(), gw_token()[0] ? "baked" : "NONE — uploads gated off");
        }
        claim_console_status();
        return;
    }
    if (!strcmp(line, "claim clear")) {
        claim_forget();
        Serial.println("[gw] claim block cleared — baked fallback until re-claimed");
        return;
    }
    if (!strncmp(line, "claim ", 6)) {
        claim_start(line + 6);   // prints its own accept/reject lines
        return;
    }
#endif

#ifdef GW_NET_HAS_WIFI
    // Staged until "wifi save" — see the rest-of-line note above.
    static char stage_ssid[33] = "";
    static char stage_psk[64]  = "";

    if (!strcmp(line, "wifi show")) {
        Serial.printf("[gw] provisioned ssid: %s\n", s_nvs_ssid[0] ? s_nvs_ssid : "(none)");
        Serial.printf("[gw] provisioned psk:  %s\n", s_nvs_psk[0]  ? "(set)"   : "(none)");
        Serial.printf("[gw] staged ssid: %s  psk: %s\n",
                      stage_ssid[0] ? stage_ssid : "(none)",
                      stage_psk[0]  ? "(set)"    : "(none)");
        Serial.printf("[gw] %u credential(s) in the try list (NVS first, then baked)\n",
                      (unsigned)net_cred_count());
        return;
    }
    if (!strncmp(line, "wifi ssid ", 10)) {
        const char* v = line + 10;
        if (strlen(v) >= sizeof(stage_ssid)) { Serial.println("[gw] ssid too long (max 32)"); return; }
        strlcpy(stage_ssid, v, sizeof(stage_ssid));
        Serial.printf("[gw] staged ssid \"%s\" — now `wifi psk <v>` then `wifi save`\n", stage_ssid);
        return;
    }
    if (!strncmp(line, "wifi psk ", 9)) {
        const char* v = line + 9;
        if (strlen(v) >= sizeof(stage_psk)) { Serial.println("[gw] psk too long (max 63)"); return; }
        strlcpy(stage_psk, v, sizeof(stage_psk));
        Serial.println("[gw] staged psk — now `wifi save`");
        return;
    }
    if (!strcmp(line, "wifi save")) {
        if (!stage_ssid[0]) { Serial.println("[gw] no staged ssid — `wifi ssid <v>` first"); return; }
        if (!net_creds_save(stage_ssid, stage_psk)) { Serial.println("[gw] rejected, nothing written"); return; }
        stage_ssid[0] = 0;
        stage_psk[0]  = 0;   // don't leave a passphrase sitting in RAM after it is committed
        net_creds_changed();
        return;
    }
    if (!strcmp(line, "wifi clear")) {
        net_creds_clear();
        net_creds_changed();
        return;
    }
#endif

    Serial.printf("[gw] unknown command \"%s\" — try `help`\n", line);
}

// Non-blocking: at most one line is assembled per call, and an over-long line is discarded
// whole rather than silently truncated into a half-command that might still parse.
static void serial_console_loop() {
    static char buf[GW_CONSOLE_LINE_MAX];
    static size_t len = 0;
    static bool overflow = false;

    while (Serial.available()) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            if (overflow) { Serial.println("[gw] line too long, ignored"); overflow = false; }
            else if (len) { buf[len] = '\0'; serial_exec(buf); }
            len = 0;
            continue;
        }
        if (c == 8 || c == 127) { if (len) len--; continue; }   // backspace / DEL
        if (len + 1 >= sizeof(buf)) { overflow = true; continue; }
        buf[len++] = (char)c;
    }
}

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

#ifndef GW_NO_CLAIM
#include "claim_client.inc"
#endif

#include "portal.inc"

#if defined(GW_HAS_PORTAL) || !defined(GW_NO_CLAIM)
// /setup wears two hats. Over the SoftAP it is the full portal form (WiFi + claim code).
// On the uplink IP it is the claim-code-only form, live while the unit is not claimed,
// 404 forever after — never the WiFi fields, which would be an unauthenticated re-point
// endpoint on a customer's LAN (the surface the portal design refused to create).
static void setup_route_get() {
#ifdef GW_HAS_PORTAL
    if (s_portal_up) { portal_page(); return; }
#endif
#ifndef GW_NO_CLAIM
    claim_setup_page();
#else
    s_web.send(404, "text/plain", "not found\n");
#endif
}
static void setup_route_post() {
#ifdef GW_HAS_PORTAL
    if (s_portal_up) { portal_submit(); return; }
#endif
#ifndef GW_NO_CLAIM
    claim_setup_submit();
#else
    s_web.send(404, "text/plain", "not found\n");
#endif
}
#endif

// Multipart POST per the ingest contract (mirrors host-bridge _post_photo), streamed
// from three segments (head / body / tail). The body segment is either a RAM buffer
// (thumbs) or an open LittleFS File (spooled fulls) — either way a 135 KB full never
// touches the heap.
#include "sd_queue_upload.inc"

// The depth-1 slot's uploader — now the FALLBACK path, reached only when no usable card
// is present. Returns true when the frame is handled (uploaded or permanently dropped).
static bool upload_pending() {
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
    const UploadOutcome r =
        upload_body(s_pending.event_id, s_pending.camera, s_pending.kind,
                    s_pending.captured_at, s_pending.jpeg,
                    s_pending.spooled ? &spool : nullptr, s_pending.len);
    if (spool) spool.close();
    return r != UP_RETRY;
}

#ifdef GW_SD_QUEUE
#include "sd_queue_drain.inc"
#endif   // GW_SD_QUEUE

// --- site name (address) fetch ----------------------------------------------------------
// GET /api/v1/site?slug=<site> with the device token; cache the display name in NVS.
// Blocking TLS GET, so callers gate it on !radio_busy() like the other HTTP work.
static void fetch_site_name() {
    if (!net_up() || !gw_provisioned()) return;
    WiFiClientSecure tls;
    tls.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(10000);
    String url = String(gw_url());
    url.replace("/ingest", String("/site?slug=") + gw_site());
    if (!http.begin(tls, url)) return;
    http.addHeader("Authorization", String("Bearer ") + gw_token());
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
    // Whatever uplink is live, not specifically WiFi. Wired reports rssi 0, which the
    // icon's thresholds render as full bars — the truthful answer for a cable.
    const bool online = net_up();
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
        draw_wifi_icon(online, net_rssi());
        u8g2.drawHLine(0, 13, 128);
        if (online) {
            u8g2.drawStr(0, 28, "See photos at:");
            u8g2.drawStr(0, 42, GW_VIEW_HOST);   // trailcam.example.com (family gallery)
        } else {
#ifdef GW_NET_HAS_WIFI
            if (s_net_state == NET_ST_PROVISION) {
#ifdef GW_HAS_PORTAL
                // The screen is the ONLY place an unprovisioned box can tell someone
                // what to do next, so it names the network to join.
                if (s_portal_up) {
                    u8g2.drawStr(0, 28, "Setup - join wifi:");
                    snprintf(line, sizeof(line), "%.21s", s_portal_ssid);
                    u8g2.drawStr(0, 42, line);
                } else
#endif
                {
                    u8g2.drawStr(0, 28, "Needs setup:");
                    u8g2.drawStr(0, 42, "no network yet");
                }
            } else if (net_cred_at(s_wifi_idx).ssid) {
                u8g2.drawStr(0, 28, "trying network:");
                snprintf(line, sizeof(line), "%.21s", net_cred_at(s_wifi_idx).ssid);
                u8g2.drawStr(0, 42, line);
            } else
#endif
            {
                u8g2.drawStr(0, 28, "trying network:");
                u8g2.drawStr(0, 42, "ethernet");
            }
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
    // Upload queue health. `alarm` is the field to page on: no card (or a card that went
    // bad) means this gateway is back to depth 1 and an outage loses frames, which is the
    // whole defect the queue exists to close. Empty string on builds with no TF slot.
    char sdq[240] = "";
#ifdef GW_SD_QUEUE
    {
        const long age = sdq_oldest_age_s();
        char agec[24];
        if (age < 0) snprintf(agec, sizeof(agec), "null");
        else         snprintf(agec, sizeof(agec), "%ld", age);
        snprintf(sdq, sizeof(sdq),
                 ",\"sd_queue\":{\"card\":%s,\"depth\":%lu,\"thumbs\":%lu,\"fulls\":%lu,"
                 "\"oldest_age_s\":%s,\"free_mb\":%lu,\"card_mb\":%lu,\"evicted\":%lu,"
                 "\"write_errors\":%lu,\"read_errors\":%lu,\"mount_fails\":%lu"
                 ",\"alarm\":%s}",
                 s_sdq.mounted ? "true" : "false", (unsigned long)s_sdq.depth,
                 (unsigned long)s_sdq.thumbs, (unsigned long)s_sdq.fulls, agec,
                 (unsigned long)(s_sdq.free_bytes >> 20),
                 (unsigned long)(s_sdq.card_bytes >> 20),
                 (unsigned long)s_sdq.evicted, (unsigned long)s_sdq.write_errors,
                 (unsigned long)s_sdq.read_errors, (unsigned long)s_sdq.mount_fails,
                 s_sdq.mounted ? "null" : "\"no_card_depth_1\"");
    }
#endif

    queue_beat("{\"site\":\"%s\",\"node\":\"gw-%s\",\"kind\":\"gateway\","
               "\"uptime_s\":%lu,\"fw_version\":\"" GW_FW_VERSION "\","
               "\"rssi\":%d,\"boot_reason\":\"%s\","
               "\"extra\":{\"free_heap\":%u,\"wifi_ssid\":\"%s\",\"ip\":\"%s\","
               "\"soc_temp_c\":%.1f,\"lora_profile\":\"%s\",%s,%s,"
               "\"uploads\":%lu,\"res_ok\":%lu,\"res_fail\":%lu,"
               "\"announces\":%lu,\"announces_rf\":%lu,\"announces_relayed\":%lu,"
               "\"announces_foreign\":%lu,\"leaf_pin\":%s%s%s,\"roster\":%s,"
               // TOFU auto re-arm: a pin dropped without a human asking is a
               // security-relevant event, so the count is queryable remotely, not only
               // visible on the local page.
               "\"tofu_rearms\":%u,"
               "\"chunks\":%lu%s}}",
               gw_site(), gw_site(), (unsigned long)(millis() / 1000),
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
               (unsigned)s_rearms_done,
               (unsigned long)s_stats.chunks, sdq);
}

// --- factory reset: 5 s BOOT hold --------------------------------------------------------
// Design of record: homelab docs/trailcam/gateway-provisioning.md §4, "Factory reset".
// Only the production gateway PCB (v0.2) has this button + LED wired: BOOT is IO0/SW2,
// STATUS is IO4 (GPIO map v0.2, docs/trailcam/gateway-pcb.md). GW_NET_ETH_W5500 is set
// ONLY by [s3_gateway_common] in platformio.ini — the W5500 chip only exists on that
// board — so this whole feature compiles out on the Heltec V3 and T-Internet-POE
// envs, which have no STATUS LED at this pin to drive.
#ifdef GW_NET_ETH_W5500
#define GW_FACTORY_RESET_PIN     0      // IO0, SW2 — BOOT button, active LOW (board pull-up)
#define GW_FACTORY_RESET_LED     4      // IO4 — STATUS LED, active HIGH
#define GW_FACTORY_RESET_HOLD_MS 5000u

static uint32_t s_reset_hold_start = 0;   // 0 = button not currently held
static bool     s_reset_fired      = false;   // latches once per hold; a bounce can't refire mid-blink

// Blocking on purpose: this runs once, right before ESP.restart(), so borrowing the loop
// for ~1 s here costs nothing a reboot doesn't already cost. Six fast blinks is a pattern
// nothing else on this pin will ever show, since nothing else drives IO4 yet.
static void factory_reset_led_confirm() {
    for (int i = 0; i < 6; i++) {
        digitalWrite(GW_FACTORY_RESET_LED, HIGH);
        delay(80);
        digitalWrite(GW_FACTORY_RESET_LED, LOW);
        delay(80);
    }
}

// THE INVARIANT (stated twice in the design doc, and here a third time): this clears
// wifi_* and claim_* and NOTHING else. gateway/rns_prv is NEVER touched here — it is
// this device's serial number to the whole mesh (node-identity.md) and the second claim
// factor, and a reset that re-mints it turns a supported recovery into an ops ticket.
// net_creds_clear() and claim_forget() are both guarded to their own keys under the
// "gateway" NVS namespace already; do not replace either with an erase_flash-style wipe
// to "really" reset the box — that IS the identity-mint path, and it is out of scope by
// design (gateway-provisioning.md §4: equivalent to replacing the board).
static void factory_reset_fire() {
    Serial.println("[gw] factory reset: BOOT held 5s -> clearing WiFi + claim (identity kept)");
    net_creds_clear();       // wifi_ssid / wifi_psk / wifi_ok only — never rns_prv
#ifndef GW_NO_CLAIM
    claim_forget();          // claim_code / claim_site / claim_url / claim_tok / claim_tid only — never rns_prv
#endif
    factory_reset_led_confirm();
    Serial.println("[gw] factory reset done -> rebooting into UNCLAIMED + PROVISION");
    delay(150);
    ESP.restart();
}

// Non-blocking hold detection, polled from loop() like every other tick here. Fires once
// per hold (s_reset_fired latches until release), so there is no double-fire and no
// dependence on how long the button stays down after the 5 s mark.
static void factory_reset_tick() {
    if (digitalRead(GW_FACTORY_RESET_PIN) != LOW) {   // released (or never pressed): rearm
        s_reset_hold_start = 0;
        s_reset_fired = false;
        return;
    }
    if (!s_reset_hold_start) { s_reset_hold_start = millis(); return; }
    if (s_reset_fired) return;
    if (millis() - s_reset_hold_start < GW_FACTORY_RESET_HOLD_MS) return;
    s_reset_fired = true;
    factory_reset_fire();
}
#endif   // GW_NET_ETH_W5500

// --- OTA rollback -------------------------------------------------------------------------
// Two app slots are only a RE-FLASH path unless something can decide the new image is
// bad and go back on its own. On a tower node the difference is a climb.
//
// The core-3.x bootloader already has CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y (and the
// app side CONFIG_APP_ROLLBACK_ENABLE=y, which is what gates the hook below), so an
// espota image boots in ESP_OTA_IMG_PENDING_VERIFY. What arduino-esp32 then does by
// default is mark it valid inside initArduino(), BEFORE setup() runs — which only ever
// catches an image that cannot reach initArduino() at all. An image that boots fine and
// then cannot get on the network is exactly the one that strands the box, and the default
// blesses it.
//
// verifyRollbackLater() is the core's weak hook for saying "not yet". Overriding it to
// true defers the verdict to us, and we cast it in loop() the moment the uplink is up and
// ArduinoOTA is listening — i.e. once the box has proven it is re-flashable. An image that
// never gets there is rolled back to the previous slot on the next reboot.
//
// OPT-IN, and only the production gateway PCB opts in ([s3_gateway_common] sets
// GW_OTA_ROLLBACK). The Heltec and T-Internet-POE gateways keep the mark-at-boot
// behaviour they were flashed with; changing the recovery semantics of a board already
// up a tree is not this bead's business.
#ifdef GW_OTA_ROLLBACK
#include "esp_ota_ops.h"

// Weak symbol in esp32-hal-misc.c. Returning true means "I will call
// esp_ota_mark_app_valid_cancel_rollback() myself"; the core then leaves the partition
// PENDING_VERIFY through setup().
//
// extern "C" IS LOAD-BEARING. esp32-hal-misc.c is C, so the weak symbol is the unmangled
// `verifyRollbackLater`. Defining it as plain C++ here emits _Z20verifyRollbackLaterv,
// which overrides nothing: the build succeeds, the core keeps its own default, and the
// image goes on being marked valid at boot. The failure is silent, which is the worst
// kind for a recovery path, so verify with:
//   xtensa-esp32s3-elf-nm .pio/build/s3-gateway/firmware.elf | grep verifyRollbackLater
// and expect ONE unmangled T symbol.
extern "C" bool verifyRollbackLater() { return true; }

static bool s_ota_marked_valid = false;   // settled: nothing left to do this boot
static bool s_ota_mark_warned  = false;   // the failure has been said once

// Called on every network-up edge until it settles. The latch is set only when the
// verdict is actually in — either the partition was never pending (a normal boot, where
// esp_ota_get_state_partition reports ESP_OTA_IMG_VALID and there is nothing to cancel),
// or the mark succeeded. A failed mark deliberately does NOT latch, so the next link-up
// tries again; the alarm is rate-limited to once per boot instead.
static void ota_mark_valid_once() {
    if (s_ota_marked_valid) return;
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_VALID;
    if (!running || esp_ota_get_state_partition(running, &state) != ESP_OK) return;
    if (state != ESP_OTA_IMG_PENDING_VERIFY) { s_ota_marked_valid = true; return; }
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        s_ota_marked_valid = true;
        Serial.println("[gw] OTA image CONFIRMED (network + OTA up) — rollback cancelled");
        log_event("OTA image confirmed good — rollback cancelled");
        return;
    }
    if (!s_ota_mark_warned) {
        s_ota_mark_warned = true;
        Serial.println("[gw] *** OTA image could NOT be marked valid — this build rolls "
                       "back to the previous slot on the next reboot ***");
        log_event("OTA mark-valid FAILED — will roll back on reboot");
    }
}
#endif   // GW_OTA_ROLLBACK

// --- bring-up ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(100);
    Serial.printf("[gw] firmware %s booting\n", GW_FW_VERSION);
#ifdef GW_NET_ETH_W5500
    pinMode(GW_FACTORY_RESET_PIN, INPUT_PULLUP);   // BOOT/SW2 — board already pulls up; belt+braces
    pinMode(GW_FACTORY_RESET_LED, OUTPUT);
    digitalWrite(GW_FACTORY_RESET_LED, LOW);       // STATUS LED off until a reset fires
#endif
#ifndef GW_NO_CLAIM
    claim_load();
#endif
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
    RNS::set_log_callback(rns_log_cb);   // mirror NOTICE+ into the /debuglog ring (bug 6)
    Serial.printf("[gw] boot. free heap=%u\n", (unsigned)ESP.getFreeHeap());

    // Filesystem: mount existing, format only on the very first boot (identity's stores
    // and Transport's path tables persist — unlike gate-a, which formatted every boot).
    filesystem.init(/*reformatOnFail=*/true);
    RNS::Utilities::OS::register_filesystem(filesystem);
    // Crash leftovers: a spool interrupted by a reboot is worthless (the server
    // re-delivers the fetch command), and stale files would confuse the next rename.
    // Note this is the LittleFS staging area only — /spool.jpg's metadata (event_id,
    // camera, captured_at) lived in RAM, so a leftover is an unlabelled JPEG nobody can
    // ingest. The SD queue below is the opposite: its entries carry their own header and
    // are deliberately KEPT across a reboot.
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
    // Deep upload queue on the TF slot: the first mount attempt is DELIBERATELY NOT made
    // here. It happens in sdq_loop(), after net_setup() and ota_setup() have run, and it
    // runs in a timeout-bounded task (see sdq_probe_spawn). Mounting here was a boot hang
    // on an empty slot — bench-proven 2026-08-09 — and because it sits ~200 lines ahead
    // of net_setup(), a hang meant no ethernet and no OTA on a firmware with no rollback:
    // an unrecoverable board, which on the tower is a climb. Never put a call that can
    // block ahead of the network and OTA. See homelab docs/trailcam/gateway-sd-queue.md.

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

#ifdef GW_NET_HAS_ETH
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
    snprintf(s_pubkey_hex, sizeof(s_pubkey_hex), "%s", id.get_public_key().toHex().c_str());
    snprintf(s_device_id, sizeof(s_device_id), "gw-%.8s", id.hexhash().c_str());
    snprintf(s_identity_hash, sizeof(s_identity_hash), "%s", id.hexhash().c_str());
    Serial.printf("[gw] identity public key: %s\n", s_pubkey_hex);
    Serial.printf("[gw] device id: %s\n", s_device_id);
#ifndef GW_NO_CLAIM
    s_identity = id;        // the claim request is signed with this key
    claim_init_state();     // resume CLAIMED / an interrupted claim from NVS
#endif
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
    // gateway-0.13.0: the park now HOLDS — ADR is suspended for ?min=N minutes
    // (default GW_RF_BASE_HOLD_DEFAULT_MIN, clamp 1..GW_RF_BASE_HOLD_MAX_MIN) so the
    // site stays audible on base for the whole enroll/survey window instead of
    // re-granting up after a few announces. ?min=0 = legacy park, no hold.
    s_web.on("/rf/base", HTTP_POST, []() {
        long min = GW_RF_BASE_HOLD_DEFAULT_MIN;
        if (s_web.hasArg("min")) min = s_web.arg("min").toInt();
        if (min < 0) min = 0;
        if (min > GW_RF_BASE_HOLD_MAX_MIN) min = GW_RF_BASE_HOLD_MAX_MIN;
        rf_retune(LORA_PROFILE_BASE);
        s_rf.granted  = LORA_PROFILE_BASE;
        s_rf.pending  = 0xFF;
        s_rf.scanning = false;
        rf_persist(LORA_PROFILE_BASE);
        rf_clear_snr();
        s_rf_hold_until = min ? millis() + (uint32_t)min * 60000u : 0;
        char resp[96];
        if (min)
            snprintf(resp, sizeof(resp),
                     "radio parked on base, ADR held %ld min (POST /rf/resume to release)\n",
                     min);
        else
            snprintf(resp, sizeof(resp), "radio parked on base (no hold)\n");
        Serial.printf("[gw] rf/base: parked on base, hold %ld min\n", min);
        log_event("operator parked radio on base (hold %ld min)", min);
        s_web.send(200, "text/plain", resp);
    });
    // Release a survey/commissioning hold early; ADR resumes on the next announces.
    s_web.on("/rf/resume", HTTP_POST, []() {
        const bool was = rf_hold_active();
        s_rf_hold_until = 0;
        if (was) {
            Serial.println("[gw] rf/resume: hold released -> ADR resumed");
            log_event("operator released rf/base hold");
        }
        s_web.send(200, "text/plain",
                   was ? "hold released, ADR resumes\n" : "no hold was active\n");
    });
    // Radio test mode: reversible cert-bench emission for §15.247 DTS / 996369-D04
    // measurements. NON-DESTRUCTIVE — WiFi + OTA stay up, a hard timeout (?secs, clamp
    // 1..GW_RFTEST_MAX_SECS) auto-reverts, and POST /rf/test/stop reverts immediately.
    // While active, the mesh radio is dedicated to the emission (all mesh TX/RX suspended).
    // SAFETY: radiates continuously up to +22 dBm — bench into attenuation/a shield only.
    //   POST /rf/test?mode=cw&freq=915.0&pwr=17&secs=30
    //   POST /rf/test?mode=mod&freq=915.0&bw=500&sf=9&cr=5&pwr=17&secs=30
    s_web.on("/rf/test", HTTP_POST, []() {
        if (!s_radio_ok || !LoRaInterface::active) {
            s_web.send(503, "text/plain", "radio absent — cannot run RF test\n");
            return;
        }
        String mode = s_web.hasArg("mode") ? s_web.arg("mode") : String("cw");
        float  freq = s_web.hasArg("freq") ? s_web.arg("freq").toFloat() : 915.0f;
        int    pwr  = s_web.hasArg("pwr")  ? s_web.arg("pwr").toInt()    : 17;
        long   secs = s_web.hasArg("secs") ? s_web.arg("secs").toInt()   : 30;
        float  bw   = s_web.hasArg("bw")   ? s_web.arg("bw").toFloat()   : 500.0f;
        int    sf   = s_web.hasArg("sf")   ? s_web.arg("sf").toInt()     : 9;
        int    cr   = s_web.hasArg("cr")   ? s_web.arg("cr").toInt()     : 5;
        if (secs < 1) secs = 1;
        if (secs > GW_RFTEST_MAX_SECS) secs = GW_RFTEST_MAX_SECS;
        if (pwr < -9) pwr = -9;
        if (pwr > 22) pwr = 22;
        if (freq < 902.0f || freq > 928.0f) {
            s_web.send(400, "text/plain", "freq must be within the 902-928 MHz ISM band\n");
            return;
        }
        bool ok;
        if (mode == "mod") {
            ok = LoRaInterface::active->radio_test_mod(freq, bw, sf, cr, (int8_t)pwr);
        } else {
            mode = "cw";
            ok = LoRaInterface::active->radio_test_cw(freq, (int8_t)pwr);
        }
        if (!ok) {
            LoRaInterface::active->radio_test_stop();
            s_rftest_until = 0;
            s_web.send(500, "text/plain", "radio rejected the test command — reverted\n");
            return;
        }
        s_rftest_until = millis() + (uint32_t)secs * 1000u;
        if (!s_rftest_until) s_rftest_until = 1;   // never 0 while active (millis() wrap)
        if (mode == "mod")
            snprintf(s_rftest_desc, sizeof(s_rftest_desc),
                     "MOD %.3f MHz BW%.0f SF%d @ %d dBm for %lds", freq, bw, sf, pwr, secs);
        else
            snprintf(s_rftest_desc, sizeof(s_rftest_desc),
                     "CW %.3f MHz @ %d dBm for %lds", freq, pwr, secs);
        char resp[192];
        snprintf(resp, sizeof(resp),
                 "RF TEST ON: %s. Mesh suspended; auto-reverts (POST /rf/test/stop to end now).\n",
                 s_rftest_desc);
        Serial.printf("[gw] rf/test: %s\n", s_rftest_desc);
        log_event("rf/test start: %s", s_rftest_desc);
        s_web.send(200, "text/plain", resp);
    });
    s_web.on("/rf/test/stop", HTTP_POST, []() {
        if (!rftest_active()) { s_web.send(200, "text/plain", "no RF test was active\n"); return; }
        if (LoRaInterface::active) LoRaInterface::active->radio_test_stop();
        s_rftest_until = 0;
        s_rftest_desc[0] = 0;
        Serial.println("[gw] rf/test: stopped by operator -> normal mesh restored");
        log_event("rf/test stopped by operator");
        s_web.send(200, "text/plain", "RF test stopped, normal mesh operation restored\n");
    });
    // Bug-6 diagnosis: dump the RNS/microStore debug ring (see dbglog_push above).
    s_web.on("/debuglog", []() {
        String out;
        out.reserve(DBGLOG_N * (DBGLOG_W / 2));
        const uint32_t start = (s_dbglog_seq > (uint32_t)DBGLOG_N)
                                   ? s_dbglog_seq - DBGLOG_N : 0;
        for (uint32_t i = start; i < s_dbglog_seq; i++) {
            const uint32_t slot = i % DBGLOG_N;
            char pre[16];
            snprintf(pre, sizeof(pre), "%9lu ", (unsigned long)s_dbglog_ms[slot]);
            out += pre;
            out += s_dbglog[slot];
            out += "\n";
        }
        s_web.send(200, "text/plain", out);
    });
#if defined(GW_HAS_PORTAL) || !defined(GW_NO_CLAIM)
    // Setup surfaces (step 8): the SoftAP portal form and/or the LAN
    // claim form, dispatched by setup_route_*. Registered always; the handlers decide
    // per-request which hat /setup wears, and 404 when neither applies.
    s_web.on("/setup", HTTP_GET,  setup_route_get);
    s_web.on("/setup", HTTP_POST, setup_route_post);
#endif
#ifdef GW_HAS_PORTAL
    // onNotFound is what turns a phone's captive-portal probe into the setup form; when
    // the AP is down it degrades to a plain 404.
    s_web.onNotFound(portal_not_found);
#endif
    // s_web.begin() happens on the first network-up in loop(), or earlier, when the
    // provisioning portal raises its AP. Binding before any interface exists leaves the
    // listener dead (connection refused), so it is never done here.

    net_setup();
}

void loop() {
    // Radio test mode: while a test emission runs, the radio is dedicated to it —
    // skip ALL mesh servicing (reticulum RX/TX, ADR, command polls, beats) so nothing
    // keys the radio over the test carrier. Keep ONLY the web server + OTA + network
    // alive, so the test is always stoppable (POST /rf/test/stop) and the box stays
    // OTA-recoverable. The hard deadline guarantees a return to normal even if no stop
    // ever arrives (operator walked away, laptop died).
    if (s_rftest_until) {
        if ((int32_t)(millis() - s_rftest_until) >= 0) {
            if (LoRaInterface::active) LoRaInterface::active->radio_test_stop();
            s_rftest_until = 0;
            s_rftest_desc[0] = 0;
            Serial.println("[gw] rf/test window expired -> normal mesh operation restored");
            log_event("rf/test window expired, radio restored");
            // fall through into the normal loop body this tick
        } else {
            s_web.handleClient();
            portal_loop();
            ArduinoOTA.handle();
            serial_console_loop();   // stays live during an rf/test window on purpose
#ifdef GW_NET_ETH_W5500
            factory_reset_tick();    // BOOT hold must work during an rf/test window too
#endif
            net_ensure();
            return;
        }
    }

    reticulum.loop();

    // Staged "have?" reply — IMMEDIATELY after the radio pump, before any WiFi/HTTP
    // work. First placement (bottom of loop) verified failed on air 2026-07-18: the
    // beat flush's TLS POST sat between dispatch and the send, the reply left ~5 s
    // late, missed the leaf's wait window, and the late TX collided with the leaf's
    // fail-open chunk 1. Radio replies outrank telemetry.
    if (s_have_reply_out[0] && latest_link) {
        RNS::Packet pkt(latest_link, RNS::Bytes((const uint8_t*)s_have_reply_out,
                                                strlen(s_have_reply_out)));
        pkt.send();
        Serial.printf("[gw] have? reply delivered: %s\n", s_have_reply_out);
        s_have_reply_out[0] = 0;
    }

    // A roster leaf just announced -> it's awake and listening; fire ITS pending
    // commands NOW, plus a time_sync when due (cold-boot hello, or the periodic
    // drift refresh). Per entry: each leaf wakes on its own schedule.
    //
    // ⚠️ POSITION IS THE FIX (2026-09-03). This ran at the BOTTOM of loop(),
    // behind gw_destination.announce(), poll_commands() and fetch_site_name() — two
    // blocking TLS GETs. Sniffing both sides off one RAK breakout on the shared ladder
    // measured the gateway's reply at a **median 7.0 s** after the leaf finished
    // announcing (mean 8.2, max 20), while the leaf's window is
    // leaf_radio_poll_commands(2500) — **75 % of replies landed after it had shut**, so
    // no command ever arrived. Same failure the "have?" reply hit on 2026-07-18 and for
    // the same reason: HTTP work sat between dispatch and the send. Radio replies
    // outrank telemetry — keep this above the WiFi/HTTP section.
    for (int i = 0; i < s_roster_n; i++) {
        LeafEntry& L = s_roster[i];
        if (!L.announced) continue;
        L.announced = false;
        const bool direct = L.announce_direct;
        // Heard direct while WE sit on base = this member is reachable on
        // base — the walk-back shepherd's per-leaf exit evidence. Direct only (a
        // relay can hear a rung we can't, bug 1); read s_rf.current before anything
        // below retunes it.
        if (direct && s_rf.current == LORA_PROFILE_BASE) {
            L.base_heard_ms = millis();
            if (L.shep_wait) {
                L.shep_wait = false;
                Serial.printf("[gw] ADR: shepherd — %s heard on base, debt cleared\n",
                              L.slug[0] ? L.slug : L.hex);
                log_event("ADR shepherd: %s on base", L.slug[0] ? L.slug : "leaf");
            }
        }
        rf_on_leaf_announce(direct);                 // confirm/adopt profile state first
        // Commands still deliver on relayed announces (RNS routes them through the
        // relay — legitimate in a relay topology); only ADR is direct-only.
        const int cmds_sent = deliver_pending_commands(L);
        // ADR GOES BEFORE THE TIME SYNC, AND SUPPRESSES IT (2026-09-07).
        //
        // A grant gets exactly ONE transmission. We retune to the granted profile the
        // instant we send it, which makes us deaf to a leaf still sitting on the old
        // one — so a grant that misses the leaf's RX window can never be retried, it
        // can only time out and revert. That window is leaf_radio_poll_commands(2500):
        // 2.5 s, opening the moment the leaf's own announce clears the air.
        //
        // Sent last, the grant was losing that window to the time sync. Measured on the
        // bench against board 0016 (leaf-0.23.0, two feet away, rssi -4): across eleven
        // consecutive downlink windows the leaf received exactly one frame, a time_sync,
        // and never once saw a grant — every attempt ended "ADR revert (no confirm)"
        // ~20 s later. A freshly pinned leaf has synced_once=false, so maybe_time_sync()
        // fires on the very same announce that first makes the site single-leaf and
        // therefore grantable: the sync takes the slot, its RNS ack puts the leaf's
        // half-duplex radio into TX (deaf), and the grant lands in the gap or after the
        // window has shut. That is every TOFU re-arm, every gateway reboot, every newly
        // enrolled node — i.e. exactly when a site is trying to settle on a rung.
        //
        // Skipping the sync costs nothing: it is periodic, the leaf re-requests nothing,
        // and rf_on_leaf_announce() calls arm_urgent_sync_all() the moment a grant is
        // CONFIRMED — so a successful grant re-arms the very sync this suppressed, one
        // announce later and now on the correct profile.
        const bool adr_sent = direct && maybe_grant_profile(L, cmds_sent);
        if (!adr_sent) maybe_time_sync(L);
    }

    // Garbage-collect a pin whose leaf is provably gone. Deliberately AFTER the
    // delivery loop and outside packet dispatch: it rewrites the roster array the loop
    // above is iterating, and it defers its NVS write like enrollment does.
    tofu_rearm_scan();

    s_web.handleClient();
    portal_loop();
    ArduinoOTA.handle();
    serial_console_loop();
#ifdef GW_NET_ETH_W5500
    factory_reset_tick();
#endif

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
            web_begin_once();
            static bool ota_started = false;
            if (!ota_started) { ota_setup(); ota_started = true; }
#ifdef GW_OTA_ROLLBACK
            // The bar for "this image is good": an uplink AND a listening espota server.
            // Anything short of that is a box that would need a climb, so it rolls back.
            ota_mark_valid_once();
#endif
#ifdef GW_NET_HAS_ETH
            // WIRED ONLY, deliberately (an open sub-question, resolved this way for
            // now): the RNS-over-UDP bridge is a wired-tower role — it exposes the mesh to
            // Python RNS peers on a LAN we control. A WiFi-fallback unit sits on a
            // customer's household network, where publishing a mesh bridge on :4242 is an
            // attack surface nobody asked for. Revisit when a WiFi box actually needs
            // local RNS peers. The socket binds INADDR_ANY, so once it is up it keeps
            // working across an interface switch; it is simply never FIRST started on WiFi.
            if (!s_udp_started && s_net_active == NET_IFACE_ETH) {
                udp_interface.start(); s_udp_started = true;
                Serial.println("[gw] RNS UDP interface up on the wire (:4242)");
            } else if (s_udp_started && s_net_active == NET_IFACE_ETH) {
                // Re-arm after net_ensure() paused it for a down window — start()
                // is safe to call again: on ARDUINO it's udp.begin(), which stops+recreates
                // the underlying socket itself (NetworkUDP::begin() calls stop() first).
                // Gated on ETH same as the first-start branch above: without it, an
                // ETH-then-WiFi failover re-arms the bridge on WiFi too, exposing :4242 on
                // the customer's household LAN — exactly what "WIRED ONLY" above rules out.
                udp_interface.start();
                Serial.println("[gw] RNS UDP interface resumed");
            }
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

#ifndef GW_NO_CLAIM
    // Claim state machine: a blocking TLS POST when one is due, so it rides
    // the same radio-quiet gate as the rest of the HTTP work.
    if (!radio_busy() && net_up()) claim_tick();
#endif

    // TOFU enrollment landed inside announce dispatch -> persist it here (no NVS
    // writes from inside packet handling).
    if (s_pin_save_pending) {
        s_pin_save_pending = false;
        save_roster();
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
#ifdef GW_SD_QUEUE
        sdq_loop();        // re-mount attempts (card inserted with no reboot) + stats
        // One frame per pass: coming back through loop() between POSTs keeps the radio
        // and the web server serviced while a long backlog drains.
        if (s_sdq.mounted && s_sdq.depth && net_up() && gw_provisioned()) sdq_drain_one();
#endif
        if ((s_pending.jpeg || s_pending.spooled) && net_up() && gw_provisioned()) {
            if (upload_pending()) pending_clear();
        }
    }

    // Poll tight while frames are arriving (a 10 ms nap per loop adds up across a
    // windowed transfer's hundreds of frames); relax when the mesh is idle.
    RNS::Utilities::OS::sleep(radio_busy() ? 0.001 : 0.01);
}
